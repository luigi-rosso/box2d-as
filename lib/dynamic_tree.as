// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/dynamic_tree.h and src/dynamic_tree.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_ENABLE_VALIDATION, B2_NULL_INDEX, FLT_MAX } from "./core";
import {
	b2AABB,
	b2AABB_Center,
	b2AABB_Contains,
	b2AABB_Extents,
	b2AABB_Union,
	b2Abs,
	b2AbsFloat,
	b2Add,
	b2CrossSV,
	b2DistanceSquared,
	b2Dot,
	b2LengthSquared,
	b2Max,
	b2MaxInt,
	b2Min,
	b2MinFloat,
	b2MulAdd,
	b2MulSV,
	b2Normalize,
	b2Sub,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import {
	b2BoxCastInput,
	b2DynamicTree,
	b2RayCastInput,
	b2TreeNode,
	b2TreeProxy,
	b2TreeStats,
} from "./collision";
import { b2EnlargeAABB, b2Perimeter } from "./aabb";
import { b2LoadAABBV, b2OverlapNode, b2StoreAABBV, b2UnionPairV, b2UnionV } from "./simd";

// dynamic_tree.h

export const B2_TREE_STACK_SIZE: i32 = 512;

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body
export const B2_MOVED_NODE: u32 = 1 << 30;

export const B2_LEAF_NODE: u32 = 1 << 31;
export const B2_NODE_INDEX_MASK: u32 = 0xffffffff & ~(B2_MOVED_NODE | B2_LEAF_NODE);

// Used to indicate empty nodes. It also passes as a leaf so that internal node
// processing will naturally skip it.
export const B2_EMPTY_NODE: u32 = B2_NODE_INDEX_MASK | B2_LEAF_NODE;

export const B2_ROOT_NODE: i32 = 0;

// Nodes are taken by value where C takes a pointer.
@inline export function b2IsLeaf(node: b2TreeNode): bool {
	return (node.flagIndex & B2_LEAF_NODE) == B2_LEAF_NODE;
}

@inline export function b2IsNodeMoved(node: b2TreeNode): bool {
	return (node.flagIndex & B2_MOVED_NODE) == B2_MOVED_NODE;
}

@inline export function b2IsEmptyNode(node: b2TreeNode): bool {
	return node.flagIndex == B2_EMPTY_NODE;
}

@inline export function b2GetLeftChild(node: b2TreeNode): i32 {
	return <i32>(node.flagIndex & B2_NODE_INDEX_MASK);
}

@inline export function b2GetProxyId(node: b2TreeNode): i32 {
	return <i32>(node.flagIndex & B2_NODE_INDEX_MASK);
}

@inline export function b2GetRootPair(nodes: StaticArray<b2TreeNode>): i32 {
	let flags = nodes[B2_ROOT_NODE].flagIndex;
	return (flags & B2_LEAF_NODE) == B2_LEAF_NODE ? B2_ROOT_NODE : <i32>(flags & B2_NODE_INDEX_MASK);
}

// The union is two fields here, so the one a node does not use stays zero.
export function b2MakeEmptyNode(): b2TreeNode {
	let node = b2TreeNode();
	node.aabb.lowerBound = b2Vec2(Infinity, Infinity);
	node.aabb.upperBound = b2Vec2(-Infinity, -Infinity);
	node.flagIndex = B2_EMPTY_NODE;
	node.height = 0;
	return node;
}

@inline export function b2HasTreeMoved(tree: b2DynamicTree): bool {
	return (tree.nodes[B2_ROOT_NODE].flagIndex & B2_MOVED_NODE) == B2_MOVED_NODE;
}

@inline export function b2NeedsRebuild(tree: b2DynamicTree): bool {
	return (tree.nodes[B2_ROOT_NODE].flagIndex & B2_MOVED_NODE) == B2_MOVED_NODE || tree.dfsOrdered == false;
}

// dynamic_tree.c

// C sizes on a 64 bit target, for b2DynamicTree_GetByteCount
const B2_DYNAMIC_TREE_BYTES: i32 = 104;
const B2_TREE_NODE_BYTES: i32 = 32;
const B2_TREE_PROXY_BYTES: i32 = 24;
const B2_AABB_BYTES: i32 = 16;
const B2_VEC2_BYTES: i32 = 8;

// Validation stays off as in C; -D B2_VALIDATE_TREE turns it on with -D B2_ENABLE_ASSERT.
const B2_TREE_VALIDATION: bool = B2_ENABLE_VALIDATION || isDefined(B2_VALIDATE_TREE);

function b2CopyArray<A>(dst: A, src: A, count: i32): void {
	for (let i = 0; i < count; ++i) {
		dst[i] = src[i];
	}
}

// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
// allocations can just bump.
function b2AllocateSiblingPair(tree: b2DynamicTree): i32 {
	// Allocate from free list.
	if (tree.pairFreeList != B2_NULL_INDEX) {
		let pair = tree.pairFreeList;
		tree.pairFreeList = tree.parents[pair];
		return pair;
	}

	// Extend the bump allocator if needed.
	if (tree.nodeEnd + 2 > tree.nodeCapacity) {
		let oldCapacity = tree.nodeCapacity;
		let newCapacity = oldCapacity + (oldCapacity >> 1);
		newCapacity += newCapacity & 1;
		let nodes = new StaticArray<b2TreeNode>(newCapacity);
		b2CopyArray(nodes, tree.nodes, oldCapacity);
		tree.nodes = nodes;
		let parents = new StaticArray<i32>(newCapacity);
		b2CopyArray(parents, tree.parents, oldCapacity);
		tree.parents = parents;
		tree.nodeCapacity = newCapacity;

		// The spare has to match, the rebuild allocates it again.
		tree.swapNodes = new StaticArray<b2TreeNode>(0);
	}

	let pair = tree.nodeEnd;
	tree.nodeEnd += 2;
	return pair;
}

function b2FreePair(tree: b2DynamicTree, pair: i32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd);
	tree.nodes[pair] = b2MakeEmptyNode();
	tree.nodes[pair + 1] = b2MakeEmptyNode();
	tree.parents[pair] = tree.pairFreeList;
	tree.parents[pair + 1] = B2_NULL_INDEX;
	tree.pairFreeList = pair;
}

// The memset of C: every field back to zero, arrays to empty.
function b2ResetDynamicTree(tree: b2DynamicTree): void {
	tree.nodes = new StaticArray<b2TreeNode>(0);
	tree.parents = new StaticArray<i32>(0);
	tree.proxies = new StaticArray<b2TreeProxy>(0);
	tree.nodeEnd = 0;
	tree.nodeCapacity = 0;
	tree.pairFreeList = 0;
	tree.proxyCount = 0;
	tree.proxyCapacity = 0;
	tree.proxyFreeList = 0;
	tree.swapNodes = new StaticArray<b2TreeNode>(0);
	tree.leafIndices = new StaticArray<i32>(0);
	tree.leafNodes = new StaticArray<b2TreeNode>(0);
	tree.leafBoxes = new StaticArray<b2AABB>(0);
	tree.leafCenters = new StaticArray<b2Vec2>(0);
	tree.binIndices = new StaticArray<i32>(0);
	tree.rebuildCapacity = 0;
	tree.dfsOrdered = false;
}

export function b2CreateDynamicTreeInternal(tree: b2DynamicTree, proxyCapacity: i32): void {
	let capacity = b2MaxInt(proxyCapacity, 16);

	// memset needed for deterministic serialization.
	b2ResetDynamicTree(tree);

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
	tree.nodeCapacity = 2 * capacity;
	tree.nodes = new StaticArray<b2TreeNode>(tree.nodeCapacity);
	tree.parents = new StaticArray<i32>(tree.nodeCapacity);
	tree.pairFreeList = B2_NULL_INDEX;

	// The root and the empty node always exist
	tree.nodes[B2_ROOT_NODE] = b2MakeEmptyNode();
	tree.nodes[B2_ROOT_NODE + 1] = b2MakeEmptyNode();
	tree.parents[B2_ROOT_NODE] = B2_NULL_INDEX;
	tree.parents[B2_ROOT_NODE + 1] = B2_NULL_INDEX;

	// This is the bump index.
	tree.nodeEnd = 2;
	tree.dfsOrdered = true;

	tree.proxyCapacity = capacity;
	tree.proxyCount = 0;
	tree.proxies = new StaticArray<b2TreeProxy>(tree.proxyCapacity);

	// Build a linked list for the free list.
	let proxies = tree.proxies;
	for (let i = 0; i < tree.proxyCapacity - 1; ++i) {
		proxies[i].node = B2_NULL_INDEX;
		proxies[i].next = i + 1;
	}

	proxies[tree.proxyCapacity - 1].node = B2_NULL_INDEX;
	proxies[tree.proxyCapacity - 1].next = B2_NULL_INDEX;
	tree.proxyFreeList = 0;

	tree.rebuildCapacity = 0;
}

export function b2DestroyDynamicTreeInternal(tree: b2DynamicTree): void {
	b2ResetDynamicTree(tree);
}

/// Constructing the tree initializes the node pool.
export function b2CreateDynamicTree(proxyCapacity: i32): b2DynamicTree {
	let tree = new b2DynamicTree();
	b2CreateDynamicTreeInternal(tree, proxyCapacity);
	return tree;
}

/// Destroy the tree, freeing the node pool.
export function b2DestroyDynamicTree(tree: b2DynamicTree?): void {
	if (tree == null) {
		return;
	}

	b2DestroyDynamicTreeInternal(tree);
}

// Allocate a proxy from the pool. Grow the pool if necessary.
function b2AllocateProxy(tree: b2DynamicTree): i32 {
	// Expand the pool as needed.
	if (tree.proxyFreeList == B2_NULL_INDEX) {
		if (B2_ASSERT_ENABLED) B2_ASSERT(tree.proxyCount == tree.proxyCapacity);

		// The free list is empty. Rebuild a bigger pool.
		// A restored pool can have capacity 1, so guarantee strict growth.
		let oldCapacity = tree.proxyCapacity;
		tree.proxyCapacity += b2MaxInt(oldCapacity >> 1, 1);
		let grown = new StaticArray<b2TreeProxy>(tree.proxyCapacity);
		b2CopyArray(grown, tree.proxies, oldCapacity);
		tree.proxies = grown;

		// Build a linked list for the free list.
		for (let i = oldCapacity; i < tree.proxyCapacity - 1; ++i) {
			grown[i].node = B2_NULL_INDEX;
			grown[i].next = i + 1;
		}

		grown[tree.proxyCapacity - 1].node = B2_NULL_INDEX;
		grown[tree.proxyCapacity - 1].next = B2_NULL_INDEX;
		tree.proxyFreeList = oldCapacity;
	}

	// Peel a proxy off the free list.
	let proxyIndex = tree.proxyFreeList;
	ref proxy = tree.proxies[proxyIndex];
	tree.proxyFreeList = proxy.next;
	proxy = b2TreeProxy();
	proxy.node = B2_NULL_INDEX;
	proxy.next = B2_NULL_INDEX;
	++tree.proxyCount;
	return proxyIndex;
}

// Return a proxy to the pool.
function b2FreeProxy(tree: b2DynamicTree, proxyId: i32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 < tree.proxyCount);
	tree.proxies[proxyId].node = B2_NULL_INDEX;
	tree.proxies[proxyId].next = tree.proxyFreeList;
	tree.proxyFreeList = proxyId;
	--tree.proxyCount;
}

@inline function b2GetNodeHeight(node: b2TreeNode): i32 {
	return b2IsLeaf(node) ? 0 : node.height;
}

// The internal node above a children pair
function b2MakeInternalNode(nodes: StaticArray<b2TreeNode>, pair: i32): b2TreeNode {
	let flags1 = nodes[pair].flagIndex;
	let flags2 = nodes[pair + 1].flagIndex;
	let height1 = (flags1 & B2_LEAF_NODE) == B2_LEAF_NODE ? 0 : nodes[pair].height;
	let height2 = (flags2 & B2_LEAF_NODE) == B2_LEAF_NODE ? 0 : nodes[pair + 1].height;

	let node = b2TreeNode();
	node.aabb = b2AABB_Union(nodes[pair].aabb, nodes[pair + 1].aabb);
	node.flagIndex = <u32>pair | ((flags1 | flags2) & B2_MOVED_NODE);
	node.height = 1 + b2MaxInt(height1, height2);
	return node;
}

function b2MakeLeafNode(aabb: b2AABB, proxyId: i32, userData: u64, moved: bool): b2TreeNode {
	let node = b2TreeNode();
	node.aabb = aabb;
	node.flagIndex = <u32>proxyId | B2_LEAF_NODE | (moved ? B2_MOVED_NODE : 0);
	node.shapeIndex = <i32><u32>userData;
	return node;
}

// A node landed at a new index, so tell what hangs below it
@inline function b2LinkChildren(tree: b2DynamicTree, nodeIndex: i32): void {
	let flags = tree.nodes[nodeIndex].flagIndex;
	if ((flags & B2_LEAF_NODE) == B2_LEAF_NODE) {
		tree.proxies[<i32>(flags & B2_NODE_INDEX_MASK)].node = nodeIndex;
	} else {
		let pair = <i32>(flags & B2_NODE_INDEX_MASK);
		tree.parents[pair] = nodeIndex;
		tree.parents[pair + 1] = nodeIndex;
	}
}

// The sweep refit visits indices from high to low, so it needs every child above its parent
@inline function b2IsNodeOrdered(nodes: StaticArray<b2TreeNode>, nodeIndex: i32): bool {
	let flags = nodes[nodeIndex].flagIndex;
	return (flags & B2_LEAF_NODE) == B2_LEAF_NODE || nodeIndex < <i32>(flags & B2_NODE_INDEX_MASK);
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
function b2FindBestSibling(tree: b2DynamicTree, boxD: b2AABB): i32 {
	let centerD = b2AABB_Center(boxD);
	let areaD = b2Perimeter(boxD);

	let nodes = tree.nodes;
	let nodeIndex = B2_ROOT_NODE;

	if (b2IsLeaf(nodes[nodeIndex])) {
		return nodeIndex;
	}

	let rootBox = nodes[nodeIndex].aabb;

	// Area of current node
	let areaBase = b2Perimeter(rootBox);

	// Area of inflated node
	let directCost = b2Perimeter(b2AABB_Union(rootBox, boxD));
	let inheritedCost: f32 = 0.0;

	let bestSibling = nodeIndex;
	let bestCost = directCost;

	// Descend the tree, following a single greedy path.
	for (;;) {
		let child1 = b2GetLeftChild(nodes[nodeIndex]);
		let child2 = child1 + 1;

		// Cost of creating a new parent for this node and the new leaf
		let cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if (cost < bestCost) {
			bestSibling = nodeIndex;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		let leaf1 = (nodes[child1].flagIndex & B2_LEAF_NODE) == B2_LEAF_NODE;
		let leaf2 = (nodes[child2].flagIndex & B2_LEAF_NODE) == B2_LEAF_NODE;

		// Cost of descending into child 1
		let lowerCost1 = FLT_MAX;
		let box1 = nodes[child1].aabb;
		let directCost1 = b2Perimeter(b2AABB_Union(box1, boxD));
		let area1: f32 = 0.0;
		if (leaf1) {
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			let cost1 = directCost1 + inheritedCost;
			if (cost1 < bestCost) {
				bestSibling = child1;
				bestCost = cost1;
			}
		} else {
			// Child 1 is an internal node
			area1 = b2Perimeter(box1);

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + b2MinFloat(areaD - area1, 0.0);
		}

		// Cost of descending into child 2
		let lowerCost2 = FLT_MAX;
		let box2 = nodes[child2].aabb;
		let directCost2 = b2Perimeter(b2AABB_Union(box2, boxD));
		let area2: f32 = 0.0;
		if (leaf2) {
			let cost2 = directCost2 + inheritedCost;
			if (cost2 < bestCost) {
				bestSibling = child2;
				bestCost = cost2;
			}
		} else {
			area2 = b2Perimeter(box2);
			lowerCost2 = inheritedCost + directCost2 + b2MinFloat(areaD - area2, 0.0);
		}

		if (leaf1 && leaf2) {
			break;
		}

		// Can the cost possibly be decreased?
		if (bestCost <= lowerCost1 && bestCost <= lowerCost2) {
			break;
		}

		if (lowerCost1 == lowerCost2 && leaf1 == false) {
			if (B2_ASSERT_ENABLED) B2_ASSERT(lowerCost1 < FLT_MAX);
			if (B2_ASSERT_ENABLED) B2_ASSERT(lowerCost2 < FLT_MAX);

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			let d1 = b2Sub(b2AABB_Center(box1), centerD);
			let d2 = b2Sub(b2AABB_Center(box2), centerD);
			lowerCost1 = b2LengthSquared(d1);
			lowerCost2 = b2LengthSquared(d2);
		}

		// Descend
		if (lowerCost1 < lowerCost2 && leaf1 == false) {
			nodeIndex = child1;
			areaBase = area1;
			directCost = directCost1;
		} else {
			nodeIndex = child2;
			areaBase = area2;
			directCost = directCost2;
		}
	}

	return bestSibling;
}

// Swap a child of A with a grandchild under the other child.
// Example swapping B with G:
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
function b2SwapNodes(tree: b2DynamicTree, iDown: i32, iUp: i32): void {
	let nodes = tree.nodes;
	let temp = nodes[iDown];
	nodes[iDown] = nodes[iUp];
	nodes[iUp] = temp;
	b2LinkChildren(tree, iDown);
	b2LinkChildren(tree, iUp);

	if (b2IsNodeOrdered(nodes, iDown) == false || b2IsNodeOrdered(nodes, iUp) == false) {
		tree.dfsOrdered = false;
	}

	// The sibling of the node that went down holds a different subtree now
	let iC = iDown ^ 1;
	nodes[iC] = b2MakeInternalNode(nodes, b2GetLeftChild(nodes[iC]));
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
function b2RotateNodes(tree: b2DynamicTree, iA: i32): void {
	let nodes = tree.nodes;
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsLeaf(nodes[iA]) == false);

	let iB = b2GetLeftChild(nodes[iA]);
	let iC = iB + 1;
	ref B = nodes[iB];
	ref C = nodes[iC];

	let leafB = b2IsLeaf(B);
	let leafC = b2IsLeaf(C);
	if (leafB && leafC) {
		return;
	}

	let bestDown = B2_NULL_INDEX;
	let bestUp = B2_NULL_INDEX;
	let bestDelta: f32 = 0.0;

	if (leafC == false) {
		// Swap B with F or G
		let iF = b2GetLeftChild(C);
		let iG = iF + 1;
		let areaC = b2Perimeter(C.aabb);

		// B <-> F then C (B G)
		let deltaBF = b2Perimeter(b2UnionV(B.aabb, nodes[iG].aabb)) - areaC;
		if (deltaBF < bestDelta) {
			bestDown = iB;
			bestUp = iF;
			bestDelta = deltaBF;
		}

		// B <-> G then C (F B)
		let deltaBG = b2Perimeter(b2UnionV(B.aabb, nodes[iF].aabb)) - areaC;
		if (deltaBG < bestDelta) {
			bestDown = iB;
			bestUp = iG;
			bestDelta = deltaBG;
		}
	}

	if (leafB == false) {
		// Swap C with D or E
		let iD = b2GetLeftChild(B);
		let iE = iD + 1;
		let areaB = b2Perimeter(B.aabb);

		// C <-> D then B (C E)
		let deltaCD = b2Perimeter(b2UnionV(C.aabb, nodes[iE].aabb)) - areaB;
		if (deltaCD < bestDelta) {
			bestDown = iC;
			bestUp = iD;
			bestDelta = deltaCD;
		}

		// C <-> E then B (D C)
		let deltaCE = b2Perimeter(b2UnionV(C.aabb, nodes[iD].aabb)) - areaB;
		if (deltaCE < bestDelta) {
			bestDown = iC;
			bestUp = iE;
			bestDelta = deltaCE;
		}
	}

	if (bestDown != B2_NULL_INDEX) {
		b2SwapNodes(tree, bestDown, bestUp);
	}
}

function b2InsertLeaf(tree: b2DynamicTree, aabb: b2AABB, proxyId: i32, moved: bool, shouldRotate: bool): void {
	let leaf = b2MakeLeafNode(aabb, proxyId, tree.proxies[proxyId].userData, moved);

	if (b2IsEmptyNode(tree.nodes[B2_ROOT_NODE])) {
		tree.nodes[B2_ROOT_NODE] = leaf;
		tree.proxies[proxyId].node = B2_ROOT_NODE;
		return;
	}

	let sibling = b2FindBestSibling(tree, aabb);

	// The sibling's position becomes the new parent. The sibling moves down into a new pair
	// beside the leaf, which puts it below its own children when it is internal.
	let pair = b2AllocateSiblingPair(tree);
	let nodes = tree.nodes;
	let parents = tree.parents;

	nodes[pair] = nodes[sibling];
	nodes[pair + 1] = leaf;
	parents[pair] = sibling;
	parents[pair + 1] = sibling;
	b2LinkChildren(tree, pair);
	tree.proxies[proxyId].node = pair + 1;

	nodes[sibling] = b2MakeInternalNode(nodes, pair);

	if (b2IsNodeOrdered(nodes, sibling) == false || b2IsNodeOrdered(nodes, pair) == false) {
		tree.dfsOrdered = false;
	}

	// Walk back up the tree refitting ancestors, the root included.
	let index = sibling;
	while (index != B2_NULL_INDEX) {
		if (shouldRotate) {
			b2RotateNodes(tree, index);
		}

		nodes[index] = b2MakeInternalNode(nodes, b2GetLeftChild(nodes[index]));
		index = parents[index];
	}
}

function b2RemoveLeaf(tree: b2DynamicTree, proxyId: i32): void {
	let nodes = tree.nodes;
	let parents = tree.parents;

	let leaf = tree.proxies[proxyId].node;
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= leaf && leaf < tree.nodeEnd);
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsLeaf(nodes[leaf]) && b2GetProxyId(nodes[leaf]) == proxyId);

	if (leaf == B2_ROOT_NODE) {
		nodes[B2_ROOT_NODE] = b2MakeEmptyNode();
		return;
	}

	// The sibling takes the parent's position and the pair is freed. This keeps the order,
	// the sibling only moves down.
	let parent = parents[leaf];
	nodes[parent] = nodes[leaf ^ 1];
	b2LinkChildren(tree, parent);
	b2FreePair(tree, leaf & ~1);

	// Update ancestors.
	let index = parents[parent];
	while (index != B2_NULL_INDEX) {
		nodes[index] = b2MakeInternalNode(nodes, b2GetLeftChild(nodes[index]));
		index = parents[index];
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
export function b2CreateTreeProxyInternal(tree: b2DynamicTree, aabb: b2AABB, categoryBits: u64, userData: u64, markMoved: bool): i32 {
	let proxyId = b2AllocateProxy(tree);

	ref proxy = tree.proxies[proxyId];
	proxy.categoryBits = categoryBits;
	proxy.userData = userData;

	let shouldRotate = true;
	b2InsertLeaf(tree, aabb, proxyId, markMoved, shouldRotate);

	return proxyId;
}

/// Create a proxy. Provide an AABB and a userData value.
export function b2CreateTreeProxy(tree: b2DynamicTree, aabb: b2AABB, categoryBits: u64, userData: u64): i32 {
	return b2CreateTreeProxyInternal(tree, aabb, categoryBits, userData, false);
}

/// Destroy a proxy. This asserts if the id is invalid.
export function b2DestroyTreeProxy(tree: b2DynamicTree, proxyId: i32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);

	b2RemoveLeaf(tree, proxyId);
	b2FreeProxy(tree, proxyId);
}

/// Get the number of proxies created
export function b2DynamicTree_GetProxyCount(tree: b2DynamicTree): i32 {
	return tree.proxyCount;
}

export function b2DynamicTree_MoveProxyInternal(tree: b2DynamicTree, proxyId: i32, aabb: b2AABB, markMoved: bool): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);

	b2RemoveLeaf(tree, proxyId);

	let shouldRotate = false;
	b2InsertLeaf(tree, aabb, proxyId, markMoved, shouldRotate);
}

/// Move a proxy to a new AABB by removing and reinserting into the tree.
export function b2DynamicTree_MoveProxy(tree: b2DynamicTree, proxyId: i32, aabb: b2AABB): void {
	b2DynamicTree_MoveProxyInternal(tree, proxyId, aabb, false);
}

export function b2DynamicTree_EnlargeProxy(tree: b2DynamicTree, proxyId: i32, aabb: b2AABB): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);

	let nodes = tree.nodes;
	let parents = tree.parents;

	let index = tree.proxies[proxyId].node;
	ref node = nodes[index];
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsLeaf(node));

	node.aabb = aabb;
	node.flagIndex |= B2_MOVED_NODE;

	index = parents[index];
	while (index != B2_NULL_INDEX) {
		ref ancestor = nodes[index];
		let changed = b2EnlargeAABB(ancestor.aabb, aabb);

		// This is marked to ensure the root is marked in this loop or the one below.
		ancestor.flagIndex |= B2_MOVED_NODE;

		index = parents[index];

		if (changed == false) {
			break;
		}
	}

	// Mark all the way up to the root.
	while (index != B2_NULL_INDEX) {
		ref ancestor = nodes[index];
		if (ancestor.flagIndex & B2_MOVED_NODE) {
			// Early out because this ancestor was previously ascended and marked as moved.
			break;
		}

		ancestor.flagIndex |= B2_MOVED_NODE;
		index = parents[index];
	}
}

/// Modify the category bits on a proxy. This is an expensive operation.
export function b2DynamicTree_SetCategoryBits(tree: b2DynamicTree, proxyId: i32, categoryBits: u64): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	tree.proxies[proxyId].categoryBits = categoryBits;
}

/// Get the category bits on a proxy.
export function b2DynamicTree_GetCategoryBits(tree: b2DynamicTree, proxyId: i32): u64 {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	return tree.proxies[proxyId].categoryBits;
}

/// Get the height of the binary tree.
export function b2DynamicTree_GetHeight(tree: b2DynamicTree): i32 {
	return b2GetNodeHeight(tree.nodes[B2_ROOT_NODE]);
}

// The area ratio is the thing that SAH seeks to minimize. SAH
// cannot do anything about leaf boxes or the root box. It seeks
// to minimize the area of all non-root internal nodes. Divide this
// by the root area to make the metric non-dimensional.
// So this becomes a meaningful measure of tree quality.
/// Get the ratio of the sum of the node areas to the root area.
export function b2DynamicTree_GetAreaRatio(tree: b2DynamicTree): f32 {
	if (tree.proxyCount == 0) {
		return 0.0;
	}

	let nodes = tree.nodes;
	let rootArea = b2Perimeter(nodes[B2_ROOT_NODE].aabb);
	if (rootArea <= 0.0) {
		return 0.0;
	}

	// Free nodes and the empty node are leaf tagged.
	let internalArea: f32 = 0.0;
	let nodeEnd = tree.nodeEnd;
	for (let i = 2; i < nodeEnd; ++i) {
		if (b2IsLeaf(nodes[i]) == false) {
			internalArea += b2Perimeter(nodes[i].aabb);
		}
	}

	return internalArea / rootArea;
}

/// Get the bounding box that contains the entire tree
export function b2DynamicTree_GetRootBounds(tree: b2DynamicTree): b2AABB {
	if (tree.proxyCount == 0) {
		return b2AABB(b2Vec2_zero, b2Vec2_zero);
	}

	return tree.nodes[B2_ROOT_NODE].aabb;
}

function b2ValidateSubtree(tree: b2DynamicTree, nodeIndex: i32, leafCount: ref i32): i32 {
	B2_ASSERT(0 <= nodeIndex && nodeIndex < tree.nodeEnd);
	let node = tree.nodes[nodeIndex];
	B2_ASSERT(b2IsEmptyNode(node) == false);

	if (b2IsLeaf(node)) {
		let proxyId = b2GetProxyId(node);
		B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
		B2_ASSERT(tree.proxies[proxyId].node == nodeIndex);
		B2_ASSERT(<i32>tree.proxies[proxyId].userData == node.shapeIndex);
		B2_ASSERT(node.height == 0);
		leafCount += 1;
		return 0;
	}

	let pair = b2GetLeftChild(node);
	B2_ASSERT((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd);
	B2_ASSERT(tree.parents[pair] == nodeIndex);
	B2_ASSERT(tree.parents[pair + 1] == nodeIndex);
	B2_ASSERT(tree.dfsOrdered == false || nodeIndex < pair);
	B2_ASSERT(node.shapeIndex == 0);

	let c1 = tree.nodes[pair];
	let c2 = tree.nodes[pair + 1];
	B2_ASSERT(b2AABB_Contains(node.aabb, c1.aabb));
	B2_ASSERT(b2AABB_Contains(node.aabb, c2.aabb));
	B2_ASSERT(b2IsNodeMoved(node) == (b2IsNodeMoved(c1) || b2IsNodeMoved(c2)));

	// A bad tree can stack overflow, but that is validation on its own.
	let height1 = b2ValidateSubtree(tree, pair, leafCount);
	let height2 = b2ValidateSubtree(tree, pair + 1, leafCount);
	let height = 1 + b2MaxInt(height1, height2);
	B2_ASSERT(node.height == height);
	return height;
}

/// Validate this tree. For testing.
export function b2DynamicTree_Validate(tree: b2DynamicTree): void {
	if (B2_TREE_VALIDATION) {
		B2_ASSERT(2 <= tree.nodeEnd && tree.nodeEnd <= tree.nodeCapacity);
		B2_ASSERT((tree.nodeEnd & 1) == 0);
		B2_ASSERT(tree.parents[B2_ROOT_NODE] == B2_NULL_INDEX);
		B2_ASSERT(b2IsEmptyNode(tree.nodes[B2_ROOT_NODE + 1]));

		let freePairCount = 0;
		let pair = tree.pairFreeList;
		while (pair != B2_NULL_INDEX) {
			B2_ASSERT((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd);
			B2_ASSERT(b2IsEmptyNode(tree.nodes[pair]));
			B2_ASSERT(b2IsEmptyNode(tree.nodes[pair + 1]));
			pair = tree.parents[pair];
			++freePairCount;
			B2_ASSERT(2 * freePairCount < tree.nodeEnd);
		}

		// Validate proxy free list.
		let freeProxyCount = 0;
		let freeIndex = tree.proxyFreeList;
		while (freeIndex != B2_NULL_INDEX) {
			B2_ASSERT(0 <= freeIndex && freeIndex < tree.proxyCapacity);
			B2_ASSERT(tree.proxies[freeIndex].node == B2_NULL_INDEX);
			freeIndex = tree.proxies[freeIndex].next;
			++freeProxyCount;
		}
		B2_ASSERT(tree.proxyCount + freeProxyCount == tree.proxyCapacity);

		B2_ASSERT(tree.nodeEnd == 2 * b2MaxInt(tree.proxyCount, 1) + 2 * freePairCount);

		if (tree.proxyCount == 0) {
			B2_ASSERT(b2IsEmptyNode(tree.nodes[B2_ROOT_NODE]));
			return;
		}

		// A ref binds a fixed array element but not a plain local
		let leafCount: FixedArray<i32, 1>;
		b2ValidateSubtree(tree, B2_ROOT_NODE, leafCount[0]);
		B2_ASSERT(leafCount[0] == tree.proxyCount);
	}
}

/// Validate this tree has no moved nodes. For testing.
export function b2DynamicTree_ValidateNoMoved(tree: b2DynamicTree): void {
	if (B2_TREE_VALIDATION) {
		let nodes = tree.nodes;
		let nodeEnd = tree.nodeEnd;
		for (let i = 0; i < nodeEnd; ++i) {
			B2_ASSERT(b2IsNodeMoved(nodes[i]) == false);
		}
	}
}

/// Get the number of bytes used by this tree
export function b2DynamicTree_GetByteCount(tree: b2DynamicTree): i32 {
	let size = B2_DYNAMIC_TREE_BYTES;
	size += tree.nodeCapacity * B2_TREE_NODE_BYTES;
	size += tree.nodeCapacity * sizeof<i32>();
	size += tree.proxyCapacity * B2_TREE_PROXY_BYTES;
	size += tree.swapNodes.length == 0 ? 0 : tree.nodeCapacity * B2_TREE_NODE_BYTES;

	// leafIndices
	size += tree.rebuildCapacity * sizeof<i32>();
	// leafNodes
	size += tree.rebuildCapacity * B2_TREE_NODE_BYTES;
	// leafBoxes
	size += tree.rebuildCapacity * B2_AABB_BYTES;
	// leafCenters
	size += tree.rebuildCapacity * B2_VEC2_BYTES;
	// binIndices
	size += tree.rebuildCapacity * sizeof<i32>();

	return size;
}

/// Get proxy user data
export function b2DynamicTree_GetUserData(tree: b2DynamicTree, proxyId: i32): u64 {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	return tree.proxies[proxyId].userData;
}

/// Get the AABB of a proxy
export function b2DynamicTree_GetProxyBounds(tree: b2DynamicTree, proxyId: i32): b2AABB {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	let nodeIndex = tree.proxies[proxyId].node;
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= nodeIndex && nodeIndex < tree.nodeEnd);
	return tree.nodes[nodeIndex].aabb;
}

// C keeps the query stacks on the C stack. Here each nesting depth keeps one, so a callback may query
// again and no query allocates once its depth has been reached.
const b2_queryStack = new StaticArray<i32>(B2_TREE_STACK_SIZE);
const b2_nestedQueryStacks = new Array<StaticArray<i32>>();
let b2_queryDepth = 0;

function b2AcquireQueryStack(): StaticArray<i32> {
	b2_queryDepth += 1;
	if (b2_queryDepth == 1) {
		return b2_queryStack;
	}
	while (b2_nestedQueryStacks.length < b2_queryDepth - 1) {
		b2_nestedQueryStacks.push(new StaticArray<i32>(B2_TREE_STACK_SIZE));
	}
	return b2_nestedQueryStacks[b2_queryDepth - 2];
}

// V is b2TreeQueryCallbackFcn, or a class whose visit(proxyId, userData) method is called directly.
function b2OverlapAABBStack<V>(
	tree: b2DynamicTree,
	aabb: b2AABB,
	maskBits: u64,
	testCategory: bool,
	callback: V,
	stack: StaticArray<i32>,
): b2TreeStats {
	let result = b2TreeStats();

	let nodes = tree.nodes;
	let proxies = tree.proxies;

	let stackCount = 0;
	stack[stackCount++] = b2GetRootPair(nodes);

	let boxv = b2LoadAABBV(aabb);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		result.nodeVisits += 1;

		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];
			if (b2OverlapNode(boxv, node)) {
				let flags = node.flagIndex;
				if ((flags & B2_LEAF_NODE) == B2_LEAF_NODE) {
					// callback to user code with proxy id
					let proxyId = <i32>(flags & B2_NODE_INDEX_MASK);
					ref proxy = proxies[proxyId];
					if (testCategory == false || (proxy.categoryBits & maskBits) != 0) {
						let proceed: bool;
						if (isFunction<V>()) {
							proceed = callback(proxyId, proxy.userData);
						} else {
							proceed = callback.visit(proxyId, proxy.userData);
						}
						result.leafVisits += 1;

						if (proceed == false) {
							return result;
						}
					}
				} else {
					if (stackCount < B2_TREE_STACK_SIZE - 1) {
						stack[stackCount++] = <i32>(flags & B2_NODE_INDEX_MASK);
					} else {
						if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE - 1);
					}
				}
			}
		}
	}

	return result;
}

/// Query an AABB for overlapping proxies. The callback function is called for each proxy that overlaps the supplied AABB.
/// The callback is a b2TreeQueryCallbackFcn or an object with a visit(proxyId, userData) method.
/// @return performance data
export function b2DynamicTree_OverlapAABB<V>(tree: b2DynamicTree, aabb: b2AABB, maskBits: u64, callback: V): b2TreeStats {
	if (tree.proxyCount == 0) {
		return b2TreeStats();
	}

	let stack = b2AcquireQueryStack();
	let result = b2OverlapAABBStack<V>(tree, aabb, maskBits, true, callback, stack);
	b2_queryDepth -= 1;
	return result;
}

/// As b2DynamicTree_OverlapAABB on the caller's own stack of B2_TREE_STACK_SIZE entries, as a worker's query needs
export function b2DynamicTree_OverlapAABBWith<V>(
	tree: b2DynamicTree,
	aabb: b2AABB,
	maskBits: u64,
	callback: V,
	stack: StaticArray<i32>,
): b2TreeStats {
	if (tree.proxyCount == 0) {
		return b2TreeStats();
	}

	return b2OverlapAABBStack<V>(tree, aabb, maskBits, true, callback, stack);
}

// This ignores the category bits. Useful for debug draw.
/// Query an AABB for overlapping proxies, ignoring the category bits.
export function b2DynamicTree_OverlapAABBAll<V>(tree: b2DynamicTree, aabb: b2AABB, callback: V): b2TreeStats {
	if (tree.proxyCount == 0) {
		return b2TreeStats();
	}

	let stack = b2AcquireQueryStack();
	let result = b2OverlapAABBStack<V>(tree, aabb, 0, false, callback, stack);
	b2_queryDepth -= 1;
	return result;
}

// A lot of optimization work went into this. It beats the slab test by a significant margin.
// It is faster than having category bits in the nodes because of the cache line friendly node
// size (64 bytes).
function b2CastRayStack<V>(tree: b2DynamicTree, input: b2RayCastInput, maskBits: u64, callback: V, stack: StaticArray<i32>): b2TreeStats {
	let result = b2TreeStats();

	let p1 = input.origin;
	let d = input.translation;

	let r = b2Normalize(d);

	// v is perpendicular to the segment.
	let v = b2CrossSV(1.0, r);
	let abs_v = b2Abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)
	let maxFraction = input.maxFraction;

	let p2 = b2MulAdd(p1, maxFraction, d);

	// Build a bounding box for the segment.
	let segmentAABB = b2AABB(b2Min(p1, p2), b2Max(p1, p2));

	let boxv = b2LoadAABBV(segmentAABB);

	let nodes = tree.nodes;
	let proxies = tree.proxies;

	let stackCount = 0;
	stack[stackCount++] = b2GetRootPair(nodes);

	let subInput = input;

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		result.nodeVisits += 1;

		// C's hit pointers become node indices.
		let hit0 = 0;
		let hit1 = 0;
		let isLeaf0 = false;
		let isLeaf1 = false;
		let hitCount = 0;
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];

			if (b2OverlapNode(boxv, node) == false) {
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			let nodeAABB = node.aabb;
			let c = b2AABB_Center(nodeAABB);
			let h = b2AABB_Extents(nodeAABB);
			let term1 = b2AbsFloat(b2Dot(v, b2Sub(p1, c)));
			let term2 = b2Dot(abs_v, h);
			if (term2 < term1) {
				continue;
			}

			if (hitCount == 0) {
				isLeaf0 = b2IsLeaf(node);
				hit0 = pair + i;
			} else {
				isLeaf1 = b2IsLeaf(node);
				hit1 = pair + i;
			}
			hitCount += 1;
		}

		if (hitCount == 2 && isLeaf0 == false && isLeaf1 == false) {
			let center1 = b2AABB_Center(nodes[hit0].aabb);
			let center2 = b2AABB_Center(nodes[hit1].aabb);
			let d1 = b2DistanceSquared(center1, p1);
			let d2 = b2DistanceSquared(center2, p1);

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if (d1 < d2) {
				let temp = hit0;
				hit0 = hit1;
				hit1 = temp;
			}
		}

		for (let i = 0; i < hitCount; ++i) {
			let hitIndex = i == 0 ? hit0 : hit1;
			let hitLeaf = i == 0 ? isLeaf0 : isLeaf1;
			if (hitLeaf) {
				let proxyId = b2GetProxyId(nodes[hitIndex]);
				ref proxy = proxies[proxyId];

				if ((proxy.categoryBits & maskBits) == 0) {
					continue;
				}

				subInput.maxFraction = maxFraction;

				let value: f32;
				if (isFunction<V>()) {
					value = callback(subInput, proxyId, proxy.userData);
				} else {
					value = callback.visit(subInput, proxyId, proxy.userData);
				}
				result.leafVisits += 1;

				// The user may return -1 to indicate this shape should be skipped

				if (value == 0.0) {
					// The client has terminated the ray cast.
					return result;
				}

				if (0.0 < value && value <= maxFraction) {
					// Update segment bounding box.
					maxFraction = value;
					p2 = b2MulAdd(p1, maxFraction, d);
					segmentAABB.lowerBound = b2Min(p1, p2);
					segmentAABB.upperBound = b2Max(p1, p2);

					boxv = b2LoadAABBV(segmentAABB);
				}
			} else {
				if (stackCount < B2_TREE_STACK_SIZE - 1) {
					stack[stackCount++] = b2GetLeftChild(nodes[hitIndex]);
				} else {
					if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE - 1);
				}
			}
		}
	}

	return result;
}

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform an exact ray cast in the case where the proxy contains a shape.
/// The callback also performs any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// The callback is a b2TreeRayCastCallbackFcn or an object with a visit(input, proxyId, userData) method.
/// @return performance data
export function b2DynamicTree_CastRay<V>(tree: b2DynamicTree, input: b2RayCastInput, maskBits: u64, callback: V): b2TreeStats {
	if (tree.proxyCount == 0) {
		return b2TreeStats();
	}

	let stack = b2AcquireQueryStack();
	let result = b2CastRayStack<V>(tree, input, maskBits, callback, stack);
	b2_queryDepth -= 1;
	return result;
}

// Follows structure of ray cast with small tweaks to handle a swept box.
function b2CastBoxStack<V>(tree: b2DynamicTree, input: b2BoxCastInput, maskBits: u64, callback: V, stack: StaticArray<i32>): b2TreeStats {
	let result = b2TreeStats();

	let originAABB = input.box;

	let p1 = b2AABB_Center(originAABB);
	let extension = b2AABB_Extents(originAABB);

	// v is perpendicular to the segment.
	let r = input.translation;
	let v = b2CrossSV(1.0, r);
	let abs_v = b2Abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	let maxFraction = input.maxFraction;

	// Build total box for the cast
	let t = b2MulSV(maxFraction, input.translation);
	let totalAABB = b2AABB(
		b2Min(originAABB.lowerBound, b2Add(originAABB.lowerBound, t)),
		b2Max(originAABB.upperBound, b2Add(originAABB.upperBound, t)),
	);

	let boxv = b2LoadAABBV(totalAABB);

	let nodes = tree.nodes;
	let proxies = tree.proxies;

	let stackCount = 0;
	stack[stackCount++] = b2GetRootPair(nodes);

	let subInput = input;

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		result.nodeVisits += 1;

		let hit0 = 0;
		let hit1 = 0;
		let isLeaf0 = false;
		let isLeaf1 = false;
		let hitCount = 0;
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];

			if (b2OverlapNode(boxv, node) == false) {
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			// radius extension is added to the node in this case
			let nodeAABB = node.aabb;
			let c = b2AABB_Center(nodeAABB);
			let h = b2Add(b2AABB_Extents(nodeAABB), extension);
			let term1 = b2AbsFloat(b2Dot(v, b2Sub(p1, c)));
			let term2 = b2Dot(abs_v, h);
			if (term2 < term1) {
				continue;
			}

			if (hitCount == 0) {
				isLeaf0 = b2IsLeaf(node);
				hit0 = pair + i;
			} else {
				isLeaf1 = b2IsLeaf(node);
				hit1 = pair + i;
			}
			hitCount += 1;
		}

		if (hitCount == 2 && isLeaf0 == false && isLeaf1 == false) {
			let center1 = b2AABB_Center(nodes[hit0].aabb);
			let center2 = b2AABB_Center(nodes[hit1].aabb);
			let d1 = b2DistanceSquared(center1, p1);
			let d2 = b2DistanceSquared(center2, p1);

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if (d1 < d2) {
				let temp = hit0;
				hit0 = hit1;
				hit1 = temp;
			}
		}

		for (let i = 0; i < hitCount; ++i) {
			let hitIndex = i == 0 ? hit0 : hit1;
			let hitLeaf = i == 0 ? isLeaf0 : isLeaf1;
			if (hitLeaf) {
				let proxyId = b2GetProxyId(nodes[hitIndex]);
				ref proxy = proxies[proxyId];

				if ((proxy.categoryBits & maskBits) == 0) {
					continue;
				}

				subInput.maxFraction = maxFraction;

				let value: f32;
				if (isFunction<V>()) {
					value = callback(subInput, proxyId, proxy.userData);
				} else {
					value = callback.visit(subInput, proxyId, proxy.userData);
				}
				result.leafVisits += 1;

				// The user may return -1 to indicate this shape should be skipped

				if (value == 0.0) {
					// The client has terminated the ray cast.
					return result;
				}

				if (0.0 < value && value < maxFraction) {
					// Update cast bounding box.
					maxFraction = value;
					t = b2MulSV(maxFraction, input.translation);
					totalAABB.lowerBound = b2Min(originAABB.lowerBound, b2Add(originAABB.lowerBound, t));
					totalAABB.upperBound = b2Max(originAABB.upperBound, b2Add(originAABB.upperBound, t));
					boxv = b2LoadAABBV(totalAABB);
				}
			} else {
				if (stackCount < B2_TREE_STACK_SIZE - 1) {
					stack[stackCount++] = b2GetLeftChild(nodes[hitIndex]);
				} else {
					if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE - 1);
				}
			}
		}
	}

	return result;
}

/// Sweep an AABB through the tree. The callback is a b2TreeBoxCastCallbackFcn or an object with a
/// visit(input, proxyId, userData) method.
/// @return performance data
export function b2DynamicTree_CastBox<V>(tree: b2DynamicTree, input: b2BoxCastInput, maskBits: u64, callback: V): b2TreeStats {
	if (tree.proxyCount == 0) {
		return b2TreeStats();
	}

	let stack = b2AcquireQueryStack();
	let result = b2CastBoxStack<V>(tree, input, maskBits, callback, stack);
	b2_queryDepth -= 1;
	return result;
}

// Median split == 0, Surface area heuristic == 1. C compiles only the median split.

// Median split heuristic. C's pointers into the leaf arrays become the arrays plus a base index.
// Unchecked: every index is in [base, base + count), which b2BuildTree keeps inside the rebuild's leaf arrays
@unchecked
function b2PartitionMid(indices: StaticArray<i32>, centers: StaticArray<b2Vec2>, base: i32, count: i32): i32 {
	// Handle trivial case
	if (count <= 2) {
		return count / 2;
	}

	let lowerBound = centers[base];
	let upperBound = centers[base];

	for (let i = 1; i < count; ++i) {
		lowerBound = b2Min(lowerBound, centers[base + i]);
		upperBound = b2Max(upperBound, centers[base + i]);
	}

	let d = b2Sub(upperBound, lowerBound);
	let c = b2Vec2(0.5 * (lowerBound.x + upperBound.x), 0.5 * (lowerBound.y + upperBound.y));

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	let i1 = 0;
	let i2 = count;
	if (d.x > d.y) {
		let pivot = c.x;

		while (i1 < i2) {
			while (i1 < i2 && centers[base + i1].x < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && centers[base + i2 - 1].x >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				b2SwapLeaves(indices, centers, base + i1, base + i2 - 1);
				i1 += 1;
				i2 -= 1;
			}
		}
	} else {
		let pivot = c.y;

		while (i1 < i2) {
			while (i1 < i2 && centers[base + i1].y < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && centers[base + i2 - 1].y >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				b2SwapLeaves(indices, centers, base + i1, base + i2 - 1);
				i1 += 1;
				i2 -= 1;
			}
		}
	}
	if (B2_ASSERT_ENABLED) B2_ASSERT(i1 == i2);

	if (i1 > 0 && i1 < count) {
		return i1;
	}

	return count / 2;
}

// Swap indices and centers. Only b2PartitionMid calls it, with indices in its range.
@unchecked
@inline function b2SwapLeaves(indices: StaticArray<i32>, centers: StaticArray<b2Vec2>, a: i32, b: i32): void {
	let tempIndex = indices[a];
	indices[a] = indices[b];
	indices[b] = tempIndex;

	let tempCenter = centers[a];
	centers[a] = centers[b];
	centers[b] = tempCenter;
}

// Temporary data used to track the rebuild of a tree node.
export struct b2RebuildItem {
	// Where this node is written and the pair its children go in.
	nodeIndex: i32 = 0;
	pair: i32 = 0;
	childCount: i32 = 0;

	// Leaf indices
	startIndex: i32 = 0;
	splitIndex: i32 = 0;
	endIndex: i32 = 0;
}

export struct b2CopyItem {
	oldPair: i32 = 0;
	newIndex: i32 = 0;

	constructor(oldPair: i32, newIndex: i32) {
		this.oldPair = oldPair;
		this.newIndex = newIndex;
	}
}

// The rebuild, refit and moved flag walks never call out, so they share static stacks. In a parallel call only the
// part that rebuilds walks a tree.
@parallel.shared const b2_rebuildStack = new StaticArray<b2RebuildItem>(B2_TREE_STACK_SIZE);
@parallel.shared const b2_copyStack = new StaticArray<b2CopyItem>(B2_TREE_STACK_SIZE);
@parallel.shared const b2_walkStack = new StaticArray<i32>(B2_TREE_STACK_SIZE);

// Bump allocate a pair of sibling nodes. Returns index to the first one.
@inline function b2BumpPair(tree: b2DynamicTree, parent: i32): i32 {
	let pair = tree.nodeEnd;
	if (B2_ASSERT_ENABLED) B2_ASSERT(pair + 2 <= tree.nodeCapacity);
	tree.nodeEnd += 2;
	tree.parents[pair] = parent;
	tree.parents[pair + 1] = parent;
	return pair;
}

@inline function b2SetLeftChild(node: ref b2TreeNode, pair: i32): void {
	node.flagIndex = (node.flagIndex & ~B2_NODE_INDEX_MASK) | <u32>pair;
}

// Copy a retained subtree from the old array into the rebuilt DFS array using
// a bump allocator.
function b2CopySubtree(tree: b2DynamicTree, node: b2TreeNode, newIndex: i32): void {
	let oldNodes = tree.nodes;
	let newNodes = tree.swapNodes;
	let proxies = tree.proxies;

	if (b2IsLeaf(node)) {
		newNodes[newIndex] = node;
		proxies[b2GetProxyId(node)].node = newIndex;
		return;
	}

	let stack = b2_copyStack;
	let stackCount = 0;

	let oldPair = b2GetLeftChild(node);
	let newPair = b2BumpPair(tree, newIndex);
	b2SetLeftChild(node, newPair);

	// Copy the subtree root. This can be a left or right sibling (even or odd).
	newNodes[newIndex] = node;

	for (;;) {
		// Copy the siblings.
		newNodes[newPair] = oldNodes[oldPair];
		newNodes[newPair + 1] = oldNodes[oldPair + 1];

		let flags0 = newNodes[newPair].flagIndex;
		let flags1 = newNodes[newPair + 1].flagIndex;

		if ((flags1 & B2_LEAF_NODE) == B2_LEAF_NODE) {
			// Hook up proxy on right sibling.
			let proxyId = <i32>(flags1 & B2_NODE_INDEX_MASK);
			proxies[proxyId].node = newPair + 1;
		} else {
			// Push the left child of the right sibling.
			if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE);
			let leftChild = <i32>(flags1 & B2_NODE_INDEX_MASK);
			stack[stackCount++] = b2CopyItem(leftChild, newPair + 1);
		}

		if ((flags0 & B2_LEAF_NODE) != B2_LEAF_NODE) {
			// Descend into left subtree.
			let leftIndex = newPair;
			oldPair = <i32>(flags0 & B2_NODE_INDEX_MASK);
			newPair = b2BumpPair(tree, leftIndex);
			b2SetLeftChild(newNodes[leftIndex], newPair);
			continue;
		}

		// Hook up proxy on left sibling.
		proxies[<i32>(flags0 & B2_NODE_INDEX_MASK)].node = newPair;

		if (stackCount == 0) {
			break;
		}

		// Descend a right subtree that was previously pushed.
		let item = stack[--stackCount];
		oldPair = item.oldPair;
		newPair = b2BumpPair(tree, item.newIndex);
		b2SetLeftChild(newNodes[item.newIndex], newPair);
	}
}

@inline function b2PlaceLeaf(tree: b2DynamicTree, leaf: b2TreeNode, newIndex: i32): void {
	if (b2IsLeaf(leaf)) {
		tree.swapNodes[newIndex] = leaf;
		tree.proxies[b2GetProxyId(leaf)].node = newIndex;
	} else {
		b2CopySubtree(tree, leaf, newIndex);
	}
}

function b2BuildTree(tree: b2DynamicTree, leafCount: i32): void {
	let nodes = tree.swapNodes;
	let leaves = tree.leafNodes;
	let leafIndices = tree.leafIndices;
	let leafCenters = tree.leafCenters;

	// Bump allocation into the swap nodes.
	tree.nodeEnd = 2;
	nodes[B2_ROOT_NODE + 1] = b2MakeEmptyNode();
	tree.parents[B2_ROOT_NODE] = B2_NULL_INDEX;
	tree.parents[B2_ROOT_NODE + 1] = B2_NULL_INDEX;

	if (leafCount == 1) {
		b2PlaceLeaf(tree, leaves[leafIndices[0]], B2_ROOT_NODE);
		return;
	}

	let stack = b2_rebuildStack;
	let top = 0;

	stack[0].nodeIndex = B2_ROOT_NODE;
	stack[0].pair = b2BumpPair(tree, B2_ROOT_NODE);
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
	stack[0].splitIndex = b2PartitionMid(leafIndices, leafCenters, 0, leafCount);

	while (true) {
		ref item = stack[top];
		item.childCount += 1;

		if (item.childCount == 2) {
			// Both children written, so the parent node can be finalized.
			nodes[item.nodeIndex] = b2MakeInternalNode(nodes, item.pair);
			if (top == 0) {
				break;
			}

			top -= 1;
			continue;
		}

		let slot = item.childCount;
		let startIndex = slot == 0 ? item.startIndex : item.splitIndex;
		let endIndex = slot == 0 ? item.splitIndex : item.endIndex;
		let count = endIndex - startIndex;
		if (B2_ASSERT_ENABLED) B2_ASSERT(count > 0);

		let nodeIndex = item.pair + slot;
		if (count == 1) {
			b2PlaceLeaf(tree, leaves[leafIndices[startIndex]], nodeIndex);
			continue;
		}

		// todo fail gracefully if the stack runs out in release
		if (B2_ASSERT_ENABLED) B2_ASSERT(top < B2_TREE_STACK_SIZE - 1);

		top += 1;
		ref newItem = stack[top];
		newItem.nodeIndex = nodeIndex;
		newItem.pair = b2BumpPair(tree, nodeIndex);
		newItem.childCount = -1;
		newItem.startIndex = startIndex;
		newItem.endIndex = endIndex;
		newItem.splitIndex = startIndex;
		newItem.splitIndex += b2PartitionMid(leafIndices, leafCenters, startIndex, count);
	}
}

// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
// refitting much faster. This is done async with threading, so the cost is hidden.
// Not safe to access tree during this operation.
/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
export function b2DynamicTree_Rebuild(tree: b2DynamicTree, fullBuild: bool): i32 {
	b2DynamicTree_ReserveRebuild(tree, fullBuild);
	let leafCount = b2DynamicTree_BuildRebuild(tree, fullBuild);
	b2DynamicTree_FinishRebuild(tree);
	return leafCount;
}

// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
@inline function b2RebuildNeeded(tree: b2DynamicTree, fullBuild: bool): bool {
	if (tree.proxyCount == 0) {
		return false;
	}

	return fullBuild || b2IsNodeMoved(tree.nodes[B2_ROOT_NODE]) || tree.dfsOrdered == false;
}

// b2DynamicTree_Rebuild in three steps, so a parallel part can run the middle one while the others collide: this
// one makes the allocations, the build writes numbers only, and the finish swaps the node arrays.
export function b2DynamicTree_ReserveRebuild(tree: b2DynamicTree, fullBuild: bool): void {
	if (b2RebuildNeeded(tree, fullBuild) == false) {
		return;
	}

	if (tree.swapNodes.length == 0) {
		tree.swapNodes = new StaticArray<b2TreeNode>(tree.nodeCapacity);
	}

	let proxyCount = tree.proxyCount;
	if (proxyCount > tree.rebuildCapacity) {
		let newCapacity = proxyCount + proxyCount / 2;
		tree.leafIndices = new StaticArray<i32>(newCapacity);
		tree.leafNodes = new StaticArray<b2TreeNode>(newCapacity);
		tree.leafCenters = new StaticArray<b2Vec2>(newCapacity);
		tree.rebuildCapacity = newCapacity;
	}
}

// Builds into swapNodes what b2DynamicTree_ReserveRebuild made room for, unless a build already waits for its finish.
// Returns the number of boxes sorted.
export function b2DynamicTree_BuildRebuild(tree: b2DynamicTree, fullBuild: bool): i32 {
	if (tree.rebuildPending || b2RebuildNeeded(tree, fullBuild) == false) {
		return 0;
	}

	let proxyCount = tree.proxyCount;
	let nodes = tree.nodes;
	let rootFlags = nodes[B2_ROOT_NODE].flagIndex;
	let rootMoved = (rootFlags & B2_MOVED_NODE) == B2_MOVED_NODE;
	let leafIndices = tree.leafIndices;
	let leaves = tree.leafNodes;
	let leafCenters = tree.leafCenters;

	// Gather build leaves. A marked internal node is descended and abandoned in the old array.
	// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
	// on an unordered array, is the one build leaf.
	let leafCount = 0;
	let stack = b2_walkStack;
	let stackCount = 0;
	if ((rootFlags & B2_LEAF_NODE) != B2_LEAF_NODE && (fullBuild || rootMoved)) {
		stack[stackCount++] = <i32>(rootFlags & B2_NODE_INDEX_MASK);
	} else {
		let node = nodes[B2_ROOT_NODE];
		node.flagIndex &= ~B2_MOVED_NODE;
		leafIndices[0] = 0;
		leaves[0] = node;
		leafCenters[0] = b2AABB_Center(node.aabb);
		leafCount = 1;
	}

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			let node = nodes[pair + i];
			if (b2IsLeaf(node) == false && (fullBuild || b2IsNodeMoved(node))) {
				// todo fail gracefully if the stack runs out in release
				if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE);
				stack[stackCount++] = b2GetLeftChild(node);
				continue;
			}

			node.flagIndex &= ~B2_MOVED_NODE;
			leafIndices[leafCount] = leafCount;
			leaves[leafCount] = node;
			leafCenters[leafCount] = b2AABB_Center(node.aabb);
			leafCount += 1;
		}
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(0 < leafCount && leafCount <= proxyCount);

	b2BuildTree(tree, leafCount);
	tree.rebuildPending = true;

	return leafCount;
}

// Swaps in the nodes b2DynamicTree_BuildRebuild built, if it built any
export function b2DynamicTree_FinishRebuild(tree: b2DynamicTree): void {
	if (tree.rebuildPending == false) {
		return;
	}

	// The spare is now the tree and the old array is the spare. The build is dense, so the
	// stale tail of the old array sits above the end and is never read.
	let temp = tree.nodes;
	tree.nodes = tree.swapNodes;
	tree.swapNodes = temp;
	tree.pairFreeList = B2_NULL_INDEX;
	tree.dfsOrdered = true;
	tree.rebuildPending = false;

	b2DynamicTree_Validate(tree);
	b2DynamicTree_ValidateNoMoved(tree);
}

// Set the moved flag on the ancestors of the proxy. Serial use case.
export function b2DynamicTree_MarkProxyMovedSerial(tree: b2DynamicTree, proxyId: i32): void {
	let nodes = tree.nodes;
	let parents = tree.parents;

	let index = tree.proxies[proxyId].node;

	while (index != B2_NULL_INDEX) {
		nodes[index].flagIndex |= B2_MOVED_NODE;
		index = parents[index];
	}
}

// Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics.
// Single threaded here: the atomic load and fetch-or are a plain read and or.
@inline
export function b2DynamicTree_MarkProxyMoved(tree: b2DynamicTree, proxyId: i32, aabb: b2AABB): void {
	let nodes = tree.nodes;
	let parents = tree.parents;

	let index = tree.proxies[proxyId].node;

	ref node = nodes[index];

	// This is not raced since it is the leaf.
	node.aabb = aabb;
	node.flagIndex |= B2_MOVED_NODE;

	index = parents[index];
	while (index != B2_NULL_INDEX) {
		ref ancestor = nodes[index];

		// Read first to avoid the FetchOr if possible.
		if ((ancestor.flagIndex & B2_MOVED_NODE) != 0) {
			break;
		}

		let previousFlags = ancestor.flagIndex;
		ancestor.flagIndex = previousFlags | B2_MOVED_NODE;
		if ((previousFlags & B2_MOVED_NODE) != 0) {
			// Ancestor already visited.
			break;
		}

		index = parents[index];
	}
}

// Clear the moved flags from the entire tree.
export function b2DynamicTree_ClearMoved(tree: b2DynamicTree): void {
	let nodes = tree.nodes;

	ref root = nodes[B2_ROOT_NODE];
	if (b2IsNodeMoved(root) == false) {
		return;
	}

	root.flagIndex &= ~B2_MOVED_NODE;
	if (b2IsLeaf(root)) {
		return;
	}

	let stack = b2_walkStack;
	let stackCount = 0;
	stack[stackCount++] = b2GetLeftChild(root);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];
			if (node.flagIndex & B2_MOVED_NODE) {
				node.flagIndex &= ~B2_MOVED_NODE;
				if (b2IsLeaf(node) == false) {
					if (stackCount < B2_TREE_STACK_SIZE) {
						stack[stackCount++] = b2GetLeftChild(node);
					} else {
						// Bad stuff will happen if the moved flags don't get cleared.
						if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE);
					}
				}
			}
		}
	}
}

// proxyIds is any i32 array with room for every moved proxy.
export function b2DynamicTree_GatherMovedProxies<A>(tree: b2DynamicTree, proxyIds: A): i32 {
	let nodes = tree.nodes;
	ref root = nodes[B2_ROOT_NODE];
	if (b2IsNodeMoved(root) == false) {
		return 0;
	}

	if (b2IsLeaf(root)) {
		proxyIds[0] = b2GetProxyId(root);
		return 1;
	}

	let count = 0;
	let stack = b2_walkStack;
	let stackCount = 0;
	stack[stackCount++] = b2GetLeftChild(root);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];
			if (b2IsNodeMoved(node) == false) {
				continue;
			}

			if (b2IsLeaf(node)) {
				proxyIds[count++] = b2GetProxyId(node);
			} else {
				if (stackCount < B2_TREE_STACK_SIZE) {
					stack[stackCount++] = b2GetLeftChild(node);
				} else {
					if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE);
				}
			}
		}
	}

	return count;
}

// Slow refit for unit tests.
function b2RefitSubtree(nodes: StaticArray<b2TreeNode>, rootIndex: i32): void {
	ref root = nodes[rootIndex];
	if (b2IsLeaf(root) || b2IsNodeMoved(root) == false) {
		return;
	}

	// Negative entries indicate deferred unions. A parent is pushed before its children so the
	// LIFO order guarantees both children are finished when the union runs.
	let stack = b2_walkStack;
	let stackCount = 0;
	// ~0 == -1
	stack[stackCount++] = ~rootIndex;
	stack[stackCount++] = b2GetLeftChild(root);
	stack[stackCount++] = b2GetLeftChild(root) + 1;

	while (stackCount > 0) {
		let item = stack[--stackCount];
		if (item < 0) {
			ref parent = nodes[~item];
			let pair = b2GetLeftChild(parent);
			parent.aabb = b2UnionV(nodes[pair].aabb, nodes[pair + 1].aabb);
			continue;
		}

		ref node = nodes[item];
		if (b2IsLeaf(node) || b2IsNodeMoved(node) == false) {
			continue;
		}

		if (stackCount + 3 <= B2_TREE_STACK_SIZE) {
			let pair = b2GetLeftChild(node);
			stack[stackCount++] = ~item;
			stack[stackCount++] = pair;
			stack[stackCount++] = pair + 1;
		} else {
			if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount + 3 <= B2_TREE_STACK_SIZE);
		}
	}
}

export function b2DynamicTree_Refit(tree: b2DynamicTree): void {
	if (b2HasTreeMoved(tree) == false) {
		return;
	}

	let nodes = tree.nodes;

	if (tree.dfsOrdered == false) {
		// This only happens in unit tests.
		b2RefitSubtree(nodes, B2_ROOT_NODE);
		return;
	}

	for (let pair = tree.nodeEnd - 2; pair >= 0; pair -= 2) {
		let flags1 = nodes[pair].flagIndex;
		let flags2 = nodes[pair + 1].flagIndex;

		// Did either move?
		if (((flags1 | flags2) & B2_MOVED_NODE) == 0) {
			continue;
		}

		// Is the node internal and moved?
		let refit1 = (flags1 & (B2_LEAF_NODE | B2_MOVED_NODE)) == B2_MOVED_NODE;
		let refit2 = (flags2 & (B2_LEAF_NODE | B2_MOVED_NODE)) == B2_MOVED_NODE;

		// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
		// The store function won't actually modify the node at the dummy index. This
		// avoids an unpredictable branch. SIMD FTW.
		let children1 = refit1 ? <i32>(flags1 & B2_NODE_INDEX_MASK) : pair;
		let children2 = refit2 ? <i32>(flags2 & B2_NODE_INDEX_MASK) : pair;

		// Conditionally store the union.
		b2StoreAABBV(nodes[pair].aabb, b2UnionPairV(nodes, children1), refit1);
		b2StoreAABBV(nodes[pair + 1].aabb, b2UnionPairV(nodes, children2), refit2);
	}
}
