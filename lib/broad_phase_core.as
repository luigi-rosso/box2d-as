// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/broad_phase.h and the world independent part of src/broad_phase.c.
// lib/broad_phase.as re-exports this and adds the parts that read the world.

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_ENABLE_VALIDATION } from "./core";
import { b2AABB, b2MaxInt, b2MinInt } from "./math_functions";
import { b2DynamicTree, b2TreeNode } from "./collision";
import {
	B2_MOVED_NODE,
	B2_ROOT_NODE,
	B2_TREE_STACK_SIZE,
	b2CreateDynamicTreeInternal,
	b2CreateTreeProxyInternal,
	b2DestroyDynamicTreeInternal,
	b2DestroyTreeProxy,
	b2DynamicTree_MarkProxyMoved,
	b2DynamicTree_MarkProxyMovedSerial,
	b2DynamicTree_MoveProxyInternal,
	b2DynamicTree_Validate,
	b2DynamicTree_ValidateNoMoved,
	b2GetLeftChild,
	b2IsLeaf,
	b2IsNodeMoved,
} from "./dynamic_tree";
import { B2_SHAPE_PAIR_KEY, b2ContainsHashedKey, b2CreateSet, b2DestroySet, b2HashSet, b2KeyHash } from "./table";
import { b2BodyType, b2Capacity, b2_bodyTypeCount, b2_dynamicBody, b2_kinematicBody, b2_staticBody } from "./types";
import { b2LoadAABBV, b2OverlapNodePairMask, b2OverlapPairMask, b2OverlapV } from "./simd";
import { b2CTZ32 } from "./ctz";

// broad_phase.h

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
@inline export function B2_PROXY_TYPE(KEY: i32): b2BodyType {
	return KEY & 3;
}

@inline export function B2_PROXY_ID(KEY: i32): i32 {
	return KEY >> 2;
}

@inline export function B2_PROXY_KEY(ID: i32, TYPE: b2BodyType): i32 {
	return (ID << 2) | TYPE;
}

const b2_emptySiblings = new StaticArray<i32>(0);

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
export class b2BroadPhase {
	// One tree for each body type.
	trees: StaticArray<b2DynamicTree> = new StaticArray<b2DynamicTree>(b2_bodyTypeCount);

	// The moved siblings gathered from the dynamic body tree.
	movedSiblings: StaticArray<i32> = b2_emptySiblings;

	// Tracks shape pairs that have a b2Contact
	pairSet: b2HashSet = new b2HashSet();

	// C's per update stack buffers, kept and grown across updates
	siblingBuffer: StaticArray<i32> = b2_emptySiblings;
	pairKeyBuffer: StaticArray<u64> = new StaticArray<u64>(0);
	tempKeyBuffer: StaticArray<u64> = new StaticArray<u64>(0);

	constructor() {
		for (let i = 0; i < b2_bodyTypeCount; ++i) {
			this.trees[i] = new b2DynamicTree();
		}
	}
}

@inline export function b2BroadPhase_MarkProxyMovedSerial(bp: b2BroadPhase, proxyKey: i32): void {
	let proxyType = B2_PROXY_TYPE(proxyKey);
	let proxyId = B2_PROXY_ID(proxyKey);
	b2DynamicTree_MarkProxyMovedSerial(bp.trees[proxyType], proxyId);
}

@inline export function b2BroadPhase_MarkProxyMoved(bp: b2BroadPhase, proxyKey: i32, aabb: b2AABB): void {
	let proxyType = B2_PROXY_TYPE(proxyKey);
	let proxyId = B2_PROXY_ID(proxyKey);
	b2DynamicTree_MarkProxyMoved(bp.trees[proxyType], proxyId, aabb);
}

// broad_phase.c

export function b2CreateBroadPhase(bp: b2BroadPhase, capacity: b2Capacity): void {
	bp.pairSet = b2CreateSet(b2MaxInt(32, 2 * capacity.contactCount));

	let staticCapacity = b2MaxInt(16, capacity.staticShapeCount);
	b2CreateDynamicTreeInternal(bp.trees[b2_staticBody], staticCapacity);

	let kinematicCapacity = 16;
	b2CreateDynamicTreeInternal(bp.trees[b2_kinematicBody], kinematicCapacity);

	let dynamicCapacity = b2MaxInt(16, capacity.dynamicShapeCount);
	b2CreateDynamicTreeInternal(bp.trees[b2_dynamicBody], dynamicCapacity);

	bp.movedSiblings = b2_emptySiblings;
}

export function b2DestroyBroadPhase(bp: b2BroadPhase): void {
	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		b2DestroyDynamicTreeInternal(bp.trees[i]);
	}

	b2DestroySet(bp.pairSet);

	// memset
	bp.movedSiblings = b2_emptySiblings;
	bp.pairSet = new b2HashSet();
	bp.siblingBuffer = b2_emptySiblings;
	bp.pairKeyBuffer = new StaticArray<u64>(0);
	bp.tempKeyBuffer = new StaticArray<u64>(0);
}

export function b2BroadPhase_CreateProxy(
	bp: b2BroadPhase,
	proxyType: b2BodyType,
	aabb: b2AABB,
	categoryBits: u64,
	shapeIndex: i32,
	forcePairCreation: bool,
): i32 {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyType && proxyType < b2_bodyTypeCount);

	let mark = proxyType != b2_staticBody || forcePairCreation;

	let proxyId = b2CreateTreeProxyInternal(bp.trees[proxyType], aabb, categoryBits, <u64>shapeIndex, mark);
	let proxyKey = B2_PROXY_KEY(proxyId, proxyType);
	return proxyKey;
}

export function b2BroadPhase_DestroyProxy(bp: b2BroadPhase, proxyKey: i32): void {
	let proxyType = B2_PROXY_TYPE(proxyKey);
	let proxyId = B2_PROXY_ID(proxyKey);

	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= proxyType && proxyType < b2_bodyTypeCount);
	b2DestroyTreeProxy(bp.trees[proxyType], proxyId);
}

export function b2BroadPhase_MoveProxy(bp: b2BroadPhase, proxyKey: i32, aabb: b2AABB): void {
	let proxyType = B2_PROXY_TYPE(proxyKey);
	let proxyId = B2_PROXY_ID(proxyKey);

	let mark = true;
	b2DynamicTree_MoveProxyInternal(bp.trees[proxyType], proxyId, aabb, mark);
}

// Gather the sibling pairs with a moved node. This is done serially it is cache friendly.
export function b2GatherMovedSiblings(tree: b2DynamicTree, pairIndices: StaticArray<i32>): i32 {
	let nodes = tree.nodes;
	let nodeEnd = tree.nodeEnd;

	let count = 0;

	// Skip the root.
	for (let pair = 2; pair < nodeEnd; pair += 2) {
		// Push when either sibling moved.
		if (((nodes[pair].flagIndex | nodes[pair + 1].flagIndex) & B2_MOVED_NODE) != 0) {
			pairIndices[count++] = pair;
		}
	}

	return count;
}

export const B2_CANDIDATE_BATCH: i32 = 32;

export struct b2CandidatePair {
	shapeIdA: i32 = 0;
	shapeIdB: i32 = 0;
}

// C's pair context holds the world and lives on the stack of each task. Here it holds the world's pair
// filter F, whose shouldCreatePair(shapeIdA, shapeIdB) runs the world's tests, plus C's stack buffers,
// so one context per worker serves every update.
export class b2PairContext<F> {
	filter: F;
	pairSet: b2HashSet;
	pairKeys: Array<u64>;
	batch: StaticArray<b2CandidatePair> = new StaticArray<b2CandidatePair>(B2_CANDIDATE_BATCH);
	batchCount: i32 = 0;

	keys: StaticArray<u64> = new StaticArray<u64>(B2_CANDIDATE_BATCH);
	hashes: StaticArray<u64> = new StaticArray<u64>(B2_CANDIDATE_BATCH);
	candidates: StaticArray<b2CandidatePair> = new StaticArray<b2CandidatePair>(B2_CANDIDATE_BATCH);
	proxyStack: StaticArray<i32> = new StaticArray<i32>(B2_TREE_STACK_SIZE);
	pairStack: StaticArray<b2IndexPair> = new StaticArray<b2IndexPair>(B2_TREE_STACK_SIZE);

	constructor(filter: F, pairSet: b2HashSet, pairKeys: Array<u64>) {
		this.filter = filter;
		this.pairSet = pairSet;
		this.pairKeys = pairKeys;
	}
}

export struct b2NodePair {
	a: b2TreeNode = b2TreeNode();
	b: b2TreeNode = b2TreeNode();

	constructor(a: b2TreeNode, b: b2TreeNode) {
		this.a = a;
		this.b = b;
	}
}

export struct b2IndexPair {
	a: i32 = 0;
	b: i32 = 0;

	constructor(a: i32, b: i32) {
		this.a = a;
		this.b = b;
	}
}

// b2PrefetchHash and the shape prefetches are cache hints only and are not ported.

export function b2FlushCandidatePairs<F>(context: b2PairContext<F>): void {
	let count1 = context.batchCount;
	context.batchCount = 0;

	// Every buffer holds B2_CANDIDATE_BATCH items and count1 never exceeds it
	let batch = context.batch;
	let keys = context.keys;
	let hashes = context.hashes;
	for (let i = 0; i < count1; ++i) {
		let key = B2_SHAPE_PAIR_KEY(unchecked(batch[i].shapeIdA), unchecked(batch[i].shapeIdB));
		unchecked(keys[i] = key);
		unchecked(hashes[i] = b2KeyHash(key));
	}

	// Cull existing pairs.
	let pairSet = context.pairSet;
	let candidates = context.candidates;
	let count2 = 0;
	for (let i = 0; i < count1; ++i) {
		let pairExists = b2ContainsHashedKey(pairSet, unchecked(keys[i]), unchecked(hashes[i]));
		if (pairExists == false) {
			unchecked(candidates[count2] = batch[i]);
			count2 += 1;
		}
	}

	// Filter candidates.
	let pairKeys = context.pairKeys;
	for (let i = 0; i < count2; ++i) {
		let shapeIdA = unchecked(candidates[i].shapeIdA);
		let shapeIdB = unchecked(candidates[i].shapeIdB);

		if (context.filter.shouldCreatePair(shapeIdA, shapeIdB) == false) {
			continue;
		}

		// The pair passed the gauntlet. A new contact will be created.
		pairKeys.push(B2_SHAPE_PAIR_KEY(shapeIdA, shapeIdB));
	}
}

@inline function b2AddCandidatePair<F>(shapeIdA: i32, shapeIdB: i32, context: b2PairContext<F>): void {
	// Follow shape index order.
	ref candidate = context.batch[context.batchCount];
	candidate.shapeIdA = b2MinInt(shapeIdA, shapeIdB);
	candidate.shapeIdB = b2MaxInt(shapeIdA, shapeIdB);
	context.batchCount += 1;
	if (context.batchCount == B2_CANDIDATE_BATCH) {
		b2FlushCandidatePairs<F>(context);
	}
}

// Did either move and if so do they overlap?
@inline function b2TestPair(a: b2TreeNode, b: b2TreeNode): bool {
	if (((a.flagIndex | b.flagIndex) & B2_MOVED_NODE) == 0) {
		return false;
	}

	return b2OverlapV(a.aabb, b.aabb);
}

function b2CollideProxyAndSubtree<F>(proxy: b2TreeNode, nodes: StaticArray<b2TreeNode>, pair: i32, context: b2PairContext<F>): void {
	let proxyMark = proxy.flagIndex & B2_MOVED_NODE;
	let boxv = b2LoadAABBV(proxy.aabb);
	let shapeId = proxy.shapeIndex;

	let stack = context.proxyStack;
	let stackCount = 0;
	stack[stackCount++] = pair;

	while (stackCount > 0) {
		pair = stack[--stackCount];

		let movedMask = 3;
		if (proxyMark == 0) {
			movedMask = (b2IsNodeMoved(nodes[pair]) ? 1 : 0) | (b2IsNodeMoved(nodes[pair + 1]) ? 2 : 0);
		}

		let mask = b2OverlapNodePairMask(boxv, nodes, pair) & movedMask;
		while (mask != 0) {
			let i = <i32>b2CTZ32(<u32>mask);
			mask &= mask - 1;

			ref node = nodes[pair + i];
			if (b2IsLeaf(node)) {
				b2AddCandidatePair<F>(shapeId, node.shapeIndex, context);
			} else {
				if (stackCount < B2_TREE_STACK_SIZE) {
					stack[stackCount++] = b2GetLeftChild(node);
				} else {
					if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE);
				}
			}
		}
	}
}

// Returns the new stack count, which C writes through a pointer.
@inline function b2ExpandPair<F>(
	arrayA: StaticArray<b2TreeNode>,
	arrayB: StaticArray<b2TreeNode>,
	nodeA: b2TreeNode,
	nodeB: b2TreeNode,
	stack: StaticArray<b2IndexPair>,
	stackCount: i32,
	context: b2PairContext<F>,
): i32 {
	let leafA = b2IsLeaf(nodeA);
	let leafB = b2IsLeaf(nodeB);
	if (leafA && leafB) {
		b2AddCandidatePair<F>(nodeA.shapeIndex, nodeB.shapeIndex, context);
	} else if (leafA) {
		b2CollideProxyAndSubtree<F>(nodeA, arrayB, b2GetLeftChild(nodeB), context);
	} else if (leafB) {
		b2CollideProxyAndSubtree<F>(nodeB, arrayA, b2GetLeftChild(nodeA), context);
	} else {
		if (stackCount < B2_TREE_STACK_SIZE) {
			stack[stackCount] = b2IndexPair(b2GetLeftChild(nodeA), b2GetLeftChild(nodeB));
			stackCount += 1;
		} else {
			if (B2_ASSERT_ENABLED) B2_ASSERT(stackCount < B2_TREE_STACK_SIZE);
		}
	}

	return stackCount;
}

// This collides two sub-trees against each other. They can live in the same dynamic tree.
// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
// this does not generate duplicate pairs.
// For example consider the full binary tree A (B (D  E) C (F G))
// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
// Then colliding children of B can give the pair (D,E) and for C (F,G).
// So no duplicates even when used for self-collision.
// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
// self collision gathers all those moved internal nodes and collides their subtrees together.
// See Real-time collision detection section 6.3.2. This is faster than querying every moved
// proxy against the whole tree. Scaling is linear instead of linear * log.
// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
function b2CollideCrossPairs<F>(
	arrayA: StaticArray<b2TreeNode>,
	arrayB: StaticArray<b2TreeNode>,
	subtreeA: b2TreeNode,
	subtreeB: b2TreeNode,
	context: b2PairContext<F>,
): void {
	let stack = context.pairStack;
	let stackCount = 0;

	// Seed the stack.
	if (b2TestPair(subtreeA, subtreeB)) {
		stackCount = b2ExpandPair<F>(arrayA, arrayB, subtreeA, subtreeB, stack, stackCount, context);
	}

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		let pairA = pair.a;
		let pairB = pair.b;

		let movedA = (b2IsNodeMoved(arrayA[pairA]) ? 0x3 : 0) | (b2IsNodeMoved(arrayA[pairA + 1]) ? 0xc : 0);
		let movedB = (b2IsNodeMoved(arrayB[pairB]) ? 0x5 : 0) | (b2IsNodeMoved(arrayB[pairB + 1]) ? 0xa : 0);

		let mask = b2OverlapPairMask(arrayA, pairA, arrayB, pairB) & (movedA | movedB);
		while (mask != 0) {
			let lane = <i32>b2CTZ32(<u32>mask);
			mask &= mask - 1;
			stackCount = b2ExpandPair<F>(arrayA, arrayB, arrayA[pairA + (lane >> 1)], arrayB[pairB + (lane & 1)], stack, stackCount, context);
		}
	}
}

// This takes moved internal nodes and collides their sub-trees against each other.
// C's world context gives the broad-phase and the worker's pair context; here the caller passes them.
export function b2SelfPairsTask<F>(startIndex: i32, endIndex: i32, bp: b2BroadPhase, pairContext: b2PairContext<F>): void {
	let tree = bp.trees[b2_dynamicBody];
	let nodes = tree.nodes;
	let siblingIndices = bp.movedSiblings;

	// C starts each task with a fresh context
	pairContext.batchCount = 0;

	for (let i = startIndex; i < endIndex; ++i) {
		let nodeIndex = siblingIndices[i];
		b2CollideCrossPairs<F>(nodes, nodes, nodes[nodeIndex], nodes[nodeIndex + 1], pairContext);
	}

	b2FlushCandidatePairs<F>(pairContext);
}

export const B2_CROSS_SEED_COUNT: i32 = 64;

// C's stack buffer for the search queue
const b2_crossQueue = new StaticArray<b2NodePair>(2 * B2_CROSS_SEED_COUNT);

// This does a serial cross-tree breadth first search until the queue is full. Then it returns
// the queue pairs as seeds for a parallel search. C's seeds pointer is seeds plus seedBase.
export function b2GatherCrossSeeds(treeA: b2DynamicTree, treeB: b2DynamicTree, seeds: StaticArray<b2NodePair>, seedBase: i32): i32 {
	let nodesA = treeA.nodes;
	let nodesB = treeB.nodes;

	// Breadth-first search from the two roots as one pair. An empty tree has a sentinel root that
	// survives nothing.
	let queue = b2_crossQueue;
	let mask = 2 * B2_CROSS_SEED_COUNT - 1;
	let head = 0;
	let tail = 0;

	let rootA = nodesA[B2_ROOT_NODE];
	let rootB = nodesB[B2_ROOT_NODE];
	if (b2TestPair(rootA, rootB)) {
		queue[tail & mask] = b2NodePair(rootA, rootB);
		tail += 1;
	}

	let seedCount = 0;
	while (head < tail && seedCount + (tail - head) + 3 < B2_CROSS_SEED_COUNT) {
		let pair = queue[head & mask];
		head += 1;

		if (b2IsLeaf(pair.a) || b2IsLeaf(pair.b)) {
			seeds[seedBase + seedCount++] = pair;
			continue;
		}

		let a = b2GetLeftChild(pair.a);
		let b = b2GetLeftChild(pair.b);

		// Nodes have two children each, so four combinations.
		for (let i = 0; i < 2; ++i) {
			for (let j = 0; j < 2; ++j) {
				if (b2TestPair(nodesA[a + i], nodesB[b + j])) {
					queue[tail & mask] = b2NodePair(nodesA[a + i], nodesB[b + j]);
					tail += 1;
				}
			}
		}
	}

	while (head < tail) {
		seeds[seedBase + seedCount] = queue[head & mask];
		seedCount += 1;
		head += 1;
	}

	return seedCount;
}

// C's b2CrossContext holds the world, the seeds and the static seed count; here the caller passes them.
export function b2CrossPairsTask<F>(
	startIndex: i32,
	endIndex: i32,
	bp: b2BroadPhase,
	seeds: StaticArray<b2NodePair>,
	staticSeedCount: i32,
	pairContext: b2PairContext<F>,
): void {
	let staticNodes = bp.trees[b2_staticBody].nodes;
	let kinematicNodes = bp.trees[b2_kinematicBody].nodes;
	let dynamicNodes = bp.trees[b2_dynamicBody].nodes;

	pairContext.batchCount = 0;

	for (let i = startIndex; i < endIndex; ++i) {
		let nodesB = i < staticSeedCount ? staticNodes : kinematicNodes;
		let seed = seeds[i];
		b2CollideCrossPairs<F>(dynamicNodes, nodesB, seed.a, seed.b, pairContext);
	}

	b2FlushCandidatePairs<F>(pairContext);
}

// C's digit counts on the stack
const b2_digitCounts = new StaticArray<i32>(8 * 256);

// LSD radix sort. Faster than qsort when count is large and fast enough otherwise.
// https://en.wikipedia.org/wiki/Radix_sort
export function b2RadixSortKeys(keys: StaticArray<u64>, tempKeys: StaticArray<u64>, count: i32): void {
	let digitCounts = b2_digitCounts;
	for (let i = 0; i < 8 * 256; ++i) {
		digitCounts[i] = 0;
	}

	for (let i = 0; i < count; ++i) {
		let key = keys[i];
		for (let digit = 0; digit < 8; ++digit) {
			digitCounts[digit * 256 + <i32>((key >> <u64>(8 * digit)) & 0xff)] += 1;
		}
	}

	let source = keys;
	let target = tempKeys;

	for (let digit = 0; digit < 8; ++digit) {
		let shift = <u64>(8 * digit);
		let offsets = digit * 256;

		if (digitCounts[offsets + <i32>((source[0] >> shift) & 0xff)] == count) {
			// All keys have the same value for this digit. Common for shape pair keys.
			continue;
		}

		let sum = 0;
		for (let i = 0; i < 256; ++i) {
			let digitCount = digitCounts[offsets + i];
			digitCounts[offsets + i] = sum;
			sum += digitCount;
		}

		for (let i = 0; i < count; ++i) {
			let key = source[i];
			let slot = offsets + <i32>((key >> shift) & 0xff);
			target[digitCounts[slot]] = key;
			digitCounts[slot] += 1;
		}

		// Swap for the next pass
		let swap = source;
		source = target;
		target = swap;
	}

	if (source != keys) {
		for (let i = 0; i < count; ++i) {
			keys[i] = source[i];
		}
	}
}

export function b2ValidateBroadphase(bp: b2BroadPhase): void {
	b2DynamicTree_Validate(bp.trees[b2_dynamicBody]);
	b2DynamicTree_Validate(bp.trees[b2_kinematicBody]);

	// TODO_ERIN validate every shape AABB is contained in tree AABB
}

export function b2ValidateNoEnlarged(bp: b2BroadPhase): void {
	if (B2_ENABLE_VALIDATION) {
		for (let j = 0; j < b2_bodyTypeCount; ++j) {
			let tree = bp.trees[j];
			b2DynamicTree_ValidateNoMoved(tree);
		}
	}
}
