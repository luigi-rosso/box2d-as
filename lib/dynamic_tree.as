// Ported from box2d src/dynamic_tree.c (median-split rebuild heuristic, the
// compiled default). The C node union (children vs userData, parent vs next)
// becomes separate fields; every read site only ever sees the member that was
// last written for that node role, so behavior is identical. Callbacks are
// abstract classes because the wasm target has no C function pointers.

import {
    b2Vec2, b2AABB,
    b2Add, b2Sub, b2Dot, b2Abs, b2Min, b2Max, b2MulAdd, b2MulSV, b2CrossSV, b2Normalize,
    b2AABB_Union, b2AABB_Center, b2AABB_Extents, b2AABB_Overlaps, b2AABB_Contains,
    b2LengthSquared, b2DistanceSquared, b2MinFloat, b2AbsFloat, FLT_MAX,
} from "./math_functions";
import { b2Perimeter, b2IsValidAABB, b2EnlargeAABB } from "./aabb";
import { b2RayCastInput, b2ShapeCastInput, b2TreeStats } from "./collision";
import { B2_NULL_INDEX } from "./core";
import { B2_HUGE, b2_allocatedNode, b2_enlargedNode, b2_leafNode } from "./constants";

export const B2_TREE_STACK_SIZE: i32 = 1024;
const B2_DEFAULT_CATEGORY_BITS: u64 = 1;

// A node in the dynamic tree.
export class b2TreeNode {
    aabb: b2AABB = new b2AABB();
    categoryBits: u64 = 0;
    // C union: children (internal node) or userData (leaf node)
    child1: i32 = 0;
    child2: i32 = 0;
    userData: u64 = 0;
    // C union: parent (allocated node) or freelist next (free node)
    parent: i32 = 0;
    height: u16 = 0;
    flags: u16 = 0;

    // Matches C's zero memset of fresh pool storage.
    zero(): void {
        this.aabb.lowerBound.x = 0;
        this.aabb.lowerBound.y = 0;
        this.aabb.upperBound.x = 0;
        this.aabb.upperBound.y = 0;
        this.categoryBits = 0;
        this.child1 = 0;
        this.child2 = 0;
        this.userData = 0;
        this.parent = 0;
        this.height = 0;
        this.flags = 0;
    }

    // Matches C's *node = b2_defaultTreeNode.
    setDefault(): void {
        this.aabb.lowerBound.x = 0;
        this.aabb.lowerBound.y = 0;
        this.aabb.upperBound.x = 0;
        this.aabb.upperBound.y = 0;
        this.categoryBits = B2_DEFAULT_CATEGORY_BITS;
        this.child1 = B2_NULL_INDEX;
        this.child2 = B2_NULL_INDEX;
        this.userData = 0;
        this.parent = B2_NULL_INDEX;
        this.height = 0;
        this.flags = <u16>b2_allocatedNode;
    }
}

function b2IsLeaf(node: b2TreeNode): bool {
    return (node.flags & <u16>b2_leafNode) != 0;
}

function b2IsAllocated(node: b2TreeNode): bool {
    return (node.flags & <u16>b2_allocatedNode) != 0;
}

function b2MaxUInt16(a: u16, b: u16): u16 {
    return a > b ? a : b;
}

export class b2DynamicTree {
    nodes: Array<b2TreeNode> = new Array<b2TreeNode>(0);
    root: i32 = B2_NULL_INDEX;
    nodeCount: i32 = 0;
    nodeCapacity: i32 = 0;
    freeList: i32 = B2_NULL_INDEX;
    proxyCount: i32 = 0;
    leafIndices: Int32Array = new Int32Array(0);
    leafCenters: Array<b2Vec2> = new Array<b2Vec2>(0);
    rebuildCapacity: i32 = 0;
}

// Callback bases carry default bodies because rasc has no abstract methods;
// subclasses override.

// Query callback: return false to terminate the query.
export abstract class b2TreeQueryCallback {
    queryCallback(proxyId: i32, userData: u64): bool {
        return true;
    }
}

// Ray cast callback: returns the new max fraction, 0 to terminate.
export abstract class b2TreeRayCastCallback {
    rayCastCallback(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
        return input.maxFraction;
    }
}

// Shape cast callback: returns the new max fraction, 0 to terminate.
export abstract class b2TreeShapeCastCallback {
    shapeCastCallback(input: b2ShapeCastInput, proxyId: i32, userData: u64): f32 {
        return input.maxFraction;
    }
}

export function b2DynamicTree_Create(): b2DynamicTree {
    let tree = new b2DynamicTree();
    tree.root = B2_NULL_INDEX;

    tree.nodeCapacity = 16;
    tree.nodeCount = 0;
    tree.nodes = new Array<b2TreeNode>(tree.nodeCapacity);
    for (let i = 0; i < tree.nodeCapacity; ++i) {
        tree.nodes[i] = new b2TreeNode();
    }

    // Build a linked list for the free list.
    for (let i = 0; i < tree.nodeCapacity - 1; ++i) {
        tree.nodes[i].parent = i + 1;
    }

    tree.nodes[tree.nodeCapacity - 1].parent = B2_NULL_INDEX;
    tree.freeList = 0;

    tree.proxyCount = 0;
    tree.rebuildCapacity = 0;

    return tree;
}

export function b2DynamicTree_Destroy(tree: b2DynamicTree): void {
    tree.nodes = new Array<b2TreeNode>(0);
    tree.root = B2_NULL_INDEX;
    tree.nodeCount = 0;
    tree.nodeCapacity = 0;
    tree.freeList = 0;
    tree.proxyCount = 0;
    tree.leafIndices = new Int32Array(0);
    tree.leafCenters = new Array<b2Vec2>(0);
    tree.rebuildCapacity = 0;
}

// Allocate a node from the pool. Grow the pool if necessary.
function b2AllocateNode(tree: b2DynamicTree): i32 {
    // Expand the node pool as needed.
    if (tree.freeList == B2_NULL_INDEX) {
        assert(tree.nodeCount == tree.nodeCapacity);

        // The free list is empty. Rebuild a bigger pool.
        let oldCapacity = tree.nodeCapacity;
        tree.nodeCapacity += oldCapacity >> 1;
        let newNodes = new Array<b2TreeNode>(tree.nodeCapacity);
        for (let i = 0; i < tree.nodeCount; ++i) {
            newNodes[i] = tree.nodes[i];
        }
        for (let i = tree.nodeCount; i < tree.nodeCapacity; ++i) {
            newNodes[i] = new b2TreeNode();
        }
        tree.nodes = newNodes;

        // Build a linked list for the free list. The parent pointer becomes the "next" pointer.
        for (let i = tree.nodeCount; i < tree.nodeCapacity - 1; ++i) {
            tree.nodes[i].parent = i + 1;
        }

        tree.nodes[tree.nodeCapacity - 1].parent = B2_NULL_INDEX;
        tree.freeList = tree.nodeCount;
    }

    // Peel a node off the free list.
    let nodeIndex = tree.freeList;
    let node = tree.nodes[nodeIndex];
    tree.freeList = node.parent;
    node.setDefault();
    ++tree.nodeCount;
    return nodeIndex;
}

// Return a node to the pool.
function b2FreeNode(tree: b2DynamicTree, nodeId: i32): void {
    assert(0 <= nodeId && nodeId < tree.nodeCapacity);
    assert(0 < tree.nodeCount);
    tree.nodes[nodeId].parent = tree.freeList;
    tree.nodes[nodeId].flags = 0;
    tree.freeList = nodeId;
    --tree.nodeCount;
}

// Greedy algorithm for sibling selection using the SAH. See the C source for
// the cost derivation.
function b2FindBestSibling(tree: b2DynamicTree, boxD: b2AABB): i32 {
    let centerD = b2AABB_Center(boxD);
    let areaD: f32 = b2Perimeter(boxD);

    let nodes = tree.nodes;
    let rootIndex = tree.root;

    let rootBox = unchecked(nodes[rootIndex]).aabb;

    // Area of current node
    let areaBase: f32 = b2Perimeter(rootBox);

    // Area of inflated node
    let directCost: f32 = b2Perimeter(b2AABB_Union(rootBox, boxD));
    let inheritedCost: f32 = 0.0;

    let bestSibling = rootIndex;
    let bestCost: f32 = directCost;

    // Descend the tree from root, following a single greedy path.
    let index = rootIndex;
    while (unchecked(nodes[index]).height > 0) {
        let child1 = unchecked(nodes[index]).child1;
        let child2 = unchecked(nodes[index]).child2;

        // Cost of creating a new parent for this node and the new leaf
        let cost: f32 = directCost + inheritedCost;

        // Sometimes there are multiple identical costs within tolerance.
        // This breaks the ties using the centroid distance.
        if (cost < bestCost) {
            bestSibling = index;
            bestCost = cost;
        }

        // Inheritance cost seen by children
        inheritedCost += directCost - areaBase;

        let leaf1 = unchecked(nodes[child1]).height == 0;
        let leaf2 = unchecked(nodes[child2]).height == 0;

        // Cost of descending into child 1
        let lowerCost1: f32 = FLT_MAX;
        let box1 = unchecked(nodes[child1]).aabb;
        let directCost1: f32 = b2Perimeter(b2AABB_Union(box1, boxD));
        let area1: f32 = 0.0;
        if (leaf1) {
            // Child 1 is a leaf
            // Cost of creating new node and increasing area of node P
            let cost1: f32 = directCost1 + inheritedCost;

            // Need this here due to while condition above
            if (cost1 < bestCost) {
                bestSibling = child1;
                bestCost = cost1;
            }
        } else {
            // Child 1 is an internal node
            area1 = b2Perimeter(box1);
            lowerCost1 = inheritedCost + directCost1 + b2MinFloat(areaD - area1, 0.0);
        }

        // Cost of descending into child 2
        let lowerCost2: f32 = FLT_MAX;
        let box2 = unchecked(nodes[child2]).aabb;
        let directCost2: f32 = b2Perimeter(b2AABB_Union(box2, boxD));
        let area2: f32 = 0.0;
        if (leaf2) {
            let cost2: f32 = directCost2 + inheritedCost;

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
            assert(lowerCost1 < FLT_MAX);
            assert(lowerCost2 < FLT_MAX);

            // No clear choice based on lower bound surface area. This can happen when both
            // children fully contain D. Fall back to node distance.
            let d1 = b2Sub(b2AABB_Center(box1), centerD);
            let d2 = b2Sub(b2AABB_Center(box2), centerD);
            lowerCost1 = b2LengthSquared(d1);
            lowerCost2 = b2LengthSquared(d2);
        }

        // Descend
        if (lowerCost1 < lowerCost2 && leaf1 == false) {
            index = child1;
            areaBase = area1;
            directCost = directCost1;
        } else {
            index = child2;
            areaBase = area2;
            directCost = directCost2;
        }

        assert(unchecked(nodes[index]).height > 0);
    }

    return bestSibling;
}

// enum b2RotateType
const b2_rotateNone: i32 = 0;
const b2_rotateBF: i32 = 1;
const b2_rotateBG: i32 = 2;
const b2_rotateCD: i32 = 3;
const b2_rotateCE: i32 = 4;

// Perform a left or right rotation if node A is imbalanced.
function b2RotateNodes(tree: b2DynamicTree, iA: i32): void {
    assert(iA != B2_NULL_INDEX);

    let nodes = tree.nodes;

    let A = unchecked(nodes[iA]);
    if (A.height < 2) {
        return;
    }

    let iB = A.child1;
    let iC = A.child2;
    assert(0 <= iB && iB < tree.nodeCapacity);
    assert(0 <= iC && iC < tree.nodeCapacity);

    let B = unchecked(nodes[iB]);
    let C = unchecked(nodes[iC]);

    if (B.height == 0) {
        // B is a leaf and C is internal
        assert(C.height > 0);

        let iF = C.child1;
        let iG = C.child2;
        let F = unchecked(nodes[iF]);
        let G = unchecked(nodes[iG]);
        assert(0 <= iF && iF < tree.nodeCapacity);
        assert(0 <= iG && iG < tree.nodeCapacity);

        // Base cost
        let costBase: f32 = b2Perimeter(C.aabb);

        // Cost of swapping B and F
        let aabbBG = b2AABB_Union(B.aabb, G.aabb);
        let costBF: f32 = b2Perimeter(aabbBG);

        // Cost of swapping B and G
        let aabbBF = b2AABB_Union(B.aabb, F.aabb);
        let costBG: f32 = b2Perimeter(aabbBF);

        if (costBase < costBF && costBase < costBG) {
            // Rotation does not improve cost
            return;
        }

        if (costBF < costBG) {
            // Swap B and F
            A.child1 = iF;
            C.child1 = iB;

            B.parent = iC;
            F.parent = iA;

            C.aabb = aabbBG;

            C.height = 1 + b2MaxUInt16(B.height, G.height);
            A.height = 1 + b2MaxUInt16(C.height, F.height);
            C.categoryBits = B.categoryBits | G.categoryBits;
            A.categoryBits = C.categoryBits | F.categoryBits;
            C.flags |= (B.flags | G.flags) & <u16>b2_enlargedNode;
            A.flags |= (C.flags | F.flags) & <u16>b2_enlargedNode;
        } else {
            // Swap B and G
            A.child1 = iG;
            C.child2 = iB;

            B.parent = iC;
            G.parent = iA;

            C.aabb = aabbBF;

            C.height = 1 + b2MaxUInt16(B.height, F.height);
            A.height = 1 + b2MaxUInt16(C.height, G.height);
            C.categoryBits = B.categoryBits | F.categoryBits;
            A.categoryBits = C.categoryBits | G.categoryBits;
            C.flags |= (B.flags | F.flags) & <u16>b2_enlargedNode;
            A.flags |= (C.flags | G.flags) & <u16>b2_enlargedNode;
        }
    } else if (C.height == 0) {
        // C is a leaf and B is internal
        assert(B.height > 0);

        let iD = B.child1;
        let iE = B.child2;
        let D = unchecked(nodes[iD]);
        let E = unchecked(nodes[iE]);
        assert(0 <= iD && iD < tree.nodeCapacity);
        assert(0 <= iE && iE < tree.nodeCapacity);

        // Base cost
        let costBase: f32 = b2Perimeter(B.aabb);

        // Cost of swapping C and D
        let aabbCE = b2AABB_Union(C.aabb, E.aabb);
        let costCD: f32 = b2Perimeter(aabbCE);

        // Cost of swapping C and E
        let aabbCD = b2AABB_Union(C.aabb, D.aabb);
        let costCE: f32 = b2Perimeter(aabbCD);

        if (costBase < costCD && costBase < costCE) {
            // Rotation does not improve cost
            return;
        }

        if (costCD < costCE) {
            // Swap C and D
            A.child2 = iD;
            B.child1 = iC;

            C.parent = iB;
            D.parent = iA;

            B.aabb = aabbCE;

            B.height = 1 + b2MaxUInt16(C.height, E.height);
            A.height = 1 + b2MaxUInt16(B.height, D.height);
            B.categoryBits = C.categoryBits | E.categoryBits;
            A.categoryBits = B.categoryBits | D.categoryBits;
            B.flags |= (C.flags | E.flags) & <u16>b2_enlargedNode;
            A.flags |= (B.flags | D.flags) & <u16>b2_enlargedNode;
        } else {
            // Swap C and E
            A.child2 = iE;
            B.child2 = iC;

            C.parent = iB;
            E.parent = iA;

            B.aabb = aabbCD;
            B.height = 1 + b2MaxUInt16(C.height, D.height);
            A.height = 1 + b2MaxUInt16(B.height, E.height);
            B.categoryBits = C.categoryBits | D.categoryBits;
            A.categoryBits = B.categoryBits | E.categoryBits;
            B.flags |= (C.flags | D.flags) & <u16>b2_enlargedNode;
            A.flags |= (B.flags | E.flags) & <u16>b2_enlargedNode;
        }
    } else {
        let iD = B.child1;
        let iE = B.child2;
        let iF = C.child1;
        let iG = C.child2;

        assert(0 <= iD && iD < tree.nodeCapacity);
        assert(0 <= iE && iE < tree.nodeCapacity);
        assert(0 <= iF && iF < tree.nodeCapacity);
        assert(0 <= iG && iG < tree.nodeCapacity);

        let D = unchecked(nodes[iD]);
        let E = unchecked(nodes[iE]);
        let F = unchecked(nodes[iF]);
        let G = unchecked(nodes[iG]);

        // Base cost
        let areaB: f32 = b2Perimeter(B.aabb);
        let areaC: f32 = b2Perimeter(C.aabb);
        let costBase: f32 = areaB + areaC;
        let bestRotation = b2_rotateNone;
        let bestCost: f32 = costBase;

        // Cost of swapping B and F
        let aabbBG = b2AABB_Union(B.aabb, G.aabb);
        let costBF: f32 = areaB + b2Perimeter(aabbBG);
        if (costBF < bestCost) {
            bestRotation = b2_rotateBF;
            bestCost = costBF;
        }

        // Cost of swapping B and G
        let aabbBF = b2AABB_Union(B.aabb, F.aabb);
        let costBG: f32 = areaB + b2Perimeter(aabbBF);
        if (costBG < bestCost) {
            bestRotation = b2_rotateBG;
            bestCost = costBG;
        }

        // Cost of swapping C and D
        let aabbCE = b2AABB_Union(C.aabb, E.aabb);
        let costCD: f32 = areaC + b2Perimeter(aabbCE);
        if (costCD < bestCost) {
            bestRotation = b2_rotateCD;
            bestCost = costCD;
        }

        // Cost of swapping C and E
        let aabbCD = b2AABB_Union(C.aabb, D.aabb);
        let costCE: f32 = areaC + b2Perimeter(aabbCD);
        if (costCE < bestCost) {
            bestRotation = b2_rotateCE;
        }

        switch (bestRotation) {
            case b2_rotateNone:
                break;

            case b2_rotateBF:
                A.child1 = iF;
                C.child1 = iB;

                B.parent = iC;
                F.parent = iA;

                C.aabb = aabbBG;
                C.height = 1 + b2MaxUInt16(B.height, G.height);
                A.height = 1 + b2MaxUInt16(C.height, F.height);
                C.categoryBits = B.categoryBits | G.categoryBits;
                A.categoryBits = C.categoryBits | F.categoryBits;
                C.flags |= (B.flags | G.flags) & <u16>b2_enlargedNode;
                A.flags |= (C.flags | F.flags) & <u16>b2_enlargedNode;
                break;

            case b2_rotateBG:
                A.child1 = iG;
                C.child2 = iB;

                B.parent = iC;
                G.parent = iA;

                C.aabb = aabbBF;
                C.height = 1 + b2MaxUInt16(B.height, F.height);
                A.height = 1 + b2MaxUInt16(C.height, G.height);
                C.categoryBits = B.categoryBits | F.categoryBits;
                A.categoryBits = C.categoryBits | G.categoryBits;
                C.flags |= (B.flags | F.flags) & <u16>b2_enlargedNode;
                A.flags |= (C.flags | G.flags) & <u16>b2_enlargedNode;
                break;

            case b2_rotateCD:
                A.child2 = iD;
                B.child1 = iC;

                C.parent = iB;
                D.parent = iA;

                B.aabb = aabbCE;
                B.height = 1 + b2MaxUInt16(C.height, E.height);
                A.height = 1 + b2MaxUInt16(B.height, D.height);
                B.categoryBits = C.categoryBits | E.categoryBits;
                A.categoryBits = B.categoryBits | D.categoryBits;
                B.flags |= (C.flags | E.flags) & <u16>b2_enlargedNode;
                A.flags |= (B.flags | D.flags) & <u16>b2_enlargedNode;
                break;

            case b2_rotateCE:
                A.child2 = iE;
                B.child2 = iC;

                C.parent = iB;
                E.parent = iA;

                B.aabb = aabbCD;
                B.height = 1 + b2MaxUInt16(C.height, D.height);
                A.height = 1 + b2MaxUInt16(B.height, E.height);
                B.categoryBits = C.categoryBits | D.categoryBits;
                A.categoryBits = B.categoryBits | E.categoryBits;
                B.flags |= (C.flags | D.flags) & <u16>b2_enlargedNode;
                A.flags |= (B.flags | E.flags) & <u16>b2_enlargedNode;
                break;

            default:
                assert(false);
                break;
        }
    }
}

function b2InsertLeaf(tree: b2DynamicTree, leaf: i32, shouldRotate: bool): void {
    if (tree.root == B2_NULL_INDEX) {
        tree.root = leaf;
        tree.nodes[tree.root].parent = B2_NULL_INDEX;
        return;
    }

    // Stage 1: find the best sibling for this node
    let leafAABB = tree.nodes[leaf].aabb;
    let sibling = b2FindBestSibling(tree, leafAABB);

    // Stage 2: create a new parent for the leaf and sibling
    let oldParent = tree.nodes[sibling].parent;
    let newParent = b2AllocateNode(tree);

    // Warning: node array can change after allocation
    let nodes = tree.nodes;
    let parentNode = unchecked(nodes[newParent]);
    parentNode.parent = oldParent;
    parentNode.userData = 0xffffffffffffffff;
    parentNode.aabb = b2AABB_Union(leafAABB, unchecked(nodes[sibling]).aabb);
    parentNode.categoryBits = unchecked(nodes[leaf]).categoryBits | unchecked(nodes[sibling]).categoryBits;
    parentNode.height = unchecked(nodes[sibling]).height + 1;
    parentNode.child1 = sibling;
    parentNode.child2 = leaf;
    unchecked(nodes[sibling]).parent = newParent;
    unchecked(nodes[leaf]).parent = newParent;

    // Fix grandparent links
    if (oldParent != B2_NULL_INDEX) {
        // The sibling was not the root
        if (unchecked(nodes[oldParent]).child1 == sibling) {
            unchecked(nodes[oldParent]).child1 = newParent;
        } else {
            assert(unchecked(nodes[oldParent]).child2 == sibling);
            unchecked(nodes[oldParent]).child2 = newParent;
        }
    } else {
        // The sibling was the root
        tree.root = newParent;
    }

    // Stage 3: walk back up the tree fixing heights and AABBs
    let index = unchecked(nodes[leaf]).parent;
    while (index != B2_NULL_INDEX) {
        let node = unchecked(nodes[index]);
        let child1 = node.child1;
        let child2 = node.child2;

        assert(child1 != B2_NULL_INDEX);
        assert(child2 != B2_NULL_INDEX);

        let c1 = unchecked(nodes[child1]);
        let c2 = unchecked(nodes[child2]);

        node.aabb = b2AABB_Union(c1.aabb, c2.aabb);
        node.categoryBits = c1.categoryBits | c2.categoryBits;
        node.height = 1 + b2MaxUInt16(c1.height, c2.height);
        node.flags |= (c1.flags | c2.flags) & <u16>b2_enlargedNode;

        if (shouldRotate) {
            b2RotateNodes(tree, index);
        }

        index = node.parent;
    }
}

function b2RemoveLeaf(tree: b2DynamicTree, leaf: i32): void {
    if (leaf == tree.root) {
        tree.root = B2_NULL_INDEX;
        return;
    }

    let nodes = tree.nodes;

    let parent = unchecked(nodes[leaf]).parent;
    let grandParent = unchecked(nodes[parent]).parent;
    let sibling: i32;
    if (unchecked(nodes[parent]).child1 == leaf) {
        sibling = unchecked(nodes[parent]).child2;
    } else {
        sibling = unchecked(nodes[parent]).child1;
    }

    if (grandParent != B2_NULL_INDEX) {
        // Destroy parent and connect sibling to grandParent.
        if (unchecked(nodes[grandParent]).child1 == parent) {
            unchecked(nodes[grandParent]).child1 = sibling;
        } else {
            unchecked(nodes[grandParent]).child2 = sibling;
        }
        unchecked(nodes[sibling]).parent = grandParent;
        b2FreeNode(tree, parent);

        // Adjust ancestor bounds.
        let index = grandParent;
        while (index != B2_NULL_INDEX) {
            let node = unchecked(nodes[index]);
            let child1 = unchecked(nodes[node.child1]);
            let child2 = unchecked(nodes[node.child2]);

            node.aabb = b2AABB_Union(child1.aabb, child2.aabb);
            node.categoryBits = child1.categoryBits | child2.categoryBits;
            node.height = 1 + b2MaxUInt16(child1.height, child2.height);

            index = node.parent;
        }
    } else {
        tree.root = sibling;
        tree.nodes[sibling].parent = B2_NULL_INDEX;
        b2FreeNode(tree, parent);
    }
}

// Create a proxy in the tree as a leaf node.
export function b2DynamicTree_CreateProxy(tree: b2DynamicTree, aabb: b2AABB, categoryBits: u64, userData: u64): i32 {
    assert(-B2_HUGE() < aabb.lowerBound.x && aabb.lowerBound.x < B2_HUGE());
    assert(-B2_HUGE() < aabb.lowerBound.y && aabb.lowerBound.y < B2_HUGE());
    assert(-B2_HUGE() < aabb.upperBound.x && aabb.upperBound.x < B2_HUGE());
    assert(-B2_HUGE() < aabb.upperBound.y && aabb.upperBound.y < B2_HUGE());

    let proxyId = b2AllocateNode(tree);
    let node = tree.nodes[proxyId];

    node.aabb = aabb.clone();
    node.userData = userData;
    node.categoryBits = categoryBits;
    node.height = 0;
    node.flags = <u16>(b2_allocatedNode | b2_leafNode);

    let shouldRotate = true;
    b2InsertLeaf(tree, proxyId, shouldRotate);

    tree.proxyCount += 1;

    return proxyId;
}

export function b2DynamicTree_DestroyProxy(tree: b2DynamicTree, proxyId: i32): void {
    assert(0 <= proxyId && proxyId < tree.nodeCapacity);
    assert(b2IsLeaf(tree.nodes[proxyId]));

    b2RemoveLeaf(tree, proxyId);
    b2FreeNode(tree, proxyId);

    assert(tree.proxyCount > 0);
    tree.proxyCount -= 1;
}

export function b2DynamicTree_GetProxyCount(tree: b2DynamicTree): i32 {
    return tree.proxyCount;
}

export function b2DynamicTree_MoveProxy(tree: b2DynamicTree, proxyId: i32, aabb: b2AABB): void {
    assert(b2IsValidAABB(aabb));
    assert(aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE());
    assert(aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE());
    assert(0 <= proxyId && proxyId < tree.nodeCapacity);
    assert(b2IsLeaf(tree.nodes[proxyId]));

    b2RemoveLeaf(tree, proxyId);

    tree.nodes[proxyId].aabb = aabb.clone();

    let shouldRotate = false;
    b2InsertLeaf(tree, proxyId, shouldRotate);
}

export function b2DynamicTree_EnlargeProxy(tree: b2DynamicTree, proxyId: i32, aabb: b2AABB): void {
    let nodes = tree.nodes;

    assert(b2IsValidAABB(aabb));
    assert(aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE());
    assert(aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE());
    assert(0 <= proxyId && proxyId < tree.nodeCapacity);
    assert(b2IsLeaf(tree.nodes[proxyId]));

    // Caller must ensure this
    assert(b2AABB_Contains(nodes[proxyId].aabb, aabb) == false);

    unchecked(nodes[proxyId]).aabb = aabb.clone();

    let parentIndex = unchecked(nodes[proxyId]).parent;
    while (parentIndex != B2_NULL_INDEX) {
        let changed = b2EnlargeAABB(unchecked(nodes[parentIndex]).aabb, aabb);
        unchecked(nodes[parentIndex]).flags |= <u16>b2_enlargedNode;
        parentIndex = unchecked(nodes[parentIndex]).parent;

        if (changed == false) {
            break;
        }
    }

    while (parentIndex != B2_NULL_INDEX) {
        if ((unchecked(nodes[parentIndex]).flags & <u16>b2_enlargedNode) != 0) {
            // early out because this ancestor was previously ascended and marked as enlarged
            break;
        }

        unchecked(nodes[parentIndex]).flags |= <u16>b2_enlargedNode;
        parentIndex = unchecked(nodes[parentIndex]).parent;
    }
}

export function b2DynamicTree_SetCategoryBits(tree: b2DynamicTree, proxyId: i32, categoryBits: u64): void {
    let nodes = tree.nodes;

    assert((nodes[proxyId].flags & <u16>b2_leafNode) == <u16>b2_leafNode);

    nodes[proxyId].categoryBits = categoryBits;

    // Fix up category bits in ancestor internal nodes
    let nodeIndex = nodes[proxyId].parent;
    while (nodeIndex != B2_NULL_INDEX) {
        let node = unchecked(nodes[nodeIndex]);
        let child1 = node.child1;
        assert(child1 != B2_NULL_INDEX);
        let child2 = node.child2;
        assert(child2 != B2_NULL_INDEX);
        node.categoryBits = unchecked(nodes[child1]).categoryBits | unchecked(nodes[child2]).categoryBits;

        nodeIndex = node.parent;
    }
}

export function b2DynamicTree_GetCategoryBits(tree: b2DynamicTree, proxyId: i32): u64 {
    assert(0 <= proxyId && proxyId < tree.nodeCapacity);
    return tree.nodes[proxyId].categoryBits;
}

export function b2DynamicTree_GetHeight(tree: b2DynamicTree): i32 {
    if (tree.root == B2_NULL_INDEX) {
        return 0;
    }

    return <i32>tree.nodes[tree.root].height;
}

export function b2DynamicTree_GetAreaRatio(tree: b2DynamicTree): f32 {
    if (tree.root == B2_NULL_INDEX) {
        return 0.0;
    }

    let root = tree.nodes[tree.root];
    let rootArea: f32 = b2Perimeter(root.aabb);

    let totalArea: f32 = 0.0;
    for (let i = 0; i < tree.nodeCapacity; ++i) {
        let node = tree.nodes[i];
        if (b2IsAllocated(node) == false || b2IsLeaf(node) || i == tree.root) {
            continue;
        }

        totalArea += b2Perimeter(node.aabb);
    }

    return totalArea / rootArea;
}

export function b2DynamicTree_GetRootBounds(tree: b2DynamicTree): b2AABB {
    if (tree.root != B2_NULL_INDEX) {
        return tree.nodes[tree.root].aabb.clone();
    }

    return new b2AABB();
}

export function b2DynamicTree_GetUserData(tree: b2DynamicTree, proxyId: i32): u64 {
    assert(0 <= proxyId && proxyId < tree.nodeCapacity);
    return tree.nodes[proxyId].userData;
}

export function b2DynamicTree_GetAABB(tree: b2DynamicTree, proxyId: i32): b2AABB {
    assert(0 <= proxyId && proxyId < tree.nodeCapacity);
    return tree.nodes[proxyId].aabb.clone();
}

let s_treeStack = new StaticArray<i32>(B2_TREE_STACK_SIZE);

export function b2DynamicTree_Query(tree: b2DynamicTree, aabb: b2AABB, maskBits: u64,
    callback: b2TreeQueryCallback): b2TreeStats {
    let result = new b2TreeStats();

    if (tree.nodeCount == 0) {
        return result;
    }

    let stack = s_treeStack;
    let stackCount = 0;
    unchecked(stack[stackCount++] = tree.root);

    while (stackCount > 0) {
        let nodeId = unchecked(stack[--stackCount]);

        let node = unchecked(tree.nodes[nodeId]);
        result.nodeVisits += 1;

        if (b2AABB_Overlaps(node.aabb, aabb) && (node.categoryBits & maskBits) != 0) {
            if (b2IsLeaf(node)) {
                // callback to user code with proxy id
                let proceed = callback.queryCallback(nodeId, node.userData);
                result.leafVisits += 1;

                if (proceed == false) {
                    return result;
                }
            } else {
                if (stackCount < B2_TREE_STACK_SIZE - 1) {
                    unchecked(stack[stackCount++] = node.child1);
                    unchecked(stack[stackCount++] = node.child2);
                } else {
                    assert(stackCount < B2_TREE_STACK_SIZE - 1);
                }
            }
        }
    }

    return result;
}

export function b2DynamicTree_QueryAll(tree: b2DynamicTree, aabb: b2AABB, callback: b2TreeQueryCallback): b2TreeStats {
    let result = new b2TreeStats();

    if (tree.nodeCount == 0) {
        return result;
    }

    let stack = s_treeStack;
    let stackCount = 0;
    unchecked(stack[stackCount++] = tree.root);

    while (stackCount > 0) {
        let nodeId = unchecked(stack[--stackCount]);

        let node = unchecked(tree.nodes[nodeId]);
        result.nodeVisits += 1;

        if (b2AABB_Overlaps(node.aabb, aabb)) {
            if (b2IsLeaf(node)) {
                // callback to user code with proxy id
                let proceed = callback.queryCallback(nodeId, node.userData);
                result.leafVisits += 1;

                if (proceed == false) {
                    return result;
                }
            } else {
                if (stackCount < B2_TREE_STACK_SIZE - 1) {
                    unchecked(stack[stackCount++] = node.child1);
                    unchecked(stack[stackCount++] = node.child2);
                } else {
                    assert(stackCount < B2_TREE_STACK_SIZE - 1);
                }
            }
        }
    }

    return result;
}

export function b2DynamicTree_RayCast(tree: b2DynamicTree, input: b2RayCastInput, maskBits: u64,
    callback: b2TreeRayCastCallback): b2TreeStats {
    let result = new b2TreeStats();

    if (tree.nodeCount == 0) {
        return result;
    }

    let p1 = input.origin;
    let d = input.translation;

    let r = b2Normalize(d);

    // v is perpendicular to the segment.
    let v = b2CrossSV(1.0, r);
    let abs_v = b2Abs(v);

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    let maxFraction: f32 = input.maxFraction;

    let p2 = b2MulAdd(p1, maxFraction, d);

    // Build a bounding box for the segment.
    let segmentAABB = new b2AABB(b2Min(p1, p2), b2Max(p1, p2));

    let stack = s_treeStack;
    let stackCount = 0;
    unchecked(stack[stackCount++] = tree.root);

    let nodes = tree.nodes;

    let subInput = new b2RayCastInput();
    subInput.origin = input.origin.clone();
    subInput.translation = input.translation.clone();
    subInput.maxFraction = input.maxFraction;

    while (stackCount > 0) {
        let nodeId = unchecked(stack[--stackCount]);
        if (nodeId == B2_NULL_INDEX) {
            assert(false);
            continue;
        }

        let node = unchecked(nodes[nodeId]);
        result.nodeVisits += 1;

        let nodeAABB = node.aabb;

        if ((node.categoryBits & maskBits) == 0 || b2AABB_Overlaps(nodeAABB, segmentAABB) == false) {
            continue;
        }

        // Separating axis for segment (Gino, p80).
        // radius extension is added to the node in this case
        let c = b2AABB_Center(nodeAABB);
        let h = b2AABB_Extents(nodeAABB);
        let term1: f32 = b2AbsFloat(b2Dot(v, b2Sub(p1, c)));
        let term2: f32 = b2Dot(abs_v, h);
        if (term2 < term1) {
            continue;
        }

        if (b2IsLeaf(node)) {
            subInput.maxFraction = maxFraction;

            let value: f32 = callback.rayCastCallback(subInput, nodeId, node.userData);
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
            }
        } else {
            if (stackCount < B2_TREE_STACK_SIZE - 1) {
                let c1 = b2AABB_Center(unchecked(nodes[node.child1]).aabb);
                let c2 = b2AABB_Center(unchecked(nodes[node.child2]).aabb);
                if (b2DistanceSquared(c1, p1) < b2DistanceSquared(c2, p1)) {
                    unchecked(stack[stackCount++] = node.child2);
                    unchecked(stack[stackCount++] = node.child1);
                } else {
                    unchecked(stack[stackCount++] = node.child1);
                    unchecked(stack[stackCount++] = node.child2);
                }
            } else {
                assert(stackCount < B2_TREE_STACK_SIZE - 1);
            }
        }
    }

    return result;
}

export function b2DynamicTree_ShapeCast(tree: b2DynamicTree, input: b2ShapeCastInput, maskBits: u64,
    callback: b2TreeShapeCastCallback): b2TreeStats {
    let stats = new b2TreeStats();

    if (tree.nodeCount == 0 || input.proxy.count == 0) {
        return stats;
    }

    let originAABB = new b2AABB(input.proxy.points[0].clone(), input.proxy.points[0].clone());
    for (let i = 1; i < input.proxy.count; ++i) {
        originAABB.lowerBound = b2Min(originAABB.lowerBound, unchecked(input.proxy.points[i]));
        originAABB.upperBound = b2Max(originAABB.upperBound, unchecked(input.proxy.points[i]));
    }

    let radius = new b2Vec2(input.proxy.radius, input.proxy.radius);

    originAABB.lowerBound = b2Sub(originAABB.lowerBound, radius);
    originAABB.upperBound = b2Add(originAABB.upperBound, radius);

    let p1 = b2AABB_Center(originAABB);
    let extension = b2AABB_Extents(originAABB);

    // v is perpendicular to the segment.
    let r = input.translation;
    let v = b2CrossSV(1.0, r);
    let abs_v = b2Abs(v);

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    let maxFraction: f32 = input.maxFraction;

    // Build total box for the shape cast
    let t = b2MulSV(maxFraction, input.translation);
    let totalAABB = new b2AABB(
        b2Min(originAABB.lowerBound, b2Add(originAABB.lowerBound, t)),
        b2Max(originAABB.upperBound, b2Add(originAABB.upperBound, t)));

    let subInput = new b2ShapeCastInput();
    subInput.proxy = input.proxy;
    subInput.translation = input.translation.clone();
    subInput.maxFraction = input.maxFraction;
    subInput.canEncroach = input.canEncroach;

    let nodes = tree.nodes;

    let stack = s_treeStack;
    let stackCount = 0;
    unchecked(stack[stackCount++] = tree.root);

    while (stackCount > 0) {
        let nodeId = unchecked(stack[--stackCount]);
        if (nodeId == B2_NULL_INDEX) {
            assert(false);
            continue;
        }

        let node = unchecked(nodes[nodeId]);
        stats.nodeVisits += 1;

        if ((node.categoryBits & maskBits) == 0 || b2AABB_Overlaps(node.aabb, totalAABB) == false) {
            continue;
        }

        // Separating axis for segment (Gino, p80).
        // radius extension is added to the node in this case
        let c = b2AABB_Center(node.aabb);
        let h = b2Add(b2AABB_Extents(node.aabb), extension);
        let term1: f32 = b2AbsFloat(b2Dot(v, b2Sub(p1, c)));
        let term2: f32 = b2Dot(abs_v, h);
        if (term2 < term1) {
            continue;
        }

        if (b2IsLeaf(node)) {
            subInput.maxFraction = maxFraction;

            let value: f32 = callback.shapeCastCallback(subInput, nodeId, node.userData);
            stats.leafVisits += 1;

            if (value == 0.0) {
                // The client has terminated the ray cast.
                return stats;
            }

            if (0.0 < value && value < maxFraction) {
                // Update segment bounding box.
                maxFraction = value;
                t = b2MulSV(maxFraction, input.translation);
                totalAABB.lowerBound = b2Min(originAABB.lowerBound, b2Add(originAABB.lowerBound, t));
                totalAABB.upperBound = b2Max(originAABB.upperBound, b2Add(originAABB.upperBound, t));
            }
        } else {
            if (stackCount < B2_TREE_STACK_SIZE - 1) {
                let c1 = b2AABB_Center(unchecked(nodes[node.child1]).aabb);
                let c2 = b2AABB_Center(unchecked(nodes[node.child2]).aabb);
                if (b2DistanceSquared(c1, p1) < b2DistanceSquared(c2, p1)) {
                    unchecked(stack[stackCount++] = node.child2);
                    unchecked(stack[stackCount++] = node.child1);
                } else {
                    unchecked(stack[stackCount++] = node.child1);
                    unchecked(stack[stackCount++] = node.child2);
                }
            } else {
                assert(stackCount < B2_TREE_STACK_SIZE - 1);
            }
        }
    }

    return stats;
}

// Median split heuristic (B2_TREE_HEURISTIC == 0, the compiled default).
// Operates on [start, start + count) to match C's pointer-offset recursion.
function b2PartitionMid(indices: Int32Array, centers: Array<b2Vec2>, start: i32, count: i32): i32 {
    // Handle trivial case
    if (count <= 2) {
        return count / 2;
    }

    let lowerBound = unchecked(centers[start]).clone();
    let upperBound = unchecked(centers[start]).clone();

    for (let i = 1; i < count; ++i) {
        let c = unchecked(centers[start + i]);
        lowerBound = b2Min(lowerBound, c);
        upperBound = b2Max(upperBound, c);
    }

    let d = b2Sub(upperBound, lowerBound);
    let cx: f32 = <f32>0.5 * (lowerBound.x + upperBound.x);
    let cy: f32 = <f32>0.5 * (lowerBound.y + upperBound.y);

    // Partition longest axis using the Hoare partition scheme
    let i1 = 0;
    let i2 = count;
    if (d.x > d.y) {
        let pivot: f32 = cx;

        while (i1 < i2) {
            while (i1 < i2 && unchecked(centers[start + i1]).x < pivot) {
                i1 += 1;
            }

            while (i1 < i2 && unchecked(centers[start + i2 - 1]).x >= pivot) {
                i2 -= 1;
            }

            if (i1 < i2) {
                // Swap indices
                let temp = unchecked(indices[start + i1]);
                unchecked(indices[start + i1] = indices[start + i2 - 1]);
                unchecked(indices[start + i2 - 1] = temp);

                // Swap centers
                let tempC = unchecked(centers[start + i1]);
                unchecked(centers[start + i1] = centers[start + i2 - 1]);
                unchecked(centers[start + i2 - 1] = tempC);

                i1 += 1;
                i2 -= 1;
            }
        }
    } else {
        let pivot: f32 = cy;

        while (i1 < i2) {
            while (i1 < i2 && unchecked(centers[start + i1]).y < pivot) {
                i1 += 1;
            }

            while (i1 < i2 && unchecked(centers[start + i2 - 1]).y >= pivot) {
                i2 -= 1;
            }

            if (i1 < i2) {
                // Swap indices
                let temp = unchecked(indices[start + i1]);
                unchecked(indices[start + i1] = indices[start + i2 - 1]);
                unchecked(indices[start + i2 - 1] = temp);

                // Swap centers
                let tempC = unchecked(centers[start + i1]);
                unchecked(centers[start + i1] = centers[start + i2 - 1]);
                unchecked(centers[start + i2 - 1] = tempC);

                i1 += 1;
                i2 -= 1;
            }
        }
    }
    assert(i1 == i2);

    if (i1 > 0 && i1 < count) {
        return i1;
    }

    return count / 2;
}

// Temporary data used to track the rebuild of a tree node.
class b2RebuildItem {
    nodeIndex: i32 = 0;
    childCount: i32 = 0;
    startIndex: i32 = 0;
    splitIndex: i32 = 0;
    endIndex: i32 = 0;
}

let s_rebuildStack: Array<b2RebuildItem> = new Array<b2RebuildItem>(0);

function rebuildStack(): Array<b2RebuildItem> {
    if (s_rebuildStack.length == 0) {
        s_rebuildStack = new Array<b2RebuildItem>(B2_TREE_STACK_SIZE);
        for (let i = 0; i < B2_TREE_STACK_SIZE; ++i) {
            s_rebuildStack[i] = new b2RebuildItem();
        }
    }
    return s_rebuildStack;
}

// Returns root node index
function b2BuildTree(tree: b2DynamicTree, leafCount: i32): i32 {
    let leafIndices = tree.leafIndices;

    if (leafCount == 1) {
        tree.nodes[leafIndices[0]].parent = B2_NULL_INDEX;
        return leafIndices[0];
    }

    let leafCenters = tree.leafCenters;

    let stack = rebuildStack();
    let top = 0;

    let item0 = unchecked(stack[0]);
    item0.nodeIndex = b2AllocateNode(tree);
    item0.childCount = -1;
    item0.startIndex = 0;
    item0.endIndex = leafCount;
    item0.splitIndex = b2PartitionMid(leafIndices, leafCenters, 0, leafCount);

    while (true) {
        let item = unchecked(stack[top]);

        item.childCount += 1;

        if (item.childCount == 2) {
            // This internal node has both children established

            if (top == 0) {
                // all done
                break;
            }

            let parentItem = unchecked(stack[top - 1]);
            let parentNode = tree.nodes[parentItem.nodeIndex];

            if (parentItem.childCount == 0) {
                assert(parentNode.child1 == B2_NULL_INDEX);
                parentNode.child1 = item.nodeIndex;
            } else {
                assert(parentItem.childCount == 1);
                assert(parentNode.child2 == B2_NULL_INDEX);
                parentNode.child2 = item.nodeIndex;
            }

            let node = tree.nodes[item.nodeIndex];

            assert(node.parent == B2_NULL_INDEX);
            node.parent = parentItem.nodeIndex;

            assert(node.child1 != B2_NULL_INDEX);
            assert(node.child2 != B2_NULL_INDEX);
            let child1 = tree.nodes[node.child1];
            let child2 = tree.nodes[node.child2];

            node.aabb = b2AABB_Union(child1.aabb, child2.aabb);
            node.height = 1 + b2MaxUInt16(child1.height, child2.height);
            node.categoryBits = child1.categoryBits | child2.categoryBits;

            // Pop stack
            top -= 1;
        } else {
            let startIndex: i32;
            let endIndex: i32;
            if (item.childCount == 0) {
                startIndex = item.startIndex;
                endIndex = item.splitIndex;
            } else {
                assert(item.childCount == 1);
                startIndex = item.splitIndex;
                endIndex = item.endIndex;
            }

            let count = endIndex - startIndex;

            if (count == 1) {
                let childIndex = leafIndices[startIndex];
                let node = tree.nodes[item.nodeIndex];

                if (item.childCount == 0) {
                    assert(node.child1 == B2_NULL_INDEX);
                    node.child1 = childIndex;
                } else {
                    assert(item.childCount == 1);
                    assert(node.child2 == B2_NULL_INDEX);
                    node.child2 = childIndex;
                }

                let childNode = tree.nodes[childIndex];
                assert(childNode.parent == B2_NULL_INDEX);
                childNode.parent = item.nodeIndex;
            } else {
                assert(count > 0);
                assert(top < B2_TREE_STACK_SIZE);

                top += 1;
                let newItem = unchecked(stack[top]);
                newItem.nodeIndex = b2AllocateNode(tree);
                newItem.childCount = -1;
                newItem.startIndex = startIndex;
                newItem.endIndex = endIndex;
                newItem.splitIndex = b2PartitionMid(leafIndices, leafCenters, startIndex, count);
                newItem.splitIndex += startIndex;
            }
        }
    }

    let rootNode = tree.nodes[unchecked(stack[0]).nodeIndex];
    assert(rootNode.parent == B2_NULL_INDEX);
    assert(rootNode.child1 != B2_NULL_INDEX);
    assert(rootNode.child2 != B2_NULL_INDEX);

    let child1 = tree.nodes[rootNode.child1];
    let child2 = tree.nodes[rootNode.child2];

    rootNode.aabb = b2AABB_Union(child1.aabb, child2.aabb);
    rootNode.height = 1 + b2MaxUInt16(child1.height, child2.height);
    rootNode.categoryBits = child1.categoryBits | child2.categoryBits;

    return unchecked(stack[0]).nodeIndex;
}

// Not safe to access tree during this operation because it may grow
export function b2DynamicTree_Rebuild(tree: b2DynamicTree, fullBuild: bool): i32 {
    let proxyCount = tree.proxyCount;
    if (proxyCount == 0) {
        return 0;
    }

    // Ensure capacity for rebuild space
    if (proxyCount > tree.rebuildCapacity) {
        let newCapacity = proxyCount + proxyCount / 2;

        tree.leafIndices = new Int32Array(newCapacity);
        tree.leafCenters = new Array<b2Vec2>(newCapacity);
        for (let i = 0; i < newCapacity; ++i) {
            tree.leafCenters[i] = new b2Vec2();
        }
        tree.rebuildCapacity = newCapacity;
    }

    let leafCount = 0;
    let stack = s_treeStack;
    let stackCount = 0;

    let nodeIndex = tree.root;
    let nodes = tree.nodes;
    let node = nodes[nodeIndex];

    // These are the nodes that get sorted to rebuild the tree.
    // I'm using indices because the node pool may grow during the build.
    let leafIndices = tree.leafIndices;
    let leafCenters = tree.leafCenters;

    // Gather all proxy nodes that have grown and all internal nodes that haven't grown. Both are
    // considered leaves in the tree rebuild.
    // Free all internal nodes that have grown.
    while (true) {
        if (node.height == 0 || ((node.flags & <u16>b2_enlargedNode) == 0 && fullBuild == false)) {
            unchecked(leafIndices[leafCount] = nodeIndex);
            unchecked(leafCenters[leafCount] = b2AABB_Center(node.aabb));
            leafCount += 1;

            // Detach
            node.parent = B2_NULL_INDEX;
        } else {
            let doomedNodeIndex = nodeIndex;

            // Handle children
            nodeIndex = node.child1;

            if (stackCount < B2_TREE_STACK_SIZE) {
                unchecked(stack[stackCount++] = node.child2);
            } else {
                assert(stackCount < B2_TREE_STACK_SIZE);
            }

            node = unchecked(nodes[nodeIndex]);

            // Remove doomed node
            b2FreeNode(tree, doomedNodeIndex);

            continue;
        }

        if (stackCount == 0) {
            break;
        }

        nodeIndex = unchecked(stack[--stackCount]);
        node = unchecked(nodes[nodeIndex]);
    }

    assert(leafCount <= proxyCount);

    tree.root = b2BuildTree(tree, leafCount);

    return leafCount;
}
