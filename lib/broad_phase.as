// Ported from box2d src/broad_phase.h/.c. The move buffers, proxy lifecycle,
// and the pair sweep with its de-duplication logic are verbatim. The world
// dependent parts of b2PairQueryCallback (same body, sensors, filters,
// joints, custom filter) hook in through b2PairFilter so Phase C can supply
// them; the base filter admits every pair. b2CreateContact is Phase C: the
// caller owns adding created pairs to pairSet, matching the C layering.

import { b2AABB, b2AABB_Overlaps } from "./math_functions";
import { b2IntArray, b2IntArray_Create, b2IntArray_Push, b2IntArray_RemoveSwap, b2IntArray_Clear } from "./array";
import { b2HashSet, b2CreateSet, b2DestroySet, b2ClearSet, b2AddKey, b2RemoveKey, b2ContainsKey, B2_SHAPE_PAIR_KEY } from "./table";
import {
    b2DynamicTree, b2TreeQueryCallback, b2DynamicTree_Create, b2DynamicTree_CreateProxy,
    b2DynamicTree_DestroyProxy, b2DynamicTree_MoveProxy, b2DynamicTree_EnlargeProxy, b2DynamicTree_Query,
    b2DynamicTree_Rebuild, b2DynamicTree_GetAABB, b2DynamicTree_GetUserData,
} from "./dynamic_tree";
import { B2_NULL_INDEX } from "./core";

// b2BodyType
export const b2_staticBody: i32 = 0;
export const b2_kinematicBody: i32 = 1;
export const b2_dynamicBody: i32 = 2;
export const b2_bodyTypeCount: i32 = 3;

const B2_DEFAULT_MASK_BITS: u64 = 0xffffffffffffffff;

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
export function B2_PROXY_TYPE(key: i32): i32 {
    return key & 3;
}

export function B2_PROXY_ID(key: i32): i32 {
    return key >> 2;
}

export function B2_PROXY_KEY(id: i32, proxyType: i32): i32 {
    return (id << 2) | proxyType;
}

export class b2BroadPhase {
    trees: StaticArray<b2DynamicTree> = new StaticArray<b2DynamicTree>(b2_bodyTypeCount);
    moveSet: b2HashSet = new b2HashSet();
    moveArray: b2IntArray = new b2IntArray();
    pairSet: b2HashSet = new b2HashSet();
}

// Phase C overrides this with the world dependent shape filtering.
export class b2PairFilter {
    shouldCollide(shapeIdA: i32, shapeIdB: i32): bool {
        return true;
    }
}

export function b2CreateBroadPhase(bp: b2BroadPhase): void {
    bp.moveSet = b2CreateSet(16);
    bp.moveArray = b2IntArray_Create(16);
    bp.pairSet = b2CreateSet(32);

    for (let i = 0; i < b2_bodyTypeCount; ++i) {
        bp.trees[i] = b2DynamicTree_Create();
    }
}

export function b2DestroyBroadPhase(bp: b2BroadPhase): void {
    b2DestroySet(bp.moveSet);
    bp.moveArray = new b2IntArray();
    b2DestroySet(bp.pairSet);
    bp.trees = new StaticArray<b2DynamicTree>(b2_bodyTypeCount);
}

// This is what triggers new contact pairs to be created.
// Warning: this must be called in deterministic order.
export function b2BufferMove(bp: b2BroadPhase, queryProxy: i32): void {
    // Adding 1 because 0 is the sentinel
    let alreadyAdded = b2AddKey(bp.moveSet, <u64>(queryProxy + 1));
    if (alreadyAdded == false) {
        b2IntArray_Push(bp.moveArray, queryProxy);
    }
}

function b2UnBufferMove(bp: b2BroadPhase, proxyKey: i32): void {
    let found = b2RemoveKey(bp.moveSet, <u64>(proxyKey + 1));

    if (found) {
        // Purge from move buffer. Linear search.
        let count = bp.moveArray.count;
        for (let i = 0; i < count; ++i) {
            if (bp.moveArray.data[i] == proxyKey) {
                b2IntArray_RemoveSwap(bp.moveArray, i);
                break;
            }
        }
    }
}

export function b2BroadPhase_CreateProxy(bp: b2BroadPhase, proxyType: i32, aabb: b2AABB, categoryBits: u64,
    shapeIndex: i32, forcePairCreation: bool): i32 {
    assert(0 <= proxyType && proxyType < b2_bodyTypeCount);
    let proxyId = b2DynamicTree_CreateProxy(bp.trees[proxyType], aabb, categoryBits, <u64>shapeIndex);
    let proxyKey = B2_PROXY_KEY(proxyId, proxyType);
    if (proxyType != b2_staticBody || forcePairCreation) {
        b2BufferMove(bp, proxyKey);
    }
    return proxyKey;
}

export function b2BroadPhase_DestroyProxy(bp: b2BroadPhase, proxyKey: i32): void {
    assert(bp.moveArray.count == <i32>bp.moveSet.count);
    b2UnBufferMove(bp, proxyKey);

    let proxyType = B2_PROXY_TYPE(proxyKey);
    let proxyId = B2_PROXY_ID(proxyKey);

    assert(0 <= proxyType && proxyType <= b2_bodyTypeCount);
    b2DynamicTree_DestroyProxy(bp.trees[proxyType], proxyId);
}

export function b2BroadPhase_MoveProxy(bp: b2BroadPhase, proxyKey: i32, aabb: b2AABB): void {
    let proxyType = B2_PROXY_TYPE(proxyKey);
    let proxyId = B2_PROXY_ID(proxyKey);

    b2DynamicTree_MoveProxy(bp.trees[proxyType], proxyId, aabb);
    b2BufferMove(bp, proxyKey);
}

export function b2BroadPhase_EnlargeProxy(bp: b2BroadPhase, proxyKey: i32, aabb: b2AABB): void {
    assert(proxyKey != B2_NULL_INDEX);
    let typeIndex = B2_PROXY_TYPE(proxyKey);
    let proxyId = B2_PROXY_ID(proxyKey);

    assert(typeIndex != b2_staticBody);

    b2DynamicTree_EnlargeProxy(bp.trees[typeIndex], proxyId, aabb);
    b2BufferMove(bp, proxyKey);
}

// C prepends b2MovePair nodes to a per-move list; the array plays the list
// role so traversal order is reproduced by walking it backwards.
class b2QueryPairContext extends b2TreeQueryCallback {
    bp: b2BroadPhase;
    filter: b2PairFilter;
    // pair list for the current moved proxy, in prepend order
    pairsA: b2IntArray = new b2IntArray();
    pairsB: b2IntArray = new b2IntArray();
    queryTreeType: i32 = 0;
    queryProxyKey: i32 = 0;
    queryShapeIndex: i32 = 0;

    constructor(bp: b2BroadPhase, filter: b2PairFilter) {
        super();
        this.bp = bp;
        this.filter = filter;
    }

    // This is the portable body of b2PairQueryCallback.
    queryCallback(proxyId: i32, userData: u64): bool {
        let shapeId = <i32>userData;

        let broadPhase = this.bp;

        let proxyKey = B2_PROXY_KEY(proxyId, this.queryTreeType);
        let queryProxyKey = this.queryProxyKey;

        // A proxy cannot form a pair with itself.
        if (proxyKey == queryProxyKey) {
            return true;
        }

        let treeType = this.queryTreeType;
        let queryProxyType = B2_PROXY_TYPE(queryProxyKey);

        // De-duplication: prevent duplicate contacts when both proxies moved.
        if (queryProxyType == b2_dynamicBody) {
            if (treeType == b2_dynamicBody && proxyKey < queryProxyKey) {
                let moved = b2ContainsKey(broadPhase.moveSet, <u64>(proxyKey + 1));
                if (moved) {
                    // Both proxies are moving. Avoid duplicate pairs.
                    return true;
                }
            }
        } else {
            assert(treeType == b2_dynamicBody);
            let moved = b2ContainsKey(broadPhase.moveSet, <u64>(proxyKey + 1));
            if (moved) {
                // Both proxies are moving. Avoid duplicate pairs.
                return true;
            }
        }

        let pairKey = B2_SHAPE_PAIR_KEY(shapeId, this.queryShapeIndex);
        if (b2ContainsKey(broadPhase.pairSet, pairKey)) {
            // contact exists
            return true;
        }

        let shapeIdA: i32;
        let shapeIdB: i32;
        if (proxyKey < queryProxyKey) {
            shapeIdA = shapeId;
            shapeIdB = this.queryShapeIndex;
        } else {
            shapeIdA = this.queryShapeIndex;
            shapeIdB = shapeId;
        }

        // Phase C world filtering (same body, sensors, filters, joints, custom)
        if (this.filter.shouldCollide(shapeIdA, shapeIdB) == false) {
            return true;
        }

        b2IntArray_Push(this.pairsA, shapeIdA);
        b2IntArray_Push(this.pairsB, shapeIdB);

        // continue the query
        return true;
    }
}

// Pair sweep of b2UpdateBroadPhasePairs: pushes (shapeIdA, shapeIdB) into
// outPairs in the deterministic creation order C hands to b2CreateContact,
// then clears the move buffer. The caller creates contacts and adds each
// pair key to pairSet, as b2CreateContact does in C.
export function b2UpdateBroadPhasePairs(bp: b2BroadPhase, filter: b2PairFilter, outPairs: b2IntArray): void {
    let moveCount = bp.moveArray.count;
    assert(moveCount == <i32>bp.moveSet.count);

    if (moveCount == 0) {
        return;
    }

    let queryContext = new b2QueryPairContext(bp, filter);

    for (let i = 0; i < moveCount; ++i) {
        let proxyKey = bp.moveArray.data[i];
        if (proxyKey == B2_NULL_INDEX) {
            // proxy was destroyed after it moved
            continue;
        }

        let proxyType = B2_PROXY_TYPE(proxyKey);
        let proxyId = B2_PROXY_ID(proxyKey);
        queryContext.queryProxyKey = proxyKey;

        let baseTree = bp.trees[proxyType];

        // We have to query the tree with the fat AABB so that
        // we don't fail to create a contact that may touch later.
        let fatAABB = b2DynamicTree_GetAABB(baseTree, proxyId);
        queryContext.queryShapeIndex = <i32>b2DynamicTree_GetUserData(baseTree, proxyId);

        // Mark the start of this move's pair list.
        let pairStart = queryContext.pairsA.count;

        // Query trees. Only dynamic proxies collide with kinematic and static proxies.
        if (proxyType == b2_dynamicBody) {
            queryContext.queryTreeType = b2_kinematicBody;
            b2DynamicTree_Query(bp.trees[b2_kinematicBody], fatAABB, B2_DEFAULT_MASK_BITS, queryContext);

            queryContext.queryTreeType = b2_staticBody;
            b2DynamicTree_Query(bp.trees[b2_staticBody], fatAABB, B2_DEFAULT_MASK_BITS, queryContext);
        }

        // All proxies collide with dynamic proxies
        queryContext.queryTreeType = b2_dynamicBody;
        b2DynamicTree_Query(bp.trees[b2_dynamicBody], fatAABB, B2_DEFAULT_MASK_BITS, queryContext);

        // C prepends into pairList and then walks it, so creation order per
        // move is the reverse of query order.
        for (let j = queryContext.pairsA.count - 1; j >= pairStart; --j) {
            b2IntArray_Push(outPairs, queryContext.pairsA.data[j]);
            b2IntArray_Push(outPairs, queryContext.pairsB.data[j]);
        }
    }

    // Reset move buffer
    b2IntArray_Clear(bp.moveArray);
    b2ClearSet(bp.moveSet);
}

export function b2BroadPhase_TestOverlap(bp: b2BroadPhase, proxyKeyA: i32, proxyKeyB: i32): bool {
    let typeIndexA = B2_PROXY_TYPE(proxyKeyA);
    let proxyIdA = B2_PROXY_ID(proxyKeyA);
    let typeIndexB = B2_PROXY_TYPE(proxyKeyB);
    let proxyIdB = B2_PROXY_ID(proxyKeyB);

    let aabbA = b2DynamicTree_GetAABB(bp.trees[typeIndexA], proxyIdA);
    let aabbB = b2DynamicTree_GetAABB(bp.trees[typeIndexB], proxyIdB);
    return b2AABB_Overlaps(aabbA, aabbB);
}

export function b2BroadPhase_RebuildTrees(bp: b2BroadPhase): void {
    b2DynamicTree_Rebuild(bp.trees[b2_dynamicBody], false);
    b2DynamicTree_Rebuild(bp.trees[b2_kinematicBody], false);
}

export function b2BroadPhase_GetShapeIndex(bp: b2BroadPhase, proxyKey: i32): i32 {
    let typeIndex = B2_PROXY_TYPE(proxyKey);
    let proxyId = B2_PROXY_ID(proxyKey);

    return <i32>b2DynamicTree_GetUserData(bp.trees[typeIndex], proxyId);
}
