// Phase B verification entry: bit-level drivers for the node harness covering
// distance/GJK, shape cast, TOI, every manifold collide function, the dynamic
// tree, the hash set, and the broad phase. Inputs and outputs travel as raw
// f32 bit patterns so the comparison is bit-exact.

import { b2Vec2, b2Rot, b2Transform, b2AABB } from "./lib/math_functions";
import {
    b2ShapeProxy, b2SimplexCache, b2DistanceInput, b2ShapeCastPairInput, b2Sweep, b2TOIInput,
    b2Circle, b2Capsule, b2Polygon, b2Segment, b2ChainSegment, b2Manifold, b2RayCastInput,
    b2ShapeCastInput, b2CastOutput, makeVecArray,
} from "./lib/collision";
import { b2MakeProxy, b2ShapeDistance, b2ShapeCast, b2TimeOfImpact } from "./lib/distance";
import {
    b2CollideCircles, b2CollideCapsuleAndCircle, b2CollidePolygonAndCircle, b2CollideCapsules,
    b2CollideSegmentAndCapsule, b2CollidePolygonAndCapsule, b2CollidePolygons, b2CollideSegmentAndCircle,
    b2CollideSegmentAndPolygon, b2CollideChainSegmentAndCircle, b2CollideChainSegmentAndCapsule,
    b2CollideChainSegmentAndPolygon,
} from "./lib/manifold";
import { b2ComputeHull } from "./lib/hull";
import {
    b2MakePolygon, b2MakeRoundedBox, b2PointInPolygon, b2RayCastPolygon,
    b2ShapeCastCircle, b2ShapeCastCapsule, b2ShapeCastSegment, b2ShapeCastPolygon,
} from "./lib/geometry";
import { b2AABB_RayCast } from "./lib/aabb";
import {
    b2DynamicTree, b2TreeQueryCallback, b2TreeRayCastCallback, b2TreeShapeCastCallback,
    b2DynamicTree_Create, b2DynamicTree_CreateProxy, b2DynamicTree_DestroyProxy, b2DynamicTree_MoveProxy,
    b2DynamicTree_EnlargeProxy, b2DynamicTree_Rebuild, b2DynamicTree_Query, b2DynamicTree_RayCast,
    b2DynamicTree_ShapeCast, b2DynamicTree_GetAABB,
} from "./lib/dynamic_tree";
import {
    b2HashSet, b2CreateSet, b2ClearSet, b2AddKey, b2RemoveKey, b2ContainsKey, B2_SHAPE_PAIR_KEY,
} from "./lib/table";
import {
    b2BroadPhase, b2PairFilter, b2CreateBroadPhase, b2BroadPhase_CreateProxy, b2BroadPhase_DestroyProxy,
    b2BroadPhase_MoveProxy, b2BroadPhase_EnlargeProxy, b2UpdateBroadPhasePairs, b2BroadPhase_TestOverlap,
    b2BroadPhase_RebuildTrees,
} from "./lib/broad_phase";
import { b2IntArray } from "./lib/array";
import { b2_allocatedNode, b2_leafNode } from "./lib/constants";

function F(bits: u32): f32 {
    return reinterpret<f32>(bits);
}

function B(x: f32): u32 {
    return reinterpret<u32>(x);
}

let res = new StaticArray<u32>(64);

export function resGet(i: i32): u32 {
    return res[i];
}

function xfOf(px: u32, py: u32, qc: u32, qs: u32): b2Transform {
    return new b2Transform(new b2Vec2(F(px), F(py)), new b2Rot(F(qc), F(qs)));
}

function aabbOf(l0: u32, l1: u32, u0: u32, u1: u32): b2AABB {
    return new b2AABB(new b2Vec2(F(l0), F(l1)), new b2Vec2(F(u0), F(u1)));
}

// ---- staged points and proxies ----

let ptsA = makeVecArray(16);
let ptsB = makeVecArray(16);
let proxyA = new b2ShapeProxy();
let proxyB = new b2ShapeProxy();

export function dSetPtA(i: i32, xb: u32, yb: u32): void {
    ptsA[i] = new b2Vec2(F(xb), F(yb));
}

export function dSetPtB(i: i32, xb: u32, yb: u32): void {
    ptsB[i] = new b2Vec2(F(xb), F(yb));
}

export function dStageProxyA(count: i32, radius: u32): void {
    proxyA = b2MakeProxy(ptsA, count, F(radius));
}

export function dStageProxyB(count: i32, radius: u32): void {
    proxyB = b2MakeProxy(ptsB, count, F(radius));
}

// ---- simplex cache staging ----

let cache = new b2SimplexCache();

export function dSetCache(cc: i32, a0: i32, a1: i32, a2: i32, b0: i32, b1: i32, b2i: i32): void {
    cache = new b2SimplexCache();
    cache.count = <u16>cc;
    cache.indexA[0] = <u8>a0;
    cache.indexA[1] = <u8>a1;
    cache.indexA[2] = <u8>a2;
    cache.indexB[0] = <u8>b0;
    cache.indexB[1] = <u8>b1;
    cache.indexB[2] = <u8>b2i;
}

function setResCache(at: i32): void {
    res[at] = <u32>cache.count;
    res[at + 1] = <u32>cache.indexA[0];
    res[at + 2] = <u32>cache.indexA[1];
    res[at + 3] = <u32>cache.indexA[2];
    res[at + 4] = <u32>cache.indexB[0];
    res[at + 5] = <u32>cache.indexB[1];
    res[at + 6] = <u32>cache.indexB[2];
}

// ---- distance / shape cast / TOI ----

export function dDistance(pax: u32, pay: u32, qac: u32, qas: u32,
    pbx: u32, pby: u32, qbc: u32, qbs: u32, useRadii: i32): void {
    let input = new b2DistanceInput();
    input.proxyA = proxyA;
    input.proxyB = proxyB;
    input.transformA = xfOf(pax, pay, qac, qas);
    input.transformB = xfOf(pbx, pby, qbc, qbs);
    input.useRadii = useRadii != 0;

    let out = b2ShapeDistance(input, cache);

    res[0] = B(out.pointA.x);
    res[1] = B(out.pointA.y);
    res[2] = B(out.pointB.x);
    res[3] = B(out.pointB.y);
    res[4] = B(out.normal.x);
    res[5] = B(out.normal.y);
    res[6] = B(out.distance);
    res[7] = <u32>out.iterations;
    setResCache(8);
}

export function dShapeCast(pax: u32, pay: u32, qac: u32, qas: u32,
    pbx: u32, pby: u32, qbc: u32, qbs: u32, tx: u32, ty: u32, mf: u32, canEncroach: i32): void {
    let input = new b2ShapeCastPairInput();
    input.proxyA = proxyA;
    input.proxyB = proxyB;
    input.transformA = xfOf(pax, pay, qac, qas);
    input.transformB = xfOf(pbx, pby, qbc, qbs);
    input.translationB = new b2Vec2(F(tx), F(ty));
    input.maxFraction = F(mf);
    input.canEncroach = canEncroach != 0;

    let out = b2ShapeCast(input);

    res[0] = out.hit ? 1 : 0;
    res[1] = B(out.fraction);
    res[2] = B(out.normal.x);
    res[3] = B(out.normal.y);
    res[4] = B(out.point.x);
    res[5] = B(out.point.y);
    res[6] = <u32>out.iterations;
}

let sweepA = new b2Sweep();
let sweepB = new b2Sweep();

function sweepOf(lcx: u32, lcy: u32, c1x: u32, c1y: u32, c2x: u32, c2y: u32,
    q1c: u32, q1s: u32, q2c: u32, q2s: u32): b2Sweep {
    let s = new b2Sweep();
    s.localCenter = new b2Vec2(F(lcx), F(lcy));
    s.c1 = new b2Vec2(F(c1x), F(c1y));
    s.c2 = new b2Vec2(F(c2x), F(c2y));
    s.q1 = new b2Rot(F(q1c), F(q1s));
    s.q2 = new b2Rot(F(q2c), F(q2s));
    return s;
}

export function dSetSweepA(lcx: u32, lcy: u32, c1x: u32, c1y: u32, c2x: u32, c2y: u32,
    q1c: u32, q1s: u32, q2c: u32, q2s: u32): void {
    sweepA = sweepOf(lcx, lcy, c1x, c1y, c2x, c2y, q1c, q1s, q2c, q2s);
}

export function dSetSweepB(lcx: u32, lcy: u32, c1x: u32, c1y: u32, c2x: u32, c2y: u32,
    q1c: u32, q1s: u32, q2c: u32, q2s: u32): void {
    sweepB = sweepOf(lcx, lcy, c1x, c1y, c2x, c2y, q1c, q1s, q2c, q2s);
}

export function dTOI(mf: u32): void {
    let input = new b2TOIInput();
    input.proxyA = proxyA;
    input.proxyB = proxyB;
    input.sweepA = sweepA;
    input.sweepB = sweepB;
    input.maxFraction = F(mf);

    let out = b2TimeOfImpact(input);

    res[0] = <u32>out.state;
    res[1] = B(out.fraction);
    res[2] = B(out.point.x);
    res[3] = B(out.point.y);
    res[4] = B(out.normal.x);
    res[5] = B(out.normal.y);
}

// ---- geometry tail: point in polygon, rounded ray cast, cast wrappers ----

let stagedPolyA = new b2Polygon();
let stagedPolyB = new b2Polygon();

export function mStagePolyA(count: i32, radius: u32): void {
    let hull = b2ComputeHull(ptsA, count);
    stagedPolyA = b2MakePolygon(hull, F(radius));
}

export function mStagePolyB(count: i32, radius: u32): void {
    let hull = b2ComputeHull(ptsB, count);
    stagedPolyB = b2MakePolygon(hull, F(radius));
}

export function pPointInPolygon(px: u32, py: u32): u32 {
    return b2PointInPolygon(stagedPolyA, new b2Vec2(F(px), F(py))) ? 1 : 0;
}

function setResCast(out: b2CastOutput): void {
    res[0] = out.hit ? 1 : 0;
    res[1] = B(out.fraction);
    res[2] = B(out.normal.x);
    res[3] = B(out.normal.y);
    res[4] = B(out.point.x);
    res[5] = B(out.point.y);
    res[6] = <u32>out.iterations;
}

export function pRayCastPolygon(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32): void {
    let input = new b2RayCastInput();
    input.origin = new b2Vec2(F(ox), F(oy));
    input.translation = new b2Vec2(F(tx), F(ty));
    input.maxFraction = F(mf);
    setResCast(b2RayCastPolygon(stagedPolyA, input));
}

// Fixed target shapes match the generator's constants.
export function wShapeCastWrapper(which: i32, tx: u32, ty: u32, mf: u32, canEncroach: i32): void {
    let input = new b2ShapeCastInput();
    input.proxy = proxyB;
    input.translation = new b2Vec2(F(tx), F(ty));
    input.maxFraction = F(mf);
    input.canEncroach = canEncroach != 0;

    let out: b2CastOutput;
    if (which == 0) {
        let circle = new b2Circle(new b2Vec2(0.1, -0.05), 0.4);
        out = b2ShapeCastCircle(circle, input);
    } else if (which == 1) {
        let capsule = new b2Capsule(new b2Vec2(-0.4, 0.1), new b2Vec2(0.45, -0.1), 0.3);
        out = b2ShapeCastCapsule(capsule, input);
    } else if (which == 2) {
        let segment = new b2Segment(new b2Vec2(-0.55, -0.2), new b2Vec2(0.5, 0.25));
        out = b2ShapeCastSegment(segment, input);
    } else {
        let box = b2MakeRoundedBox(0.45, 0.3, 0.1);
        out = b2ShapeCastPolygon(box, input);
    }
    setResCast(out);
}

// ---- manifolds ----

function setResManifold(m: b2Manifold): void {
    res[0] = <u32>m.pointCount;
    res[1] = B(m.normal.x);
    res[2] = B(m.normal.y);
    for (let i = 0; i < m.pointCount; ++i) {
        let mp = m.points[i];
        let at = 3 + 8 * i;
        res[at] = B(mp.point.x);
        res[at + 1] = B(mp.point.y);
        res[at + 2] = B(mp.anchorA.x);
        res[at + 3] = B(mp.anchorA.y);
        res[at + 4] = B(mp.anchorB.x);
        res[at + 5] = B(mp.anchorB.y);
        res[at + 6] = B(mp.separation);
        res[at + 7] = <u32>mp.id;
    }
}

export function mCollideCircles(cax: u32, cay: u32, car: u32, cbx: u32, cby: u32, cbr: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let circleA = new b2Circle(new b2Vec2(F(cax), F(cay)), F(car));
    let circleB = new b2Circle(new b2Vec2(F(cbx), F(cby)), F(cbr));
    setResManifold(b2CollideCircles(circleA, xfOf(apx, apy, aqc, aqs), circleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollideCapsuleAndCircle(k1x: u32, k1y: u32, k2x: u32, k2y: u32, kr: u32,
    cbx: u32, cby: u32, cbr: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let capsuleA = new b2Capsule(new b2Vec2(F(k1x), F(k1y)), new b2Vec2(F(k2x), F(k2y)), F(kr));
    let circleB = new b2Circle(new b2Vec2(F(cbx), F(cby)), F(cbr));
    setResManifold(b2CollideCapsuleAndCircle(capsuleA, xfOf(apx, apy, aqc, aqs), circleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollideCapsules(a1x: u32, a1y: u32, a2x: u32, a2y: u32, ar: u32,
    b1x: u32, b1y: u32, b2x: u32, b2y: u32, br: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let capsuleA = new b2Capsule(new b2Vec2(F(a1x), F(a1y)), new b2Vec2(F(a2x), F(a2y)), F(ar));
    let capsuleB = new b2Capsule(new b2Vec2(F(b1x), F(b1y)), new b2Vec2(F(b2x), F(b2y)), F(br));
    setResManifold(b2CollideCapsules(capsuleA, xfOf(apx, apy, aqc, aqs), capsuleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollideSegmentAndCapsule(s1x: u32, s1y: u32, s2x: u32, s2y: u32,
    b1x: u32, b1y: u32, b2x: u32, b2y: u32, br: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let segmentA = new b2Segment(new b2Vec2(F(s1x), F(s1y)), new b2Vec2(F(s2x), F(s2y)));
    let capsuleB = new b2Capsule(new b2Vec2(F(b1x), F(b1y)), new b2Vec2(F(b2x), F(b2y)), F(br));
    setResManifold(b2CollideSegmentAndCapsule(segmentA, xfOf(apx, apy, aqc, aqs), capsuleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollidePolygonAndCircle(cbx: u32, cby: u32, cbr: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let circleB = new b2Circle(new b2Vec2(F(cbx), F(cby)), F(cbr));
    setResManifold(b2CollidePolygonAndCircle(stagedPolyA, xfOf(apx, apy, aqc, aqs), circleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollidePolygonAndCapsule(b1x: u32, b1y: u32, b2x: u32, b2y: u32, br: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let capsuleB = new b2Capsule(new b2Vec2(F(b1x), F(b1y)), new b2Vec2(F(b2x), F(b2y)), F(br));
    setResManifold(b2CollidePolygonAndCapsule(stagedPolyA, xfOf(apx, apy, aqc, aqs), capsuleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollidePolygons(apx: u32, apy: u32, aqc: u32, aqs: u32,
    bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    setResManifold(b2CollidePolygons(stagedPolyA, xfOf(apx, apy, aqc, aqs), stagedPolyB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollideSegmentAndCircle(s1x: u32, s1y: u32, s2x: u32, s2y: u32,
    cbx: u32, cby: u32, cbr: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let segmentA = new b2Segment(new b2Vec2(F(s1x), F(s1y)), new b2Vec2(F(s2x), F(s2y)));
    let circleB = new b2Circle(new b2Vec2(F(cbx), F(cby)), F(cbr));
    setResManifold(b2CollideSegmentAndCircle(segmentA, xfOf(apx, apy, aqc, aqs), circleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollideSegmentAndPolygon(s1x: u32, s1y: u32, s2x: u32, s2y: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let segmentA = new b2Segment(new b2Vec2(F(s1x), F(s1y)), new b2Vec2(F(s2x), F(s2y)));
    setResManifold(b2CollideSegmentAndPolygon(segmentA, xfOf(apx, apy, aqc, aqs), stagedPolyB, xfOf(bpx, bpy, bqc, bqs)));
}

let stagedChain = new b2ChainSegment();

export function mSetChain(g1x: u32, g1y: u32, p1x: u32, p1y: u32, p2x: u32, p2y: u32, g2x: u32, g2y: u32): void {
    stagedChain = new b2ChainSegment();
    stagedChain.ghost1 = new b2Vec2(F(g1x), F(g1y));
    stagedChain.segment = new b2Segment(new b2Vec2(F(p1x), F(p1y)), new b2Vec2(F(p2x), F(p2y)));
    stagedChain.ghost2 = new b2Vec2(F(g2x), F(g2y));
    stagedChain.chainId = 0;
}

export function mCollideChainSegmentAndCircle(cbx: u32, cby: u32, cbr: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let circleB = new b2Circle(new b2Vec2(F(cbx), F(cby)), F(cbr));
    setResManifold(b2CollideChainSegmentAndCircle(stagedChain, xfOf(apx, apy, aqc, aqs), circleB, xfOf(bpx, bpy, bqc, bqs)));
}

export function mCollideChainSegmentAndCapsule(b1x: u32, b1y: u32, b2x: u32, b2y: u32, br: u32,
    apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let capsuleB = new b2Capsule(new b2Vec2(F(b1x), F(b1y)), new b2Vec2(F(b2x), F(b2y)), F(br));
    setResManifold(b2CollideChainSegmentAndCapsule(stagedChain, xfOf(apx, apy, aqc, aqs), capsuleB,
        xfOf(bpx, bpy, bqc, bqs), cache));
    setResCache(20);
}

export function mCollideChainSegmentAndPolygon(apx: u32, apy: u32, aqc: u32, aqs: u32,
    bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    setResManifold(b2CollideChainSegmentAndPolygon(stagedChain, xfOf(apx, apy, aqc, aqs), stagedPolyB,
        xfOf(bpx, bpy, bqc, bqs), cache));
    setResCache(20);
}

// ---- dynamic tree ----

let tree = b2DynamicTree_Create();

// FNV-style structure hash mirroring ref_tree.c treeHash.
let hashState: u64 = 0;

function mix(v: u32): void {
    hashState ^= <u64>v;
    hashState *= 0x100000001b3;
}

function mixF(f: f32): void {
    mix(B(f));
}

function treeHashOf(t: b2DynamicTree): u64 {
    hashState = 0xcbf29ce484222325;
    mix(<u32>t.root);
    mix(<u32>t.nodeCount);
    mix(<u32>t.nodeCapacity);
    mix(<u32>t.freeList);
    mix(<u32>t.proxyCount);
    for (let i = 0; i < t.nodeCapacity; ++i) {
        let n = unchecked(t.nodes[i]);
        mix(<u32>n.flags);
        if ((n.flags & <u16>b2_allocatedNode) != 0) {
            mix(<u32>n.parent);
            mix(<u32>n.height);
            mix(<u32>n.categoryBits);
            mix(<u32>(n.categoryBits >> 32));
            mixF(n.aabb.lowerBound.x);
            mixF(n.aabb.lowerBound.y);
            mixF(n.aabb.upperBound.x);
            mixF(n.aabb.upperBound.y);
            if ((n.flags & <u16>b2_leafNode) != 0) {
                mix(<u32>n.userData);
                mix(<u32>(n.userData >> 32));
            } else {
                mix(<u32>n.child1);
                mix(<u32>n.child2);
            }
        } else {
            mix(<u32>n.parent);
        }
    }
    return hashState;
}

export function trReset(): void {
    tree = b2DynamicTree_Create();
}

export function trCreate(l0: u32, l1: u32, u0: u32, u1: u32, cat: u64, ud: u64): i32 {
    return b2DynamicTree_CreateProxy(tree, aabbOf(l0, l1, u0, u1), cat, ud);
}

export function trDestroy(proxyId: i32): void {
    b2DynamicTree_DestroyProxy(tree, proxyId);
}

export function trMove(proxyId: i32, l0: u32, l1: u32, u0: u32, u1: u32): void {
    b2DynamicTree_MoveProxy(tree, proxyId, aabbOf(l0, l1, u0, u1));
}

export function trEnlarge(proxyId: i32, l0: u32, l1: u32, u0: u32, u1: u32): void {
    b2DynamicTree_EnlargeProxy(tree, proxyId, aabbOf(l0, l1, u0, u1));
}

export function trRebuild(full: i32): i32 {
    return b2DynamicTree_Rebuild(tree, full != 0);
}

export function trState(): void {
    res[0] = <u32>tree.root;
    res[1] = <u32>tree.nodeCount;
    res[2] = <u32>tree.nodeCapacity;
    res[3] = <u32>tree.freeList;
    res[4] = <u32>tree.proxyCount;
}

export function trHash(): u64 {
    return treeHashOf(tree);
}

// Visit recorders mirroring ref_tree.c (cap 512).
let visited = new StaticArray<i32>(512);
let visitCount = 0;

class RecordQuery extends b2TreeQueryCallback {
    queryCallback(proxyId: i32, userData: u64): bool {
        if (visitCount < 512) {
            visited[visitCount++] = proxyId;
        }
        return true;
    }
}

class RecordRay extends b2TreeRayCastCallback {
    rayCastCallback(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
        if (visitCount < 512) {
            visited[visitCount++] = proxyId;
        }
        return input.maxFraction;
    }
}

class ClipRay extends b2TreeRayCastCallback {
    rayCastCallback(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
        if (visitCount < 512) {
            visited[visitCount++] = proxyId;
        }

        let aabb = b2DynamicTree_GetAABB(tree, proxyId);
        let p2 = new b2Vec2(input.origin.x + input.translation.x, input.origin.y + input.translation.y);
        let out = b2AABB_RayCast(aabb, input.origin, p2);
        if (out.hit && out.fraction <= input.maxFraction) {
            return out.fraction;
        }
        return input.maxFraction;
    }
}

class RecordShapeCast extends b2TreeShapeCastCallback {
    shapeCastCallback(input: b2ShapeCastInput, proxyId: i32, userData: u64): f32 {
        if (visitCount < 512) {
            visited[visitCount++] = proxyId;
        }
        return input.maxFraction;
    }
}

class ClipShapeCast extends b2TreeShapeCastCallback {
    shapeCastCallback(input: b2ShapeCastInput, proxyId: i32, userData: u64): f32 {
        if (visitCount < 512) {
            visited[visitCount++] = proxyId;
        }

        let aabb = b2DynamicTree_GetAABB(tree, proxyId);
        let p1 = input.proxy.points[0];
        let p2 = new b2Vec2(p1.x + input.translation.x, p1.y + input.translation.y);
        let out = b2AABB_RayCast(aabb, p1, p2);
        if (out.hit && out.fraction <= input.maxFraction) {
            return out.fraction;
        }
        return input.maxFraction;
    }
}

let recordQuery = new RecordQuery();
let recordRay = new RecordRay();
let clipRay = new ClipRay();
let recordShapeCast = new RecordShapeCast();
let clipShapeCast = new ClipShapeCast();

export function vGet(i: i32): i32 {
    return visited[i];
}

export function trQuery(l0: u32, l1: u32, u0: u32, u1: u32, mask: u64): void {
    visitCount = 0;
    let stats = b2DynamicTree_Query(tree, aabbOf(l0, l1, u0, u1), mask, recordQuery);
    res[0] = <u32>stats.nodeVisits;
    res[1] = <u32>stats.leafVisits;
    res[2] = <u32>visitCount;
}

export function trRayCast(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32, mode: i32): void {
    let input = new b2RayCastInput();
    input.origin = new b2Vec2(F(ox), F(oy));
    input.translation = new b2Vec2(F(tx), F(ty));
    input.maxFraction = F(mf);

    visitCount = 0;
    let stats = b2DynamicTree_RayCast(tree, input, 0xffffffffffffffff,
        mode == 0 ? <b2TreeRayCastCallback>recordRay : <b2TreeRayCastCallback>clipRay);
    res[0] = <u32>stats.nodeVisits;
    res[1] = <u32>stats.leafVisits;
    res[2] = <u32>visitCount;
}

export function trShapeCast(count: i32, radius: u32, tx: u32, ty: u32, mf: u32, mode: i32): void {
    let input = new b2ShapeCastInput();
    input.proxy = b2MakeProxy(ptsA, count, F(radius));
    input.translation = new b2Vec2(F(tx), F(ty));
    input.maxFraction = F(mf);
    input.canEncroach = false;

    visitCount = 0;
    let stats = b2DynamicTree_ShapeCast(tree, input, 0xffffffffffffffff,
        mode == 0 ? <b2TreeShapeCastCallback>recordShapeCast : <b2TreeShapeCastCallback>clipShapeCast);
    res[0] = <u32>stats.nodeVisits;
    res[1] = <u32>stats.leafVisits;
    res[2] = <u32>visitCount;
}

// ---- hash set ----

let hashSet = new b2HashSet();

function setHashOf(s: b2HashSet): u64 {
    hashState = 0xcbf29ce484222325;
    mix(s.capacity);
    mix(s.count);
    for (let i: u32 = 0; i < s.capacity; ++i) {
        let key = unchecked(s.items[i]);
        mix(<u32>key);
        mix(<u32>(key >> 32));
    }
    return hashState;
}

export function hsReset(cap: i32): i32 {
    hashSet = b2CreateSet(cap);
    return <i32>hashSet.capacity;
}

export function hsAdd(key: u64): i32 {
    return b2AddKey(hashSet, key) ? 1 : 0;
}

export function hsRemove(key: u64): i32 {
    return b2RemoveKey(hashSet, key) ? 1 : 0;
}

export function hsContains(key: u64): i32 {
    return b2ContainsKey(hashSet, key) ? 1 : 0;
}

export function hsClear(): void {
    b2ClearSet(hashSet);
}

export function hsCount(): i32 {
    return <i32>hashSet.count;
}

export function hsCapacity(): i32 {
    return <i32>hashSet.capacity;
}

export function hsHash(): u64 {
    return setHashOf(hashSet);
}

// ---- broad phase ----

let broadPhase = new b2BroadPhase();
let pairFilter = new b2PairFilter();
let bpPairs = new b2IntArray();

export function bpReset(): void {
    broadPhase = new b2BroadPhase();
    b2CreateBroadPhase(broadPhase);
}

export function bpCreate(proxyType: i32, l0: u32, l1: u32, u0: u32, u1: u32, cat: u64,
    shapeIndex: i32, force: i32): i32 {
    return b2BroadPhase_CreateProxy(broadPhase, proxyType, aabbOf(l0, l1, u0, u1), cat, shapeIndex, force != 0);
}

export function bpDestroy(proxyKey: i32): void {
    b2BroadPhase_DestroyProxy(broadPhase, proxyKey);
}

export function bpMove(proxyKey: i32, l0: u32, l1: u32, u0: u32, u1: u32): void {
    b2BroadPhase_MoveProxy(broadPhase, proxyKey, aabbOf(l0, l1, u0, u1));
}

export function bpEnlarge(proxyKey: i32, l0: u32, l1: u32, u0: u32, u1: u32): void {
    b2BroadPhase_EnlargeProxy(broadPhase, proxyKey, aabbOf(l0, l1, u0, u1));
}

// Runs the pair sweep then adds pair keys like b2CreateContact. Returns the
// created pair count.
export function bpUpdate(): i32 {
    bpPairs = new b2IntArray();
    b2UpdateBroadPhasePairs(broadPhase, pairFilter, bpPairs);
    let n = bpPairs.count / 2;
    for (let i = 0; i < n; ++i) {
        let pairKey = B2_SHAPE_PAIR_KEY(bpPairs.data[2 * i], bpPairs.data[2 * i + 1]);
        b2AddKey(broadPhase.pairSet, pairKey);
    }
    return n;
}

export function bpPairGet(i: i32): i32 {
    return bpPairs.data[i];
}

export function bpMoveCount(): i32 {
    return broadPhase.moveArray.count;
}

export function bpMoveGet(i: i32): i32 {
    return broadPhase.moveArray.data[i];
}

export function bpMoveSetCount(): i32 {
    return <i32>broadPhase.moveSet.count;
}

export function bpMoveSetCapacity(): i32 {
    return <i32>broadPhase.moveSet.capacity;
}

export function bpMoveSetHash(): u64 {
    return setHashOf(broadPhase.moveSet);
}

export function bpPairSetCount(): i32 {
    return <i32>broadPhase.pairSet.count;
}

export function bpPairSetCapacity(): i32 {
    return <i32>broadPhase.pairSet.capacity;
}

export function bpTest(keyA: i32, keyB: i32): i32 {
    return b2BroadPhase_TestOverlap(broadPhase, keyA, keyB) ? 1 : 0;
}

export function bpRebuild(): void {
    b2BroadPhase_RebuildTrees(broadPhase);
}

export function bpTreeHash(which: i32): u64 {
    return treeHashOf(broadPhase.trees[which]);
}
