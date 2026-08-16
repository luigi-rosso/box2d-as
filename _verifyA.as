// Phase A verification entry: exports bit-level drivers for the node harness
// to compare against the C reference sweeps in verify/vectors/. Inputs and
// outputs travel as raw f32 bit patterns so the comparison is bit-exact.

import {
    b2Vec2, b2Rot, b2Transform, b2Mat22, b2AABB, b2Plane,
    b2MinFloat, b2MaxFloat, b2AbsFloat, b2ClampFloat, b2IsValidFloat, b2IsValidVec2, b2IsValidRotation,
    b2IsValidTransform, b2IsValidPlane, b2UnwindAngle, b2Atan2, b2ComputeCosSin, b2SpringDamper,
    b2Add, b2Sub, b2Neg, b2Lerp, b2Mul, b2MulSV, b2MulAdd, b2MulSub, b2Abs, b2Min, b2Max, b2Clamp,
    b2Normalize, b2GetLengthAndNormalize, b2CrossVS, b2CrossSV, b2LeftPerp, b2RightPerp,
    b2Dot, b2Cross, b2Length, b2Distance, b2LengthSquared, b2DistanceSquared, b2IsNormalized,
    b2MakeRot, b2NormalizeRot, b2IntegrateRotation, b2MulRot, b2InvMulRot, b2InvertRot,
    b2MakeRotFromUnitVector, b2ComputeRotationBetweenUnitVectors, b2NLerp, b2Rot_GetAngle,
    b2RelativeAngle, b2ComputeAngularVelocity, b2IsNormalizedRot, b2Rot_GetXAxis, b2Rot_GetYAxis,
    b2RotateVector, b2InvRotateVector, b2TransformPoint, b2InvTransformPoint, b2MulTransforms,
    b2InvMulTransforms, b2MulMV, b2GetInverse22, b2Solve22,
    b2AABB_Contains, b2AABB_Overlaps, b2AABB_Union, b2AABB_Center, b2AABB_Extents, b2MakeAABB,
    b2PlaneSeparation,
} from "./lib/math_functions";
import { b2IsValidAABB, b2Perimeter, b2EnlargeAABB, b2AABB_RayCast } from "./lib/aabb";
import {
    B2_LINEAR_SLOP, B2_HUGE, B2_AABB_MARGIN, B2_SPECULATIVE_DISTANCE, B2_MAX_ROTATION, B2_TIME_TO_SLEEP,
} from "./lib/constants";
import { b2SetLengthUnitsPerMeter, b2GetLengthUnitsPerMeter } from "./lib/core";
import {
    b2Hull, b2Polygon, b2Circle, b2Capsule, b2Segment, b2RayCastInput, b2CastOutput, makeVecArray,
} from "./lib/collision";
import { b2ComputeHull, b2ValidateHull } from "./lib/hull";
import {
    b2MakePolygon, b2MakeOffsetRoundedPolygon, b2MakeSquare, b2MakeBox, b2MakeRoundedBox, b2MakeOffsetBox,
    b2MakeOffsetRoundedBox, b2TransformPolygon, b2ComputeCircleMass, b2ComputeCapsuleMass,
    b2ComputePolygonMass, b2ComputeCircleAABB, b2ComputeCapsuleAABB, b2ComputePolygonAABB,
    b2ComputeSegmentAABB, b2PointInCircle, b2PointInCapsule, b2RayCastCircle, b2RayCastCapsule,
    b2RayCastSegment, b2RayCastPolygon,
} from "./lib/geometry";
import {
    b2IntArray, b2IntArray_Create, b2IntArray_Reserve, b2IntArray_Resize, b2IntArray_Get, b2IntArray_Push,
    b2IntArray_Pop, b2IntArray_RemoveSwap,
} from "./lib/array";
import { b2IdPool, b2CreateIdPool, b2AllocId, b2FreeId, b2GetIdCount, b2GetIdCapacity } from "./lib/id_pool";
import {
    b2BitSet, b2CreateBitSet, b2SetBitCountAndClear, b2SetBitGrow, b2SetBit, b2ClearBit, b2GetBit,
    b2InPlaceUnion, b2CountSetBits,
} from "./lib/bitset";
import {
    b2ArenaAllocator, b2CreateArenaAllocator, b2AllocateArenaItem, b2FreeArenaItem, b2GrowArena,
} from "./lib/arena_allocator";
import { b2WorldId, b2BodyId, b2ContactId, b2StoreWorldId, b2LoadWorldId, b2StoreBodyId, b2LoadBodyId, b2StoreContactId, b2LoadContactId } from "./lib/id";

function F(bits: u32): f32 {
    return reinterpret<f32>(bits);
}

function B(x: f32): u32 {
    return reinterpret<u32>(x);
}

function pack(x: f32, y: f32): u64 {
    return (<u64>B(x) << 32) | <u64>B(y);
}

let res = new StaticArray<u32>(64);

export function resGet(i: i32): u32 {
    return res[i];
}

function setResAABB(a: b2AABB, at: i32 = 0): void {
    res[at] = B(a.lowerBound.x);
    res[at + 1] = B(a.lowerBound.y);
    res[at + 2] = B(a.upperBound.x);
    res[at + 3] = B(a.upperBound.y);
}

function setResCast(o: b2CastOutput): void {
    res[0] = o.hit ? 1 : 0;
    res[1] = B(o.fraction);
    res[2] = B(o.normal.x);
    res[3] = B(o.normal.y);
    res[4] = B(o.point.x);
    res[5] = B(o.point.y);
}

// ---- scalar families ----

export function mF1(op: i32, a: u32): u32 {
    let x = F(a);
    if (op == 0) return B(b2UnwindAngle(x));
    if (op == 1) return B(b2AbsFloat(x));
    if (op == 2) return b2IsValidFloat(x) ? 1 : 0;
    return 0xdeadbeef;
}

export function mF2(op: i32, a: u32, b: u32): u32 {
    if (op == 0) return B(b2MinFloat(F(a), F(b)));
    if (op == 1) return B(b2MaxFloat(F(a), F(b)));
    return 0xdeadbeef;
}

export function mF3(a: u32, lo: u32, hi: u32): u32 {
    return B(b2ClampFloat(F(a), F(lo), F(hi)));
}

export function mAtan2(y: u32, x: u32): u32 {
    return B(b2Atan2(F(y), F(x)));
}

export function mCosSin(a: u32): u64 {
    let cs = b2ComputeCosSin(F(a));
    return pack(cs.cosine, cs.sine);
}

export function mSpring(h: u32, d: u32, p: u32, v: u32, t: u32): u32 {
    return B(b2SpringDamper(F(h), F(d), F(p), F(v), F(t)));
}

// ---- vec families ----

export function mV(op: i32, ax: u32, ay: u32, bx: u32, by: u32, s: u32): u64 {
    let a = new b2Vec2(F(ax), F(ay));
    let b = new b2Vec2(F(bx), F(by));
    let sf = F(s);
    let o: b2Vec2;
    if (op == 0) o = b2Add(a, b);
    else if (op == 1) o = b2Sub(a, b);
    else if (op == 2) o = b2Neg(a);
    else if (op == 3) o = b2Lerp(a, b, sf);
    else if (op == 4) o = b2Mul(a, b);
    else if (op == 5) o = b2MulSV(sf, a);
    else if (op == 6) o = b2MulAdd(a, sf, b);
    else if (op == 7) o = b2MulSub(a, sf, b);
    else if (op == 8) o = b2Abs(a);
    else if (op == 9) o = b2Min(a, b);
    else if (op == 10) o = b2Max(a, b);
    else if (op == 11) o = b2Normalize(a);
    else if (op == 12) o = b2CrossVS(a, sf);
    else if (op == 13) o = b2CrossSV(sf, a);
    else if (op == 14) o = b2LeftPerp(a);
    else if (op == 15) o = b2RightPerp(a);
    else if (op == 16) o = b2GetLengthAndNormalize(a).normal;
    else o = new b2Vec2(F(0xdeadbeef), F(0xdeadbeef));
    return pack(o.x, o.y);
}

export function mVClamp(ax: u32, ay: u32, bx: u32, by: u32, cx: u32, cy: u32): u64 {
    let o = b2Clamp(new b2Vec2(F(ax), F(ay)), new b2Vec2(F(bx), F(by)), new b2Vec2(F(cx), F(cy)));
    return pack(o.x, o.y);
}

export function mVF(op: i32, ax: u32, ay: u32, bx: u32, by: u32): u32 {
    let a = new b2Vec2(F(ax), F(ay));
    let b = new b2Vec2(F(bx), F(by));
    if (op == 0) return B(b2Dot(a, b));
    if (op == 1) return B(b2Cross(a, b));
    if (op == 2) return B(b2Length(a));
    if (op == 3) return B(b2Distance(a, b));
    if (op == 4) return B(b2LengthSquared(a));
    if (op == 5) return B(b2DistanceSquared(a, b));
    if (op == 6) return b2IsNormalized(a) ? 1 : 0;
    if (op == 7) return B(b2GetLengthAndNormalize(a).length);
    if (op == 8) return b2IsValidVec2(a) ? 1 : 0;
    return 0xdeadbeef;
}

// ---- rot families ----

export function mR(op: i32, a: u32, b: u32, c: u32, d: u32): u64 {
    let q = new b2Rot(F(a), F(b));
    let r = new b2Rot(F(c), F(d));
    let o: b2Rot;
    if (op == 0) o = b2MakeRot(F(a));
    else if (op == 1) o = b2NormalizeRot(q);
    else if (op == 2) o = b2IntegrateRotation(q, F(c));
    else if (op == 3) o = b2MulRot(q, r);
    else if (op == 4) o = b2InvMulRot(q, r);
    else if (op == 5) o = b2InvertRot(q);
    else if (op == 6) o = b2MakeRotFromUnitVector(new b2Vec2(F(a), F(b)));
    else if (op == 7) o = b2ComputeRotationBetweenUnitVectors(new b2Vec2(F(a), F(b)), new b2Vec2(F(c), F(d)));
    else o = new b2Rot(F(0xdeadbeef), F(0xdeadbeef));
    return pack(o.c, o.s);
}

export function mNLerp(a: u32, b: u32, c: u32, d: u32, t: u32): u64 {
    let o = b2NLerp(new b2Rot(F(a), F(b)), new b2Rot(F(c), F(d)), F(t));
    return pack(o.c, o.s);
}

export function mRF(op: i32, a: u32, b: u32, c: u32, d: u32, e: u32): u32 {
    let q = new b2Rot(F(a), F(b));
    let r = new b2Rot(F(c), F(d));
    if (op == 0) return B(b2Rot_GetAngle(q));
    if (op == 1) return B(b2RelativeAngle(q, r));
    if (op == 2) return B(b2ComputeAngularVelocity(q, r, F(e)));
    if (op == 3) return b2IsNormalizedRot(q) ? 1 : 0;
    if (op == 4) return b2IsValidRotation(q) ? 1 : 0;
    return 0xdeadbeef;
}

export function mRV(op: i32, qc: u32, qs: u32, vx: u32, vy: u32): u64 {
    let q = new b2Rot(F(qc), F(qs));
    let v = new b2Vec2(F(vx), F(vy));
    let o: b2Vec2;
    if (op == 0) o = b2RotateVector(q, v);
    else if (op == 1) o = b2InvRotateVector(q, v);
    else if (op == 2) o = b2Rot_GetXAxis(q);
    else if (op == 3) o = b2Rot_GetYAxis(q);
    else o = new b2Vec2(F(0xdeadbeef), F(0xdeadbeef));
    return pack(o.x, o.y);
}

// ---- transform families ----

function xfOf(px: u32, py: u32, qc: u32, qs: u32): b2Transform {
    return new b2Transform(new b2Vec2(F(px), F(py)), new b2Rot(F(qc), F(qs)));
}

export function mTP(op: i32, px: u32, py: u32, qc: u32, qs: u32, vx: u32, vy: u32): u64 {
    let t = xfOf(px, py, qc, qs);
    let v = new b2Vec2(F(vx), F(vy));
    let o = op == 0 ? b2TransformPoint(t, v) : b2InvTransformPoint(t, v);
    return pack(o.x, o.y);
}

export function mTT(op: i32, apx: u32, apy: u32, aqc: u32, aqs: u32, bpx: u32, bpy: u32, bqc: u32, bqs: u32): void {
    let A = xfOf(apx, apy, aqc, aqs);
    let Bx = xfOf(bpx, bpy, bqc, bqs);
    let C = op == 0 ? b2MulTransforms(A, Bx) : b2InvMulTransforms(A, Bx);
    res[0] = B(C.p.x);
    res[1] = B(C.p.y);
    res[2] = B(C.q.c);
    res[3] = B(C.q.s);
}

export function mTValid(px: u32, py: u32, qc: u32, qs: u32): u32 {
    return b2IsValidTransform(xfOf(px, py, qc, qs)) ? 1 : 0;
}

// ---- mat22 families ----

function matOf(a: u32, b: u32, c: u32, d: u32): b2Mat22 {
    // a = cx.x, b = cx.y, c = cy.x, d = cy.y
    return new b2Mat22(new b2Vec2(F(a), F(b)), new b2Vec2(F(c), F(d)));
}

export function mM(op: i32, a: u32, b: u32, c: u32, d: u32, vx: u32, vy: u32): u64 {
    let A = matOf(a, b, c, d);
    let v = new b2Vec2(F(vx), F(vy));
    let o = op == 0 ? b2MulMV(A, v) : b2Solve22(A, v);
    return pack(o.x, o.y);
}

export function mMInv(a: u32, b: u32, c: u32, d: u32): void {
    let O = b2GetInverse22(matOf(a, b, c, d));
    res[0] = B(O.cx.x);
    res[1] = B(O.cx.y);
    res[2] = B(O.cy.x);
    res[3] = B(O.cy.y);
}

// ---- aabb families ----

function aabbOf(lx: u32, ly: u32, ux: u32, uy: u32): b2AABB {
    return new b2AABB(new b2Vec2(F(lx), F(ly)), new b2Vec2(F(ux), F(uy)));
}

export function mAB2(op: i32, a0: u32, a1: u32, a2: u32, a3: u32, b0: u32, b1: u32, b2v: u32, b3: u32): u32 {
    let a = aabbOf(a0, a1, a2, a3);
    let b = aabbOf(b0, b1, b2v, b3);
    if (op == 0) return b2AABB_Contains(a, b) ? 1 : 0;
    if (op == 1) return b2AABB_Overlaps(a, b) ? 1 : 0;
    return 0xdeadbeef;
}

export function mABUnion(a0: u32, a1: u32, a2: u32, a3: u32, b0: u32, b1: u32, b2v: u32, b3: u32): void {
    setResAABB(b2AABB_Union(aabbOf(a0, a1, a2, a3), aabbOf(b0, b1, b2v, b3)));
}

export function mAB1(op: i32, a0: u32, a1: u32, a2: u32, a3: u32): u64 {
    let a = aabbOf(a0, a1, a2, a3);
    if (op == 0) {
        let c = b2AABB_Center(a);
        return pack(c.x, c.y);
    }
    if (op == 1) {
        let e = b2AABB_Extents(a);
        return pack(e.x, e.y);
    }
    if (op == 2) return b2IsValidAABB(a) ? 1 : 0;
    if (op == 3) return <u64>B(b2Perimeter(a));
    return 0xdeadbeef;
}

export function mEnlarge(a0: u32, a1: u32, a2: u32, a3: u32, b0: u32, b1: u32, b2v: u32, b3: u32): u32 {
    let a = aabbOf(a0, a1, a2, a3);
    let changed = b2EnlargeAABB(a, aabbOf(b0, b1, b2v, b3));
    setResAABB(a);
    return changed ? 1 : 0;
}

export function mAabbRayCast(a0: u32, a1: u32, a2: u32, a3: u32, p1x: u32, p1y: u32, p2x: u32, p2y: u32): void {
    let o = b2AABB_RayCast(aabbOf(a0, a1, a2, a3), new b2Vec2(F(p1x), F(p1y)), new b2Vec2(F(p2x), F(p2y)));
    setResCast(o);
}

// ---- plane families ----

export function mPlane(nx: u32, ny: u32, off: u32, px: u32, py: u32): u32 {
    return B(b2PlaneSeparation(new b2Plane(new b2Vec2(F(nx), F(ny)), F(off)), new b2Vec2(F(px), F(py))));
}

export function mPlaneValid(nx: u32, ny: u32, off: u32): u32 {
    return b2IsValidPlane(new b2Plane(new b2Vec2(F(nx), F(ny)), F(off))) ? 1 : 0;
}

// ---- constants ----

export function kConst(op: i32): u32 {
    if (op == 0) return B(B2_LINEAR_SLOP());
    if (op == 1) return B(B2_HUGE());
    if (op == 2) return B(B2_AABB_MARGIN());
    if (op == 3) return B(B2_SPECULATIVE_DISTANCE());
    if (op == 4) return B(B2_MAX_ROTATION);
    if (op == 5) return B(B2_TIME_TO_SLEEP);
    return 0xdeadbeef;
}

export function kSetLUPM(bits: u32): void {
    b2SetLengthUnitsPerMeter(F(bits));
}

export function kGetLUPM(): u32 {
    return B(b2GetLengthUnitsPerMeter());
}

// ---- ids ----

export function idWorldStore(idx: i32, gen: i32): u32 {
    return b2StoreWorldId(new b2WorldId(<u16>idx, <u16>gen));
}

export function idWorldLoad(x: u32): void {
    let id = b2LoadWorldId(x);
    res[0] = id.index1;
    res[1] = id.generation;
}

export function idBodyStore(index1: i32, world0: i32, gen: i32): u64 {
    return b2StoreBodyId(new b2BodyId(index1, <u16>world0, <u16>gen));
}

export function idBodyLoad(x: u64): void {
    let id = b2LoadBodyId(x);
    res[0] = <u32>id.index1;
    res[1] = id.world0;
    res[2] = id.generation;
}

let contactValues = new Uint32Array(3);

export function idContactStore(index1: i32, world0: i32, gen: u32): void {
    b2StoreContactId(new b2ContactId(index1, <u16>world0, gen), contactValues);
    res[0] = contactValues[0];
    res[1] = contactValues[1];
    res[2] = contactValues[2];
}

export function idContactLoad(v0: u32, v1: u32, v2: u32): void {
    contactValues[0] = v0;
    contactValues[1] = v1;
    contactValues[2] = v2;
    let id = b2LoadContactId(contactValues);
    res[0] = <u32>id.index1;
    res[1] = id.world0;
    res[2] = id.generation;
}

// ---- staged points, hull, polygon ----

let stagedPts = makeVecArray(16);
let curHull = new b2Hull();
let curPoly = new b2Polygon();

export function setPt(i: i32, xb: u32, yb: u32): void {
    stagedPts[i] = new b2Vec2(F(xb), F(yb));
}

export function mMakeAABB(count: i32, radius: u32): void {
    setResAABB(b2MakeAABB(stagedPts, count, F(radius)));
}

export function gComputeHull(count: i32): i32 {
    curHull = b2ComputeHull(stagedPts, count);
    return curHull.count;
}

export function hullGet(i: i32): u32 {
    let p = curHull.points[i >> 1];
    return (i & 1) == 0 ? B(p.x) : B(p.y);
}

export function gValidateCurHull(): i32 {
    return b2ValidateHull(curHull) ? 1 : 0;
}

// Stages an arbitrary hull (bypassing compute) for b2ValidateHull sweeps.
export function gStageHull(count: i32): i32 {
    let h = new b2Hull();
    h.count = count;
    for (let i = 0; i < count; i++) {
        h.points[i] = stagedPts[i].clone();
    }
    curHull = h;
    return b2ValidateHull(curHull) ? 1 : 0;
}

function setCurPoly(p: b2Polygon): void {
    curPoly = p;
}

// Flattened polygon accessor: 0 count, 1 radius, 2..3 centroid, 4..19 verts,
// 20..35 normals.
export function polyGet(i: i32): u32 {
    if (i == 0) return <u32>curPoly.count;
    if (i == 1) return B(curPoly.radius);
    if (i == 2) return B(curPoly.centroid.x);
    if (i == 3) return B(curPoly.centroid.y);
    if (i < 20) {
        let k = i - 4;
        let v = curPoly.vertices[k >> 1];
        return (k & 1) == 0 ? B(v.x) : B(v.y);
    }
    let k = i - 20;
    let n = curPoly.normals[k >> 1];
    return (k & 1) == 0 ? B(n.x) : B(n.y);
}

export function gMakePolygon(radius: u32): void {
    setCurPoly(b2MakePolygon(curHull, F(radius)));
}

export function gMakeOffsetRoundedPolygon(cx: u32, cy: u32, qc: u32, qs: u32, radius: u32): void {
    setCurPoly(b2MakeOffsetRoundedPolygon(curHull, new b2Vec2(F(cx), F(cy)), new b2Rot(F(qc), F(qs)), F(radius)));
}

export function gMakeSquare(h: u32): void {
    setCurPoly(b2MakeSquare(F(h)));
}

export function gMakeBox(hw: u32, hh: u32): void {
    setCurPoly(b2MakeBox(F(hw), F(hh)));
}

export function gMakeRoundedBox(hw: u32, hh: u32, r: u32): void {
    setCurPoly(b2MakeRoundedBox(F(hw), F(hh), F(r)));
}

export function gMakeOffsetBox(hw: u32, hh: u32, cx: u32, cy: u32, qc: u32, qs: u32): void {
    setCurPoly(b2MakeOffsetBox(F(hw), F(hh), new b2Vec2(F(cx), F(cy)), new b2Rot(F(qc), F(qs))));
}

export function gMakeOffsetRoundedBox(hw: u32, hh: u32, cx: u32, cy: u32, qc: u32, qs: u32, r: u32): void {
    setCurPoly(b2MakeOffsetRoundedBox(F(hw), F(hh), new b2Vec2(F(cx), F(cy)), new b2Rot(F(qc), F(qs)), F(r)));
}

export function gTransformPolygon(px: u32, py: u32, qc: u32, qs: u32): void {
    setCurPoly(b2TransformPolygon(xfOf(px, py, qc, qs), curPoly));
}

// ---- mass ----

export function gCircleMass(cx: u32, cy: u32, r: u32, density: u32): void {
    let m = b2ComputeCircleMass(new b2Circle(new b2Vec2(F(cx), F(cy)), F(r)), F(density));
    res[0] = B(m.mass);
    res[1] = B(m.center.x);
    res[2] = B(m.center.y);
    res[3] = B(m.rotationalInertia);
}

export function gCapsuleMass(c1x: u32, c1y: u32, c2x: u32, c2y: u32, r: u32, density: u32): void {
    let m = b2ComputeCapsuleMass(
        new b2Capsule(new b2Vec2(F(c1x), F(c1y)), new b2Vec2(F(c2x), F(c2y)), F(r)), F(density));
    res[0] = B(m.mass);
    res[1] = B(m.center.x);
    res[2] = B(m.center.y);
    res[3] = B(m.rotationalInertia);
}

export function gPolygonMass(density: u32): void {
    let m = b2ComputePolygonMass(curPoly, F(density));
    res[0] = B(m.mass);
    res[1] = B(m.center.x);
    res[2] = B(m.center.y);
    res[3] = B(m.rotationalInertia);
}

// ---- shape aabbs ----

export function gCircleAABB(cx: u32, cy: u32, r: u32, px: u32, py: u32, qc: u32, qs: u32): void {
    setResAABB(b2ComputeCircleAABB(new b2Circle(new b2Vec2(F(cx), F(cy)), F(r)), xfOf(px, py, qc, qs)));
}

export function gCapsuleAABB(c1x: u32, c1y: u32, c2x: u32, c2y: u32, r: u32,
    px: u32, py: u32, qc: u32, qs: u32): void {
    setResAABB(b2ComputeCapsuleAABB(
        new b2Capsule(new b2Vec2(F(c1x), F(c1y)), new b2Vec2(F(c2x), F(c2y)), F(r)), xfOf(px, py, qc, qs)));
}

export function gSegmentAABB(p1x: u32, p1y: u32, p2x: u32, p2y: u32, px: u32, py: u32, qc: u32, qs: u32): void {
    setResAABB(b2ComputeSegmentAABB(
        new b2Segment(new b2Vec2(F(p1x), F(p1y)), new b2Vec2(F(p2x), F(p2y))), xfOf(px, py, qc, qs)));
}

export function gPolygonAABB(px: u32, py: u32, qc: u32, qs: u32): void {
    setResAABB(b2ComputePolygonAABB(curPoly, xfOf(px, py, qc, qs)));
}

// ---- point tests ----

export function gPointInCircle(cx: u32, cy: u32, r: u32, px: u32, py: u32): u32 {
    return b2PointInCircle(new b2Circle(new b2Vec2(F(cx), F(cy)), F(r)), new b2Vec2(F(px), F(py))) ? 1 : 0;
}

export function gPointInCapsule(c1x: u32, c1y: u32, c2x: u32, c2y: u32, r: u32, px: u32, py: u32): u32 {
    return b2PointInCapsule(
        new b2Capsule(new b2Vec2(F(c1x), F(c1y)), new b2Vec2(F(c2x), F(c2y)), F(r)),
        new b2Vec2(F(px), F(py))) ? 1 : 0;
}

// ---- ray casts ----

function rayOf(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32): b2RayCastInput {
    let input = new b2RayCastInput();
    input.origin = new b2Vec2(F(ox), F(oy));
    input.translation = new b2Vec2(F(tx), F(ty));
    input.maxFraction = F(mf);
    return input;
}

export function gRayCastCircle(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32, cx: u32, cy: u32, r: u32): void {
    setResCast(b2RayCastCircle(new b2Circle(new b2Vec2(F(cx), F(cy)), F(r)), rayOf(ox, oy, tx, ty, mf)));
}

export function gRayCastCapsule(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32,
    c1x: u32, c1y: u32, c2x: u32, c2y: u32, r: u32): void {
    setResCast(b2RayCastCapsule(
        new b2Capsule(new b2Vec2(F(c1x), F(c1y)), new b2Vec2(F(c2x), F(c2y)), F(r)),
        rayOf(ox, oy, tx, ty, mf)));
}

export function gRayCastSegment(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32,
    p1x: u32, p1y: u32, p2x: u32, p2y: u32, oneSided: i32): void {
    setResCast(b2RayCastSegment(
        new b2Segment(new b2Vec2(F(p1x), F(p1y)), new b2Vec2(F(p2x), F(p2y))),
        rayOf(ox, oy, tx, ty, mf), oneSided != 0));
}

export function gRayCastPolygon(ox: u32, oy: u32, tx: u32, ty: u32, mf: u32): void {
    setResCast(b2RayCastPolygon(curPoly, rayOf(ox, oy, tx, ty, mf)));
}

// ---- id pool ----

let idPool = new b2IdPool();

export function idpReset(): void {
    idPool = b2CreateIdPool();
}

export function idpAlloc(): i32 {
    return b2AllocId(idPool);
}

export function idpFree(id: i32): void {
    b2FreeId(idPool, id);
}

export function idpCount(): i32 {
    return b2GetIdCount(idPool);
}

export function idpCapacity(): i32 {
    return b2GetIdCapacity(idPool);
}

export function idpFreeCount(): i32 {
    return idPool.freeArray.count;
}

export function idpFreeCap(): i32 {
    return idPool.freeArray.capacity;
}

// ---- int array ----

let intArr = new b2IntArray();

export function iaReset(cap: i32): void {
    intArr = b2IntArray_Create(cap);
}

export function iaPush(v: i32): void {
    b2IntArray_Push(intArr, v);
}

export function iaPop(): i32 {
    return b2IntArray_Pop(intArr);
}

export function iaRemoveSwap(i: i32): i32 {
    return b2IntArray_RemoveSwap(intArr, i);
}

export function iaReserve(n: i32): void {
    b2IntArray_Reserve(intArr, n);
}

export function iaResize(n: i32): void {
    b2IntArray_Resize(intArr, n);
}

export function iaGet(i: i32): i32 {
    return b2IntArray_Get(intArr, i);
}

export function iaCount(): i32 {
    return intArr.count;
}

export function iaCapacity(): i32 {
    return intArr.capacity;
}

// ---- bitset ----

let bitSetA = new b2BitSet();
let bitSetB = new b2BitSet();

function pickSet(which: i32): b2BitSet {
    return which == 0 ? bitSetA : bitSetB;
}

export function bsReset(which: i32, bitCapacity: i32): void {
    if (which == 0) {
        bitSetA = b2CreateBitSet(<u32>bitCapacity);
    } else {
        bitSetB = b2CreateBitSet(<u32>bitCapacity);
    }
}

export function bsSetBitGrow(which: i32, i: i32): void {
    b2SetBitGrow(pickSet(which), <u32>i);
}

export function bsSetBit(which: i32, i: i32): void {
    b2SetBit(pickSet(which), <u32>i);
}

export function bsClearBit(which: i32, i: i32): void {
    b2ClearBit(pickSet(which), <u32>i);
}

export function bsGetBit(which: i32, i: i32): i32 {
    return b2GetBit(pickSet(which), <u32>i) ? 1 : 0;
}

export function bsSetBitCountAndClear(which: i32, n: i32): void {
    b2SetBitCountAndClear(pickSet(which), <u32>n);
}

export function bsUnion(): void {
    b2InPlaceUnion(bitSetA, bitSetB);
}

export function bsCount(which: i32): i32 {
    return b2CountSetBits(pickSet(which));
}

export function bsBlockCount(which: i32): i32 {
    return <i32>pickSet(which).blockCount;
}

export function bsBlockCapacity(which: i32): i32 {
    return <i32>pickSet(which).blockCapacity;
}

// FNV-1a over the live blocks, for cheap whole-state comparison.
export function bsHash(which: i32): u64 {
    let s = pickSet(which);
    let h: u64 = 0xcbf29ce484222325;
    for (let i: u32 = 0; i < s.blockCount; i++) {
        let w = s.bits[i];
        for (let k = 0; k < 8; k++) {
            h ^= (w >> (k * 8)) & 0xff;
            h *= 0x100000001b3;
        }
    }
    return h;
}

// ---- arena ----

let arena = new b2ArenaAllocator();
let arenaPtrs = new StaticArray<usize>(64);
let arenaDepth = 0;

export function arReset(cap: i32): void {
    arena = b2CreateArenaAllocator(cap);
    arenaDepth = 0;
}

// Returns the offset from the arena base, or -1 for a heap-fallback entry.
export function arAlloc(size: i32): i32 {
    let p = b2AllocateArenaItem(arena, size, "verify");
    arenaPtrs[arenaDepth++] = p;
    let entry = arena.entries.get(arena.entries.count - 1);
    if (entry.usedMalloc) {
        return -1;
    }
    return <i32>(p - changetype<usize>(arena.data));
}

export function arFree(): void {
    arenaDepth--;
    b2FreeArenaItem(arena, arenaPtrs[arenaDepth]);
}

export function arGrow(): void {
    b2GrowArena(arena);
}

export function arIndex(): i32 {
    return arena.index;
}

export function arAllocation(): i32 {
    return arena.allocation;
}

export function arMaxAllocation(): i32 {
    return arena.maxAllocation;
}

export function arCapacity(): i32 {
    return arena.capacity;
}

export function arEntryCount(): i32 {
    return arena.entries.count;
}

export function arEntryCapacity(): i32 {
    return arena.entries.capacity;
}
