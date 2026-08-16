// Ported from box2d src/contact_solver.h/.c with BOX2D_DISABLE_SIMD
// (B2_SIMD_NONE, width 4): the wide types are structs of 4 scalar lanes and
// every wide op runs per lane in C's order. The constraint arrays are
// allocated in solver.c from the arena; solver.as calls the alloc helpers
// below and assigns them to the graph colors and step context.

import { B2_NULL_INDEX } from "./core";
import {
    b2Vec2,
    b2Vec2_zero,
    b2Add,
    b2Sub,
    b2Dot,
    b2Cross,
    b2CrossSV,
    b2MulSV,
    b2MulAdd,
    b2MulSub,
    b2RightPerp,
    b2RotateVector,
    b2MaxFloat,
    b2MinFloat,
    b2ClampFloat,
} from "./math_functions";
import { b2Manifold } from "./collision";
import { b2StepContext, b2Softness, b2MakeSoft } from "./solver";
import { b2BodyState, b2_dynamicFlag } from "./body";
import { B2_OVERFLOW_INDEX } from "./constraint_graph";
import { b2_awakeSet } from "./physics_world";

export class b2ContactConstraintPoint {
    anchorA: b2Vec2 = new b2Vec2();
    anchorB: b2Vec2 = new b2Vec2();
    baseSeparation: f32 = 0;
    relativeVelocity: f32 = 0;
    normalImpulse: f32 = 0;
    tangentImpulse: f32 = 0;
    totalNormalImpulse: f32 = 0;
    normalMass: f32 = 0;
    tangentMass: f32 = 0;
}

function makeConstraintPoints(): StaticArray<b2ContactConstraintPoint> {
    let points = new StaticArray<b2ContactConstraintPoint>(2);
    for (let i = 0; i < 2; ++i) {
        points[i] = new b2ContactConstraintPoint();
    }
    return points;
}

export class b2ContactConstraint {
    indexA: i32 = 0;
    indexB: i32 = 0;
    points: StaticArray<b2ContactConstraintPoint> = makeConstraintPoints();
    normal: b2Vec2 = new b2Vec2();
    invMassA: f32 = 0;
    invMassB: f32 = 0;
    invIA: f32 = 0;
    invIB: f32 = 0;
    friction: f32 = 0;
    restitution: f32 = 0;
    tangentSpeed: f32 = 0;
    rollingResistance: f32 = 0;
    rollingMass: f32 = 0;
    rollingImpulse: f32 = 0;
    softness: b2Softness = new b2Softness();
    pointCount: i32 = 0;
}

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

export function b2PrepareOverflowContacts(context: b2StepContext): void {
    let world = context.world!;
    let graph = context.graph!;
    let color = graph.colors[B2_OVERFLOW_INDEX];
    let constraints = color.overflowConstraints!;
    let contactCount = color.contactSims.count;
    let contacts = color.contactSims.data;
    let awakeStates = context.states;

    // Stiffer for static contacts to avoid bodies getting pushed through the ground
    let contactSoftness = context.contactSoftness;
    let staticSoftness = context.staticSoftness;

    let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;

    for (let i = 0; i < contactCount; ++i) {
        let contactSim = unchecked(contacts[i])!;

        let manifold = contactSim.manifold;
        let pointCount = manifold.pointCount;

        assert(0 < pointCount && pointCount <= 2);

        let indexA = contactSim.bodySimIndexA;
        let indexB = contactSim.bodySimIndexB;

        let constraint = unchecked(constraints[i]);
        constraint.indexA = indexA;
        constraint.indexB = indexB;
        constraint.normal = manifold.normal.clone();
        constraint.friction = contactSim.friction;
        constraint.restitution = contactSim.restitution;
        constraint.rollingResistance = contactSim.rollingResistance;
        constraint.rollingImpulse = warmStartScale * manifold.rollingImpulse;
        constraint.tangentSpeed = contactSim.tangentSpeed;
        constraint.pointCount = pointCount;

        let vA = b2Vec2_zero();
        let wA: f32 = 0.0;
        let mA = contactSim.invMassA;
        let iA = contactSim.invIA;
        if (indexA != B2_NULL_INDEX) {
            let stateA = unchecked(awakeStates[indexA])!;
            vA = stateA.linearVelocity;
            wA = stateA.angularVelocity;
        }

        let vB = b2Vec2_zero();
        let wB: f32 = 0.0;
        let mB = contactSim.invMassB;
        let iB = contactSim.invIB;
        if (indexB != B2_NULL_INDEX) {
            let stateB = unchecked(awakeStates[indexB])!;
            vB = stateB.linearVelocity;
            wB = stateB.angularVelocity;
        }

        if (indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX) {
            constraint.softness = staticSoftness.clone();
        } else {
            constraint.softness = contactSoftness.clone();
        }

        // copy mass into constraint to avoid cache misses during sub-stepping
        constraint.invMassA = mA;
        constraint.invIA = iA;
        constraint.invMassB = mB;
        constraint.invIB = iB;

        {
            let k = iA + iB;
            constraint.rollingMass = k > 0.0 ? <f32>1.0 / k : <f32>0.0;
        }

        let normal = constraint.normal;
        let tangent = b2RightPerp(constraint.normal);

        for (let j = 0; j < pointCount; ++j) {
            let mp = unchecked(manifold.points[j]);
            let cp = unchecked(constraint.points[j]);

            cp.normalImpulse = warmStartScale * mp.normalImpulse;
            cp.tangentImpulse = warmStartScale * mp.tangentImpulse;
            cp.totalNormalImpulse = 0.0;

            let rA = mp.anchorA;
            let rB = mp.anchorB;

            cp.anchorA = rA.clone();
            cp.anchorB = rB.clone();
            cp.baseSeparation = mp.separation - b2Dot(b2Sub(rB, rA), normal);

            let rnA = b2Cross(rA, normal);
            let rnB = b2Cross(rB, normal);
            let kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
            cp.normalMass = kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0;

            let rtA = b2Cross(rA, tangent);
            let rtB = b2Cross(rB, tangent);
            let kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
            cp.tangentMass = kTangent > 0.0 ? <f32>1.0 / kTangent : <f32>0.0;

            // Save relative velocity for restitution
            let vrA = b2Add(vA, b2CrossSV(wA, rA));
            let vrB = b2Add(vB, b2CrossSV(wB, rB));
            cp.relativeVelocity = b2Dot(normal, b2Sub(vrB, vrA));
        }
    }
}

export function b2WarmStartOverflowContacts(context: b2StepContext): void {
    let graph = context.graph!;
    let color = graph.colors[B2_OVERFLOW_INDEX];
    let constraints = color.overflowConstraints!;
    let contactCount = color.contactSims.count;
    let world = context.world!;
    let awakeSet = world.solverSets.get(b2_awakeSet);
    let states = awakeSet.bodyStates.data;

    // This is a dummy state to represent a static body because static bodies don't have a solver body.
    let dummyState = new b2BodyState();

    for (let i = 0; i < contactCount; ++i) {
        let constraint = unchecked(constraints[i]);

        let indexA = constraint.indexA;
        let indexB = constraint.indexB;

        let stateA = indexA == B2_NULL_INDEX ? dummyState : unchecked(states[indexA])!;
        let stateB = indexB == B2_NULL_INDEX ? dummyState : unchecked(states[indexB])!;

        let vA = stateA.linearVelocity;
        let wA = stateA.angularVelocity;
        let vB = stateB.linearVelocity;
        let wB = stateB.angularVelocity;

        let mA = constraint.invMassA;
        let iA = constraint.invIA;
        let mB = constraint.invMassB;
        let iB = constraint.invIB;

        let normal = constraint.normal;
        let tangent = b2RightPerp(constraint.normal);
        let pointCount = constraint.pointCount;

        for (let j = 0; j < pointCount; ++j) {
            let cp = unchecked(constraint.points[j]);

            // fixed anchors
            let rA = cp.anchorA;
            let rB = cp.anchorB;

            let P = b2Add(b2MulSV(cp.normalImpulse, normal), b2MulSV(cp.tangentImpulse, tangent));

            cp.totalNormalImpulse += cp.normalImpulse;

            wA -= iA * b2Cross(rA, P);
            vA = b2MulAdd(vA, -mA, P);
            wB += iB * b2Cross(rB, P);
            vB = b2MulAdd(vB, mB, P);
        }

        wA -= iA * constraint.rollingImpulse;
        wB += iB * constraint.rollingImpulse;

        if ((stateA.flags & b2_dynamicFlag) != 0) {
            stateA.linearVelocity = vA;
            stateA.angularVelocity = wA;
        }

        if ((stateB.flags & b2_dynamicFlag) != 0) {
            stateB.linearVelocity = vB;
            stateB.angularVelocity = wB;
        }
    }
}

export function b2SolveOverflowContacts(context: b2StepContext, useBias: bool): void {
    let graph = context.graph!;
    let color = graph.colors[B2_OVERFLOW_INDEX];
    let constraints = color.overflowConstraints!;
    let contactCount = color.contactSims.count;
    let world = context.world!;
    let awakeSet = world.solverSets.get(b2_awakeSet);
    let states = awakeSet.bodyStates.data;

    let inv_h = context.inv_h;
    let contactSpeed = world.contactSpeed;

    // This is a dummy body to represent a static body since static bodies don't have a solver body.
    let dummyState = new b2BodyState();

    for (let i = 0; i < contactCount; ++i) {
        let constraint = unchecked(constraints[i]);
        let mA = constraint.invMassA;
        let iA = constraint.invIA;
        let mB = constraint.invMassB;
        let iB = constraint.invIB;

        let stateA = constraint.indexA == B2_NULL_INDEX ? dummyState : unchecked(states[constraint.indexA])!;
        let vA = stateA.linearVelocity;
        let wA = stateA.angularVelocity;
        let dqA = stateA.deltaRotation;

        let stateB = constraint.indexB == B2_NULL_INDEX ? dummyState : unchecked(states[constraint.indexB])!;
        let vB = stateB.linearVelocity;
        let wB = stateB.angularVelocity;
        let dqB = stateB.deltaRotation;

        let dp = b2Sub(stateB.deltaPosition, stateA.deltaPosition);

        let normal = constraint.normal;
        let tangent = b2RightPerp(normal);
        let friction = constraint.friction;
        let softness = constraint.softness;

        let pointCount = constraint.pointCount;
        let totalNormalImpulse: f32 = 0.0;

        // Non-penetration
        for (let j = 0; j < pointCount; ++j) {
            let cp = unchecked(constraint.points[j]);

            // fixed anchor points
            let rA = cp.anchorA;
            let rB = cp.anchorB;

            // compute current separation
            // this is subject to round-off error if the anchor is far from the body center of mass
            let ds = b2Add(dp, b2Sub(b2RotateVector(dqB, rB), b2RotateVector(dqA, rA)));
            let s = cp.baseSeparation + b2Dot(ds, normal);

            let velocityBias: f32 = 0.0;
            let massScale: f32 = 1.0;
            let impulseScale: f32 = 0.0;
            if (s > 0.0) {
                // speculative bias
                velocityBias = s * inv_h;
            } else if (useBias) {
                velocityBias = b2MaxFloat(softness.massScale * softness.biasRate * s, -contactSpeed);
                massScale = softness.massScale;
                impulseScale = softness.impulseScale;
            }

            // relative normal velocity at contact
            let vrA = b2Add(vA, b2CrossSV(wA, rA));
            let vrB = b2Add(vB, b2CrossSV(wB, rB));
            let vn = b2Dot(b2Sub(vrB, vrA), normal);

            // incremental normal impulse
            let impulse = -cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse;

            // clamp the accumulated impulse
            let newImpulse = b2MaxFloat(cp.normalImpulse + impulse, 0.0);
            impulse = newImpulse - cp.normalImpulse;
            cp.normalImpulse = newImpulse;
            cp.totalNormalImpulse += impulse;

            totalNormalImpulse += newImpulse;

            // apply normal impulse
            let P = b2MulSV(impulse, normal);
            vA = b2MulSub(vA, mA, P);
            wA -= iA * b2Cross(rA, P);

            vB = b2MulAdd(vB, mB, P);
            wB += iB * b2Cross(rB, P);
        }

        // Friction
        for (let j = 0; j < pointCount; ++j) {
            let cp = unchecked(constraint.points[j]);

            // fixed anchor points
            let rA = cp.anchorA;
            let rB = cp.anchorB;

            // relative tangent velocity at contact
            let vrB = b2Add(vB, b2CrossSV(wB, rB));
            let vrA = b2Add(vA, b2CrossSV(wA, rA));

            // vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
            //    = dot(vrB - vrA, tangent) - (sA + sB)
            let vt = b2Dot(b2Sub(vrB, vrA), tangent) - constraint.tangentSpeed;

            // incremental tangent impulse
            let impulse = cp.tangentMass * (-vt);

            // clamp the accumulated force
            let maxFriction = friction * cp.normalImpulse;
            let newImpulse = b2ClampFloat(cp.tangentImpulse + impulse, -maxFriction, maxFriction);
            impulse = newImpulse - cp.tangentImpulse;
            cp.tangentImpulse = newImpulse;

            // apply tangent impulse
            let P = b2MulSV(impulse, tangent);
            vA = b2MulSub(vA, mA, P);
            wA -= iA * b2Cross(rA, P);
            vB = b2MulAdd(vB, mB, P);
            wB += iB * b2Cross(rB, P);
        }

        // Rolling resistance
        {
            let deltaLambda = -constraint.rollingMass * (wB - wA);
            let lambda = constraint.rollingImpulse;
            let maxLambda = constraint.rollingResistance * totalNormalImpulse;
            constraint.rollingImpulse = b2ClampFloat(lambda + deltaLambda, -maxLambda, maxLambda);
            deltaLambda = constraint.rollingImpulse - lambda;

            wA -= iA * deltaLambda;
            wB += iB * deltaLambda;
        }

        if ((stateA.flags & b2_dynamicFlag) != 0) {
            stateA.linearVelocity = vA;
            stateA.angularVelocity = wA;
        }

        if ((stateB.flags & b2_dynamicFlag) != 0) {
            stateB.linearVelocity = vB;
            stateB.angularVelocity = wB;
        }
    }
}

export function b2ApplyOverflowRestitution(context: b2StepContext): void {
    let graph = context.graph!;
    let color = graph.colors[B2_OVERFLOW_INDEX];
    let constraints = color.overflowConstraints!;
    let contactCount = color.contactSims.count;
    let world = context.world!;
    let awakeSet = world.solverSets.get(b2_awakeSet);
    let states = awakeSet.bodyStates.data;

    let threshold = world.restitutionThreshold;

    // dummy state to represent a static body
    let dummyState = new b2BodyState();

    for (let i = 0; i < contactCount; ++i) {
        let constraint = unchecked(constraints[i]);

        let restitution = constraint.restitution;
        if (restitution == 0.0) {
            continue;
        }

        let mA = constraint.invMassA;
        let iA = constraint.invIA;
        let mB = constraint.invMassB;
        let iB = constraint.invIB;

        let stateA = constraint.indexA == B2_NULL_INDEX ? dummyState : unchecked(states[constraint.indexA])!;
        let vA = stateA.linearVelocity;
        let wA = stateA.angularVelocity;

        let stateB = constraint.indexB == B2_NULL_INDEX ? dummyState : unchecked(states[constraint.indexB])!;
        let vB = stateB.linearVelocity;
        let wB = stateB.angularVelocity;

        let normal = constraint.normal;
        let pointCount = constraint.pointCount;

        // it is possible to get more accurate restitution by iterating
        // this only makes a difference if there are two contact points
        for (let j = 0; j < pointCount; ++j) {
            let cp = unchecked(constraint.points[j]);

            // if the normal impulse is zero then there was no collision
            // this skips speculative contact points that didn't generate an impulse
            if (cp.relativeVelocity > -threshold || cp.totalNormalImpulse == 0.0) {
                continue;
            }

            // fixed anchor points
            let rA = cp.anchorA;
            let rB = cp.anchorB;

            // relative normal velocity at contact
            let vrB = b2Add(vB, b2CrossSV(wB, rB));
            let vrA = b2Add(vA, b2CrossSV(wA, rA));
            let vn = b2Dot(b2Sub(vrB, vrA), normal);

            // compute normal impulse
            let impulse = -cp.normalMass * (vn + restitution * cp.relativeVelocity);

            // clamp the accumulated impulse
            let newImpulse = b2MaxFloat(cp.normalImpulse + impulse, 0.0);
            impulse = newImpulse - cp.normalImpulse;
            cp.normalImpulse = newImpulse;

            // Add the incremental impulse rather than the full impulse because this is not a sub-step
            cp.totalNormalImpulse += impulse;

            // apply contact impulse
            let P = b2MulSV(impulse, normal);
            vA = b2MulSub(vA, mA, P);
            wA -= iA * b2Cross(rA, P);
            vB = b2MulAdd(vB, mB, P);
            wB += iB * b2Cross(rB, P);
        }

        if ((stateA.flags & b2_dynamicFlag) != 0) {
            stateA.linearVelocity = vA;
            stateA.angularVelocity = wA;
        }

        if ((stateB.flags & b2_dynamicFlag) != 0) {
            stateB.linearVelocity = vB;
            stateB.angularVelocity = wB;
        }
    }
}

export function b2StoreOverflowImpulses(context: b2StepContext): void {
    let graph = context.graph!;
    let color = graph.colors[B2_OVERFLOW_INDEX];
    let constraints = color.overflowConstraints!;
    let contacts = color.contactSims.data;
    let contactCount = color.contactSims.count;

    for (let i = 0; i < contactCount; ++i) {
        let constraint = unchecked(constraints[i]);
        let contact = contacts[i]!;
        let manifold = contact.manifold;
        let pointCount = manifold.pointCount;

        for (let j = 0; j < pointCount; ++j) {
            unchecked(manifold.points[j].normalImpulse = constraint.points[j].normalImpulse);
            unchecked(manifold.points[j].tangentImpulse = constraint.points[j].tangentImpulse);
            unchecked(manifold.points[j].totalNormalImpulse = constraint.points[j].totalNormalImpulse);
            unchecked(manifold.points[j].normalVelocity = constraint.points[j].relativeVelocity);
        }

        manifold.rollingImpulse = constraint.rollingImpulse;
    }
}

// C defines these in core.h; with BOX2D_DISABLE_SIMD the width is 4.
export const B2_SIMD_WIDTH: i32 = 4;
export const B2_SIMD_SHIFT: i32 = 2;

// Wide math. Lanes live in the four f32 fields either way; under
// ASC_FEATURE_SIMD the W ops load the 16 bytes as one v128 and use ops
// whose per-lane semantics match the scalar bodies bit-exactly, masks
// excepted: compares hold all-ones lanes instead of 1.0, and only
// blend/or/allzero ever consume them.
export class b2FloatW {
    x: f32 = 0;
    y: f32 = 0;
    z: f32 = 0;
    w: f32 = 0;
}

// @ts-ignore: decorator
@inline
function wptr(a: b2FloatW): usize {
    return changetype<usize>(a);
}

// Wide vec2
export class b2Vec2W {
    X: b2FloatW = new b2FloatW();
    Y: b2FloatW = new b2FloatW();
}

// Wide rotation
export class b2RotW {
    C: b2FloatW = new b2FloatW();
    S: b2FloatW = new b2FloatW();
}

// C compound literal (b2FloatW){...}
function b2MakeW(x: f32, y: f32, z: f32, w: f32): b2FloatW {
    let r = new b2FloatW();
    r.x = x;
    r.y = y;
    r.z = z;
    r.w = w;
    return r;
}

function b2ZeroW(): b2FloatW {
    return new b2FloatW();
}

function b2SplatW(scalar: f32): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), f32x4.splat(scalar));
        return r;
    } else {
        return b2MakeW(scalar, scalar, scalar, scalar);
    }
}

function b2AddW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), f32x4.add(load<v128>(wptr(a)), load<v128>(wptr(b))));
        return r;
    } else {
        return b2MakeW(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }
}

function b2SubW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), f32x4.sub(load<v128>(wptr(a)), load<v128>(wptr(b))));
        return r;
    } else {
        return b2MakeW(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
    }
}

function b2MulW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), f32x4.mul(load<v128>(wptr(a)), load<v128>(wptr(b))));
        return r;
    } else {
        return b2MakeW(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
    }
}

function b2MulAddW(a: b2FloatW, b: b2FloatW, c: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        // Separate mul then add keeps the scalar rounding; wasm simd has no fma.
        store<v128>(wptr(r),
                    f32x4.add(load<v128>(wptr(a)),
                              f32x4.mul(load<v128>(wptr(b)), load<v128>(wptr(c)))));
        return r;
    } else {
        return b2MakeW(a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w);
    }
}

function b2MulSubW(a: b2FloatW, b: b2FloatW, c: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r),
                    f32x4.sub(load<v128>(wptr(a)),
                              f32x4.mul(load<v128>(wptr(b)), load<v128>(wptr(c)))));
        return r;
    } else {
        return b2MakeW(a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w);
    }
}

// min/max keep the scalar ternaries via compare+bitselect; f32x4.min/max
// diverge from them on NaN and signed-zero lanes.
function b2MinW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        let va = load<v128>(wptr(a));
        let vb = load<v128>(wptr(b));
        store<v128>(wptr(r), v128.bitselect(va, vb, f32x4.le(va, vb)));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = a.x <= b.x ? a.x : b.x;
        r.y = a.y <= b.y ? a.y : b.y;
        r.z = a.z <= b.z ? a.z : b.z;
        r.w = a.w <= b.w ? a.w : b.w;
        return r;
    }
}

function b2MaxW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        let va = load<v128>(wptr(a));
        let vb = load<v128>(wptr(b));
        store<v128>(wptr(r), v128.bitselect(va, vb, f32x4.ge(va, vb)));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = a.x >= b.x ? a.x : b.x;
        r.y = a.y >= b.y ? a.y : b.y;
        r.z = a.z >= b.z ? a.z : b.z;
        r.w = a.w >= b.w ? a.w : b.w;
        return r;
    }
}

// a = clamp(a, -b, b)
function b2SymClampW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        let va = load<v128>(wptr(a));
        let hi = load<v128>(wptr(b));
        let lo = f32x4.neg(hi);
        let t = v128.bitselect(hi, va, f32x4.gt(va, hi));
        store<v128>(wptr(r), v128.bitselect(lo, t, f32x4.lt(va, lo)));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = b2ClampFloat(a.x, -b.x, b.x);
        r.y = b2ClampFloat(a.y, -b.y, b.y);
        r.z = b2ClampFloat(a.z, -b.z, b.z);
        r.w = b2ClampFloat(a.w, -b.w, b.w);
        return r;
    }
}

// Mask ops: wide lanes are all-ones where scalar lanes are 1.0; both read
// as true everywhere masks are consumed.
function b2OrW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), v128.or(load<v128>(wptr(a)), load<v128>(wptr(b))));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = a.x != 0.0 || b.x != 0.0 ? <f32>1.0 : <f32>0.0;
        r.y = a.y != 0.0 || b.y != 0.0 ? <f32>1.0 : <f32>0.0;
        r.z = a.z != 0.0 || b.z != 0.0 ? <f32>1.0 : <f32>0.0;
        r.w = a.w != 0.0 || b.w != 0.0 ? <f32>1.0 : <f32>0.0;
        return r;
    }
}

function b2GreaterThanW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), f32x4.gt(load<v128>(wptr(a)), load<v128>(wptr(b))));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = a.x > b.x ? <f32>1.0 : <f32>0.0;
        r.y = a.y > b.y ? <f32>1.0 : <f32>0.0;
        r.z = a.z > b.z ? <f32>1.0 : <f32>0.0;
        r.w = a.w > b.w ? <f32>1.0 : <f32>0.0;
        return r;
    }
}

function b2EqualsW(a: b2FloatW, b: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r), f32x4.eq(load<v128>(wptr(a)), load<v128>(wptr(b))));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = a.x == b.x ? <f32>1.0 : <f32>0.0;
        r.y = a.y == b.y ? <f32>1.0 : <f32>0.0;
        r.z = a.z == b.z ? <f32>1.0 : <f32>0.0;
        r.w = a.w == b.w ? <f32>1.0 : <f32>0.0;
        return r;
    }
}

function b2AllZeroW(a: b2FloatW): bool {
    if (ASC_FEATURE_SIMD) {
        return i32x4.all_true(f32x4.eq(load<v128>(wptr(a)), f32x4.splat(0)));
    } else {
        return a.x == 0.0 && a.y == 0.0 && a.z == 0.0 && a.w == 0.0;
    }
}

// component-wise returns mask ? b : a
function b2BlendW(a: b2FloatW, b: b2FloatW, mask: b2FloatW): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r),
                    v128.bitselect(load<v128>(wptr(b)),
                                   load<v128>(wptr(a)),
                                   load<v128>(wptr(mask))));
        return r;
    } else {
        let r = new b2FloatW();
        r.x = mask.x != 0.0 ? b.x : a.x;
        r.y = mask.y != 0.0 ? b.y : a.y;
        r.z = mask.z != 0.0 ? b.z : a.z;
        r.w = mask.w != 0.0 ? b.w : a.w;
        return r;
    }
}

function b2DotW(a: b2Vec2W, b: b2Vec2W): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r),
                    f32x4.add(f32x4.mul(load<v128>(wptr(a.X)), load<v128>(wptr(b.X))),
                              f32x4.mul(load<v128>(wptr(a.Y)), load<v128>(wptr(b.Y)))));
        return r;
    } else {
        return b2AddW(b2MulW(a.X, b.X), b2MulW(a.Y, b.Y));
    }
}

function b2CrossW(a: b2Vec2W, b: b2Vec2W): b2FloatW {
    if (ASC_FEATURE_SIMD) {
        let r = new b2FloatW();
        store<v128>(wptr(r),
                    f32x4.sub(f32x4.mul(load<v128>(wptr(a.X)), load<v128>(wptr(b.Y))),
                              f32x4.mul(load<v128>(wptr(a.Y)), load<v128>(wptr(b.X)))));
        return r;
    } else {
        return b2SubW(b2MulW(a.X, b.Y), b2MulW(a.Y, b.X));
    }
}

function b2RotateVectorW(q: b2RotW, v: b2Vec2W): b2Vec2W {
    if (ASC_FEATURE_SIMD) {
        let r = new b2Vec2W();
        let c = load<v128>(wptr(q.C));
        let s = load<v128>(wptr(q.S));
        let vx = load<v128>(wptr(v.X));
        let vy = load<v128>(wptr(v.Y));
        store<v128>(wptr(r.X), f32x4.sub(f32x4.mul(c, vx), f32x4.mul(s, vy)));
        store<v128>(wptr(r.Y), f32x4.add(f32x4.mul(s, vx), f32x4.mul(c, vy)));
        return r;
    } else {
        let r = new b2Vec2W();
        r.X = b2SubW(b2MulW(q.C, v.X), b2MulW(q.S, v.Y));
        r.Y = b2AddW(b2MulW(q.S, v.X), b2MulW(q.C, v.Y));
        return r;
    }
}

// C accesses lanes through float pointer casts.
function b2GetLaneW(a: b2FloatW, lane: i32): f32 {
    switch (lane) {
        case 0:
            return a.x;
        case 1:
            return a.y;
        case 2:
            return a.z;
        default:
            return a.w;
    }
}

function b2SetLaneW(a: b2FloatW, lane: i32, value: f32): void {
    switch (lane) {
        case 0:
            a.x = value;
            break;
        case 1:
            a.y = value;
            break;
        case 2:
            a.z = value;
            break;
        default:
            a.w = value;
            break;
    }
}

// Destructive wide ops for the hot solver tasks: lane-wise writes into
// caller-owned scratch, so dst may alias any operand. Values and op order
// match the pure ops exactly; only the per-op allocation goes away.
function b2CopyW(dst: b2FloatW, a: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst), load<v128>(wptr(a)));
    } else {
        dst.x = a.x;
        dst.y = a.y;
        dst.z = a.z;
        dst.w = a.w;
    }
}

function b2SetZeroW(dst: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst), f32x4.splat(0));
    } else {
        dst.x = 0.0;
        dst.y = 0.0;
        dst.z = 0.0;
        dst.w = 0.0;
    }
}

function b2AddWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst), f32x4.add(load<v128>(wptr(a)), load<v128>(wptr(b))));
    } else {
        dst.x = a.x + b.x;
        dst.y = a.y + b.y;
        dst.z = a.z + b.z;
        dst.w = a.w + b.w;
    }
}

function b2SubWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst), f32x4.sub(load<v128>(wptr(a)), load<v128>(wptr(b))));
    } else {
        dst.x = a.x - b.x;
        dst.y = a.y - b.y;
        dst.z = a.z - b.z;
        dst.w = a.w - b.w;
    }
}

function b2MulWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst), f32x4.mul(load<v128>(wptr(a)), load<v128>(wptr(b))));
    } else {
        dst.x = a.x * b.x;
        dst.y = a.y * b.y;
        dst.z = a.z * b.z;
        dst.w = a.w * b.w;
    }
}

function b2MulAddWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW, c: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst),
                    f32x4.add(load<v128>(wptr(a)),
                              f32x4.mul(load<v128>(wptr(b)), load<v128>(wptr(c)))));
    } else {
        dst.x = a.x + b.x * c.x;
        dst.y = a.y + b.y * c.y;
        dst.z = a.z + b.z * c.z;
        dst.w = a.w + b.w * c.w;
    }
}

function b2MulSubWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW, c: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst),
                    f32x4.sub(load<v128>(wptr(a)),
                              f32x4.mul(load<v128>(wptr(b)), load<v128>(wptr(c)))));
    } else {
        dst.x = a.x - b.x * c.x;
        dst.y = a.y - b.y * c.y;
        dst.z = a.z - b.z * c.z;
        dst.w = a.w - b.w * c.w;
    }
}

function b2MinWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        let va = load<v128>(wptr(a));
        let vb = load<v128>(wptr(b));
        store<v128>(wptr(dst), v128.bitselect(va, vb, f32x4.le(va, vb)));
    } else {
        dst.x = a.x <= b.x ? a.x : b.x;
        dst.y = a.y <= b.y ? a.y : b.y;
        dst.z = a.z <= b.z ? a.z : b.z;
        dst.w = a.w <= b.w ? a.w : b.w;
    }
}

function b2MaxWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        let va = load<v128>(wptr(a));
        let vb = load<v128>(wptr(b));
        store<v128>(wptr(dst), v128.bitselect(va, vb, f32x4.ge(va, vb)));
    } else {
        dst.x = a.x >= b.x ? a.x : b.x;
        dst.y = a.y >= b.y ? a.y : b.y;
        dst.z = a.z >= b.z ? a.z : b.z;
        dst.w = a.w >= b.w ? a.w : b.w;
    }
}

function b2SymClampWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        let va = load<v128>(wptr(a));
        let hi = load<v128>(wptr(b));
        let lo = f32x4.neg(hi);
        let t = v128.bitselect(hi, va, f32x4.gt(va, hi));
        store<v128>(wptr(dst), v128.bitselect(lo, t, f32x4.lt(va, lo)));
    } else {
        dst.x = b2ClampFloat(a.x, -b.x, b.x);
        dst.y = b2ClampFloat(a.y, -b.y, b.y);
        dst.z = b2ClampFloat(a.z, -b.z, b.z);
        dst.w = b2ClampFloat(a.w, -b.w, b.w);
    }
}

function b2GreaterThanWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst), f32x4.gt(load<v128>(wptr(a)), load<v128>(wptr(b))));
    } else {
        dst.x = a.x > b.x ? <f32>1.0 : <f32>0.0;
        dst.y = a.y > b.y ? <f32>1.0 : <f32>0.0;
        dst.z = a.z > b.z ? <f32>1.0 : <f32>0.0;
        dst.w = a.w > b.w ? <f32>1.0 : <f32>0.0;
    }
}

function b2BlendWTo(dst: b2FloatW, a: b2FloatW, b: b2FloatW, mask: b2FloatW): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst),
                    v128.bitselect(load<v128>(wptr(b)),
                                   load<v128>(wptr(a)),
                                   load<v128>(wptr(mask))));
    } else {
        dst.x = mask.x != 0.0 ? b.x : a.x;
        dst.y = mask.y != 0.0 ? b.y : a.y;
        dst.z = mask.z != 0.0 ? b.z : a.z;
        dst.w = mask.w != 0.0 ? b.w : a.w;
    }
}

function b2DotWTo(dst: b2FloatW, a: b2Vec2W, b: b2Vec2W): void {
    if (ASC_FEATURE_SIMD) {
        store<v128>(wptr(dst),
                    f32x4.add(f32x4.mul(load<v128>(wptr(a.X)), load<v128>(wptr(b.X))),
                              f32x4.mul(load<v128>(wptr(a.Y)), load<v128>(wptr(b.Y)))));
    } else {
        dst.x = a.X.x * b.X.x + a.Y.x * b.Y.x;
        dst.y = a.X.y * b.X.y + a.Y.y * b.Y.y;
        dst.z = a.X.z * b.X.z + a.Y.z * b.Y.z;
        dst.w = a.X.w * b.X.w + a.Y.w * b.Y.w;
    }
}

// dst must not alias v.
function b2RotateVectorWTo(dst: b2Vec2W, q: b2RotW, v: b2Vec2W): void {
    if (ASC_FEATURE_SIMD) {
        let c = load<v128>(wptr(q.C));
        let s = load<v128>(wptr(q.S));
        let vx = load<v128>(wptr(v.X));
        let vy = load<v128>(wptr(v.Y));
        store<v128>(wptr(dst.X), f32x4.sub(f32x4.mul(c, vx), f32x4.mul(s, vy)));
        store<v128>(wptr(dst.Y), f32x4.add(f32x4.mul(s, vx), f32x4.mul(c, vy)));
    } else {
        b2SubWTo(dst.X, b2MulWTmp(s_rotTmp1, q.C, v.X), b2MulWTmp(s_rotTmp2, q.S, v.Y));
        b2AddWTo(dst.Y, b2MulWTmp(s_rotTmp1, q.S, v.X), b2MulWTmp(s_rotTmp2, q.C, v.Y));
    }
}

function b2MulWTmp(dst: b2FloatW, a: b2FloatW, b: b2FloatW): b2FloatW {
    b2MulWTo(dst, a, b);
    return dst;
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

export class b2ContactConstraintSIMD {
    indexA: StaticArray<i32> = new StaticArray<i32>(B2_SIMD_WIDTH);
    indexB: StaticArray<i32> = new StaticArray<i32>(B2_SIMD_WIDTH);

    invMassA: b2FloatW = new b2FloatW();
    invMassB: b2FloatW = new b2FloatW();
    invIA: b2FloatW = new b2FloatW();
    invIB: b2FloatW = new b2FloatW();
    normal: b2Vec2W = new b2Vec2W();
    friction: b2FloatW = new b2FloatW();
    tangentSpeed: b2FloatW = new b2FloatW();
    rollingResistance: b2FloatW = new b2FloatW();
    rollingMass: b2FloatW = new b2FloatW();
    rollingImpulse: b2FloatW = new b2FloatW();
    biasRate: b2FloatW = new b2FloatW();
    massScale: b2FloatW = new b2FloatW();
    impulseScale: b2FloatW = new b2FloatW();
    anchorA1: b2Vec2W = new b2Vec2W();
    anchorB1: b2Vec2W = new b2Vec2W();
    normalMass1: b2FloatW = new b2FloatW();
    tangentMass1: b2FloatW = new b2FloatW();
    baseSeparation1: b2FloatW = new b2FloatW();
    normalImpulse1: b2FloatW = new b2FloatW();
    totalNormalImpulse1: b2FloatW = new b2FloatW();
    tangentImpulse1: b2FloatW = new b2FloatW();
    anchorA2: b2Vec2W = new b2Vec2W();
    anchorB2: b2Vec2W = new b2Vec2W();
    baseSeparation2: b2FloatW = new b2FloatW();
    normalImpulse2: b2FloatW = new b2FloatW();
    totalNormalImpulse2: b2FloatW = new b2FloatW();
    tangentImpulse2: b2FloatW = new b2FloatW();
    normalMass2: b2FloatW = new b2FloatW();
    tangentMass2: b2FloatW = new b2FloatW();
    restitution: b2FloatW = new b2FloatW();
    relativeVelocity1: b2FloatW = new b2FloatW();
    relativeVelocity2: b2FloatW = new b2FloatW();
}

// C allocates constraint memory from the arena in solver.c; solver.as calls
// these and assigns the results to the graph colors and step context.
export function b2AllocSimdConstraints(count: i32): StaticArray<b2ContactConstraintSIMD> {
    let constraints = new StaticArray<b2ContactConstraintSIMD>(count);
    for (let i = 0; i < count; ++i) {
        constraints[i] = new b2ContactConstraintSIMD();
    }
    return constraints;
}

export function b2AllocOverflowConstraints(count: i32): StaticArray<b2ContactConstraint> {
    let constraints = new StaticArray<b2ContactConstraint>(count);
    for (let i = 0; i < count; ++i) {
        constraints[i] = new b2ContactConstraint();
    }
    return constraints;
}

// wide version of b2BodyState
class b2BodyStateW {
    v: b2Vec2W = new b2Vec2W();
    w: b2FloatW = new b2FloatW();
    flags: b2FloatW = new b2FloatW();
    dp: b2Vec2W = new b2Vec2W();
    dq: b2RotW = new b2RotW();
}

// This is a load and transpose
function b2GatherBodies(states: Array<b2BodyState | null>, indices: StaticArray<i32>): b2BodyStateW {
    let identity = new b2BodyState();

    let s1 = indices[0] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[0])])!;
    let s2 = indices[1] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[1])])!;
    let s3 = indices[2] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[2])])!;
    let s4 = indices[3] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[3])])!;

    let simdBody = new b2BodyStateW();
    simdBody.v.X = b2MakeW(s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x);
    simdBody.v.Y = b2MakeW(s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y);
    simdBody.w = b2MakeW(s1.angularVelocity, s2.angularVelocity, s3.angularVelocity, s4.angularVelocity);
    simdBody.flags = b2MakeW(<f32>s1.flags, <f32>s2.flags, <f32>s3.flags, <f32>s4.flags);
    simdBody.dp.X = b2MakeW(s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x);
    simdBody.dp.Y = b2MakeW(s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y);
    simdBody.dq.C = b2MakeW(s1.deltaRotation.c, s2.deltaRotation.c, s3.deltaRotation.c, s4.deltaRotation.c);
    simdBody.dq.S = b2MakeW(s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s);

    return simdBody;
}

// This writes only the velocities back to the solver bodies
function b2ScatterBodies(states: Array<b2BodyState | null>, indices: StaticArray<i32>, simdBody: b2BodyStateW): void {
    // I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
    // associated cache flushing.
    if (indices[0] != B2_NULL_INDEX && (unchecked(states[unchecked(indices[0])])!.flags & b2_dynamicFlag) != 0) {
        let state = unchecked(states[unchecked(indices[0])])!;
        state.linearVelocity.x = simdBody.v.X.x;
        state.linearVelocity.y = simdBody.v.Y.x;
        state.angularVelocity = simdBody.w.x;
    }

    if (indices[1] != B2_NULL_INDEX && (unchecked(states[unchecked(indices[1])])!.flags & b2_dynamicFlag) != 0) {
        let state = unchecked(states[unchecked(indices[1])])!;
        state.linearVelocity.x = simdBody.v.X.y;
        state.linearVelocity.y = simdBody.v.Y.y;
        state.angularVelocity = simdBody.w.y;
    }

    if (indices[2] != B2_NULL_INDEX && (unchecked(states[unchecked(indices[2])])!.flags & b2_dynamicFlag) != 0) {
        let state = unchecked(states[unchecked(indices[2])])!;
        state.linearVelocity.x = simdBody.v.X.z;
        state.linearVelocity.y = simdBody.v.Y.z;
        state.angularVelocity = simdBody.w.z;
    }

    if (indices[3] != B2_NULL_INDEX && (unchecked(states[unchecked(indices[3])])!.flags & b2_dynamicFlag) != 0) {
        let state = unchecked(states[unchecked(indices[3])])!;
        state.linearVelocity.x = simdBody.v.X.w;
        state.linearVelocity.y = simdBody.v.Y.w;
        state.angularVelocity = simdBody.w.w;
    }
}

// Module-level scratch for the hot tasks; the solver is single threaded and
// tasks never nest, so one set suffices. All allocated at module start so the
// bench heap rewind never frees them.
const s_identityState = new b2BodyState();
const s_bA = new b2BodyStateW();
const s_bB = new b2BodyStateW();
const s_zeroW = new b2FloatW();
const s_rotTmp1 = new b2FloatW();
const s_rotTmp2 = new b2FloatW();
const s_dp = new b2Vec2W();
const s_rsA = new b2Vec2W();
const s_rsB = new b2Vec2W();
const s_ds = new b2Vec2W();
const s_biasRate = new b2FloatW();
const s_total = new b2FloatW();
const s_s = new b2FloatW();
const s_mask = new b2FloatW();
const s_specBias = new b2FloatW();
const s_softBias = new b2FloatW();
const s_bias = new b2FloatW();
const s_pointMassScale = new b2FloatW();
const s_pointImpulseScale = new b2FloatW();
const s_dvx = new b2FloatW();
const s_dvy = new b2FloatW();
const s_vn = new b2FloatW();
const s_vt = new b2FloatW();
const s_t1 = new b2FloatW();
const s_t2 = new b2FloatW();
const s_newImpulse = new b2FloatW();
const s_impulse = new b2FloatW();
const s_maxFriction = new b2FloatW();
const s_tangentY = new b2FloatW();
const s_Px = new b2FloatW();
const s_Py = new b2FloatW();
const s_lambda = new b2FloatW();
const s_deltaLambda = new b2FloatW();
const s_maxLambda = new b2FloatW();

// In-place b2GatherBodies writing into module scratch.
function b2GatherBodiesTo(dst: b2BodyStateW, states: Array<b2BodyState | null>, indices: StaticArray<i32>): void {
    let identity = s_identityState;

    let s1 = indices[0] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[0])])!;
    let s2 = indices[1] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[1])])!;
    let s3 = indices[2] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[2])])!;
    let s4 = indices[3] == B2_NULL_INDEX ? identity : unchecked(states[unchecked(indices[3])])!;

    let vX = dst.v.X;
    vX.x = s1.linearVelocity.x;
    vX.y = s2.linearVelocity.x;
    vX.z = s3.linearVelocity.x;
    vX.w = s4.linearVelocity.x;
    let vY = dst.v.Y;
    vY.x = s1.linearVelocity.y;
    vY.y = s2.linearVelocity.y;
    vY.z = s3.linearVelocity.y;
    vY.w = s4.linearVelocity.y;
    let w = dst.w;
    w.x = s1.angularVelocity;
    w.y = s2.angularVelocity;
    w.z = s3.angularVelocity;
    w.w = s4.angularVelocity;
    let flags = dst.flags;
    flags.x = <f32>s1.flags;
    flags.y = <f32>s2.flags;
    flags.z = <f32>s3.flags;
    flags.w = <f32>s4.flags;
    let dpX = dst.dp.X;
    dpX.x = s1.deltaPosition.x;
    dpX.y = s2.deltaPosition.x;
    dpX.z = s3.deltaPosition.x;
    dpX.w = s4.deltaPosition.x;
    let dpY = dst.dp.Y;
    dpY.x = s1.deltaPosition.y;
    dpY.y = s2.deltaPosition.y;
    dpY.z = s3.deltaPosition.y;
    dpY.w = s4.deltaPosition.y;
    let dqC = dst.dq.C;
    dqC.x = s1.deltaRotation.c;
    dqC.y = s2.deltaRotation.c;
    dqC.z = s3.deltaRotation.c;
    dqC.w = s4.deltaRotation.c;
    let dqS = dst.dq.S;
    dqS.x = s1.deltaRotation.s;
    dqS.y = s2.deltaRotation.s;
    dqS.z = s3.deltaRotation.s;
    dqS.w = s4.deltaRotation.s;
}

export function b2PrepareContactsTask(startIndex: i32, endIndex: i32, context: b2StepContext): void {
    let world = context.world!;
    let contacts = context.contacts;
    let constraints = context.simdContactConstraints!;
    let awakeStates = context.states;

    // Stiffer for static contacts to avoid bodies getting pushed through the ground
    let contactSoftness = context.contactSoftness;
    let staticSoftness = context.staticSoftness;
    let enableSoftening = world.enableContactSoftening;

    let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;

    for (let i = startIndex; i < endIndex; ++i) {
        let constraint = unchecked(constraints[i]);

        for (let j = 0; j < B2_SIMD_WIDTH; ++j) {
            let contactSim = unchecked(contacts[B2_SIMD_WIDTH * i + j]);

            if (contactSim != null) {
                let manifold = contactSim.manifold;

                let indexA = contactSim.bodySimIndexA;
                let indexB = contactSim.bodySimIndexB;

                unchecked(constraint.indexA[j] = indexA);
                unchecked(constraint.indexB[j] = indexB);

                let vA = b2Vec2_zero();
                let wA: f32 = 0.0;
                let mA = contactSim.invMassA;
                let iA = contactSim.invIA;
                if (indexA != B2_NULL_INDEX) {
                    let stateA = unchecked(awakeStates[indexA])!;
                    vA = stateA.linearVelocity;
                    wA = stateA.angularVelocity;
                }

                let vB = b2Vec2_zero();
                let wB: f32 = 0.0;
                let mB = contactSim.invMassB;
                let iB = contactSim.invIB;
                if (indexB != B2_NULL_INDEX) {
                    let stateB = unchecked(awakeStates[indexB])!;
                    vB = stateB.linearVelocity;
                    wB = stateB.angularVelocity;
                }

                b2SetLaneW(constraint.invMassA, j, mA);
                b2SetLaneW(constraint.invMassB, j, mB);
                b2SetLaneW(constraint.invIA, j, iA);
                b2SetLaneW(constraint.invIB, j, iB);

                {
                    let k = iA + iB;
                    b2SetLaneW(constraint.rollingMass, j, k > 0.0 ? <f32>1.0 / k : <f32>0.0);
                }

                let soft = contactSoftness;
                if (indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX) {
                    soft = staticSoftness;
                } else if (enableSoftening) {
                    // todo experimental feature
                    let contactHertz = b2MinFloat(world.contactHertz, <f32>0.125 * context.inv_h);
                    let ratio: f32 = 1.0;
                    if (mA < mB) {
                        ratio = b2MaxFloat(<f32>0.5, mA / mB);
                    } else if (mB < mA) {
                        ratio = b2MaxFloat(<f32>0.5, mB / mA);
                    }
                    soft = b2MakeSoft(ratio * contactHertz, ratio * world.contactDampingRatio, context.h);
                }

                let normal = manifold.normal;
                b2SetLaneW(constraint.normal.X, j, normal.x);
                b2SetLaneW(constraint.normal.Y, j, normal.y);

                b2SetLaneW(constraint.friction, j, contactSim.friction);
                b2SetLaneW(constraint.tangentSpeed, j, contactSim.tangentSpeed);
                b2SetLaneW(constraint.restitution, j, contactSim.restitution);
                b2SetLaneW(constraint.rollingResistance, j, contactSim.rollingResistance);
                b2SetLaneW(constraint.rollingImpulse, j, warmStartScale * manifold.rollingImpulse);

                b2SetLaneW(constraint.biasRate, j, soft.biasRate);
                b2SetLaneW(constraint.massScale, j, soft.massScale);
                b2SetLaneW(constraint.impulseScale, j, soft.impulseScale);

                let tangent = b2RightPerp(normal);

                {
                    let mp = unchecked(manifold.points[0]);

                    let rA = mp.anchorA;
                    let rB = mp.anchorB;

                    b2SetLaneW(constraint.anchorA1.X, j, rA.x);
                    b2SetLaneW(constraint.anchorA1.Y, j, rA.y);
                    b2SetLaneW(constraint.anchorB1.X, j, rB.x);
                    b2SetLaneW(constraint.anchorB1.Y, j, rB.y);

                    b2SetLaneW(constraint.baseSeparation1, j, mp.separation - b2Dot(b2Sub(rB, rA), normal));

                    b2SetLaneW(constraint.normalImpulse1, j, warmStartScale * mp.normalImpulse);
                    b2SetLaneW(constraint.tangentImpulse1, j, warmStartScale * mp.tangentImpulse);
                    b2SetLaneW(constraint.totalNormalImpulse1, j, 0.0);

                    let rnA = b2Cross(rA, normal);
                    let rnB = b2Cross(rB, normal);
                    let kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                    b2SetLaneW(constraint.normalMass1, j, kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0);

                    let rtA = b2Cross(rA, tangent);
                    let rtB = b2Cross(rB, tangent);
                    let kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                    b2SetLaneW(constraint.tangentMass1, j, kTangent > 0.0 ? <f32>1.0 / kTangent : <f32>0.0);

                    // relative velocity for restitution
                    let vrA = b2Add(vA, b2CrossSV(wA, rA));
                    let vrB = b2Add(vB, b2CrossSV(wB, rB));
                    b2SetLaneW(constraint.relativeVelocity1, j, b2Dot(normal, b2Sub(vrB, vrA)));
                }

                let pointCount = manifold.pointCount;
                assert(0 < pointCount && pointCount <= 2);

                if (pointCount == 2) {
                    let mp = unchecked(manifold.points[1]);

                    let rA = mp.anchorA;
                    let rB = mp.anchorB;

                    b2SetLaneW(constraint.anchorA2.X, j, rA.x);
                    b2SetLaneW(constraint.anchorA2.Y, j, rA.y);
                    b2SetLaneW(constraint.anchorB2.X, j, rB.x);
                    b2SetLaneW(constraint.anchorB2.Y, j, rB.y);

                    b2SetLaneW(constraint.baseSeparation2, j, mp.separation - b2Dot(b2Sub(rB, rA), normal));

                    b2SetLaneW(constraint.normalImpulse2, j, warmStartScale * mp.normalImpulse);
                    b2SetLaneW(constraint.tangentImpulse2, j, warmStartScale * mp.tangentImpulse);
                    b2SetLaneW(constraint.totalNormalImpulse2, j, 0.0);

                    let rnA = b2Cross(rA, normal);
                    let rnB = b2Cross(rB, normal);
                    let kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                    b2SetLaneW(constraint.normalMass2, j, kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0);

                    let rtA = b2Cross(rA, tangent);
                    let rtB = b2Cross(rB, tangent);
                    let kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                    b2SetLaneW(constraint.tangentMass2, j, kTangent > 0.0 ? <f32>1.0 / kTangent : <f32>0.0);

                    // relative velocity for restitution
                    let vrA = b2Add(vA, b2CrossSV(wA, rA));
                    let vrB = b2Add(vB, b2CrossSV(wB, rB));
                    b2SetLaneW(constraint.relativeVelocity2, j, b2Dot(normal, b2Sub(vrB, vrA)));
                } else {
                    // dummy data that has no effect
                    b2SetLaneW(constraint.baseSeparation2, j, 0.0);
                    b2SetLaneW(constraint.normalImpulse2, j, 0.0);
                    b2SetLaneW(constraint.tangentImpulse2, j, 0.0);
                    b2SetLaneW(constraint.totalNormalImpulse2, j, 0.0);
                    b2SetLaneW(constraint.anchorA2.X, j, 0.0);
                    b2SetLaneW(constraint.anchorA2.Y, j, 0.0);
                    b2SetLaneW(constraint.anchorB2.X, j, 0.0);
                    b2SetLaneW(constraint.anchorB2.Y, j, 0.0);
                    b2SetLaneW(constraint.normalMass2, j, 0.0);
                    b2SetLaneW(constraint.tangentMass2, j, 0.0);
                    b2SetLaneW(constraint.relativeVelocity2, j, 0.0);
                }
            } else {
                // SIMD remainder
                unchecked(constraint.indexA[j] = B2_NULL_INDEX);
                unchecked(constraint.indexB[j] = B2_NULL_INDEX);

                b2SetLaneW(constraint.invMassA, j, 0.0);
                b2SetLaneW(constraint.invMassB, j, 0.0);
                b2SetLaneW(constraint.invIA, j, 0.0);
                b2SetLaneW(constraint.invIB, j, 0.0);

                b2SetLaneW(constraint.normal.X, j, 0.0);
                b2SetLaneW(constraint.normal.Y, j, 0.0);
                b2SetLaneW(constraint.friction, j, 0.0);
                b2SetLaneW(constraint.tangentSpeed, j, 0.0);
                b2SetLaneW(constraint.rollingResistance, j, 0.0);
                b2SetLaneW(constraint.rollingMass, j, 0.0);
                b2SetLaneW(constraint.rollingImpulse, j, 0.0);
                b2SetLaneW(constraint.biasRate, j, 0.0);
                b2SetLaneW(constraint.massScale, j, 0.0);
                b2SetLaneW(constraint.impulseScale, j, 0.0);

                b2SetLaneW(constraint.anchorA1.X, j, 0.0);
                b2SetLaneW(constraint.anchorA1.Y, j, 0.0);
                b2SetLaneW(constraint.anchorB1.X, j, 0.0);
                b2SetLaneW(constraint.anchorB1.Y, j, 0.0);
                b2SetLaneW(constraint.baseSeparation1, j, 0.0);
                b2SetLaneW(constraint.normalImpulse1, j, 0.0);
                b2SetLaneW(constraint.tangentImpulse1, j, 0.0);
                b2SetLaneW(constraint.totalNormalImpulse1, j, 0.0);
                b2SetLaneW(constraint.normalMass1, j, 0.0);
                b2SetLaneW(constraint.tangentMass1, j, 0.0);

                b2SetLaneW(constraint.anchorA2.X, j, 0.0);
                b2SetLaneW(constraint.anchorA2.Y, j, 0.0);
                b2SetLaneW(constraint.anchorB2.X, j, 0.0);
                b2SetLaneW(constraint.anchorB2.Y, j, 0.0);
                b2SetLaneW(constraint.baseSeparation2, j, 0.0);
                b2SetLaneW(constraint.normalImpulse2, j, 0.0);
                b2SetLaneW(constraint.tangentImpulse2, j, 0.0);
                b2SetLaneW(constraint.totalNormalImpulse2, j, 0.0);
                b2SetLaneW(constraint.normalMass2, j, 0.0);
                b2SetLaneW(constraint.tangentMass2, j, 0.0);

                b2SetLaneW(constraint.restitution, j, 0.0);
                b2SetLaneW(constraint.relativeVelocity1, j, 0.0);
                b2SetLaneW(constraint.relativeVelocity2, j, 0.0);
            }
        }
    }
}

export function b2WarmStartContactsTask(startIndex: i32, endIndex: i32, context: b2StepContext, colorIndex: i32): void {
    let states = context.states;
    let constraints = context.graph!.colors[colorIndex].simdConstraints!;

    for (let i = startIndex; i < endIndex; ++i) {
        let c = unchecked(constraints[i]);
        let bA = s_bA;
        let bB = s_bB;
        b2GatherBodiesTo(bA, states, c.indexA);
        b2GatherBodiesTo(bB, states, c.indexB);

        let tangentX = c.normal.Y;
        let tangentY = s_tangentY;
        b2SubWTo(tangentY, s_zeroW, c.normal.X);

        {
            // fixed anchors
            let rA = c.anchorA1;
            let rB = c.anchorB1;

            let Px = s_Px;
            let Py = s_Py;
            b2MulWTo(Px, c.normalImpulse1, c.normal.X);
            b2MulWTo(s_t1, c.tangentImpulse1, tangentX);
            b2AddWTo(Px, Px, s_t1);
            b2MulWTo(Py, c.normalImpulse1, c.normal.Y);
            b2MulWTo(s_t1, c.tangentImpulse1, tangentY);
            b2AddWTo(Py, Py, s_t1);

            b2MulWTo(s_t1, rA.X, Py);
            b2MulWTo(s_t2, rA.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulSubWTo(bA.w, bA.w, c.invIA, s_t1);
            b2MulSubWTo(bA.v.X, bA.v.X, c.invMassA, Px);
            b2MulSubWTo(bA.v.Y, bA.v.Y, c.invMassA, Py);
            b2MulWTo(s_t1, rB.X, Py);
            b2MulWTo(s_t2, rB.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulAddWTo(bB.w, bB.w, c.invIB, s_t1);
            b2MulAddWTo(bB.v.X, bB.v.X, c.invMassB, Px);
            b2MulAddWTo(bB.v.Y, bB.v.Y, c.invMassB, Py);

            b2AddWTo(c.totalNormalImpulse1, c.totalNormalImpulse1, c.normalImpulse1);
        }

        {
            // fixed anchors
            let rA = c.anchorA2;
            let rB = c.anchorB2;

            let Px = s_Px;
            let Py = s_Py;
            b2MulWTo(Px, c.normalImpulse2, c.normal.X);
            b2MulWTo(s_t1, c.tangentImpulse2, tangentX);
            b2AddWTo(Px, Px, s_t1);
            b2MulWTo(Py, c.normalImpulse2, c.normal.Y);
            b2MulWTo(s_t1, c.tangentImpulse2, tangentY);
            b2AddWTo(Py, Py, s_t1);

            b2MulWTo(s_t1, rA.X, Py);
            b2MulWTo(s_t2, rA.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulSubWTo(bA.w, bA.w, c.invIA, s_t1);
            b2MulSubWTo(bA.v.X, bA.v.X, c.invMassA, Px);
            b2MulSubWTo(bA.v.Y, bA.v.Y, c.invMassA, Py);
            b2MulWTo(s_t1, rB.X, Py);
            b2MulWTo(s_t2, rB.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulAddWTo(bB.w, bB.w, c.invIB, s_t1);
            b2MulAddWTo(bB.v.X, bB.v.X, c.invMassB, Px);
            b2MulAddWTo(bB.v.Y, bB.v.Y, c.invMassB, Py);

            b2AddWTo(c.totalNormalImpulse2, c.totalNormalImpulse2, c.normalImpulse2);
        }

        b2MulSubWTo(bA.w, bA.w, c.invIA, c.rollingImpulse);
        b2MulAddWTo(bB.w, bB.w, c.invIB, c.rollingImpulse);

        b2ScatterBodies(states, c.indexA, bA);
        b2ScatterBodies(states, c.indexB, bB);
    }
}

export function b2SolveContactsTask(startIndex: i32, endIndex: i32, context: b2StepContext, colorIndex: i32, useBias: bool): void {
    let states = context.states;
    let constraints = context.graph!.colors[colorIndex].simdConstraints!;
    let inv_h = b2SplatW(context.inv_h);
    let contactSpeed = b2SplatW(-context.world!.contactSpeed);
    let oneW = b2SplatW(1.0);
    let zeroW = s_zeroW;

    for (let i = startIndex; i < endIndex; ++i) {
        let c = unchecked(constraints[i]);

        let bA = s_bA;
        let bB = s_bB;
        b2GatherBodiesTo(bA, states, c.indexA);
        b2GatherBodiesTo(bB, states, c.indexB);

        let biasRate: b2FloatW = zeroW;
        let massScale: b2FloatW = oneW;
        let impulseScale: b2FloatW = zeroW;
        if (useBias) {
            b2MulWTo(s_biasRate, c.massScale, c.biasRate);
            biasRate = s_biasRate;
            massScale = c.massScale;
            impulseScale = c.impulseScale;
        }

        let totalNormalImpulse = s_total;
        b2SetZeroW(totalNormalImpulse);

        let dp = s_dp;
        b2SubWTo(dp.X, bB.dp.X, bA.dp.X);
        b2SubWTo(dp.Y, bB.dp.Y, bA.dp.Y);

        // point1 non-penetration constraint
        {
            // Fixed anchors for impulses
            let rA = c.anchorA1;
            let rB = c.anchorB1;

            // Moving anchors for current separation
            let rsA = s_rsA;
            let rsB = s_rsB;
            b2RotateVectorWTo(rsA, bA.dq, rA);
            b2RotateVectorWTo(rsB, bB.dq, rB);

            // compute current separation
            // this is subject to round-off error if the anchor is far from the body center of mass
            let ds = s_ds;
            b2SubWTo(ds.X, rsB.X, rsA.X);
            b2AddWTo(ds.X, dp.X, ds.X);
            b2SubWTo(ds.Y, rsB.Y, rsA.Y);
            b2AddWTo(ds.Y, dp.Y, ds.Y);
            let s = s_s;
            b2DotWTo(s, c.normal, ds);
            b2AddWTo(s, s, c.baseSeparation1);

            // Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
            // The contactSpeed is meant to limit stiffness, not increase it.
            let mask = s_mask;
            b2GreaterThanWTo(mask, s, zeroW);
            let specBias = s_specBias;
            b2MulWTo(specBias, s, inv_h);
            let softBias = s_softBias;
            b2MulWTo(softBias, biasRate, s);
            b2MaxWTo(softBias, softBias, contactSpeed);
            let bias = s_bias;
            b2BlendWTo(bias, softBias, specBias, mask);

            let pointMassScale = s_pointMassScale;
            b2BlendWTo(pointMassScale, massScale, oneW, mask);
            let pointImpulseScale = s_pointImpulseScale;
            b2BlendWTo(pointImpulseScale, impulseScale, zeroW, mask);

            // Relative velocity at contact
            let dvx = s_dvx;
            b2MulWTo(s_t1, bB.w, rB.Y);
            b2SubWTo(dvx, bB.v.X, s_t1);
            b2MulWTo(s_t1, bA.w, rA.Y);
            b2SubWTo(s_t1, bA.v.X, s_t1);
            b2SubWTo(dvx, dvx, s_t1);
            let dvy = s_dvy;
            b2MulWTo(s_t1, bB.w, rB.X);
            b2AddWTo(dvy, bB.v.Y, s_t1);
            b2MulWTo(s_t1, bA.w, rA.X);
            b2AddWTo(s_t1, bA.v.Y, s_t1);
            b2SubWTo(dvy, dvy, s_t1);
            let vn = s_vn;
            b2MulWTo(vn, dvx, c.normal.X);
            b2MulWTo(s_t1, dvy, c.normal.Y);
            b2AddWTo(vn, vn, s_t1);

            // Compute normal impulse
            b2MulWTo(s_t1, pointMassScale, vn);
            b2AddWTo(s_t1, s_t1, bias);
            b2MulWTo(s_t1, c.normalMass1, s_t1);
            b2MulWTo(s_t2, pointImpulseScale, c.normalImpulse1);
            b2AddWTo(s_t1, s_t1, s_t2);

            // Clamp the accumulated impulse
            let newImpulse = s_newImpulse;
            b2SubWTo(newImpulse, c.normalImpulse1, s_t1);
            b2MaxWTo(newImpulse, newImpulse, zeroW);
            let impulse = s_impulse;
            b2SubWTo(impulse, newImpulse, c.normalImpulse1);
            b2CopyW(c.normalImpulse1, newImpulse);
            b2AddWTo(c.totalNormalImpulse1, c.totalNormalImpulse1, impulse);

            b2AddWTo(totalNormalImpulse, totalNormalImpulse, newImpulse);

            // Apply contact impulse
            let Px = s_Px;
            b2MulWTo(Px, impulse, c.normal.X);
            let Py = s_Py;
            b2MulWTo(Py, impulse, c.normal.Y);

            b2MulSubWTo(bA.v.X, bA.v.X, c.invMassA, Px);
            b2MulSubWTo(bA.v.Y, bA.v.Y, c.invMassA, Py);
            b2MulWTo(s_t1, rA.X, Py);
            b2MulWTo(s_t2, rA.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulSubWTo(bA.w, bA.w, c.invIA, s_t1);

            b2MulAddWTo(bB.v.X, bB.v.X, c.invMassB, Px);
            b2MulAddWTo(bB.v.Y, bB.v.Y, c.invMassB, Py);
            b2MulWTo(s_t1, rB.X, Py);
            b2MulWTo(s_t2, rB.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulAddWTo(bB.w, bB.w, c.invIB, s_t1);
        }

        // second point non-penetration constraint
        {
            // moving anchors for current separation
            let rsA = s_rsA;
            let rsB = s_rsB;
            b2RotateVectorWTo(rsA, bA.dq, c.anchorA2);
            b2RotateVectorWTo(rsB, bB.dq, c.anchorB2);

            // compute current separation
            let ds = s_ds;
            b2SubWTo(ds.X, rsB.X, rsA.X);
            b2AddWTo(ds.X, dp.X, ds.X);
            b2SubWTo(ds.Y, rsB.Y, rsA.Y);
            b2AddWTo(ds.Y, dp.Y, ds.Y);
            let s = s_s;
            b2DotWTo(s, c.normal, ds);
            b2AddWTo(s, s, c.baseSeparation2);

            let mask = s_mask;
            b2GreaterThanWTo(mask, s, zeroW);
            let specBias = s_specBias;
            b2MulWTo(specBias, s, inv_h);
            let softBias = s_softBias;
            b2MulWTo(softBias, biasRate, s);
            b2MaxWTo(softBias, softBias, contactSpeed);
            let bias = s_bias;
            b2BlendWTo(bias, softBias, specBias, mask);

            let pointMassScale = s_pointMassScale;
            b2BlendWTo(pointMassScale, massScale, oneW, mask);
            let pointImpulseScale = s_pointImpulseScale;
            b2BlendWTo(pointImpulseScale, impulseScale, zeroW, mask);

            // fixed anchors for Jacobians
            let rA = c.anchorA2;
            let rB = c.anchorB2;

            // Relative velocity at contact
            let dvx = s_dvx;
            b2MulWTo(s_t1, bB.w, rB.Y);
            b2SubWTo(dvx, bB.v.X, s_t1);
            b2MulWTo(s_t1, bA.w, rA.Y);
            b2SubWTo(s_t1, bA.v.X, s_t1);
            b2SubWTo(dvx, dvx, s_t1);
            let dvy = s_dvy;
            b2MulWTo(s_t1, bB.w, rB.X);
            b2AddWTo(dvy, bB.v.Y, s_t1);
            b2MulWTo(s_t1, bA.w, rA.X);
            b2AddWTo(s_t1, bA.v.Y, s_t1);
            b2SubWTo(dvy, dvy, s_t1);
            let vn = s_vn;
            b2MulWTo(vn, dvx, c.normal.X);
            b2MulWTo(s_t1, dvy, c.normal.Y);
            b2AddWTo(vn, vn, s_t1);

            // Compute normal impulse
            b2MulWTo(s_t1, pointMassScale, vn);
            b2AddWTo(s_t1, s_t1, bias);
            b2MulWTo(s_t1, c.normalMass2, s_t1);
            b2MulWTo(s_t2, pointImpulseScale, c.normalImpulse2);
            b2AddWTo(s_t1, s_t1, s_t2);

            // Clamp the accumulated impulse
            let newImpulse = s_newImpulse;
            b2SubWTo(newImpulse, c.normalImpulse2, s_t1);
            b2MaxWTo(newImpulse, newImpulse, zeroW);
            let impulse = s_impulse;
            b2SubWTo(impulse, newImpulse, c.normalImpulse2);
            b2CopyW(c.normalImpulse2, newImpulse);
            b2AddWTo(c.totalNormalImpulse2, c.totalNormalImpulse2, impulse);

            b2AddWTo(totalNormalImpulse, totalNormalImpulse, newImpulse);

            // Apply contact impulse
            let Px = s_Px;
            b2MulWTo(Px, impulse, c.normal.X);
            let Py = s_Py;
            b2MulWTo(Py, impulse, c.normal.Y);

            b2MulSubWTo(bA.v.X, bA.v.X, c.invMassA, Px);
            b2MulSubWTo(bA.v.Y, bA.v.Y, c.invMassA, Py);
            b2MulWTo(s_t1, rA.X, Py);
            b2MulWTo(s_t2, rA.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulSubWTo(bA.w, bA.w, c.invIA, s_t1);

            b2MulAddWTo(bB.v.X, bB.v.X, c.invMassB, Px);
            b2MulAddWTo(bB.v.Y, bB.v.Y, c.invMassB, Py);
            b2MulWTo(s_t1, rB.X, Py);
            b2MulWTo(s_t2, rB.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulAddWTo(bB.w, bB.w, c.invIB, s_t1);
        }

        let tangentX = c.normal.Y;
        let tangentY = s_tangentY;
        b2SubWTo(tangentY, zeroW, c.normal.X);

        // point 1 friction constraint
        {
            // fixed anchors for Jacobians
            let rA = c.anchorA1;
            let rB = c.anchorB1;

            // Relative velocity at contact
            let dvx = s_dvx;
            b2MulWTo(s_t1, bB.w, rB.Y);
            b2SubWTo(dvx, bB.v.X, s_t1);
            b2MulWTo(s_t1, bA.w, rA.Y);
            b2SubWTo(s_t1, bA.v.X, s_t1);
            b2SubWTo(dvx, dvx, s_t1);
            let dvy = s_dvy;
            b2MulWTo(s_t1, bB.w, rB.X);
            b2AddWTo(dvy, bB.v.Y, s_t1);
            b2MulWTo(s_t1, bA.w, rA.X);
            b2AddWTo(s_t1, bA.v.Y, s_t1);
            b2SubWTo(dvy, dvy, s_t1);
            let vt = s_vt;
            b2MulWTo(vt, dvx, tangentX);
            b2MulWTo(s_t1, dvy, tangentY);
            b2AddWTo(vt, vt, s_t1);

            // Tangent speed (conveyor belt)
            b2SubWTo(vt, vt, c.tangentSpeed);

            // Compute tangent force
            b2MulWTo(s_t1, c.tangentMass1, vt);

            // Clamp the accumulated force
            let maxFriction = s_maxFriction;
            b2MulWTo(maxFriction, c.friction, c.normalImpulse1);
            let newImpulse = s_newImpulse;
            b2SubWTo(newImpulse, c.tangentImpulse1, s_t1);
            b2MinWTo(newImpulse, newImpulse, maxFriction);
            b2SubWTo(s_t1, zeroW, maxFriction);
            b2MaxWTo(newImpulse, s_t1, newImpulse);
            let impulse = s_impulse;
            b2SubWTo(impulse, newImpulse, c.tangentImpulse1);
            b2CopyW(c.tangentImpulse1, newImpulse);

            // Apply contact impulse
            let Px = s_Px;
            b2MulWTo(Px, impulse, tangentX);
            let Py = s_Py;
            b2MulWTo(Py, impulse, tangentY);

            b2MulSubWTo(bA.v.X, bA.v.X, c.invMassA, Px);
            b2MulSubWTo(bA.v.Y, bA.v.Y, c.invMassA, Py);
            b2MulWTo(s_t1, rA.X, Py);
            b2MulWTo(s_t2, rA.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulSubWTo(bA.w, bA.w, c.invIA, s_t1);

            b2MulAddWTo(bB.v.X, bB.v.X, c.invMassB, Px);
            b2MulAddWTo(bB.v.Y, bB.v.Y, c.invMassB, Py);
            b2MulWTo(s_t1, rB.X, Py);
            b2MulWTo(s_t2, rB.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulAddWTo(bB.w, bB.w, c.invIB, s_t1);
        }

        // second point friction constraint
        {
            // fixed anchors for Jacobians
            let rA = c.anchorA2;
            let rB = c.anchorB2;

            // Relative velocity at contact
            let dvx = s_dvx;
            b2MulWTo(s_t1, bB.w, rB.Y);
            b2SubWTo(dvx, bB.v.X, s_t1);
            b2MulWTo(s_t1, bA.w, rA.Y);
            b2SubWTo(s_t1, bA.v.X, s_t1);
            b2SubWTo(dvx, dvx, s_t1);
            let dvy = s_dvy;
            b2MulWTo(s_t1, bB.w, rB.X);
            b2AddWTo(dvy, bB.v.Y, s_t1);
            b2MulWTo(s_t1, bA.w, rA.X);
            b2AddWTo(s_t1, bA.v.Y, s_t1);
            b2SubWTo(dvy, dvy, s_t1);
            let vt = s_vt;
            b2MulWTo(vt, dvx, tangentX);
            b2MulWTo(s_t1, dvy, tangentY);
            b2AddWTo(vt, vt, s_t1);

            // Tangent speed (conveyor belt)
            b2SubWTo(vt, vt, c.tangentSpeed);

            // Compute tangent force
            b2MulWTo(s_t1, c.tangentMass2, vt);

            // Clamp the accumulated force
            let maxFriction = s_maxFriction;
            b2MulWTo(maxFriction, c.friction, c.normalImpulse2);
            let newImpulse = s_newImpulse;
            b2SubWTo(newImpulse, c.tangentImpulse2, s_t1);
            b2MinWTo(newImpulse, newImpulse, maxFriction);
            b2SubWTo(s_t1, zeroW, maxFriction);
            b2MaxWTo(newImpulse, s_t1, newImpulse);
            let impulse = s_impulse;
            b2SubWTo(impulse, newImpulse, c.tangentImpulse2);
            b2CopyW(c.tangentImpulse2, newImpulse);

            // Apply contact impulse
            let Px = s_Px;
            b2MulWTo(Px, impulse, tangentX);
            let Py = s_Py;
            b2MulWTo(Py, impulse, tangentY);

            b2MulSubWTo(bA.v.X, bA.v.X, c.invMassA, Px);
            b2MulSubWTo(bA.v.Y, bA.v.Y, c.invMassA, Py);
            b2MulWTo(s_t1, rA.X, Py);
            b2MulWTo(s_t2, rA.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulSubWTo(bA.w, bA.w, c.invIA, s_t1);

            b2MulAddWTo(bB.v.X, bB.v.X, c.invMassB, Px);
            b2MulAddWTo(bB.v.Y, bB.v.Y, c.invMassB, Py);
            b2MulWTo(s_t1, rB.X, Py);
            b2MulWTo(s_t2, rB.Y, Px);
            b2SubWTo(s_t1, s_t1, s_t2);
            b2MulAddWTo(bB.w, bB.w, c.invIB, s_t1);
        }

        // Rolling resistance
        {
            let deltaLambda = s_deltaLambda;
            b2SubWTo(s_t1, bA.w, bB.w);
            b2MulWTo(deltaLambda, c.rollingMass, s_t1);
            let lambda = s_lambda;
            b2CopyW(lambda, c.rollingImpulse);
            let maxLambda = s_maxLambda;
            b2MulWTo(maxLambda, c.rollingResistance, totalNormalImpulse);
            b2AddWTo(s_t1, lambda, deltaLambda);
            b2SymClampWTo(c.rollingImpulse, s_t1, maxLambda);
            b2SubWTo(deltaLambda, c.rollingImpulse, lambda);

            b2MulSubWTo(bA.w, bA.w, c.invIA, deltaLambda);
            b2MulAddWTo(bB.w, bB.w, c.invIB, deltaLambda);
        }

        b2ScatterBodies(states, c.indexA, bA);
        b2ScatterBodies(states, c.indexB, bB);
    }
}

export function b2ApplyRestitutionTask(startIndex: i32, endIndex: i32, context: b2StepContext, colorIndex: i32): void {
    let states = context.states;
    let constraints = context.graph!.colors[colorIndex].simdConstraints!;
    let threshold = b2SplatW(context.world!.restitutionThreshold);
    let zero = b2ZeroW();

    for (let i = startIndex; i < endIndex; ++i) {
        let c = unchecked(constraints[i]);

        if (b2AllZeroW(c.restitution)) {
            // No lanes have restitution. Common case.
            continue;
        }

        // Create a mask based on restitution so that lanes with no restitution are not affected
        // by the calculations below.
        let restitutionMask = b2EqualsW(c.restitution, zero);

        let bA = b2GatherBodies(states, c.indexA);
        let bB = b2GatherBodies(states, c.indexB);

        // first point non-penetration constraint
        {
            // Set effective mass to zero if restitution should not be applied
            let mask1 = b2GreaterThanW(b2AddW(c.relativeVelocity1, threshold), zero);
            let mask2 = b2EqualsW(c.totalNormalImpulse1, zero);
            let mask = b2OrW(b2OrW(mask1, mask2), restitutionMask);
            let mass = b2BlendW(c.normalMass1, zero, mask);

            // fixed anchors for Jacobians
            let rA = c.anchorA1;
            let rB = c.anchorB1;

            // Relative velocity at contact
            let dvx = b2SubW(b2SubW(bB.v.X, b2MulW(bB.w, rB.Y)), b2SubW(bA.v.X, b2MulW(bA.w, rA.Y)));
            let dvy = b2SubW(b2AddW(bB.v.Y, b2MulW(bB.w, rB.X)), b2AddW(bA.v.Y, b2MulW(bA.w, rA.X)));
            let vn = b2AddW(b2MulW(dvx, c.normal.X), b2MulW(dvy, c.normal.Y));

            // Compute normal impulse
            let negImpulse = b2MulW(mass, b2AddW(vn, b2MulW(c.restitution, c.relativeVelocity1)));

            // Clamp the accumulated impulse
            let newImpulse = b2MaxW(b2SubW(c.normalImpulse1, negImpulse), b2ZeroW());
            let deltaImpulse = b2SubW(newImpulse, c.normalImpulse1);
            c.normalImpulse1 = newImpulse;

            c.totalNormalImpulse1 = b2AddW(c.totalNormalImpulse1, deltaImpulse);

            // Apply contact impulse
            let Px = b2MulW(deltaImpulse, c.normal.X);
            let Py = b2MulW(deltaImpulse, c.normal.Y);

            bA.v.X = b2MulSubW(bA.v.X, c.invMassA, Px);
            bA.v.Y = b2MulSubW(bA.v.Y, c.invMassA, Py);
            bA.w = b2MulSubW(bA.w, c.invIA, b2SubW(b2MulW(rA.X, Py), b2MulW(rA.Y, Px)));

            bB.v.X = b2MulAddW(bB.v.X, c.invMassB, Px);
            bB.v.Y = b2MulAddW(bB.v.Y, c.invMassB, Py);
            bB.w = b2MulAddW(bB.w, c.invIB, b2SubW(b2MulW(rB.X, Py), b2MulW(rB.Y, Px)));
        }

        // second point non-penetration constraint
        {
            // Set effective mass to zero if restitution should not be applied
            let mask1 = b2GreaterThanW(b2AddW(c.relativeVelocity2, threshold), zero);
            let mask2 = b2EqualsW(c.totalNormalImpulse2, zero);
            let mask = b2OrW(b2OrW(mask1, mask2), restitutionMask);
            let mass = b2BlendW(c.normalMass2, zero, mask);

            // fixed anchors for Jacobians
            let rA = c.anchorA2;
            let rB = c.anchorB2;

            // Relative velocity at contact
            let dvx = b2SubW(b2SubW(bB.v.X, b2MulW(bB.w, rB.Y)), b2SubW(bA.v.X, b2MulW(bA.w, rA.Y)));
            let dvy = b2SubW(b2AddW(bB.v.Y, b2MulW(bB.w, rB.X)), b2AddW(bA.v.Y, b2MulW(bA.w, rA.X)));
            let vn = b2AddW(b2MulW(dvx, c.normal.X), b2MulW(dvy, c.normal.Y));

            // Compute normal impulse
            let negImpulse = b2MulW(mass, b2AddW(vn, b2MulW(c.restitution, c.relativeVelocity2)));

            // Clamp the accumulated impulse
            let newImpulse = b2MaxW(b2SubW(c.normalImpulse2, negImpulse), b2ZeroW());
            let deltaImpulse = b2SubW(newImpulse, c.normalImpulse2);
            c.normalImpulse2 = newImpulse;

            c.totalNormalImpulse2 = b2AddW(c.totalNormalImpulse2, deltaImpulse);

            // Apply contact impulse
            let Px = b2MulW(deltaImpulse, c.normal.X);
            let Py = b2MulW(deltaImpulse, c.normal.Y);

            bA.v.X = b2MulSubW(bA.v.X, c.invMassA, Px);
            bA.v.Y = b2MulSubW(bA.v.Y, c.invMassA, Py);
            bA.w = b2MulSubW(bA.w, c.invIA, b2SubW(b2MulW(rA.X, Py), b2MulW(rA.Y, Px)));

            bB.v.X = b2MulAddW(bB.v.X, c.invMassB, Px);
            bB.v.Y = b2MulAddW(bB.v.Y, c.invMassB, Py);
            bB.w = b2MulAddW(bB.w, c.invIB, b2SubW(b2MulW(rB.X, Py), b2MulW(rB.Y, Px)));
        }

        b2ScatterBodies(states, c.indexA, bA);
        b2ScatterBodies(states, c.indexB, bB);
    }
}

export function b2StoreImpulsesTask(startIndex: i32, endIndex: i32, context: b2StepContext): void {
    let contacts = context.contacts;
    let constraints = context.simdContactConstraints!;

    let dummy = new b2Manifold();

    for (let constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex) {
        let c = unchecked(constraints[constraintIndex]);
        let rollingImpulse = c.rollingImpulse;
        let normalImpulse1 = c.normalImpulse1;
        let normalImpulse2 = c.normalImpulse2;
        let tangentImpulse1 = c.tangentImpulse1;
        let tangentImpulse2 = c.tangentImpulse2;
        let totalNormalImpulse1 = c.totalNormalImpulse1;
        let totalNormalImpulse2 = c.totalNormalImpulse2;
        let normalVelocity1 = c.relativeVelocity1;
        let normalVelocity2 = c.relativeVelocity2;

        let baseIndex = B2_SIMD_WIDTH * constraintIndex;

        for (let laneIndex = 0; laneIndex < B2_SIMD_WIDTH; ++laneIndex) {
            let contactSim = unchecked(contacts[baseIndex + laneIndex]);
            let m = dummy;
            if (contactSim != null) {
                m = contactSim.manifold;
            }
            m.rollingImpulse = b2GetLaneW(rollingImpulse, laneIndex);

            unchecked(m.points[0].normalImpulse = b2GetLaneW(normalImpulse1, laneIndex));
            unchecked(m.points[0].tangentImpulse = b2GetLaneW(tangentImpulse1, laneIndex));
            unchecked(m.points[0].totalNormalImpulse = b2GetLaneW(totalNormalImpulse1, laneIndex));
            unchecked(m.points[0].normalVelocity = b2GetLaneW(normalVelocity1, laneIndex));

            unchecked(m.points[1].normalImpulse = b2GetLaneW(normalImpulse2, laneIndex));
            unchecked(m.points[1].tangentImpulse = b2GetLaneW(tangentImpulse2, laneIndex));
            unchecked(m.points[1].totalNormalImpulse = b2GetLaneW(totalNormalImpulse2, laneIndex));
            unchecked(m.points[1].normalVelocity = b2GetLaneW(normalVelocity2, laneIndex));
        }
    }
}
