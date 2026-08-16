// Ported from box2d src/distance_joint.c. Debug draw omitted.

import { B2_NULL_INDEX } from "./core";
import { B2_LINEAR_SLOP, B2_HUGE } from "./constants";
import {
    b2Vec2, b2Add, b2Sub, b2Dot, b2Cross, b2CrossSV, b2Length, b2Normalize, b2MulSV, b2MulAdd,
    b2MulSub, b2RotateVector, b2TransformPoint, b2ClampFloat, b2MinFloat, b2MaxFloat,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_distanceJoint } from "./types";
import { b2World, b2GetWorld, b2_awakeSet } from "./physics_world";
import { b2BodyState, b2_dynamicFlag, b2GetBodyTransform } from "./body";
import { b2StepContext, b2MakeSoft } from "./solver";
import { b2JointSim, b2GetJointSimCheckType } from "./joint";

export function b2DistanceJoint_SetLength(jointId: b2JointId, length: f32): void {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    let joint = base.distanceJoint!;

    joint.length = b2ClampFloat(length, B2_LINEAR_SLOP(), B2_HUGE());
    joint.impulse = 0.0;
    joint.lowerImpulse = 0.0;
    joint.upperImpulse = 0.0;
}

export function b2DistanceJoint_GetLength(jointId: b2JointId): f32 {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }
    let joint = base.distanceJoint!;
    return joint.length;
}

export function b2DistanceJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    let joint = base.distanceJoint!;
    joint.enableLimit = enableLimit;
}

export function b2DistanceJoint_IsLimitEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return false;
    }
    return joint.distanceJoint!.enableLimit;
}

export function b2DistanceJoint_SetLengthRange(jointId: b2JointId, minLength: f32, maxLength: f32): void {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    let joint = base.distanceJoint!;

    minLength = b2ClampFloat(minLength, B2_LINEAR_SLOP(), B2_HUGE());
    maxLength = b2ClampFloat(maxLength, B2_LINEAR_SLOP(), B2_HUGE());
    joint.minLength = b2MinFloat(minLength, maxLength);
    joint.maxLength = b2MaxFloat(minLength, maxLength);
    joint.impulse = 0.0;
    joint.lowerImpulse = 0.0;
    joint.upperImpulse = 0.0;
}

export function b2DistanceJoint_GetMinLength(jointId: b2JointId): f32 {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }
    let joint = base.distanceJoint!;
    return joint.minLength;
}

export function b2DistanceJoint_GetMaxLength(jointId: b2JointId): f32 {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }
    let joint = base.distanceJoint!;
    return joint.maxLength;
}

export function b2DistanceJoint_GetCurrentLength(jointId: b2JointId): f32 {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }

    let world = b2GetWorld(jointId.world0);
    assert(world.locked == false);
    if (world.locked) {
        return 0.0;
    }

    let transformA = b2GetBodyTransform(world, base.bodyIdA);
    let transformB = b2GetBodyTransform(world, base.bodyIdB);

    let pA = b2TransformPoint(transformA, base.localFrameA.p);
    let pB = b2TransformPoint(transformB, base.localFrameB.p);
    let d = b2Sub(pB, pA);
    let length = b2Length(d);
    return length;
}

export function b2DistanceJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    base.distanceJoint!.enableSpring = enableSpring;
}

export function b2DistanceJoint_IsSpringEnabled(jointId: b2JointId): bool {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return false;
    }
    return base.distanceJoint!.enableSpring;
}

export function b2DistanceJoint_SetSpringForceRange(jointId: b2JointId, lowerForce: f32, upperForce: f32): void {
    assert(lowerForce <= upperForce);
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    base.distanceJoint!.lowerSpringForce = lowerForce;
    base.distanceJoint!.upperSpringForce = upperForce;
}

// C returns the range through float pointers.
export class b2SpringForceRange {
    lowerForce: f32 = 0;
    upperForce: f32 = 0;
}

export function b2DistanceJoint_GetSpringForceRange(jointId: b2JointId): b2SpringForceRange {
    let out = new b2SpringForceRange();
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return out;
    }
    out.lowerForce = base.distanceJoint!.lowerSpringForce;
    out.upperForce = base.distanceJoint!.upperSpringForce;
    return out;
}

export function b2DistanceJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    base.distanceJoint!.hertz = hertz;
}

export function b2DistanceJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return;
    }
    base.distanceJoint!.dampingRatio = dampingRatio;
}

export function b2DistanceJoint_GetSpringHertz(jointId: b2JointId): f32 {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }
    let joint = base.distanceJoint!;
    return joint.hertz;
}

export function b2DistanceJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }
    let joint = base.distanceJoint!;
    return joint.dampingRatio;
}

export function b2DistanceJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return;
    }
    if (enableMotor != joint.distanceJoint!.enableMotor) {
        joint.distanceJoint!.enableMotor = enableMotor;
        joint.distanceJoint!.motorImpulse = 0.0;
    }
}

export function b2DistanceJoint_IsMotorEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return false;
    }
    return joint.distanceJoint!.enableMotor;
}

export function b2DistanceJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return;
    }
    joint.distanceJoint!.motorSpeed = motorSpeed;
}

export function b2DistanceJoint_GetMotorSpeed(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.distanceJoint!.motorSpeed;
}

export function b2DistanceJoint_GetMotorForce(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (base == null) {
        return 0.0;
    }
    return world.inv_h * base.distanceJoint!.motorImpulse;
}

export function b2DistanceJoint_SetMaxMotorForce(jointId: b2JointId, force: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return;
    }
    joint.distanceJoint!.maxMotorForce = force;
}

export function b2DistanceJoint_GetMaxMotorForce(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_distanceJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.distanceJoint!.maxMotorForce;
}

export function b2GetDistanceJointForce(world: b2World, base: b2JointSim): b2Vec2 {
    let joint = base.distanceJoint!;

    let transformA = b2GetBodyTransform(world, base.bodyIdA);
    let transformB = b2GetBodyTransform(world, base.bodyIdB);

    let pA = b2TransformPoint(transformA, base.localFrameA.p);
    let pB = b2TransformPoint(transformB, base.localFrameB.p);
    let d = b2Sub(pB, pA);
    let axis = b2Normalize(d);
    let force = (joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse) * world.inv_h;
    return b2MulSV(force, axis);
}

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

export function b2PrepareDistanceJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_distanceJoint);

    // chase body id to the solver set where the body lives
    let idA = base.bodyIdA;
    let idB = base.bodyIdB;

    let world = context.world!;
    let bodyA = world.bodies.get(idA);
    let bodyB = world.bodies.get(idB);

    assert(bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet);

    let setA = world.solverSets.get(bodyA.setIndex);
    let setB = world.solverSets.get(bodyB.setIndex);

    let localIndexA = bodyA.localIndex;
    let localIndexB = bodyB.localIndex;

    let bodySimA = setA.bodySims.get(localIndexA);
    let bodySimB = setB.bodySims.get(localIndexB);

    let mA = bodySimA.invMass;
    let iA = bodySimA.invInertia;
    let mB = bodySimB.invMass;
    let iB = bodySimB.invInertia;

    base.invMassA = mA;
    base.invMassB = mB;
    base.invIA = iA;
    base.invIB = iB;

    let joint = base.distanceJoint!;

    joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
    joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

    // initial anchors in world space
    joint.anchorA = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
    joint.anchorB = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));
    joint.deltaCenter = b2Sub(bodySimB.center, bodySimA.center);

    let rA = joint.anchorA;
    let rB = joint.anchorB;
    let separation = b2Add(b2Sub(rB, rA), joint.deltaCenter);
    let axis = b2Normalize(separation);

    // compute effective mass
    let crA = b2Cross(rA, axis);
    let crB = b2Cross(rB, axis);
    let k = mA + mB + iA * crA * crA + iB * crB * crB;
    joint.axialMass = k > 0.0 ? <f32>1.0 / k : <f32>0.0;

    joint.distanceSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

    if (context.enableWarmStarting == false) {
        joint.impulse = 0.0;
        joint.lowerImpulse = 0.0;
        joint.upperImpulse = 0.0;
        joint.motorImpulse = 0.0;
    }
}

export function b2WarmStartDistanceJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_distanceJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.distanceJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let rA = b2RotateVector(stateA.deltaRotation, joint.anchorA);
    let rB = b2RotateVector(stateB.deltaRotation, joint.anchorB);

    let ds = b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), b2Sub(rB, rA));
    let separation = b2Add(joint.deltaCenter, ds);
    let axis = b2Normalize(separation);

    let axialImpulse = joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse;
    let P = b2MulSV(axialImpulse, axis);

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = b2MulSub(stateA.linearVelocity, mA, P);
        stateA.angularVelocity -= iA * b2Cross(rA, P);
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = b2MulAdd(stateB.linearVelocity, mB, P);
        stateB.angularVelocity += iB * b2Cross(rB, P);
    }
}

export function b2SolveDistanceJoint(base: b2JointSim, context: b2StepContext, useBias: bool): void {
    assert(base.type == b2_distanceJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.distanceJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let vA = stateA.linearVelocity;
    let wA = stateA.angularVelocity;
    let vB = stateB.linearVelocity;
    let wB = stateB.angularVelocity;

    // current anchors
    let rA = b2RotateVector(stateA.deltaRotation, joint.anchorA);
    let rB = b2RotateVector(stateB.deltaRotation, joint.anchorB);

    // current separation
    let ds = b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), b2Sub(rB, rA));
    let separation = b2Add(joint.deltaCenter, ds);

    let length = b2Length(separation);
    let axis = b2Normalize(separation);

    // joint is soft if
    // - spring is enabled
    // - and (joint limit is disabled or limits are not equal)
    if (joint.enableSpring && (joint.minLength < joint.maxLength || joint.enableLimit == false)) {
        // spring
        if (joint.hertz > 0.0) {
            // Cdot = dot(u, v + cross(w, r))
            let vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
            let Cdot = b2Dot(axis, vr);
            let C = length - joint.length;
            let bias = joint.distanceSoftness.biasRate * C;

            let m = joint.distanceSoftness.massScale * joint.axialMass;
            let oldImpulse = joint.impulse;
            let impulse = -m * (Cdot + bias) - joint.distanceSoftness.impulseScale * oldImpulse;

            let h = context.h;
            joint.impulse = b2ClampFloat(joint.impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h);
            impulse = joint.impulse - oldImpulse;

            let P = b2MulSV(impulse, axis);
            vA = b2MulSub(vA, mA, P);
            wA -= iA * b2Cross(rA, P);
            vB = b2MulAdd(vB, mB, P);
            wB += iB * b2Cross(rB, P);
        }

        if (joint.enableMotor) {
            let vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
            let Cdot = b2Dot(axis, vr);
            let impulse = joint.axialMass * (joint.motorSpeed - Cdot);
            let oldImpulse = joint.motorImpulse;
            let maxImpulse = context.h * joint.maxMotorForce;
            joint.motorImpulse = b2ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = joint.motorImpulse - oldImpulse;

            let P = b2MulSV(impulse, axis);
            vA = b2MulSub(vA, mA, P);
            wA -= iA * b2Cross(rA, P);
            vB = b2MulAdd(vB, mB, P);
            wB += iB * b2Cross(rB, P);
        }

        if (joint.enableLimit) {
            // lower limit
            {
                let vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
                let Cdot = b2Dot(axis, vr);

                let C = length - joint.minLength;

                let bias: f32 = 0.0;
                let massCoeff: f32 = 1.0;
                let impulseCoeff: f32 = 0.0;
                if (C > 0.0) {
                    // speculative
                    bias = C * context.inv_h;
                } else if (useBias) {
                    bias = base.constraintSoftness.biasRate * C;
                    massCoeff = base.constraintSoftness.massScale;
                    impulseCoeff = base.constraintSoftness.impulseScale;
                }

                let impulse = -massCoeff * joint.axialMass * (Cdot + bias) - impulseCoeff * joint.lowerImpulse;
                let newImpulse = b2MaxFloat(0.0, joint.lowerImpulse + impulse);
                impulse = newImpulse - joint.lowerImpulse;
                joint.lowerImpulse = newImpulse;

                let P = b2MulSV(impulse, axis);
                vA = b2MulSub(vA, mA, P);
                wA -= iA * b2Cross(rA, P);
                vB = b2MulAdd(vB, mB, P);
                wB += iB * b2Cross(rB, P);
            }

            // upper
            {
                let vr = b2Add(b2Sub(vA, vB), b2Sub(b2CrossSV(wA, rA), b2CrossSV(wB, rB)));
                let Cdot = b2Dot(axis, vr);

                let C = joint.maxLength - length;

                let bias: f32 = 0.0;
                let massScale: f32 = 1.0;
                let impulseScale: f32 = 0.0;
                if (C > 0.0) {
                    // speculative
                    bias = C * context.inv_h;
                } else if (useBias) {
                    bias = base.constraintSoftness.biasRate * C;
                    massScale = base.constraintSoftness.massScale;
                    impulseScale = base.constraintSoftness.impulseScale;
                }

                let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.upperImpulse;
                let newImpulse = b2MaxFloat(0.0, joint.upperImpulse + impulse);
                impulse = newImpulse - joint.upperImpulse;
                joint.upperImpulse = newImpulse;

                let P = b2MulSV(-impulse, axis);
                vA = b2MulSub(vA, mA, P);
                wA -= iA * b2Cross(rA, P);
                vB = b2MulAdd(vB, mB, P);
                wB += iB * b2Cross(rB, P);
            }
        }
    } else {
        // rigid constraint
        let vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
        let Cdot = b2Dot(axis, vr);

        let C = length - joint.length;

        let bias: f32 = 0.0;
        let massScale: f32 = 1.0;
        let impulseScale: f32 = 0.0;
        if (useBias) {
            bias = base.constraintSoftness.biasRate * C;
            massScale = base.constraintSoftness.massScale;
            impulseScale = base.constraintSoftness.impulseScale;
        }

        let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.impulse;
        joint.impulse += impulse;

        let P = b2MulSV(impulse, axis);
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
