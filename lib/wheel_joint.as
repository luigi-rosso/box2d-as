// Ported from box2d src/wheel_joint.c. Debug draw omitted.

import { B2_NULL_INDEX } from "./core";
import {
    b2Vec2, b2Add, b2Sub, b2MulSV, b2MulAdd, b2MulSub, b2Cross, b2Dot, b2RotateVector, b2MulRot,
    b2LeftPerp, b2ClampFloat, b2MaxFloat, b2MinFloat,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_wheelJoint } from "./types";
import { b2JointSim, b2GetJointSimCheckType } from "./joint";
import { b2StepContext, b2MakeSoft } from "./solver";
import { b2BodyState, b2_dynamicFlag, b2GetBodyTransform } from "./body";
import { b2World, b2GetWorld, b2_awakeSet } from "./physics_world";

export function b2WheelJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }

    if (enableSpring != joint.wheelJoint!.enableSpring) {
        joint.wheelJoint!.enableSpring = enableSpring;
        joint.wheelJoint!.springImpulse = 0.0;
    }
}

export function b2WheelJoint_IsSpringEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return false;
    }
    return joint.wheelJoint!.enableSpring;
}

export function b2WheelJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    joint.wheelJoint!.hertz = hertz;
}

export function b2WheelJoint_GetSpringHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.wheelJoint!.hertz;
}

export function b2WheelJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    joint.wheelJoint!.dampingRatio = dampingRatio;
}

export function b2WheelJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.wheelJoint!.dampingRatio;
}

export function b2WheelJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    if (joint.wheelJoint!.enableLimit != enableLimit) {
        joint.wheelJoint!.lowerImpulse = 0.0;
        joint.wheelJoint!.upperImpulse = 0.0;
        joint.wheelJoint!.enableLimit = enableLimit;
    }
}

export function b2WheelJoint_IsLimitEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return false;
    }
    return joint.wheelJoint!.enableLimit;
}

export function b2WheelJoint_GetLowerLimit(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.wheelJoint!.lowerTranslation;
}

export function b2WheelJoint_GetUpperLimit(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.wheelJoint!.upperTranslation;
}

export function b2WheelJoint_SetLimits(jointId: b2JointId, lower: f32, upper: f32): void {
    assert(lower <= upper);

    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    if (lower != joint.wheelJoint!.lowerTranslation || upper != joint.wheelJoint!.upperTranslation) {
        joint.wheelJoint!.lowerTranslation = b2MinFloat(lower, upper);
        joint.wheelJoint!.upperTranslation = b2MaxFloat(lower, upper);
        joint.wheelJoint!.lowerImpulse = 0.0;
        joint.wheelJoint!.upperImpulse = 0.0;
    }
}

export function b2WheelJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    if (joint.wheelJoint!.enableMotor != enableMotor) {
        joint.wheelJoint!.motorImpulse = 0.0;
        joint.wheelJoint!.enableMotor = enableMotor;
    }
}

export function b2WheelJoint_IsMotorEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return false;
    }
    return joint.wheelJoint!.enableMotor;
}

export function b2WheelJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    joint.wheelJoint!.motorSpeed = motorSpeed;
}

export function b2WheelJoint_GetMotorSpeed(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.wheelJoint!.motorSpeed;
}

export function b2WheelJoint_GetMotorTorque(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return world.inv_h * joint.wheelJoint!.motorImpulse;
}

export function b2WheelJoint_SetMaxMotorTorque(jointId: b2JointId, torque: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return;
    }
    joint.wheelJoint!.maxMotorTorque = torque;
}

export function b2WheelJoint_GetMaxMotorTorque(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_wheelJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.wheelJoint!.maxMotorTorque;
}

export function b2GetWheelJointForce(world: b2World, base: b2JointSim): b2Vec2 {
    let idA = base.bodyIdA;
    let transformA = b2GetBodyTransform(world, idA);

    let localAxisA = b2RotateVector(base.localFrameA.q, new b2Vec2(1.0, 0.0));
    let axisA = b2RotateVector(transformA.q, localAxisA);
    let perpA = b2LeftPerp(axisA);

    let joint = base.wheelJoint!;

    let perpForce = world.inv_h * joint.perpImpulse;
    let axialForce = world.inv_h * (joint.springImpulse + joint.lowerImpulse - joint.upperImpulse);

    let force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
    return force;
}

export function b2GetWheelJointTorque(world: b2World, base: b2JointSim): f32 {
    return world.inv_h * base.wheelJoint!.motorImpulse;
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

export function b2PrepareWheelJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_wheelJoint);

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

    let joint = base.wheelJoint!;

    joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
    joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

    // Compute joint anchor frames with world space rotation, relative to center of mass
    joint.frameA.q = b2MulRot(bodySimA.transform.q, base.localFrameA.q);
    joint.frameA.p = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
    joint.frameB.q = b2MulRot(bodySimB.transform.q, base.localFrameB.q);
    joint.frameB.p = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));

    // Compute the initial center delta. Incremental position updates are relative to this.
    joint.deltaCenter = b2Sub(bodySimB.center, bodySimA.center);

    let rA = joint.frameA.p;
    let rB = joint.frameB.p;

    let d = b2Add(joint.deltaCenter, b2Sub(rB, rA));
    let axisA = b2RotateVector(joint.frameA.q, new b2Vec2(1.0, 0.0));
    let perpA = b2LeftPerp(axisA);

    // perpendicular constraint (keep wheel on line)
    let s1 = b2Cross(b2Add(d, rA), perpA);
    let s2 = b2Cross(rB, perpA);

    let kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
    joint.perpMass = kp > 0.0 ? <f32>1.0 / kp : <f32>0.0;

    // spring constraint
    let a1 = b2Cross(b2Add(d, rA), axisA);
    let a2 = b2Cross(rB, axisA);

    let ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
    joint.axialMass = ka > 0.0 ? <f32>1.0 / ka : <f32>0.0;

    joint.springSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

    let km = iA + iB;
    joint.motorMass = km > 0.0 ? <f32>1.0 / km : <f32>0.0;

    if (context.enableWarmStarting == false) {
        joint.perpImpulse = 0.0;
        joint.springImpulse = 0.0;
        joint.motorImpulse = 0.0;
        joint.lowerImpulse = 0.0;
        joint.upperImpulse = 0.0;
    }
}

export function b2WarmStartWheelJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_wheelJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.wheelJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));
    let axisA = b2RotateVector(joint.frameA.q, new b2Vec2(1.0, 0.0));
    axisA = b2RotateVector(stateA.deltaRotation, axisA);
    let perpA = b2LeftPerp(axisA);

    let a1 = b2Cross(b2Add(d, rA), axisA);
    let a2 = b2Cross(rB, axisA);
    let s1 = b2Cross(b2Add(d, rA), perpA);
    let s2 = b2Cross(rB, perpA);

    let axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;

    let P = b2Add(b2MulSV(axialImpulse, axisA), b2MulSV(joint.perpImpulse, perpA));
    let LA = axialImpulse * a1 + joint.perpImpulse * s1 + joint.motorImpulse;
    let LB = axialImpulse * a2 + joint.perpImpulse * s2 + joint.motorImpulse;

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = b2MulSub(stateA.linearVelocity, mA, P);
        stateA.angularVelocity -= iA * LA;
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = b2MulAdd(stateB.linearVelocity, mB, P);
        stateB.angularVelocity += iB * LB;
    }
}

export function b2SolveWheelJoint(base: b2JointSim, context: b2StepContext, useBias: bool): void {
    assert(base.type == b2_wheelJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.wheelJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let vA = stateA.linearVelocity;
    let wA = stateA.angularVelocity;
    let vB = stateB.linearVelocity;
    let wB = stateB.angularVelocity;

    let fixedRotation = iA + iB == 0.0;

    // current anchors
    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));
    let axisA = b2RotateVector(joint.frameA.q, new b2Vec2(1.0, 0.0));
    axisA = b2RotateVector(stateA.deltaRotation, axisA);
    let translation = b2Dot(axisA, d);

    let a1 = b2Cross(b2Add(d, rA), axisA);
    let a2 = b2Cross(rB, axisA);

    // motor constraint
    if (joint.enableMotor && fixedRotation == false) {
        let Cdot = wB - wA - joint.motorSpeed;
        let impulse = -joint.motorMass * Cdot;
        let oldImpulse = joint.motorImpulse;
        let maxImpulse = context.h * joint.maxMotorTorque;
        joint.motorImpulse = b2ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = joint.motorImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    // spring constraint
    if (joint.enableSpring) {
        // This is a real spring and should be applied even during relax
        let C = translation;
        let bias = joint.springSoftness.biasRate * C;
        let massScale = joint.springSoftness.massScale;
        let impulseScale = joint.springSoftness.impulseScale;

        let Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
        let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.springImpulse;
        joint.springImpulse += impulse;

        let P = b2MulSV(impulse, axisA);
        let LA = impulse * a1;
        let LB = impulse * a2;

        vA = b2MulSub(vA, mA, P);
        wA -= iA * LA;
        vB = b2MulAdd(vB, mB, P);
        wB += iB * LB;
    }

    if (joint.enableLimit) {
        // Lower limit
        {
            let C = translation - joint.lowerTranslation;
            let bias: f32 = 0.0;
            let massScale: f32 = 1.0;
            let impulseScale: f32 = 0.0;

            if (C > 0.0) {
                // speculation
                bias = C * context.inv_h;
            } else if (useBias) {
                bias = base.constraintSoftness.biasRate * C;
                massScale = base.constraintSoftness.massScale;
                impulseScale = base.constraintSoftness.impulseScale;
            }

            let Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
            let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.lowerImpulse;
            let oldImpulse = joint.lowerImpulse;
            joint.lowerImpulse = b2MaxFloat(oldImpulse + impulse, 0.0);
            impulse = joint.lowerImpulse - oldImpulse;

            let P = b2MulSV(impulse, axisA);
            let LA = impulse * a1;
            let LB = impulse * a2;

            vA = b2MulSub(vA, mA, P);
            wA -= iA * LA;
            vB = b2MulAdd(vB, mB, P);
            wB += iB * LB;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
            // sign flipped
            let C = joint.upperTranslation - translation;
            let bias: f32 = 0.0;
            let massScale: f32 = 1.0;
            let impulseScale: f32 = 0.0;

            if (C > 0.0) {
                // speculation
                bias = C * context.inv_h;
            } else if (useBias) {
                bias = base.constraintSoftness.biasRate * C;
                massScale = base.constraintSoftness.massScale;
                impulseScale = base.constraintSoftness.impulseScale;
            }

            // sign flipped on Cdot
            let Cdot = b2Dot(axisA, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
            let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.upperImpulse;
            let oldImpulse = joint.upperImpulse;
            joint.upperImpulse = b2MaxFloat(oldImpulse + impulse, 0.0);
            impulse = joint.upperImpulse - oldImpulse;

            let P = b2MulSV(impulse, axisA);
            let LA = impulse * a1;
            let LB = impulse * a2;

            // sign flipped on applied impulse
            vA = b2MulAdd(vA, mA, P);
            wA += iA * LA;
            vB = b2MulSub(vB, mB, P);
            wB -= iB * LB;
        }
    }

    // point to line constraint
    {
        let perpA = b2LeftPerp(axisA);

        let bias: f32 = 0.0;
        let massScale: f32 = 1.0;
        let impulseScale: f32 = 0.0;
        if (useBias) {
            let C = b2Dot(perpA, d);
            bias = base.constraintSoftness.biasRate * C;
            massScale = base.constraintSoftness.massScale;
            impulseScale = base.constraintSoftness.impulseScale;
        }

        let s1 = b2Cross(b2Add(d, rA), perpA);
        let s2 = b2Cross(rB, perpA);
        let Cdot = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;

        let impulse = -massScale * joint.perpMass * (Cdot + bias) - impulseScale * joint.perpImpulse;
        joint.perpImpulse += impulse;

        let P = b2MulSV(impulse, perpA);
        let LA = impulse * s1;
        let LB = impulse * s2;

        vA = b2MulSub(vA, mA, P);
        wA -= iA * LA;
        vB = b2MulAdd(vB, mB, P);
        wB += iB * LB;
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
