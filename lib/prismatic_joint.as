// Ported from box2d src/prismatic_joint.c. Debug draw omitted.

import { B2_NULL_INDEX, b2_lengthUnitsPerMeter } from "./core";
import {
    b2Vec2, b2Mat22, b2Vec2_zero, b2Add, b2Sub, b2Dot, b2Cross, b2CrossSV, b2MulSV, b2MulAdd,
    b2MulSub, b2LeftPerp, b2RotateVector, b2TransformPoint, b2MulRot, b2InvMulRot, b2Rot_GetAngle,
    b2Solve22, b2MinFloat, b2MaxFloat, b2ClampFloat, b2IsValidVec2, b2IsValidFloat,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_prismaticJoint } from "./types";
import { b2World, b2GetWorld, b2_awakeSet } from "./physics_world";
import { b2BodyState, b2GetBodyTransform, b2GetBodySim, b2GetBodyState, b2_dynamicFlag } from "./body";
import { b2JointSim, b2GetJointSimCheckType, b2GetJointFullId, b2GetJointSim } from "./joint";
import { b2StepContext, b2MakeSoft } from "./solver";

export function b2PrismaticJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    if (enableSpring != joint.prismaticJoint!.enableSpring) {
        joint.prismaticJoint!.enableSpring = enableSpring;
        joint.prismaticJoint!.springImpulse = 0.0;
    }
}

export function b2PrismaticJoint_IsSpringEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return false;
    }
    return joint.prismaticJoint!.enableSpring;
}

export function b2PrismaticJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    joint.prismaticJoint!.hertz = hertz;
}

export function b2PrismaticJoint_GetSpringHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.hertz;
}

export function b2PrismaticJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    joint.prismaticJoint!.dampingRatio = dampingRatio;
}

export function b2PrismaticJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.dampingRatio;
}

export function b2PrismaticJoint_SetTargetTranslation(jointId: b2JointId, translation: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    joint.prismaticJoint!.targetTranslation = translation;
}

export function b2PrismaticJoint_GetTargetTranslation(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.targetTranslation;
}

export function b2PrismaticJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    if (enableLimit != joint.prismaticJoint!.enableLimit) {
        joint.prismaticJoint!.enableLimit = enableLimit;
        joint.prismaticJoint!.lowerImpulse = 0.0;
        joint.prismaticJoint!.upperImpulse = 0.0;
    }
}

export function b2PrismaticJoint_IsLimitEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return false;
    }
    return joint.prismaticJoint!.enableLimit;
}

export function b2PrismaticJoint_GetLowerLimit(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.lowerTranslation;
}

export function b2PrismaticJoint_GetUpperLimit(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.upperTranslation;
}

export function b2PrismaticJoint_SetLimits(jointId: b2JointId, lower: f32, upper: f32): void {
    assert(lower <= upper);

    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    if (lower != joint.prismaticJoint!.lowerTranslation || upper != joint.prismaticJoint!.upperTranslation) {
        joint.prismaticJoint!.lowerTranslation = b2MinFloat(lower, upper);
        joint.prismaticJoint!.upperTranslation = b2MaxFloat(lower, upper);
        joint.prismaticJoint!.lowerImpulse = 0.0;
        joint.prismaticJoint!.upperImpulse = 0.0;
    }
}

export function b2PrismaticJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    if (enableMotor != joint.prismaticJoint!.enableMotor) {
        joint.prismaticJoint!.enableMotor = enableMotor;
        joint.prismaticJoint!.motorImpulse = 0.0;
    }
}

export function b2PrismaticJoint_IsMotorEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return false;
    }
    return joint.prismaticJoint!.enableMotor;
}

export function b2PrismaticJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    joint.prismaticJoint!.motorSpeed = motorSpeed;
}

export function b2PrismaticJoint_GetMotorSpeed(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.motorSpeed;
}

export function b2PrismaticJoint_GetMotorForce(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let base = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (base == null) {
        return 0.0;
    }
    return world.inv_h * base.prismaticJoint!.motorImpulse;
}

export function b2PrismaticJoint_SetMaxMotorForce(jointId: b2JointId, force: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return;
    }
    joint.prismaticJoint!.maxMotorForce = force;
}

export function b2PrismaticJoint_GetMaxMotorForce(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.prismaticJoint!.maxMotorForce;
}

export function b2PrismaticJoint_GetTranslation(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let jointSim = b2GetJointSimCheckType(jointId, b2_prismaticJoint);
    if (jointSim == null) {
        return 0.0;
    }
    let transformA = b2GetBodyTransform(world, jointSim.bodyIdA);
    let transformB = b2GetBodyTransform(world, jointSim.bodyIdB);

    let localAxisA = b2RotateVector(jointSim.localFrameA.q, new b2Vec2(1.0, 0.0));
    let axisA = b2RotateVector(transformA.q, localAxisA);
    let pA = b2TransformPoint(transformA, jointSim.localFrameA.p);
    let pB = b2TransformPoint(transformB, jointSim.localFrameB.p);
    let d = b2Sub(pB, pA);
    let translation = b2Dot(d, axisA);
    return translation;
}

export function b2PrismaticJoint_GetSpeed(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    assert(joint.type == b2_prismaticJoint);
    let base = b2GetJointSim(world, joint);
    assert(base.type == b2_prismaticJoint);

    let bodyA = world.bodies.get(base.bodyIdA);
    let bodyB = world.bodies.get(base.bodyIdB);
    let bodySimA = b2GetBodySim(world, bodyA);
    let bodySimB = b2GetBodySim(world, bodyB);
    let bodyStateA = b2GetBodyState(world, bodyA);
    let bodyStateB = b2GetBodyState(world, bodyB);

    let transformA = bodySimA.transform;
    let transformB = bodySimB.transform;

    let localAxisA = b2RotateVector(base.localFrameA.q, new b2Vec2(1.0, 0.0));
    let axisA = b2RotateVector(transformA.q, localAxisA);
    let cA = bodySimA.center;
    let cB = bodySimB.center;
    let rA = b2RotateVector(transformA.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
    let rB = b2RotateVector(transformB.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));

    let d = b2Add(b2Sub(cB, cA), b2Sub(rB, rA));

    let vA = bodyStateA != null ? bodyStateA!.linearVelocity : b2Vec2_zero();
    let vB = bodyStateB != null ? bodyStateB!.linearVelocity : b2Vec2_zero();
    let wA: f32 = bodyStateA != null ? bodyStateA!.angularVelocity : 0.0;
    let wB: f32 = bodyStateB != null ? bodyStateB!.angularVelocity : 0.0;

    let vRel = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
    let speed = b2Dot(d, b2CrossSV(wA, axisA)) + b2Dot(axisA, vRel);
    return speed;
}

export function b2GetPrismaticJointForce(world: b2World, base: b2JointSim): b2Vec2 {
    let idA = base.bodyIdA;
    let transformA = b2GetBodyTransform(world, idA);

    let joint = base.prismaticJoint!;

    let localAxisA = b2RotateVector(base.localFrameA.q, new b2Vec2(1.0, 0.0));
    let axisA = b2RotateVector(transformA.q, localAxisA);
    let perpA = b2LeftPerp(axisA);

    let inv_h = world.inv_h;
    let perpForce = inv_h * joint.impulse.x;
    let axialForce = inv_h * (joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse);

    let force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
    return force;
}

export function b2GetPrismaticJointTorque(world: b2World, base: b2JointSim): f32 {
    return world.inv_h * base.prismaticJoint!.impulse.y;
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(perp, d)
// Cdot = dot(d, cross(wA, perp)) + dot(perp, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-perp, -cross(rA + d, perp), perp, cross(rB, perp)]
//
// Angular constraint
// C = aB - aA + a_initial
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// Motor/Limit linear constraint
// C = dot(axA, d)
// J = [-axA -cross(rA + d, axA) axA cross(rB, ax1)]
//
// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0, so: Cdot + max(C1, 0)/h >= 0
//
// Block Solver
// The block solver includes the angular and linear constraints. This makes the limit stiffer.
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

export function b2PreparePrismaticJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_prismaticJoint);

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

    let joint = base.prismaticJoint!;
    joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
    joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

    // Compute joint anchor frames with world space rotation, relative to center of mass
    joint.frameA.q = b2MulRot(bodySimA.transform.q, base.localFrameA.q);
    joint.frameA.p = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
    joint.frameB.q = b2MulRot(bodySimB.transform.q, base.localFrameB.q);
    joint.frameB.p = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));

    // Compute the initial center delta. Incremental position updates are relative to this.
    joint.deltaCenter = b2Sub(bodySimB.center, bodySimA.center);

    joint.springSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

    if (context.enableWarmStarting == false) {
        joint.impulse = b2Vec2_zero();
        joint.springImpulse = 0.0;
        joint.motorImpulse = 0.0;
        joint.lowerImpulse = 0.0;
        joint.upperImpulse = 0.0;
    }
}

export function b2WarmStartPrismaticJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_prismaticJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.prismaticJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));

    let axisA = b2RotateVector(joint.frameA.q, new b2Vec2(1.0, 0.0));
    axisA = b2RotateVector(stateA.deltaRotation, axisA);

    // impulse is applied at anchor point on body B
    let a1 = b2Cross(b2Add(rA, d), axisA);
    let a2 = b2Cross(rB, axisA);
    let axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

    // perpendicular constraint
    let perpA = b2LeftPerp(axisA);
    let s1 = b2Cross(b2Add(rA, d), perpA);
    let s2 = b2Cross(rB, perpA);
    let perpImpulse = joint.impulse.x;
    let angleImpulse = joint.impulse.y;

    let P = b2Add(b2MulSV(axialImpulse, axisA), b2MulSV(perpImpulse, perpA));
    let LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
    let LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = b2MulSub(stateA.linearVelocity, mA, P);
        stateA.angularVelocity -= iA * LA;
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = b2MulAdd(stateB.linearVelocity, mB, P);
        stateB.angularVelocity += iB * LB;
    }
}

export function b2SolvePrismaticJoint(base: b2JointSim, context: b2StepContext, useBias: bool): void {
    assert(base.type == b2_prismaticJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.prismaticJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let vA = stateA.linearVelocity;
    let wA = stateA.angularVelocity;
    let vB = stateB.linearVelocity;
    let wB = stateB.angularVelocity;

    let qA = b2MulRot(stateA.deltaRotation, joint.frameA.q);
    let qB = b2MulRot(stateB.deltaRotation, joint.frameB.q);
    let relQ = b2InvMulRot(qA, qB);

    // current anchors
    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));

    let axisA = b2RotateVector(joint.frameA.q, new b2Vec2(1.0, 0.0));
    axisA = b2RotateVector(stateA.deltaRotation, axisA);
    let translation = b2Dot(axisA, d);

    // These scalars are for torques generated by axial forces
    let a1 = b2Cross(b2Add(rA, d), axisA);
    let a2 = b2Cross(rB, axisA);

    let k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
    let axialMass = k > 0.0 ? <f32>1.0 / k : <f32>0.0;

    let softness = base.constraintSoftness;

    // spring constraint
    if (joint.enableSpring) {
        // This is a real spring and should be applied even during relax
        let C = translation - joint.targetTranslation;
        let bias = joint.springSoftness.biasRate * C;
        let massScale = joint.springSoftness.massScale;
        let impulseScale = joint.springSoftness.impulseScale;

        let Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
        let deltaImpulse = -massScale * axialMass * (Cdot + bias) - impulseScale * joint.springImpulse;
        joint.springImpulse += deltaImpulse;

        let P = b2MulSV(deltaImpulse, axisA);
        let LA = deltaImpulse * a1;
        let LB = deltaImpulse * a2;

        vA = b2MulSub(vA, mA, P);
        wA -= iA * LA;
        vB = b2MulAdd(vB, mB, P);
        wB += iB * LB;
    }

    // Solve motor constraint
    if (joint.enableMotor) {
        let Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
        let impulse = axialMass * (joint.motorSpeed - Cdot);
        let oldImpulse = joint.motorImpulse;
        let maxImpulse = context.h * joint.maxMotorForce;
        joint.motorImpulse = b2ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = joint.motorImpulse - oldImpulse;

        let P = b2MulSV(impulse, axisA);
        let LA = impulse * a1;
        let LB = impulse * a2;

        vA = b2MulSub(vA, mA, P);
        wA -= iA * LA;
        vB = b2MulAdd(vB, mB, P);
        wB += iB * LB;
    }

    if (joint.enableLimit) {
        // Clamp the speculative distance to a reasonable value
        let speculativeDistance = <f32>0.25 * (joint.upperTranslation - joint.lowerTranslation);

        // Lower limit
        {
            let C = translation - joint.lowerTranslation;

            if (C < speculativeDistance) {
                let bias: f32 = 0.0;
                let massScale: f32 = 1.0;
                let impulseScale: f32 = 0.0;

                if (C > 0.0) {
                    // speculation
                    let safe: f32 = b2_lengthUnitsPerMeter;
                    bias = b2MinFloat(C, safe) * context.inv_h;
                } else if (useBias) {
                    bias = softness.biasRate * C;
                    massScale = softness.massScale;
                    impulseScale = softness.impulseScale;
                }

                let oldImpulse = joint.lowerImpulse;
                let Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
                let deltaImpulse = -axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                joint.lowerImpulse = b2MaxFloat(oldImpulse + deltaImpulse, 0.0);
                deltaImpulse = joint.lowerImpulse - oldImpulse;

                let P = b2MulSV(deltaImpulse, axisA);
                let LA = deltaImpulse * a1;
                let LB = deltaImpulse * a2;

                vA = b2MulSub(vA, mA, P);
                wA -= iA * LA;
                vB = b2MulAdd(vB, mB, P);
                wB += iB * LB;
            } else {
                joint.lowerImpulse = 0.0;
            }
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
            // sign flipped
            let C = joint.upperTranslation - translation;

            if (C < speculativeDistance) {
                let bias: f32 = 0.0;
                let massScale: f32 = 1.0;
                let impulseScale: f32 = 0.0;

                if (C > 0.0) {
                    // speculation
                    let safe: f32 = b2_lengthUnitsPerMeter;
                    bias = b2MinFloat(C, safe) * context.inv_h;
                } else if (useBias) {
                    bias = softness.biasRate * C;
                    massScale = softness.massScale;
                    impulseScale = softness.impulseScale;
                }

                let oldImpulse = joint.upperImpulse;

                // sign flipped
                let Cdot = b2Dot(axisA, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
                let deltaImpulse = -axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                joint.upperImpulse = b2MaxFloat(oldImpulse + deltaImpulse, 0.0);
                deltaImpulse = joint.upperImpulse - oldImpulse;

                let P = b2MulSV(deltaImpulse, axisA);
                let LA = deltaImpulse * a1;
                let LB = deltaImpulse * a2;

                // sign flipped
                vA = b2MulAdd(vA, mA, P);
                wA += iA * LA;
                vB = b2MulSub(vB, mB, P);
                wB -= iB * LB;
            } else {
                joint.upperImpulse = 0.0;
            }
        }
    }

    // Solve the prismatic constraint in block form
    {
        let perpA = b2LeftPerp(axisA);

        // These scalars are for torques generated by the perpendicular constraint force
        let s1 = b2Cross(b2Add(d, rA), perpA);
        let s2 = b2Cross(rB, perpA);

        let Cdot = new b2Vec2();
        Cdot.x = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;
        Cdot.y = wB - wA;

        let bias = b2Vec2_zero();
        let massScale: f32 = 1.0;
        let impulseScale: f32 = 0.0;
        if (useBias) {
            let C = new b2Vec2();
            C.x = b2Dot(perpA, d);
            C.y = b2Rot_GetAngle(relQ);

            bias = b2MulSV(softness.biasRate, C);
            massScale = softness.massScale;
            impulseScale = softness.impulseScale;
        }

        let k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        let k12 = iA * s1 + iB * s2;
        let k22 = iA + iB;
        if (k22 == 0.0) {
            // For bodies with fixed rotation.
            k22 = 1.0;
        }

        let K = new b2Mat22(new b2Vec2(k11, k12), new b2Vec2(k12, k22));

        let b = b2Solve22(K, b2Add(Cdot, bias));
        let deltaImpulse = new b2Vec2();
        deltaImpulse.x = -massScale * b.x - impulseScale * joint.impulse.x;
        deltaImpulse.y = -massScale * b.y - impulseScale * joint.impulse.y;

        joint.impulse.x += deltaImpulse.x;
        joint.impulse.y += deltaImpulse.y;

        let P = b2MulSV(deltaImpulse.x, perpA);
        let LA = deltaImpulse.x * s1 + deltaImpulse.y;
        let LB = deltaImpulse.x * s2 + deltaImpulse.y;

        vA = b2MulSub(vA, mA, P);
        wA -= iA * LA;
        vB = b2MulAdd(vB, mB, P);
        wB += iB * LB;
    }

    assert(b2IsValidVec2(vA));
    assert(b2IsValidFloat(wA));
    assert(b2IsValidVec2(vB));
    assert(b2IsValidFloat(wB));

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = vA;
        stateA.angularVelocity = wA;
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = vB;
        stateB.angularVelocity = wB;
    }
}
