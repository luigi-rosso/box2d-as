// Ported from box2d src/revolute_joint.c. Debug draw omitted.

// Point-to-point constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// J = [-E -skew(rA) E skew(rB) ]

// Motor constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
// K = invIA + invIB

import { B2_NULL_INDEX } from "./core";
import {
    b2Vec2, b2Vec2_zero, b2Mat22, b2Add, b2Sub, b2Cross, b2CrossSV, b2MulSV, b2MulAdd, b2MulSub,
    b2RotateVector, b2MulRot, b2InvMulRot, b2Rot_GetAngle, b2RelativeAngle, b2UnwindAngle,
    b2ClampFloat, b2MinFloat, b2MaxFloat, b2Solve22, B2_PI,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_revoluteJoint } from "./types";
import { b2World, b2GetWorld, b2_awakeSet } from "./physics_world";
import { b2BodyState, b2_dynamicFlag, b2GetBodyTransform } from "./body";
import { b2StepContext, b2MakeSoft } from "./solver";
import { b2JointSim, b2GetJointSimCheckType } from "./joint";

export function b2RevoluteJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    if (enableSpring != joint.revoluteJoint!.enableSpring) {
        joint.revoluteJoint!.enableSpring = enableSpring;
        joint.revoluteJoint!.springImpulse = 0.0;
    }
}

export function b2RevoluteJoint_IsSpringEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return false;
    }
    return joint.revoluteJoint!.enableSpring;
}

export function b2RevoluteJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    joint.revoluteJoint!.hertz = hertz;
}

export function b2RevoluteJoint_GetSpringHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.hertz;
}

export function b2RevoluteJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    joint.revoluteJoint!.dampingRatio = dampingRatio;
}

export function b2RevoluteJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.dampingRatio;
}

export function b2RevoluteJoint_SetTargetAngle(jointId: b2JointId, angle: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    joint.revoluteJoint!.targetAngle = angle;
}

export function b2RevoluteJoint_GetTargetAngle(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.targetAngle;
}

export function b2RevoluteJoint_GetAngle(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let jointSim = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (jointSim == null) {
        return 0.0;
    }
    let transformA = b2GetBodyTransform(world, jointSim.bodyIdA);
    let transformB = b2GetBodyTransform(world, jointSim.bodyIdB);
    let qA = b2MulRot(transformA.q, jointSim.localFrameA.q);
    let qB = b2MulRot(transformB.q, jointSim.localFrameB.q);

    let angle = b2RelativeAngle(qA, qB);
    return angle;
}

export function b2RevoluteJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    if (enableLimit != joint.revoluteJoint!.enableLimit) {
        joint.revoluteJoint!.enableLimit = enableLimit;
        joint.revoluteJoint!.lowerImpulse = 0.0;
        joint.revoluteJoint!.upperImpulse = 0.0;
    }
}

export function b2RevoluteJoint_IsLimitEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return false;
    }
    return joint.revoluteJoint!.enableLimit;
}

export function b2RevoluteJoint_GetLowerLimit(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.lowerAngle;
}

export function b2RevoluteJoint_GetUpperLimit(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.upperAngle;
}

export function b2RevoluteJoint_SetLimits(jointId: b2JointId, lower: f32, upper: f32): void {
    assert(lower <= upper);
    assert(lower >= <f32>-0.99 * B2_PI);
    assert(upper <= <f32>0.99 * B2_PI);

    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    if (lower != joint.revoluteJoint!.lowerAngle || upper != joint.revoluteJoint!.upperAngle) {
        joint.revoluteJoint!.lowerAngle = b2MinFloat(lower, upper);
        joint.revoluteJoint!.upperAngle = b2MaxFloat(lower, upper);
        joint.revoluteJoint!.lowerImpulse = 0.0;
        joint.revoluteJoint!.upperImpulse = 0.0;
    }
}

export function b2RevoluteJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    if (enableMotor != joint.revoluteJoint!.enableMotor) {
        joint.revoluteJoint!.enableMotor = enableMotor;
        joint.revoluteJoint!.motorImpulse = 0.0;
    }
}

export function b2RevoluteJoint_IsMotorEnabled(jointId: b2JointId): bool {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return false;
    }
    return joint.revoluteJoint!.enableMotor;
}

export function b2RevoluteJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    joint.revoluteJoint!.motorSpeed = motorSpeed;
}

export function b2RevoluteJoint_GetMotorSpeed(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.motorSpeed;
}

export function b2RevoluteJoint_GetMotorTorque(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return world.inv_h * joint.revoluteJoint!.motorImpulse;
}

export function b2RevoluteJoint_SetMaxMotorTorque(jointId: b2JointId, torque: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return;
    }
    joint.revoluteJoint!.maxMotorTorque = torque;
}

export function b2RevoluteJoint_GetMaxMotorTorque(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_revoluteJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.revoluteJoint!.maxMotorTorque;
}

export function b2GetRevoluteJointForce(world: b2World, base: b2JointSim): b2Vec2 {
    let force = b2MulSV(world.inv_h, base.revoluteJoint!.linearImpulse);
    return force;
}

export function b2GetRevoluteJointTorque(world: b2World, base: b2JointSim): f32 {
    let revolute = base.revoluteJoint!;
    let torque = world.inv_h * (revolute.motorImpulse + revolute.lowerImpulse - revolute.upperImpulse);
    return torque;
}

export function b2PrepareRevoluteJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_revoluteJoint);

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

    let joint = base.revoluteJoint!;

    joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
    joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

    // Compute joint anchor frames with world space rotation, relative to center of mass.
    // Avoid round-off here as much as possible.
    // b2Vec2 pf = (xf.p - c) + rot(xf.q, f.p)
    // pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
    // pf = rot(xf.q, f.p - lc)
    joint.frameA.q = b2MulRot(bodySimA.transform.q, base.localFrameA.q);
    joint.frameA.p = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
    joint.frameB.q = b2MulRot(bodySimB.transform.q, base.localFrameB.q);
    joint.frameB.p = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));

    // Compute the initial center delta. Incremental position updates are relative to this.
    joint.deltaCenter = b2Sub(bodySimB.center, bodySimA.center);

    let k = iA + iB;
    joint.axialMass = k > 0.0 ? <f32>1.0 / k : <f32>0.0;

    joint.springSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

    if (context.enableWarmStarting == false) {
        joint.linearImpulse = b2Vec2_zero();
        joint.springImpulse = 0.0;
        joint.motorImpulse = 0.0;
        joint.lowerImpulse = 0.0;
        joint.upperImpulse = 0.0;
    }
}

export function b2WarmStartRevoluteJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_revoluteJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.revoluteJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    let axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = b2MulSub(stateA.linearVelocity, mA, joint.linearImpulse);
        stateA.angularVelocity -= iA * (b2Cross(rA, joint.linearImpulse) + axialImpulse);
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = b2MulAdd(stateB.linearVelocity, mB, joint.linearImpulse);
        stateB.angularVelocity += iB * (b2Cross(rB, joint.linearImpulse) + axialImpulse);
    }
}

export function b2SolveRevoluteJoint(base: b2JointSim, context: b2StepContext, useBias: bool): void {
    assert(base.type == b2_revoluteJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.revoluteJoint!;

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

    let fixedRotation = (iA + iB == 0.0);

    // Solve spring.
    if (joint.enableSpring && fixedRotation == false) {
        let jointAngle = b2Rot_GetAngle(relQ);
        let jointAngleDelta = b2UnwindAngle(jointAngle - joint.targetAngle);

        let C = jointAngleDelta;
        let bias = joint.springSoftness.biasRate * C;
        let massScale = joint.springSoftness.massScale;
        let impulseScale = joint.springSoftness.impulseScale;

        let Cdot = wB - wA;
        let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.springImpulse;
        joint.springImpulse += impulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    // Solve motor constraint.
    if (joint.enableMotor && fixedRotation == false) {
        let Cdot = wB - wA - joint.motorSpeed;
        let impulse = -joint.axialMass * Cdot;
        let oldImpulse = joint.motorImpulse;
        let maxImpulse = context.h * joint.maxMotorTorque;
        joint.motorImpulse = b2ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = joint.motorImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    if (joint.enableLimit && fixedRotation == false) {
        let jointAngle = b2Rot_GetAngle(relQ);

        // Lower limit
        {
            let C = jointAngle - joint.lowerAngle;
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

            let Cdot = wB - wA;
            let oldImpulse = joint.lowerImpulse;
            let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * oldImpulse;
            joint.lowerImpulse = b2MaxFloat(oldImpulse + impulse, 0.0);
            impulse = joint.lowerImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
            let C = joint.upperAngle - jointAngle;
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
            let Cdot = wA - wB;
            let oldImpulse = joint.upperImpulse;
            let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * oldImpulse;
            joint.upperImpulse = b2MaxFloat(oldImpulse + impulse, 0.0);
            impulse = joint.upperImpulse - oldImpulse;

            // sign flipped on applied impulse
            wA += iA * impulse;
            wB -= iB * impulse;
        }
    }

    // Solve point-to-point constraint
    {
        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

        // current anchors
        let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
        let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

        let Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));

        let bias = b2Vec2_zero();
        let massScale: f32 = 1.0;
        let impulseScale: f32 = 0.0;
        if (useBias) {
            let dcA = stateA.deltaPosition;
            let dcB = stateB.deltaPosition;

            let separation = b2Add(b2Add(b2Sub(dcB, dcA), b2Sub(rB, rA)), joint.deltaCenter);
            bias = b2MulSV(base.constraintSoftness.biasRate, separation);
            massScale = base.constraintSoftness.massScale;
            impulseScale = base.constraintSoftness.impulseScale;
        }

        let K = new b2Mat22();
        K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        K.cx.y = K.cy.x;
        K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        let b = b2Solve22(K, b2Add(Cdot, bias));

        let impulse = new b2Vec2();
        impulse.x = -massScale * b.x - impulseScale * joint.linearImpulse.x;
        impulse.y = -massScale * b.y - impulseScale * joint.linearImpulse.y;
        joint.linearImpulse.x += impulse.x;
        joint.linearImpulse.y += impulse.y;

        vA = b2MulSub(vA, mA, impulse);
        wA -= iA * b2Cross(rA, impulse);
        vB = b2MulAdd(vB, mB, impulse);
        wB += iB * b2Cross(rB, impulse);
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
