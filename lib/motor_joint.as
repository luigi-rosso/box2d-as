// Ported from box2d src/motor_joint.c. Debug draw omitted.

import { B2_NULL_INDEX } from "./core";
import {
    b2Vec2, b2Mat22, b2Vec2_zero, b2Add, b2Sub, b2MulSV, b2MulAdd, b2MulSub, b2Cross, b2CrossSV,
    b2RotateVector, b2MulRot, b2InvMulRot, b2Rot_GetAngle, b2GetInverse22, b2MulMV, b2LengthSquared,
    b2Normalize, b2ClampFloat, b2MaxFloat,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_motorJoint } from "./types";
import { b2JointSim, b2GetJointSimCheckType } from "./joint";
import { b2StepContext, b2MakeSoft } from "./solver";
import { b2BodyState, b2_dynamicFlag } from "./body";
import { b2World, b2_awakeSet } from "./physics_world";

export function b2MotorJoint_SetLinearVelocity(jointId: b2JointId, velocity: b2Vec2): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.linearVelocity = velocity.clone();
}

export function b2MotorJoint_GetLinearVelocity(jointId: b2JointId): b2Vec2 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return new b2Vec2();
    }
    return joint.motorJoint!.linearVelocity.clone();
}

export function b2MotorJoint_SetAngularVelocity(jointId: b2JointId, velocity: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.angularVelocity = velocity;
}

export function b2MotorJoint_GetAngularVelocity(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.angularVelocity;
}

export function b2MotorJoint_SetMaxVelocityTorque(jointId: b2JointId, maxTorque: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.maxVelocityTorque = maxTorque;
}

export function b2MotorJoint_GetMaxVelocityTorque(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.maxVelocityTorque;
}

export function b2MotorJoint_SetMaxVelocityForce(jointId: b2JointId, maxForce: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.maxVelocityForce = maxForce;
}

export function b2MotorJoint_GetMaxVelocityForce(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.maxVelocityForce;
}

export function b2MotorJoint_SetLinearHertz(jointId: b2JointId, hertz: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.linearHertz = hertz;
}

export function b2MotorJoint_GetLinearHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.linearHertz;
}

export function b2MotorJoint_SetLinearDampingRatio(jointId: b2JointId, damping: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.linearDampingRatio = damping;
}

export function b2MotorJoint_GetLinearDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.linearDampingRatio;
}

export function b2MotorJoint_SetAngularHertz(jointId: b2JointId, hertz: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.angularHertz = hertz;
}

export function b2MotorJoint_GetAngularHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.angularHertz;
}

export function b2MotorJoint_SetAngularDampingRatio(jointId: b2JointId, damping: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.angularDampingRatio = damping;
}

export function b2MotorJoint_GetAngularDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.angularDampingRatio;
}

export function b2MotorJoint_SetMaxSpringForce(jointId: b2JointId, maxForce: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.maxSpringForce = b2MaxFloat(0.0, maxForce);
}

export function b2MotorJoint_GetMaxSpringForce(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.maxSpringForce;
}

export function b2MotorJoint_SetMaxSpringTorque(jointId: b2JointId, maxTorque: f32): void {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return;
    }
    joint.motorJoint!.maxSpringTorque = b2MaxFloat(0.0, maxTorque);
}

export function b2MotorJoint_GetMaxSpringTorque(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_motorJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.motorJoint!.maxSpringTorque;
}

export function b2GetMotorJointForce(world: b2World, base: b2JointSim): b2Vec2 {
    let force = b2MulSV(world.inv_h, b2Add(base.motorJoint!.linearVelocityImpulse, base.motorJoint!.linearSpringImpulse));
    return force;
}

export function b2GetMotorJointTorque(world: b2World, base: b2JointSim): f32 {
    return world.inv_h * (base.motorJoint!.angularVelocityImpulse + base.motorJoint!.angularSpringImpulse);
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

export function b2PrepareMotorJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_motorJoint);

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

    let joint = base.motorJoint!;
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

    joint.linearSpring = b2MakeSoft(joint.linearHertz, joint.linearDampingRatio, context.h);
    joint.angularSpring = b2MakeSoft(joint.angularHertz, joint.angularDampingRatio, context.h);

    let kl = new b2Mat22();
    kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
    kl.cy.x = kl.cx.y;
    kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
    joint.linearMass = b2GetInverse22(kl);

    let ka = iA + iB;
    joint.angularMass = ka > 0.0 ? <f32>1.0 / ka : <f32>0.0;

    if (context.enableWarmStarting == false) {
        joint.linearVelocityImpulse = b2Vec2_zero();
        joint.angularVelocityImpulse = 0.0;
        joint.linearSpringImpulse = b2Vec2_zero();
        joint.angularSpringImpulse = 0.0;
    }
}

export function b2WarmStartMotorJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_motorJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.motorJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    let linearImpulse = b2Add(joint.linearVelocityImpulse, joint.linearSpringImpulse);
    let angularImpulse = joint.angularVelocityImpulse + joint.angularSpringImpulse;

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = b2MulSub(stateA.linearVelocity, mA, linearImpulse);
        stateA.angularVelocity -= iA * (b2Cross(rA, linearImpulse) + angularImpulse);
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = b2MulAdd(stateB.linearVelocity, mB, linearImpulse);
        stateB.angularVelocity += iB * (b2Cross(rB, linearImpulse) + angularImpulse);
    }
}

export function b2SolveMotorJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_motorJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    // dummy state for static bodies
    let joint = base.motorJoint!;
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let vA = stateA.linearVelocity;
    let wA = stateA.angularVelocity;
    let vB = stateB.linearVelocity;
    let wB = stateB.angularVelocity;

    // angular spring
    if (joint.maxSpringTorque > 0.0 && joint.angularHertz > 0.0) {
        let qA = b2MulRot(stateA.deltaRotation, joint.frameA.q);
        let qB = b2MulRot(stateB.deltaRotation, joint.frameB.q);
        let relQ = b2InvMulRot(qA, qB);

        let c = b2Rot_GetAngle(relQ);
        let bias = joint.angularSpring.biasRate * c;
        let massScale = joint.angularSpring.massScale;
        let impulseScale = joint.angularSpring.impulseScale;

        let cdot = wB - wA;

        let maxImpulse = context.h * joint.maxSpringTorque;
        let oldImpulse = joint.angularSpringImpulse;
        let impulse = -massScale * joint.angularMass * (cdot + bias) - impulseScale * oldImpulse;
        joint.angularSpringImpulse = b2ClampFloat(oldImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = joint.angularSpringImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    // angular velocity
    if (joint.maxVelocityTorque > 0.0) {
        let cdot = wB - wA - joint.angularVelocity;
        let impulse = -joint.angularMass * cdot;

        let maxImpulse = context.h * joint.maxVelocityTorque;
        let oldImpulse = joint.angularVelocityImpulse;
        joint.angularVelocityImpulse = b2ClampFloat(oldImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = joint.angularVelocityImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    // linear spring
    if (joint.maxSpringForce > 0.0 && joint.linearHertz > 0.0) {
        let dcA = stateA.deltaPosition;
        let dcB = stateB.deltaPosition;
        let c = b2Add(b2Add(b2Sub(dcB, dcA), b2Sub(rB, rA)), joint.deltaCenter);

        let bias = b2MulSV(joint.linearSpring.biasRate, c);
        let massScale = joint.linearSpring.massScale;
        let impulseScale = joint.linearSpring.impulseScale;

        let cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
        cdot = b2Add(cdot, bias);

        // Updating the effective mass here may be overkill
        let kl = new b2Mat22();
        kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        kl.cy.x = kl.cx.y;
        kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        joint.linearMass = b2GetInverse22(kl);

        let b = b2MulMV(joint.linearMass, cdot);

        let oldImpulse = joint.linearSpringImpulse;
        let impulse = new b2Vec2(
            -massScale * b.x - impulseScale * oldImpulse.x,
            -massScale * b.y - impulseScale * oldImpulse.y);

        let maxImpulse = context.h * joint.maxSpringForce;
        joint.linearSpringImpulse = b2Add(joint.linearSpringImpulse, impulse);

        if (b2LengthSquared(joint.linearSpringImpulse) > maxImpulse * maxImpulse) {
            joint.linearSpringImpulse = b2Normalize(joint.linearSpringImpulse);
            joint.linearSpringImpulse.x *= maxImpulse;
            joint.linearSpringImpulse.y *= maxImpulse;
        }

        impulse = b2Sub(joint.linearSpringImpulse, oldImpulse);

        vA = b2MulSub(vA, mA, impulse);
        wA -= iA * b2Cross(rA, impulse);
        vB = b2MulAdd(vB, mB, impulse);
        wB += iB * b2Cross(rB, impulse);
    }

    // linear velocity
    if (joint.maxVelocityForce > 0.0) {
        let cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
        cdot = b2Sub(cdot, joint.linearVelocity);
        let b = b2MulMV(joint.linearMass, cdot);
        let impulse = new b2Vec2(-b.x, -b.y);

        let oldImpulse = joint.linearVelocityImpulse;
        let maxImpulse = context.h * joint.maxVelocityForce;
        joint.linearVelocityImpulse = b2Add(joint.linearVelocityImpulse, impulse);

        if (b2LengthSquared(joint.linearVelocityImpulse) > maxImpulse * maxImpulse) {
            joint.linearVelocityImpulse = b2Normalize(joint.linearVelocityImpulse);
            joint.linearVelocityImpulse.x *= maxImpulse;
            joint.linearVelocityImpulse.y *= maxImpulse;
        }

        impulse = b2Sub(joint.linearVelocityImpulse, oldImpulse);

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
