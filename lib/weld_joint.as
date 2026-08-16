// Ported from box2d src/weld_joint.c. Debug draw omitted; the C
// B2_WELD_BLOCK_SOLVE path is compiled out upstream and not ported.

import { B2_NULL_INDEX } from "./core";
import {
    b2Vec2, b2Mat22, b2Vec2_zero, b2Add, b2Sub, b2Cross, b2CrossSV, b2MulSV, b2MulAdd, b2MulSub,
    b2RotateVector, b2MulRot, b2InvMulRot, b2Rot_GetAngle, b2Solve22, b2IsValidVec2, b2IsValidFloat,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_weldJoint } from "./types";
import { b2World, b2_awakeSet } from "./physics_world";
import { b2BodyState, b2_dynamicFlag } from "./body";
import { b2JointSim, b2GetJointSimCheckType } from "./joint";
import { b2StepContext, b2MakeSoft } from "./solver";

export function b2WeldJoint_SetLinearHertz(jointId: b2JointId, hertz: f32): void {
    assert(b2IsValidFloat(hertz) && hertz >= 0.0);
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return;
    }
    joint.weldJoint!.linearHertz = hertz;
}

export function b2WeldJoint_GetLinearHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.weldJoint!.linearHertz;
}

export function b2WeldJoint_SetLinearDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
    assert(b2IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return;
    }
    joint.weldJoint!.linearDampingRatio = dampingRatio;
}

export function b2WeldJoint_GetLinearDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.weldJoint!.linearDampingRatio;
}

export function b2WeldJoint_SetAngularHertz(jointId: b2JointId, hertz: f32): void {
    assert(b2IsValidFloat(hertz) && hertz >= 0.0);
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return;
    }
    joint.weldJoint!.angularHertz = hertz;
}

export function b2WeldJoint_GetAngularHertz(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.weldJoint!.angularHertz;
}

export function b2WeldJoint_SetAngularDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
    assert(b2IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return;
    }
    joint.weldJoint!.angularDampingRatio = dampingRatio;
}

export function b2WeldJoint_GetAngularDampingRatio(jointId: b2JointId): f32 {
    let joint = b2GetJointSimCheckType(jointId, b2_weldJoint);
    if (joint == null) {
        return 0.0;
    }
    return joint.weldJoint!.angularDampingRatio;
}

export function b2GetWeldJointForce(world: b2World, base: b2JointSim): b2Vec2 {
    let force = b2MulSV(world.inv_h, base.weldJoint!.linearImpulse);
    return force;
}

export function b2GetWeldJointTorque(world: b2World, base: b2JointSim): f32 {
    return world.inv_h * base.weldJoint!.angularImpulse;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-E -r1_skew E r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

export function b2PrepareWeldJoint(base: b2JointSim, context: b2StepContext): void {
    assert(base.type == b2_weldJoint);

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

    let joint = base.weldJoint!;
    joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
    joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

    // Compute joint anchor frames with world space rotation, relative to center of mass
    joint.frameA.q = b2MulRot(bodySimA.transform.q, base.localFrameA.q);
    joint.frameA.p = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
    joint.frameB.q = b2MulRot(bodySimB.transform.q, base.localFrameB.q);
    joint.frameB.p = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));

    // Compute the initial center delta. Incremental position updates are relative to this.
    joint.deltaCenter = b2Sub(bodySimB.center, bodySimA.center);

    let ka = iA + iB;
    joint.axialMass = ka > 0.0 ? <f32>1.0 / ka : <f32>0.0;

    if (joint.linearHertz == 0.0) {
        joint.linearSpring = base.constraintSoftness.clone();
    } else {
        joint.linearSpring = b2MakeSoft(joint.linearHertz, joint.linearDampingRatio, context.h);
    }

    if (joint.angularHertz == 0.0) {
        joint.angularSpring = base.constraintSoftness.clone();
    } else {
        joint.angularSpring = b2MakeSoft(joint.angularHertz, joint.angularDampingRatio, context.h);
    }

    if (context.enableWarmStarting == false) {
        joint.linearImpulse = b2Vec2_zero();
        joint.angularImpulse = 0.0;
    }
}

export function b2WarmStartWeldJoint(base: b2JointSim, context: b2StepContext): void {
    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.weldJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
    let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

    if ((stateA.flags & b2_dynamicFlag) != 0) {
        stateA.linearVelocity = b2MulSub(stateA.linearVelocity, mA, joint.linearImpulse);
        stateA.angularVelocity -= iA * (b2Cross(rA, joint.linearImpulse) + joint.angularImpulse);
    }

    if ((stateB.flags & b2_dynamicFlag) != 0) {
        stateB.linearVelocity = b2MulAdd(stateB.linearVelocity, mB, joint.linearImpulse);
        stateB.angularVelocity += iB * (b2Cross(rB, joint.linearImpulse) + joint.angularImpulse);
    }
}

export function b2SolveWeldJoint(base: b2JointSim, context: b2StepContext, useBias: bool): void {
    assert(base.type == b2_weldJoint);

    let mA = base.invMassA;
    let mB = base.invMassB;
    let iA = base.invIA;
    let iB = base.invIB;

    let joint = base.weldJoint!;

    // dummy state for static bodies
    let stateA = joint.indexA == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexA]!;
    let stateB = joint.indexB == B2_NULL_INDEX ? new b2BodyState() : context.states[joint.indexB]!;

    let vA = stateA.linearVelocity;
    let wA = stateA.angularVelocity;
    let vB = stateB.linearVelocity;
    let wB = stateB.angularVelocity;

    // angular constraint
    {
        let qA = b2MulRot(stateA.deltaRotation, joint.frameA.q);
        let qB = b2MulRot(stateB.deltaRotation, joint.frameB.q);
        let relQ = b2InvMulRot(qA, qB);
        let jointAngle = b2Rot_GetAngle(relQ);

        let bias: f32 = 0.0;
        let massScale: f32 = 1.0;
        let impulseScale: f32 = 0.0;
        if (useBias || joint.angularHertz > 0.0) {
            let C = jointAngle;
            bias = joint.angularSpring.biasRate * C;
            massScale = joint.angularSpring.massScale;
            impulseScale = joint.angularSpring.impulseScale;
        }

        let Cdot = wB - wA;
        let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.angularImpulse;
        joint.angularImpulse += impulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    // linear constraint
    {
        let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
        let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

        let bias = b2Vec2_zero();
        let massScale: f32 = 1.0;
        let impulseScale: f32 = 0.0;
        if (useBias || joint.linearHertz > 0.0) {
            let dcA = stateA.deltaPosition;
            let dcB = stateB.deltaPosition;
            let C = b2Add(b2Add(b2Sub(dcB, dcA), b2Sub(rB, rA)), joint.deltaCenter);

            bias = b2MulSV(joint.linearSpring.biasRate, C);
            massScale = joint.linearSpring.massScale;
            impulseScale = joint.linearSpring.impulseScale;
        }

        let Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));

        let K = new b2Mat22();
        K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        K.cx.y = K.cy.x;
        K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        let b = b2Solve22(K, b2Add(Cdot, bias));

        let impulse = new b2Vec2(
            -massScale * b.x - impulseScale * joint.linearImpulse.x,
            -massScale * b.y - impulseScale * joint.linearImpulse.y);

        joint.linearImpulse = b2Add(joint.linearImpulse, impulse);

        vA = b2MulSub(vA, mA, impulse);
        wA -= iA * b2Cross(rA, impulse);
        vB = b2MulAdd(vB, mB, impulse);
        wB += iB * b2Cross(rB, impulse);
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
