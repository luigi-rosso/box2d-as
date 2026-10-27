// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/motor_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import {
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2GetInverse22,
	b2InvMulRot,
	b2IsValidFloat,
	b2IsValidVec2,
	b2LengthSquared,
	b2Mat22,
	b2MaxFloat,
	b2MulAdd,
	b2MulMV,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2Normalize,
	b2RotateVector,
	b2Rot_GetAngle,
	b2Sub,
	b2SubPos,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_motorJoint } from "./types";
import { b2BodyState, b2StateAddress, b2_dynamicFlag, b2BodySimView } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2MakeSoft, b2StepContext } from "./solver";
import { b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2GetJointLocalIndex,
	b2GetJointSimAddressCheckType,
	b2GetJointSimViewCheckType,
	b2JointSim,
	b2JointSimAt,
	b2MotorJointAt,
	b2WriteStateVelocity,
} from "./joint";

export unsafe function b2MotorJoint_SetLinearVelocity(jointId: b2JointId, velocity: b2Vec2): void {
	if (!b2CheckJointInput(b2IsValidVec2(velocity), "b2IsValidVec2( velocity )", "b2MotorJoint_SetLinearVelocity")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.linearVelocity = velocity;
}

export unsafe function b2MotorJoint_GetLinearVelocity(jointId: b2JointId): b2Vec2 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.linearVelocity;
}

export unsafe function b2MotorJoint_SetAngularVelocity(jointId: b2JointId, velocity: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(velocity), "b2IsValidFloat( velocity )", "b2MotorJoint_SetAngularVelocity")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.angularVelocity = velocity;
}

export unsafe function b2MotorJoint_GetAngularVelocity(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.angularVelocity;
}

export unsafe function b2MotorJoint_SetMaxVelocityTorque(jointId: b2JointId, maxTorque: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(maxTorque), "b2IsValidFloat( maxTorque )", "b2MotorJoint_SetMaxVelocityTorque")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.maxVelocityTorque = maxTorque;
}

export unsafe function b2MotorJoint_GetMaxVelocityTorque(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxVelocityTorque;
}

export unsafe function b2MotorJoint_SetMaxVelocityForce(jointId: b2JointId, maxForce: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(maxForce), "b2IsValidFloat( maxForce )", "b2MotorJoint_SetMaxVelocityForce")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.maxVelocityForce = maxForce;
}

export unsafe function b2MotorJoint_GetMaxVelocityForce(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxVelocityForce;
}

export unsafe function b2MotorJoint_SetLinearHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2MotorJoint_SetLinearHertz")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.linearHertz = hertz;
}

export unsafe function b2MotorJoint_GetLinearHertz(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.linearHertz;
}

export unsafe function b2MotorJoint_SetLinearDampingRatio(jointId: b2JointId, damping: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(damping), "b2IsValidFloat( damping )", "b2MotorJoint_SetLinearDampingRatio")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.linearDampingRatio = damping;
}

export unsafe function b2MotorJoint_GetLinearDampingRatio(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.linearDampingRatio;
}

export unsafe function b2MotorJoint_SetAngularHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2MotorJoint_SetAngularHertz")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.angularHertz = hertz;
}

export unsafe function b2MotorJoint_GetAngularHertz(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.angularHertz;
}

export unsafe function b2MotorJoint_SetAngularDampingRatio(jointId: b2JointId, damping: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(damping), "b2IsValidFloat( damping )", "b2MotorJoint_SetAngularDampingRatio")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.angularDampingRatio = damping;
}

export unsafe function b2MotorJoint_GetAngularDampingRatio(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.angularDampingRatio;
}

export unsafe function b2MotorJoint_SetMaxSpringForce(jointId: b2JointId, maxForce: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(maxForce), "b2IsValidFloat( maxForce )", "b2MotorJoint_SetMaxSpringForce")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.maxSpringForce = b2MaxFloat(0.0, maxForce);
}

export unsafe function b2MotorJoint_GetMaxSpringForce(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxSpringForce;
}

export unsafe function b2MotorJoint_SetMaxSpringTorque(jointId: b2JointId, maxTorque: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(maxTorque), "b2IsValidFloat( maxTorque )", "b2MotorJoint_SetMaxSpringTorque")) return;

	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	joint.maxSpringTorque = b2MaxFloat(0.0, maxTorque);
}

export unsafe function b2MotorJoint_GetMaxSpringTorque(jointId: b2JointId): f32 {
	let joints = b2MotorJointAt(b2GetJointSimAddressCheckType(jointId, b2_motorJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxSpringTorque;
}

export function b2GetMotorJointForce(world: b2World, baseAddress: usize): b2Vec2 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseMotorView = b2MotorJointAt(baseAddress);
	ref baseMotor = unchecked(baseMotorView[0]);
	let force = b2MulSV(world.inv_h, b2Add(baseMotor.linearVelocityImpulse, baseMotor.linearSpringImpulse));
	return force;
}

export function b2GetMotorJointTorque(world: b2World, baseAddress: usize): f32 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseMotorView = b2MotorJointAt(baseAddress);
	ref baseMotor = unchecked(baseMotorView[0]);
	return world.inv_h * (baseMotor.angularVelocityImpulse + baseMotor.angularSpringImpulse);
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

@noinline
export function b2PrepareMotorJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_motorJoint);

	// chase body id to the solver set where the body lives
	let idA = base.bodyIdA;
	let idB = base.bodyIdB;

	let world = context.world;

	let setIndexA = world.bodies[idA].setIndex;
	let setIndexB = world.bodies[idB].setIndex;

	if (B2_ASSERT_ENABLED) B2_ASSERT(setIndexA == b2_awakeSet || setIndexB == b2_awakeSet);

	let setA = world.solverSets[setIndexA];
	let setB = world.solverSets[setIndexB];

	let localIndexA = world.bodies[idA].localIndex;
	let localIndexB = world.bodies[idB].localIndex;

	let simsA = b2BodySimView(setA.bodySims);
	let simsB = b2BodySimView(setB.bodySims);
	ref bodySimA = simsA[localIndexA];
	ref bodySimB = simsB[localIndexB];

	let mA: f32 = bodySimA.invMass;
	let iA: f32 = bodySimA.invInertia;
	let mB: f32 = bodySimB.invMass;
	let iB: f32 = bodySimB.invInertia;

	base.invMassA = mA;
	base.invMassB = mB;
	base.invIA = iA;
	base.invIB = iB;

	let jointView = b2MotorJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	joint.indexA = setIndexA == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = setIndexB == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b2MulRot(bodySimA.transform.q, base.localFrameA.q);
	joint.frameA.p = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.frameB.q = b2MulRot(bodySimB.transform.q, base.localFrameB.q);
	joint.frameB.p = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = b2SubPos(bodySimB.center, bodySimA.center);

	let rA = joint.frameA.p;
	let rB = joint.frameB.p;

	joint.linearSpring = b2MakeSoft(joint.linearHertz, joint.linearDampingRatio, context.h);
	joint.angularSpring = b2MakeSoft(joint.angularHertz, joint.angularDampingRatio, context.h);

	let kl = b2Mat22();
	kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	kl.cy.x = kl.cx.y;
	kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	joint.linearMass = b2GetInverse22(kl);

	let ka: f32 = iA + iB;
	joint.angularMass = ka > 0.0 ? 1.0 / ka : 0.0;

	if (context.enableWarmStarting == false) {
		joint.linearVelocityImpulse = b2Vec2_zero;
		joint.angularVelocityImpulse = 0.0;
		joint.linearSpringImpulse = b2Vec2_zero;
		joint.angularSpringImpulse = 0.0;
	}
}

@noinline
export unsafe function b2WarmStartMotorJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_motorJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	let jointView = b2MotorJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	// dummy state for static bodies
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let linearImpulse = b2Add(joint.linearVelocityImpulse, joint.linearSpringImpulse);
	let angularImpulse: f32 = joint.angularVelocityImpulse + joint.angularSpringImpulse;

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressA,
			b2MulSub(stateA.linearVelocity, mA, linearImpulse),
			stateA.angularVelocity - iA * (b2Cross(rA, linearImpulse) + angularImpulse),
		);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressB,
			b2MulAdd(stateB.linearVelocity, mB, linearImpulse),
			stateB.angularVelocity + iB * (b2Cross(rB, linearImpulse) + angularImpulse),
		);
	}
}

@noinline
export unsafe function b2SolveMotorJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_motorJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2MotorJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let wA: f32 = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB: f32 = stateB.angularVelocity;

	// angular spring
	if (joint.maxSpringTorque > 0.0 && joint.angularHertz > 0.0) {
		let qA = b2MulRot(stateA.deltaRotation, joint.frameA.q);
		let qB = b2MulRot(stateB.deltaRotation, joint.frameB.q);
		let relQ = b2InvMulRot(qA, qB);

		let c: f32 = b2Rot_GetAngle(relQ);
		let bias: f32 = joint.angularSpring.biasRate * c;
		let massScale: f32 = joint.angularSpring.massScale;
		let impulseScale: f32 = joint.angularSpring.impulseScale;

		let cdot: f32 = wB - wA;

		let maxImpulse: f32 = context.h * joint.maxSpringTorque;
		let oldImpulse: f32 = joint.angularSpringImpulse;
		let impulse: f32 = -massScale * joint.angularMass * (cdot + bias) - impulseScale * oldImpulse;
		joint.angularSpringImpulse = b2ClampFloat(oldImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint.angularSpringImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// angular velocity
	if (joint.maxVelocityTorque > 0.0) {
		let cdot: f32 = wB - wA - joint.angularVelocity;
		let impulse: f32 = -joint.angularMass * cdot;

		let maxImpulse: f32 = context.h * joint.maxVelocityTorque;
		let oldImpulse: f32 = joint.angularVelocityImpulse;
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
		let massScale: f32 = joint.linearSpring.massScale;
		let impulseScale: f32 = joint.linearSpring.impulseScale;

		let cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
		cdot = b2Add(cdot, bias);

		// Updating the effective mass here may be overkill
		let kl = b2Mat22();
		kl.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		kl.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		kl.cy.x = kl.cx.y;
		kl.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		joint.linearMass = b2GetInverse22(kl);

		let b = b2MulMV(joint.linearMass, cdot);

		let oldImpulse = joint.linearSpringImpulse;
		let impulse = b2Vec2(-massScale * b.x - impulseScale * oldImpulse.x, -massScale * b.y - impulseScale * oldImpulse.y);

		let maxImpulse: f32 = context.h * joint.maxSpringForce;
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
		let impulse = b2Vec2(-b.x, -b.y);

		let oldImpulse = joint.linearVelocityImpulse;
		let maxImpulse: f32 = context.h * joint.maxVelocityForce;
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

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, vA, wA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, vB, wB);
	}
}
