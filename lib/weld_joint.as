// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/weld_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import {
	b2Add,
	b2Cross,
	b2CrossSV,
	b2InvMulRot,
	b2IsValidFloat,
	b2IsValidVec2,
	b2Mat22,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2RotateVector,
	b2Rot_GetAngle,
	b2Solve22,
	b2Sub,
	b2SubPos,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_weldJoint } from "./types";
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
	b2WeldJointAt,
	b2WriteStateVelocity,
} from "./joint";

// B2_WELD_BLOCK_SOLVE is 0 in C; the block solve path is not ported

export unsafe function b2WeldJoint_SetLinearHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2WeldJoint_SetLinearHertz")) return;

	B2_ASSERT(b2IsValidFloat(hertz) && hertz >= 0.0);
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	joint.linearHertz = hertz;
}

export unsafe function b2WeldJoint_GetLinearHertz(jointId: b2JointId): f32 {
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	return joint.linearHertz;
}

export unsafe function b2WeldJoint_SetLinearDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2WeldJoint_SetLinearDampingRatio")) {
		return;
	}

	B2_ASSERT(b2IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	joint.linearDampingRatio = dampingRatio;
}

export unsafe function b2WeldJoint_GetLinearDampingRatio(jointId: b2JointId): f32 {
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	return joint.linearDampingRatio;
}

export unsafe function b2WeldJoint_SetAngularHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2WeldJoint_SetAngularHertz")) return;

	B2_ASSERT(b2IsValidFloat(hertz) && hertz >= 0.0);
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	joint.angularHertz = hertz;
}

export unsafe function b2WeldJoint_GetAngularHertz(jointId: b2JointId): f32 {
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	return joint.angularHertz;
}

export unsafe function b2WeldJoint_SetAngularDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2WeldJoint_SetAngularDampingRatio")) {
		return;
	}

	B2_ASSERT(b2IsValidFloat(dampingRatio) && dampingRatio >= 0.0);
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	joint.angularDampingRatio = dampingRatio;
}

export unsafe function b2WeldJoint_GetAngularDampingRatio(jointId: b2JointId): f32 {
	let joints = b2WeldJointAt(b2GetJointSimAddressCheckType(jointId, b2_weldJoint));
	ref joint = unchecked(joints[0]);
	return joint.angularDampingRatio;
}

export function b2GetWeldJointForce(world: b2World, baseAddress: usize): b2Vec2 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseWeldView = b2WeldJointAt(baseAddress);
	ref baseWeld = unchecked(baseWeldView[0]);
	let force = b2MulSV(world.inv_h, baseWeld.linearImpulse);
	return force;
}

export function b2GetWeldJointTorque(world: b2World, baseAddress: usize): f32 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseWeldView = b2WeldJointAt(baseAddress);
	ref baseWeld = unchecked(baseWeldView[0]);
	return world.inv_h * baseWeld.angularImpulse;
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

// 3x3 Block
// K = [J1] * invM * [J1T J2T]
//     [J2]
//   = [J1] * [invM * J1T invM * J2T]
//     [J2]
//   = [J1 * invM * J1T J1 * invM * J2T]
//     [J2 * invM * J1T J2 * invM * J2T]

@noinline
export function b2PrepareWeldJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_weldJoint);

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

	let jointView = b2WeldJointAt(baseAddress);
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

	let ka: f32 = iA + iB;
	joint.axialMass = ka > 0.0 ? 1.0 / ka : 0.0;

	if (joint.linearHertz == 0.0) {
		joint.linearSpring = base.constraintSoftness;
	} else {
		joint.linearSpring = b2MakeSoft(joint.linearHertz, joint.linearDampingRatio, context.h);
	}

	if (joint.angularHertz == 0.0) {
		joint.angularSpring = base.constraintSoftness;
	} else {
		joint.angularSpring = b2MakeSoft(joint.angularHertz, joint.angularDampingRatio, context.h);
	}

	if (context.enableWarmStarting == false) {
		joint.linearImpulse = b2Vec2_zero;
		joint.angularImpulse = 0.0;
	}
}

@noinline
export unsafe function b2WarmStartWeldJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2WeldJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressA,
			b2MulSub(stateA.linearVelocity, mA, joint.linearImpulse),
			stateA.angularVelocity - iA * (b2Cross(rA, joint.linearImpulse) + joint.angularImpulse),
		);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressB,
			b2MulAdd(stateB.linearVelocity, mB, joint.linearImpulse),
			stateB.angularVelocity + iB * (b2Cross(rB, joint.linearImpulse) + joint.angularImpulse),
		);
	}
}

@noinline
export unsafe function b2SolveWeldJoint(baseAddress: usize, context: b2StepContext, useBias: bool): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_weldJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2WeldJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let wA: f32 = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB: f32 = stateB.angularVelocity;

	// Block solve doesn't work correctly with mixed stiffness values

	// angular constraint
	{
		let qA = b2MulRot(stateA.deltaRotation, joint.frameA.q);
		let qB = b2MulRot(stateB.deltaRotation, joint.frameB.q);
		let relQ = b2InvMulRot(qA, qB);
		let jointAngle: f32 = b2Rot_GetAngle(relQ);

		let bias: f32 = 0.0;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias || joint.angularHertz > 0.0) {
			let C: f32 = jointAngle;
			bias = joint.angularSpring.biasRate * C;
			massScale = joint.angularSpring.massScale;
			impulseScale = joint.angularSpring.impulseScale;
		}

		let Cdot: f32 = wB - wA;
		let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.angularImpulse;
		joint.angularImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// linear constraint
	{
		let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
		let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

		let bias = b2Vec2_zero;
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

		let K = b2Mat22();
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		let b = b2Solve22(K, b2Add(Cdot, bias));

		let impulse = b2Vec2(
			-massScale * b.x - impulseScale * joint.linearImpulse.x,
			-massScale * b.y - impulseScale * joint.linearImpulse.y,
		);

		joint.linearImpulse = b2Add(joint.linearImpulse, impulse);

		vA = b2MulSub(vA, mA, impulse);
		wA -= iA * b2Cross(rA, impulse);
		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * b2Cross(rB, impulse);
	}

	if (B2_ASSERT_ENABLED) {
		B2_ASSERT(b2IsValidVec2(vA));
		B2_ASSERT(b2IsValidFloat(wA));
		B2_ASSERT(b2IsValidVec2(vB));
		B2_ASSERT(b2IsValidFloat(wB));
	}

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, vA, wA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, vB, wB);
	}
}
