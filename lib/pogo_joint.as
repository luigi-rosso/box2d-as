// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/pogo_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import {
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2Dot,
	b2IsValidFloat,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2RotateVector,
	b2SpringDamper,
	b2Sub,
	b2SubPos,
	b2ToRelativeTransform,
	b2TransformPoint,
	b2Vec2,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_pogoJoint } from "./types";
import { b2BodyState, b2GetBodyTransform, b2StateAddress, b2_dynamicFlag, b2ArrayView } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2StepContext } from "./solver";
import { b2GetWorld, b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2GetJointLocalIndex,
	b2GetJointSimAddressCheckType,
	b2GetJointSimViewCheckType,
	b2JointSimAt,
	b2PogoJointAt,
	b2WriteStateVelocity,
} from "./joint";

const b2_pogoAxis: b2Vec2 = b2Vec2(0.0, 1.0);

export unsafe function b2PogoJoint_SetRestLength(jointId: b2JointId, length: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(length), "b2IsValidFloat( length )", "b2PogoJoint_SetRestLength")) return;

	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	joint.restLength = length;
}

export unsafe function b2PogoJoint_GetRestLength(jointId: b2JointId): f32 {
	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	return joint.restLength;
}

export unsafe function b2PogoJoint_GetSpringHertz(jointId: b2JointId): f32 {
	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	return joint.hertz;
}

export unsafe function b2PogoJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2PogoJoint_SetSpringHertz")) return;

	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	joint.hertz = hertz;
}

export unsafe function b2PogoJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2PogoJoint_SetSpringDampingRatio")) {
		return;
	}

	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	joint.dampingRatio = dampingRatio;
}

export unsafe function b2PogoJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	return joint.dampingRatio;
}

export unsafe function b2PogoJoint_GetCurrentLength(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_pogoJoint);
	ref jointSim = sims[b2GetJointLocalIndex(jointId)];

	// Relative to body A so the difference stays in float precision far from the origin
	let wxfA = b2GetBodyTransform(world, jointSim.bodyIdA);
	let transformA = b2ToRelativeTransform(wxfA, wxfA.p);
	let transformB = b2ToRelativeTransform(b2GetBodyTransform(world, jointSim.bodyIdB), wxfA.p);

	let axis = b2RotateVector(jointSim.localFrameB.q, b2_pogoAxis);
	axis = b2RotateVector(transformB.q, axis);
	let pA = b2TransformPoint(transformA, jointSim.localFrameA.p);
	let pB = b2TransformPoint(transformB, jointSim.localFrameB.p);
	let d = b2Sub(pB, pA);
	let length: f32 = b2Dot(d, axis);
	return length;
}

export unsafe function b2PogoJoint_GetImpulse(jointId: b2JointId): f32 {
	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	return joint.impulse;
}

export unsafe function b2PogoJoint_GetCurrentSpeed(jointId: b2JointId): f32 {
	let joints = b2PogoJointAt(b2GetJointSimAddressCheckType(jointId, b2_pogoJoint));
	ref joint = unchecked(joints[0]);
	return joint.velocity;
}

export function b2GetPogoJointForce(world: b2World, baseAddress: usize): b2Vec2 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let basePogoView = b2PogoJointAt(baseAddress);
	ref basePogo = unchecked(basePogoView[0]);
	let force = b2MulSV(world.inv_h * basePogo.impulse, basePogo.normal);
	return force;
}

@noinline
export function b2PreparePogoJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_pogoJoint);

	let world = context.world;

	// chase body id to the solver set where the body lives
	let idA = base.bodyIdA;
	let idB = base.bodyIdB;

	let setIndexA = world.bodies[idA].setIndex;
	let setIndexB = world.bodies[idB].setIndex;

	if (B2_ASSERT_ENABLED) B2_ASSERT(setIndexA == b2_awakeSet || setIndexB == b2_awakeSet);

	let setA = world.solverSets[setIndexA];
	let setB = world.solverSets[setIndexB];

	let localIndexA = world.bodies[idA].localIndex;
	let localIndexB = world.bodies[idB].localIndex;

	let simsA = b2ArrayView(setA.bodySims);
	let simsB = b2ArrayView(setB.bodySims);
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

	let jointView = b2PogoJointAt(baseAddress);
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

	// compute effective mass
	let crA: f32 = b2Cross(rA, joint.normal);
	let crB: f32 = b2Cross(rB, joint.normal);
	let k: f32 = mA + mB + iA * crA * crA + iB * crB * crB;
	joint.linearMass = k > 0.0 ? 1.0 / k : 0.0;

	if (context.enableWarmStarting == false) {
		joint.impulse = 0.0;
	}
}

@noinline
export unsafe function b2WarmStartPogoJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_pogoJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	let jointView = b2PogoJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	if (joint.hertz == 0.0) {
		joint.impulse = 0.0;
		return;
	}

	// dummy state for static bodies
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);
	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let linearImpulse = b2MulSV(joint.impulse, joint.normal);

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressA,
			b2MulSub(stateA.linearVelocity, mA, linearImpulse),
			stateA.angularVelocity - iA * b2Cross(rA, linearImpulse),
		);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressB,
			b2MulAdd(stateB.linearVelocity, mB, linearImpulse),
			stateB.angularVelocity + iB * b2Cross(rB, linearImpulse),
		);
	}
}

@noinline
export unsafe function b2SolvePogoJoint(baseAddress: usize, context: b2StepContext, useBias: bool): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_pogoJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2PogoJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	if (joint.hertz == 0.0) {
		joint.impulse = 0.0;
		return;
	}

	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let wA: f32 = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB: f32 = stateB.angularVelocity;

	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let bias: f32 = 0.0;
	if (useBias) {
		// This is a custom pogo position correction designed to avoid feeding velocity into
		// the mover body. Otherwise the mover can make huge jumps going up stairs. This still
		// works with continuous collision.

		let dcA = stateA.deltaPosition;
		let dcB = stateB.deltaPosition;
		let d = b2Add(b2Add(b2Sub(dcB, dcA), b2Sub(rB, rA)), joint.deltaCenter);

		// The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
		let pogoAxis = b2RotateVector(joint.frameB.q, b2_pogoAxis);
		let c: f32 = b2Dot(pogoAxis, d) - joint.restLength;

		// This could be divided by dot(joint->normal, pogoAxis) to account for the constraint
		// direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
		joint.velocity = b2SpringDamper(joint.hertz, joint.dampingRatio, c, joint.velocity, context.h);
		bias = -joint.velocity;
	}

	let vr = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
	let cdot: f32 = b2Dot(joint.normal, vr);

	let maxTensionImpulse: f32 = context.h * joint.maxTensionForce;
	let maxCompressionImpulse: f32 = context.h * joint.maxCompressionForce;
	let oldImpulse: f32 = joint.impulse;
	let impulse: f32 = -joint.linearMass * (cdot + bias);
	joint.impulse = b2ClampFloat(joint.impulse + impulse, -maxTensionImpulse, maxCompressionImpulse);
	impulse = joint.impulse - oldImpulse;

	let P = b2MulSV(impulse, joint.normal);
	vA = b2MulSub(vA, mA, P);
	wA -= iA * b2Cross(rA, P);
	vB = b2MulAdd(vB, mB, P);
	wB += iB * b2Cross(rB, P);

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, vA, wA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, vB, wB);
	}
}
