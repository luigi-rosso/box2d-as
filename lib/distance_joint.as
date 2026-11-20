// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/distance_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import { B2_HUGE, B2_LINEAR_SLOP } from "./constants";
import {
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2Dot,
	b2IsValidFloat,
	b2Length,
	b2MaxFloat,
	b2MinFloat,
	b2MulAdd,
	b2MulSV,
	b2MulSub,
	b2Normalize,
	b2RotateVector,
	b2Sub,
	b2SubPos,
	b2ToRelativeTransform,
	b2TransformPoint,
	b2Vec2,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_distanceJoint } from "./types";
import { b2BodyState, b2GetBodyTransform, b2StateAddress, b2_dynamicFlag, b2ArrayView } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2MakeSoft, b2StepContext } from "./solver";
import { b2GetWorld, b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2DistanceJointAt,
	b2GetJointLocalIndex,
	b2GetJointSimAddressCheckType,
	b2GetJointSimViewCheckType,
	b2JointSimAddress,
	b2JointSimAt,
	b2WriteStateVelocity,
} from "./joint";

export function b2DistanceJoint_SetRestLength(jointId: b2JointId, length: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(length), "b2IsValidFloat( length )", "b2DistanceJoint_SetRestLength")) {
		return;
	}

	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	joint.length = b2ClampFloat(length, B2_LINEAR_SLOP(), B2_HUGE());
	joint.impulse = 0.0;
	joint.lowerImpulse = 0.0;
	joint.upperImpulse = 0.0;
}

export function b2DistanceJoint_GetRestLength(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return baseDistance.length;
}

export function b2DistanceJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	baseDistance.enableLimit = enableLimit;
}

export function b2DistanceJoint_IsLimitEnabled(jointId: b2JointId): bool {
	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableLimit;
}

export function b2DistanceJoint_SetLimits(jointId: b2JointId, minLength: f32, maxLength: f32): void {
	const fn = "b2DistanceJoint_SetLimits";
	if (!b2CheckJointInput(b2IsValidFloat(minLength), "b2IsValidFloat( minLength )", fn)) return;
	if (!b2CheckJointInput(b2IsValidFloat(maxLength), "b2IsValidFloat( maxLength )", fn)) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	minLength = b2ClampFloat(minLength, B2_LINEAR_SLOP(), B2_HUGE());
	maxLength = b2ClampFloat(maxLength, B2_LINEAR_SLOP(), B2_HUGE());
	joint.minLength = b2MinFloat(minLength, maxLength);
	joint.maxLength = b2MaxFloat(minLength, maxLength);
}

export function b2DistanceJoint_GetLowerLimit(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return baseDistance.minLength;
}

export function b2DistanceJoint_GetUpperLimit(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return baseDistance.maxLength;
}

export function b2DistanceJoint_GetCurrentLength(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	ref base = sims[b2GetJointLocalIndex(jointId)];

	let world = b2GetWorld(jointId.world0);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return 0.0;
	}

	// Relative to body A so the difference stays in float precision far from the origin
	let wxfA = b2GetBodyTransform(world, base.bodyIdA);
	let transformA = b2ToRelativeTransform(wxfA, wxfA.p);
	let transformB = b2ToRelativeTransform(b2GetBodyTransform(world, base.bodyIdB), wxfA.p);

	let pA = b2TransformPoint(transformA, base.localFrameA.p);
	let pB = b2TransformPoint(transformB, base.localFrameB.p);
	let d = b2Sub(pB, pA);
	let length: f32 = b2Length(d);
	return length;
}

export function b2DistanceJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	baseDistance.enableSpring = enableSpring;
}

export function b2DistanceJoint_IsSpringEnabled(jointId: b2JointId): bool {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return baseDistance.enableSpring;
}

export function b2DistanceJoint_SetSpringForceRange(jointId: b2JointId, lowerForce: f32, upperForce: f32): void {
	const fn = "b2DistanceJoint_SetSpringForceRange";
	if (!b2CheckJointInput(b2IsValidFloat(lowerForce), "b2IsValidFloat( lowerForce )", fn)) return;
	if (!b2CheckJointInput(b2IsValidFloat(upperForce), "b2IsValidFloat( upperForce )", fn)) return;

	B2_ASSERT(lowerForce <= upperForce);
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	baseDistance.lowerSpringForce = b2MinFloat(lowerForce, upperForce);
	baseDistance.upperSpringForce = b2MaxFloat(lowerForce, upperForce);
}

export function b2DistanceJoint_GetSpringForceRange(jointId: b2JointId, lowerForce: ref f32, upperForce: ref f32): void {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	lowerForce = baseDistance.lowerSpringForce;
	upperForce = baseDistance.upperSpringForce;
}

export function b2DistanceJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2DistanceJoint_SetSpringHertz")) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	baseDistance.hertz = hertz;
}

export function b2DistanceJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2DistanceJoint_SetSpringDampingRatio")) {
		return;
	}

	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	baseDistance.dampingRatio = dampingRatio;
}

export function b2DistanceJoint_GetSpringHertz(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return baseDistance.hertz;
}

export function b2DistanceJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return baseDistance.dampingRatio;
}

export function b2DistanceJoint_GetSpringForce(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	if (joint.enableSpring) {
		return joint.impulse * world.inv_h;
	}

	return 0.0;
}

export function b2DistanceJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	if (enableMotor != joint.enableMotor) {
		joint.enableMotor = enableMotor;
		joint.motorImpulse = 0.0;
	}
}

export function b2DistanceJoint_IsMotorEnabled(jointId: b2JointId): bool {
	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableMotor;
}

export function b2DistanceJoint_SetTargetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(motorSpeed), "b2IsValidFloat( motorSpeed )", "b2DistanceJoint_SetTargetMotorSpeed")) {
		return;
	}

	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	joint.motorSpeed = motorSpeed;
}

export function b2DistanceJoint_GetTargetMotorSpeed(jointId: b2JointId): f32 {
	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	return joint.motorSpeed;
}

export function b2DistanceJoint_GetCurrentMotorForce(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_distanceJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseDistanceView = b2DistanceJointAt(baseAddress);
	ref baseDistance = unchecked(baseDistanceView[0]);
	return world.inv_h * baseDistance.motorImpulse;
}

export function b2DistanceJoint_SetMaxMotorForce(jointId: b2JointId, force: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(force), "b2IsValidFloat( force )", "b2DistanceJoint_SetMaxMotorForce")) return;

	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	joint.maxMotorForce = force;
}

export function b2DistanceJoint_GetMaxMotorForce(jointId: b2JointId): f32 {
	let joints = b2DistanceJointAt(b2GetJointSimAddressCheckType(jointId, b2_distanceJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxMotorForce;
}

export function b2GetDistanceJointForce(world: b2World, baseAddress: usize): b2Vec2 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	let worldTransformA = b2GetBodyTransform(world, base.bodyIdA);
	let transformA = b2ToRelativeTransform(worldTransformA, worldTransformA.p);
	let worldTransformB = b2GetBodyTransform(world, base.bodyIdB);
	let transformB = b2ToRelativeTransform(worldTransformB, worldTransformA.p);

	let pA = b2TransformPoint(transformA, base.localFrameA.p);
	let pB = b2TransformPoint(transformB, base.localFrameB.p);
	let d = b2Sub(pB, pA);
	let axis = b2Normalize(d);
	let force: f32 = (joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse) * world.inv_h;
	return b2MulSV(force, axis);
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

@noinline
export function b2PrepareDistanceJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_distanceJoint);

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

	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	joint.indexA = setIndexA == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = setIndexB == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// initial anchors in world space
	joint.anchorA = b2RotateVector(bodySimA.transform.q, b2Sub(base.localFrameA.p, bodySimA.localCenter));
	joint.anchorB = b2RotateVector(bodySimB.transform.q, b2Sub(base.localFrameB.p, bodySimB.localCenter));
	joint.deltaCenter = b2SubPos(bodySimB.center, bodySimA.center);

	let rA = joint.anchorA;
	let rB = joint.anchorB;
	let separation = b2Add(b2Sub(rB, rA), joint.deltaCenter);
	let axis = b2Normalize(separation);

	// compute effective mass
	let crA: f32 = b2Cross(rA, axis);
	let crB: f32 = b2Cross(rB, axis);
	let k: f32 = mA + mB + iA * crA * crA + iB * crB * crB;
	joint.axialMass = k > 0.0 ? 1.0 / k : 0.0;

	joint.distanceSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.impulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
		joint.motorImpulse = 0.0;
	}
}

@noinline
export function b2WarmStartDistanceJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_distanceJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let rA = b2RotateVector(stateA.deltaRotation, joint.anchorA);
	let rB = b2RotateVector(stateB.deltaRotation, joint.anchorB);

	let ds = b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), b2Sub(rB, rA));
	let separation = b2Add(joint.deltaCenter, ds);
	let axis = b2Normalize(separation);

	let axialImpulse: f32 = joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse;
	let P = b2MulSV(axialImpulse, axis);

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, b2MulSub(stateA.linearVelocity, mA, P), stateA.angularVelocity - iA * b2Cross(rA, P));
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, b2MulAdd(stateB.linearVelocity, mB, P), stateB.angularVelocity + iB * b2Cross(rB, P));
	}
}

@noinline
export function b2SolveDistanceJoint(baseAddress: usize, context: b2StepContext, useBias: bool): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_distanceJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2DistanceJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let wA: f32 = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB: f32 = stateB.angularVelocity;

	// current anchors
	let rA = b2RotateVector(stateA.deltaRotation, joint.anchorA);
	let rB = b2RotateVector(stateB.deltaRotation, joint.anchorB);

	// current separation
	let ds = b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), b2Sub(rB, rA));
	let separation = b2Add(joint.deltaCenter, ds);

	let length: f32 = b2Length(separation);
	let axis = b2Normalize(separation);

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if (joint.enableSpring && (joint.minLength < joint.maxLength || joint.enableLimit == false)) {
		// spring
		if (joint.hertz > 0.0) {
			// Cdot = dot(u, v + cross(w, r))
			let vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
			let cdot: f32 = b2Dot(axis, vr);
			let c: f32 = length - joint.length;
			let bias: f32 = joint.distanceSoftness.biasRate * c;

			let m: f32 = joint.distanceSoftness.massScale * joint.axialMass;
			let oldImpulse: f32 = joint.impulse;
			let impulse: f32 = -m * (cdot + bias) - joint.distanceSoftness.impulseScale * oldImpulse;

			let h: f32 = context.h;
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
			let Cdot: f32 = b2Dot(axis, vr);
			let impulse: f32 = joint.axialMass * (joint.motorSpeed - Cdot);
			let oldImpulse: f32 = joint.motorImpulse;
			let maxImpulse: f32 = context.h * joint.maxMotorForce;
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
				let cdot: f32 = b2Dot(axis, vr);

				let c: f32 = length - joint.minLength;

				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;
				if (c > 0.0) {
					// speculative
					bias = c * context.inv_h;
				} else if (useBias) {
					bias = base.constraintSoftness.biasRate * c;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				let impulse: f32 = -massScale * joint.axialMass * (cdot + bias) - impulseScale * joint.lowerImpulse;
				let newImpulse: f32 = b2MaxFloat(0.0, joint.lowerImpulse + impulse);
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
				let Cdot: f32 = b2Dot(axis, vr);

				let C: f32 = joint.maxLength - length;

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

				let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.upperImpulse;
				let newImpulse: f32 = b2MaxFloat(0.0, joint.upperImpulse + impulse);
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
		let Cdot: f32 = b2Dot(axis, vr);

		let C: f32 = length - joint.length;

		let bias: f32 = 0.0;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias) {
			bias = base.constraintSoftness.biasRate * C;
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.impulse;
		joint.impulse += impulse;

		let P = b2MulSV(impulse, axis);
		vA = b2MulSub(vA, mA, P);
		wA -= iA * b2Cross(rA, P);
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(rB, P);
	}

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, vA, wA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, vB, wB);
	}
}
