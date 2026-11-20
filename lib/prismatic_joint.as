// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/prismatic_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX, b2GetLengthUnitsPerMeter } from "./core";
import {
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2Dot,
	b2InvMulRot,
	b2IsValidFloat,
	b2IsValidVec2,
	b2LeftPerp,
	b2Mat22,
	b2MaxFloat,
	b2MinFloat,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2RotateVector,
	b2Rot_GetAngle,
	b2Solve22,
	b2Sub,
	b2SubPos,
	b2ToRelativeTransform,
	b2TransformPoint,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_prismaticJoint } from "./types";
import { b2BodyState, b2GetBodySim, b2GetBodyState, b2GetBodyTransform, b2StateAddress, b2_dynamicFlag, b2ArrayView } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2MakeSoft, b2StepContext } from "./solver";
import { b2GetWorld, b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2GetJointFullId,
	b2GetJointLocalIndex,
	b2GetJointSimAddressCheckType,
	b2GetJointSimView,
	b2GetJointSimViewCheckType,
	b2JointSimAddress,
	b2JointSimAt,
	b2PrismaticJointAt,
	b2WriteStateVelocity,
} from "./joint";

export function b2PrismaticJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	if (enableSpring != joint.enableSpring) {
		joint.enableSpring = enableSpring;
		joint.springImpulse = 0.0;
	}
}

export function b2PrismaticJoint_IsSpringEnabled(jointId: b2JointId): bool {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableSpring;
}

export function b2PrismaticJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2PrismaticJoint_SetSpringHertz")) return;

	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	joint.hertz = hertz;
}

export function b2PrismaticJoint_GetSpringHertz(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.hertz;
}

export function b2PrismaticJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2PrismaticJoint_SetSpringDampingRatio")) {
		return;
	}

	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	joint.dampingRatio = dampingRatio;
}

export function b2PrismaticJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.dampingRatio;
}

export function b2PrismaticJoint_SetTargetTranslation(jointId: b2JointId, translation: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(translation), "b2IsValidFloat( translation )", "b2PrismaticJoint_SetTargetTranslation")) {
		return;
	}

	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	joint.targetTranslation = translation;
}

export function b2PrismaticJoint_GetTargetTranslation(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.targetTranslation;
}

export function b2PrismaticJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	if (enableLimit != joint.enableLimit) {
		joint.enableLimit = enableLimit;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
	}
}

export function b2PrismaticJoint_IsLimitEnabled(jointId: b2JointId): bool {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableLimit;
}

export function b2PrismaticJoint_GetLowerLimit(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.lowerTranslation;
}

export function b2PrismaticJoint_GetUpperLimit(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.upperTranslation;
}

export function b2PrismaticJoint_SetLimits(jointId: b2JointId, lower: f32, upper: f32): void {
	const fn = "b2PrismaticJoint_SetLimits";
	if (!b2CheckJointInput(b2IsValidFloat(lower), "b2IsValidFloat( lower )", fn)) return;
	if (!b2CheckJointInput(b2IsValidFloat(upper), "b2IsValidFloat( upper )", fn)) return;

	B2_ASSERT(lower <= upper);

	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	joint.lowerTranslation = b2MinFloat(lower, upper);
	joint.upperTranslation = b2MaxFloat(lower, upper);
}

export function b2PrismaticJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	if (enableMotor != joint.enableMotor) {
		joint.enableMotor = enableMotor;
		joint.motorImpulse = 0.0;
	}
}

export function b2PrismaticJoint_IsMotorEnabled(jointId: b2JointId): bool {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableMotor;
}

export function b2PrismaticJoint_SetTargetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(motorSpeed), "b2IsValidFloat( motorSpeed )", "b2PrismaticJoint_SetTargetMotorSpeed")) {
		return;
	}

	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	joint.motorSpeed = motorSpeed;
}

export function b2PrismaticJoint_GetTargetMotorSpeed(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.motorSpeed;
}

export function b2PrismaticJoint_GetCurrentMotorForce(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_prismaticJoint);
	let baseAddress = b2JointSimAddress(sims, b2GetJointLocalIndex(jointId));
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let basePrismaticView = b2PrismaticJointAt(baseAddress);
	ref basePrismatic = unchecked(basePrismaticView[0]);
	return world.inv_h * basePrismatic.motorImpulse;
}

export function b2PrismaticJoint_SetMaxMotorForce(jointId: b2JointId, force: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(force), "b2IsValidFloat( force )", "b2PrismaticJoint_SetMaxMotorForce")) return;

	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	joint.maxMotorForce = force;
}

export function b2PrismaticJoint_GetMaxMotorForce(jointId: b2JointId): f32 {
	let joints = b2PrismaticJointAt(b2GetJointSimAddressCheckType(jointId, b2_prismaticJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxMotorForce;
}

export function b2PrismaticJoint_GetCurrentTranslation(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_prismaticJoint);
	ref jointSim = sims[b2GetJointLocalIndex(jointId)];
	// Relative to body A so the difference stays in float precision far from the origin
	let wxfA = b2GetBodyTransform(world, jointSim.bodyIdA);
	let transformA = b2ToRelativeTransform(wxfA, wxfA.p);
	let transformB = b2ToRelativeTransform(b2GetBodyTransform(world, jointSim.bodyIdB), wxfA.p);

	let localAxisA = b2RotateVector(jointSim.localFrameA.q, b2Vec2(1.0, 0.0));
	let axisA = b2RotateVector(transformA.q, localAxisA);
	let pA = b2TransformPoint(transformA, jointSim.localFrameA.p);
	let pB = b2TransformPoint(transformB, jointSim.localFrameB.p);
	let d = b2Sub(pB, pA);
	let translation: f32 = b2Dot(d, axisA);
	return translation;
}

export function b2PrismaticJoint_GetCurrentSpeed(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	B2_ASSERT(world.joints[joint].type == b2_prismaticJoint);
	let view = b2GetJointSimView(world, joint);
	ref base = view[world.joints[joint].localIndex];
	B2_ASSERT(base.type == b2_prismaticJoint);

	let bodySimA = b2GetBodySim(world, base.bodyIdA);
	let bodySimB = b2GetBodySim(world, base.bodyIdB);
	let bodyStateA = b2GetBodyState(world, base.bodyIdA);
	let bodyStateB = b2GetBodyState(world, base.bodyIdB);

	let qA = bodySimA.transform.q;
	let qB = bodySimB.transform.q;

	let localAxisA = b2RotateVector(base.localFrameA.q, b2Vec2(1.0, 0.0));
	let axisA = b2RotateVector(qA, localAxisA);
	let rA = b2RotateVector(qA, b2Sub(base.localFrameA.p, bodySimA.localCenter));
	let rB = b2RotateVector(qB, b2Sub(base.localFrameB.p, bodySimB.localCenter));

	// Difference the centers in double so the speed stays exact far from the origin
	let dc = b2SubPos(bodySimB.center, bodySimA.center);
	let d = b2Add(dc, b2Sub(rB, rA));

	// b2GetBodyState returns an awake state index, B2_NULL_INDEX where C returns NULL
	let states = world.solverSets[b2_awakeSet].bodyStates;
	let vA = bodyStateA != B2_NULL_INDEX ? states[bodyStateA].linearVelocity : b2Vec2_zero;
	let vB = bodyStateB != B2_NULL_INDEX ? states[bodyStateB].linearVelocity : b2Vec2_zero;
	let wA: f32 = bodyStateA != B2_NULL_INDEX ? states[bodyStateA].angularVelocity : 0.0;
	let wB: f32 = bodyStateB != B2_NULL_INDEX ? states[bodyStateB].angularVelocity : 0.0;

	let vRel = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
	let speed: f32 = b2Dot(d, b2CrossSV(wA, axisA)) + b2Dot(axisA, vRel);
	return speed;
}

export function b2GetPrismaticJointForce(world: b2World, baseAddress: usize): b2Vec2 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let qA = b2GetBodyTransform(world, base.bodyIdA).q;

	let jointView = b2PrismaticJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	let localAxisA = b2RotateVector(base.localFrameA.q, b2Vec2(1.0, 0.0));
	let axisA = b2RotateVector(qA, localAxisA);
	let perpA = b2LeftPerp(axisA);

	let inv_h: f32 = world.inv_h;
	let perpForce: f32 = inv_h * joint.impulse.x;
	let axialForce: f32 = inv_h * (joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse);

	let force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
	return force;
}

export function b2GetPrismaticJointTorque(world: b2World, baseAddress: usize): f32 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let basePrismaticView = b2PrismaticJointAt(baseAddress);
	ref basePrismatic = unchecked(basePrismaticView[0]);
	return world.inv_h * basePrismatic.impulse.y;
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(perp, d)
// Cdot = dot(d, cross(wA, perp)) + dot(perp, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perp, vA) - dot(cross(rA + d, perp), wA) + dot(perp, vB) + dot(cross(rB, perp), vB)
// J = [-perp, -cross(rA + d, perp), perp, cross(rB, perp)]
//
// Angular constraint
// C = aB - aA + a_initial
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -sA a sB]
//     [0  -1  0  1]
// a = perp
// sA = cross(rA + d, a) = cross(pB - xA, a)
// sB = cross(rB, a) = cross(pB - xB, a)

// Motor/Limit linear constraint
// C = dot(axA, d)
// Cdot = -dot(axA, vA) - dot(cross(rA + d, axA), wA) + dot(axA, vB) + dot(cross(rB, axA), vB)
// J = [-axA -cross(rA + d, axA) axA cross(rB, ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

@noinline
export function b2PreparePrismaticJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_prismaticJoint);

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

	let jointView = b2PrismaticJointAt(baseAddress);
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

	joint.springSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.impulse = b2Vec2_zero;
		joint.springImpulse = 0.0;
		joint.motorImpulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
	}
}

@noinline
export function b2WarmStartPrismaticJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_prismaticJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2PrismaticJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));
	let axisA = b2RotateVector(joint.frameA.q, b2Vec2(1.0, 0.0));
	axisA = b2RotateVector(stateA.deltaRotation, axisA);

	// impulse is applied at anchor point on body B
	let a1: f32 = b2Cross(b2Add(rA, d), axisA);
	let a2: f32 = b2Cross(rB, axisA);
	let axialImpulse: f32 = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	// perpendicular constraint
	let perpA = b2LeftPerp(axisA);
	let s1: f32 = b2Cross(b2Add(rA, d), perpA);
	let s2: f32 = b2Cross(rB, perpA);
	let perpImpulse: f32 = joint.impulse.x;
	let angleImpulse: f32 = joint.impulse.y;

	let P = b2Add(b2MulSV(axialImpulse, axisA), b2MulSV(perpImpulse, perpA));
	let LA: f32 = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
	let LB: f32 = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, b2MulSub(stateA.linearVelocity, mA, P), stateA.angularVelocity - iA * LA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, b2MulAdd(stateB.linearVelocity, mB, P), stateB.angularVelocity + iB * LB);
	}
}

@noinline
export function b2SolvePrismaticJoint(baseAddress: usize, context: b2StepContext, useBias: bool): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_prismaticJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2PrismaticJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let wA: f32 = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB: f32 = stateB.angularVelocity;

	let qA = b2MulRot(stateA.deltaRotation, joint.frameA.q);
	let qB = b2MulRot(stateB.deltaRotation, joint.frameB.q);
	let relQ = b2InvMulRot(qA, qB);

	// current anchors
	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));
	let axisA = b2RotateVector(joint.frameA.q, b2Vec2(1.0, 0.0));
	axisA = b2RotateVector(stateA.deltaRotation, axisA);
	let translation: f32 = b2Dot(axisA, d);

	// These scalars are for torques generated by axial forces
	let a1: f32 = b2Cross(b2Add(rA, d), axisA);
	let a2: f32 = b2Cross(rB, axisA);

	let k: f32 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	let axialMass: f32 = k > 0.0 ? 1.0 / k : 0.0;

	let softness = base.constraintSoftness;

	// spring constraint
	if (joint.enableSpring) {
		// This is a real spring and should be applied even during relax
		let C: f32 = translation - joint.targetTranslation;
		let bias: f32 = joint.springSoftness.biasRate * C;
		let massScale: f32 = joint.springSoftness.massScale;
		let impulseScale: f32 = joint.springSoftness.impulseScale;

		let Cdot: f32 = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		let deltaImpulse: f32 = -massScale * axialMass * (Cdot + bias) - impulseScale * joint.springImpulse;
		joint.springImpulse += deltaImpulse;

		let P = b2MulSV(deltaImpulse, axisA);
		let LA: f32 = deltaImpulse * a1;
		let LB: f32 = deltaImpulse * a2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	// Solve motor constraint
	if (joint.enableMotor) {
		let Cdot: f32 = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		let impulse: f32 = axialMass * (joint.motorSpeed - Cdot);
		let oldImpulse: f32 = joint.motorImpulse;
		let maxImpulse: f32 = context.h * joint.maxMotorForce;
		joint.motorImpulse = b2ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint.motorImpulse - oldImpulse;

		let P = b2MulSV(impulse, axisA);
		let LA: f32 = impulse * a1;
		let LB: f32 = impulse * a2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	if (joint.enableLimit) {
		// Clamp the speculative distance to a reasonable value
		let speculativeDistance: f32 = 0.25 * (joint.upperTranslation - joint.lowerTranslation);

		// Lower limit
		{
			let C: f32 = translation - joint.lowerTranslation;

			if (C < speculativeDistance) {
				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;

				if (C > 0.0) {
					// speculation
					let safe: f32 = b2GetLengthUnitsPerMeter();
					bias = b2MinFloat(C, safe) * context.inv_h;
				} else if (useBias) {
					bias = softness.biasRate * C;
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				let oldImpulse: f32 = joint.lowerImpulse;
				let Cdot: f32 = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
				let deltaImpulse: f32 = -axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
				joint.lowerImpulse = b2MaxFloat(oldImpulse + deltaImpulse, 0.0);
				deltaImpulse = joint.lowerImpulse - oldImpulse;

				let P = b2MulSV(deltaImpulse, axisA);
				let LA: f32 = deltaImpulse * a1;
				let LB: f32 = deltaImpulse * a2;

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
			let C: f32 = joint.upperTranslation - translation;

			if (C < speculativeDistance) {
				let bias: f32 = 0.0;
				let massScale: f32 = 1.0;
				let impulseScale: f32 = 0.0;

				if (C > 0.0) {
					// speculation
					let safe: f32 = b2GetLengthUnitsPerMeter();
					bias = b2MinFloat(C, safe) * context.inv_h;
				} else if (useBias) {
					bias = softness.biasRate * C;
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				let oldImpulse: f32 = joint.upperImpulse;
				// sign flipped
				let Cdot: f32 = b2Dot(axisA, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
				let deltaImpulse: f32 = -axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
				joint.upperImpulse = b2MaxFloat(oldImpulse + deltaImpulse, 0.0);
				deltaImpulse = joint.upperImpulse - oldImpulse;

				let P = b2MulSV(deltaImpulse, axisA);
				let LA: f32 = deltaImpulse * a1;
				let LB: f32 = deltaImpulse * a2;

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
		let s1: f32 = b2Cross(b2Add(d, rA), perpA);
		let s2: f32 = b2Cross(rB, perpA);

		let Cdot = b2Vec2();
		Cdot.x = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;
		Cdot.y = wB - wA;

		let bias = b2Vec2_zero;
		let massScale: f32 = 1.0;
		let impulseScale: f32 = 0.0;
		if (useBias) {
			let C = b2Vec2();
			C.x = b2Dot(perpA, d);
			C.y = b2Rot_GetAngle(relQ);

			bias = b2MulSV(softness.biasRate, C);
			massScale = softness.massScale;
			impulseScale = softness.impulseScale;
		}

		let k11: f32 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		let k12: f32 = iA * s1 + iB * s2;
		let k22: f32 = iA + iB;
		if (k22 == 0.0) {
			// For bodies with fixed rotation.
			k22 = 1.0;
		}

		let K = b2Mat22(b2Vec2(k11, k12), b2Vec2(k12, k22));

		let b = b2Solve22(K, b2Add(Cdot, bias));

		let deltaImpulse = b2Vec2();
		deltaImpulse.x = -massScale * b.x - impulseScale * joint.impulse.x;
		deltaImpulse.y = -massScale * b.y - impulseScale * joint.impulse.y;

		joint.impulse.x += deltaImpulse.x;
		joint.impulse.y += deltaImpulse.y;

		let P = b2MulSV(deltaImpulse.x, perpA);
		let LA: f32 = deltaImpulse.x * s1 + deltaImpulse.y;
		let LB: f32 = deltaImpulse.x * s2 + deltaImpulse.y;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
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
