// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/wheel_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import {
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2Dot,
	b2IsValidFloat,
	b2LeftPerp,
	b2MaxFloat,
	b2MinFloat,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2RotateVector,
	b2Sub,
	b2SubPos,
	b2Vec2,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_wheelJoint } from "./types";
import { b2BodyState, b2GetBodyTransform, b2StateAddress, b2_dynamicFlag, b2ArrayView } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2MakeSoft, b2StepContext } from "./solver";
import { b2GetWorld, b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2GetJointSimAddressCheckType,
	b2JointSimAt,
	b2WheelJointAt,
	b2WriteStateVelocity,
} from "./joint";

export unsafe function b2WheelJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	if (enableSpring != joint.enableSpring) {
		joint.enableSpring = enableSpring;
		joint.springImpulse = 0.0;
	}
}

export unsafe function b2WheelJoint_IsSpringEnabled(jointId: b2JointId): bool {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableSpring;
}

export unsafe function b2WheelJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2WheelJoint_SetSpringHertz")) return;

	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	joint.hertz = hertz;
}

export unsafe function b2WheelJoint_GetSpringHertz(jointId: b2JointId): f32 {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.hertz;
}

export unsafe function b2WheelJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2WheelJoint_SetSpringDampingRatio")) {
		return;
	}

	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	joint.dampingRatio = dampingRatio;
}

export unsafe function b2WheelJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.dampingRatio;
}

export unsafe function b2WheelJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	if (joint.enableLimit != enableLimit) {
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
		joint.enableLimit = enableLimit;
	}
}

export unsafe function b2WheelJoint_IsLimitEnabled(jointId: b2JointId): bool {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableLimit;
}

export unsafe function b2WheelJoint_GetLowerLimit(jointId: b2JointId): f32 {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.lowerTranslation;
}

export unsafe function b2WheelJoint_GetUpperLimit(jointId: b2JointId): f32 {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.upperTranslation;
}

export unsafe function b2WheelJoint_SetLimits(jointId: b2JointId, lower: f32, upper: f32): void {
	const fn = "b2WheelJoint_SetLimits";
	if (!b2CheckJointInput(b2IsValidFloat(lower), "b2IsValidFloat( lower )", fn)) return;
	if (!b2CheckJointInput(b2IsValidFloat(upper), "b2IsValidFloat( upper )", fn)) return;

	B2_ASSERT(lower <= upper);

	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	joint.lowerTranslation = b2MinFloat(lower, upper);
	joint.upperTranslation = b2MaxFloat(lower, upper);
}

export unsafe function b2WheelJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	if (joint.enableMotor != enableMotor) {
		joint.motorImpulse = 0.0;
		joint.enableMotor = enableMotor;
	}
}

export unsafe function b2WheelJoint_IsMotorEnabled(jointId: b2JointId): bool {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.enableMotor;
}

export unsafe function b2WheelJoint_SetTargetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(motorSpeed), "b2IsValidFloat( motorSpeed )", "b2WheelJoint_SetTargetMotorSpeed")) {
		return;
	}

	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	joint.motorSpeed = motorSpeed;
}

export unsafe function b2WheelJoint_GetTargetMotorSpeed(jointId: b2JointId): f32 {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.motorSpeed;
}

export unsafe function b2WheelJoint_GetCurrentMotorTorque(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return world.inv_h * joint.motorImpulse;
}

export unsafe function b2WheelJoint_SetMaxMotorTorque(jointId: b2JointId, torque: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(torque), "b2IsValidFloat( torque )", "b2WheelJoint_SetMaxMotorTorque")) return;

	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	joint.maxMotorTorque = torque;
}

export unsafe function b2WheelJoint_GetMaxMotorTorque(jointId: b2JointId): f32 {
	let joints = b2WheelJointAt(b2GetJointSimAddressCheckType(jointId, b2_wheelJoint));
	ref joint = unchecked(joints[0]);
	return joint.maxMotorTorque;
}

export function b2GetWheelJointForce(world: b2World, baseAddress: usize): b2Vec2 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let qA = b2GetBodyTransform(world, base.bodyIdA).q;
	let localAxisA = b2RotateVector(base.localFrameA.q, b2Vec2(1.0, 0.0));
	let axisA = b2RotateVector(qA, localAxisA);
	let perpA = b2LeftPerp(axisA);

	let jointView = b2WheelJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);

	let perpForce: f32 = world.inv_h * joint.perpImpulse;
	let axialForce: f32 = world.inv_h * (joint.springImpulse + joint.lowerImpulse - joint.upperImpulse);

	let force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
	return force;
}

export function b2GetWheelJointTorque(world: b2World, baseAddress: usize): f32 {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseWheelView = b2WheelJointAt(baseAddress);
	ref baseWheel = unchecked(baseWheelView[0]);
	return world.inv_h * baseWheel.motorImpulse;
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

@noinline
export function b2PrepareWheelJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_wheelJoint);

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

	let jointView = b2WheelJointAt(baseAddress);
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

	let d = b2Add(joint.deltaCenter, b2Sub(rB, rA));
	let axisA = b2RotateVector(joint.frameA.q, b2Vec2(1.0, 0.0));
	let perpA = b2LeftPerp(axisA);

	// perpendicular constraint (keep wheel on line)
	let s1: f32 = b2Cross(b2Add(d, rA), perpA);
	let s2: f32 = b2Cross(rB, perpA);

	let kp: f32 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
	joint.perpMass = kp > 0.0 ? 1.0 / kp : 0.0;

	// spring constraint
	let a1: f32 = b2Cross(b2Add(d, rA), axisA);
	let a2: f32 = b2Cross(rB, axisA);

	let ka: f32 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	joint.axialMass = ka > 0.0 ? 1.0 / ka : 0.0;

	joint.springSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	let km: f32 = iA + iB;
	joint.motorMass = km > 0.0 ? 1.0 / km : 0.0;

	if (context.enableWarmStarting == false) {
		joint.perpImpulse = 0.0;
		joint.springImpulse = 0.0;
		joint.motorImpulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
	}
}

@noinline
export unsafe function b2WarmStartWheelJoint(baseAddress: usize, context: b2StepContext): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_wheelJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2WheelJointAt(baseAddress);
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
	let perpA = b2LeftPerp(axisA);

	let a1: f32 = b2Cross(b2Add(d, rA), axisA);
	let a2: f32 = b2Cross(rB, axisA);
	let s1: f32 = b2Cross(b2Add(d, rA), perpA);
	let s2: f32 = b2Cross(rB, perpA);

	let axialImpulse: f32 = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;

	let P = b2Add(b2MulSV(axialImpulse, axisA), b2MulSV(joint.perpImpulse, perpA));
	let LA: f32 = axialImpulse * a1 + joint.perpImpulse * s1 + joint.motorImpulse;
	let LB: f32 = axialImpulse * a2 + joint.perpImpulse * s2 + joint.motorImpulse;

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, b2MulSub(stateA.linearVelocity, mA, P), stateA.angularVelocity - iA * LA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, b2MulAdd(stateB.linearVelocity, mB, P), stateB.angularVelocity + iB * LB);
	}
}

@noinline
export unsafe function b2SolveWheelJoint(baseAddress: usize, context: b2StepContext, useBias: bool): void {
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_wheelJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	let jointView = b2WheelJointAt(baseAddress);
	ref joint = unchecked(jointView[0]);
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let wA: f32 = stateA.angularVelocity;
	let vB = stateB.linearVelocity;
	let wB: f32 = stateB.angularVelocity;

	let fixedRotation = iA + iB == 0.0;

	// current anchors
	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let d = b2Add(b2Add(b2Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), b2Sub(rB, rA));
	let axisA = b2RotateVector(joint.frameA.q, b2Vec2(1.0, 0.0));
	axisA = b2RotateVector(stateA.deltaRotation, axisA);
	let translation: f32 = b2Dot(axisA, d);

	let a1: f32 = b2Cross(b2Add(d, rA), axisA);
	let a2: f32 = b2Cross(rB, axisA);

	// motor constraint
	if (joint.enableMotor && fixedRotation == false) {
		let Cdot: f32 = wB - wA - joint.motorSpeed;
		let impulse: f32 = -joint.motorMass * Cdot;
		let oldImpulse: f32 = joint.motorImpulse;
		let maxImpulse: f32 = context.h * joint.maxMotorTorque;
		joint.motorImpulse = b2ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint.motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// spring constraint
	if (joint.enableSpring) {
		// This is a real spring and should be applied even during relax
		let C: f32 = translation;
		let bias: f32 = joint.springSoftness.biasRate * C;
		let massScale: f32 = joint.springSoftness.massScale;
		let impulseScale: f32 = joint.springSoftness.impulseScale;

		let Cdot: f32 = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.springImpulse;
		joint.springImpulse += impulse;

		let P = b2MulSV(impulse, axisA);
		let LA: f32 = impulse * a1;
		let LB: f32 = impulse * a2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	if (joint.enableLimit) {
		// Lower limit
		{
			let C: f32 = translation - joint.lowerTranslation;
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

			let Cdot: f32 = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
			let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.lowerImpulse;
			let oldImpulse: f32 = joint.lowerImpulse;
			joint.lowerImpulse = b2MaxFloat(oldImpulse + impulse, 0.0);
			impulse = joint.lowerImpulse - oldImpulse;

			let P = b2MulSV(impulse, axisA);
			let LA: f32 = impulse * a1;
			let LB: f32 = impulse * a2;

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
			let C: f32 = joint.upperTranslation - translation;
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
			let Cdot: f32 = b2Dot(axisA, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
			let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.upperImpulse;
			let oldImpulse: f32 = joint.upperImpulse;
			joint.upperImpulse = b2MaxFloat(oldImpulse + impulse, 0.0);
			impulse = joint.upperImpulse - oldImpulse;

			let P = b2MulSV(impulse, axisA);
			let LA: f32 = impulse * a1;
			let LB: f32 = impulse * a2;

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
			let C: f32 = b2Dot(perpA, d);
			bias = base.constraintSoftness.biasRate * C;
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		let s1: f32 = b2Cross(b2Add(d, rA), perpA);
		let s2: f32 = b2Cross(rB, perpA);
		let Cdot: f32 = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;

		let impulse: f32 = -massScale * joint.perpMass * (Cdot + bias) - impulseScale * joint.perpImpulse;
		joint.perpImpulse += impulse;

		let P = b2MulSV(impulse, perpA);
		let LA: f32 = impulse * s1;
		let LB: f32 = impulse * s2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, vA, wA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, vB, wB);
	}
}
