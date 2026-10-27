// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/revolute_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import {
	B2_PI,
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2InvMulRot,
	b2IsValidFloat,
	b2Mat22,
	b2MaxFloat,
	b2MinFloat,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2RelativeAngle,
	b2RotateVector,
	b2Rot_GetAngle,
	b2Solve22,
	b2Sub,
	b2SubPos,
	b2UnwindAngle,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2JointId } from "./id";
import { b2_revoluteJoint } from "./types";
import { b2BodyState, b2GetBodyState, b2GetBodyTransform, b2StateAddress, b2_dynamicFlag, b2BodySimView } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2MakeSoft, b2StepContext } from "./solver";
import { b2GetWorld, b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2GetJointLocalIndex,
	b2GetJointSimViewCheckType,
	b2JointSim,
	b2WriteStateVelocity,
} from "./joint";

// Point-to-point constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// J = [-E -skew(rA) E skew(rB) ]

// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
// K = invIA + invIB

export unsafe function b2RevoluteJoint_EnableSpring(jointId: b2JointId, enableSpring: bool): void {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	if (enableSpring != joint.revoluteJoint.enableSpring) {
		joint.revoluteJoint.enableSpring = enableSpring;
		joint.revoluteJoint.springImpulse = 0.0;
	}
}

export unsafe function b2RevoluteJoint_IsSpringEnabled(jointId: b2JointId): bool {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.enableSpring;
}

export unsafe function b2RevoluteJoint_SetSpringHertz(jointId: b2JointId, hertz: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(hertz), "b2IsValidFloat( hertz )", "b2RevoluteJoint_SetSpringHertz")) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.revoluteJoint.hertz = hertz;
}

export unsafe function b2RevoluteJoint_GetSpringHertz(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.hertz;
}

export unsafe function b2RevoluteJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio), "b2IsValidFloat( dampingRatio )", "b2RevoluteJoint_SetSpringDampingRatio")) {
		return;
	}

	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.revoluteJoint.dampingRatio = dampingRatio;
}

export unsafe function b2RevoluteJoint_GetSpringDampingRatio(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.dampingRatio;
}

export unsafe function b2RevoluteJoint_SetTargetAngle(jointId: b2JointId, angle: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(angle), "b2IsValidFloat( angle )", "b2RevoluteJoint_SetTargetAngle")) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.revoluteJoint.targetAngle = angle;
}

export unsafe function b2RevoluteJoint_GetTargetAngle(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.targetAngle;
}

export unsafe function b2RevoluteJoint_EnableLimit(jointId: b2JointId, enableLimit: bool): void {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	if (enableLimit != joint.revoluteJoint.enableLimit) {
		joint.revoluteJoint.enableLimit = enableLimit;
		joint.revoluteJoint.lowerImpulse = 0.0;
		joint.revoluteJoint.upperImpulse = 0.0;
	}
}

export unsafe function b2RevoluteJoint_IsLimitEnabled(jointId: b2JointId): bool {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.enableLimit;
}

export unsafe function b2RevoluteJoint_GetLowerLimit(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.lowerAngle;
}

export unsafe function b2RevoluteJoint_GetUpperLimit(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.upperAngle;
}

export unsafe function b2RevoluteJoint_SetLimits(jointId: b2JointId, lower: f32, upper: f32): void {
	const fn = "b2RevoluteJoint_SetLimits";
	if (!b2CheckJointInput(b2IsValidFloat(lower), "b2IsValidFloat( lower )", fn)) return;
	if (!b2CheckJointInput(b2IsValidFloat(upper), "b2IsValidFloat( upper )", fn)) return;
	B2_ASSERT(lower <= upper);

	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	let lowerAngle: f32 = b2MinFloat(lower, upper);
	let upperAngle: f32 = b2MaxFloat(lower, upper);
	joint.revoluteJoint.lowerAngle = b2ClampFloat(lowerAngle, -0.99 * B2_PI, 0.99 * B2_PI);
	joint.revoluteJoint.upperAngle = b2ClampFloat(upperAngle, -0.99 * B2_PI, 0.99 * B2_PI);
}

export unsafe function b2RevoluteJoint_EnableMotor(jointId: b2JointId, enableMotor: bool): void {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	if (enableMotor != joint.revoluteJoint.enableMotor) {
		joint.revoluteJoint.enableMotor = enableMotor;
		joint.revoluteJoint.motorImpulse = 0.0;
	}
}

export unsafe function b2RevoluteJoint_IsMotorEnabled(jointId: b2JointId): bool {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.enableMotor;
}

export unsafe function b2RevoluteJoint_SetTargetMotorSpeed(jointId: b2JointId, motorSpeed: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(motorSpeed), "b2IsValidFloat( motorSpeed )", "b2RevoluteJoint_SetTargetMotorSpeed")) {
		return;
	}

	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.revoluteJoint.motorSpeed = motorSpeed;
}

export unsafe function b2RevoluteJoint_GetTargetMotorSpeed(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.motorSpeed;
}

export unsafe function b2RevoluteJoint_SetMaxMotorTorque(jointId: b2JointId, torque: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(torque), "b2IsValidFloat( torque )", "b2RevoluteJoint_SetMaxMotorTorque")) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.revoluteJoint.maxMotorTorque = torque;
}

export unsafe function b2RevoluteJoint_GetMaxMotorTorque(jointId: b2JointId): f32 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.revoluteJoint.maxMotorTorque;
}

export unsafe function b2RevoluteJoint_GetCurrentAngle(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref jointSim = sims[b2GetJointLocalIndex(jointId)];
	let qA = b2MulRot(b2GetBodyTransform(world, jointSim.bodyIdA).q, jointSim.localFrameA.q);
	let qB = b2MulRot(b2GetBodyTransform(world, jointSim.bodyIdB).q, jointSim.localFrameB.q);

	let angle: f32 = b2RelativeAngle(qA, qB);
	return angle;
}

export unsafe function b2RevoluteJoint_GetCurrentSpeed(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref base = sims[b2GetJointLocalIndex(jointId)];

	// b2GetBodyState returns an awake state index, B2_NULL_INDEX where C returns NULL
	let bodyStateA = b2GetBodyState(world, base.bodyIdA);
	let bodyStateB = b2GetBodyState(world, base.bodyIdB);

	let states = world.solverSets[b2_awakeSet].bodyStates;
	let wA: f32 = bodyStateA != B2_NULL_INDEX ? states[bodyStateA].angularVelocity : 0.0;
	let wB: f32 = bodyStateB != B2_NULL_INDEX ? states[bodyStateB].angularVelocity : 0.0;
	return wB - wA;
}

export unsafe function b2RevoluteJoint_GetCurrentMotorTorque(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let sims = b2GetJointSimViewCheckType(jointId, b2_revoluteJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return world.inv_h * joint.revoluteJoint.motorImpulse;
}

export function b2GetRevoluteJointForce(world: b2World, base: ref b2JointSim): b2Vec2 {
	let force = b2MulSV(world.inv_h, base.revoluteJoint.linearImpulse);
	return force;
}

export function b2GetRevoluteJointTorque(world: b2World, base: ref b2JointSim): f32 {
	ref revolute = base.revoluteJoint;
	let torque: f32 = world.inv_h * (revolute.motorImpulse + revolute.lowerImpulse - revolute.upperImpulse);
	return torque;
}

export function b2PrepareRevoluteJoint(base: ref b2JointSim, context: b2StepContext): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_revoluteJoint);

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

	ref joint = base.revoluteJoint;

	joint.indexA = setIndexA == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = setIndexB == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

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
	joint.deltaCenter = b2SubPos(bodySimB.center, bodySimA.center);

	let k: f32 = iA + iB;
	joint.axialMass = k > 0.0 ? 1.0 / k : 0.0;

	joint.springSoftness = b2MakeSoft(joint.hertz, joint.dampingRatio, context.h);

	if (context.enableWarmStarting == false) {
		joint.linearImpulse = b2Vec2_zero;
		joint.springImpulse = 0.0;
		joint.motorImpulse = 0.0;
		joint.lowerImpulse = 0.0;
		joint.upperImpulse = 0.0;
	}
}

export unsafe function b2WarmStartRevoluteJoint(base: ref b2JointSim, context: b2StepContext): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_revoluteJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	ref joint = base.revoluteJoint;
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let rA = b2RotateVector(stateA.deltaRotation, joint.frameA.p);
	let rB = b2RotateVector(stateB.deltaRotation, joint.frameB.p);

	let axialImpulse: f32 = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressA,
			b2MulSub(stateA.linearVelocity, mA, joint.linearImpulse),
			stateA.angularVelocity - iA * (b2Cross(rA, joint.linearImpulse) + axialImpulse),
		);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(
			addressB,
			b2MulAdd(stateB.linearVelocity, mB, joint.linearImpulse),
			stateB.angularVelocity + iB * (b2Cross(rB, joint.linearImpulse) + axialImpulse),
		);
	}
}

export unsafe function b2SolveRevoluteJoint(base: ref b2JointSim, context: b2StepContext, useBias: bool): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_revoluteJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;
	let iA: f32 = base.invIA;
	let iB: f32 = base.invIB;

	// dummy state for static bodies
	ref joint = base.revoluteJoint;
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

	let linearCdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
	let angularCdot: f32 = wB - wA;
	let axialImpulse: f32 = 0.0;

	let k: f32 = iA + iB;
	let fixedRotation = k == 0.0;

	// Solve spring.
	if (joint.enableSpring && fixedRotation == false) {
		let jointAngle: f32 = b2Rot_GetAngle(relQ);
		let jointAngleDelta: f32 = b2UnwindAngle(jointAngle - joint.targetAngle);

		let C: f32 = jointAngleDelta;
		let bias: f32 = joint.springSoftness.biasRate * C;
		let massScale: f32 = joint.springSoftness.massScale;
		let impulseScale: f32 = joint.springSoftness.impulseScale;

		let Cdot: f32 = angularCdot;
		let impulse: f32 = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.springImpulse;
		joint.springImpulse += impulse;

		angularCdot += k * impulse;
		axialImpulse += impulse;
	}

	// Solve motor constraint.
	if (joint.enableMotor && fixedRotation == false) {
		let Cdot: f32 = angularCdot - joint.motorSpeed;
		let oldImpulse: f32 = joint.motorImpulse;
		let maxImpulse: f32 = context.h * joint.maxMotorTorque;
		let impulse: f32 = b2ClampFloat(-joint.axialMass * Cdot, -maxImpulse - oldImpulse, maxImpulse - oldImpulse);
		joint.motorImpulse = oldImpulse + impulse;

		angularCdot += k * impulse;
		axialImpulse += impulse;
	}

	if (joint.enableLimit && fixedRotation == false) {
		let jointAngle: f32 = b2Rot_GetAngle(relQ);

		// Lower limit
		{
			let C: f32 = jointAngle - joint.lowerAngle;
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

			let Cdot: f32 = angularCdot;
			let oldImpulse: f32 = joint.lowerImpulse;
			let impulse: f32 = b2MaxFloat(-massScale * joint.axialMass * (Cdot + bias) - impulseScale * oldImpulse, -oldImpulse);
			joint.lowerImpulse = oldImpulse + impulse;

			angularCdot += k * impulse;
			axialImpulse += impulse;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			let C: f32 = joint.upperAngle - jointAngle;
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
			let Cdot: f32 = -angularCdot;
			let oldImpulse: f32 = joint.upperImpulse;
			let impulse: f32 = b2MaxFloat(-massScale * joint.axialMass * (Cdot + bias) - impulseScale * oldImpulse, -oldImpulse);
			joint.upperImpulse = oldImpulse + impulse;

			// sign flipped on applied impulse
			axialImpulse -= impulse;
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		let Cdot = b2MulAdd(linearCdot, axialImpulse, b2Add(b2CrossSV(iA, rA), b2CrossSV(iB, rB)));

		let bias = b2Vec2_zero;
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

		let K = b2Mat22();
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		let b = b2Solve22(K, b2Add(Cdot, bias));

		let impulse = b2Vec2();
		impulse.x = -massScale * b.x - impulseScale * joint.linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint.linearImpulse.y;
		joint.linearImpulse.x += impulse.x;
		joint.linearImpulse.y += impulse.y;

		vA = b2MulSub(vA, mA, impulse);
		wA -= iA * (b2Cross(rA, impulse) + axialImpulse);
		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * (b2Cross(rB, impulse) + axialImpulse);
	}

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressA, vA, wA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateVelocity(addressB, vB, wB);
	}
}
