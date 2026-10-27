// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/mover_joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import { b2Add, b2Clamp, b2IsValidVec2, b2MulAdd, b2MulSV, b2MulSub, b2Neg, b2Sub, b2Vec2, b2Vec2_zero } from "./math_functions";
import { b2JointId } from "./id";
import { b2_moverJoint } from "./types";
import { b2BodyState, b2StateAddress, b2_dynamicFlag } from "./body";
import { b2_awakeSet } from "./solver_set";
import { b2StepContext } from "./solver";
import { b2World } from "./physics_world";
import {
	b2CheckJointInput,
	b2GetJointLocalIndex,
	b2GetJointSimViewCheckType,
	b2JointSim,
	b2WriteStateLinearVelocity,
} from "./joint";

export unsafe function b2MoverJoint_SetLinearVelocity(jointId: b2JointId, velocity: b2Vec2): void {
	if (!b2CheckJointInput(b2IsValidVec2(velocity), "b2IsValidVec2( velocity )", "b2MoverJoint_SetLinearVelocity")) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_moverJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.moverJoint.linearVelocity = velocity;
}

export unsafe function b2MoverJoint_GetLinearVelocity(jointId: b2JointId): b2Vec2 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_moverJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.moverJoint.linearVelocity;
}

export unsafe function b2MoverJoint_SetMaxVelocityForce(jointId: b2JointId, maxForce: b2Vec2): void {
	if (!b2CheckJointInput(b2IsValidVec2(maxForce), "b2IsValidVec2( maxForce )", "b2MoverJoint_SetMaxVelocityForce")) return;

	let sims = b2GetJointSimViewCheckType(jointId, b2_moverJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	joint.moverJoint.maxVelocityForce = maxForce;
}

export unsafe function b2MoverJoint_GetMaxVelocityForce(jointId: b2JointId): b2Vec2 {
	let sims = b2GetJointSimViewCheckType(jointId, b2_moverJoint);
	ref joint = sims[b2GetJointLocalIndex(jointId)];
	return joint.moverJoint.maxVelocityForce;
}

export function b2GetMoverJointForce(world: b2World, base: ref b2JointSim): b2Vec2 {
	let force = b2MulSV(world.inv_h, base.moverJoint.linearVelocityImpulse);
	return force;
}

export function b2PrepareMoverJoint(base: ref b2JointSim, context: b2StepContext): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_moverJoint);

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

	let mA: f32 = setA.bodySims[localIndexA].invMass;
	let mB: f32 = setB.bodySims[localIndexB].invMass;

	base.invMassA = mA;
	base.invMassB = mB;
	base.invIA = 0.0;
	base.invIB = 0.0;

	ref joint = base.moverJoint;
	joint.indexA = setIndexA == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = setIndexB == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	let k: f32 = mA + mB;
	joint.linearMass = k > 0.0 ? 1.0 / k : 0.0;

	if (context.enableWarmStarting == false) {
		joint.linearVelocityImpulse = b2Vec2_zero;
	}
}

export unsafe function b2WarmStartMoverJoint(base: ref b2JointSim, context: b2StepContext): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_moverJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;

	ref joint = base.moverJoint;

	// dummy state for static bodies
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateLinearVelocity(addressA, b2MulSub(stateA.linearVelocity, mA, joint.linearVelocityImpulse));
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateLinearVelocity(addressB, b2MulAdd(stateB.linearVelocity, mB, joint.linearVelocityImpulse));
	}
}

export unsafe function b2SolveMoverJoint(base: ref b2JointSim, context: b2StepContext): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(base.type == b2_moverJoint);

	let mA: f32 = base.invMassA;
	let mB: f32 = base.invMassB;

	// dummy state for static bodies
	ref joint = base.moverJoint;
	let addressA = b2StateAddress(context.states, joint.indexA);
	let addressB = b2StateAddress(context.states, joint.indexB);
	let stateA = load<b2BodyState>(addressA);
	let stateB = load<b2BodyState>(addressB);

	let vA = stateA.linearVelocity;
	let vB = stateB.linearVelocity;

	// linear velocity
	if (joint.maxVelocityForce.x > 0.0 || joint.maxVelocityForce.y > 0.0) {
		let cdot = b2Sub(vB, vA);
		cdot = b2Sub(cdot, joint.linearVelocity);
		let b = b2MulSV(joint.linearMass, cdot);
		let impulse = b2Vec2(-b.x, -b.y);

		let oldImpulse = joint.linearVelocityImpulse;
		joint.linearVelocityImpulse = b2Add(joint.linearVelocityImpulse, impulse);
		let maxImpulse = b2MulSV(context.h, joint.maxVelocityForce);

		// clamp x and y separately
		joint.linearVelocityImpulse = b2Clamp(joint.linearVelocityImpulse, b2Neg(maxImpulse), maxImpulse);

		impulse = b2Sub(joint.linearVelocityImpulse, oldImpulse);
		vA = b2MulSub(vA, mA, impulse);
		vB = b2MulAdd(vB, mB, impulse);
	} else {
		joint.linearVelocityImpulse = b2Vec2_zero;
	}

	if (stateA.flags & b2_dynamicFlag) {
		b2WriteStateLinearVelocity(addressA, vA);
	}

	if (stateB.flags & b2_dynamicFlag) {
		b2WriteStateLinearVelocity(addressB, vB);
	}
}
