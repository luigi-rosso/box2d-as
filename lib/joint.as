// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/joint.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX, B2_SECRET_COOKIE, FLT_MAX, b2Log } from "./core";
import { B2_GRAPH_COLOR_COUNT, B2_LINEAR_SLOP } from "./constants";
import {
	B2_PI,
	b2AbsFloat,
	b2Add,
	b2ClampFloat,
	b2Dot,
	b2IsValidFloat,
	b2IsValidTransform,
	b2IsValidVec2,
	b2LeftPerp,
	b2Length,
	b2Mat22,
	b2MaxFloat,
	b2MaxInt,
	b2MinFloat,
	b2MinInt,
	b2RelativeAngle,
	b2RotateVector,
	b2Sub,
	b2ToRelativeTransform,
	b2Transform,
	b2TransformPoint,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2BodyId, b2JointId, b2WorldId } from "./id";
import {
	b2BodyType,
	b2DistanceJointDef,
	b2FilterJointDef,
	b2JointDef,
	b2JointType,
	b2MotorJointDef,
	b2MoverJointDef,
	b2PogoJointDef,
	b2PrismaticJointDef,
	b2RevoluteJointDef,
	b2WeldJointDef,
	b2WheelJointDef,
	b2_distanceJoint,
	b2_dynamicBody,
	b2_filterJoint,
	b2_motorJoint,
	b2_moverJoint,
	b2_pogoJoint,
	b2_prismaticJoint,
	b2_revoluteJoint,
	b2_weldJoint,
	b2_wheelJoint,
} from "./types";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2Array_AddIndex, b2Array_RemoveSwap } from "./container";
import { b2BitSet, b2GetBit, b2SetBit } from "./bitset";
import { b2BroadPhase_MarkProxyMovedSerial } from "./broad_phase_core";
import { b2ArrayData, b2BodyState, b2GetBodyFullId, b2GetBodyTransform, b2MakeBodyId, b2WakeBody } from "./body";
import { b2DestroyContact } from "./contact";
import { b2LinkJoint, b2UnlinkJoint } from "./island";
import { B2_OVERFLOW_INDEX, b2CreateJointInGraph, b2RemoveJointFromGraph } from "./constraint_graph";
import {
	b2MergeSolverSets,
	b2WakeSolverSet,
	b2_awakeSet,
	b2_disabledSet,
	b2_firstSleepingSet,
	b2_staticSet,
} from "./solver_set";
import { b2MakeSoft, b2Softness, b2SolverBlock, b2StepContext } from "./solver";
import { b2GetWorld, b2GetWorldFromId, b2GetWorldLocked, b2World } from "./physics_world";
import {
	b2GetDistanceJointForce,
	b2PrepareDistanceJoint,
	b2SolveDistanceJoint,
	b2WarmStartDistanceJoint,
} from "./distance_joint";
import {
	b2GetMotorJointForce,
	b2GetMotorJointTorque,
	b2PrepareMotorJoint,
	b2SolveMotorJoint,
	b2WarmStartMotorJoint,
} from "./motor_joint";
import { b2GetMoverJointForce, b2PrepareMoverJoint, b2SolveMoverJoint, b2WarmStartMoverJoint } from "./mover_joint";
import { b2GetPogoJointForce, b2PreparePogoJoint, b2SolvePogoJoint, b2WarmStartPogoJoint } from "./pogo_joint";
import {
	b2GetPrismaticJointForce,
	b2GetPrismaticJointTorque,
	b2PreparePrismaticJoint,
	b2SolvePrismaticJoint,
	b2WarmStartPrismaticJoint,
} from "./prismatic_joint";
import {
	b2GetRevoluteJointForce,
	b2GetRevoluteJointTorque,
	b2PrepareRevoluteJoint,
	b2SolveRevoluteJoint,
	b2WarmStartRevoluteJoint,
} from "./revolute_joint";
import {
	b2GetWeldJointForce,
	b2GetWeldJointTorque,
	b2PrepareWeldJoint,
	b2SolveWeldJoint,
	b2WarmStartWeldJoint,
} from "./weld_joint";
import {
	b2GetWheelJointForce,
	b2GetWheelJointTorque,
	b2PrepareWheelJoint,
	b2SolveWheelJoint,
	b2WarmStartWheelJoint,
} from "./wheel_joint";

export {
	b2DefaultDistanceJointDef,
	b2DefaultExplosionDef,
	b2DefaultFilterJointDef,
	b2DefaultMotorJointDef,
	b2DefaultMoverJointDef,
	b2DefaultPogoJointDef,
	b2DefaultPrismaticJointDef,
	b2DefaultRevoluteJointDef,
	b2DefaultWeldJointDef,
	b2DefaultWheelJointDef,
} from "./types";

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export struct b2JointEdge {
	bodyId: i32 = 0;
	prevKey: i32 = 0;
	nextKey: i32 = 0;
}

// Map from b2JointId to b2Joint in the solver sets
export struct b2Joint {
	userData: u64 = 0;

	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	setIndex: i32 = 0;

	// index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
	// B2_NULL_INDEX when slot is free
	colorIndex: i32 = 0;

	// joint index within set or graph color
	// B2_NULL_INDEX when slot is free
	localIndex: i32 = 0;

	edges: FixedArray<b2JointEdge, 2>;

	jointId: i32 = 0;
	islandId: i32 = 0;

	// Index into the island's joints array for O(1) swap-removal.
	// B2_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	drawScale: f32 = 0;

	type: b2JointType = 0;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2JointId
	generation: u16 = 0;

	collideConnected: bool = false;
}

// The pogo joint is a hybrid between a contact and a joint
export struct b2PogoJoint {
	// Settings
	normal: b2Vec2 = b2Vec2();
	restLength: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	maxTensionForce: f32 = 0;
	maxCompressionForce: f32 = 0;

	// Runtime
	impulse: f32 = 0;
	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	linearMass: f32 = 0;
	velocity: f32 = 0;
}

export struct b2DistanceJoint {
	length: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	lowerSpringForce: f32 = 0;
	upperSpringForce: f32 = 0;
	minLength: f32 = 0;
	maxLength: f32 = 0;

	maxMotorForce: f32 = 0;
	motorSpeed: f32 = 0;

	impulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	motorImpulse: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	anchorA: b2Vec2 = b2Vec2();
	anchorB: b2Vec2 = b2Vec2();
	deltaCenter: b2Vec2 = b2Vec2();
	distanceSoftness: b2Softness = b2Softness();
	axialMass: f32 = 0;

	enableSpring: bool = false;
	enableLimit: bool = false;
	enableMotor: bool = false;
}

export struct b2MotorJoint {
	linearVelocity: b2Vec2 = b2Vec2();
	maxVelocityForce: f32 = 0;
	angularVelocity: f32 = 0;
	maxVelocityTorque: f32 = 0;
	linearHertz: f32 = 0;
	linearDampingRatio: f32 = 0;
	maxSpringForce: f32 = 0;
	angularHertz: f32 = 0;
	angularDampingRatio: f32 = 0;
	maxSpringTorque: f32 = 0;

	linearVelocityImpulse: b2Vec2 = b2Vec2();
	angularVelocityImpulse: f32 = 0;
	linearSpringImpulse: b2Vec2 = b2Vec2();
	angularSpringImpulse: f32 = 0;

	linearSpring: b2Softness = b2Softness();
	angularSpring: b2Softness = b2Softness();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	linearMass: b2Mat22 = b2Mat22();
	angularMass: f32 = 0;
}

export struct b2MoverJoint {
	linearVelocity: b2Vec2 = b2Vec2();
	maxVelocityForce: b2Vec2 = b2Vec2();

	linearVelocityImpulse: b2Vec2 = b2Vec2();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	linearMass: f32 = 0;
}

export struct b2PrismaticJoint {
	impulse: b2Vec2 = b2Vec2();
	springImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	targetTranslation: f32 = 0;
	maxMotorForce: f32 = 0;
	motorSpeed: f32 = 0;
	lowerTranslation: f32 = 0;
	upperTranslation: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	springSoftness: b2Softness = b2Softness();

	enableSpring: bool = false;
	enableLimit: bool = false;
	enableMotor: bool = false;
}

export struct b2RevoluteJoint {
	linearImpulse: b2Vec2 = b2Vec2();
	springImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	targetAngle: f32 = 0;
	maxMotorTorque: f32 = 0;
	motorSpeed: f32 = 0;
	lowerAngle: f32 = 0;
	upperAngle: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	axialMass: f32 = 0;
	springSoftness: b2Softness = b2Softness();

	enableSpring: bool = false;
	enableMotor: bool = false;
	enableLimit: bool = false;
}

export struct b2WeldJoint {
	linearHertz: f32 = 0;
	linearDampingRatio: f32 = 0;
	angularHertz: f32 = 0;
	angularDampingRatio: f32 = 0;

	linearSpring: b2Softness = b2Softness();
	angularSpring: b2Softness = b2Softness();
	linearImpulse: b2Vec2 = b2Vec2();
	angularImpulse: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	axialMass: f32 = 0;
}

export struct b2WheelJoint {
	perpImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	springImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	maxMotorTorque: f32 = 0;
	motorSpeed: f32 = 0;
	lowerTranslation: f32 = 0;
	upperTranslation: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	perpMass: f32 = 0;
	motorMass: f32 = 0;
	axialMass: f32 = 0;
	springSoftness: b2Softness = b2Softness();

	enableSpring: bool = false;
	enableMotor: bool = false;
	enableLimit: bool = false;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export struct b2JointSim {
	// C's union: each joint type overlays these words through b2<Type>JointAt. First, so the word before a
	// sim and before its payload is the same: the previous sim's viewGuard, or the array buffer's byte size.
	payload: b2JointPayload = b2JointPayload();

	jointId: i32 = 0;

	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;

	type: b2JointType = 0;

	localFrameA: b2Transform = b2Transform();
	localFrameB: b2Transform = b2Transform();

	invMassA: f32 = 0;
	invMassB: f32 = 0;
	invIA: f32 = 0;
	invIB: f32 = 0;

	constraintHertz: f32 = 0;
	constraintDampingRatio: f32 = 0;

	constraintSoftness: b2Softness = b2Softness();

	forceThreshold: f32 = 0;
	torqueThreshold: f32 = 0;

	// rasc range checks a ref into a view even under unchecked, against the word before the element read as a
	// byte length; this keeps that word large for the next sim in the array. b2CreateJoint sets it, copies keep it.
	viewGuard: u32 = B2_JOINT_VIEW_GUARD;
}

export const B2_JOINT_VIEW_GUARD: u32 = 0x7fffffff;

// As large as the largest joint type (b2MotorJoint, 160 bytes); copied as words, so every type's bits survive
export struct b2JointPayload {
	words: FixedArray<u32, 40>;
}

// C points at a joint sim and its union member; these view memory at a sim's address so they bind by ref
// in place, valid until the sim's array grows
@inline export function b2JointSimAddress(sims: StaticArray<b2JointSim>, index: i32): usize {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= index && index < sims.length);
	return changetype<usize>(sims) + <usize>index * sizeof<b2JointSim>();
}

@inline export function b2JointSimAt(sim: usize): StaticArray<b2JointSim> {
	return changetype<StaticArray<b2JointSim>>(sim);
}

@inline export function b2DistanceJointAt(sim: usize): StaticArray<b2DistanceJoint> {
	return changetype<StaticArray<b2DistanceJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2MotorJointAt(sim: usize): StaticArray<b2MotorJoint> {
	return changetype<StaticArray<b2MotorJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2MoverJointAt(sim: usize): StaticArray<b2MoverJoint> {
	return changetype<StaticArray<b2MoverJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2PogoJointAt(sim: usize): StaticArray<b2PogoJoint> {
	return changetype<StaticArray<b2PogoJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2RevoluteJointAt(sim: usize): StaticArray<b2RevoluteJoint> {
	return changetype<StaticArray<b2RevoluteJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2PrismaticJointAt(sim: usize): StaticArray<b2PrismaticJoint> {
	return changetype<StaticArray<b2PrismaticJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2WeldJointAt(sim: usize): StaticArray<b2WeldJoint> {
	return changetype<StaticArray<b2WeldJoint>>(sim + offsetof<b2JointSim>("payload"));
}

@inline export function b2WheelJointAt(sim: usize): StaticArray<b2WheelJoint> {
	return changetype<StaticArray<b2WheelJoint>>(sim + offsetof<b2JointSim>("payload"));
}


// C points into a growable Array of joint sims; this views the Array's buffer as a StaticArray so an element binds
// by ref in place. An Array's data is its buffer object, so the view's length is the capacity, never past it.
@inline export function b2JointSimView(sims: Array<b2JointSim>): StaticArray<b2JointSim> {
	return changetype<StaticArray<b2JointSim>>(b2ArrayData(sims));
}

// C's writes through a body state pointer from b2StateAddress
@inline export function b2WriteStateVelocity(state: usize, v: b2Vec2, w: f32): void {
	store<b2Vec2>(state + offsetof<b2BodyState>("linearVelocity"), v);
	store<f32>(state + offsetof<b2BodyState>("angularVelocity"), w);
}

@inline export function b2WriteStateLinearVelocity(state: usize, v: b2Vec2): void {
	store<b2Vec2>(state + offsetof<b2BodyState>("linearVelocity"), v);
}

// B2_CHECK_INPUT: false when the input is invalid, and the caller returns
export function b2CheckJointInput(condition: bool, text: string, functionName: string): bool {
	if (condition == false) {
		b2Log("invalid input: " + text + " in " + functionName + "\n");
		B2_ASSERT(false);
		return false;
	}

	return true;
}

export function b2GetJointFullId(world: b2World, jointId: b2JointId): i32 {
	let id = jointId.index1 - 1;
	B2_ASSERT(world.joints[id].jointId == id && world.joints[id].generation == jointId.generation);
	return id;
}

// The array holding the joint sim, the color's in the awake set, else the set's
export function b2GetJointSimArray(world: b2World, jointId: i32): Array<b2JointSim> {
	let setIndex = world.joints[jointId].setIndex;
	if (setIndex == b2_awakeSet) {
		let colorIndex = world.joints[jointId].colorIndex;
		B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
		return world.constraintGraph.colors[colorIndex].jointSims;
	}

	return world.solverSets[setIndex].jointSims;
}

// A copy; write through b2GetJointSimView or b2GetJointSimArray
export function b2GetJointSim(world: b2World, jointId: i32): b2JointSim {
	return b2GetJointSimArray(world, jointId)[world.joints[jointId].localIndex];
}

// Bind the sim in place with ref sim = view[world.joints[jointId].localIndex]
export function b2GetJointSimView(world: b2World, jointId: i32): StaticArray<b2JointSim> {
	return b2JointSimView(b2GetJointSimArray(world, jointId));
}

// Returns the joint index, B2_NULL_INDEX where C returns NULL
export function b2GetJointSimCheckType(jointId: b2JointId, type: b2JointType): i32 {
	let world = b2GetWorld(jointId.world0);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return B2_NULL_INDEX;
	}

	let joint = b2GetJointFullId(world, jointId);
	B2_ASSERT(world.joints[joint].type == type);
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2GetJointSimArray(world, joint)[world.joints[joint].localIndex].type == type);
	return joint;
}

// b2GetJointSimCheckType for the joint files: the view holding the sim, bound with
// ref base = view[b2GetJointLocalIndex(jointId)]
export function b2GetJointSimViewCheckType(jointId: b2JointId, type: b2JointType): StaticArray<b2JointSim> {
	let joint = b2GetJointSimCheckType(jointId, type);
	return b2GetJointSimView(b2GetWorld(jointId.world0), joint);
}

// C's b2GetJointSimCheckType pointer, as an address for b2JointSimAt and the b2<Type>JointAt views
export function b2GetJointSimAddressCheckType(jointId: b2JointId, type: b2JointType): usize {
	let joint = b2GetJointSimCheckType(jointId, type);
	let world = b2GetWorld(jointId.world0);
	return b2JointSimAddress(b2GetJointSimView(world, joint), world.joints[joint].localIndex);
}

export function b2GetJointLocalIndex(jointId: b2JointId): i32 {
	return b2GetWorld(jointId.world0).joints[jointId.index1 - 1].localIndex;
}

function b2DestroyContactsBetweenBodies(world: b2World, bodyIdA: i32, bodyIdB: i32): void {
	let contactKey: i32;
	let otherBodyId: i32;

	// use the smaller of the two contact lists
	if (world.bodies[bodyIdA].contactCount < world.bodies[bodyIdB].contactCount) {
		contactKey = world.bodies[bodyIdA].headContactKey;
		otherBodyId = world.bodies[bodyIdB].id;
	} else {
		contactKey = world.bodies[bodyIdB].headContactKey;
		otherBodyId = world.bodies[bodyIdA].id;
	}

	// destroy the contacts
	while (contactKey != B2_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		contactKey = world.contacts[contactId].edges[edgeIndex].nextKey;

		let otherEdgeIndex = edgeIndex ^ 1;
		if (world.contacts[contactId].edges[otherEdgeIndex].bodyId == otherBodyId) {
			// Careful, this removes the contact from the current doubly linked list
			b2DestroyContact(world, contactId);
		}
	}
}

function b2IsValidJointDef(def: b2JointDef): bool {
	return (
		b2IsValidTransform(def.localFrameA) &&
		b2IsValidTransform(def.localFrameB) &&
		b2IsValidFloat(def.forceThreshold) &&
		def.forceThreshold >= 0.0 &&
		b2IsValidFloat(def.torqueThreshold) &&
		def.torqueThreshold >= 0.0 &&
		b2IsValidFloat(def.constraintHertz) &&
		def.constraintHertz >= 0.0 &&
		b2IsValidFloat(def.constraintDampingRatio) &&
		def.constraintDampingRatio >= 0.0 &&
		b2IsValidFloat(def.drawScale)
	);
}

// C returns the joint and sim pointers; the port returns the joint index and callers bind the sim through
// b2GetJointSimView
function b2CreateJoint(world: b2World, def: b2JointDef, type: b2JointType): i32 {
	B2_ASSERT(b2IsValidTransform(def.localFrameA));
	B2_ASSERT(b2IsValidTransform(def.localFrameB));
	B2_ASSERT(world.worldId == def.bodyIdA.world0);
	B2_ASSERT(world.worldId == def.bodyIdB.world0);
	B2_ASSERT((def.bodyIdA == def.bodyIdB) == false);

	let bodyA = b2GetBodyFullId(world, def.bodyIdA);
	let bodyB = b2GetBodyFullId(world, def.bodyIdB);

	// If the joint prevents collisions, then destroy all contacts between attached bodies.
	// Do this first so world arrays are not disturbed in the following code.
	if (def.collideConnected == false) {
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	let bodyIdA = world.bodies[bodyA].id;
	let bodyIdB = world.bodies[bodyB].id;
	let maxSetIndex = b2MaxInt(world.bodies[bodyA].setIndex, world.bodies[bodyB].setIndex);

	// Create joint id and joint
	let jointId = b2AllocId(world.jointIdPool);
	if (jointId == world.joints.length) {
		let empty: b2Joint;
		world.joints.push(empty);
	}

	let joint = world.joints[jointId];
	joint.jointId = jointId;
	joint.userData = def.userData;
	joint.generation += 1;
	joint.setIndex = B2_NULL_INDEX;
	joint.colorIndex = B2_NULL_INDEX;
	joint.localIndex = B2_NULL_INDEX;
	joint.islandId = B2_NULL_INDEX;
	joint.islandIndex = B2_NULL_INDEX;
	joint.drawScale = def.drawScale;
	joint.type = type;
	joint.collideConnected = def.collideConnected;

	// Doubly linked list on bodyA
	joint.edges[0].bodyId = bodyIdA;
	joint.edges[0].prevKey = B2_NULL_INDEX;
	joint.edges[0].nextKey = world.bodies[bodyA].headJointKey;

	// Doubly linked list on bodyB
	joint.edges[1].bodyId = bodyIdB;
	joint.edges[1].prevKey = B2_NULL_INDEX;
	joint.edges[1].nextKey = world.bodies[bodyB].headJointKey;

	// The edges of other joints below never alias this one, so it is stored once here
	world.joints[jointId] = joint;

	let keyA = (jointId << 1) | 0;
	let headA = world.bodies[bodyA].headJointKey;
	if (headA != B2_NULL_INDEX) {
		world.joints[headA >> 1].edges[headA & 1].prevKey = keyA;
	}
	world.bodies[bodyA].headJointKey = keyA;
	world.bodies[bodyA].jointCount += 1;

	let keyB = (jointId << 1) | 1;
	let headB = world.bodies[bodyB].headJointKey;
	if (headB != B2_NULL_INDEX) {
		world.joints[headB >> 1].edges[headB & 1].prevKey = keyB;
	}
	world.bodies[bodyB].headJointKey = keyB;
	world.bodies[bodyB].jointCount += 1;

	let setIndexA = world.bodies[bodyA].setIndex;
	let setIndexB = world.bodies[bodyB].setIndex;

	if (setIndexA == b2_disabledSet || setIndexB == b2_disabledSet) {
		// if either body is disabled, create in disabled set
		let set = world.solverSets[b2_disabledSet];
		world.joints[jointId].setIndex = b2_disabledSet;
		world.joints[jointId].localIndex = set.jointSims.length;

		let localIndex = b2Array_AddIndex(set.jointSims);
		set.jointSims[localIndex].jointId = jointId;
		set.jointSims[localIndex].bodyIdA = bodyIdA;
		set.jointSims[localIndex].bodyIdB = bodyIdB;
		set.jointSims[localIndex].viewGuard = B2_JOINT_VIEW_GUARD;
	} else if (world.bodies[bodyA].type != b2_dynamicBody && world.bodies[bodyB].type != b2_dynamicBody) {
		// joint is not attached to a dynamic body
		let set = world.solverSets[b2_staticSet];
		world.joints[jointId].setIndex = b2_staticSet;
		world.joints[jointId].localIndex = set.jointSims.length;

		let localIndex = b2Array_AddIndex(set.jointSims);
		set.jointSims[localIndex].jointId = jointId;
		set.jointSims[localIndex].bodyIdA = bodyIdA;
		set.jointSims[localIndex].bodyIdB = bodyIdB;
		set.jointSims[localIndex].viewGuard = B2_JOINT_VIEW_GUARD;
	} else if (setIndexA == b2_awakeSet || setIndexB == b2_awakeSet) {
		// if either body is sleeping, wake it
		if (maxSetIndex >= b2_firstSleepingSet) {
			b2WakeSolverSet(world, maxSetIndex);
		}

		world.joints[jointId].setIndex = b2_awakeSet;

		let localIndex = b2CreateJointInGraph(world, jointId);
		let sims = world.constraintGraph.colors[world.joints[jointId].colorIndex].jointSims;
		sims[localIndex].jointId = jointId;
		sims[localIndex].bodyIdA = bodyIdA;
		sims[localIndex].bodyIdB = bodyIdB;
		sims[localIndex].viewGuard = B2_JOINT_VIEW_GUARD;
	} else {
		// joint connected between sleeping and/or static bodies
		B2_ASSERT(setIndexA >= b2_firstSleepingSet || setIndexB >= b2_firstSleepingSet);
		B2_ASSERT(setIndexA != b2_staticSet || setIndexB != b2_staticSet);

		// joint should go into the sleeping set (not static set)
		let setIndex = maxSetIndex;

		let set = world.solverSets[setIndex];
		world.joints[jointId].setIndex = setIndex;
		world.joints[jointId].localIndex = set.jointSims.length;

		// These must be set to accommodate the merge below
		let localIndex = b2Array_AddIndex(set.jointSims);
		set.jointSims[localIndex].jointId = jointId;
		set.jointSims[localIndex].bodyIdA = bodyIdA;
		set.jointSims[localIndex].bodyIdB = bodyIdB;
		set.jointSims[localIndex].viewGuard = B2_JOINT_VIEW_GUARD;

		if (setIndexA != setIndexB && setIndexA >= b2_firstSleepingSet && setIndexB >= b2_firstSleepingSet) {
			// merge sleeping sets
			b2MergeSolverSets(world, setIndexA, setIndexB);
			B2_ASSERT(world.bodies[bodyA].setIndex == world.bodies[bodyB].setIndex);

			// fix potentially invalid set index
			setIndex = world.bodies[bodyA].setIndex;
		}

		B2_ASSERT(world.joints[jointId].setIndex == setIndex);
	}

	// Careful! A set merge moves the sim, so it is bound after placement.
	let view = b2GetJointSimView(world, jointId);
	ref jointSim = view[world.joints[jointId].localIndex];

	jointSim.localFrameA = def.localFrameA;
	jointSim.localFrameB = def.localFrameB;
	jointSim.type = type;
	jointSim.constraintHertz = def.constraintHertz;
	jointSim.constraintDampingRatio = def.constraintDampingRatio;
	jointSim.constraintSoftness.biasRate = 0.0;
	jointSim.constraintSoftness.massScale = 1.0;
	jointSim.constraintSoftness.impulseScale = 0.0;

	B2_ASSERT(b2IsValidFloat(def.forceThreshold) && def.forceThreshold >= 0.0);
	B2_ASSERT(b2IsValidFloat(def.torqueThreshold) && def.torqueThreshold >= 0.0);

	jointSim.forceThreshold = def.forceThreshold;
	jointSim.torqueThreshold = def.torqueThreshold;

	B2_ASSERT(jointSim.jointId == jointId);
	B2_ASSERT(jointSim.bodyIdA == bodyIdA);
	B2_ASSERT(jointSim.bodyIdB == bodyIdB);

	if (world.joints[jointId].setIndex > b2_disabledSet) {
		// Add edge to island graph
		b2LinkJoint(world, jointId);
	}

	return jointId;
}

// C builds the id inline in each create function
function b2MakeJointId(world: b2World, jointId: i32): b2JointId {
	return b2JointId(jointId + 1, world.worldId, world.joints[jointId].generation);
}

export function b2CreateDistanceJoint(worldId: b2WorldId, def: b2DistanceJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreateDistanceJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.length) && def.length > 0.0, "b2IsValidFloat( def->length ) && def->length > 0.0f", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.hertz), "b2IsValidFloat( def->hertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.dampingRatio), "b2IsValidFloat( def->dampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.minLength), "b2IsValidFloat( def->minLength )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxLength), "b2IsValidFloat( def->maxLength )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxMotorForce), "b2IsValidFloat( def->maxMotorForce )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.motorSpeed), "b2IsValidFloat( def->motorSpeed )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.lowerSpringForce), "b2IsValidFloat( def->lowerSpringForce )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.upperSpringForce), "b2IsValidFloat( def->upperSpringForce )", fn)) return b2JointId();
	B2_ASSERT(def.lowerSpringForce <= def.upperSpringForce);

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_distanceJoint);

	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointDistanceView = b2DistanceJointAt(jointAddress);
	ref jointDistance = unchecked(jointDistanceView[0]);

	jointDistance = b2DistanceJoint();
	jointDistance.length = b2MaxFloat(def.length, B2_LINEAR_SLOP());
	jointDistance.hertz = def.hertz;
	jointDistance.dampingRatio = def.dampingRatio;
	jointDistance.minLength = b2MaxFloat(def.minLength, B2_LINEAR_SLOP());
	jointDistance.maxLength = b2MaxFloat(def.minLength, def.maxLength);
	jointDistance.maxMotorForce = def.maxMotorForce;
	jointDistance.motorSpeed = def.motorSpeed;
	jointDistance.enableSpring = def.enableSpring;
	jointDistance.lowerSpringForce = b2MinFloat(def.lowerSpringForce, def.upperSpringForce);
	jointDistance.upperSpringForce = b2MaxFloat(def.lowerSpringForce, def.upperSpringForce);
	jointDistance.enableLimit = def.enableLimit;
	jointDistance.enableMotor = def.enableMotor;
	jointDistance.impulse = 0.0;
	jointDistance.lowerImpulse = 0.0;
	jointDistance.upperImpulse = 0.0;
	jointDistance.motorImpulse = 0.0;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreateFilterJoint(worldId: b2WorldId, def: b2FilterJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", "b2CreateFilterJoint")) {
		return b2JointId();
	}

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_filterJoint);

	return b2MakeJointId(world, id);
}

export function b2CreateMotorJoint(worldId: b2WorldId, def: b2MotorJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreateMotorJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidVec2(def.linearVelocity), "b2IsValidVec2( def->linearVelocity )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxVelocityForce), "b2IsValidFloat( def->maxVelocityForce )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.angularVelocity), "b2IsValidFloat( def->angularVelocity )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxVelocityTorque), "b2IsValidFloat( def->maxVelocityTorque )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.linearHertz), "b2IsValidFloat( def->linearHertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.linearDampingRatio), "b2IsValidFloat( def->linearDampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxSpringForce), "b2IsValidFloat( def->maxSpringForce )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.angularHertz), "b2IsValidFloat( def->angularHertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.angularDampingRatio), "b2IsValidFloat( def->angularDampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxSpringTorque), "b2IsValidFloat( def->maxSpringTorque )", fn)) return b2JointId();

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_motorJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointMotorView = b2MotorJointAt(jointAddress);
	ref jointMotor = unchecked(jointMotorView[0]);

	jointMotor = b2MotorJoint();
	jointMotor.linearVelocity = def.linearVelocity;
	jointMotor.maxVelocityForce = def.maxVelocityForce;
	jointMotor.angularVelocity = def.angularVelocity;
	jointMotor.maxVelocityTorque = def.maxVelocityTorque;
	jointMotor.linearHertz = def.linearHertz;
	jointMotor.linearDampingRatio = def.linearDampingRatio;
	jointMotor.maxSpringForce = def.maxSpringForce;
	jointMotor.angularHertz = def.angularHertz;
	jointMotor.angularDampingRatio = def.angularDampingRatio;
	jointMotor.maxSpringTorque = def.maxSpringTorque;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreateMoverJoint(worldId: b2WorldId, def: b2MoverJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreateMoverJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidVec2(def.linearVelocity), "b2IsValidVec2( def->linearVelocity )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidVec2(def.maxVelocityForce), "b2IsValidVec2( def->maxVelocityForce )", fn)) return b2JointId();

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_moverJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointMoverView = b2MoverJointAt(jointAddress);
	ref jointMover = unchecked(jointMoverView[0]);

	jointMover = b2MoverJoint();
	jointMover.linearVelocity = def.linearVelocity;
	jointMover.maxVelocityForce = def.maxVelocityForce;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreatePogoJoint(worldId: b2WorldId, def: b2PogoJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreatePogoJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidVec2(def.normal), "b2IsValidVec2( def->normal )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.restLength) && def.restLength >= 0.0, "b2IsValidFloat( def->restLength ) && def->restLength >= 0.0f", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.hertz) && def.hertz >= 0.0, "b2IsValidFloat( def->hertz ) && def->hertz >= 0.0f", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.dampingRatio) && def.dampingRatio >= 0.0, "b2IsValidFloat( def->dampingRatio ) && def->dampingRatio >= 0.0f", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxTensionForce) && def.maxTensionForce >= 0.0, "b2IsValidFloat( def->maxTensionForce ) && def->maxTensionForce >= 0.0f", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxCompressionForce) && def.maxCompressionForce >= 0.0, "b2IsValidFloat( def->maxCompressionForce ) && def->maxCompressionForce >= 0.0f", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.impulse), "b2IsValidFloat( def->impulse )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.velocity), "b2IsValidFloat( def->velocity )", fn)) return b2JointId();

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_pogoJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointPogoView = b2PogoJointAt(jointAddress);
	ref jointPogo = unchecked(jointPogoView[0]);

	jointPogo = b2PogoJoint();
	jointPogo.restLength = def.restLength;
	jointPogo.hertz = def.hertz;
	jointPogo.dampingRatio = def.dampingRatio;
	jointPogo.maxTensionForce = def.maxTensionForce;
	jointPogo.maxCompressionForce = def.maxCompressionForce;
	jointPogo.normal = def.normal;
	jointPogo.impulse = def.impulse;
	jointPogo.velocity = def.velocity;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreatePrismaticJoint(worldId: b2WorldId, def: b2PrismaticJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreatePrismaticJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.hertz), "b2IsValidFloat( def->hertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.dampingRatio), "b2IsValidFloat( def->dampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.targetTranslation), "b2IsValidFloat( def->targetTranslation )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.lowerTranslation), "b2IsValidFloat( def->lowerTranslation )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.upperTranslation), "b2IsValidFloat( def->upperTranslation )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxMotorForce), "b2IsValidFloat( def->maxMotorForce )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.motorSpeed), "b2IsValidFloat( def->motorSpeed )", fn)) return b2JointId();
	B2_ASSERT(def.lowerTranslation <= def.upperTranslation);

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_prismaticJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointPrismaticView = b2PrismaticJointAt(jointAddress);
	ref jointPrismatic = unchecked(jointPrismaticView[0]);

	jointPrismatic = b2PrismaticJoint();
	jointPrismatic.hertz = def.hertz;
	jointPrismatic.dampingRatio = def.dampingRatio;
	jointPrismatic.targetTranslation = def.targetTranslation;
	jointPrismatic.lowerTranslation = b2MinFloat(def.lowerTranslation, def.upperTranslation);
	jointPrismatic.upperTranslation = b2MaxFloat(def.lowerTranslation, def.upperTranslation);
	jointPrismatic.maxMotorForce = def.maxMotorForce;
	jointPrismatic.motorSpeed = def.motorSpeed;
	jointPrismatic.enableSpring = def.enableSpring;
	jointPrismatic.enableLimit = def.enableLimit;
	jointPrismatic.enableMotor = def.enableMotor;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreateRevoluteJoint(worldId: b2WorldId, def: b2RevoluteJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreateRevoluteJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.targetAngle), "b2IsValidFloat( def->targetAngle )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.hertz), "b2IsValidFloat( def->hertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.dampingRatio), "b2IsValidFloat( def->dampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.lowerAngle), "b2IsValidFloat( def->lowerAngle )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.upperAngle), "b2IsValidFloat( def->upperAngle )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxMotorTorque), "b2IsValidFloat( def->maxMotorTorque )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.motorSpeed), "b2IsValidFloat( def->motorSpeed )", fn)) return b2JointId();
	B2_ASSERT(def.lowerAngle <= def.upperAngle);

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_revoluteJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointRevoluteView = b2RevoluteJointAt(jointAddress);
	ref jointRevolute = unchecked(jointRevoluteView[0]);

	jointRevolute = b2RevoluteJoint();

	jointRevolute.targetAngle = b2ClampFloat(def.targetAngle, -B2_PI, B2_PI);
	jointRevolute.hertz = def.hertz;
	jointRevolute.dampingRatio = def.dampingRatio;

	let lowerAngle = b2MinFloat(def.lowerAngle, def.upperAngle);
	let upperAngle = b2MaxFloat(def.lowerAngle, def.upperAngle);
	jointRevolute.lowerAngle = b2ClampFloat(lowerAngle, -0.99 * B2_PI, 0.99 * B2_PI);
	jointRevolute.upperAngle = b2ClampFloat(upperAngle, -0.99 * B2_PI, 0.99 * B2_PI);

	jointRevolute.maxMotorTorque = def.maxMotorTorque;
	jointRevolute.motorSpeed = def.motorSpeed;
	jointRevolute.enableSpring = def.enableSpring;
	jointRevolute.enableLimit = def.enableLimit;
	jointRevolute.enableMotor = def.enableMotor;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreateWeldJoint(worldId: b2WorldId, def: b2WeldJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreateWeldJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.linearHertz), "b2IsValidFloat( def->linearHertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.angularHertz), "b2IsValidFloat( def->angularHertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.linearDampingRatio), "b2IsValidFloat( def->linearDampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.angularDampingRatio), "b2IsValidFloat( def->angularDampingRatio )", fn)) return b2JointId();

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_weldJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointWeldView = b2WeldJointAt(jointAddress);
	ref jointWeld = unchecked(jointWeldView[0]);

	jointWeld = b2WeldJoint();
	jointWeld.linearHertz = def.linearHertz;
	jointWeld.linearDampingRatio = def.linearDampingRatio;
	jointWeld.angularHertz = def.angularHertz;
	jointWeld.angularDampingRatio = def.angularDampingRatio;
	jointWeld.linearImpulse = b2Vec2_zero;
	jointWeld.angularImpulse = 0.0;

	return b2MakeJointId(world, joint.jointId);
}

export function b2CreateWheelJoint(worldId: b2WorldId, def: b2WheelJointDef): b2JointId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	const fn = "b2CreateWheelJoint";
	if (!b2CheckJointInput(b2IsValidJointDef(def.base), "b2IsValidJointDef( &def->base )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.hertz), "b2IsValidFloat( def->hertz )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.dampingRatio), "b2IsValidFloat( def->dampingRatio )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.lowerTranslation), "b2IsValidFloat( def->lowerTranslation )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.upperTranslation), "b2IsValidFloat( def->upperTranslation )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.maxMotorTorque), "b2IsValidFloat( def->maxMotorTorque )", fn)) return b2JointId();
	if (!b2CheckJointInput(b2IsValidFloat(def.motorSpeed), "b2IsValidFloat( def->motorSpeed )", fn)) return b2JointId();
	B2_ASSERT(def.lowerTranslation <= def.upperTranslation);

	let world = b2GetWorldFromId(worldId);

	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2JointId();
	}

	let id = b2CreateJoint(world, def.base, b2_wheelJoint);
	let view = b2GetJointSimView(world, id);
	let jointAddress = b2JointSimAddress(view, world.joints[id].localIndex);
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	let jointWheelView = b2WheelJointAt(jointAddress);
	ref jointWheel = unchecked(jointWheelView[0]);

	jointWheel = b2WheelJoint();
	jointWheel.perpMass = 0.0;
	jointWheel.axialMass = 0.0;
	jointWheel.motorImpulse = 0.0;
	jointWheel.lowerImpulse = 0.0;
	jointWheel.upperImpulse = 0.0;
	jointWheel.lowerTranslation = b2MinFloat(def.lowerTranslation, def.upperTranslation);
	jointWheel.upperTranslation = b2MaxFloat(def.lowerTranslation, def.upperTranslation);
	jointWheel.maxMotorTorque = def.maxMotorTorque;
	jointWheel.motorSpeed = def.motorSpeed;
	jointWheel.hertz = def.hertz;
	jointWheel.dampingRatio = def.dampingRatio;
	jointWheel.enableSpring = def.enableSpring;
	jointWheel.enableLimit = def.enableLimit;
	jointWheel.enableMotor = def.enableMotor;

	return b2MakeJointId(world, joint.jointId);
}

export function b2DestroyJointInternal(world: b2World, jointId: i32): void {
	// Copies: the edges of other joints relinked below never alias this joint's edges
	let edgeA = world.joints[jointId].edges[0];
	let edgeB = world.joints[jointId].edges[1];

	let idA = edgeA.bodyId;
	let idB = edgeB.bodyId;

	// Remove from body A
	if (edgeA.prevKey != B2_NULL_INDEX) {
		world.joints[edgeA.prevKey >> 1].edges[edgeA.prevKey & 1].nextKey = edgeA.nextKey;
	}

	if (edgeA.nextKey != B2_NULL_INDEX) {
		world.joints[edgeA.nextKey >> 1].edges[edgeA.nextKey & 1].prevKey = edgeA.prevKey;
	}

	let edgeKeyA = (jointId << 1) | 0;
	if (world.bodies[idA].headJointKey == edgeKeyA) {
		world.bodies[idA].headJointKey = edgeA.nextKey;
	}

	world.bodies[idA].jointCount -= 1;

	// Remove from body B
	if (edgeB.prevKey != B2_NULL_INDEX) {
		world.joints[edgeB.prevKey >> 1].edges[edgeB.prevKey & 1].nextKey = edgeB.nextKey;
	}

	if (edgeB.nextKey != B2_NULL_INDEX) {
		world.joints[edgeB.nextKey >> 1].edges[edgeB.nextKey & 1].prevKey = edgeB.prevKey;
	}

	let edgeKeyB = (jointId << 1) | 1;
	if (world.bodies[idB].headJointKey == edgeKeyB) {
		world.bodies[idB].headJointKey = edgeB.nextKey;
	}

	world.bodies[idB].jointCount -= 1;

	if (world.joints[jointId].islandId != B2_NULL_INDEX) {
		B2_ASSERT(world.joints[jointId].setIndex > b2_disabledSet);
		b2UnlinkJoint(world, jointId);
	} else {
		B2_ASSERT(world.joints[jointId].setIndex <= b2_disabledSet);
	}

	// Remove joint from solver set that owns it
	let setIndex = world.joints[jointId].setIndex;
	let localIndex = world.joints[jointId].localIndex;

	if (setIndex == b2_awakeSet) {
		b2RemoveJointFromGraph(world, idA, idB, world.joints[jointId].colorIndex, localIndex);
	} else {
		let set = world.solverSets[setIndex];
		let movedIndex = b2Array_RemoveSwap(set.jointSims, localIndex);
		if (movedIndex != B2_NULL_INDEX) {
			// Fix moved joint
			let movedId = set.jointSims[localIndex].jointId;
			B2_ASSERT(world.joints[movedId].localIndex == movedIndex);
			world.joints[movedId].localIndex = localIndex;
		}
	}

	// Free joint and id (preserve joint generation)
	world.joints[jointId].setIndex = B2_NULL_INDEX;
	world.joints[jointId].localIndex = B2_NULL_INDEX;
	world.joints[jointId].colorIndex = B2_NULL_INDEX;
	world.joints[jointId].jointId = B2_NULL_INDEX;
	b2FreeId(world.jointIdPool, jointId);

	b2WakeBody(world, idA);
	b2WakeBody(world, idB);
}

export function b2DestroyJoint(jointId: b2JointId): void {
	let world = b2GetWorld(jointId.world0);
	B2_ASSERT(world.locked == false);

	if (world.locked) {
		return;
	}

	let joint = b2GetJointFullId(world, jointId);

	b2DestroyJointInternal(world, joint);
}

export function b2Joint_GetType(jointId: b2JointId): b2JointType {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return world.joints[joint].type;
}

export function b2Joint_GetBodyA(jointId: b2JointId): b2BodyId {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return b2MakeBodyId(world, world.joints[joint].edges[0].bodyId);
}

export function b2Joint_GetBodyB(jointId: b2JointId): b2BodyId {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return b2MakeBodyId(world, world.joints[joint].edges[1].bodyId);
}

export function b2Joint_GetWorld(jointId: b2JointId): b2WorldId {
	let world = b2GetWorld(jointId.world0);
	return b2WorldId(jointId.world0 + 1, world.generation);
}

export function b2Joint_SetLocalFrameA(jointId: b2JointId, localFrame: b2Transform): void {
	if (!b2CheckJointInput(b2IsValidTransform(localFrame), "b2IsValidTransform( localFrame )", "b2Joint_SetLocalFrameA")) {
		return;
	}

	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	view[world.joints[joint].localIndex].localFrameA = localFrame;
}

export function b2Joint_GetLocalFrameA(jointId: b2JointId): b2Transform {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	return view[world.joints[joint].localIndex].localFrameA;
}

export function b2Joint_SetLocalFrameB(jointId: b2JointId, localFrame: b2Transform): void {
	if (!b2CheckJointInput(b2IsValidTransform(localFrame), "b2IsValidTransform( localFrame )", "b2Joint_SetLocalFrameB")) {
		return;
	}

	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	view[world.joints[joint].localIndex].localFrameB = localFrame;
}

export function b2Joint_GetLocalFrameB(jointId: b2JointId): b2Transform {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	return view[world.joints[joint].localIndex].localFrameB;
}

export function b2Joint_SetCollideConnected(jointId: b2JointId, shouldCollide: bool): void {
	let lockedWorld = b2GetWorldLocked(jointId.world0);
	if (lockedWorld == null) {
		return;
	}
	let world = lockedWorld!;

	let joint = b2GetJointFullId(world, jointId);
	if (world.joints[joint].collideConnected == shouldCollide) {
		return;
	}

	world.joints[joint].collideConnected = shouldCollide;

	let bodyA = world.joints[joint].edges[0].bodyId;
	let bodyB = world.joints[joint].edges[1].bodyId;

	if (shouldCollide) {
		let typeA: b2BodyType = world.bodies[bodyA].type;
		let typeB: b2BodyType = world.bodies[bodyB].type;

		let shapeId = B2_NULL_INDEX;
		if (typeA == b2_dynamicBody && typeB == b2_dynamicBody) {
			shapeId =
				world.bodies[bodyA].shapeCount < world.bodies[bodyB].shapeCount
					? world.bodies[bodyA].headShapeId
					: world.bodies[bodyB].headShapeId;
		} else if (typeA == b2_dynamicBody) {
			shapeId = world.bodies[bodyA].headShapeId;
		} else if (typeB == b2_dynamicBody) {
			shapeId = world.bodies[bodyB].headShapeId;
		}

		while (shapeId != B2_NULL_INDEX) {
			let proxyKey = world.shapes[shapeId].proxyKey;
			if (proxyKey != B2_NULL_INDEX) {
				b2BroadPhase_MarkProxyMovedSerial(world.broadPhase, proxyKey);
			}

			shapeId = world.shapes[shapeId].nextShapeId;
		}
	} else {
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}
}

export function b2Joint_GetCollideConnected(jointId: b2JointId): bool {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return world.joints[joint].collideConnected;
}

export function b2Joint_SetUserData(jointId: b2JointId, userData: u64): void {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	world.joints[joint].userData = userData;
}

export function b2Joint_GetUserData(jointId: b2JointId): u64 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return world.joints[joint].userData;
}

export function b2Joint_WakeBodies(jointId: b2JointId): void {
	let lockedWorld = b2GetWorldLocked(jointId.world0);
	if (lockedWorld == null) {
		return;
	}
	let world = lockedWorld!;

	let joint = b2GetJointFullId(world, jointId);
	let bodyA = world.joints[joint].edges[0].bodyId;
	let bodyB = world.joints[joint].edges[1].bodyId;

	b2WakeBody(world, bodyA);
	b2WakeBody(world, bodyB);
}

export function b2GetJointReaction(simAddress: usize, invTimeStep: f32, force: ref f32, torque: ref f32): void {
	let simView = b2JointSimAt(simAddress);
	ref sim = unchecked(simView[0]);
	let simMoverView = b2MoverJointAt(simAddress);
	ref simMover = unchecked(simMoverView[0]);
	let simPogoView = b2PogoJointAt(simAddress);
	ref simPogo = unchecked(simPogoView[0]);
	let linearImpulse: f32 = 0.0;
	let angularImpulse: f32 = 0.0;

	switch (sim.type) {
		case b2_distanceJoint: {
			let jointView = b2DistanceJointAt(simAddress);
			ref joint = unchecked(jointView[0]);
			linearImpulse = b2AbsFloat(joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse);
			break;
		}

		case b2_motorJoint: {
			let jointView = b2MotorJointAt(simAddress);
			ref joint = unchecked(jointView[0]);
			linearImpulse = b2Length(b2Add(joint.linearVelocityImpulse, joint.linearSpringImpulse));
			angularImpulse = b2AbsFloat(joint.angularVelocityImpulse + joint.angularSpringImpulse);
			break;
		}

		case b2_moverJoint: {
			linearImpulse = b2Length(simMover.linearVelocityImpulse);
			break;
		}

		case b2_pogoJoint: {
			linearImpulse = b2AbsFloat(simPogo.impulse);
			break;
		}

		case b2_prismaticJoint: {
			let jointView = b2PrismaticJointAt(simAddress);
			ref joint = unchecked(jointView[0]);
			let perpImpulse: f32 = joint.impulse.x;
			let axialImpulse: f32 = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
			linearImpulse = Mathf.sqrt(perpImpulse * perpImpulse + axialImpulse * axialImpulse);
			angularImpulse = b2AbsFloat(joint.impulse.y);
			break;
		}

		case b2_revoluteJoint: {
			let jointView = b2RevoluteJointAt(simAddress);
			ref joint = unchecked(jointView[0]);

			linearImpulse = b2Length(joint.linearImpulse);
			angularImpulse = b2AbsFloat(joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse);
			break;
		}

		case b2_weldJoint: {
			let jointView = b2WeldJointAt(simAddress);
			ref joint = unchecked(jointView[0]);
			linearImpulse = b2Length(joint.linearImpulse);
			angularImpulse = b2AbsFloat(joint.angularImpulse);
			break;
		}

		case b2_wheelJoint: {
			let jointView = b2WheelJointAt(simAddress);
			ref joint = unchecked(jointView[0]);
			let perpImpulse: f32 = joint.perpImpulse;
			let axialImpulse: f32 = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;
			linearImpulse = Mathf.sqrt(perpImpulse * perpImpulse + axialImpulse * axialImpulse);
			angularImpulse = b2AbsFloat(joint.motorImpulse);
			break;
		}

		default:
			break;
	}

	force = linearImpulse * invTimeStep;
	torque = angularImpulse * invTimeStep;
}

function b2GetJointConstraintForce(world: b2World, joint: i32): b2Vec2 {
	let view = b2GetJointSimView(world, joint);
	let baseAddress = b2JointSimAddress(view, world.joints[joint].localIndex);
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);

	switch (world.joints[joint].type) {
		case b2_distanceJoint:
			return b2GetDistanceJointForce(world, baseAddress);

		case b2_motorJoint:
			return b2GetMotorJointForce(world, baseAddress);

		case b2_moverJoint:
			return b2GetMoverJointForce(world, baseAddress);

		case b2_filterJoint:
			return b2Vec2_zero;

		case b2_pogoJoint:
			return b2GetPogoJointForce(world, baseAddress);

		case b2_prismaticJoint:
			return b2GetPrismaticJointForce(world, baseAddress);

		case b2_revoluteJoint:
			return b2GetRevoluteJointForce(world, baseAddress);

		case b2_weldJoint:
			return b2GetWeldJointForce(world, baseAddress);

		case b2_wheelJoint:
			return b2GetWheelJointForce(world, baseAddress);

		default:
			B2_ASSERT(false);
			return b2Vec2_zero;
	}
}

function b2GetJointConstraintTorque(world: b2World, joint: i32): f32 {
	let view = b2GetJointSimView(world, joint);
	let baseAddress = b2JointSimAddress(view, world.joints[joint].localIndex);
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);

	switch (world.joints[joint].type) {
		case b2_distanceJoint:
			return 0.0;

		case b2_motorJoint:
			return b2GetMotorJointTorque(world, baseAddress);

		case b2_filterJoint:
			return 0.0;

		case b2_prismaticJoint:
			return b2GetPrismaticJointTorque(world, baseAddress);

		case b2_revoluteJoint:
			return b2GetRevoluteJointTorque(world, baseAddress);

		case b2_weldJoint:
			return b2GetWeldJointTorque(world, baseAddress);

		case b2_wheelJoint:
			return b2GetWheelJointTorque(world, baseAddress);

		default:
			B2_ASSERT(false);
			return 0.0;
	}
}

export function b2Joint_GetConstraintForce(jointId: b2JointId): b2Vec2 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return b2GetJointConstraintForce(world, joint);
}

export function b2Joint_GetConstraintTorque(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	return b2GetJointConstraintTorque(world, joint);
}

export function b2Joint_GetLinearSeparation(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	let baseAddress = b2JointSimAddress(view, world.joints[joint].localIndex);
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseWeldView = b2WeldJointAt(baseAddress);
	ref baseWeld = unchecked(baseWeldView[0]);

	// Relative to body A so the difference stays in float precision far from the origin
	let wxfA = b2GetBodyTransform(world, world.joints[joint].edges[0].bodyId);
	let xfA = b2ToRelativeTransform(wxfA, wxfA.p);
	let xfB = b2ToRelativeTransform(b2GetBodyTransform(world, world.joints[joint].edges[1].bodyId), wxfA.p);

	let pA = b2TransformPoint(xfA, base.localFrameA.p);
	let pB = b2TransformPoint(xfB, base.localFrameB.p);
	let dp = b2Sub(pB, pA);

	switch (world.joints[joint].type) {
		case b2_distanceJoint: {
			let distanceJointView = b2DistanceJointAt(baseAddress);
			ref distanceJoint = unchecked(distanceJointView[0]);
			let length: f32 = b2Length(dp);
			if (distanceJoint.enableSpring) {
				if (distanceJoint.enableLimit) {
					if (length < distanceJoint.minLength) {
						return distanceJoint.minLength - length;
					}

					if (length > distanceJoint.maxLength) {
						return length - distanceJoint.maxLength;
					}

					return 0.0;
				}

				return 0.0;
			}

			return b2AbsFloat(length - distanceJoint.length);
		}

		case b2_motorJoint:
			return 0.0;

		case b2_filterJoint:
			return 0.0;

		case b2_prismaticJoint: {
			let prismaticJointView = b2PrismaticJointAt(baseAddress);
			ref prismaticJoint = unchecked(prismaticJointView[0]);
			let axisA = b2RotateVector(xfA.q, b2Vec2(1.0, 0.0));
			let perpA = b2LeftPerp(axisA);
			let perpendicularSeparation: f32 = b2AbsFloat(b2Dot(perpA, dp));
			let limitSeparation: f32 = 0.0;

			if (prismaticJoint.enableLimit) {
				let translation: f32 = b2Dot(axisA, dp);
				if (translation < prismaticJoint.lowerTranslation) {
					limitSeparation = prismaticJoint.lowerTranslation - translation;
				}

				if (prismaticJoint.upperTranslation < translation) {
					limitSeparation = translation - prismaticJoint.upperTranslation;
				}
			}

			return Mathf.sqrt(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
		}

		case b2_revoluteJoint:
			return b2Length(dp);

		case b2_weldJoint: {
			if (baseWeld.linearHertz == 0.0) {
				return b2Length(dp);
			}

			return 0.0;
		}

		case b2_wheelJoint: {
			let wheelJointView = b2WheelJointAt(baseAddress);
			ref wheelJoint = unchecked(wheelJointView[0]);
			let axisA = b2RotateVector(xfA.q, b2Vec2(1.0, 0.0));
			let perpA = b2LeftPerp(axisA);
			let perpendicularSeparation: f32 = b2AbsFloat(b2Dot(perpA, dp));
			let limitSeparation: f32 = 0.0;

			if (wheelJoint.enableLimit) {
				let translation: f32 = b2Dot(axisA, dp);
				if (translation < wheelJoint.lowerTranslation) {
					limitSeparation = wheelJoint.lowerTranslation - translation;
				}

				if (wheelJoint.upperTranslation < translation) {
					limitSeparation = translation - wheelJoint.upperTranslation;
				}
			}

			return Mathf.sqrt(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
		}

		default:
			B2_ASSERT(false);
			return 0.0;
	}
}

export function b2Joint_GetAngularSeparation(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	let baseAddress = b2JointSimAddress(view, world.joints[joint].localIndex);
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	let baseWeldView = b2WeldJointAt(baseAddress);
	ref baseWeld = unchecked(baseWeldView[0]);

	let qA = b2GetBodyTransform(world, world.joints[joint].edges[0].bodyId).q;
	let qB = b2GetBodyTransform(world, world.joints[joint].edges[1].bodyId).q;
	let relativeAngle: f32 = b2RelativeAngle(qA, qB);

	switch (world.joints[joint].type) {
		case b2_distanceJoint:
			return 0.0;

		case b2_motorJoint:
			return 0.0;

		case b2_filterJoint:
			return 0.0;

		case b2_prismaticJoint: {
			return relativeAngle;
		}

		case b2_revoluteJoint: {
			let revoluteJointView = b2RevoluteJointAt(baseAddress);
			ref revoluteJoint = unchecked(revoluteJointView[0]);
			if (revoluteJoint.enableLimit) {
				let angle: f32 = relativeAngle;
				if (angle < revoluteJoint.lowerAngle) {
					return revoluteJoint.lowerAngle - angle;
				}

				if (revoluteJoint.upperAngle < angle) {
					return angle - revoluteJoint.upperAngle;
				}
			}

			return 0.0;
		}

		case b2_weldJoint: {
			if (baseWeld.angularHertz == 0.0) {
				return relativeAngle;
			}

			return 0.0;
		}

		case b2_wheelJoint:
			return 0.0;

		default:
			B2_ASSERT(false);
			return 0.0;
	}
}

export function b2Joint_SetConstraintTuning(jointId: b2JointId, hertz: f32, dampingRatio: f32): void {
	const fn = "b2Joint_SetConstraintTuning";
	if (!b2CheckJointInput(b2IsValidFloat(hertz) && hertz >= 0.0, "b2IsValidFloat( hertz ) && hertz >= 0.0f", fn)) return;
	if (!b2CheckJointInput(b2IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f", fn)) return;

	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	let baseAddress = b2JointSimAddress(view, world.joints[joint].localIndex);
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	base.constraintHertz = hertz;
	base.constraintDampingRatio = dampingRatio;
}

export function b2Joint_GetConstraintTuning(jointId: b2JointId, hertz: ref f32, dampingRatio: ref f32): void {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	let baseAddress = b2JointSimAddress(view, world.joints[joint].localIndex);
	let baseView = b2JointSimAt(baseAddress);
	ref base = unchecked(baseView[0]);
	hertz = base.constraintHertz;
	dampingRatio = base.constraintDampingRatio;
}

export function b2Joint_SetForceThreshold(jointId: b2JointId, threshold: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(threshold) && threshold >= 0.0, "b2IsValidFloat( threshold ) && threshold >= 0.0f", "b2Joint_SetForceThreshold")) {
		return;
	}

	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	view[world.joints[joint].localIndex].forceThreshold = threshold;
}

export function b2Joint_GetForceThreshold(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	return view[world.joints[joint].localIndex].forceThreshold;
}

export function b2Joint_SetTorqueThreshold(jointId: b2JointId, threshold: f32): void {
	if (!b2CheckJointInput(b2IsValidFloat(threshold) && threshold >= 0.0, "b2IsValidFloat( threshold ) && threshold >= 0.0f", "b2Joint_SetTorqueThreshold")) {
		return;
	}

	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	view[world.joints[joint].localIndex].torqueThreshold = threshold;
}

export function b2Joint_GetTorqueThreshold(jointId: b2JointId): f32 {
	let world = b2GetWorld(jointId.world0);
	let joint = b2GetJointFullId(world, jointId);
	let view = b2GetJointSimView(world, joint);
	return view[world.joints[joint].localIndex].torqueThreshold;
}

export function b2PrepareJoint(jointAddress: usize, context: b2StepContext): void {
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	// Clamp joint hertz based on the time step to reduce jitter.
	let hertz: f32 = b2MinFloat(joint.constraintHertz, 0.25 * context.inv_h);
	joint.constraintSoftness = b2MakeSoft(hertz, joint.constraintDampingRatio, context.h);

	switch (joint.type) {
		case b2_distanceJoint:
			b2PrepareDistanceJoint(jointAddress, context);
			break;

		case b2_motorJoint:
			b2PrepareMotorJoint(jointAddress, context);
			break;

		case b2_moverJoint:
			b2PrepareMoverJoint(jointAddress, context);
			break;

		case b2_filterJoint:
			break;

		case b2_pogoJoint:
			b2PreparePogoJoint(jointAddress, context);
			break;

		case b2_prismaticJoint:
			b2PreparePrismaticJoint(jointAddress, context);
			break;

		case b2_revoluteJoint:
			b2PrepareRevoluteJoint(jointAddress, context);
			break;

		case b2_weldJoint:
			b2PrepareWeldJoint(jointAddress, context);
			break;

		case b2_wheelJoint:
			b2PrepareWheelJoint(jointAddress, context);
			break;

		default:
			B2_ASSERT(false);
	}
}

export function b2WarmStartJoint(jointAddress: usize, context: b2StepContext): void {
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	switch (joint.type) {
		case b2_distanceJoint:
			b2WarmStartDistanceJoint(jointAddress, context);
			break;

		case b2_motorJoint:
			b2WarmStartMotorJoint(jointAddress, context);
			break;

		case b2_moverJoint:
			b2WarmStartMoverJoint(jointAddress, context);
			break;

		case b2_filterJoint:
			break;

		case b2_pogoJoint:
			b2WarmStartPogoJoint(jointAddress, context);
			break;

		case b2_prismaticJoint:
			b2WarmStartPrismaticJoint(jointAddress, context);
			break;

		case b2_revoluteJoint:
			b2WarmStartRevoluteJoint(jointAddress, context);
			break;

		case b2_weldJoint:
			b2WarmStartWeldJoint(jointAddress, context);
			break;

		case b2_wheelJoint:
			b2WarmStartWheelJoint(jointAddress, context);
			break;

		default:
			B2_ASSERT(false);
	}
}

export function b2SolveJoint(jointAddress: usize, context: b2StepContext, useBias: bool): void {
	let jointView = b2JointSimAt(jointAddress);
	ref joint = unchecked(jointView[0]);
	switch (joint.type) {
		case b2_distanceJoint:
			b2SolveDistanceJoint(jointAddress, context, useBias);
			break;

		case b2_motorJoint:
			b2SolveMotorJoint(jointAddress, context);
			break;

		case b2_moverJoint:
			b2SolveMoverJoint(jointAddress, context);
			break;

		case b2_filterJoint:
			break;

		case b2_pogoJoint:
			b2SolvePogoJoint(jointAddress, context, useBias);
			break;

		case b2_prismaticJoint:
			b2SolvePrismaticJoint(jointAddress, context, useBias);
			break;

		case b2_revoluteJoint:
			b2SolveRevoluteJoint(jointAddress, context, useBias);
			break;

		case b2_weldJoint:
			b2SolveWeldJoint(jointAddress, context, useBias);
			break;

		case b2_wheelJoint:
			b2SolveWheelJoint(jointAddress, context, useBias);
			break;

		default:
			B2_ASSERT(false);
	}
}

export function b2PrepareJoints_Overflow(context: b2StepContext): void {
	let graph = context.graph;
	let sims = graph.colors[B2_OVERFLOW_INDEX].jointSims;
	let joints = b2JointSimView(sims);
	let jointCount = sims.length;

	for (let i = 0; i < jointCount; ++i) {
		b2PrepareJoint(b2JointSimAddress(joints, i), context);
	}
}

export function b2WarmStartJoints_Overflow(context: b2StepContext): void {
	let graph = context.graph;
	let sims = graph.colors[B2_OVERFLOW_INDEX].jointSims;
	let joints = b2JointSimView(sims);
	let jointCount = sims.length;

	for (let i = 0; i < jointCount; ++i) {
		b2WarmStartJoint(b2JointSimAddress(joints, i), context);
	}
}

export function b2SolveJoints_Overflow(context: b2StepContext, useBias: bool): void {
	let graph = context.graph;
	let sims = graph.colors[B2_OVERFLOW_INDEX].jointSims;
	let joints = b2JointSimView(sims);
	let jointCount = sims.length;

	for (let i = 0; i < jointCount; ++i) {
		b2SolveJoint(b2JointSimAddress(joints, i), context, useBias);
	}
}

export function b2PrepareJointsTask(block: b2SolverBlock, context: b2StepContext): void {
	let spans = context.jointPrepareSpans;

	let index = block.startIndex;
	let endIndex = block.startIndex + <i32>block.count;

	// Find color for start index. Linear search but fast.
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= index) {
		colorIndex += 1;
	}

	// Loop over block
	while (index < endIndex) {
		let colorStart = spans[colorIndex].start;
		let colorEndIndex = b2MinInt(spans[colorIndex + 1].start, endIndex);
		let joints = b2JointSimView(spans[colorIndex].joints);

		// Loop over color
		for (; index < colorEndIndex; ++index) {
			if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= index - colorStart && index - colorStart < spans[colorIndex].count);
			b2PrepareJoint(b2JointSimAddress(joints, index - colorStart), context);
		}

		// Advance to next color
		colorIndex += 1;
	}
}

export function b2WarmStartJointsTask(block: b2SolverBlock, context: b2StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	let joints = b2JointSimView(color.jointSims);

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		b2WarmStartJoint(b2JointSimAddress(joints, i), context);
	}
}

export function b2SolveJointsTask(block: b2SolverBlock, context: b2StepContext, useBias: bool, workerIndex: i32): void {
	let color = context.graph.colors[block.colorIndex];
	let joints = b2JointSimView(color.jointSims);

	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= block.startIndex && block.startIndex + <i32>block.count <= color.jointSims.length);

	let jointStateBitSet: b2BitSet = context.world.taskContexts[workerIndex].jointStateBitSet;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let jointAddress = b2JointSimAddress(joints, i);
		let jointView = b2JointSimAt(jointAddress);
		ref joint = unchecked(jointView[0]);
		b2SolveJoint(jointAddress, context, useBias);

		if (
			useBias &&
			(joint.forceThreshold < FLT_MAX || joint.torqueThreshold < FLT_MAX) &&
			b2GetBit(jointStateBitSet, <u32>joint.jointId) == false
		) {
			// force in x, torque in y: ref needs a struct leaf
			let reaction = b2Vec2();
			b2GetJointReaction(jointAddress, context.inv_h, reaction.x, reaction.y);

			// Check thresholds. A zero threshold means all awake joints get reported.
			if (reaction.x >= joint.forceThreshold || reaction.y >= joint.torqueThreshold) {
				// Flag this joint for processing.
				b2SetBit(jointStateBitSet, <u32>joint.jointId);
			}
		}
	}
}
