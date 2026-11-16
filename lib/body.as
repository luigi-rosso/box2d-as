// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/body.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX, B2_SECRET_COOKIE, b2Log } from "./core";
import { B2_HUGE, B2_NAME_LENGTH, B2_SPECULATIVE_DISTANCE } from "./constants";
import {
	b2AABB,
	b2AABB_Contains,
	b2AABB_Union,
	b2AbsFloat,
	b2Add,
	b2Cross,
	b2CrossSV,
	b2Dot,
	b2InvRotateVector,
	b2InvTransformWorldPoint,
	b2IsValidFloat,
	b2IsValidPos,
	b2IsValidRot,
	b2IsValidVec2,
	b2IsValidWorldTransform,
	b2Length,
	b2LengthSquared,
	b2MaxFloat,
	b2MinFloat,
	b2MulAdd,
	b2MulSV,
	b2Pos,
	b2RelativeAngle,
	b2Rot,
	b2Rot_identity,
	b2RotateVector,
	b2RoundDownFloat,
	b2RoundUpFloat,
	b2Sub,
	b2SubPos,
	b2Transform,
	b2TransformWorldPoint,
	b2Vec2,
	b2Vec2_zero,
	b2WorldTransform,
} from "./math_functions";
import { b2MassData, b2Sweep } from "./collision";
import { b2BodyId, b2ContactId, b2JointId, b2ShapeId, b2WorldId, b2_nullBodyId } from "./id";
import { b2BodyDef, b2BodyType, b2ContactData, b2MotionLocks, b2_dynamicBody, b2_kinematicBody, b2_staticBody } from "./types";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2Array_RemoveSwap } from "./container";
import { b2BroadPhase_MoveProxy } from "./broad_phase";
import { b2Body_IsValid, b2GetWorld, b2GetWorldFromId, b2GetWorldLocked, b2World } from "./physics_world";
import {
	b2DestroySolverSet,
	b2RemoveBodySim,
	b2SolverSet,
	b2TransferBody,
	b2TransferJoint,
	b2TrySleepIsland,
	b2WakeSolverSet,
	b2_awakeSet,
	b2_disabledSet,
	b2_firstSleepingSet,
	b2_staticSet,
} from "./solver_set";
import { b2CreateIsland, b2DestroyIsland, b2LinkJoint, b2SplitIsland, b2UnlinkJoint } from "./island";
import { b2DestroyContact, b2GetContactSimArray, b2_contactTouchingFlag, b2_simRelativeTransformValid } from "./contact";
import { b2DestroyJointInternal } from "./joint";
import { b2DestroySensor } from "./sensor";
import {
	b2ComputeFatShapeAABB,
	b2ComputeShapeExtent,
	b2ComputeShapeMass,
	b2CreateShapeProxy,
	b2DestroyShapeProxy,
	b2FreeChainData,
} from "./shape";

export type b2BodyFlags = u32;

// This body has fixed translation along the x-axis
export const b2_lockLinearX: b2BodyFlags = 0x00000001;

// This body has fixed translation along the y-axis
export const b2_lockLinearY: b2BodyFlags = 0x00000002;

// This body has fixed rotation
export const b2_lockAngularZ: b2BodyFlags = 0x00000004;

// This flag is used for debug draw and contact recycling. Only on b2BodySim.
export const b2_isFast: b2BodyFlags = 0x00000008;

// This dynamic body does a final CCD pass against all body types, but not other bullets
export const b2_isBullet: b2BodyFlags = 0x00000010;

// This body was speed capped in the current time step
export const b2_isSpeedCapped: b2BodyFlags = 0x00000020;

// This body had a time of impact event in the current time step (body sim only).
export const b2_hadTimeOfImpact: b2BodyFlags = 0x00000040;

// This body has no limit on angular velocity
export const b2_allowFastRotation: b2BodyFlags = 0x00000080;

// This bullet body needs to have its AABB increased. Needed because bullets don't follow
// the standard broad-phase update.
export const b2_enlargeBulletBounds: b2BodyFlags = 0x00000100;

// This body is dynamic so the solver should write to it.
// This prevents writing to kinematic bodies that causes a multithreaded sharing
// cache coherence problem even when the values are not changing.
// Used for b2BodyState flags.
export const b2_dynamicFlag: b2BodyFlags = 0x00000200;

// Flag to indicate the user has used the updateBodyMass option to defer mass
// computation but b2Body_UpdateMassFromShapes was not called before the world step.
export const b2_dirtyMass: b2BodyFlags = 0x00000400;

export const b2_enableSleep: b2BodyFlags = 0x00000800;

export const b2_bodyEnableContactRecycling: b2BodyFlags = 0x00001000;

// All lock flags
export const b2_allLocks: b2BodyFlags = b2_lockAngularZ | b2_lockLinearX | b2_lockLinearY;

// If this flag is set then the body has fixed rotation
export const b2_fixedRotation: b2BodyFlags = b2_lockAngularZ;

// These flags are transient per time step. These may be different across b2Body, b2BodySim, and b2BodyState.
export const b2_bodyTransientFlags: b2BodyFlags = b2_isSpeedCapped | b2_hadTimeOfImpact;

// Body organizational details that are not used in the solver.
export struct b2Body {
	userData: u64 = 0;

	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	setIndex: i32 = 0;

	// body sim and state index within set
	// may be B2_NULL_INDEX
	localIndex: i32 = 0;

	// [31 : contactId | 1 : edgeIndex]
	headContactKey: i32 = 0;
	contactCount: i32 = 0;

	// todo maybe move this to the body sim
	headShapeId: i32 = 0;
	shapeCount: i32 = 0;

	headChainId: i32 = 0;

	// [31 : jointId | 1 : edgeIndex]
	headJointKey: i32 = 0;
	jointCount: i32 = 0;

	// All enabled dynamic and kinematic bodies are in an island.
	islandId: i32 = 0;

	// Index into the island's bodies array for O(1) swap-removal.
	// B2_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	mass: f32 = 0;

	// Rotational inertia about the center of mass.
	inertia: f32 = 0;

	sleepThreshold: f32 = 0;
	sleepTime: f32 = 0;
	safetyFactor: f32 = 0;

	// this is used to adjust the fellAsleep flag in the body move array
	bodyMoveIndex: i32 = 0;

	id: i32 = 0;

	// b2BodyFlags
	flags: u32 = 0;

	type: b2BodyType = 0;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2BodyId
	generation: u16 = 0;

	// B2_NAME_LENGTH + 1, zero terminated
	name: FixedArray<u8, 11>;
}

// Body State
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// 32 bytes
export struct b2BodyState {
	linearVelocity: b2Vec2 = b2Vec2(); // 8
	angularVelocity: f32 = 0; // 4

	// b2BodyFlags
	// Important flags: locking, dynamic
	flags: u32 = 0; // 4

	// Using delta position reduces round-off error far from the origin
	deltaPosition: b2Vec2 = b2Vec2(); // 8

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	deltaRotation: b2Rot = b2Rot(); // 8
}

function b2MakeIdentityBodyState(): b2BodyState {
	let state = b2BodyState();
	state.deltaRotation.c = 1.0;
	return state;
}

// Identity body state, notice the deltaRotation is {1, 0}
export const b2_identityBodyState: b2BodyState = b2MakeIdentityBodyState();

// The element data of an Array of structs, for the solver's pointer walks; rasc refuses dataStart on value class
// arrays. The address holds until the array grows, which no solver stage does.
@inline export unsafe function b2ArrayData<A>(a: A): usize {
	return load<usize>(changetype<usize>(a) + offsetof<Array<i32>>("dataStart"));
}

// C points into a growable Array of structs; this views its buffer as a StaticArray so an element binds by ref in
// place, valid until the array grows. An Array's data is its buffer object, so the view's length is the capacity.
@inline export unsafe function b2ArrayView<A>(a: A): StaticArray<valueof<A>> {
	return changetype<StaticArray<valueof<A>>>(b2ArrayData(a));
}

function b2MakeIdentityStates(): StaticArray<b2BodyState> {
	let states = new StaticArray<b2BodyState>(1);
	states[0] = b2_identityBodyState;
	return states;
}

// b2_identityBodyState in memory, where C points static lanes and joint ends. It has no dynamic flag, so nothing
// writes it.
@parallel.shared export const b2_identityStates = b2MakeIdentityStates();

// C's state pointer: the awake state at index, or the identity state for B2_NULL_INDEX. Index comes from a
// constraint the solver prepared, so it is below the awake count.
@inline export unsafe function b2StateAddress(states: Array<b2BodyState>, index: i32): usize {
	return index == B2_NULL_INDEX
		? changetype<usize>(b2_identityStates)
		: b2ArrayData(states) + <usize>index * sizeof<b2BodyState>();
}

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
export struct b2BodySim {
	// transform for body origin
	transform: b2WorldTransform = b2Transform();

	// center of mass position in world space
	center: b2Pos = b2Vec2();

	// previous rotation and COM for TOI computation
	rotation0: b2Rot = b2Rot();
	center0: b2Pos = b2Vec2();

	// location of center of mass relative to the body origin
	localCenter: b2Vec2 = b2Vec2();

	force: b2Vec2 = b2Vec2();
	torque: f32 = 0;

	// inverse inertia
	invMass: f32 = 0;
	invInertia: f32 = 0;

	minExtent: f32 = 0;
	maxExtent: f32 = 0;
	linearDamping: f32 = 0;
	angularDamping: f32 = 0;
	gravityScale: f32 = 0;

	// body data can be moved around, the id is stable (used in b2BodyId)
	bodyId: i32 = 0;

	// b2BodyFlags
	flags: u32 = 0;
}

// Make a sweep relative to a base position to keep TOI in float precision far from the origin.
export function b2MakeRelativeSweep(bodySim: b2BodySim, base: b2Pos): b2Sweep {
	let s: b2Sweep;
	s.c1 = b2SubPos(bodySim.center0, base);
	s.c2 = b2SubPos(bodySim.center, base);
	s.q1 = bodySim.rotation0;
	s.q2 = bodySim.transform.q;
	s.localCenter = bodySim.localCenter;
	return s;
}

// b2MakeRelativeSweep of the sim at an address, read in place as C reads through its pointer
export unsafe function b2MakeRelativeSweepAt(bodySim: usize, base: b2Pos): b2Sweep {
	let s: b2Sweep;
	s.c1 = b2SubPos(load<b2Vec2>(bodySim + offsetof<b2BodySim>("center0")), base);
	s.c2 = b2SubPos(load<b2Vec2>(bodySim + offsetof<b2BodySim>("center")), base);
	s.q1 = load<b2Rot>(bodySim + offsetof<b2BodySim>("rotation0"));
	s.q2 = load<b2Rot>(bodySim + offsetof<b2BodySim>("transform") + offsetof<b2Transform>("q"));
	s.localCenter = load<b2Vec2>(bodySim + offsetof<b2BodySim>("localCenter"));
	return s;
}

// Returns a copy. Write through world.solverSets[body.setIndex].bodySims[body.localIndex].
export function b2GetBodySim(world: b2World, bodyId: i32): b2BodySim {
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	return set.bodySims[world.bodies[bodyId].localIndex];
}

// The address of the sim b2GetBodySim returns, where C returns its pointer. Valid until the set's sim array grows.
export unsafe function b2GetBodySimAddress(world: b2World, bodyId: i32): usize {
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	let localIndex = world.bodies[bodyId].localIndex;
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= localIndex && localIndex < set.bodySims.length);
	return b2ArrayData(set.bodySims) + <usize>localIndex * sizeof<b2BodySim>();
}

// Returns the index into the awake set's body states, B2_NULL_INDEX where C returns NULL.
export function b2GetBodyState(world: b2World, bodyId: i32): i32 {
	if (world.bodies[bodyId].setIndex == b2_awakeSet) {
		let localIndex = world.bodies[bodyId].localIndex;
		if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= localIndex && localIndex < world.solverSets[b2_awakeSet].bodyStates.length);
		return localIndex;
	}

	return B2_NULL_INDEX;
}

export function b2EncodeBodySimIndex(body: ref b2Body): i32 {
	if (body.setIndex == b2_awakeSet) {
		return body.localIndex;
	}

	if (body.setIndex == b2_staticSet) {
		return -(body.localIndex + 2);
	}

	return B2_NULL_INDEX;
}

@inline export function b2IsStaticSimIndex(encodedBodySimIndex: i32): bool {
	return encodedBodySimIndex < B2_NULL_INDEX;
}

@inline export function b2DecodeAwakeIndex(encodedBodySimIndex: i32): i32 {
	return encodedBodySimIndex >= 0 ? encodedBodySimIndex : B2_NULL_INDEX;
}

@inline export function b2SleepBodySimIndex(encodedIndex: i32): i32 {
	return encodedIndex >= 0 ? B2_NULL_INDEX : encodedIndex;
}

// functions from body.c follow

function b2LimitVelocity(state: ref b2BodyState, maxLinearSpeed: f32): void {
	let v2 = b2LengthSquared(state.linearVelocity);
	if (v2 > maxLinearSpeed * maxLinearSpeed) {
		state.linearVelocity = b2MulSV(maxLinearSpeed / Mathf.sqrt(v2), state.linearVelocity);
	}
}

// Refresh the body sim index on associated contact sims.
export function b2RefreshBodyContactIndices(world: b2World, bodyId: i32): void {
	let bodyView = b2ArrayView(world.bodies);
	let encodedIndex = b2EncodeBodySimIndex(bodyView[bodyId]);

	let contactKey = world.bodies[bodyId].headContactKey;
	while (contactKey != B2_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contactSims = b2GetContactSimArray(world, contactId);
		let localIndex = world.contacts[contactId].localIndex;

		if (edgeIndex == 0) {
			contactSims[localIndex].encodedBodySimA = encodedIndex;
		} else {
			contactSims[localIndex].encodedBodySimB = encodedIndex;
		}

		contactKey = world.contacts[contactId].edges[edgeIndex].nextKey;
	}
}

// Get a validated body from a world using an id.
export function b2GetBodyFullId(world: b2World, bodyId: b2BodyId): i32 {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2Body_IsValid(bodyId));

	// id index starts at one so that zero can represent null
	let index = bodyId.index1 - 1;
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= index && index < world.bodies.length);
	return index;
}

export function b2GetBodyTransformQuick(world: b2World, bodyId: i32): b2WorldTransform {
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	return set.bodySims[world.bodies[bodyId].localIndex].transform;
}

export function b2GetBodyTransform(world: b2World, bodyId: i32): b2WorldTransform {
	return b2GetBodyTransformQuick(world, bodyId);
}

// Create a b2BodyId from a raw id.
export function b2MakeBodyId(world: b2World, bodyId: i32): b2BodyId {
	return b2BodyId(bodyId + 1, world.worldId, world.bodies[bodyId].generation);
}

// Sync the body flags when something external changes them,
// such as the user changing the motion locks.
export function b2SyncBodyFlags(world: b2World, bodyId: i32): void {
	let bodyFlags = world.bodies[bodyId].flags;
	let set = world.solverSets[world.bodies[bodyId].setIndex];
	let localIndex = world.bodies[bodyId].localIndex;

	// Preserve the sim only flags: fast for contact recycling, time of impact for debug draw.
	set.bodySims[localIndex].flags =
		(set.bodySims[localIndex].flags & (b2_isFast | b2_hadTimeOfImpact)) | (bodyFlags & ~b2_bodyTransientFlags);

	let stateIndex = b2GetBodyState(world, bodyId);
	if (stateIndex != B2_NULL_INDEX) {
		world.solverSets[b2_awakeSet].bodyStates[stateIndex].flags = bodyFlags & ~b2_bodyTransientFlags;
	}
}

function b2CreateIslandForBody(world: b2World, setIndex: i32, bodyId: i32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.bodies[bodyId].islandId == B2_NULL_INDEX);
	if (B2_ASSERT_ENABLED) B2_ASSERT(setIndex != b2_disabledSet);

	let island = b2CreateIsland(world, setIndex);
	island.bodies.push(world.bodies[bodyId].id);
	world.bodies[bodyId].islandId = island.islandId;
	world.bodies[bodyId].islandIndex = 0;
}

function b2RemoveBodyFromIsland(world: b2World, bodyId: i32): void {
	let bodies = world.bodies;
	if (bodies[bodyId].islandId == B2_NULL_INDEX) {
		if (B2_ASSERT_ENABLED) B2_ASSERT(bodies[bodyId].islandIndex == B2_NULL_INDEX);
		return;
	}

	let islandId = bodies[bodyId].islandId;
	let island = world.islands[islandId];
	{
		let localIndex = bodies[bodyId].islandIndex;
		let movedBodyId = island.bodies[island.bodies.length - 1];
		island.bodies[localIndex] = movedBodyId;
		bodies[movedBodyId].islandIndex = localIndex;
		island.bodies.length = island.bodies.length - 1;
	}

	if (island.bodies.length == 0) {
		// Destroy empty island
		if (B2_ASSERT_ENABLED) B2_ASSERT(island.contacts.length == 0);
		if (B2_ASSERT_ENABLED) B2_ASSERT(island.joints.length == 0);

		// Free the island
		b2DestroyIsland(world, island.islandId);
	}

	bodies[bodyId].islandId = B2_NULL_INDEX;
	bodies[bodyId].islandIndex = B2_NULL_INDEX;
}

function b2DestroyBodyContacts(world: b2World, bodyId: i32): void {
	// Destroy the attached contacts
	let edgeKey = world.bodies[bodyId].headContactKey;
	while (edgeKey != B2_NULL_INDEX) {
		let contactId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		edgeKey = world.contacts[contactId].edges[edgeIndex].nextKey;
		b2DestroyContact(world, contactId);
	}
}

// C's strncpy into the fixed name buffer: at most B2_NAME_LENGTH bytes, zero padded, stops at a zero.
function b2SetBodyName(world: b2World, bodyId: i32, name: string): void {
	let bytes = Uint8Array.wrap(String.UTF8.encode(name));
	let count = bytes.length;
	let ended = false;
	for (let i = 0; i < B2_NAME_LENGTH; ++i) {
		let c: u8 = 0;
		if (ended == false && i < count) {
			c = bytes[i];
		}
		ended = ended || c == 0;
		world.bodies[bodyId].name[i] = c;
	}
	world.bodies[bodyId].name[B2_NAME_LENGTH] = 0;
}

export function b2CreateBody(worldId: b2WorldId, def: b2BodyDef): b2BodyId {
	if (B2_ASSERT_ENABLED) B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	if (b2IsValidPos(def.position) == false) {
		b2Log("invalid input: b2IsValidPos( def->position ) in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if (b2IsValidRot(def.rotation) == false) {
		b2Log("invalid input: b2IsValidRot( def->rotation ) in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if (b2IsValidVec2(def.linearVelocity) == false) {
		b2Log("invalid input: b2IsValidVec2( def->linearVelocity ) in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if (b2IsValidFloat(def.angularVelocity) == false) {
		b2Log("invalid input: b2IsValidFloat( def->angularVelocity ) in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if ((b2IsValidFloat(def.linearDamping) && def.linearDamping >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( def->linearDamping ) && def->linearDamping >= 0.0f in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if ((b2IsValidFloat(def.angularDamping) && def.angularDamping >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( def->angularDamping ) && def->angularDamping >= 0.0f in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if ((b2IsValidFloat(def.sleepThreshold) && def.sleepThreshold >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( def->sleepThreshold ) && def->sleepThreshold >= 0.0f in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if ((b2IsValidFloat(def.safetyFactor) && def.safetyFactor >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( def->safetyFactor ) && def->safetyFactor >= 0.0f in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}
	if (b2IsValidFloat(def.gravityScale) == false) {
		b2Log("invalid input: b2IsValidFloat( def->gravityScale ) in b2CreateBody\n");
		B2_ASSERT(false);
		return b2_nullBodyId;
	}

	let world = b2GetWorldFromId(worldId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.locked == false);

	if (world.locked) {
		return b2_nullBodyId;
	}

	let isAwake = (def.isAwake || def.enableSleep == false) && def.isEnabled;

	// determine the solver set
	let setId: i32;
	if (def.isEnabled == false) {
		// any body type can be disabled
		setId = b2_disabledSet;
	} else if (def.type == b2_staticBody) {
		setId = b2_staticSet;
	} else if (isAwake == true) {
		setId = b2_awakeSet;
	} else {
		// new set for a sleeping body in its own island
		setId = b2AllocId(world.solverSetIdPool);
		if (setId == world.solverSets.length) {
			// Create a zero initialized solver set. All sub-arrays are also zero initialized.
			world.solverSets.push(new b2SolverSet());
		} else {
			if (B2_ASSERT_ENABLED) B2_ASSERT(world.solverSets[setId].setIndex == B2_NULL_INDEX);
		}

		world.solverSets[setId].setIndex = setId;
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= setId && setId < world.solverSets.length);

	let bodyId = b2AllocId(world.bodyIdPool);

	let lockFlags: u32 = 0;
	lockFlags |= def.motionLocks.linearX ? b2_lockLinearX : 0;
	lockFlags |= def.motionLocks.linearY ? b2_lockLinearY : 0;
	lockFlags |= def.motionLocks.angularZ ? b2_lockAngularZ : 0;

	let set = world.solverSets[setId];
	let bodySim = b2BodySim();
	bodySim.transform.p = def.position;
	bodySim.transform.q = def.rotation;
	bodySim.center = def.position;
	bodySim.rotation0 = bodySim.transform.q;
	bodySim.center0 = bodySim.center;
	bodySim.minExtent = B2_HUGE();
	bodySim.maxExtent = 0.0;
	bodySim.linearDamping = def.linearDamping;
	bodySim.angularDamping = def.angularDamping;
	bodySim.gravityScale = def.gravityScale;
	bodySim.bodyId = bodyId;
	bodySim.flags = lockFlags;
	bodySim.flags |= def.isBullet ? b2_isBullet : 0;
	bodySim.flags |= def.allowFastRotation ? b2_allowFastRotation : 0;
	bodySim.flags |= def.type == b2_dynamicBody ? b2_dynamicFlag : 0;
	bodySim.flags |= def.enableSleep ? b2_enableSleep : 0;
	bodySim.flags |= def.enableContactRecycling ? b2_bodyEnableContactRecycling : 0;
	set.bodySims.push(bodySim);

	if (setId == b2_awakeSet) {
		let bodyState = b2BodyState();
		bodyState.linearVelocity = def.linearVelocity;
		bodyState.angularVelocity = def.angularVelocity;
		bodyState.deltaRotation = b2Rot_identity;
		bodyState.flags = bodySim.flags;
		set.bodyStates.push(bodyState);
	}

	let bodies = world.bodies;
	if (bodyId == bodies.length) {
		bodies.push(b2Body());
	} else {
		if (B2_ASSERT_ENABLED) B2_ASSERT(bodies[bodyId].id == B2_NULL_INDEX);
	}

	b2SetBodyName(world, bodyId, def.name);

	bodies[bodyId].userData = def.userData;
	bodies[bodyId].setIndex = setId;
	bodies[bodyId].localIndex = set.bodySims.length - 1;
	bodies[bodyId].generation += 1;
	bodies[bodyId].headShapeId = B2_NULL_INDEX;
	bodies[bodyId].shapeCount = 0;
	bodies[bodyId].headChainId = B2_NULL_INDEX;
	bodies[bodyId].headContactKey = B2_NULL_INDEX;
	bodies[bodyId].contactCount = 0;
	bodies[bodyId].headJointKey = B2_NULL_INDEX;
	bodies[bodyId].jointCount = 0;
	bodies[bodyId].islandId = B2_NULL_INDEX;
	bodies[bodyId].islandIndex = B2_NULL_INDEX;
	bodies[bodyId].bodyMoveIndex = B2_NULL_INDEX;
	bodies[bodyId].id = bodyId;
	bodies[bodyId].mass = 0.0;
	bodies[bodyId].inertia = 0.0;
	bodies[bodyId].sleepThreshold = def.sleepThreshold;
	bodies[bodyId].sleepTime = 0.0;
	bodies[bodyId].safetyFactor = def.safetyFactor;
	bodies[bodyId].type = def.type;
	bodies[bodyId].flags = bodySim.flags;

	// dynamic and kinematic bodies that are enabled need a island
	if (setId >= b2_awakeSet) {
		b2CreateIslandForBody(world, setId, bodyId);
	}

	return b2BodyId(bodyId + 1, world.worldId, bodies[bodyId].generation);
}

// Careful: this moves body sims and states, re-read any copy taken before.
export function b2WakeBody(world: b2World, bodyId: i32): bool {
	let setIndex = world.bodies[bodyId].setIndex;
	if (setIndex >= b2_firstSleepingSet) {
		b2WakeSolverSet(world, setIndex);
		return true;
	}

	return false;
}

export function b2DestroyBody(bodyId: b2BodyId): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let bodies = world.bodies;

	// Destroy the attached joints
	let edgeKey = bodies[body].headJointKey;
	while (edgeKey != B2_NULL_INDEX) {
		let jointId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		edgeKey = world.joints[jointId].edges[edgeIndex].nextKey;

		// Careful because this modifies the list being traversed
		b2DestroyJointInternal(world, jointId);
	}

	// Destroy all contacts attached to this body.
	b2DestroyBodyContacts(world, body);

	// Destroy the attached shapes and their broad-phase proxies.
	let shapes = world.shapes;
	let shapeId = bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		if (shapes[shapeId].sensorIndex != B2_NULL_INDEX) {
			b2DestroySensor(world, shapeId);
		}

		b2DestroyShapeProxy(world, shapeId);

		// Return shape to free list.
		b2FreeId(world.shapeIdPool, shapeId);
		shapes[shapeId].id = B2_NULL_INDEX;

		shapeId = shapes[shapeId].nextShapeId;
	}

	// Destroy the attached chains. The associated shapes have already been destroyed above.
	let chainId = bodies[body].headChainId;
	while (chainId != B2_NULL_INDEX) {
		let chain = world.chainShapes[chainId];

		b2FreeChainData(chain);

		// Return chain to free list.
		b2FreeId(world.chainIdPool, chainId);
		chain.id = B2_NULL_INDEX;

		chainId = chain.nextChainId;
	}

	b2RemoveBodyFromIsland(world, body);

	// Remove body sim from solver set that owns it
	let set = world.solverSets[bodies[body].setIndex];
	let movedBody = b2RemoveBodySim(world, set, bodies[body].localIndex);
	if (movedBody != B2_NULL_INDEX) {
		b2RefreshBodyContactIndices(world, movedBody);
	}

	// Remove body state from awake set
	if (bodies[body].setIndex == b2_awakeSet) {
		b2Array_RemoveSwap(set.bodyStates, bodies[body].localIndex);
	} else if (set.setIndex >= b2_firstSleepingSet && set.bodySims.length == 0) {
		// Remove solver set if it is empty
		b2DestroySolverSet(world, set.setIndex);
	}

	// Free body and id (preserve body generation)
	b2FreeId(world.bodyIdPool, bodies[body].id);

	bodies[body].setIndex = B2_NULL_INDEX;
	bodies[body].localIndex = B2_NULL_INDEX;
	bodies[body].id = B2_NULL_INDEX;
}

export function b2Body_GetContactCapacity(bodyId: b2BodyId): i32 {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return 0;
	}

	let body = b2GetBodyFullId(world, bodyId);

	// Conservative and fast
	return world.bodies[body].contactCount;
}

export function b2Body_GetContactData(bodyId: b2BodyId, contactData: StaticArray<b2ContactData>, capacity: i32): i32 {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return 0;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let shapes = world.shapes;

	let contactKey = world.bodies[body].headContactKey;
	let index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];

		// Is contact touching?
		if (contact.flags & b2_contactTouchingFlag) {
			let shapeIdA = contact.shapeIdA;
			let shapeIdB = contact.shapeIdB;

			contactData[index].contactId = b2MakeContactIdFrom(contact.contactId + 1, bodyId.world0, contact.generation);
			contactData[index].shapeIdA = b2ShapeId(shapes[shapeIdA].id + 1, bodyId.world0, shapes[shapeIdA].generation);
			contactData[index].shapeIdB = b2ShapeId(shapes[shapeIdB].id + 1, bodyId.world0, shapes[shapeIdB].generation);

			let contactSims = b2GetContactSimArray(world, contactId);
			contactData[index].manifold = contactSims[contact.localIndex].manifold;

			index += 1;
		}

		contactKey = contact.edges[edgeIndex].nextKey;
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(index <= capacity);

	return index;
}

export function b2Body_ComputeBounds(bodyId: b2BodyId): b2AABB {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return b2AABB();
	}

	let body = b2GetBodyFullId(world, bodyId);
	if (world.bodies[body].headShapeId == B2_NULL_INDEX) {
		// No shapes, bracket the body origin so the box still contains the true position far away
		let transform = b2GetBodyTransform(world, world.bodies[body].id);
		let lower = b2Vec2(b2RoundDownFloat(<f64>transform.p.x), b2RoundDownFloat(<f64>transform.p.y));
		let upper = b2Vec2(b2RoundUpFloat(<f64>transform.p.x), b2RoundUpFloat(<f64>transform.p.y));
		return b2AABB(lower, upper);
	}

	let shapes = world.shapes;
	let shapeId = world.bodies[body].headShapeId;
	let aabb = shapes[shapeId].aabb;
	while (shapes[shapeId].nextShapeId != B2_NULL_INDEX) {
		shapeId = shapes[shapeId].nextShapeId;
		aabb = b2AABB_Union(aabb, shapes[shapeId].aabb);
	}

	return aabb;
}

// Per call mass data scratch, C's stack allocation. Only grows.
let b2_massDataScratch = new StaticArray<b2MassData>(0);

export function b2UpdateBodyMassData(world: b2World, bodyId: i32): void {
	let bodies = world.bodies;
	let shapes = world.shapes;
	let set = world.solverSets[bodies[bodyId].setIndex];
	let localIndex = bodies[bodyId].localIndex;
	let bodySim = set.bodySims[localIndex];

	// Mass is no longer dirty
	bodies[bodyId].flags &= ~b2_dirtyMass;

	// Compute mass data from shapes. Each shape has its own density.
	let bodyMass: f32 = 0.0;
	let bodyInertia: f32 = 0.0;

	bodySim.invMass = 0.0;
	bodySim.invInertia = 0.0;
	bodySim.localCenter = b2Vec2_zero;
	bodySim.minExtent = B2_HUGE();
	bodySim.maxExtent = 0.0;

	// Static and kinematic sims have zero mass.
	let bodyType = bodies[bodyId].type;
	if (bodyType != b2_dynamicBody) {
		bodySim.center = bodySim.transform.p;
		bodySim.center0 = bodySim.center;

		// Need extents for kinematic bodies for sleeping to work correctly.
		if (bodyType == b2_kinematicBody) {
			let shapeId = bodies[bodyId].headShapeId;
			while (shapeId != B2_NULL_INDEX) {
				let extent = b2ComputeShapeExtent(world, shapeId, b2Vec2_zero);
				bodySim.minExtent = b2MinFloat(bodySim.minExtent, extent.minExtent);
				bodySim.maxExtent = b2MaxFloat(bodySim.maxExtent, extent.maxExtent);

				shapeId = shapes[shapeId].nextShapeId;
			}
		}

		bodies[bodyId].mass = bodyMass;
		bodies[bodyId].inertia = bodyInertia;
		set.bodySims[localIndex] = bodySim;
		return;
	}

	let shapeCount = bodies[bodyId].shapeCount;
	if (b2_massDataScratch.length < shapeCount) {
		b2_massDataScratch = new StaticArray<b2MassData>(shapeCount + shapeCount / 4 + 16);
	}
	let masses = b2_massDataScratch;

	// Accumulate mass over all shapes.
	let localCenter = b2Vec2_zero;
	let shapeId = bodies[bodyId].headShapeId;
	let shapeIndex = 0;
	while (shapeId != B2_NULL_INDEX) {
		let density = shapes[shapeId].density;
		let currentId = shapeId;
		shapeId = shapes[shapeId].nextShapeId;

		if (density == 0.0) {
			masses[shapeIndex] = b2MassData();
			shapeIndex += 1;
			continue;
		}

		let massData = b2ComputeShapeMass(world, currentId);
		bodyMass += massData.mass;
		localCenter = b2MulAdd(localCenter, massData.mass, massData.center);

		masses[shapeIndex] = massData;
		shapeIndex += 1;
	}

	// Compute center of mass.
	if (bodyMass > 0.0) {
		bodySim.invMass = 1.0 / bodyMass;
		localCenter = b2MulSV(bodySim.invMass, localCenter);
	}

	// Second loop to accumulate the rotational inertia about the center of mass
	for (shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex) {
		let massData = masses[shapeIndex];
		if (massData.mass == 0.0) {
			continue;
		}

		// Shift to center of mass. This is safe because it can only increase.
		let offset = b2Sub(localCenter, massData.center);
		let inertia: f32 = massData.rotationalInertia + massData.mass * b2Dot(offset, offset);
		bodyInertia += inertia;
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(bodyInertia >= 0.0);

	if (bodyInertia > 0.0 && (bodies[bodyId].flags & b2_fixedRotation) == 0) {
		bodySim.invInertia = 1.0 / bodyInertia;
	} else {
		bodyInertia = 0.0;
		bodySim.invInertia = 0.0;
	}

	bodies[bodyId].mass = bodyMass;
	bodies[bodyId].inertia = bodyInertia;

	// Move center of mass.
	let oldCenter = bodySim.center;
	bodySim.localCenter = localCenter;
	bodySim.center = b2TransformWorldPoint(bodySim.transform, bodySim.localCenter);
	bodySim.center0 = bodySim.center;

	// Update center of mass velocity
	let state = b2GetBodyState(world, bodyId);
	if (state != B2_NULL_INDEX) {
		let states = world.solverSets[b2_awakeSet].bodyStates;
		let deltaLinear = b2CrossSV(states[state].angularVelocity, b2SubPos(bodySim.center, oldCenter));
		states[state].linearVelocity = b2Add(states[state].linearVelocity, deltaLinear);
	}

	// Compute body extents relative to center of mass
	shapeId = bodies[bodyId].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let extent = b2ComputeShapeExtent(world, shapeId, localCenter);
		bodySim.minExtent = b2MinFloat(bodySim.minExtent, extent.minExtent);
		bodySim.maxExtent = b2MaxFloat(bodySim.maxExtent, extent.maxExtent);

		shapeId = shapes[shapeId].nextShapeId;
	}

	set.bodySims[localIndex] = bodySim;

	// When the center of mass changes, any cached contact manifold becomes invalid.
	b2InvalidateContactTransforms(world, bodyId);
}

function b2InvalidateContactTransforms(world: b2World, bodyId: i32): void {
	let edgeKey = world.bodies[bodyId].headContactKey;
	while (edgeKey != B2_NULL_INDEX) {
		let contactId = edgeKey >> 1;
		let edgeIndex = edgeKey & 1;

		let contactSims = b2GetContactSimArray(world, contactId);
		contactSims[world.contacts[contactId].localIndex].simFlags &= ~b2_simRelativeTransformValid;

		edgeKey = world.contacts[contactId].edges[edgeIndex].nextKey;
	}
}

export function b2Body_GetPosition(bodyId: b2BodyId): b2Pos {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);
	return transform.p;
}

export function b2Body_GetRotation(bodyId: b2BodyId): b2Rot {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);
	return transform.q;
}

export function b2Body_GetTransform(bodyId: b2BodyId): b2WorldTransform {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return b2GetBodyTransformQuick(world, body);
}

export function b2Body_GetLocalPoint(bodyId: b2BodyId, worldPoint: b2Pos): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);
	return b2InvTransformWorldPoint(transform, worldPoint);
}

export function b2Body_GetWorldPoint(bodyId: b2BodyId, localPoint: b2Vec2): b2Pos {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);
	return b2TransformWorldPoint(transform, localPoint);
}

export function b2Body_GetLocalVector(bodyId: b2BodyId, worldVector: b2Vec2): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);
	return b2InvRotateVector(transform.q, worldVector);
}

export function b2Body_GetWorldVector(bodyId: b2BodyId, localVector: b2Vec2): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);
	return b2RotateVector(transform.q, localVector);
}

export function b2Body_SetTransform(bodyId: b2BodyId, position: b2Pos, rotation: b2Rot): void {
	if (b2IsValidPos(position) == false) {
		b2Log("invalid input: b2IsValidPos( position ) in b2Body_SetTransform\n");
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidRot(rotation) == false) {
		b2Log("invalid input: b2IsValidRot( rotation ) in b2Body_SetTransform\n");
		B2_ASSERT(false);
		return;
	}
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2Body_IsValid(bodyId));
	let world = b2GetWorld(bodyId.world0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.locked == false);

	let body = b2GetBodyFullId(world, bodyId);
	let set = world.solverSets[world.bodies[body].setIndex];
	let localIndex = world.bodies[body].localIndex;
	let bodySim = set.bodySims[localIndex];

	bodySim.transform.p = position;
	bodySim.transform.q = rotation;
	bodySim.center = b2TransformWorldPoint(bodySim.transform, bodySim.localCenter);

	bodySim.rotation0 = bodySim.transform.q;
	bodySim.center0 = bodySim.center;
	set.bodySims[localIndex] = bodySim;

	let broadPhase = world.broadPhase;
	let shapes = world.shapes;
	let fatAABBs = world.fatAABBs;

	let transform = bodySim.transform;
	let speculativeDistance = B2_SPECULATIVE_DISTANCE();

	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let aabb = b2ComputeFatShapeAABB(world, shapeId, transform, speculativeDistance);
		shapes[shapeId].aabb = aabb;

		if (b2AABB_Contains(fatAABBs[shapeId], aabb) == false) {
			let margin = shapes[shapeId].aabbMargin;
			let fatAABB = b2AABB();
			fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
			fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
			fatAABB.upperBound.x = aabb.upperBound.x + margin;
			fatAABB.upperBound.y = aabb.upperBound.y + margin;
			fatAABBs[shapeId] = fatAABB;

			// They body could be disabled
			let proxyKey = shapes[shapeId].proxyKey;
			if (proxyKey != B2_NULL_INDEX) {
				b2BroadPhase_MoveProxy(broadPhase, proxyKey, fatAABB);
			}
		}

		shapeId = shapes[shapeId].nextShapeId;
	}
}

export function b2Body_GetLinearVelocity(bodyId: b2BodyId): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let state = b2GetBodyState(world, body);
	if (state != B2_NULL_INDEX) {
		return world.solverSets[b2_awakeSet].bodyStates[state].linearVelocity;
	}
	return b2Vec2_zero;
}

export function b2Body_GetAngularVelocity(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let state = b2GetBodyState(world, body);
	if (state != B2_NULL_INDEX) {
		return world.solverSets[b2_awakeSet].bodyStates[state].angularVelocity;
	}
	return 0.0;
}

export function b2Body_SetLinearVelocity(bodyId: b2BodyId, linearVelocity: b2Vec2): void {
	if (b2IsValidVec2(linearVelocity) == false) {
		b2Log("invalid input: b2IsValidVec2( linearVelocity ) in b2Body_SetLinearVelocity\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type == b2_staticBody) {
		return;
	}

	if (b2LengthSquared(linearVelocity) > 0.0) {
		b2WakeBody(world, body);
	}

	let state = b2GetBodyState(world, body);
	if (state == B2_NULL_INDEX) {
		return;
	}

	world.solverSets[b2_awakeSet].bodyStates[state].linearVelocity = linearVelocity;
}

export function b2Body_SetAngularVelocity(bodyId: b2BodyId, angularVelocity: f32): void {
	if (b2IsValidFloat(angularVelocity) == false) {
		b2Log("invalid input: b2IsValidFloat( angularVelocity ) in b2Body_SetAngularVelocity\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type == b2_staticBody || (world.bodies[body].flags & b2_lockAngularZ) != 0) {
		return;
	}

	if (angularVelocity != 0.0) {
		b2WakeBody(world, body);
	}

	let state = b2GetBodyState(world, body);
	if (state == B2_NULL_INDEX) {
		return;
	}

	world.solverSets[b2_awakeSet].bodyStates[state].angularVelocity = angularVelocity;
}

export function b2Body_SetTargetTransform(bodyId: b2BodyId, target: b2WorldTransform, timeStep: f32, wake: bool): void {
	if (b2IsValidWorldTransform(target) == false) {
		b2Log("invalid input: b2IsValidWorldTransform( target ) in b2Body_SetTargetTransform\n");
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidFloat(timeStep) == false) {
		b2Log("invalid input: b2IsValidFloat( timeStep ) in b2Body_SetTargetTransform\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let bodies = world.bodies;

	if (bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (bodies[body].type == b2_staticBody || timeStep <= 0.0) {
		return;
	}

	if (bodies[body].setIndex != b2_awakeSet && wake == false) {
		return;
	}

	let sim = b2GetBodySim(world, body);

	// Compute linear velocity. The center difference is taken in world precision then demoted
	let delta = b2SubPos(b2TransformWorldPoint(target, sim.localCenter), sim.center);
	let invTimeStep: f32 = 1.0 / timeStep;
	let linearVelocity = b2MulSV(invTimeStep, delta);

	// Compute angular velocity
	let q1 = sim.transform.q;
	let q2 = target.q;
	let deltaAngle = b2RelativeAngle(q1, q2);
	let angularVelocity: f32 = invTimeStep * deltaAngle;

	// Early out if the body is asleep already and the desired movement is small
	if (bodies[body].setIndex != b2_awakeSet) {
		let maxVelocity: f32 = b2Length(linearVelocity) + b2AbsFloat(angularVelocity) * sim.maxExtent;

		// Return if velocity would be sleepy
		if (maxVelocity < bodies[body].sleepThreshold) {
			return;
		}

		// Must wake for state to exist
		b2WakeBody(world, body);
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(bodies[body].setIndex == b2_awakeSet);

	let state = b2GetBodyState(world, body);
	let states = world.solverSets[b2_awakeSet].bodyStates;
	states[state].linearVelocity = linearVelocity;
	states[state].angularVelocity = angularVelocity;
}

export function b2Body_GetLocalPointVelocity(bodyId: b2BodyId, localPoint: b2Vec2): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let state = b2GetBodyState(world, body);
	if (state == B2_NULL_INDEX) {
		return b2Vec2_zero;
	}

	let set = world.solverSets[world.bodies[body].setIndex];
	let localIndex = world.bodies[body].localIndex;
	let q = set.bodySims[localIndex].transform.q;
	let localCenter = set.bodySims[localIndex].localCenter;
	let states = world.solverSets[b2_awakeSet].bodyStates;

	let r = b2RotateVector(q, b2Sub(localPoint, localCenter));
	let v = b2Add(states[state].linearVelocity, b2CrossSV(states[state].angularVelocity, r));
	return v;
}

export function b2Body_GetWorldPointVelocity(bodyId: b2BodyId, worldPoint: b2Pos): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let state = b2GetBodyState(world, body);
	if (state == B2_NULL_INDEX) {
		return b2Vec2_zero;
	}

	let set = world.solverSets[world.bodies[body].setIndex];
	let center = set.bodySims[world.bodies[body].localIndex].center;
	let states = world.solverSets[b2_awakeSet].bodyStates;

	let r = b2SubPos(worldPoint, center);
	let v = b2Add(states[state].linearVelocity, b2CrossSV(states[state].angularVelocity, r));
	return v;
}

export function b2Body_ApplyForce(bodyId: b2BodyId, force: b2Vec2, point: b2Pos, wake: bool): void {
	if (b2IsValidVec2(force) == false) {
		b2Log("invalid input: b2IsValidVec2( force ) in b2Body_ApplyForce\n");
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidPos(point) == false) {
		b2Log("invalid input: b2IsValidPos( point ) in b2Body_ApplyForce\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type != b2_dynamicBody || world.bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (wake && world.bodies[body].setIndex >= b2_firstSleepingSet) {
		b2WakeBody(world, body);
	}

	if (world.bodies[body].setIndex == b2_awakeSet) {
		let sims = world.solverSets[b2_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].force = b2Add(sims[localIndex].force, force);
		sims[localIndex].torque += b2Cross(b2SubPos(point, sims[localIndex].center), force);
	}
}

export function b2Body_ApplyForceToCenter(bodyId: b2BodyId, force: b2Vec2, wake: bool): void {
	if (b2IsValidVec2(force) == false) {
		b2Log("invalid input: b2IsValidVec2( force ) in b2Body_ApplyForceToCenter\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type != b2_dynamicBody || world.bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (wake && world.bodies[body].setIndex >= b2_firstSleepingSet) {
		b2WakeBody(world, body);
	}

	if (world.bodies[body].setIndex == b2_awakeSet) {
		let sims = world.solverSets[b2_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].force = b2Add(sims[localIndex].force, force);
	}
}

export function b2Body_ApplyTorque(bodyId: b2BodyId, torque: f32, wake: bool): void {
	if (b2IsValidFloat(torque) == false) {
		b2Log("invalid input: b2IsValidFloat( torque ) in b2Body_ApplyTorque\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type != b2_dynamicBody || world.bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (wake && world.bodies[body].setIndex >= b2_firstSleepingSet) {
		b2WakeBody(world, body);
	}

	if (world.bodies[body].setIndex == b2_awakeSet) {
		world.solverSets[b2_awakeSet].bodySims[world.bodies[body].localIndex].torque += torque;
	}
}

export function b2Body_ClearForces(bodyId: b2BodyId): void {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let sims = world.solverSets[world.bodies[body].setIndex].bodySims;
	let localIndex = world.bodies[body].localIndex;
	sims[localIndex].force = b2Vec2_zero;
	sims[localIndex].torque = 0.0;
}

export function b2Body_ApplyLinearImpulse(bodyId: b2BodyId, impulse: b2Vec2, point: b2Pos, wake: bool): void {
	if (b2IsValidVec2(impulse) == false) {
		b2Log("invalid input: b2IsValidVec2( impulse ) in b2Body_ApplyLinearImpulse\n");
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidPos(point) == false) {
		b2Log("invalid input: b2IsValidPos( point ) in b2Body_ApplyLinearImpulse\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type != b2_dynamicBody || world.bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (wake && world.bodies[body].setIndex >= b2_firstSleepingSet) {
		b2WakeBody(world, body);
	}

	if (world.bodies[body].setIndex == b2_awakeSet) {
		let localIndex = world.bodies[body].localIndex;
		let set = world.solverSets[b2_awakeSet];
		let state = set.bodyStates[localIndex];
		let invMass = set.bodySims[localIndex].invMass;
		let invInertia = set.bodySims[localIndex].invInertia;
		let center = set.bodySims[localIndex].center;
		state.linearVelocity = b2MulAdd(state.linearVelocity, invMass, impulse);
		state.angularVelocity += invInertia * b2Cross(b2SubPos(point, center), impulse);

		b2LimitVelocity(state, world.maxLinearSpeed);
		set.bodyStates[localIndex] = state;
	}
}

export function b2Body_ApplyLinearImpulseToCenter(bodyId: b2BodyId, impulse: b2Vec2, wake: bool): void {
	if (b2IsValidVec2(impulse) == false) {
		b2Log("invalid input: b2IsValidVec2( impulse ) in b2Body_ApplyLinearImpulseToCenter\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type != b2_dynamicBody || world.bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (wake && world.bodies[body].setIndex >= b2_firstSleepingSet) {
		b2WakeBody(world, body);
	}

	if (world.bodies[body].setIndex == b2_awakeSet) {
		let localIndex = world.bodies[body].localIndex;
		let set = world.solverSets[b2_awakeSet];
		let state = set.bodyStates[localIndex];
		state.linearVelocity = b2MulAdd(state.linearVelocity, set.bodySims[localIndex].invMass, impulse);

		b2LimitVelocity(state, world.maxLinearSpeed);
		set.bodyStates[localIndex] = state;
	}
}

export function b2Body_ApplyAngularImpulse(bodyId: b2BodyId, impulse: f32, wake: bool): void {
	if (b2IsValidFloat(impulse) == false) {
		b2Log("invalid input: b2IsValidFloat( impulse ) in b2Body_ApplyAngularImpulse\n");
		B2_ASSERT(false);
		return;
	}
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2Body_IsValid(bodyId));
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);

	if (world.bodies[body].type != b2_dynamicBody || world.bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	if (wake && world.bodies[body].setIndex >= b2_firstSleepingSet) {
		// this will not invalidate body pointer
		b2WakeBody(world, body);
	}

	if (world.bodies[body].setIndex == b2_awakeSet) {
		let localIndex = world.bodies[body].localIndex;
		let set = world.solverSets[b2_awakeSet];
		set.bodyStates[localIndex].angularVelocity += set.bodySims[localIndex].invInertia * impulse;
	}
}

export function b2Body_GetType(bodyId: b2BodyId): b2BodyType {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].type;
}

// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
// todo with a bit more effort I could support an option to let the body sleep
//
// Revised steps:
// 1 Skip disabled bodies
// 2 Destroy all contacts on the body
// 3 Wake the body
// 4 For all joints attached to the body
//  - wake attached bodies
//  - remove from island
//  - move to static set temporarily
// 5 Change the body type and transfer the body
// 6 If the body was static
//   - create an island for the body
//   Else if the body is becoming static
//   - remove it from the island
// 7 For all joints
//  - if either body is non-static
//    - link into island
//    - transfer to constraint graph
// 8 For all shapes
//  - Destroy proxy in old tree
//  - Create proxy in new tree
// Notes:
// - the implementation below tries to minimize the number of predicates, so some
//   operations may have no effect, such as transferring a joint to the same set
export function b2Body_SetType(bodyId: b2BodyId, type: b2BodyType): void {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let bodies = world.bodies;
	let joints = world.joints;

	let originalType = bodies[body].type;
	if (originalType == type) {
		return;
	}

	// Stage 1: skip disabled bodies
	if (bodies[body].setIndex == b2_disabledSet) {
		// Disabled bodies don't change solver sets or islands when they change type.
		bodies[body].type = type;

		if (type == b2_dynamicBody) {
			bodies[body].flags |= b2_dynamicFlag;
		} else {
			bodies[body].flags &= ~b2_dynamicFlag;
		}

		b2SyncBodyFlags(world, body);

		// Body type affects the mass properties
		b2UpdateBodyMassData(world, body);
		return;
	}

	// Stage 2: destroy all contacts
	b2DestroyBodyContacts(world, body);

	// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
	// all bodies in the same sleeping solver set.
	b2WakeBody(world, body);

	// Stage 4: move joints to temporary storage
	let staticSet = world.solverSets[b2_staticSet];

	let jointKey = bodies[body].headJointKey;
	while (jointKey != B2_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = joints[jointId].edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if (joints[jointId].setIndex == b2_disabledSet) {
			continue;
		}

		// Wake attached bodies. The b2WakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may have
		// no joints.
		b2WakeBody(world, joints[jointId].edges[0].bodyId);
		b2WakeBody(world, joints[jointId].edges[1].bodyId);

		// Remove joint from island
		b2UnlinkJoint(world, jointId);

		// It is necessary to transfer all joints to the static set
		// so they can be added to the constraint graph below and acquire consistent colors.
		let jointSourceSet = world.solverSets[joints[jointId].setIndex];
		b2TransferJoint(world, staticSet, jointSourceSet, jointId);
	}

	// Stage 5: change the body type and transfer body
	bodies[body].type = type;

	if (type == b2_dynamicBody) {
		bodies[body].flags |= b2_dynamicFlag;
	} else {
		bodies[body].flags &= ~b2_dynamicFlag;
	}

	let awakeSet = world.solverSets[b2_awakeSet];
	let sourceSet = world.solverSets[bodies[body].setIndex];
	let targetSet = type == b2_staticBody ? staticSet : awakeSet;

	// Transfer body
	b2TransferBody(world, targetSet, sourceSet, body);

	// Stage 6: update island participation for the body
	if (originalType == b2_staticBody) {
		// Create island for body
		b2CreateIslandForBody(world, b2_awakeSet, body);
	} else if (type == b2_staticBody) {
		// Remove body from island.
		b2RemoveBodyFromIsland(world, body);
	}

	// Stage 7: Transfer joints to the target set
	jointKey = bodies[body].headJointKey;
	while (jointKey != B2_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = joints[jointId].edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if (joints[jointId].setIndex == b2_disabledSet) {
			continue;
		}

		// All joints were transferred to the static set in an earlier stage
		if (B2_ASSERT_ENABLED) B2_ASSERT(joints[jointId].setIndex == b2_staticSet);

		let bodyIdA = joints[jointId].edges[0].bodyId;
		let bodyIdB = joints[jointId].edges[1].bodyId;
		if (B2_ASSERT_ENABLED) {
			B2_ASSERT(bodies[bodyIdA].setIndex == b2_staticSet || bodies[bodyIdA].setIndex == b2_awakeSet);
			B2_ASSERT(bodies[bodyIdB].setIndex == b2_staticSet || bodies[bodyIdB].setIndex == b2_awakeSet);
		}

		if (bodies[bodyIdA].type == b2_dynamicBody || bodies[bodyIdB].type == b2_dynamicBody) {
			b2TransferJoint(world, awakeSet, staticSet, jointId);
		}
	}

	// Recreate shape proxies in broadphase
	let transform = b2GetBodyTransformQuick(world, body);
	let shapeId = bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let currentId = shapeId;
		shapeId = world.shapes[shapeId].nextShapeId;
		b2DestroyShapeProxy(world, currentId);
		let forcePairCreation = true;
		b2CreateShapeProxy(world, currentId, type, transform, forcePairCreation);
	}

	// Relink all joints
	jointKey = bodies[body].headJointKey;
	while (jointKey != B2_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = joints[jointId].edges[edgeIndex].nextKey;

		let otherEdgeIndex = edgeIndex ^ 1;
		let otherBodyId = joints[jointId].edges[otherEdgeIndex].bodyId;

		if (bodies[otherBodyId].setIndex == b2_disabledSet) {
			continue;
		}

		if (bodies[body].type != b2_dynamicBody && bodies[otherBodyId].type != b2_dynamicBody) {
			continue;
		}

		b2LinkJoint(world, jointId);
	}

	b2SyncBodyFlags(world, body);

	// Body type affects the mass
	b2UpdateBodyMassData(world, body);
}

export function b2Body_SetName(bodyId: b2BodyId, name: string): void {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	b2SetBodyName(world, body, name);
}

export function b2Body_GetName(bodyId: b2BodyId): string {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let count = 0;
	while (count < B2_NAME_LENGTH && world.bodies[body].name[count] != 0) {
		count += 1;
	}

	let bytes = new Uint8Array(count);
	for (let i = 0; i < count; ++i) {
		bytes[i] = world.bodies[body].name[i];
	}

	return String.UTF8.decode(bytes.buffer);
}

export function b2Body_SetUserData(bodyId: b2BodyId, userData: u64): void {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	world.bodies[body].userData = userData;
}

export function b2Body_GetUserData(bodyId: b2BodyId): u64 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].userData;
}

export function b2Body_GetMass(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].mass;
}

export function b2Body_GetRotationalInertia(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].inertia;
}

export function b2Body_GetLocalCenter(bodyId: b2BodyId): b2Vec2 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].localCenter;
}

export function b2Body_GetWorldCenter(bodyId: b2BodyId): b2Pos {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].center;
}

export function b2Body_SetMassData(bodyId: b2BodyId, massData: b2MassData): void {
	if ((b2IsValidFloat(massData.mass) && massData.mass >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( massData.mass ) && massData.mass >= 0.0f in b2Body_SetMassData\n");
		B2_ASSERT(false);
		return;
	}
	if ((b2IsValidFloat(massData.rotationalInertia) && massData.rotationalInertia >= 0.0) == false) {
		b2Log(
			"invalid input: b2IsValidFloat( massData.rotationalInertia ) && massData.rotationalInertia >= 0.0f in b2Body_SetMassData\n",
		);
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidVec2(massData.center) == false) {
		b2Log("invalid input: b2IsValidVec2( massData.center ) in b2Body_SetMassData\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let bodies = world.bodies;
	let set = world.solverSets[bodies[body].setIndex];
	let localIndex = bodies[body].localIndex;
	let bodySim = set.bodySims[localIndex];

	bodies[body].mass = massData.mass;
	bodies[body].inertia = massData.rotationalInertia;
	bodySim.localCenter = massData.center;

	let oldCenter = bodySim.center;
	let center = b2TransformWorldPoint(bodySim.transform, massData.center);
	bodySim.center = center;
	bodySim.center0 = center;

	// Update center of mass velocity
	let state = b2GetBodyState(world, body);
	if (state != B2_NULL_INDEX) {
		let states = world.solverSets[b2_awakeSet].bodyStates;
		let deltaLinear = b2CrossSV(states[state].angularVelocity, b2SubPos(bodySim.center, oldCenter));
		states[state].linearVelocity = b2Add(states[state].linearVelocity, deltaLinear);
	}

	let mass = bodies[body].mass;
	let inertia = bodies[body].inertia;
	bodySim.invMass = mass > 0.0 ? 1.0 / mass : <f32>0.0;
	bodySim.invInertia = inertia > 0.0 ? 1.0 / inertia : <f32>0.0;

	// Update extents using supplied mass center.
	bodySim.minExtent = B2_HUGE();
	bodySim.maxExtent = 0.0;
	let shapeId = bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let extent = b2ComputeShapeExtent(world, shapeId, massData.center);
		bodySim.minExtent = b2MinFloat(bodySim.minExtent, extent.minExtent);
		bodySim.maxExtent = b2MaxFloat(bodySim.maxExtent, extent.maxExtent);
		shapeId = world.shapes[shapeId].nextShapeId;
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	b2InvalidateContactTransforms(world, body);

	// Motion locks take priority over mass data.
	if (bodies[body].flags & b2_fixedRotation) {
		bodies[body].inertia = 0.0;
		bodySim.invInertia = 0.0;
	}

	set.bodySims[localIndex] = bodySim;

	bodies[body].flags &= ~b2_dirtyMass;
}

export function b2Body_GetMassData(bodyId: b2BodyId): b2MassData {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let massData = b2MassData();
	massData.mass = world.bodies[body].mass;
	massData.center = world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].localCenter;
	massData.rotationalInertia = world.bodies[body].inertia;
	return massData;
}

export function b2Body_UpdateMassFromShapes(bodyId: b2BodyId): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	b2UpdateBodyMassData(world, body);
}

export function b2Body_SetLinearDamping(bodyId: b2BodyId, linearDamping: f32): void {
	if ((b2IsValidFloat(linearDamping) && linearDamping >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( linearDamping ) && linearDamping >= 0.0f in b2Body_SetLinearDamping\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].linearDamping = linearDamping;
}

export function b2Body_GetLinearDamping(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].linearDamping;
}

export function b2Body_SetAngularDamping(bodyId: b2BodyId, angularDamping: f32): void {
	if ((b2IsValidFloat(angularDamping) && angularDamping >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( angularDamping ) && angularDamping >= 0.0f in b2Body_SetAngularDamping\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].angularDamping = angularDamping;
}

export function b2Body_GetAngularDamping(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].angularDamping;
}

export function b2Body_SetGravityScale(bodyId: b2BodyId, gravityScale: f32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2Body_IsValid(bodyId));
	if (b2IsValidFloat(gravityScale) == false) {
		b2Log("invalid input: b2IsValidFloat( gravityScale ) in b2Body_SetGravityScale\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].gravityScale = gravityScale;
}

export function b2Body_GetGravityScale(bodyId: b2BodyId): f32 {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2Body_IsValid(bodyId));
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].gravityScale;
}

export function b2Body_IsAwake(bodyId: b2BodyId): bool {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].setIndex == b2_awakeSet;
}

export function b2Body_SetAwake(bodyId: b2BodyId, awake: bool): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let setIndex = world.bodies[body].setIndex;

	if (awake && setIndex >= b2_firstSleepingSet) {
		b2WakeBody(world, body);
	} else if (awake == false && setIndex == b2_awakeSet) {
		let islandId = world.bodies[body].islandId;
		let island = world.islands[islandId];
		if (island.constraintRemoveCount > 0) {
			// Must split the island before sleeping. This is expensive.
			b2SplitIsland(world, islandId);
		}

		b2TrySleepIsland(world, world.bodies[body].islandId);
	}
}

export function b2Body_WakeTouching(bodyId: b2BodyId): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let shapes = world.shapes;

	let contactKey = world.bodies[body].headContactKey;
	while (contactKey != B2_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let shapeIdA = world.contacts[contactId].shapeIdA;
		let shapeIdB = world.contacts[contactId].shapeIdB;

		if (shapes[shapeIdA].bodyId == bodyId.index1 - 1) {
			b2WakeBody(world, shapes[shapeIdB].bodyId);
		} else {
			b2WakeBody(world, shapes[shapeIdA].bodyId);
		}

		contactKey = world.contacts[contactId].edges[edgeIndex].nextKey;
	}
}

export function b2Body_IsEnabled(bodyId: b2BodyId): bool {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].setIndex != b2_disabledSet;
}

export function b2Body_IsSleepingEnabled(bodyId: b2BodyId): bool {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return (world.bodies[body].flags & b2_enableSleep) == b2_enableSleep;
}

export function b2Body_SetSleepThreshold(bodyId: b2BodyId, sleepThreshold: f32): void {
	if ((b2IsValidFloat(sleepThreshold) && sleepThreshold >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( sleepThreshold ) && sleepThreshold >= 0.0f in b2Body_SetSleepThreshold\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	world.bodies[body].sleepThreshold = sleepThreshold;
}

export function b2Body_GetSleepThreshold(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].sleepThreshold;
}

export function b2Body_SetSafetyFactor(bodyId: b2BodyId, safetyFactor: f32): void {
	if ((b2IsValidFloat(safetyFactor) && safetyFactor >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( safetyFactor ) && safetyFactor >= 0.0f in b2Body_SetSafetyFactor\n");
		B2_ASSERT(false);
		return;
	}
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	world.bodies[body].safetyFactor = safetyFactor;
}

export function b2Body_GetSafetyFactor(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].safetyFactor;
}

export function b2Body_EnableSleeping(bodyId: b2BodyId, enableSleep: bool): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let flags = world.bodies[body].flags;

	let flag = (flags & b2_enableSleep) == b2_enableSleep;
	if (enableSleep == flag) {
		return;
	}

	world.bodies[body].flags = enableSleep ? flags | b2_enableSleep : flags & ~b2_enableSleep;
	b2SyncBodyFlags(world, body);

	if (enableSleep == false) {
		b2WakeBody(world, body);
	}
}

// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
// The most challenging aspect is that joints may connect to bodies that are not disabled.
export function b2Body_Disable(bodyId: b2BodyId): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let bodies = world.bodies;
	let joints = world.joints;
	if (bodies[body].setIndex == b2_disabledSet) {
		return;
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	b2DestroyBodyContacts(world, body);

	// The current solver set of the body
	let set = world.solverSets[bodies[body].setIndex];

	// Disabled bodies and connected joints are moved to the disabled set
	let disabledSet = world.solverSets[b2_disabledSet];

	// Unlink joints and transfer them to the disabled set
	let jointKey = bodies[body].headJointKey;
	while (jointKey != B2_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointKey = joints[jointId].edges[edgeIndex].nextKey;

		// joint may already be disabled by other body
		if (joints[jointId].setIndex == b2_disabledSet) {
			continue;
		}

		if (B2_ASSERT_ENABLED) {
			let jointSetIndex = joints[jointId].setIndex;
			B2_ASSERT(jointSetIndex == set.setIndex || set.setIndex == b2_staticSet || jointSetIndex == b2_staticSet);
		}

		// Remove joint from island
		b2UnlinkJoint(world, jointId);

		// Transfer joint to disabled set
		let jointSet = world.solverSets[joints[jointId].setIndex];
		b2TransferJoint(world, disabledSet, jointSet, jointId);
	}

	// Remove shapes from broad-phase
	let shapeId = bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let currentId = shapeId;
		shapeId = world.shapes[shapeId].nextShapeId;
		b2DestroyShapeProxy(world, currentId);
	}

	// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
	b2RemoveBodyFromIsland(world, body);

	// Transfer body sim
	b2TransferBody(world, disabledSet, set, body);
}

export function b2Body_Enable(bodyId: b2BodyId): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let body = b2GetBodyFullId(world, bodyId);
	let bodies = world.bodies;
	let joints = world.joints;
	if (bodies[body].setIndex != b2_disabledSet) {
		return;
	}

	let disabledSet = world.solverSets[b2_disabledSet];
	let setId = bodies[body].type == b2_staticBody ? b2_staticSet : b2_awakeSet;
	let targetSet = world.solverSets[setId];

	b2TransferBody(world, targetSet, disabledSet, body);

	let transform = b2GetBodyTransformQuick(world, body);

	// Add shapes to broad-phase
	let proxyType = bodies[body].type;
	let forcePairCreation = true;
	let shapeId = bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let currentId = shapeId;
		shapeId = world.shapes[shapeId].nextShapeId;

		b2CreateShapeProxy(world, currentId, proxyType, transform, forcePairCreation);
	}

	if (setId != b2_staticSet) {
		b2CreateIslandForBody(world, setId, body);
	}

	// Transfer joints. If the other body is disabled, don't transfer.
	// If the other body is sleeping, wake it.
	let jointKey = bodies[body].headJointKey;
	while (jointKey != B2_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		if (B2_ASSERT_ENABLED) B2_ASSERT(joints[jointId].setIndex == b2_disabledSet);
		if (B2_ASSERT_ENABLED) B2_ASSERT(joints[jointId].islandId == B2_NULL_INDEX);

		jointKey = joints[jointId].edges[edgeIndex].nextKey;

		let bodyIdA = joints[jointId].edges[0].bodyId;
		let bodyIdB = joints[jointId].edges[1].bodyId;

		if (bodies[bodyIdA].setIndex == b2_disabledSet || bodies[bodyIdB].setIndex == b2_disabledSet) {
			// one body is still disabled
			continue;
		}

		// Transfer joint first
		let jointSetId: i32;
		if (bodies[bodyIdA].type != b2_dynamicBody && bodies[bodyIdB].type != b2_dynamicBody) {
			jointSetId = b2_staticSet;
		} else if (bodies[bodyIdA].setIndex == b2_staticSet) {
			jointSetId = bodies[bodyIdB].setIndex;
		} else {
			jointSetId = bodies[bodyIdA].setIndex;
		}

		let jointSet = world.solverSets[jointSetId];
		b2TransferJoint(world, jointSet, disabledSet, jointId);

		// Now that the joint is in the correct set, I can link the joint in the island.
		if (jointSetId != b2_staticSet) {
			b2LinkJoint(world, jointId);
		}
	}
}

export function b2Body_SetMotionLocks(bodyId: b2BodyId, locks: b2MotionLocks): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let newFlags: u32 = 0;
	newFlags |= locks.linearX ? b2_lockLinearX : 0;
	newFlags |= locks.linearY ? b2_lockLinearY : 0;
	newFlags |= locks.angularZ ? b2_lockAngularZ : 0;

	let body = b2GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b2_allLocks) != newFlags) {
		world.bodies[body].flags &= ~b2_allLocks;
		world.bodies[body].flags |= newFlags;

		b2SyncBodyFlags(world, body);

		let state = b2GetBodyState(world, body);

		if (state != B2_NULL_INDEX) {
			let states = world.solverSets[b2_awakeSet].bodyStates;
			if (locks.linearX) {
				states[state].linearVelocity.x = 0.0;
			}

			if (locks.linearY) {
				states[state].linearVelocity.y = 0.0;
			}

			if (locks.angularZ) {
				states[state].angularVelocity = 0.0;
			}
		}

		// Motion locks can affect mass properties.
		b2UpdateBodyMassData(world, body);
	}
}

export function b2Body_GetMotionLocks(bodyId: b2BodyId): b2MotionLocks {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let flags = world.bodies[body].flags;

	let locks = b2MotionLocks();
	locks.linearX = (flags & b2_lockLinearX) != 0;
	locks.linearY = (flags & b2_lockLinearY) != 0;
	locks.angularZ = (flags & b2_lockAngularZ) != 0;
	return locks;
}

export function b2Body_SetBullet(bodyId: b2BodyId, flag: bool): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let newFlag: u32 = flag ? b2_isBullet : 0;

	let body = b2GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b2_isBullet) == newFlag) {
		return;
	}

	world.bodies[body].flags &= ~b2_isBullet;
	world.bodies[body].flags |= newFlag;

	b2SyncBodyFlags(world, body);
}

export function b2Body_IsBullet(bodyId: b2BodyId): bool {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let flags = world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].flags;
	return (flags & b2_isBullet) != 0;
}

export function b2Body_EnableContactRecycling(bodyId: b2BodyId, flag: bool): void {
	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return;
	}

	let newFlag: u32 = flag ? b2_bodyEnableContactRecycling : 0;

	let body = b2GetBodyFullId(world, bodyId);
	if ((world.bodies[body].flags & b2_bodyEnableContactRecycling) == newFlag) {
		return;
	}

	world.bodies[body].flags &= ~b2_bodyEnableContactRecycling;
	world.bodies[body].flags |= newFlag;

	b2SyncBodyFlags(world, body);
}

export function b2Body_IsContactRecyclingEnabled(bodyId: b2BodyId): bool {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return (world.bodies[body].flags & b2_bodyEnableContactRecycling) != 0;
}

export function b2Body_EnableContactEvents(bodyId: b2BodyId, flag: bool): void {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let shapes = world.shapes;
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		shapes[shapeId].enableContactEvents = flag;
		shapeId = shapes[shapeId].nextShapeId;
	}
}

export function b2Body_EnableHitEvents(bodyId: b2BodyId, flag: bool): void {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let shapes = world.shapes;
	let shapeId = world.bodies[body].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		shapes[shapeId].enableHitEvents = flag;
		shapeId = shapes[shapeId].nextShapeId;
	}
}

export function b2Body_GetWorld(bodyId: b2BodyId): b2WorldId {
	let world = b2GetWorld(bodyId.world0);
	return b2WorldId(<u16>(bodyId.world0 + 1), world.generation);
}

export function b2Body_GetShapeCount(bodyId: b2BodyId): i32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].shapeCount;
}

export function b2Body_GetShapes(bodyId: b2BodyId, shapeArray: StaticArray<b2ShapeId>, capacity: i32): i32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let shapes = world.shapes;
	let shapeId = world.bodies[body].headShapeId;
	let shapeCount = 0;
	while (shapeId != B2_NULL_INDEX && shapeCount < capacity) {
		shapeArray[shapeCount] = b2ShapeId(shapes[shapeId].id + 1, bodyId.world0, shapes[shapeId].generation);
		shapeCount += 1;

		shapeId = shapes[shapeId].nextShapeId;
	}

	return shapeCount;
}

export function b2Body_GetJointCount(bodyId: b2BodyId): i32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.bodies[body].jointCount;
}

export function b2Body_GetJoints(bodyId: b2BodyId, jointArray: StaticArray<b2JointId>, capacity: i32): i32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let jointKey = world.bodies[body].headJointKey;

	let jointCount = 0;
	while (jointKey != B2_NULL_INDEX && jointCount < capacity) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;

		jointArray[jointCount] = b2JointId(jointId + 1, bodyId.world0, world.joints[jointId].generation);
		jointCount += 1;

		jointKey = world.joints[jointId].edges[edgeIndex].nextKey;
	}

	return jointCount;
}

export unsafe function b2ShouldBodiesCollide(world: b2World, bodyIdA: i32, bodyIdB: i32): bool {
	let bodies = b2ArrayView(world.bodies);
	ref bodyA = bodies[bodyIdA];
	ref bodyB = bodies[bodyIdB];
	if (bodyA.type != b2_dynamicBody && bodyB.type != b2_dynamicBody) {
		return false;
	}

	let jointKey: i32;
	let otherBodyId: i32;
	if (bodyA.jointCount < bodyB.jointCount) {
		jointKey = bodyA.headJointKey;
		otherBodyId = bodyB.id;
	} else {
		jointKey = bodyB.headJointKey;
		otherBodyId = bodyA.id;
	}

	let joints = b2ArrayView(world.joints);
	while (jointKey != B2_NULL_INDEX) {
		let jointId = jointKey >> 1;
		let edgeIndex = jointKey & 1;
		let otherEdgeIndex = edgeIndex ^ 1;

		ref joint = joints[jointId];
		if (joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId) {
			return false;
		}

		jointKey = joint.edges[edgeIndex].nextKey;
	}

	return true;
}

export function b2Body_GetMinExtent(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].minExtent;
}

export function b2Body_GetMaxExtent(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	return world.solverSets[world.bodies[body].setIndex].bodySims[world.bodies[body].localIndex].maxExtent;
}

export function b2Body_GetMaxExtentFromOrigin(bodyId: b2BodyId): f32 {
	let world = b2GetWorld(bodyId.world0);
	let body = b2GetBodyFullId(world, bodyId);
	let set = world.solverSets[world.bodies[body].setIndex];
	let localIndex = world.bodies[body].localIndex;
	return set.bodySims[localIndex].maxExtent + b2Length(set.bodySims[localIndex].localCenter);
}

function b2MakeContactIdFrom(index1: i32, world0: u16, generation: u32): b2ContactId {
	let id = b2ContactId();
	id.index1 = index1;
	id.world0 = world0;
	id.padding = 0;
	id.generation = generation;
	return id;
}
