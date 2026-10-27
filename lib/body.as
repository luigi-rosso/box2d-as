// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/body.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import { b2Pos, b2Rot, b2SubPos, b2Transform, b2Vec2, b2WorldTransform } from "./math_functions";
import { b2Sweep } from "./collision";
import { b2BodyType } from "./types";
import { b2World } from "./physics_world";
import { b2_awakeSet, b2_staticSet } from "./solver_set";

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

function b2MakeIdentityStates(): StaticArray<b2BodyState> {
	let states = new StaticArray<b2BodyState>(1);
	states[0] = b2_identityBodyState;
	return states;
}

// b2_identityBodyState in memory, where C points static lanes and joint ends
export const b2_identityStates = b2MakeIdentityStates();

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

// Encode the body sim index for storage in the contact.
export function b2EncodeBodySimIndex(body: b2Body): i32 {
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
