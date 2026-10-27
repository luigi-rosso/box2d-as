// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact.c

import { b2MaxFloat, b2Rot, b2Transform } from "./math_functions";
import { b2Manifold, b2SimplexCache } from "./collision";

export type b2ContactFlags = u32;

// Set when the solid shapes are touching.
export const b2_contactTouchingFlag: b2ContactFlags = 0x00000001;

// Contact has a hit event
export const b2_contactHitEventFlag: b2ContactFlags = 0x00000002;

// This contact wants contact events
export const b2_contactEnableContactEvents: b2ContactFlags = 0x00000004;

export const b2_contactRecycleFlag: b2ContactFlags = 0x00000008;

// Set when the shapes are touching
export const b2_simTouchingFlag: b2ContactFlags = 0x00010000;

// This contact no longer has overlapping AABBs
export const b2_simDisjoint: b2ContactFlags = 0x00020000;

// This contact started touching
export const b2_simStartedTouching: b2ContactFlags = 0x00040000;

// This contact stopped touching
export const b2_simStoppedTouching: b2ContactFlags = 0x00080000;

// This contact has a hit event
export const b2_simEnableHitEvent: b2ContactFlags = 0x00100000;

// This contact wants pre-solve events
export const b2_simEnablePreSolveEvents: b2ContactFlags = 0x00200000;

// This contact has a cached relative transform
export const b2_simRelativeTransformValid: b2ContactFlags = 0x00400000;

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
export struct b2ContactEdge {
	bodyId: i32 = 0;
	prevKey: i32 = 0;
	nextKey: i32 = 0;
}

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
export struct b2Contact {
	edges: FixedArray<b2ContactEdge, 2>;

	// A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
	islandId: i32 = 0;

	// Index into the island's contacts array for O(1) swap-removal.
	// B2_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	setIndex: i32 = 0;

	// index into the constraint graph color array
	// B2_NULL_INDEX for non-touching or sleeping contacts
	// B2_NULL_INDEX when slot is free
	colorIndex: i32 = 0;

	// contact index within set or graph color
	// B2_NULL_INDEX when slot is free
	localIndex: i32 = 0;

	shapeIdA: i32 = 0;
	shapeIdB: i32 = 0;
	contactId: i32 = 0;

	// b2ContactFlags
	flags: u32 = 0;

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b2ContactId
	generation: u32 = 0;
}

// The class manages contact between two shapes. A contact exists for each overlapping
// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
// that has no contact points.
export struct b2ContactSim {
	contactId: i32 = 0;

	// Cache for contact recycling.
	cachedRotationA: b2Rot = b2Rot();
	cachedRotationB: b2Rot = b2Rot();
	cachedRelativePose: b2Transform = b2Transform();

	encodedBodySimA: i32 = 0;
	encodedBodySimB: i32 = 0;

	// b2ContactFlags
	simFlags: u32 = 0;

	shapeIdA: i32 = 0;
	shapeIdB: i32 = 0;

	invMassA: f32 = 0;
	invIA: f32 = 0;

	invMassB: f32 = 0;
	invIB: f32 = 0;

	manifold: b2Manifold = b2Manifold();

	// Mixed friction and restitution
	friction: f32 = 0;
	restitution: f32 = 0;
	rollingResistance: f32 = 0;
	tangentSpeed: f32 = 0;

	cache: b2SimplexCache = b2SimplexCache();
}

@inline export function b2MixFriction(frictionA: f32, frictionB: f32): f32 {
	return Mathf.sqrt(frictionA * frictionB);
}

@inline export function b2MixRestitution(restitutionA: f32, restitutionB: f32): f32 {
	return b2MaxFloat(restitutionA, restitutionB);
}

// functions from contact.c follow
