// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact.c

import { B2_ASSERT, B2_NULL_INDEX } from "./core";
import { B2_GRAPH_COLOR_COUNT } from "./constants";
import {
	b2Add,
	b2InvMulWorldTransforms,
	b2MaxFloat,
	b2RotateVector,
	b2Rot,
	b2Sub,
	b2SubPos,
	b2Transform,
	b2Vec2,
	b2WorldTransform,
} from "./math_functions";
import { b2LocalManifold, b2Manifold, b2SimplexCache, b2_emptySimplexCache } from "./collision";
import {
	b2CollideCapsuleAndCircle,
	b2CollideCapsules,
	b2CollideChainSegmentAndCapsule,
	b2CollideChainSegmentAndCircle,
	b2CollideChainSegmentAndPolygon,
	b2CollideCircles,
	b2CollidePolygonAndCapsule,
	b2CollidePolygonAndCircle,
	b2CollidePolygons,
	b2CollideSegmentAndCapsule,
	b2CollideSegmentAndCircle,
	b2CollideSegmentAndPolygon,
} from "./manifold";
import { b2ContactId, b2ShapeId } from "./id";
import { B2_SHAPE_PAIR_KEY, b2AddKey, b2RemoveKey } from "./table";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2Array_RemoveSwap } from "./container";
import {
	b2ContactData,
	b2ContactEndTouchEvent,
	b2PreSolveManifold,
	b2ShapeType,
	b2_capsuleShape,
	b2_chainSegmentShape,
	b2_circleShape,
	b2_polygonShape,
	b2_segmentShape,
	b2_shapeTypeCount,
} from "./types";
import { b2ArrayData, b2EncodeBodySimIndex, b2WakeBody, b2_bodyEnableContactRecycling } from "./body";
import { b2GetShapeRadius } from "./shape";
import { b2UnlinkContact } from "./island";
import { b2GetWorld, b2World } from "./physics_world";
import { b2_awakeSet, b2_disabledSet, b2_staticSet } from "./solver_set";
import { b2RemoveContactFromGraph } from "./constraint_graph";

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

// C points into a growable Array of contact sims; this views the Array's buffer as a StaticArray so an element
// binds by ref in place, valid until the array grows
@inline export unsafe function b2ContactSimView(sims: Array<b2ContactSim>): StaticArray<b2ContactSim> {
	return changetype<StaticArray<b2ContactSim>>(b2ArrayData(sims));
}

@inline export function b2MixFriction(frictionA: f32, frictionB: f32): f32 {
	return Mathf.sqrt(frictionA * frictionB);
}

@inline export function b2MixRestitution(restitutionA: f32, restitutionB: f32): f32 {
	return b2MaxFloat(restitutionA, restitutionB);
}

// functions from contact.c follow

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b2Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b2UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Manifold functions should compute important results in local space to improve precision. However, this
// interface function takes two world transforms instead of a relative transform for these reasons:
//
// First:
// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
// solver does not need to access static body transforms. Not even in constraint preparation. This approach
// has world space vectors yet retains precision.
//
// Second:
// b2ManifoldPoint::point is very useful for debugging and it is in world space.
//
// Third:
// The user may call the manifold functions directly and they should be easy to use and have easy to use
// results.

function b2GetContactFullId(world: b2World, contactId: b2ContactId): i32 {
	let id = contactId.index1 - 1;
	B2_ASSERT(world.contacts[id].contactId == id && world.contacts[id].generation == contactId.generation);
	return id;
}

export function b2Contact_GetData(contactId: b2ContactId): b2ContactData {
	let world = b2GetWorld(contactId.world0);
	let id = b2GetContactFullId(world, contactId);
	let shapeIdA = world.contacts[id].shapeIdA;
	let shapeIdB = world.contacts[id].shapeIdB;

	let data = b2ContactData();
	data.contactId = contactId;
	data.shapeIdA.index1 = world.shapes[shapeIdA].id + 1;
	data.shapeIdA.world0 = contactId.world0;
	data.shapeIdA.generation = world.shapes[shapeIdA].generation;
	data.shapeIdB.index1 = world.shapes[shapeIdB].id + 1;
	data.shapeIdB.world0 = contactId.world0;
	data.shapeIdB.generation = world.shapes[shapeIdB].generation;
	data.manifold = b2GetContactSimArray(world, id)[world.contacts[id].localIndex].manifold;

	return data;
}

// C's b2_contactRegistry: whether a manifold function exists for the pair and whether the first type is primary
function b2MakeRegistry(primary: bool): StaticArray<bool> {
	let r = new StaticArray<bool>(b2_shapeTypeCount * b2_shapeTypeCount);
	let n = b2_shapeTypeCount;
	r[b2_circleShape * n + b2_circleShape] = true;
	r[b2_capsuleShape * n + b2_circleShape] = true;
	r[b2_capsuleShape * n + b2_capsuleShape] = true;
	r[b2_segmentShape * n + b2_circleShape] = true;
	r[b2_segmentShape * n + b2_capsuleShape] = true;
	r[b2_segmentShape * n + b2_polygonShape] = true;
	r[b2_polygonShape * n + b2_circleShape] = true;
	r[b2_polygonShape * n + b2_capsuleShape] = true;
	r[b2_polygonShape * n + b2_polygonShape] = true;
	r[b2_chainSegmentShape * n + b2_circleShape] = true;
	r[b2_chainSegmentShape * n + b2_capsuleShape] = true;
	r[b2_chainSegmentShape * n + b2_polygonShape] = true;
	if (primary == false) {
		// The flipped entries have a function but are not primary
		r[b2_circleShape * n + b2_capsuleShape] = true;
		r[b2_circleShape * n + b2_segmentShape] = true;
		r[b2_circleShape * n + b2_polygonShape] = true;
		r[b2_circleShape * n + b2_chainSegmentShape] = true;
		r[b2_capsuleShape * n + b2_segmentShape] = true;
		r[b2_capsuleShape * n + b2_polygonShape] = true;
		r[b2_capsuleShape * n + b2_chainSegmentShape] = true;
		r[b2_polygonShape * n + b2_segmentShape] = true;
		r[b2_polygonShape * n + b2_chainSegmentShape] = true;
	}
	return r;
}

const b2_contactRegistryHasFcn = b2MakeRegistry(false);
const b2_contactRegistryPrimary = b2MakeRegistry(true);

// The primary manifold function of the registry for shapeA's type against shapeB's type
function b2ComputeLocalManifold(world: b2World, shapeIdA: i32, shapeIdB: i32, xf: b2Transform, cache: ref b2SimplexCache): b2LocalManifold {
	let shapes = world.shapes;
	let typeA = shapes[shapeIdA].type;
	let typeB = shapes[shapeIdB].type;
	switch (typeA) {
		case b2_circleShape:
			return b2CollideCircles(shapes[shapeIdA].circle, shapes[shapeIdB].circle, xf);
		case b2_capsuleShape:
			if (typeB == b2_circleShape) {
				return b2CollideCapsuleAndCircle(shapes[shapeIdA].capsule, shapes[shapeIdB].circle, xf);
			}
			return b2CollideCapsules(shapes[shapeIdA].capsule, shapes[shapeIdB].capsule, xf);
		case b2_segmentShape:
			if (typeB == b2_circleShape) {
				return b2CollideSegmentAndCircle(shapes[shapeIdA].segment, shapes[shapeIdB].circle, xf);
			}
			if (typeB == b2_capsuleShape) {
				return b2CollideSegmentAndCapsule(shapes[shapeIdA].segment, shapes[shapeIdB].capsule, xf);
			}
			return b2CollideSegmentAndPolygon(shapes[shapeIdA].segment, shapes[shapeIdB].polygon, xf);
		case b2_polygonShape:
			if (typeB == b2_circleShape) {
				return b2CollidePolygonAndCircle(shapes[shapeIdA].polygon, shapes[shapeIdB].circle, xf);
			}
			if (typeB == b2_capsuleShape) {
				return b2CollidePolygonAndCapsule(shapes[shapeIdA].polygon, shapes[shapeIdB].capsule, xf);
			}
			return b2CollidePolygons(shapes[shapeIdA].polygon, shapes[shapeIdB].polygon, xf);
		default:
			B2_ASSERT(typeA == b2_chainSegmentShape);
			if (typeB == b2_circleShape) {
				return b2CollideChainSegmentAndCircle(shapes[shapeIdA].chainSegment, shapes[shapeIdB].circle, xf);
			}
			if (typeB == b2_capsuleShape) {
				return b2CollideChainSegmentAndCapsule(shapes[shapeIdA].chainSegment, shapes[shapeIdB].capsule, xf, cache);
			}
			return b2CollideChainSegmentAndPolygon(shapes[shapeIdA].chainSegment, shapes[shapeIdB].polygon, xf, cache);
	}
}

export function b2CanCollide(typeA: b2ShapeType, typeB: b2ShapeType): bool {
	return b2_contactRegistryHasFcn[typeA * b2_shapeTypeCount + typeB];
}

function b2MixFrictionFor(world: b2World, shapeIdA: i32, shapeIdB: i32): f32 {
	let shapes = world.shapes;
	let frictionCallback = world.frictionCallback;
	if (frictionCallback == null) {
		return b2MixFriction(shapes[shapeIdA].material.friction, shapes[shapeIdB].material.friction);
	}

	return frictionCallback(
		shapes[shapeIdA].material.friction,
		shapes[shapeIdA].material.userMaterialId,
		shapes[shapeIdB].material.friction,
		shapes[shapeIdB].material.userMaterialId,
	);
}

function b2MixRestitutionFor(world: b2World, shapeIdA: i32, shapeIdB: i32): f32 {
	let shapes = world.shapes;
	let restitutionCallback = world.restitutionCallback;
	if (restitutionCallback == null) {
		return b2MixRestitution(shapes[shapeIdA].material.restitution, shapes[shapeIdB].material.restitution);
	}

	return restitutionCallback(
		shapes[shapeIdA].material.restitution,
		shapes[shapeIdA].material.userMaterialId,
		shapes[shapeIdB].material.restitution,
		shapes[shapeIdB].material.userMaterialId,
	);
}

// WARNING: this should never fail to create a contact because the pair already exists in the pairSet.
export function b2CreateContact(world: b2World, shapeIdA: i32, shapeIdB: i32): void {
	let type1 = world.shapes[shapeIdA].type;
	let type2 = world.shapes[shapeIdB].type;

	B2_ASSERT(0 <= type1 && type1 < b2_shapeTypeCount);
	B2_ASSERT(0 <= type2 && type2 < b2_shapeTypeCount);

	if (b2_contactRegistryHasFcn[type1 * b2_shapeTypeCount + type2] == false) {
		// For example, no segment vs segment collision
		return;
	}

	if (b2_contactRegistryPrimary[type1 * b2_shapeTypeCount + type2] == false) {
		// flip order
		b2CreateContact(world, shapeIdB, shapeIdA);
		return;
	}

	let bodyIdA = world.shapes[shapeIdA].bodyId;
	let bodyIdB = world.shapes[shapeIdB].bodyId;

	B2_ASSERT(world.bodies[bodyIdA].setIndex != b2_disabledSet && world.bodies[bodyIdB].setIndex != b2_disabledSet);
	B2_ASSERT(world.bodies[bodyIdA].setIndex != b2_staticSet || world.bodies[bodyIdB].setIndex != b2_staticSet);

	let setIndex: i32;
	if (world.bodies[bodyIdA].setIndex == b2_awakeSet || world.bodies[bodyIdB].setIndex == b2_awakeSet) {
		setIndex = b2_awakeSet;
	} else {
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island
		setIndex = b2_disabledSet;
	}

	let set = world.solverSets[setIndex];

	// Create contact key and contact
	let contactId = b2AllocId(world.contactIdPool);
	if (contactId == world.contacts.length) {
		world.contacts.push(b2Contact());
	}

	let contact = world.contacts[contactId];
	contact.contactId = contactId;
	contact.generation += 1;
	contact.setIndex = setIndex;
	contact.colorIndex = B2_NULL_INDEX;
	contact.localIndex = set.contactSims.length;
	contact.islandId = B2_NULL_INDEX;
	contact.islandIndex = B2_NULL_INDEX;
	contact.shapeIdA = shapeIdA;
	contact.shapeIdB = shapeIdB;
	contact.flags = 0;

	// Both bodies must enable recycling
	if (
		(world.bodies[bodyIdA].flags & b2_bodyEnableContactRecycling) != 0 &&
		(world.bodies[bodyIdB].flags & b2_bodyEnableContactRecycling) != 0
	) {
		contact.flags |= b2_contactRecycleFlag;
	}

	B2_ASSERT(world.shapes[shapeIdA].sensorIndex == B2_NULL_INDEX && world.shapes[shapeIdB].sensorIndex == B2_NULL_INDEX);

	if (world.shapes[shapeIdA].enableContactEvents || world.shapes[shapeIdB].enableContactEvents) {
		contact.flags |= b2_contactEnableContactEvents;
	}

	// Connect to body A
	{
		contact.edges[0].bodyId = bodyIdA;
		contact.edges[0].prevKey = B2_NULL_INDEX;
		contact.edges[0].nextKey = world.bodies[bodyIdA].headContactKey;

		let keyA = (contactId << 1) | 0;
		let headContactKey = world.bodies[bodyIdA].headContactKey;
		if (headContactKey != B2_NULL_INDEX) {
			world.contacts[headContactKey >> 1].edges[headContactKey & 1].prevKey = keyA;
		}
		world.bodies[bodyIdA].headContactKey = keyA;
		world.bodies[bodyIdA].contactCount += 1;
	}

	// Connect to body B
	{
		contact.edges[1].bodyId = bodyIdB;
		contact.edges[1].prevKey = B2_NULL_INDEX;
		contact.edges[1].nextKey = world.bodies[bodyIdB].headContactKey;

		let keyB = (contactId << 1) | 1;
		let headContactKey = world.bodies[bodyIdB].headContactKey;
		if (headContactKey != B2_NULL_INDEX) {
			world.contacts[headContactKey >> 1].edges[headContactKey & 1].prevKey = keyB;
		}
		world.bodies[bodyIdB].headContactKey = keyB;
		world.bodies[bodyIdB].contactCount += 1;
	}

	world.contacts[contactId] = contact;

	// Add to pair set for fast lookup.
	let pairKey = B2_SHAPE_PAIR_KEY(shapeIdA, shapeIdB);
	b2AddKey(world.broadPhase.pairSet, pairKey);

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	let contactSim = b2ContactSim();
	contactSim.contactId = contactId;

	contactSim.encodedBodySimA = b2EncodeBodySimIndex(world.bodies[bodyIdA]);
	contactSim.encodedBodySimB = b2EncodeBodySimIndex(world.bodies[bodyIdB]);
	contactSim.invMassA = 0.0;
	contactSim.invIA = 0.0;
	contactSim.invMassB = 0.0;
	contactSim.invIB = 0.0;
	contactSim.shapeIdA = shapeIdA;
	contactSim.shapeIdB = shapeIdB;
	contactSim.cache = b2_emptySimplexCache;
	contactSim.manifold = b2Manifold();

	// These get updated in the narrow phase, but these are needed for first touch
	contactSim.friction = b2MixFrictionFor(world, shapeIdA, shapeIdB);
	contactSim.restitution = b2MixRestitutionFor(world, shapeIdA, shapeIdB);

	contactSim.tangentSpeed = 0.0;
	contactSim.simFlags = contact.flags;

	if (world.shapes[shapeIdA].enablePreSolveEvents || world.shapes[shapeIdB].enablePreSolveEvents) {
		contactSim.simFlags |= b2_simEnablePreSolveEvents;
	}

	set.contactSims.push(contactSim);
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
export function b2DestroyContact(world: b2World, contactId: i32): void {
	let contact = world.contacts[contactId];

	// Remove pair from set
	let pairKey = B2_SHAPE_PAIR_KEY(contact.shapeIdA, contact.shapeIdB);
	b2RemoveKey(world.broadPhase.pairSet, pairKey);

	let edgeA = contact.edges[0];
	let edgeB = contact.edges[1];

	let bodyIdA = edgeA.bodyId;
	let bodyIdB = edgeB.bodyId;

	let flags = contact.flags;
	let touching = (flags & b2_contactTouchingFlag) != 0;

	// End touch event
	if (touching && (flags & b2_contactEnableContactEvents) != 0) {
		let worldId = world.worldId;
		let event = b2ContactEndTouchEvent();
		event.shapeIdA.index1 = world.shapes[contact.shapeIdA].id + 1;
		event.shapeIdA.world0 = worldId;
		event.shapeIdA.generation = world.shapes[contact.shapeIdA].generation;
		event.shapeIdB.index1 = world.shapes[contact.shapeIdB].id + 1;
		event.shapeIdB.world0 = worldId;
		event.shapeIdB.generation = world.shapes[contact.shapeIdB].generation;
		event.contactId.index1 = contact.contactId + 1;
		event.contactId.world0 = world.worldId;
		event.contactId.padding = 0;
		event.contactId.generation = contact.generation;

		world.contactEndEvents[world.endEventArrayIndex].push(event);
	}

	// Remove from body A
	if (edgeA.prevKey != B2_NULL_INDEX) {
		world.contacts[edgeA.prevKey >> 1].edges[edgeA.prevKey & 1].nextKey = edgeA.nextKey;
	}

	if (edgeA.nextKey != B2_NULL_INDEX) {
		world.contacts[edgeA.nextKey >> 1].edges[edgeA.nextKey & 1].prevKey = edgeA.prevKey;
	}

	let edgeKeyA = (contactId << 1) | 0;
	if (world.bodies[bodyIdA].headContactKey == edgeKeyA) {
		world.bodies[bodyIdA].headContactKey = edgeA.nextKey;
	}

	world.bodies[bodyIdA].contactCount -= 1;

	// Remove from body B
	if (edgeB.prevKey != B2_NULL_INDEX) {
		world.contacts[edgeB.prevKey >> 1].edges[edgeB.prevKey & 1].nextKey = edgeB.nextKey;
	}

	if (edgeB.nextKey != B2_NULL_INDEX) {
		world.contacts[edgeB.nextKey >> 1].edges[edgeB.nextKey & 1].prevKey = edgeB.prevKey;
	}

	let edgeKeyB = (contactId << 1) | 1;
	if (world.bodies[bodyIdB].headContactKey == edgeKeyB) {
		world.bodies[bodyIdB].headContactKey = edgeB.nextKey;
	}

	world.bodies[bodyIdB].contactCount -= 1;

	// Remove contact from island.
	if (world.contacts[contactId].islandId != B2_NULL_INDEX) {
		b2UnlinkContact(world, contactId);
	}

	let colorIndex = world.contacts[contactId].colorIndex;
	let localIndex = world.contacts[contactId].localIndex;
	let setIndex = world.contacts[contactId].setIndex;
	if (colorIndex != B2_NULL_INDEX) {
		// contact is an active constraint
		B2_ASSERT(setIndex == b2_awakeSet);
		b2RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex);
	} else {
		// contact is non-touching or is sleeping
		B2_ASSERT(setIndex != b2_awakeSet || (world.contacts[contactId].flags & b2_contactTouchingFlag) == 0);
		let set = world.solverSets[setIndex];

		let movedIndex = b2Array_RemoveSwap(set.contactSims, localIndex);
		if (movedIndex != B2_NULL_INDEX) {
			let movedContactId = set.contactSims[localIndex].contactId;
			world.contacts[movedContactId].localIndex = localIndex;
		}
	}

	// Free contact and id (preserve generation)
	world.contacts[contactId].contactId = B2_NULL_INDEX;
	world.contacts[contactId].setIndex = B2_NULL_INDEX;
	world.contacts[contactId].colorIndex = B2_NULL_INDEX;
	world.contacts[contactId].localIndex = B2_NULL_INDEX;
	b2FreeId(world.contactIdPool, contactId);

	if (touching) {
		b2WakeBody(world, bodyIdA);
		b2WakeBody(world, bodyIdB);
	}
}

// The array that holds the contact's sim, at world.contacts[contactId].localIndex. C returns the sim pointer.
export function b2GetContactSimArray(world: b2World, contactId: i32): Array<b2ContactSim> {
	let setIndex = world.contacts[contactId].setIndex;
	let colorIndex = world.contacts[contactId].colorIndex;
	if (setIndex == b2_awakeSet && colorIndex != B2_NULL_INDEX) {
		// contact lives in constraint graph
		B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
		return world.constraintGraph.colors[colorIndex].contactSims;
	}

	return world.solverSets[setIndex].contactSims;
}

// Returns a copy
export function b2GetContactSim(world: b2World, contactId: i32): b2ContactSim {
	return b2GetContactSimArray(world, contactId)[world.contacts[contactId].localIndex];
}

const b2_preSolveManifold = new b2PreSolveManifold();

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
export function b2UpdateContact(
	world: b2World,
	contactSim: ref b2ContactSim,
	shapeIdA: i32,
	transformA: b2WorldTransform,
	centerOffsetA: b2Vec2,
	shapeIdB: i32,
	transformB: b2WorldTransform,
	centerOffsetB: b2Vec2,
): bool {
	// Save old manifold
	let oldManifold = contactSim.manifold;

	// Compute the manifold in frame A, then marshal it to world anchors relative to each shape origin.
	// The relative pose differences the two world positions before the narrow phase runs in frame A,
	// so precision is retained far from the origin.
	// anchorB = worldPoint - pB = rot(qA, localAnchorA) + pA - pB = anchorA + (pA - pB)
	let relativeTransform = b2InvMulWorldTransforms(transformA, transformB);
	let local = b2ComputeLocalManifold(world, shapeIdA, shapeIdB, relativeTransform, contactSim.cache);

	contactSim.manifold = b2Manifold();
	contactSim.manifold.normal = b2RotateVector(transformA.q, local.normal);
	contactSim.manifold.pointCount = local.pointCount;

	let originDelta = b2SubPos(transformA.p, transformB.p);
	for (let i = 0; i < local.pointCount; ++i) {
		ref mp = contactSim.manifold.points[i];
		mp.anchorA = b2RotateVector(transformA.q, local.points[i].point);
		mp.anchorB = b2Add(mp.anchorA, originDelta);
		mp.separation = local.points[i].separation;
		mp.id = local.points[i].id;
	}

	let shapes = world.shapes;

	// Keep these updated in case the values on the shapes are modified
	contactSim.friction = b2MixFrictionFor(world, shapeIdA, shapeIdB);
	contactSim.restitution = b2MixRestitutionFor(world, shapeIdA, shapeIdB);

	let rollingA = shapes[shapeIdA].material.rollingResistance;
	let rollingB = shapes[shapeIdB].material.rollingResistance;
	if (rollingA > 0.0 || rollingB > 0.0) {
		let radiusA = b2GetShapeRadius(shapes[shapeIdA]);
		let radiusB = b2GetShapeRadius(shapes[shapeIdB]);
		let maxRadius = b2MaxFloat(radiusA, radiusB);
		contactSim.rollingResistance = b2MaxFloat(rollingA, rollingB) * maxRadius;
	} else {
		contactSim.rollingResistance = 0.0;
	}

	contactSim.tangentSpeed = shapes[shapeIdA].material.tangentSpeed + shapes[shapeIdB].material.tangentSpeed;

	let pointCount = contactSim.manifold.pointCount;
	let touching = pointCount > 0;

	let preSolveFcn = world.preSolveFcn;
	if (touching && preSolveFcn != null && (contactSim.simFlags & b2_simEnablePreSolveEvents) != 0) {
		let idA = b2ShapeId();
		idA.index1 = shapes[shapeIdA].id + 1;
		idA.world0 = world.worldId;
		idA.generation = shapes[shapeIdA].generation;
		let idB = b2ShapeId();
		idB.index1 = shapes[shapeIdB].id + 1;
		idB.world0 = world.worldId;
		idB.generation = shapes[shapeIdB].generation;

		// This call assumes thread safety.
		let holder = b2_preSolveManifold;
		holder.manifold = contactSim.manifold;
		preSolveFcn(idA, idB, holder);
		contactSim.manifold = holder.manifold;

		// Keep these current.
		pointCount = contactSim.manifold.pointCount;
		touching = pointCount > 0;

		// Mark recycling dirty.
		if (touching == false) {
			contactSim.simFlags &= ~b2_simRelativeTransformValid;
		}
	}

	if (touching && (shapes[shapeIdA].enableHitEvents || shapes[shapeIdB].enableHitEvents)) {
		contactSim.simFlags |= b2_simEnableHitEvent;
	} else {
		contactSim.simFlags &= ~b2_simEnableHitEvent;
	}

	if (pointCount > 0) {
		contactSim.manifold.rollingImpulse = oldManifold.rollingImpulse;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (let i = 0; i < pointCount; ++i) {
			ref mp2 = contactSim.manifold.points[i];

			// shift anchors to be center of mass relative
			mp2.anchorA = b2Sub(mp2.anchorA, centerOffsetA);
			mp2.anchorB = b2Sub(mp2.anchorB, centerOffsetB);

			mp2.normalImpulse = 0.0;
			mp2.tangentImpulse = 0.0;
			mp2.totalNormalImpulse = 0.0;
			mp2.normalVelocity = 0.0;
			mp2.persisted = false;

			let id2 = mp2.id;

			for (let j = 0; j < oldManifold.pointCount; ++j) {
				ref mp1 = oldManifold.points[j];

				if (mp1.id == id2) {
					mp2.normalImpulse = mp1.normalImpulse;
					mp2.tangentImpulse = mp1.tangentImpulse;
					mp2.persisted = true;

					break;
				}
			}
		}
	}

	if (touching) {
		contactSim.simFlags |= b2_simTouchingFlag;
	} else {
		contactSim.simFlags &= ~b2_simTouchingFlag;
	}

	return touching;
}
