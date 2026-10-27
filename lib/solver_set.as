// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver_set.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import { B2_GRAPH_COLOR_COUNT } from "./constants";
import { b2ClearBit } from "./bitset";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2Array_RemoveSwap } from "./container";
import {
	b2BodySim,
	b2BodyState,
	b2RefreshBodyContactIndices,
	b2SleepBodySimIndex,
	b2_bodyTransientFlags,
	b2_identityBodyState,
	b2_isFast,
} from "./body";
import { b2ContactSim, b2ContactSimView, b2_contactTouchingFlag, b2_simTouchingFlag } from "./contact";
import { b2IslandSim } from "./island";
import { b2JointSim } from "./joint";
import { b2World } from "./physics_world";
import { B2_OVERFLOW_INDEX, b2AddContactToGraph, b2AddJointToGraph, b2RemoveJointFromGraph } from "./constraint_graph";

// The solver set type by index
export type b2SolverSetType = i32;

// Static set for static bodies and joints between static bodies
export const b2_staticSet: b2SolverSetType = 0;

// Disabled set for disabled bodies and their joints
export const b2_disabledSet: b2SolverSetType = 1;

// Awake set for awake bodies and awake non-touching contacts. Awake touching contacts
// and awake joints live in the constraint graph
export const b2_awakeSet: b2SolverSetType = 2;

// The index of the first sleeping set. Each island that goes to sleep is put into
// a sleeping set. This holds all bodies, contacts, and joints from the sleeping island.
// A separate set for each sleeping island makes it very efficient to wake a single island.
export const b2_firstSleepingSet: b2SolverSetType = 3;

// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
export class b2SolverSet {
	// Body array. Empty for unused set.
	bodySims: Array<b2BodySim> = new Array<b2BodySim>();

	// Body state only exists for active set
	bodyStates: Array<b2BodyState> = new Array<b2BodyState>();

	// This holds sleeping/disabled joints. Empty for static/active set.
	jointSims: Array<b2JointSim> = new Array<b2JointSim>();

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	contactSims: Array<b2ContactSim> = new Array<b2ContactSim>();

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	islandSims: Array<b2IslandSim> = new Array<b2IslandSim>();

	// Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	setIndex: i32 = 0;
}

// functions from solver_set.c follow

export function b2DestroySolverSet(world: b2World, setIndex: i32): void {
	let set = world.solverSets[setIndex];

	// C frees the arrays; the emptied ones are kept for the next set in this slot
	set.bodySims.length = 0;
	set.bodyStates.length = 0;
	set.contactSims.length = 0;
	set.jointSims.length = 0;
	set.islandSims.length = 0;
	b2FreeId(world.solverSetIdPool, setIndex);
	set.setIndex = B2_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
export function b2WakeSolverSet(world: b2World, setIndex: i32): void {
	B2_ASSERT(setIndex >= b2_firstSleepingSet);
	let set = world.solverSets[setIndex];
	let awakeSet = world.solverSets[b2_awakeSet];
	let disabledSet = world.solverSets[b2_disabledSet];

	let bodyCount = set.bodySims.length;
	for (let i = 0; i < bodyCount; ++i) {
		let simSrc = set.bodySims[i];

		let bodyId = simSrc.bodyId;
		B2_ASSERT(world.bodies[bodyId].setIndex == setIndex);
		world.bodies[bodyId].setIndex = b2_awakeSet;
		world.bodies[bodyId].localIndex = awakeSet.bodySims.length;

		// Reset sleep timer
		world.bodies[bodyId].sleepTime = 0.0;

		awakeSet.bodySims.push(simSrc);

		let state = b2_identityBodyState;
		state.flags = world.bodies[bodyId].flags;
		awakeSet.bodyStates.push(state);

		// Move non-touching contacts from disabled set to awake set.
		let encodedBodySimIndex = world.bodies[bodyId].localIndex;
		let contactKey = world.bodies[bodyId].headContactKey;
		while (contactKey != B2_NULL_INDEX) {
			let edgeIndex = contactKey & 1;
			let contactId = contactKey >> 1;

			contactKey = world.contacts[contactId].edges[edgeIndex].nextKey;

			if (world.contacts[contactId].setIndex != b2_disabledSet) {
				B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet || world.contacts[contactId].setIndex == setIndex);

				// If this contact is awake but not touching I need to fixup the body sim index here.
				if (world.contacts[contactId].setIndex == b2_awakeSet) {
					B2_ASSERT(world.contacts[contactId].colorIndex == B2_NULL_INDEX);
					let awakeLocalIndex = world.contacts[contactId].localIndex;
					if (edgeIndex == 0) {
						awakeSet.contactSims[awakeLocalIndex].encodedBodySimA = encodedBodySimIndex;
					} else {
						awakeSet.contactSims[awakeLocalIndex].encodedBodySimB = encodedBodySimIndex;
					}
				}

				continue;
			}

			let localIndex = world.contacts[contactId].localIndex;
			let contactSim = disabledSet.contactSims[localIndex];

			if (B2_ASSERT_ENABLED) {
				B2_ASSERT((world.contacts[contactId].flags & b2_contactTouchingFlag) == 0 && contactSim.manifold.pointCount == 0);
			}

			world.contacts[contactId].setIndex = b2_awakeSet;
			world.contacts[contactId].localIndex = awakeSet.contactSims.length;

			if (edgeIndex == 0) {
				contactSim.encodedBodySimA = encodedBodySimIndex;
			} else {
				contactSim.encodedBodySimB = encodedBodySimIndex;
			}
			awakeSet.contactSims.push(contactSim);

			let movedLocalIndex = b2Array_RemoveSwap(disabledSet.contactSims, localIndex);
			if (movedLocalIndex != B2_NULL_INDEX) {
				// fix moved element
				let movedContactId = disabledSet.contactSims[localIndex].contactId;
				B2_ASSERT(world.contacts[movedContactId].localIndex == movedLocalIndex);
				world.contacts[movedContactId].localIndex = localIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		let contactCount = set.contactSims.length;
		for (let i = 0; i < contactCount; ++i) {
			let setSims = b2ContactSimView(set.contactSims);
			ref contactSim = setSims[i];
			let contactId = contactSim.contactId;
			if (B2_ASSERT_ENABLED) {
				B2_ASSERT((world.contacts[contactId].flags & b2_contactTouchingFlag) != 0);
				B2_ASSERT((contactSim.simFlags & b2_simTouchingFlag) != 0);
				B2_ASSERT(contactSim.manifold.pointCount > 0);
				B2_ASSERT(world.contacts[contactId].setIndex == setIndex);
			}
			b2AddContactToGraph(world, contactSim, contactId);
			world.contacts[contactId].setIndex = b2_awakeSet;
		}
	}

	// transfer joints from sleeping set to awake set
	{
		let jointCount = set.jointSims.length;
		for (let i = 0; i < jointCount; ++i) {
			let jointSim = set.jointSims[i];
			let jointId = jointSim.jointId;
			B2_ASSERT(world.joints[jointId].setIndex == setIndex);
			b2AddJointToGraph(world, jointSim, jointId);
			world.joints[jointId].setIndex = b2_awakeSet;
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		let islandCount = set.islandSims.length;
		for (let i = 0; i < islandCount; ++i) {
			let islandSrc = set.islandSims[i];
			let island = world.islands[islandSrc.islandId];
			island.setIndex = b2_awakeSet;
			island.localIndex = awakeSet.islandSims.length;
			awakeSet.islandSims.push(islandSrc);
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet(world, setIndex);
}

// Islands need to have a deterministic order because data is moved to a sleeping set according
// to island order.
export function b2TrySleepIsland(world: b2World, islandId: i32): void {
	let island = world.islands[islandId];
	B2_ASSERT(island.setIndex == b2_awakeSet);

	// Cannot put an island to sleep while it has a pending split and more than one body.
	if (island.constraintRemoveCount > 0 && island.bodies.length > 1) {
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	let sleepSetId = b2AllocId(world.solverSetIdPool);
	if (sleepSetId == world.solverSets.length) {
		let set = new b2SolverSet();
		set.setIndex = B2_NULL_INDEX;
		world.solverSets.push(set);
	}

	let sleepSet = world.solverSets[sleepSetId];

	let awakeSet = world.solverSets[b2_awakeSet];
	B2_ASSERT(0 <= island.localIndex && island.localIndex < awakeSet.islandSims.length);

	sleepSet.setIndex = sleepSetId;

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		let disabledSet = world.solverSets[b2_disabledSet];
		for (let i = 0; i < island.bodies.length; ++i) {
			let bodyId = island.bodies[i];
			B2_ASSERT(world.bodies[bodyId].setIndex == b2_awakeSet);
			B2_ASSERT(world.bodies[bodyId].islandId == islandId);
			B2_ASSERT(world.bodies[bodyId].islandIndex == i);

			world.bodies[bodyId].flags &= ~b2_bodyTransientFlags;

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			let bodyMoveIndex = world.bodies[bodyId].bodyMoveIndex;
			if (bodyMoveIndex != B2_NULL_INDEX) {
				if (B2_ASSERT_ENABLED) {
					B2_ASSERT(world.bodyMoveEvents[bodyMoveIndex].bodyId.index1 - 1 == bodyId);
					B2_ASSERT(world.bodyMoveEvents[bodyMoveIndex].bodyId.generation == world.bodies[bodyId].generation);
				}
				world.bodyMoveEvents[bodyMoveIndex].fellAsleep = true;
				world.bodies[bodyId].bodyMoveIndex = B2_NULL_INDEX;
			}

			let awakeBodyIndex = world.bodies[bodyId].localIndex;
			awakeSet.bodySims[awakeBodyIndex].flags &= ~b2_bodyTransientFlags;

			// move body sim to sleep set
			let sleepBodyIndex = sleepSet.bodySims.length;
			sleepSet.bodySims.push(awakeSet.bodySims[awakeBodyIndex]);

			let movedBodyId = b2RemoveBodySim(world, awakeSet, awakeBodyIndex);
			if (movedBodyId != B2_NULL_INDEX && world.bodies[movedBodyId].islandId != islandId) {
				// Update the contact sims for the moved body.
				b2RefreshBodyContactIndices(world, movedBodyId);
			}

			// Destroy state, no need to clone.
			b2Array_RemoveSwap(awakeSet.bodyStates, awakeBodyIndex);

			world.bodies[bodyId].setIndex = sleepSetId;
			world.bodies[bodyId].localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			let contactKey = world.bodies[bodyId].headContactKey;
			while (contactKey != B2_NULL_INDEX) {
				let contactId = contactKey >> 1;
				let edgeIndex = contactKey & 1;

				B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet);

				contactKey = world.contacts[contactId].edges[edgeIndex].nextKey;

				if (world.contacts[contactId].colorIndex != B2_NULL_INDEX) {
					// contact is touching and will be moved separately
					B2_ASSERT((world.contacts[contactId].flags & b2_contactTouchingFlag) != 0);
					continue;
				}

				let localIndex = world.contacts[contactId].localIndex;

				if (edgeIndex == 0) {
					awakeSet.contactSims[localIndex].encodedBodySimA = B2_NULL_INDEX;
				} else {
					awakeSet.contactSims[localIndex].encodedBodySimB = B2_NULL_INDEX;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				let otherEdgeIndex = edgeIndex ^ 1;
				let otherBodyId = world.contacts[contactId].edges[otherEdgeIndex].bodyId;
				if (world.bodies[otherBodyId].setIndex == b2_awakeSet) {
					continue;
				}

				if (B2_ASSERT_ENABLED) {
					B2_ASSERT(awakeSet.contactSims[localIndex].manifold.pointCount == 0);
					B2_ASSERT((world.contacts[contactId].flags & b2_contactTouchingFlag) == 0);
				}

				// move the non-touching contact to the disabled set
				world.contacts[contactId].setIndex = b2_disabledSet;
				world.contacts[contactId].localIndex = disabledSet.contactSims.length;
				disabledSet.contactSims.push(awakeSet.contactSims[localIndex]);

				let movedLocalIndex = b2Array_RemoveSwap(awakeSet.contactSims, localIndex);
				if (movedLocalIndex != B2_NULL_INDEX) {
					// fix moved element
					let movedContactId = awakeSet.contactSims[localIndex].contactId;
					B2_ASSERT(world.contacts[movedContactId].localIndex == movedLocalIndex);
					world.contacts[movedContactId].localIndex = localIndex;
				}
			}
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		for (let i = 0; i < island.contacts.length; ++i) {
			let contactId = island.contacts[i].contactId;
			B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet);
			B2_ASSERT(world.contacts[contactId].islandId == islandId);
			let colorIndex = world.contacts[contactId].colorIndex;
			B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);

			let color = world.constraintGraph.colors[colorIndex];

			// Remove bodies from graph coloring associated with this constraint
			if (colorIndex != B2_OVERFLOW_INDEX) {
				// might clear a bit for a static body, but this has no effect
				b2ClearBit(color.bodySet, <u32>world.contacts[contactId].edges[0].bodyId);
				b2ClearBit(color.bodySet, <u32>world.contacts[contactId].edges[1].bodyId);
			}

			let localIndex = world.contacts[contactId].localIndex;
			let sleepContactSim = color.contactSims[localIndex];

			let sleepContactIndex = sleepSet.contactSims.length;
			sleepContactSim.encodedBodySimA = b2SleepBodySimIndex(sleepContactSim.encodedBodySimA);
			sleepContactSim.encodedBodySimB = b2SleepBodySimIndex(sleepContactSim.encodedBodySimB);
			sleepSet.contactSims.push(sleepContactSim);

			let movedLocalIndex = b2Array_RemoveSwap(color.contactSims, localIndex);
			if (movedLocalIndex != B2_NULL_INDEX) {
				// fix moved element
				let movedContactId = color.contactSims[localIndex].contactId;
				B2_ASSERT(world.contacts[movedContactId].localIndex == movedLocalIndex);
				world.contacts[movedContactId].localIndex = localIndex;
			}

			world.contacts[contactId].setIndex = sleepSetId;
			world.contacts[contactId].colorIndex = B2_NULL_INDEX;
			world.contacts[contactId].localIndex = sleepContactIndex;
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		for (let i = 0; i < island.joints.length; ++i) {
			let jointId = island.joints[i].jointId;
			B2_ASSERT(world.joints[jointId].setIndex == b2_awakeSet);
			B2_ASSERT(world.joints[jointId].islandId == islandId);
			let colorIndex = world.joints[jointId].colorIndex;
			let localIndex = world.joints[jointId].localIndex;

			B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);

			let color = world.constraintGraph.colors[colorIndex];

			if (colorIndex != B2_OVERFLOW_INDEX) {
				// might clear a bit for a static body, but this has no effect
				b2ClearBit(color.bodySet, <u32>world.joints[jointId].edges[0].bodyId);
				b2ClearBit(color.bodySet, <u32>world.joints[jointId].edges[1].bodyId);
			}

			let sleepJointIndex = sleepSet.jointSims.length;
			sleepSet.jointSims.push(color.jointSims[localIndex]);

			let movedIndex = b2Array_RemoveSwap(color.jointSims, localIndex);
			if (movedIndex != B2_NULL_INDEX) {
				// fix moved element
				let movedId = color.jointSims[localIndex].jointId;
				B2_ASSERT(world.joints[movedId].localIndex == movedIndex);
				world.joints[movedId].localIndex = localIndex;
			}

			world.joints[jointId].setIndex = sleepSetId;
			world.joints[jointId].colorIndex = B2_NULL_INDEX;
			world.joints[jointId].localIndex = sleepJointIndex;
		}
	}

	// move island struct
	{
		B2_ASSERT(island.setIndex == b2_awakeSet);

		let islandIndex = island.localIndex;
		let sleepIsland = b2IslandSim();
		sleepIsland.islandId = islandId;
		sleepSet.islandSims.push(sleepIsland);

		let movedIslandIndex = b2Array_RemoveSwap(awakeSet.islandSims, islandIndex);
		if (movedIslandIndex != B2_NULL_INDEX) {
			// fix index on moved element
			let movedIslandId = awakeSet.islandSims[islandIndex].islandId;
			let movedIsland = world.islands[movedIslandId];
			B2_ASSERT(movedIsland.localIndex == movedIslandIndex);
			movedIsland.localIndex = islandIndex;
		}

		island.setIndex = sleepSetId;
		island.localIndex = 0;
	}

	if (world.splitIslandId == islandId) {
		world.splitIslandId = B2_NULL_INDEX;
	}
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
export function b2MergeSolverSets(world: b2World, setId1: i32, setId2: i32): void {
	B2_ASSERT(setId1 >= b2_firstSleepingSet);
	B2_ASSERT(setId2 >= b2_firstSleepingSet);
	let set1 = world.solverSets[setId1];
	let set2 = world.solverSets[setId2];

	// Move the fewest number of bodies
	if (set1.bodySims.length < set2.bodySims.length) {
		let tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		let tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// transfer bodies
	{
		let bodyCount = set2.bodySims.length;
		for (let i = 0; i < bodyCount; ++i) {
			let bodyId = set2.bodySims[i].bodyId;
			B2_ASSERT(world.bodies[bodyId].setIndex == setId2);
			world.bodies[bodyId].setIndex = setId1;
			world.bodies[bodyId].localIndex = set1.bodySims.length;

			set1.bodySims.push(set2.bodySims[i]);
		}
	}

	// transfer contacts
	{
		let contactCount = set2.contactSims.length;
		for (let i = 0; i < contactCount; ++i) {
			let contactId = set2.contactSims[i].contactId;
			B2_ASSERT(world.contacts[contactId].setIndex == setId2);
			world.contacts[contactId].setIndex = setId1;
			world.contacts[contactId].localIndex = set1.contactSims.length;

			set1.contactSims.push(set2.contactSims[i]);
		}
	}

	// transfer joints
	{
		let jointCount = set2.jointSims.length;
		for (let i = 0; i < jointCount; ++i) {
			let jointId = set2.jointSims[i].jointId;
			B2_ASSERT(world.joints[jointId].setIndex == setId2);
			world.joints[jointId].setIndex = setId1;
			world.joints[jointId].localIndex = set1.jointSims.length;

			set1.jointSims.push(set2.jointSims[i]);
		}
	}

	// transfer islands
	{
		let islandCount = set2.islandSims.length;
		for (let i = 0; i < islandCount; ++i) {
			let islandSrc = set2.islandSims[i];
			let islandId = islandSrc.islandId;

			let island = world.islands[islandId];
			island.setIndex = setId1;
			island.localIndex = set1.islandSims.length;

			set1.islandSims.push(islandSrc);
		}
	}

	// destroy the merged set
	b2DestroySolverSet(world, setId2);
}

// Remove a body from a solver set and return the body swapped into the slot.
export function b2RemoveBodySim(world: b2World, set: b2SolverSet, localIndex: i32): i32 {
	let bodySims = set.bodySims;
	B2_ASSERT(0 <= localIndex && localIndex < bodySims.length);
	let lastIndex = bodySims.length - 1;

	if (localIndex == lastIndex) {
		bodySims.length = lastIndex;
		return B2_NULL_INDEX;
	}

	bodySims[localIndex] = bodySims[lastIndex];
	bodySims.length = lastIndex;
	let movedBodyId = bodySims[localIndex].bodyId;
	B2_ASSERT(world.bodies[movedBodyId].localIndex == lastIndex);
	world.bodies[movedBodyId].localIndex = localIndex;

	return movedBodyId;
}

export function b2TransferBody(world: b2World, targetSet: b2SolverSet, sourceSet: b2SolverSet, bodyId: i32): void {
	B2_ASSERT(world.bodies[bodyId].headContactKey == B2_NULL_INDEX);

	if (targetSet == sourceSet) {
		return;
	}

	// The use cases for transfer body (set body type, disable, etc) destroy all the contacts
	// first. Double check here because this function wouldn't work correctly otherwise.
	B2_ASSERT(world.bodies[bodyId].headContactKey == B2_NULL_INDEX);

	let sourceIndex = world.bodies[bodyId].localIndex;
	let targetSim = sourceSet.bodySims[sourceIndex];

	let targetIndex = targetSet.bodySims.length;

	// Clear transient body flags
	world.bodies[bodyId].flags &= ~b2_bodyTransientFlags;
	targetSim.flags &= ~(b2_isFast | b2_bodyTransientFlags);
	targetSet.bodySims.push(targetSim);

	// Remove body sim from solver set that owns it
	let movedBodyId = b2RemoveBodySim(world, sourceSet, sourceIndex);
	if (movedBodyId != B2_NULL_INDEX) {
		// Fixup the moved body's contact sims.
		b2RefreshBodyContactIndices(world, movedBodyId);
	}

	if (sourceSet.setIndex == b2_awakeSet) {
		b2Array_RemoveSwap(sourceSet.bodyStates, sourceIndex);
	} else if (targetSet.setIndex == b2_awakeSet) {
		let state = b2_identityBodyState;
		state.flags = world.bodies[bodyId].flags;
		targetSet.bodyStates.push(state);
	}

	world.bodies[bodyId].setIndex = targetSet.setIndex;
	world.bodies[bodyId].localIndex = targetIndex;
}

export function b2TransferJoint(world: b2World, targetSet: b2SolverSet, sourceSet: b2SolverSet, jointId: i32): void {
	if (targetSet == sourceSet) {
		return;
	}

	let localIndex = world.joints[jointId].localIndex;
	let colorIndex = world.joints[jointId].colorIndex;

	// Retrieve source.
	let sourceSim: b2JointSim;
	if (sourceSet.setIndex == b2_awakeSet) {
		B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
		let color = world.constraintGraph.colors[colorIndex];

		sourceSim = color.jointSims[localIndex];
	} else {
		B2_ASSERT(colorIndex == B2_NULL_INDEX);
		sourceSim = sourceSet.jointSims[localIndex];
	}

	// Create target and copy. Fix joint.
	if (targetSet.setIndex == b2_awakeSet) {
		b2AddJointToGraph(world, sourceSim, jointId);
		world.joints[jointId].setIndex = b2_awakeSet;
	} else {
		world.joints[jointId].setIndex = targetSet.setIndex;
		world.joints[jointId].localIndex = targetSet.jointSims.length;
		world.joints[jointId].colorIndex = B2_NULL_INDEX;

		targetSet.jointSims.push(sourceSim);
	}

	// Destroy source.
	if (sourceSet.setIndex == b2_awakeSet) {
		b2RemoveJointFromGraph(world, world.joints[jointId].edges[0].bodyId, world.joints[jointId].edges[1].bodyId, colorIndex, localIndex);
	} else {
		let movedIndex = b2Array_RemoveSwap(sourceSet.jointSims, localIndex);
		if (movedIndex != B2_NULL_INDEX) {
			// fix swapped element
			let movedId = sourceSet.jointSims[localIndex].jointId;
			world.joints[movedId].localIndex = localIndex;
		}
	}
}
