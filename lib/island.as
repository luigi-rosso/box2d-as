// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/island.c

import { B2_ASSERT, B2_NULL_INDEX, b2GetMilliseconds, b2GetTicks } from "./core";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2Array_RemoveSwap } from "./container";
import { b2_dynamicBody } from "./types";
import { b2_contactTouchingFlag } from "./contact";
import { b2World } from "./physics_world";
import { b2WakeSolverSet, b2_awakeSet, b2_disabledSet, b2_firstSleepingSet, b2_staticSet } from "./solver_set";

// Cached contact data stored in the island for fast contiguous iteration.
// Avoids touching b2Contact during union-find in b2SplitIsland.
export struct b2ContactLink {
	contactId: i32 = 0;
	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;
}

// Cached joint data stored in the island for fast contiguous iteration.
export struct b2JointLink {
	jointId: i32 = 0;
	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;
}

// Persistent island for awake bodies, joints, and contacts.
// Contacts are touching.
// Contacts and joints may connect to static bodies, but static bodies are not in the island.
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
export class b2Island {
	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	setIndex: i32 = 0;

	// island index within set
	// may be B2_NULL_INDEX
	localIndex: i32 = 0;

	islandId: i32 = 0;

	// Keeps track of how many contacts have been removed from this island.
	// This is used to determine if an island is a candidate for splitting.
	constraintRemoveCount: i32 = 0;

	bodies: Array<i32> = new Array<i32>();

	// Contacts and joints that belong to this island. May connect to static
	// bodies not in the island.
	// Each link has the two body ids so that b2SplitIsland's union-find pass
	// never needs to touch b2Contact/b2Joint.
	contacts: Array<b2ContactLink> = new Array<b2ContactLink>();
	joints: Array<b2JointLink> = new Array<b2JointLink>();
}

// This is used to move islands across solver sets
export struct b2IslandSim {
	islandId: i32 = 0;
}

// functions from island.c follow

export function b2CreateIsland(world: b2World, setIndex: i32): b2Island {
	B2_ASSERT(setIndex == b2_awakeSet || setIndex >= b2_firstSleepingSet);

	let islandId = b2AllocId(world.islandIdPool);

	let island: b2Island;
	if (islandId == world.islands.length) {
		island = new b2Island();
		world.islands.push(island);
	} else {
		island = world.islands[islandId];
		B2_ASSERT(island.setIndex == B2_NULL_INDEX);
	}

	let set = world.solverSets[setIndex];

	island.setIndex = setIndex;
	island.localIndex = set.islandSims.length;
	island.islandId = islandId;

	// C creates empty arrays; a destroyed island kept its emptied ones
	island.bodies.length = 0;
	island.contacts.length = 0;
	island.joints.length = 0;
	island.constraintRemoveCount = 0;

	let islandSim = b2IslandSim();
	islandSim.islandId = islandId;
	set.islandSims.push(islandSim);

	return island;
}

export function b2DestroyIsland(world: b2World, islandId: i32): void {
	if (world.splitIslandId == islandId) {
		world.splitIslandId = B2_NULL_INDEX;
	}

	// assume island is empty
	let island = world.islands[islandId];
	let set = world.solverSets[island.setIndex];
	{
		let localIndex = island.localIndex;
		let lastIndex = set.islandSims.length - 1;
		B2_ASSERT(0 <= localIndex && localIndex <= lastIndex);
		let moveIslandId = set.islandSims[lastIndex].islandId;
		set.islandSims[localIndex] = set.islandSims[lastIndex];
		world.islands[moveIslandId].localIndex = localIndex;
		set.islandSims.length = lastIndex;
	}

	// Free island and id (preserve island revision)
	island.bodies.length = 0;
	island.contacts.length = 0;
	island.joints.length = 0;
	island.constraintRemoveCount = 0;
	island.localIndex = B2_NULL_INDEX;
	island.islandId = B2_NULL_INDEX;
	island.setIndex = B2_NULL_INDEX;

	b2FreeId(world.islandIdPool, islandId);
}

function b2MergeIslands(world: b2World, islandIdA: i32, islandIdB: i32): i32 {
	if (islandIdA == islandIdB) {
		return islandIdA;
	}

	if (islandIdA == B2_NULL_INDEX) {
		B2_ASSERT(islandIdB != B2_NULL_INDEX);
		return islandIdB;
	}

	if (islandIdB == B2_NULL_INDEX) {
		B2_ASSERT(islandIdA != B2_NULL_INDEX);
		return islandIdA;
	}

	let smallIsland: b2Island;
	let bigIsland: b2Island;
	{
		let islandA = world.islands[islandIdA];
		let islandB = world.islands[islandIdB];

		// Keep the biggest island to reduce cache misses
		if (islandA.bodies.length >= islandB.bodies.length) {
			bigIsland = islandA;
			smallIsland = islandB;
		} else {
			bigIsland = islandB;
			smallIsland = islandA;
		}
	}

	let bigIslandId = bigIsland.islandId;

	// Move bodies from smaller island to larger island
	for (let i = 0; i < smallIsland.bodies.length; ++i) {
		let bodyId = smallIsland.bodies[i];
		world.bodies[bodyId].islandId = bigIslandId;
		world.bodies[bodyId].islandIndex = bigIsland.bodies.length;
		bigIsland.bodies.push(bodyId);
	}

	// Migrate contacts from smaller island to larger island
	if (smallIsland.contacts.length > 0) {
		for (let i = 0; i < smallIsland.contacts.length; ++i) {
			let link = smallIsland.contacts[i];
			world.contacts[link.contactId].islandId = bigIslandId;
			world.contacts[link.contactId].islandIndex = bigIsland.contacts.length;
			bigIsland.contacts.push(link);
		}
	}

	// Migrate joints from smaller island to larger island
	if (smallIsland.joints.length > 0) {
		for (let i = 0; i < smallIsland.joints.length; ++i) {
			let link = smallIsland.joints[i];
			world.joints[link.jointId].islandId = bigIslandId;
			world.joints[link.jointId].islandIndex = bigIsland.joints.length;
			bigIsland.joints.push(link);
		}
	}

	// Track removed constraints
	bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount;

	b2DestroyIsland(world, smallIsland.islandId);

	return bigIslandId;
}

function b2AddContactToIsland(world: b2World, islandId: i32, contactId: i32): void {
	B2_ASSERT(world.contacts[contactId].islandId == B2_NULL_INDEX);
	B2_ASSERT(world.contacts[contactId].islandIndex == B2_NULL_INDEX);

	let island = world.islands[islandId];

	world.contacts[contactId].islandId = islandId;
	world.contacts[contactId].islandIndex = island.contacts.length;

	let link = b2ContactLink();
	link.contactId = world.contacts[contactId].contactId;
	link.bodyIdA = world.contacts[contactId].edges[0].bodyId;
	link.bodyIdB = world.contacts[contactId].edges[1].bodyId;
	island.contacts.push(link);
}

// Link a contact into an island.
export function b2LinkContact(world: b2World, contactId: i32): void {
	B2_ASSERT((world.contacts[contactId].flags & b2_contactTouchingFlag) != 0);

	let bodyIdA = world.contacts[contactId].edges[0].bodyId;
	let bodyIdB = world.contacts[contactId].edges[1].bodyId;

	B2_ASSERT(world.bodies[bodyIdA].setIndex != b2_disabledSet && world.bodies[bodyIdB].setIndex != b2_disabledSet);
	B2_ASSERT(world.bodies[bodyIdA].setIndex != b2_staticSet || world.bodies[bodyIdB].setIndex != b2_staticSet);

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if (world.bodies[bodyIdA].setIndex == b2_awakeSet && world.bodies[bodyIdB].setIndex >= b2_firstSleepingSet) {
		b2WakeSolverSet(world, world.bodies[bodyIdB].setIndex);
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if (world.bodies[bodyIdB].setIndex == b2_awakeSet && world.bodies[bodyIdA].setIndex >= b2_firstSleepingSet) {
		b2WakeSolverSet(world, world.bodies[bodyIdA].setIndex);
	}

	let islandIdA = world.bodies[bodyIdA].islandId;
	let islandIdB = world.bodies[bodyIdB].islandId;

	// Static bodies have null island indices.
	B2_ASSERT(world.bodies[bodyIdA].setIndex != b2_staticSet || islandIdA == B2_NULL_INDEX);
	B2_ASSERT(world.bodies[bodyIdB].setIndex != b2_staticSet || islandIdB == B2_NULL_INDEX);
	B2_ASSERT(islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX);

	// Merge islands. This will destroy one of the islands.
	let finalIslandId = b2MergeIslands(world, islandIdA, islandIdB);

	// Add contact to the island that survived
	b2AddContactToIsland(world, finalIslandId, contactId);
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
export function b2UnlinkContact(world: b2World, contactId: i32): void {
	B2_ASSERT(world.contacts[contactId].islandId != B2_NULL_INDEX);

	// remove from island
	let islandId = world.contacts[contactId].islandId;
	let island = world.islands[islandId];

	let removeIndex = world.contacts[contactId].islandIndex;
	B2_ASSERT(0 <= removeIndex && removeIndex < island.contacts.length);
	B2_ASSERT(island.contacts[removeIndex].contactId == world.contacts[contactId].contactId);

	let movedIndex = b2Array_RemoveSwap(island.contacts, removeIndex);
	if (movedIndex != B2_NULL_INDEX) {
		// Fix islandIndex on the contact that was swapped into removeIndex
		let movedContactId = island.contacts[removeIndex].contactId;
		B2_ASSERT(world.contacts[movedContactId].islandIndex == movedIndex);
		world.contacts[movedContactId].islandIndex = removeIndex;
	}

	world.contacts[contactId].islandId = B2_NULL_INDEX;
	world.contacts[contactId].islandIndex = B2_NULL_INDEX;
	island.constraintRemoveCount += 1;
}

function b2AddJointToIsland(world: b2World, islandId: i32, jointId: i32): void {
	B2_ASSERT(world.joints[jointId].islandId == B2_NULL_INDEX);
	B2_ASSERT(world.joints[jointId].islandIndex == B2_NULL_INDEX);

	let island = world.islands[islandId];

	world.joints[jointId].islandId = islandId;
	world.joints[jointId].islandIndex = island.joints.length;

	let link = b2JointLink();
	link.jointId = world.joints[jointId].jointId;
	link.bodyIdA = world.joints[jointId].edges[0].bodyId;
	link.bodyIdB = world.joints[jointId].edges[1].bodyId;
	island.joints.push(link);
}

export function b2LinkJoint(world: b2World, jointId: i32): void {
	let bodyIdA = world.joints[jointId].edges[0].bodyId;
	let bodyIdB = world.joints[jointId].edges[1].bodyId;

	B2_ASSERT(world.bodies[bodyIdA].type == b2_dynamicBody || world.bodies[bodyIdB].type == b2_dynamicBody);

	if (world.bodies[bodyIdA].setIndex == b2_awakeSet && world.bodies[bodyIdB].setIndex >= b2_firstSleepingSet) {
		b2WakeSolverSet(world, world.bodies[bodyIdB].setIndex);
	} else if (world.bodies[bodyIdB].setIndex == b2_awakeSet && world.bodies[bodyIdA].setIndex >= b2_firstSleepingSet) {
		b2WakeSolverSet(world, world.bodies[bodyIdA].setIndex);
	}

	let islandIdA = world.bodies[bodyIdA].islandId;
	let islandIdB = world.bodies[bodyIdB].islandId;

	B2_ASSERT(islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX);

	// Merge islands. This will destroy one of the islands.
	let finalIslandId = b2MergeIslands(world, islandIdA, islandIdB);

	// Add joint the island that survived
	b2AddJointToIsland(world, finalIslandId, jointId);
}

export function b2UnlinkJoint(world: b2World, jointId: i32): void {
	if (world.joints[jointId].islandId == B2_NULL_INDEX) {
		return;
	}

	// remove from island
	let islandId = world.joints[jointId].islandId;
	let island = world.islands[islandId];

	let removeIndex = world.joints[jointId].islandIndex;
	B2_ASSERT(0 <= removeIndex && removeIndex < island.joints.length);
	B2_ASSERT(island.joints[removeIndex].jointId == world.joints[jointId].jointId);

	let movedIndex = b2Array_RemoveSwap(island.joints, removeIndex);
	if (movedIndex != B2_NULL_INDEX) {
		// Fix islandIndex on the joint that was swapped into removeIndex
		let movedJointId = island.joints[removeIndex].jointId;
		B2_ASSERT(world.joints[movedJointId].islandIndex == movedIndex);
		world.joints[movedJointId].islandIndex = removeIndex;
	}

	world.joints[jointId].islandId = B2_NULL_INDEX;
	world.joints[jointId].islandIndex = B2_NULL_INDEX;
	island.constraintRemoveCount += 1;
}

// C's stack allocations for the split, grow only
let b2_splitParents = new StaticArray<i32>(0);
let b2_splitContactCounts = new StaticArray<i32>(0);
let b2_splitJointCounts = new StaticArray<i32>(0);
let b2_splitRanks = new StaticArray<i32>(0);
let b2_splitRootMap = new StaticArray<i32>(0);
let b2_splitComponentBodyCounts = new StaticArray<i32>(0);
let b2_splitComponentContactCounts = new StaticArray<i32>(0);
let b2_splitComponentJointCounts = new StaticArray<i32>(0);
let b2_splitIslandIds = new StaticArray<i32>(0);

function b2ReserveSplit(a: StaticArray<i32>, count: i32): StaticArray<i32> {
	return a.length >= count ? a : new StaticArray<i32>(count + count / 4 + 16);
}

// Find parent of a node. Use path halving to speed up further queries.
@inline function b2IslandFindParent(parents: StaticArray<i32>, node: i32): i32 {
	// Walk the chain of parents to find the node that is its own parent (the root)
	while (parents[node] != node) {
		let grandParent = parents[parents[node]];
		parents[node] = grandParent;
		node = grandParent;
	}

	return node;
}

// Connect the components containing node1 and node2.
// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.
@inline function b2IslandUnion(
	parents: StaticArray<i32>,
	ranks: StaticArray<i32>,
	node1: i32,
	node2: i32,
	contactCounts: StaticArray<i32>,
	jointCounts: StaticArray<i32>,
): void {
	let root1 = b2IslandFindParent(parents, node1);
	let root2 = b2IslandFindParent(parents, node2);
	if (root1 != root2) {
		if (ranks[root1] < ranks[root2]) {
			parents[root1] = root2;
			contactCounts[root2] += contactCounts[root1];
			jointCounts[root2] += jointCounts[root1];
		} else if (ranks[root1] > ranks[root2]) {
			parents[root2] = root1;
			contactCounts[root1] += contactCounts[root2];
			jointCounts[root1] += jointCounts[root2];
		} else {
			parents[root2] = root1;
			ranks[root1] += 1;
			contactCounts[root1] += contactCounts[root2];
			jointCounts[root1] += jointCounts[root2];
		}
	}
}

// This uses union-find.
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
export function b2SplitIsland(world: b2World, baseId: i32): void {
	let baseIsland = world.islands[baseId];
	B2_ASSERT(baseIsland.constraintRemoveCount > 0);
	B2_ASSERT(baseIsland.setIndex == b2_awakeSet);

	let baseBodyCount = baseIsland.bodies.length;
	let baseBodyIds = baseIsland.bodies;

	let baseContactCount = baseIsland.contacts.length;
	let baseContacts = baseIsland.contacts;

	let baseJointCount = baseIsland.joints.length;
	let baseJoints = baseIsland.joints;

	b2_splitParents = b2ReserveSplit(b2_splitParents, baseBodyCount);
	b2_splitContactCounts = b2ReserveSplit(b2_splitContactCounts, baseBodyCount);
	b2_splitJointCounts = b2ReserveSplit(b2_splitJointCounts, baseBodyCount);
	b2_splitRanks = b2ReserveSplit(b2_splitRanks, baseBodyCount);
	let parents = b2_splitParents;
	let contactCounts = b2_splitContactCounts;
	let jointCounts = b2_splitJointCounts;
	let ranks = b2_splitRanks;
	for (let i = 0; i < baseBodyCount; ++i) {
		parents[i] = i;
		ranks[i] = 0;
		contactCounts[i] = 0;
		jointCounts[i] = 0;
	}

	// Union over contacts, tracking per-component contact counts
	for (let i = 0; i < baseContactCount; ++i) {
		let bodyIdA = baseContacts[i].bodyIdA;
		let bodyIdB = baseContacts[i].bodyIdB;
		let islandIndexA = world.bodies[bodyIdA].islandIndex;
		let islandIndexB = world.bodies[bodyIdB].islandIndex;

		// Only connect non-static bodies
		if (islandIndexA != B2_NULL_INDEX && islandIndexB != B2_NULL_INDEX) {
			b2IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts);
			let root = b2IslandFindParent(parents, islandIndexA);
			contactCounts[root] += 1;
		} else {
			let islandIndex = islandIndexA != B2_NULL_INDEX ? islandIndexA : islandIndexB;
			let root = b2IslandFindParent(parents, islandIndex);
			contactCounts[root] += 1;
		}
	}

	// Union over joints, tracking per-component joint counts
	for (let i = 0; i < baseJointCount; ++i) {
		let bodyIdA = baseJoints[i].bodyIdA;
		let bodyIdB = baseJoints[i].bodyIdB;
		let islandIndexA = world.bodies[bodyIdA].islandIndex;
		let islandIndexB = world.bodies[bodyIdB].islandIndex;

		// Only connect non-static bodies
		if (islandIndexA != B2_NULL_INDEX && islandIndexB != B2_NULL_INDEX) {
			b2IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts);
			let root = b2IslandFindParent(parents, islandIndexA);
			jointCounts[root] += 1;
		} else {
			let islandIndex = islandIndexA != B2_NULL_INDEX ? islandIndexA : islandIndexB;
			let root = b2IslandFindParent(parents, islandIndex);
			jointCounts[root] += 1;
		}
	}

	// Flatten all parent indices and count connected components.
	let componentCount = 0;
	for (let i = 0; i < baseBodyCount; ++i) {
		parents[i] = b2IslandFindParent(parents, i);
		if (parents[i] == i) {
			componentCount += 1;
		}
	}

	// Early return - island is still fully connected, no split needed.
	if (componentCount == 1) {
		baseIsland.constraintRemoveCount = 0;
		return;
	}

	// Detach body/contact/joint arrays from base island so b2DestroyIsland won't free them
	baseIsland.bodies = new Array<i32>();
	baseIsland.contacts = new Array<b2ContactLink>();
	baseIsland.joints = new Array<b2JointLink>();

	// Map from body index to new island index. Only set for root bodies.
	b2_splitRootMap = b2ReserveSplit(b2_splitRootMap, baseBodyCount);
	let rootMap = b2_splitRootMap;
	for (let i = 0; i < baseBodyCount; ++i) {
		rootMap[i] = B2_NULL_INDEX;
	}

	b2_splitComponentBodyCounts = b2ReserveSplit(b2_splitComponentBodyCounts, componentCount);
	b2_splitComponentContactCounts = b2ReserveSplit(b2_splitComponentContactCounts, componentCount);
	b2_splitComponentJointCounts = b2ReserveSplit(b2_splitComponentJointCounts, componentCount);
	let componentBodyCounts = b2_splitComponentBodyCounts;
	let componentContactCounts = b2_splitComponentContactCounts;
	let componentJointCounts = b2_splitComponentJointCounts;
	let islandCount = 0;

	// Find the root body for each body and create islands as needed.
	// Extract per-component counts from the root nodes' accumulated counts.
	for (let i = 0; i < baseBodyCount; ++i) {
		let rootIndex = parents[i];
		if (rootMap[rootIndex] == B2_NULL_INDEX) {
			rootMap[rootIndex] = islandCount;
			componentBodyCounts[islandCount] = 0;
			componentContactCounts[islandCount] = contactCounts[rootIndex];
			componentJointCounts[islandCount] = jointCounts[rootIndex];
			islandCount += 1;
		}

		componentBodyCounts[rootMap[rootIndex]] += 1;
	}

	B2_ASSERT(islandCount == componentCount);

	// Map from new island index to island id
	b2_splitIslandIds = b2ReserveSplit(b2_splitIslandIds, islandCount);
	let islandIds = b2_splitIslandIds;

	// Create new islands
	for (let i = 0; i < islandCount; ++i) {
		let newIsland = b2CreateIsland(world, b2_awakeSet);
		islandIds[i] = newIsland.islandId;
	}

	// Assign bodies to new islands
	for (let i = 0; i < baseBodyCount; ++i) {
		let bodyId = baseBodyIds[i];
		let root = b2IslandFindParent(parents, i);
		let newIslandId = islandIds[rootMap[root]];

		let newIsland = world.islands[newIslandId];

		world.bodies[bodyId].islandId = newIslandId;
		world.bodies[bodyId].islandIndex = newIsland.bodies.length;
		newIsland.bodies.push(bodyId);
	}

	// Assign contacts to the island of their bodies
	for (let i = 0; i < baseContactCount; ++i) {
		let link = baseContacts[i];

		// Static bodies don't have an island id.
		let islandIdA = world.bodies[link.bodyIdA].islandId;
		let targetIslandId = islandIdA != B2_NULL_INDEX ? islandIdA : world.bodies[link.bodyIdB].islandId;

		let targetIsland = world.islands[targetIslandId];
		world.contacts[link.contactId].islandId = targetIslandId;
		world.contacts[link.contactId].islandIndex = targetIsland.contacts.length;
		targetIsland.contacts.push(link);
	}

	// Assign joints to the island of their bodies
	for (let i = 0; i < baseJointCount; ++i) {
		let link = baseJoints[i];

		// Static bodies don't have an island id.
		let islandIdA = world.bodies[link.bodyIdA].islandId;
		let targetIslandId = islandIdA != B2_NULL_INDEX ? islandIdA : world.bodies[link.bodyIdB].islandId;

		let targetIsland = world.islands[targetIslandId];
		world.joints[link.jointId].islandId = targetIslandId;
		world.joints[link.jointId].islandIndex = targetIsland.joints.length;
		targetIsland.joints.push(link);
	}

	// Destroy the base island
	b2DestroyIsland(world, baseId);
}

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses union find and
// touches a lot of memory, so it can be slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
export function b2SplitIslandTask(world: b2World): void {
	let ticks = b2GetTicks();

	B2_ASSERT(world.splitIslandId != B2_NULL_INDEX);

	b2SplitIsland(world, world.splitIslandId);

	world.splitIslandId = B2_NULL_INDEX;
	world.profile.splitIslands += b2GetMilliseconds(ticks);
}
