// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/island.c

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
