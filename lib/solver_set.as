// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver_set.c

import { b2BodySim, b2BodyState } from "./body";
import { b2ContactSim } from "./contact";
import { b2IslandSim } from "./island";
import { b2JointSim } from "./joint";

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
