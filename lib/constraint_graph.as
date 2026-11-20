// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/constraint_graph.c

import { b2BitSet, b2ClearBit, b2CreateBitSet, b2DestroyBitSet, b2GetBit, b2SetBitCountAndClear, b2SetBitGrow } from "./bitset";
import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import { B2_GRAPH_COLOR_COUNT } from "./constants";
import { b2MaxInt } from "./math_functions";
import { b2BodyType, b2Capacity, b2_dynamicBody, b2_staticBody } from "./types";
import { b2EncodeBodySimIndex, b2ArrayView } from "./body";
import { b2ContactSim, b2_contactTouchingFlag, b2_simTouchingFlag } from "./contact";
import { b2JointSim } from "./joint";
import { b2ContactConstraint } from "./contact_solver";
import { b2World } from "./physics_world";
import { b2_awakeSet } from "./solver_set";
import { b2Array_Emplace, b2Array_RemoveSwap } from "./container";

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
export const B2_OVERFLOW_INDEX: i32 = B2_GRAPH_COLOR_COUNT - 1;

// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
export const B2_DYNAMIC_COLOR_COUNT: i32 = B2_GRAPH_COLOR_COUNT - 4;

const b2_emptyContactConstraints = new StaticArray<b2ContactConstraint>(0);

export class b2GraphColor {
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	bodySet: b2BitSet = new b2BitSet();

	// cache friendly arrays
	contactSims: Array<b2ContactSim> = new Array<b2ContactSim>();
	jointSims: Array<b2JointSim> = new Array<b2JointSim>();

	// transient, C's union of two pointers into step memory: this color's wide constraints start at
	// wideConstraintStart in the solver scratch, the overflow color's constraints are overflowConstraints
	wideConstraintStart: i32 = 0;
	overflowConstraints: StaticArray<b2ContactConstraint> = b2_emptyContactConstraints;

	wideConstraintCount: i32 = 0;
}

export class b2ConstraintGraph {
	// including overflow at the end
	colors: StaticArray<b2GraphColor> = new StaticArray<b2GraphColor>(B2_GRAPH_COLOR_COUNT);

	constructor() {
		for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
			this.colors[i] = new b2GraphColor();
		}
	}
}

// functions from constraint_graph.c follow

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.
// todo should be possible to branch on the scatters to avoid writing to kinematic bodies

// This is used for debugging by making all constraints be assigned to overflow.
// Don't expect the same result for overflow and graph colored. They have a different solve order.
const B2_FORCE_OVERFLOW: i32 = 0;

export function b2CreateGraph(graph: b2ConstraintGraph, capacity: b2Capacity): void {
	for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
		graph.colors[i] = new b2GraphColor();
	}

	let bodyCapacity = b2MaxInt(capacity.staticBodyCount + capacity.dynamicBodyCount, 16);

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for (let i = 0; i < B2_OVERFLOW_INDEX; ++i) {
		let color = graph.colors[i];
		color.bodySet = b2CreateBitSet(<u32>bodyCapacity);
		b2SetBitCountAndClear(color.bodySet, <u32>bodyCapacity);
	}
}

export function b2DestroyGraph(graph: b2ConstraintGraph): void {
	for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
		let color = graph.colors[i];

		// The bit set should never be used on the overflow color
		if (B2_ASSERT_ENABLED) B2_ASSERT(i != B2_OVERFLOW_INDEX || color.bodySet.bits.length == 0);

		b2DestroyBitSet(color.bodySet);
		color.contactSims = new Array<b2ContactSim>();
		color.jointSims = new Array<b2JointSim>();
	}
}

// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
function b2AssignStaticColor(graph: b2ConstraintGraph, bodyId: i32): i32 {
	for (let i = B2_OVERFLOW_INDEX - 1; i >= 1; --i) {
		let color = graph.colors[i];
		if (b2GetBit(color.bodySet, <u32>bodyId)) {
			continue;
		}

		b2SetBitGrow(color.bodySet, <u32>bodyId);
		return i;
	}

	return B2_OVERFLOW_INDEX;
}

// Notice that a joint cannot share the same color as a contact between the same two bodies. This means I can solve contacts and
// joints in parallel with each other within each color. C writes this loop out in b2AddContactToGraph too.
function b2AssignJointColor(graph: b2ConstraintGraph, bodyIdA: i32, bodyIdB: i32, typeA: b2BodyType, typeB: b2BodyType): i32 {
	B2_ASSERT(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

	if (B2_FORCE_OVERFLOW == 0) {
		if (typeA == b2_dynamicBody && typeB == b2_dynamicBody) {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for (let i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i) {
				let color = graph.colors[i];
				if (b2GetBit(color.bodySet, <u32>bodyIdA) || b2GetBit(color.bodySet, <u32>bodyIdB)) {
					continue;
				}

				b2SetBitGrow(color.bodySet, <u32>bodyIdA);
				b2SetBitGrow(color.bodySet, <u32>bodyIdB);
				return i;
			}
		} else if (typeA == b2_dynamicBody) {
			return b2AssignStaticColor(graph, bodyIdA);
		} else if (typeB == b2_dynamicBody) {
			return b2AssignStaticColor(graph, bodyIdB);
		}
	}

	return B2_OVERFLOW_INDEX;
}

// Contacts are always created as non-touching. They get moved into the constraint
// graph once they are found to be touching.
export function b2AddContactToGraph(world: b2World, contactSim: ref b2ContactSim, contactId: i32): void {
	if (B2_ASSERT_ENABLED) {
		B2_ASSERT(contactSim.manifold.pointCount > 0);
		B2_ASSERT((contactSim.simFlags & b2_simTouchingFlag) != 0);
		B2_ASSERT((world.contacts[contactId].flags & b2_contactTouchingFlag) != 0);
	}

	let graph = world.constraintGraph;

	let bodyIdA = world.contacts[contactId].edges[0].bodyId;
	let bodyIdB = world.contacts[contactId].edges[1].bodyId;

	let typeA = world.bodies[bodyIdA].type;
	let typeB = world.bodies[bodyIdB].type;

	// Same color choice as b2AssignJointColor
	let colorIndex = b2AssignJointColor(graph, bodyIdA, bodyIdB, typeA, typeB);

	let color = graph.colors[colorIndex];
	world.contacts[contactId].colorIndex = colorIndex;
	world.contacts[contactId].localIndex = color.contactSims.length;

	let newIndex = color.contactSims.length;
	color.contactSims.push(contactSim);
	let colorSims = b2ArrayView(color.contactSims);
	ref newContact = colorSims[newIndex];
	let bodyView = b2ArrayView(world.bodies);

	// todo perhaps skip this if the contact is already awake

	if (typeA == b2_staticBody) {
		newContact.encodedBodySimA = b2EncodeBodySimIndex(bodyView[bodyIdA]);
		newContact.invMassA = 0.0;
		newContact.invIA = 0.0;
	} else {
		B2_ASSERT(world.bodies[bodyIdA].setIndex == b2_awakeSet);
		let awakeSet = world.solverSets[b2_awakeSet];

		let localIndex = world.bodies[bodyIdA].localIndex;
		newContact.encodedBodySimA = localIndex;

		newContact.invMassA = awakeSet.bodySims[localIndex].invMass;
		newContact.invIA = awakeSet.bodySims[localIndex].invInertia;
	}

	if (typeB == b2_staticBody) {
		newContact.encodedBodySimB = b2EncodeBodySimIndex(bodyView[bodyIdB]);
		newContact.invMassB = 0.0;
		newContact.invIB = 0.0;
	} else {
		B2_ASSERT(world.bodies[bodyIdB].setIndex == b2_awakeSet);
		let awakeSet = world.solverSets[b2_awakeSet];

		let localIndex = world.bodies[bodyIdB].localIndex;
		newContact.encodedBodySimB = localIndex;

		newContact.invMassB = awakeSet.bodySims[localIndex].invMass;
		newContact.invIB = awakeSet.bodySims[localIndex].invInertia;
	}
}

export function b2RemoveContactFromGraph(world: b2World, bodyIdA: i32, bodyIdB: i32, colorIndex: i32, localIndex: i32): void {
	let graph = world.constraintGraph;

	B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
	let color = graph.colors[colorIndex];

	if (colorIndex != B2_OVERFLOW_INDEX) {
		// This might clear a bit for a kinematic or static body, but this has no effect
		b2ClearBit(color.bodySet, <u32>bodyIdA);
		b2ClearBit(color.bodySet, <u32>bodyIdB);
	}

	let movedIndex = b2Array_RemoveSwap(color.contactSims, localIndex);
	if (movedIndex != B2_NULL_INDEX) {
		// Fix index on swapped contact
		let movedId = color.contactSims[localIndex].contactId;

		// Fix moved contact
		if (B2_ASSERT_ENABLED) {
			B2_ASSERT(world.contacts[movedId].setIndex == b2_awakeSet);
			B2_ASSERT(world.contacts[movedId].colorIndex == colorIndex);
			B2_ASSERT(world.contacts[movedId].localIndex == movedIndex);
		}
		world.contacts[movedId].localIndex = localIndex;
	}
}

export function b2CreateJointInGraph(world: b2World, jointId: i32): i32 {
	let graph = world.constraintGraph;

	let bodyIdA = world.joints[jointId].edges[0].bodyId;
	let bodyIdB = world.joints[jointId].edges[1].bodyId;

	let colorIndex = b2AssignJointColor(graph, bodyIdA, bodyIdB, world.bodies[bodyIdA].type, world.bodies[bodyIdB].type);

	let localIndex = b2Array_Emplace(graph.colors[colorIndex].jointSims);

	world.joints[jointId].colorIndex = colorIndex;
	world.joints[jointId].localIndex = localIndex;
	return localIndex;
}

export function b2AddJointToGraph(world: b2World, jointSim: ref b2JointSim, jointId: i32): void {
	let localIndex = b2CreateJointInGraph(world, jointId);
	world.constraintGraph.colors[world.joints[jointId].colorIndex].jointSims[localIndex] = jointSim;
}

export function b2RemoveJointFromGraph(world: b2World, bodyIdA: i32, bodyIdB: i32, colorIndex: i32, localIndex: i32): void {
	let graph = world.constraintGraph;

	B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
	let color = graph.colors[colorIndex];

	if (colorIndex != B2_OVERFLOW_INDEX) {
		// May clear static bodies, no effect
		b2ClearBit(color.bodySet, <u32>bodyIdA);
		b2ClearBit(color.bodySet, <u32>bodyIdB);
	}

	let movedIndex = b2Array_RemoveSwap(color.jointSims, localIndex);
	if (movedIndex != B2_NULL_INDEX) {
		// Fix moved joint
		let movedId = color.jointSims[localIndex].jointId;
		if (B2_ASSERT_ENABLED) {
			B2_ASSERT(world.joints[movedId].setIndex == b2_awakeSet);
			B2_ASSERT(world.joints[movedId].colorIndex == colorIndex);
			B2_ASSERT(world.joints[movedId].localIndex == movedIndex);
		}
		world.joints[movedId].localIndex = localIndex;
	}
}
