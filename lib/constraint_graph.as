// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/constraint_graph.c

import { b2BitSet } from "./bitset";
import { B2_GRAPH_COLOR_COUNT } from "./constants";
import { b2ContactSim } from "./contact";
import { b2JointSim } from "./joint";
import { b2ContactConstraint } from "./contact_solver";

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
