// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver.c

import { b2AtomicInt, b2AtomicU32 } from "./core";
import { B2_PI } from "./math_functions";
import { b2BodySim, b2BodyState } from "./body";
import { b2ContactSim } from "./contact";
import { b2JointSim } from "./joint";
import { b2ConstraintGraph } from "./constraint_graph";
import { b2World } from "./physics_world";

// Solver stages. Prepare joints and prepare contacts are split up
// because there is no need to store joint impulses.
export type b2SolverStageType = i32;
export const b2_stagePrepareJoints: b2SolverStageType = 0;
export const b2_stagePrepareContacts: b2SolverStageType = 1;
export const b2_stageIntegrateVelocities: b2SolverStageType = 2;
export const b2_stageWarmStart: b2SolverStageType = 3;
export const b2_stageSolve: b2SolverStageType = 4;
export const b2_stageIntegratePositions: b2SolverStageType = 5;
export const b2_stageRelax: b2SolverStageType = 6;
export const b2_stageRestitution: b2SolverStageType = 7;
export const b2_stageStoreImpulses: b2SolverStageType = 8;

export type b2SolverBlockType = i32;
export const b2_bodyBlock: b2SolverBlockType = 0;
export const b2_jointBlock: b2SolverBlockType = 1;
export const b2_contactBlock: b2SolverBlockType = 2;
export const b2_graphJointBlock: b2SolverBlockType = 3;
export const b2_graphContactBlock: b2SolverBlockType = 4;

// Solver block describes a multithreaded unit of work.
export struct b2SolverBlock {
	startIndex: i32 = 0;
	count: u16 = 0;
	// b2SolverBlockType
	blockType: u8 = 0;
	colorIndex: u8 = 0;
}

// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
// monotonically allowing the solver block to be re-used across sub-steps.
export struct b2SyncBlock {
	block: b2SolverBlock = b2SolverBlock();
	syncIndex: b2AtomicInt = b2AtomicInt();
}

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same
// instance each iteration.
export class b2SolverStage {
	// C points into the step's block array; blocks start at blockStart in the solver scratch
	blocks: StaticArray<b2SyncBlock> = new StaticArray<b2SyncBlock>(0);
	blockStart: i32 = 0;
	type: b2SolverStageType = 0;
	blockCount: i32 = 0;
	colorIndex: u8 = 0;
	completionCount: b2AtomicInt = b2AtomicInt();
}

// Constraint softness
export struct b2Softness {
	biasRate: f32 = 0;
	massScale: f32 = 0;
	impulseScale: f32 = 0;
}

// Prepare/store run as a flat parallel-for over the whole wide-constraint
// range. Each span maps a slice of that range back to the color's
// contact sims so workers can decode flat wide-slot indices without touching
// graph state. The spans array has one entry per active color plus a sentinel
// whose start == wideContactCount.
export class b2ContactPrepareSpan {
	start: i32 = 0;
	count: i32 = 0;
	contacts: Array<b2ContactSim> = b2_emptyContactSims;
}

// Similar for the narrow phase.
export class b2ContactCollideSpan {
	start: i32 = 0;
	contacts: Array<b2ContactSim> = b2_emptyContactSims;
}

// Similar for joints
export class b2JointPrepareSpan {
	start: i32 = 0;
	count: i32 = 0;
	joints: Array<b2JointSim> = b2_emptyJointSims;
}

export const b2_emptyContactSims = new Array<b2ContactSim>();
export const b2_emptyJointSims = new Array<b2JointSim>();
export const b2_emptyBodyStates = new Array<b2BodyState>();
export const b2_emptyBodySims = new Array<b2BodySim>();

// Context for a time step. Recreated each time step.
export class b2StepContext {
	// time step
	dt: f32 = 0;

	// inverse time step (0 if dt == 0).
	inv_dt: f32 = 0;

	// sub-step
	h: f32 = 0;
	inv_h: f32 = 0;

	subStepCount: i32 = 0;

	contactSoftness: b2Softness = b2Softness();
	staticSoftness: b2Softness = b2Softness();

	restitutionThreshold: f32 = 0;
	maxLinearVelocity: f32 = 0;

	world: b2World;
	graph: b2ConstraintGraph;

	// shortcut to body states from awake set
	states: Array<b2BodyState> = b2_emptyBodyStates;

	// shortcut to body sims from awake set
	sims: Array<b2BodySim> = b2_emptyBodySims;

	// array of all shape ids for shapes that have enlarged AABBs
	enlargedShapes: StaticArray<i32> = new StaticArray<i32>(0);
	enlargedShapeCount: i32 = 0;

	// Array of bullet bodies that need continuous collision handling
	bulletBodies: StaticArray<i32> = new StaticArray<i32>(0);
	bulletBodyCount: b2AtomicInt = b2AtomicInt();

	anyRestitution: b2AtomicInt = b2AtomicInt();

	// Graph color spans for the narrow-phase, includes non-touching contacts as well.
	collideSpans: StaticArray<b2ContactCollideSpan> = new StaticArray<b2ContactCollideSpan>(0);

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. The wide constraints live in the solver scratch;
	// per-color slices start at colors[i].wideConstraintStart.
	contactPrepareSpans: StaticArray<b2ContactPrepareSpan> = new StaticArray<b2ContactPrepareSpan>(0);
	wideContactCount: i32 = 0;

	jointPrepareSpans: StaticArray<b2JointPrepareSpan> = new StaticArray<b2JointPrepareSpan>(0);
	jointCount: i32 = 0;

	activeColorCount: i32 = 0;
	workerCount: i32 = 0;

	stages: StaticArray<b2SolverStage> = new StaticArray<b2SolverStage>(0);
	stageCount: i32 = 0;
	enableWarmStarting: bool = false;

	// This atomic is central to multi-threaded solver task synchronization.
	// sync index (16-bits) | stage type (16-bits)
	atomicSyncBits: b2AtomicU32 = b2AtomicU32();

	mainClaimed: b2AtomicInt = b2AtomicInt();

	constructor(world: b2World) {
		this.world = world;
		this.graph = world.constraintGraph;
	}
}

export function b2MakeSoft(hertz: f32, zeta: f32, h: f32): b2Softness {
	if (hertz == 0.0) {
		return b2Softness();
	}

	let omega: f32 = 2.0 * B2_PI * hertz;
	let a1: f32 = 2.0 * zeta + h * omega;
	let a2: f32 = h * omega * a1;
	let a3: f32 = 1.0 / (1.0 + a2);

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	let s = b2Softness();
	s.biasRate = omega / a1;
	s.massScale = a2 * a3;
	s.impulseScale = a3;
	return s;
}

// functions from solver.c follow
