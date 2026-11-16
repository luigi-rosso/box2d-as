// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver.c

import { Parallel } from "rive/parallel";

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX, UINT16_MAX, UINT32_MAX, UINT8_MAX, b2AtomicInt, b2AtomicU32, b2GetMilliseconds, b2GetMillisecondsAndReset, b2GetTicks, b2Log } from "./core";
import { B2_GRAPH_COLOR_COUNT, B2_MAX_ROTATION, B2_MAX_TASKS, B2_MAX_WORKERS, B2_LINEAR_SLOP, B2_SPECULATIVE_DISTANCE, B2_TIME_TO_SLEEP } from "./constants";
import {
	B2_PI,
	b2AABB,
	b2AABB_Contains,
	b2AABB_Union,
	b2AbsFloat,
	b2Add,
	b2Cross,
	b2Dot,
	b2GetLengthAndNormalize,
	b2IntegrateRotation,
	b2IsValidFloat,
	b2IsValidVec2,
	b2Length,
	b2Lerp,
	b2MakeWorldTransform,
	b2MaxFloat,
	b2MinInt,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2NLerp,
	b2Neg,
	b2NormalizeRot,
	b2OffsetPos,
	b2Pos,
	b2Rot,
	b2RotateVector,
	b2Rot_identity,
	b2Sub,
	b2ToRelativeTransform,
	b2Transform,
	b2TransformPoint,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2Sweep, b2TOIInput } from "./collision";
import { b2ContactHitEvent, b2JointEvent, b2BodyMoveEvent, B2_DEFAULT_MASK_BITS, b2_chainSegmentShape, b2_dynamicBody, b2_kinematicBody, b2_staticBody } from "./types";
import { b2BodyId, b2ContactId, b2JointId, b2ShapeId } from "./id";
import { b2GetBit, b2InPlaceUnion, b2SetBit, b2SetBitCountAndClear } from "./bitset";
import { b2CTZ64 } from "./ctz";
import { b2GetIdCapacity } from "./id_pool";
import { b2OffsetAABB } from "./aabb";
import { b2ComputeTimeOfImpact, b2MakeProxy } from "./distance";
import {
	B2_TREE_STACK_SIZE,
	b2DynamicTree_EnlargeProxy,
	b2DynamicTree_GetProxyBounds,
	b2DynamicTree_OverlapAABBWith,
	b2DynamicTree_Refit,
} from "./dynamic_tree";
import { B2_PROXY_ID, b2BroadPhase_MarkProxyMoved, b2FinishTreeUpdate, b2ValidateBroadphase, b2ValidateNoEnlarged } from "./broad_phase";
import {
	b2ArrayData,
	b2Body,
	b2BodySim,
	b2BodyState,
	b2GetBodySim,
	b2GetBodySimAddress,
	b2MakeRelativeSweepAt,
	b2ShouldBodiesCollide,
	b2_allowFastRotation,
	b2_bodyTransientFlags,
	b2_dirtyMass,
	b2_enableSleep,
	b2_enlargeBulletBounds,
	b2_hadTimeOfImpact,
	b2_isBullet,
	b2_isFast,
	b2_isSpeedCapped,
	b2_lockAngularZ,
	b2_lockLinearX,
	b2_lockLinearY,
} from "./body";
import { b2ContactSim } from "./contact";
import {
	b2JointSim,
	b2PrepareJointsTask,
	b2PrepareJoints_Overflow,
	b2SolveJointsTask,
	b2SolveJoints_Overflow,
	b2WarmStartJointsTask,
	b2WarmStartJoints_Overflow,
} from "./joint";
import {
	b2ApplyRestitution_Overflow,
	b2ApplyRestitution_Wide,
	b2ContactConstraint,
	b2ContactConstraintWide,
	b2GetWideContactConstraintByteCount,
	b2PrepareContacts_Overflow,
	b2PrepareContacts_Wide,
	b2PushContacts_Overflow,
	b2PushContacts_Wide,
	b2SolveContacts_Overflow,
	b2SolveContacts_Wide,
	b2StoreImpulses_Overflow,
	b2StoreImpulses_Wide,
	b2WarmStartContacts_Overflow,
	b2WarmStartContacts_Wide,
} from "./contact_solver";
import { B2_OVERFLOW_INDEX, b2ConstraintGraph } from "./constraint_graph";
import { b2SplitIslandTask } from "./island";
import { b2TrySleepIsland, b2_awakeSet } from "./solver_set";
import { b2ComputeFatShapeAABB, b2ComputeShapeAABB, b2ComputeShapeExtent, b2GetShapeCentroid, b2MakeShapeDistanceProxy, b2ShouldShapesCollide } from "./shape";
import { b2SensorHit, b2Visitor } from "./sensor";
import { b2ParallelFor } from "./parallel_for";
import { b2TaskContext, b2World } from "./physics_world";

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
const b2_emptyWideConstraints = new StaticArray<b2ContactConstraintWide>(0);
const b2_emptyIndices = new StaticArray<i32>(0);

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
	bulletBodies: StaticArray<i32> = b2_emptyIndices;
	bulletBodyCount: b2AtomicInt = b2AtomicInt();

	anyRestitution: b2AtomicInt = b2AtomicInt();

	// Graph color spans for the narrow-phase, includes non-touching contacts as well.
	collideSpans: StaticArray<b2ContactCollideSpan> = new StaticArray<b2ContactCollideSpan>(0);

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. wideContactConstraints is the solver scratch array;
	// per-color slices start at colors[i].wideConstraintStart.
	wideContactConstraints: StaticArray<b2ContactConstraintWide> = b2_emptyWideConstraints;
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

	// How many workers sleep on the sync bits, which only the parts of a parallel call do
	sleepingWorkers: b2AtomicInt = b2AtomicInt();

	// Whether parallel parts finalizing bodies sweep the fast ones too, or leave them to the main lane
	parallelContinuous: bool = false;

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

// these are useful for solver testing
const ITERATIONS: i32 = 1;
const RELAX_ITERATIONS: i32 = 1;

// C's atomics, on the value's address. The parts of a parallel call run them as atomics; elsewhere one thread runs
// the step and they are a plain read and write.
@inline unsafe function b2AtomicLoadInt(address: usize): i32 {
	return Parallel.inPart() ? atomic.load<i32>(address) : load<i32>(address);
}

@inline unsafe function b2AtomicStoreInt(address: usize, value: i32): void {
	if (Parallel.inPart()) {
		atomic.store<i32>(address, value);
	} else {
		store<i32>(address, value);
	}
}

@inline unsafe function b2AtomicFetchAddInt(address: usize, increment: i32): i32 {
	if (Parallel.inPart()) {
		return atomic.add<i32>(address, increment);
	}

	let value = load<i32>(address);
	store<i32>(address, value + increment);
	return value;
}

@inline unsafe function b2AtomicCompareExchangeInt(address: usize, expected: i32, desired: i32): bool {
	if (Parallel.inPart()) {
		return atomic.cmpxchg<i32>(address, expected, desired) == expected;
	}

	if (load<i32>(address) == expected) {
		store<i32>(address, desired);
		return true;
	}
	return false;
}

@inline unsafe function b2AtomicLoadU32(address: usize): u32 {
	return <u32>b2AtomicLoadInt(address);
}

@inline unsafe function b2AtomicStoreU32(address: usize, value: u32): void {
	b2AtomicStoreInt(address, <i32>value);
}

// Where C's atomics live: in the step context, in a stage and in a sync block
const SC_SYNCBITS = offsetof<b2StepContext>("atomicSyncBits");
const SC_MAINCLAIMED = offsetof<b2StepContext>("mainClaimed");
const SC_BULLETBODYCOUNT = offsetof<b2StepContext>("bulletBodyCount");
const SC_ANYRESTITUTION = offsetof<b2StepContext>("anyRestitution");
const SC_SLEEPINGWORKERS = offsetof<b2StepContext>("sleepingWorkers");

@inline unsafe function b2StepAtomic(context: b2StepContext, offset: usize): usize {
	return changetype<usize>(context) + offset;
}

@inline unsafe function b2CompletionCount(stage: b2SolverStage): usize {
	return changetype<usize>(stage) + offsetof<b2SolverStage>("completionCount");
}

@inline unsafe function b2SyncIndex(blocks: StaticArray<b2SyncBlock>, index: i32): usize {
	return changetype<usize>(blocks) + <usize>index * sizeof<b2SyncBlock>() + offsetof<b2SyncBlock>("syncIndex");
}

function b2Pause(): void {}

// C yields the core to another thread here. A part of a parallel call cannot, so a worker that has yielded this many
// times in one wait sleeps on the sync bits instead, until b2PublishSyncBits wakes it or this long has passed, in ns.
const B2_WORKER_SPIN_YIELDS: i32 = 2048;
const B2_WORKER_SLEEP_NANOS: i64 = 1000000;

@inline unsafe function b2YieldWorker(context: b2StepContext, syncBits: u32, yieldCount: i32): void {
	if (Parallel.inPart() && yieldCount > B2_WORKER_SPIN_YIELDS) {
		let sleepingWorkers = b2StepAtomic(context, SC_SLEEPINGWORKERS);
		atomic.add<i32>(sleepingWorkers, 1);
		atomic.wait<u32>(b2StepAtomic(context, SC_SYNCBITS), syncBits, B2_WORKER_SLEEP_NANOS);
		atomic.sub<i32>(sleepingWorkers, 1);
	}
}

// C's store of the sync bits, which here also wakes the workers that sleep on them. A worker that is about to sleep
// on the old bits finds them changed and does not.
@inline unsafe function b2PublishSyncBits(context: b2StepContext, syncBits: u32): void {
	let address = b2StepAtomic(context, SC_SYNCBITS);
	b2AtomicStoreU32(address, syncBits);
	if (Parallel.inPart() && atomic.load<i32>(b2StepAtomic(context, SC_SLEEPINGWORKERS)) != 0) {
		atomic.notify(address, -1);
	}
}

class b2WorkerContext {
	context: b2StepContext? = null;
	workerIndex: i32 = 0;
	// Handle from the task system, 0 for C's NULL; tasks run inline
	userTask: u64 = 0;
}

// The integrations and finalize walk the state, sim, body and move event arrays through addresses as C walks them
// through pointers. Their indices stay below the awake counts and the move event count b2Solve sized, and bodyId is a
// live body, so every address is inside its array.
const BS_V = offsetof<b2BodyState>("linearVelocity");
const BS_W = offsetof<b2BodyState>("angularVelocity");
const BS_FLAGS = offsetof<b2BodyState>("flags");
const BS_DP = offsetof<b2BodyState>("deltaPosition");
const BS_DQ = offsetof<b2BodyState>("deltaRotation");
const SIM_P = offsetof<b2BodySim>("transform") + offsetof<b2Transform>("p");
const SIM_Q = offsetof<b2BodySim>("transform") + offsetof<b2Transform>("q");
const SIM_TRANSFORM = offsetof<b2BodySim>("transform");
const SIM_CENTER = offsetof<b2BodySim>("center");
const SIM_ROTATION0 = offsetof<b2BodySim>("rotation0");
const SIM_CENTER0 = offsetof<b2BodySim>("center0");
const SIM_LOCALCENTER = offsetof<b2BodySim>("localCenter");
const SIM_FORCE = offsetof<b2BodySim>("force");
const SIM_TORQUE = offsetof<b2BodySim>("torque");
const SIM_INVMASS = offsetof<b2BodySim>("invMass");
const SIM_INVINERTIA = offsetof<b2BodySim>("invInertia");
const SIM_MINEXTENT = offsetof<b2BodySim>("minExtent");
const SIM_MAXEXTENT = offsetof<b2BodySim>("maxExtent");
const SIM_LINEARDAMPING = offsetof<b2BodySim>("linearDamping");
const SIM_ANGULARDAMPING = offsetof<b2BodySim>("angularDamping");
const SIM_GRAVITYSCALE = offsetof<b2BodySim>("gravityScale");
const SIM_BODYID = offsetof<b2BodySim>("bodyId");
const SIM_FLAGS = offsetof<b2BodySim>("flags");
const B_USERDATA = offsetof<b2Body>("userData");
const B_HEADSHAPEID = offsetof<b2Body>("headShapeId");
const B_ISLANDID = offsetof<b2Body>("islandId");
const B_SLEEPTHRESHOLD = offsetof<b2Body>("sleepThreshold");
const B_SLEEPTIME = offsetof<b2Body>("sleepTime");
const B_SAFETYFACTOR = offsetof<b2Body>("safetyFactor");
const B_BODYMOVEINDEX = offsetof<b2Body>("bodyMoveIndex");
const B_FLAGS = offsetof<b2Body>("flags");
const B_TYPE = offsetof<b2Body>("type");
const B_GENERATION = offsetof<b2Body>("generation");
const ME_USERDATA = offsetof<b2BodyMoveEvent>("userData");
const ME_TRANSFORM = offsetof<b2BodyMoveEvent>("transform");
const ME_BODYID = offsetof<b2BodyMoveEvent>("bodyId");
const ME_FELLASLEEP = offsetof<b2BodyMoveEvent>("fellAsleep");

// Integrate velocities and apply damping
@noinline
unsafe function b2IntegrateVelocitiesTask(block: b2SolverBlock, context: b2StepContext): void {
	let states = b2ArrayData(context.states);
	let sims = b2ArrayData(context.sims);

	let gravity = context.world.gravity;
	let h = context.h;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let sim = sims + <usize>i * sizeof<b2BodySim>();
		let state = states + <usize>i * sizeof<b2BodyState>();

		let v = load<b2Vec2>(state + BS_V);
		let w = load<f32>(state + BS_W);
		let invMass = load<f32>(sim + SIM_INVMASS);

		// Apply forces, torque, gravity, and damping
		// Apply damping.
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		let linearDamping: f32 = 1.0 / (1.0 + h * load<f32>(sim + SIM_LINEARDAMPING));
		let angularDamping: f32 = 1.0 / (1.0 + h * load<f32>(sim + SIM_ANGULARDAMPING));

		// Gravity scale will be zero for kinematic bodies
		let gravityScale: f32 = invMass > 0.0 ? load<f32>(sim + SIM_GRAVITYSCALE) : <f32>0.0;

		// lvd = h * im * f + h * g
		let linearVelocityDelta = b2Add(b2MulSV(h * invMass, load<b2Vec2>(sim + SIM_FORCE)), b2MulSV(h * gravityScale, gravity));
		let angularVelocityDelta = h * load<f32>(sim + SIM_INVINERTIA) * load<f32>(sim + SIM_TORQUE);

		v = b2MulAdd(linearVelocityDelta, linearDamping, v);
		w = angularVelocityDelta + angularDamping * w;

		store<b2Vec2>(state + BS_V, v);
		store<f32>(state + BS_W, w);
	}
}

@noinline
unsafe function b2IntegratePositionsTask(block: b2SolverBlock, context: b2StepContext): void {
	let states = b2ArrayData(context.states);
	let h = context.h;
	let maxLinearSpeed = context.maxLinearVelocity;
	let maxAngularSpeed = B2_MAX_ROTATION * context.inv_dt;
	let maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	let maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let state = states + <usize>i * sizeof<b2BodyState>();

		let v = load<b2Vec2>(state + BS_V);
		let w = load<f32>(state + BS_W);
		let flags = load<u32>(state + BS_FLAGS);

		// Motion locks - these can be viewed as a constraint that comes last
		v.x = (flags & b2_lockLinearX) != 0 ? <f32>0.0 : v.x;
		v.y = (flags & b2_lockLinearY) != 0 ? <f32>0.0 : v.y;
		w = (flags & b2_lockAngularZ) != 0 ? <f32>0.0 : w;

		// Clamp to max linear speed
		if (b2Dot(v, v) > maxLinearSpeedSquared) {
			let ratio = maxLinearSpeed / b2Length(v);
			v = b2MulSV(ratio, v);
			flags |= b2_isSpeedCapped;
		}

		// Clamp to max angular speed
		if (w * w > maxAngularSpeedSquared && (flags & b2_allowFastRotation) == 0) {
			let ratio = maxAngularSpeed / b2AbsFloat(w);
			w *= ratio;
			flags |= b2_isSpeedCapped;
		}

		store<u32>(state + BS_FLAGS, flags);
		store<b2Vec2>(state + BS_V, v);
		store<f32>(state + BS_W, w);
		store<b2Vec2>(state + BS_DP, b2MulAdd(load<b2Vec2>(state + BS_DP), h, v));
		store<b2Rot>(state + BS_DQ, b2IntegrateRotation(load<b2Rot>(state + BS_DQ), h * w));
	}
}

const B2_MAX_CONTINUOUS_SENSOR_HITS: i32 = 8;

// C's stack context of b2SolveContinuous. Each worker's task context keeps one, with its world; the sweeps never nest.
export class b2ContinuousContext {
	world: b2World? = null;
	// C's fastBodySim pointer; the awake sims do not move during the sweep
	fastBodySim: usize = 0;
	// C's fastShape pointer as the shape index
	fastShapeId: i32 = 0;
	centroid1: b2Vec2 = b2Vec2();
	centroid2: b2Vec2 = b2Vec2();
	sweep: b2Sweep = b2Sweep();
	base: b2Pos = b2Vec2();
	fraction: f32 = 0;
	// B2_MAX_CONTINUOUS_SENSOR_HITS
	sensorHits: FixedArray<b2SensorHit, 8>;
	sensorFractions: FixedArray<f32, 8>;
	sensorCount: i32 = 0;
	// C's stack of the tree query
	queryStack: StaticArray<i32> = new StaticArray<i32>(B2_TREE_STACK_SIZE);

	// The tree query visitor
	visit(proxyId: i32, userData: u64): bool {
		return b2ContinuousQueryCallback(proxyId, userData, this);
	}
}

const B2_CORE_FRACTION: f32 = 0.25;

// This is called from b2DynamicTree_OverlapAABB for continuous collision
unsafe function b2ContinuousQueryCallback(proxyId: i32, userData: u64, continuousContext: b2ContinuousContext): bool {
	let shapeId = <i32>userData;
	let world = continuousContext.world!;
	let fastShapeId = continuousContext.fastShapeId;
	let fastBodySim = continuousContext.fastBodySim;

	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[fastShapeId].sensorIndex == B2_NULL_INDEX);

	// Skip same shape
	if (shapeId == world.shapes[fastShapeId].id) {
		return true;
	}

	// Skip same body
	if (world.shapes[shapeId].bodyId == world.shapes[fastShapeId].bodyId) {
		return true;
	}

	// Skip sensors unless the shapes want sensor events
	let isSensor = world.shapes[shapeId].sensorIndex != B2_NULL_INDEX;
	if (isSensor && (world.shapes[shapeId].enableSensorEvents == false || world.shapes[fastShapeId].enableSensorEvents == false)) {
		return true;
	}

	// Skip filtered shapes
	let canCollide = b2ShouldShapesCollide(world.shapes[fastShapeId].filter, world.shapes[shapeId].filter);
	if (canCollide == false) {
		return true;
	}

	let bodyId = world.shapes[shapeId].bodyId;

	let bodySim = b2GetBodySimAddress(world, bodyId);
	if (B2_ASSERT_ENABLED) {
		B2_ASSERT(world.bodies[bodyId].type == b2_staticBody || (load<u32>(fastBodySim + SIM_FLAGS) & b2_isBullet) != 0);
	}

	// Skip bullets
	// Warning: it is only safe to read flags from other bodies, not body sims because there are
	// body sim flag writes in the continuous solver.
	if ((world.bodies[bodyId].flags & b2_isBullet) != 0) {
		return true;
	}

	// Skip filtered bodies
	let fastBodyId = load<i32>(fastBodySim + SIM_BODYID);
	canCollide = b2ShouldBodiesCollide(world, fastBodyId, bodyId);
	if (canCollide == false) {
		return true;
	}

	// Custom user filtering. A part cannot call a function value, so a world with one sweeps on the main lane.
	if (!Parallel.inPart() && (world.shapes[shapeId].enableCustomFiltering || world.shapes[fastShapeId].enableCustomFiltering)) {
		let customFilterFcn = world.customFilterFcn;
		if (customFilterFcn != null) {
			let idA = b2ShapeId(shapeId + 1, world.worldId, world.shapes[shapeId].generation);
			let idB = b2ShapeId(fastShapeId + 1, world.worldId, world.shapes[fastShapeId].generation);
			canCollide = customFilterFcn(idA, idB);
			if (canCollide == false) {
				return true;
			}
		}
	}

	// Early out on fast parallel movement over a chain shape.
	if (world.shapes[shapeId].type == b2_chainSegmentShape) {
		let transform = b2ToRelativeTransform(load<b2Transform>(bodySim + offsetof<b2BodySim>("transform")), continuousContext.base);
		let p1 = b2TransformPoint(transform, world.shapes[shapeId].chainSegment.segment.point1);
		let p2 = b2TransformPoint(transform, world.shapes[shapeId].chainSegment.segment.point2);
		let e = b2Sub(p2, p1);
		let length: FixedArray<f32, 1>;
		e = b2GetLengthAndNormalize(length[0], e);
		if (length[0] > B2_LINEAR_SLOP()) {
			let c1 = continuousContext.centroid1;
			let separation1 = b2Cross(b2Sub(c1, p1), e);
			let c2 = continuousContext.centroid2;
			let separation2 = b2Cross(b2Sub(c2, p1), e);

			let coreDistance = B2_CORE_FRACTION * load<f32>(fastBodySim + SIM_MINEXTENT);

			if (separation1 < 0.0 || (separation1 - separation2 < coreDistance && separation2 > coreDistance)) {
				// Minimal clipping
				return true;
			}
		}
	}

	let input = b2TOIInput();
	input.proxyA = b2MakeShapeDistanceProxy(world, shapeId);
	input.proxyB = b2MakeShapeDistanceProxy(world, fastShapeId);
	input.sweepA = b2MakeRelativeSweepAt(bodySim, continuousContext.base);
	input.sweepB = continuousContext.sweep;
	input.maxFraction = continuousContext.fraction;

	let output = b2ComputeTimeOfImpact(input);
	if (isSensor) {
		// Only accept a sensor hit that is sooner than the current solid hit.
		if (output.fraction <= continuousContext.fraction && continuousContext.sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS) {
			let index = continuousContext.sensorCount;

			// The hit shape is a sensor
			let sensorHit = b2SensorHit();
			sensorHit.sensorId = shapeId;
			sensorHit.visitorId = fastShapeId;

			continuousContext.sensorHits[index] = sensorHit;
			continuousContext.sensorFractions[index] = output.fraction;
			continuousContext.sensorCount += 1;
		}
	} else {
		let hitFraction = continuousContext.fraction;
		let didHit = false;

		if (0.0 < output.fraction && output.fraction < continuousContext.fraction) {
			hitFraction = output.fraction;
			didHit = true;
		} else if (0.0 == output.fraction) {
			// fallback to TOI of a small circle around the fast shape centroid
			let centroid = b2GetShapeCentroid(world, fastShapeId);
			let extent = b2ComputeShapeExtent(world, fastShapeId, centroid);
			let radius = B2_CORE_FRACTION * extent.minExtent;
			let points: FixedArray<b2Vec2, 8>;
			points[0] = centroid;
			input.proxyB = b2MakeProxy(points, 1, radius);
			output = b2ComputeTimeOfImpact(input);
			if (0.0 < output.fraction && output.fraction < continuousContext.fraction) {
				hitFraction = output.fraction;
				didHit = true;
			}
		}

		if (
			!Parallel.inPart() &&
			didHit &&
			(world.shapes[shapeId].enablePreSolveEvents || world.shapes[fastShapeId].enablePreSolveEvents) &&
			world.preContinuousFcn != null
		) {
			let shapeIdA = b2ShapeId(shapeId + 1, world.worldId, world.shapes[shapeId].generation);
			let shapeIdB = b2ShapeId(fastShapeId + 1, world.worldId, world.shapes[fastShapeId].generation);

			// TOI runs in the base frame, lift the hit point back to world for the callback
			let worldPoint = b2OffsetPos(continuousContext.base, output.point);
			didHit = world.preContinuousFcn!(shapeIdA, shapeIdB, worldPoint, output.normal);
		}

		if (didHit) {
			continuousContext.fraction = hitFraction;
		}
	}

	// Continue query
	return true;
}

// The fat AABB margin of an AABB
@inline function b2InflateAABB(aabb: b2AABB, margin: f32): b2AABB {
	let fatAABB: b2AABB;
	fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
	fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
	fatAABB.upperBound.x = aabb.upperBound.x + margin;
	fatAABB.upperBound.y = aabb.upperBound.y + margin;
	return fatAABB;
}

// Continuous collision of dynamic versus static
unsafe function b2SolveContinuous(world: b2World, bodySimIndex: i32, taskContext: b2TaskContext, dt: f32): void {
	let awakeSet = world.solverSets[b2_awakeSet];
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= bodySimIndex && bodySimIndex < awakeSet.bodySims.length);
	let fastBodySim = b2ArrayData(awakeSet.bodySims) + <usize>bodySimIndex * sizeof<b2BodySim>();

	// Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
	let base = load<b2Vec2>(fastBodySim + SIM_CENTER0);
	let sweep = b2MakeRelativeSweepAt(fastBodySim, base);

	let xf1 = b2Transform();
	xf1.q = sweep.q1;
	xf1.p = b2Sub(sweep.c1, b2RotateVector(sweep.q1, sweep.localCenter));

	let xf2 = b2Transform();
	xf2.q = sweep.q2;
	xf2.p = b2Sub(sweep.c2, b2RotateVector(sweep.q2, sweep.localCenter));

	let staticTree = world.broadPhase.trees[b2_staticBody];
	let kinematicTree = world.broadPhase.trees[b2_kinematicBody];
	let dynamicTree = world.broadPhase.trees[b2_dynamicBody];
	let fastBodyId = load<i32>(fastBodySim + SIM_BODYID);

	let context = taskContext.continuousContext;
	context.sweep = sweep;
	context.base = base;
	context.fastBodySim = fastBodySim;
	context.fraction = 1.0;
	context.sensorCount = 0;

	let isBullet = (load<u32>(fastBodySim + SIM_FLAGS) & b2_isBullet) != 0;

	let shapeId = world.bodies[fastBodyId].headShapeId;
	while (shapeId != B2_NULL_INDEX) {
		let fastShapeId = shapeId;
		shapeId = world.shapes[fastShapeId].nextShapeId;

		context.fastShapeId = fastShapeId;
		context.centroid1 = b2TransformPoint(xf1, world.shapes[fastShapeId].localCentroid);
		context.centroid2 = b2TransformPoint(xf2, world.shapes[fastShapeId].localCentroid);

		let box1 = world.shapes[fastShapeId].aabb;

		// xf2 is in the base frame, compute the tight box near the origin then lift to world
		let box2 = b2OffsetAABB(b2ComputeShapeAABB(world, fastShapeId, b2MakeWorldTransform(xf2)), base);

		// Store this to avoid double computation in the case there is no impact event
		world.shapes[fastShapeId].aabb = box2;

		// No continuous collision for sensors (but still need the updated bounds)
		if (world.shapes[fastShapeId].sensorIndex != B2_NULL_INDEX) {
			continue;
		}

		let sweptBox = b2AABB_Union(box1, box2);

		let stack = context.queryStack;
		b2DynamicTree_OverlapAABBWith<b2ContinuousContext>(staticTree, sweptBox, B2_DEFAULT_MASK_BITS, context, stack);

		if (isBullet) {
			b2DynamicTree_OverlapAABBWith<b2ContinuousContext>(kinematicTree, sweptBox, B2_DEFAULT_MASK_BITS, context, stack);
			b2DynamicTree_OverlapAABBWith<b2ContinuousContext>(dynamicTree, sweptBox, B2_DEFAULT_MASK_BITS, context, stack);
		}
	}

	let speculativeDistance = B2_SPECULATIVE_DISTANCE();

	if (context.fraction < 1.0) {
		// Handle time of impact event
		let q = b2NLerp(sweep.q1, sweep.q2, context.fraction);
		let c = b2Lerp(sweep.c1, sweep.c2, context.fraction);
		let origin = b2Sub(c, b2RotateVector(q, sweep.localCenter));

		// Advance body, lifting the base frame result back to world
		let transform = b2Transform(b2OffsetPos(base, origin), q);
		let center = b2OffsetPos(base, c);
		store<b2Transform>(fastBodySim + SIM_TRANSFORM, transform);
		store<b2Vec2>(fastBodySim + SIM_CENTER, center);
		store<b2Rot>(fastBodySim + SIM_ROTATION0, q);
		store<b2Vec2>(fastBodySim + SIM_CENTER0, center);

		// Warning: writing to the body sim flags means we should not read from other body sim flags in this function.
		store<u32>(fastBodySim + SIM_FLAGS, load<u32>(fastBodySim + SIM_FLAGS) | b2_hadTimeOfImpact);

		// Timeloss means there is a lost gravity contribution. Other forces and torques are ignored for now.
		let fastBodyState = b2ArrayData(awakeSet.bodyStates) + <usize>bodySimIndex * sizeof<b2BodyState>();
		let stateFlags = load<u32>(fastBodyState + BS_FLAGS);
		let v = load<b2Vec2>(fastBodyState + BS_V);
		let timeLoss = (1.0 - context.fraction) * dt;
		let dv = b2MulSV(-timeLoss * load<f32>(fastBodySim + SIM_GRAVITYSCALE), world.gravity);
		dv.x = (stateFlags & b2_lockLinearX) != 0 ? <f32>0.0 : dv.x;
		dv.y = (stateFlags & b2_lockLinearY) != 0 ? <f32>0.0 : dv.y;
		store<b2Vec2>(fastBodyState + BS_V, b2Add(v, dv));

		// Update body move event
		world.bodyMoveEvents[bodySimIndex].transform = transform;

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = world.bodies[fastBodyId].headShapeId;
		while (shapeId != B2_NULL_INDEX) {
			// Must recompute aabb at the interpolated transform
			let aabb = b2ComputeFatShapeAABB(world, shapeId, transform, speculativeDistance);
			world.shapes[shapeId].aabb = aabb;

			if (b2AABB_Contains(world.fatAABBs[shapeId], aabb) == false) {
				let fatAABB = b2InflateAABB(aabb, world.shapes[shapeId].aabbMargin);
				world.fatAABBs[shapeId] = fatAABB;

				// Regular bodies mark the hierarchy as enlarged using atomic operations.
				// Bullets are handled separately at a later stage.
				if (isBullet == false) {
					b2BroadPhase_MarkProxyMoved(world.broadPhase, world.shapes[shapeId].proxyKey, fatAABB);
				} else {
					store<u32>(fastBodySim + SIM_FLAGS, load<u32>(fastBodySim + SIM_FLAGS) | b2_enlargeBulletBounds);
				}
			}

			shapeId = world.shapes[shapeId].nextShapeId;
		}
	} else {
		// No time of impact event

		// Advance body
		store<b2Rot>(fastBodySim + SIM_ROTATION0, load<b2Rot>(fastBodySim + SIM_Q));
		store<b2Vec2>(fastBodySim + SIM_CENTER0, load<b2Vec2>(fastBodySim + SIM_CENTER));

		// Prepare AABBs for broad-phase
		shapeId = world.bodies[fastBodyId].headShapeId;
		while (shapeId != B2_NULL_INDEX) {
			// shape->aabb is still valid from above

			if (b2AABB_Contains(world.fatAABBs[shapeId], world.shapes[shapeId].aabb) == false) {
				// Note: far from the origin the margin can be snapped to the nearest ULP.
				// This relevant for DP mode. So we lose the broad-phase hysteresis.
				let fatAABB = b2InflateAABB(world.shapes[shapeId].aabb, world.shapes[shapeId].aabbMargin);
				world.fatAABBs[shapeId] = fatAABB;

				if (isBullet == false) {
					b2BroadPhase_MarkProxyMoved(world.broadPhase, world.shapes[shapeId].proxyKey, fatAABB);
				} else {
					store<u32>(fastBodySim + SIM_FLAGS, load<u32>(fastBodySim + SIM_FLAGS) | b2_enlargeBulletBounds);
				}
			}

			shapeId = world.shapes[shapeId].nextShapeId;
		}
	}

	// Push sensor hits on the the task context for serial processing. A part cannot push, so a world with sensors
	// sweeps on the main lane.
	if (!Parallel.inPart()) {
		for (let i = 0; i < context.sensorCount; ++i) {
			// Skip any sensor hits that occurred after a solid hit
			if (context.sensorFractions[i] < context.fraction) {
				taskContext.sensorHits.push(context.sensorHits[i]);
			}
		}
	}
}

// C formats the name with %s; it is zero terminated in a fixed buffer
function b2LogUnstableBody(world: b2World, bodyId: i32): void {
	let name = "";
	for (let i = 0; i < 11; ++i) {
		let ch = world.bodies[bodyId].name[i];
		if (ch == 0) {
			break;
		}
		name += String.fromCharCode(ch);
	}
	b2Log("unstable: " + name + "\n");
}

// Implements b2ParallelForCallback
// One body of b2FinalizeBodiesTask. It makes no calls, so WAMR AOT keeps the memory base in registers across its
// stores. The caller logs an unstable body and runs the continuous solve, which reads only this body and static data.
const b2_finalizeUnstable: i32 = 1;
const b2_finalizeContinuous: i32 = 2;

@noinline
unsafe function b2FinalizeBody(world: b2World, stepContext: b2StepContext, taskContext: b2TaskContext, simIndex: i32): i32 {
	let bodies = b2ArrayData(world.bodies);
	let states = b2ArrayData(stepContext.states);
	let sims = b2ArrayData(stepContext.sims);
	let moveEvents = b2ArrayData(world.bodyMoveEvents);
	let enableSleep = world.enableSleep;
	let enableContinuous = world.enableContinuous;
	let timeStep = stepContext.dt;
	let invTimeStep = stepContext.inv_dt;
	let worldId = world.worldId;
	let enlargedSimBitSet = taskContext.enlargedSimBitSet;
	let awakeIslandBitSet = taskContext.awakeIslandBitSet;
	let speculativeDistance = B2_SPECULATIVE_DISTANCE();
	let result = 0;

	let state = states + <usize>simIndex * sizeof<b2BodyState>();
	let sim = sims + <usize>simIndex * sizeof<b2BodySim>();

	let v = load<b2Vec2>(state + BS_V);
	let w = load<f32>(state + BS_W);
	let simBodyId = load<i32>(sim + SIM_BODYID);

	if (b2IsValidVec2(v) == false || b2IsValidFloat(w) == false) {
		result |= b2_finalizeUnstable;
	}

	B2_ASSERT(b2IsValidVec2(v));
	B2_ASSERT(b2IsValidFloat(w));

	let deltaPosition = load<b2Vec2>(state + BS_DP);
	let deltaRotation = load<b2Rot>(state + BS_DQ);
	let center = b2OffsetPos(load<b2Vec2>(sim + SIM_CENTER), deltaPosition);
	let q = b2NormalizeRot(b2MulRot(deltaRotation, load<b2Rot>(sim + SIM_Q)));
	store<b2Vec2>(sim + SIM_CENTER, center);
	store<b2Rot>(sim + SIM_Q, q);

	// Use the velocity of the farthest point on the body to account for rotation.
	let maxExtent = load<f32>(sim + SIM_MAXEXTENT);
	let maxVelocity = b2Length(v) + b2AbsFloat(w) * maxExtent;

	// Sleep needs to observe position correction as well as true velocity.
	let maxDeltaPosition = b2Length(deltaPosition) + b2AbsFloat(deltaRotation.s) * maxExtent;

	// Position correction is not as important for sleep as true velocity.
	let positionSleepFactor: f32 = 0.5;
	let sleepVelocity = b2MaxFloat(maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition);

	// reset state deltas
	store<b2Vec2>(state + BS_DP, b2Vec2_zero);
	store<b2Rot>(state + BS_DQ, b2Rot_identity);

	let transformP = b2OffsetPos(center, b2Neg(b2RotateVector(q, load<b2Vec2>(sim + SIM_LOCALCENTER))));
	store<b2Vec2>(sim + SIM_P, transformP);

	// cache miss here, however I need the shape list below
	let body = bodies + <usize>simBodyId * sizeof<b2Body>();
	store<i32>(body + B_BODYMOVEINDEX, simIndex);
	let moveEvent = moveEvents + <usize>simIndex * sizeof<b2BodyMoveEvent>();
	store<b2Transform>(moveEvent + ME_TRANSFORM, b2Transform(transformP, q));
	store<b2BodyId>(moveEvent + ME_BODYID, b2BodyId(simBodyId + 1, worldId, load<u16>(body + B_GENERATION)));
	store<u64>(moveEvent + ME_USERDATA, load<u64>(body + B_USERDATA));
	store<bool>(moveEvent + ME_FELLASLEEP, false);

	// reset applied force and torque
	store<b2Vec2>(sim + SIM_FORCE, b2Vec2_zero);
	store<f32>(sim + SIM_TORQUE, 0.0);

	// If you hit this then it means you deferred mass computation but never called b2Body_UpdateMassFromShapes
	B2_ASSERT((load<u32>(body + B_FLAGS) & b2_dirtyMass) == 0);

	// Clear the transient flags (fast, speed capped, had TOI). These flags are conditionally set
	// as part of the code below.
	let stateFlags = load<u32>(state + BS_FLAGS);
	let bodyFlags = load<u32>(body + B_FLAGS) & ~b2_bodyTransientFlags;
	let simFlags = load<u32>(sim + SIM_FLAGS) & ~(b2_isFast | b2_bodyTransientFlags);

	// The body state flag knows about speed capping (used for debug draw).
	bodyFlags |= stateFlags & b2_isSpeedCapped;
	store<u32>(body + B_FLAGS, bodyFlags);
	store<u32>(sim + SIM_FLAGS, simFlags);
	store<u32>(state + BS_FLAGS, stateFlags & ~b2_bodyTransientFlags);

	if (enableSleep == false || (bodyFlags & b2_enableSleep) == 0 || sleepVelocity > load<f32>(body + B_SLEEPTHRESHOLD)) {
		// Body is not sleepy
		store<f32>(body + B_SLEEPTIME, 0.0);

		let safetyFactor = load<f32>(body + B_SAFETYFACTOR);
		let maxMotion = b2MaxFloat(maxDeltaPosition, maxVelocity * timeStep);
		if (load<i32>(body + B_TYPE) == b2_dynamicBody && enableContinuous && maxMotion > safetyFactor * load<f32>(sim + SIM_MINEXTENT)) {
			// This flag is used for debug draw and contact recycling.
			simFlags |= b2_isFast;
			store<u32>(sim + SIM_FLAGS, simFlags);

			// Store fast bullets for processing later.
			// This is deterministic because the order of TOI sweeps doesn't matter.
			if ((simFlags & b2_isBullet) != 0) {
				let bulletIndex = b2AtomicFetchAddInt(b2StepAtomic(stepContext, SC_BULLETBODYCOUNT), 1);
				stepContext.bulletBodies[bulletIndex] = simIndex;
			} else {
				result |= b2_finalizeContinuous;
			}
		} else {
			// Body is safe to advance
			store<b2Vec2>(sim + SIM_CENTER0, center);
			store<b2Rot>(sim + SIM_ROTATION0, q);
		}
	} else {
		// Body is safe to advance and is falling asleep
		store<b2Vec2>(sim + SIM_CENTER0, center);
		store<b2Rot>(sim + SIM_ROTATION0, q);
		store<f32>(body + B_SLEEPTIME, load<f32>(body + B_SLEEPTIME) + timeStep);
	}

	// Any single body in an island can keep it awake
	let islandId = load<i32>(body + B_ISLANDID);
	let island = world.islands[islandId];
	let sleepTime = load<f32>(body + B_SLEEPTIME);
	if (sleepTime < B2_TIME_TO_SLEEP) {
		// keep island awake
		let islandIndex = island.localIndex;
		b2SetBit(awakeIslandBitSet, <u32>islandIndex);
	} else if (island.constraintRemoveCount > 0) {
		// Body wants to sleep but its island needs splitting first. Track the sleepiest candidate.
		// Break sleep time ties using the island id to ensure determinism. The cross worker reduction
		// breaks ties the same way.
		if (sleepTime > taskContext.splitSleepTime || (sleepTime == taskContext.splitSleepTime && islandId > taskContext.splitIslandId)) {
			taskContext.splitIslandId = islandId;
			taskContext.splitSleepTime = sleepTime;
		}
	}

	// Update shapes AABBs
	let transform = load<b2Transform>(sim + SIM_TRANSFORM);
	let isFast = (load<u32>(sim + SIM_FLAGS) & b2_isFast) != 0;
	let shapeId = load<i32>(body + B_HEADSHAPEID);
	while (shapeId != B2_NULL_INDEX) {
		if (isFast) {
			// For fast non-bullet bodies the AABB has already been updated in b2SolveContinuous
			// For fast bullet bodies the AABB will be updated at a later stage

			// Add to enlarged shapes regardless of AABB changes.
			// Bit-set to keep the move array sorted
			b2SetBit(enlargedSimBitSet, <u32>simIndex);
		} else {
			// Update bounds for slow bodies, including slow bullets.

			let aabb = b2ComputeFatShapeAABB(world, shapeId, transform, speculativeDistance);
			world.shapes[shapeId].aabb = aabb;

			if (b2AABB_Contains(world.fatAABBs[shapeId], aabb) == false) {
				let fatAABB = b2InflateAABB(aabb, world.shapes[shapeId].aabbMargin);
				world.fatAABBs[shapeId] = fatAABB;

				// Mark the hierarchy as enlarged using atomic operations.
				b2BroadPhase_MarkProxyMoved(world.broadPhase, world.shapes[shapeId].proxyKey, fatAABB);

				// Bit-set to keep the move array sorted
				b2SetBit(enlargedSimBitSet, <u32>simIndex);
			}
		}

		shapeId = world.shapes[shapeId].nextShapeId;
	}

	return result;
}

export unsafe function b2FinalizeBodiesTask(startIndex: i32, endIndex: i32, workerIndex: i32, stepContext: b2StepContext): void {
	let world = stepContext.world;

	B2_ASSERT(endIndex <= world.bodyMoveEvents.length);

	let taskContext = world.taskContexts[workerIndex];

	for (let simIndex = startIndex; simIndex < endIndex; ++simIndex) {
		let result = b2FinalizeBody(world, stepContext, taskContext, simIndex);
		if ((result & b2_finalizeUnstable) != 0) {
			// A part cannot log; b2FinishParallelFinalize does
			if (Parallel.inPart()) {
				taskContext.unstableCount += 1;
			} else {
				b2LogUnstableBody(world, b2SimBodyId(stepContext, simIndex));
			}
		}
		if ((result & b2_finalizeContinuous) != 0) {
			if (Parallel.inPart() && stepContext.parallelContinuous == false) {
				taskContext.deferredBodies[taskContext.deferredBodyCount] = simIndex;
				taskContext.deferredBodyCount += 1;
			} else {
				b2SolveContinuous(world, simIndex, taskContext, stepContext.dt);
			}
		}
	}
}

@inline unsafe function b2SimBodyId(stepContext: b2StepContext, simIndex: i32): i32 {
	return load<i32>(b2ArrayData(stepContext.sims) + <usize>simIndex * sizeof<b2BodySim>() + SIM_BODYID);
}

// Readies each worker's task context for parts to finalize bodyCount bodies, and says whether they sweep the fast ones
export function b2PrepareParallelFinalize(world: b2World, stepContext: b2StepContext, bodyCount: i32): void {
	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		if (taskContext.deferredBodies.length < bodyCount) {
			taskContext.deferredBodies = new StaticArray<i32>(bodyCount + (bodyCount >> 1));
		}
		taskContext.deferredBodyCount = 0;
		taskContext.unstableCount = 0;
	}

	// A sweep may push sensor hits and call the custom filter and the pre-continuous callback, which parts cannot
	stepContext.parallelContinuous = world.sensors.length == 0 && world.customFilterFcn == null && world.preContinuousFcn == null;
}

// What b2FinalizeBodiesTask does for the fast bodies and the unstable bodies the parts left to the main lane. The
// sweeps read only their own body and static shapes, so their order does not matter.
export unsafe function b2FinishParallelFinalize(world: b2World, stepContext: b2StepContext): void {
	let unstableCount = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		let deferred = taskContext.deferredBodies;
		for (let k = 0; k < taskContext.deferredBodyCount; ++k) {
			b2SolveContinuous(world, deferred[k], taskContext, stepContext.dt);
		}
		unstableCount += taskContext.unstableCount;
	}

	// C logs each as it finds it; these are the same bodies in sim order
	if (unstableCount > 0) {
		let states = b2ArrayData(stepContext.states);
		for (let simIndex = 0; simIndex < stepContext.sims.length; ++simIndex) {
			let state = states + <usize>simIndex * sizeof<b2BodyState>();
			if (b2IsValidVec2(load<b2Vec2>(state + BS_V)) == false || b2IsValidFloat(load<f32>(state + BS_W)) == false) {
				b2LogUnstableBody(world, b2SimBodyId(stepContext, simIndex));
			}
		}
	}
}

struct b2BlockDim {
	// number of items per block (except last block)
	size: i32 = 0;

	// total number of blocks
	count: i32 = 0;
}

// A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
// Each block of work has a minimum number of elements (block size). This in turn may limit the
// number of blocks. If there are many elements then the block size is increased so there are
// still at most M blocks of work per worker. M is a tunable number that has two goals:
// 1. keep M small to reduce overhead
// 2. keep M large enough for other workers to be able to steal work
// The block size is a power of two to make math efficient.
function b2ComputeBlockCount(itemCount: i32, minSize: i32, maxBlockCount: i32): b2BlockDim {
	let dim = b2BlockDim();
	if (itemCount == 0) {
		return dim;
	}

	if (itemCount <= minSize * maxBlockCount) {
		dim.size = minSize;
	} else {
		dim.size = (itemCount + maxBlockCount - 1) / maxBlockCount;
	}

	dim.count = (itemCount + dim.size - 1) / dim.size;

	B2_ASSERT(dim.count >= 1);
	B2_ASSERT(dim.size * dim.count >= itemCount);

	return dim;
}

// Initialize solver blocks for a contiguous range of items. Computes block size internally
// from the same parameters used by b2ComputeBlockCount. The atomic claim counter is zeroed
// so workers can CAS (0, 1) on the first stage that owns these blocks.
// C passes a pointer into the block buffer; here the buffer and the offset.
unsafe function b2InitBlocks(blocks: StaticArray<b2SyncBlock>, offset: i32, dim: b2BlockDim, itemCount: i32, blockType: u8, colorIndex: u8): void {
	if (dim.count == 0) {
		return;
	}

	B2_ASSERT(itemCount >= dim.count);

	// Compute the number of elements per block
	let blockSize = dim.size;

	// Simulation too big
	B2_ASSERT(blockSize <= UINT16_MAX);

	for (let i = 0; i < dim.count; ++i) {
		ref b = blocks[offset + i];
		b.block.startIndex = i * blockSize;
		b.block.count = <u16>blockSize;
		b.block.blockType = blockType;
		b.block.colorIndex = colorIndex;
		b2AtomicStoreInt(b2SyncIndex(blocks, offset + i), 0);
	}

	// The last block may not be full
	blocks[offset + dim.count - 1].block.count = <u16>(itemCount - (dim.count - 1) * blockSize);
}

// Returns the next stage index, as C returns stage + 1.
unsafe function b2InitStage(
	stages: StaticArray<b2SolverStage>,
	stageIndex: i32,
	type: b2SolverStageType,
	blocks: StaticArray<b2SyncBlock>,
	blockStart: i32,
	blockCount: i32,
	colorIndex: u8,
): i32 {
	let stage = stages[stageIndex];
	stage.type = type;
	stage.blocks = blocks;
	stage.blockStart = blockStart;
	stage.blockCount = blockCount;
	stage.colorIndex = colorIndex;
	b2AtomicStoreInt(b2CompletionCount(stage), 0);
	return stageIndex + 1;
}

// Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
// All iterations of a given color share the same b2SyncBlock array so the per-block syncIndex
// grows monotonically across stages within that color.
unsafe function b2InitColorStages(
	stages: StaticArray<b2SolverStage>,
	stageIndex: i32,
	type: b2SolverStageType,
	iterations: i32,
	activeColorCount: i32,
	colorBlocks: StaticArray<b2SyncBlock>,
	colorBlockStarts: StaticArray<i32>,
	colorBlockCounts: StaticArray<i32>,
	activeColorIndices: StaticArray<i32>,
): i32 {
	for (let j = 0; j < iterations; ++j) {
		for (let i = 0; i < activeColorCount; ++i) {
			stageIndex = b2InitStage(stages, stageIndex, type, colorBlocks, colorBlockStarts[i], colorBlockCounts[i], <u8>activeColorIndices[i]);
		}
	}
	return stageIndex;
}

// Runs the solver stages of a step on parallel calls, or answers false to leave them here
export type b2ParallelSolveFcn = (context: b2StepContext) => bool;

let b2_parallelSolve: b2ParallelSolveFcn? = null;

export function b2SetParallelSolve(fcn: b2ParallelSolveFcn?): void {
	b2_parallelSolve = fcn;
}

// Finalizes a step's bodyCount awake bodies on parallel calls, or answers false to leave them here
export type b2ParallelFinalizeFcn = (context: b2StepContext, bodyCount: i32) => bool;

let b2_parallelFinalize: b2ParallelFinalizeFcn? = null;

export function b2SetParallelFinalize(fcn: b2ParallelFinalizeFcn?): void {
	b2_parallelFinalize = fcn;
}

unsafe function b2ExecuteBlock(stage: b2SolverStage, context: b2StepContext, block: b2SolverBlock, workerIndex: i32): void {
	let stageType = stage.type;
	let blockType = <b2SolverBlockType>block.blockType;

	switch (stageType) {
		case b2_stagePrepareJoints:
			b2PrepareJointsTask(block, context);
			break;

		case b2_stagePrepareContacts:
			b2PrepareContacts_Wide(block, context);
			break;

		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask(block, context);
			break;

		case b2_stageWarmStart:
			if (blockType == b2_graphContactBlock) {
				b2WarmStartContacts_Wide(block, context);
			} else if (blockType == b2_graphJointBlock) {
				b2WarmStartJointsTask(block, context);
			}
			break;

		case b2_stageSolve:
			if (blockType == b2_graphContactBlock) {
				b2PushContacts_Wide(block, context);
			} else if (blockType == b2_graphJointBlock) {
				let useBias = true;
				b2SolveJointsTask(block, context, useBias, workerIndex);
			}
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask(block, context);
			break;

		case b2_stageRelax:
			if (blockType == b2_graphContactBlock) {
				b2SolveContacts_Wide(block, context);
			} else if (blockType == b2_graphJointBlock) {
				let useBias = false;
				b2SolveJointsTask(block, context, useBias, workerIndex);
			}
			break;

		case b2_stageRestitution:
			if (blockType == b2_graphContactBlock) {
				b2ApplyRestitution_Wide(block, context);
			}
			break;

		case b2_stageStoreImpulses:
			b2StoreImpulses_Wide(block, context, workerIndex);
			break;
	}
}

// This staggers the worker start indices so they avoid touching the same solver blocks
@inline function GetWorkerStartIndex(workerIndex: i32, blockCount: i32, workerCount: i32): i32 {
	if (blockCount <= workerCount) {
		return workerIndex < blockCount ? workerIndex : B2_NULL_INDEX;
	}

	let blocksPerWorker = blockCount / workerCount;
	let remainder = blockCount - blocksPerWorker * workerCount;
	return blocksPerWorker * workerIndex + b2MinInt(remainder, workerIndex);
}

// Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.
unsafe function b2ExecuteStage(stage: b2SolverStage, context: b2StepContext, previousSyncIndex: i32, syncIndex: i32, workerIndex: i32): void {
	let completedCount = 0;
	let blocks = stage.blocks;
	let blockStart = stage.blockStart;
	let blockCount = stage.blockCount;

	let startIndex = GetWorkerStartIndex(workerIndex, blockCount, context.workerCount);
	if (startIndex == B2_NULL_INDEX) {
		return;
	}

	B2_ASSERT(0 <= startIndex && startIndex < blockCount);

	let blockIndex = startIndex;
	for (let i = 0; i < blockCount; ++i) {
		// Read before attempting the CAS.
		ref syncBlock = blocks[blockStart + blockIndex];
		let blockSync = b2SyncIndex(blocks, blockStart + blockIndex);
		if (b2AtomicLoadInt(blockSync) == previousSyncIndex && b2AtomicCompareExchangeInt(blockSync, previousSyncIndex, syncIndex)) {
			B2_ASSERT(stage.type != b2_stagePrepareContacts || syncIndex < 2);
			B2_ASSERT(completedCount < blockCount);

			// Pass the descriptor by value -- the wrapping b2SyncBlock holds the atomic
			// syncIndex but we only copy .block, so the struct copy never aliases the CAS target.
			b2ExecuteBlock(stage, context, syncBlock.block, workerIndex);
			completedCount += 1;
		}

		blockIndex += 1;
		if (blockIndex >= blockCount) {
			blockIndex = 0;
		}
	}

	b2AtomicFetchAddInt(b2CompletionCount(stage), completedCount);
}

// Execute a stage on worker 0 (main thread).
unsafe function b2ExecuteMainStage(stage: b2SolverStage, context: b2StepContext, syncBits: u32): void {
	let blockCount = stage.blockCount;
	if (blockCount == 0) {
		return;
	}

	let workerIndex = 0;

	if (blockCount == 1) {
		b2ExecuteBlock(stage, context, stage.blocks[stage.blockStart].block, workerIndex);
	} else {
		b2PublishSyncBits(context, syncBits);

		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
		B2_ASSERT(syncIndex > 0);
		let previousSyncIndex = syncIndex - 1;

		b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		// Spin waiting for thieves to finish
		let completionCount = b2CompletionCount(stage);
		while (b2AtomicLoadInt(completionCount) != blockCount) {
			b2Pause();
		}

		b2AtomicStoreInt(completionCount, 0);
	}
}

@inline function b2SyncBits(syncIndex: i32, stageIndex: i32): u32 {
	return (<u32>syncIndex << 16) | <u32>stageIndex;
}

// C's tick counter, in a FixedArray so b2GetMillisecondsAndReset can take it by ref
@inline function b2MillisecondsAndReset(ticks: ref FixedArray<u64, 1>): f32 {
	return b2GetMillisecondsAndReset(ticks[0]);
}

// Parallel solver task
unsafe function b2SolverTask(workerContext: b2WorkerContext): void {
	b2SolverWorker(workerContext.context!, workerContext.workerIndex);
}

// b2SolverTask for worker workerIndex of the step, which a part of a parallel call runs as its own
export unsafe function b2SolverWorker(context: b2StepContext, workerIndex: i32): void {
	let activeColorCount = context.activeColorCount;
	let stages = context.stages;
	let world = context.world;

	if (workerIndex == 0) {
		// The orchestrator slot is a race. The calling thread of b2World_Step also enters here
		// as worker 0, so progress is guaranteed even if the user's task system schedules tasks
		// out of order, has fewer threads than workerCount, or runs the task synchronously
		// inside enqueueTaskFcn. Whoever wins the CAS becomes the orchestrator; the loser
		// returns and lets the spinner-only path handle workers >0.
		if (b2AtomicCompareExchangeInt(b2StepAtomic(context, SC_MAINCLAIMED), 0, 1) == false) {
			return;
		}

		// Main thread synchronizes the workers and does work itself.
		//
		// This single task is able to fully complete all work even if all other workers are
		// blocked, so a fully serial task system still drives the simulation forward.

		// Stages are re-used by loops so that I don't need more stages for large substep counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		Stage sequence
		b2_stagePrepareJoints,
		b2_stagePrepareContacts,
		b2_stageIntegrateVelocities,
		b2_stageWarmStart,
		b2_stageSolve,
		b2_stageIntegratePositions,
		b2_stageRelax,
		b2_stageRestitution,
		b2_stageStoreImpulses
		*/

		let ticks: FixedArray<u64, 1>;
		ticks[0] = b2GetTicks();

		let bodySyncIndex = 1;
		let stageIndex = 0;

		// Prepare joint constraints
		let jointSyncIndex = 1;
		let syncBits = b2SyncBits(jointSyncIndex, stageIndex);
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareJoints);
		b2ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;
		jointSyncIndex += 1;

		// Prepare contact constraints
		let contactSyncIndex = 1;
		syncBits = b2SyncBits(contactSyncIndex, stageIndex);
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareContacts);
		b2ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;
		contactSyncIndex += 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		b2PrepareJoints_Overflow(context);
		b2PrepareContacts_Overflow(context);

		world.profile.prepareConstraints += b2MillisecondsAndReset(ticks);

		let graphSyncIndex = 1;
		let subStepCount = context.subStepCount;
		for (let subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex) {
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			let iterationStageIndex = stageIndex;

			// Integrate velocities
			syncBits = b2SyncBits(bodySyncIndex, iterationStageIndex);
			B2_ASSERT(stages[iterationStageIndex].type == b2_stageIntegrateVelocities);
			b2ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			world.profile.integrateVelocities += b2MillisecondsAndReset(ticks);

			// Warm start constraints
			b2WarmStartJoints_Overflow(context);
			b2WarmStartContacts_Overflow(context);

			for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
				syncBits = b2SyncBits(graphSyncIndex, iterationStageIndex);
				B2_ASSERT(stages[iterationStageIndex].type == b2_stageWarmStart);
				b2ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
				iterationStageIndex += 1;
			}
			graphSyncIndex += 1;

			world.profile.warmStart += b2MillisecondsAndReset(ticks);

			// Solve constraints
			let useBias = true;
			for (let j = 0; j < ITERATIONS; ++j) {
				// Overflow constraints have lower priority. Typically these are dynamic-vs-dynamic.
				b2SolveJoints_Overflow(context, useBias);
				b2PushContacts_Overflow(context);

				for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
					syncBits = b2SyncBits(graphSyncIndex, iterationStageIndex);
					B2_ASSERT(stages[iterationStageIndex].type == b2_stageSolve);
					b2ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			world.profile.solveImpulses += b2MillisecondsAndReset(ticks);

			// Integrate positions
			B2_ASSERT(stages[iterationStageIndex].type == b2_stageIntegratePositions);
			syncBits = b2SyncBits(bodySyncIndex, iterationStageIndex);
			b2ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			world.profile.integratePositions += b2MillisecondsAndReset(ticks);

			// Relax constraints
			useBias = false;
			for (let j = 0; j < RELAX_ITERATIONS; ++j) {
				b2SolveJoints_Overflow(context, useBias);
				b2SolveContacts_Overflow(context);
				for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
					syncBits = b2SyncBits(graphSyncIndex, iterationStageIndex);
					B2_ASSERT(stages[iterationStageIndex].type == b2_stageRelax);
					b2ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			world.profile.relaxImpulses += b2MillisecondsAndReset(ticks);
		}

		// Advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

		let restitutionIterations = world.restitutionIterations;
		if (restitutionIterations > 0 && b2AtomicLoadInt(b2StepAtomic(context, SC_ANYRESTITUTION)) != 0) {
			for (let j = 0; j < restitutionIterations; ++j) {
				b2ApplyRestitution_Overflow(context);

				let iterationStageIndex = stageIndex;
				for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
					syncBits = b2SyncBits(graphSyncIndex, iterationStageIndex);
					B2_ASSERT(stages[iterationStageIndex].type == b2_stageRestitution);
					b2ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			world.profile.restitution += b2MillisecondsAndReset(ticks);
		}

		stageIndex += activeColorCount;

		// Store impulses
		b2StoreImpulses_Overflow(context);

		syncBits = b2SyncBits(contactSyncIndex, stageIndex);
		B2_ASSERT(stages[stageIndex].type == b2_stageStoreImpulses);
		b2ExecuteMainStage(stages[stageIndex], context, syncBits);

		world.profile.storeImpulses += b2MillisecondsAndReset(ticks);

		// Signal workers to finish
		b2PublishSyncBits(context, UINT32_MAX);

		B2_ASSERT(stageIndex + 1 == context.stageCount);
		return;
	}

	// Worker spins and waits for work. Tasks run inline after worker 0 finished every stage, so this sees the
	// sentinel at once.
	let lastSyncBits: u32 = 0;
	while (true) {
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		let syncBitsAddress = b2StepAtomic(context, SC_SYNCBITS);
		let syncBits: u32 = b2AtomicLoadU32(syncBitsAddress);
		let spinCount = 0;
		let yieldCount = 0;
		while (syncBits == lastSyncBits) {
			if (spinCount > 5) {
				yieldCount += 1;
				b2YieldWorker(context, lastSyncBits, yieldCount);
				spinCount = 0;
			} else {
				b2Pause();
				b2Pause();
				spinCount += 1;
			}
			syncBits = b2AtomicLoadU32(syncBitsAddress);
		}

		if (syncBits == UINT32_MAX) {
			// sentinel hit
			break;
		}

		let stageIndex = <i32>(syncBits & 0xffff);
		B2_ASSERT(stageIndex < context.stageCount);

		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
		B2_ASSERT(syncIndex > 0);

		let previousSyncIndex = syncIndex - 1;

		let stage = stages[stageIndex];
		b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		lastSyncBits = syncBits;
	}
}

unsafe function b2BulletBodyTask(startIndex: i32, endIndex: i32, workerIndex: i32, stepContext: b2StepContext): void {
	let taskContext = stepContext.world.taskContexts[workerIndex];

	B2_ASSERT(startIndex <= endIndex);

	for (let i = startIndex; i < endIndex; ++i) {
		let simIndex = stepContext.bulletBodies[i];
		b2SolveContinuous(stepContext.world, simIndex, taskContext, stepContext.dt);
	}
}

// C takes the per step buffers from the world's stack allocator. These grow only, shared by every world since one
// b2Solve runs at a time, and every reader stays within the counts the step writes, so stale contents past them or
// from an earlier step are never read.
class b2SolverScratch {
	bulletBodies: StaticArray<i32> = new StaticArray<i32>(0);
	wideConstraints: StaticArray<b2ContactConstraintWide> = new StaticArray<b2ContactConstraintWide>(0);
	overflowConstraints: StaticArray<b2ContactConstraint> = new StaticArray<b2ContactConstraint>(0);
	stages: StaticArray<b2SolverStage> = new StaticArray<b2SolverStage>(0);
	bodyBlocks: StaticArray<b2SyncBlock> = new StaticArray<b2SyncBlock>(0);
	contactBlocks: StaticArray<b2SyncBlock> = new StaticArray<b2SyncBlock>(0);
	jointBlocks: StaticArray<b2SyncBlock> = new StaticArray<b2SyncBlock>(0);
	graphBlocks: StaticArray<b2SyncBlock> = new StaticArray<b2SyncBlock>(0);

	// C's arrays on b2Solve's stack
	activeColorIndices: StaticArray<i32> = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
	colorContactCounts: StaticArray<i32> = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
	colorJointCounts: StaticArray<i32> = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
	graphContactDims: StaticArray<b2BlockDim> = new StaticArray<b2BlockDim>(B2_GRAPH_COLOR_COUNT);
	graphJointDims: StaticArray<b2BlockDim> = new StaticArray<b2BlockDim>(B2_GRAPH_COLOR_COUNT);
	graphColorBlockStarts: StaticArray<i32> = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
	graphBlockCounts: StaticArray<i32> = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
	contactPrepareSpans: StaticArray<b2ContactPrepareSpan> = new StaticArray<b2ContactPrepareSpan>(B2_GRAPH_COLOR_COUNT + 1);
	jointPrepareSpans: StaticArray<b2JointPrepareSpan> = new StaticArray<b2JointPrepareSpan>(B2_GRAPH_COLOR_COUNT + 1);
	workerContexts: StaticArray<b2WorkerContext> = new StaticArray<b2WorkerContext>(B2_MAX_WORKERS);
	callerContext: b2WorkerContext = new b2WorkerContext();

	constructor() {
		for (let i = 0; i <= B2_GRAPH_COLOR_COUNT; ++i) {
			this.contactPrepareSpans[i] = new b2ContactPrepareSpan();
			this.jointPrepareSpans[i] = new b2JointPrepareSpan();
		}
		for (let i = 0; i < B2_MAX_WORKERS; ++i) {
			this.workerContexts[i] = new b2WorkerContext();
		}
	}
}

const b2_solverScratch = new b2SolverScratch();

@inline function b2GrowCapacity(count: i32): i32 {
	return count + (count >> 2) + 16;
}

function b2ReserveIndices(a: StaticArray<i32>, count: i32): StaticArray<i32> {
	return a.length >= count ? a : new StaticArray<i32>(b2GrowCapacity(count));
}

function b2ReserveSyncBlocks(a: StaticArray<b2SyncBlock>, count: i32): StaticArray<b2SyncBlock> {
	return a.length >= count ? a : new StaticArray<b2SyncBlock>(b2GrowCapacity(count));
}

function b2ReserveWideConstraints(a: StaticArray<b2ContactConstraintWide>, count: i32): StaticArray<b2ContactConstraintWide> {
	return a.length >= count ? a : new StaticArray<b2ContactConstraintWide>(b2GrowCapacity(count));
}

function b2ReserveContactConstraints(a: StaticArray<b2ContactConstraint>, count: i32): StaticArray<b2ContactConstraint> {
	return a.length >= count ? a : new StaticArray<b2ContactConstraint>(b2GrowCapacity(count));
}

function b2ReserveStages(a: StaticArray<b2SolverStage>, count: i32): StaticArray<b2SolverStage> {
	if (a.length >= count) {
		return a;
	}
	let stages = new StaticArray<b2SolverStage>(b2GrowCapacity(count));
	for (let i = 0; i < stages.length; ++i) {
		stages[i] = i < a.length ? a[i] : new b2SolverStage();
	}
	return stages;
}

// Solve with graph coloring
@noinline
export unsafe function b2Solve(world: b2World, stepContext: b2StepContext): void {
	let scratch = b2_solverScratch;
	let simdShift = world.simdWidth == 8 ? 3 : 2;

	// Only count steps that advance the simulation
	world.stepIndex += 1;

	// Are there any awake bodies? This scenario should not be important for profiling.
	let awakeSet = world.solverSets[b2_awakeSet];
	let awakeBodyCount = awakeSet.bodySims.length;
	if (awakeBodyCount == 0) {
		return;
	}

	// Solve constraints using graph coloring
	{
		let setupTicks: FixedArray<u64, 1>;
		setupTicks[0] = b2GetTicks();

		// Prepare buffers for bullets
		b2AtomicStoreInt(b2StepAtomic(stepContext, SC_BULLETBODYCOUNT), 0);
		scratch.bulletBodies = b2ReserveIndices(scratch.bulletBodies, awakeBodyCount);
		stepContext.bulletBodies = scratch.bulletBodies;

		let graph = world.constraintGraph;
		let colors = graph.colors;

		stepContext.sims = awakeSet.bodySims;
		stepContext.states = awakeSet.bodyStates;

		// count contacts, joints, and colors
		let activeColorCount = 0;
		for (let i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i) {
			let perColorContactCount = colors[i].contactSims.length;
			let perColorJointCount = colors[i].jointSims.length;
			let occupancyCount = perColorContactCount + perColorJointCount;
			activeColorCount += occupancyCount > 0 ? 1 : 0;
		}

		// prepare for move events
		world.bodyMoveEvents.length = awakeBodyCount;

		let workerCount = world.workerCount;

		// Target 4 blocks per worker to allow work stealing
		let maxBlockCount = 4 * workerCount;

		// Body blocks are for parallel iteration over bodies directly (integration, update transforms)
		let minBodiesPerBlock = 32;
		let bodyDim = b2ComputeBlockCount(awakeBodyCount, minBodiesPerBlock, maxBlockCount);

		let minContactsPerBlock = 4;
		let minJointsPerBlock = 4;

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of wide contact blocks and joint blocks
		let activeColorIndices = scratch.activeColorIndices;
		let colorContactCounts = scratch.colorContactCounts;
		let colorJointCounts = scratch.colorJointCounts;
		let graphContactDims = scratch.graphContactDims;
		let graphJointDims = scratch.graphJointDims;
		let graphBlockCount = 0;

		// c is the active color index
		let wideContactCount = 0;
		let jointCount = 0;
		let c = 0;
		for (let i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i) {
			let colorContactCount = colors[i].contactSims.length;
			let colorJointCount = colors[i].jointSims.length;

			if (colorContactCount + colorJointCount == 0) {
				continue;
			}

			activeColorIndices[c] = i;

			// Ceiling for wide constraint count
			let colorContactCountW = colorContactCount > 0 ? ((colorContactCount - 1) >> simdShift) + 1 : 0;
			wideContactCount += colorContactCountW;
			colorContactCounts[c] = colorContactCountW;

			colorJointCounts[c] = colorJointCount;
			jointCount += colorJointCount;

			// Graph solver block dimensions
			let contactDim = b2ComputeBlockCount(colorContactCountW, minContactsPerBlock, maxBlockCount);
			let jointDim = b2ComputeBlockCount(colorJointCount, minJointsPerBlock, maxBlockCount);
			graphContactDims[c] = contactDim;
			graphJointDims[c] = jointDim;
			graphBlockCount += contactDim.count + jointDim.count;

			c += 1;
		}
		activeColorCount = c;

		// Prepare and store run as one flat parallel-for over the entire wide constraint range,
		// partitioned into uniformly sized blocks. Color info is consulted inside the task via
		// a small span array, so blocks do not need to honor color boundaries here.
		let contactPrepareDim = b2ComputeBlockCount(wideContactCount, minContactsPerBlock, maxBlockCount);
		let jointPrepareDim = b2ComputeBlockCount(jointCount, minJointsPerBlock, maxBlockCount);

		B2_ASSERT(b2GetWideContactConstraintByteCount(world.simdWidth) == <i32>sizeof<b2ContactConstraintWide>());
		scratch.wideConstraints = b2ReserveWideConstraints(scratch.wideConstraints, wideContactCount);
		let wideContactConstraints = scratch.wideConstraints;

		let overflow = colors[B2_OVERFLOW_INDEX];
		let overflowCount = overflow.contactSims.length;
		scratch.overflowConstraints = b2ReserveContactConstraints(scratch.overflowConstraints, overflowCount);
		overflow.overflowConstraints = scratch.overflowConstraints;

		// Build the span table for the flat prepare/store parallel-for while I slice the
		// wide constraint buffer across colors. One entry per active color plus a sentinel
		// at wideContactCount.
		let contactPrepareSpans = scratch.contactPrepareSpans;
		let jointPrepareSpans = scratch.jointPrepareSpans;

		// Distribute transient constraints to each graph color and prepare spans
		{
			let wideBase = 0;
			let jointBase = 0;
			for (let i = 0; i < activeColorCount; ++i) {
				let j = activeColorIndices[i];
				let color = colors[j];

				let colorContactCount = color.contactSims.length;
				let contactSpan = contactPrepareSpans[i];
				contactSpan.start = wideBase;
				contactSpan.count = colorContactCount;
				contactSpan.contacts = color.contactSims;

				if (colorContactCount == 0) {
					color.wideConstraintStart = 0;
					color.wideConstraintCount = 0;
				} else {
					color.wideConstraintStart = wideBase;

					let colorContactCountW = ((colorContactCount - 1) >> simdShift) + 1;
					color.wideConstraintCount = colorContactCountW;

					// Zero remainder lanes in the tail wide slot so prepare workers don't need to
					// initialize them.
					if ((colorContactCount & (world.simdWidth - 1)) != 0) {
						let zero: b2ContactConstraintWide;
						let wide = wideContactConstraints;
						wide[wideBase + colorContactCountW - 1] = zero;
					}

					wideBase += colorContactCountW;
				}

				let jointSpan = jointPrepareSpans[i];
				jointSpan.start = jointBase;
				jointSpan.count = color.jointSims.length;
				jointSpan.joints = color.jointSims;
				jointBase += color.jointSims.length;
			}

			// Sentinel
			let contactSentinel = contactPrepareSpans[activeColorCount];
			contactSentinel.start = wideContactCount;
			contactSentinel.count = 0;
			contactSentinel.contacts = b2_emptyContactSims;
			B2_ASSERT(wideBase == wideContactCount);

			let jointSentinel = jointPrepareSpans[activeColorCount];
			jointSentinel.start = jointCount;
			jointSentinel.count = 0;
			jointSentinel.joints = b2_emptyJointSims;
			B2_ASSERT(jointBase == jointCount);
		}

		let stageCount = 0;

		// b2_stagePrepareJoints
		stageCount += 1;
		// b2_stagePrepareContacts
		stageCount += 1;
		// b2_stageIntegrateVelocities
		stageCount += 1;
		// b2_stageWarmStart
		stageCount += activeColorCount;
		// b2_stageSolve
		stageCount += ITERATIONS * activeColorCount;
		// b2_stageIntegratePositions
		stageCount += 1;
		// b2_stageRelax
		stageCount += RELAX_ITERATIONS * activeColorCount;
		// b2_stageRestitution
		stageCount += activeColorCount;
		// b2_stageStoreImpulses
		stageCount += 1;

		scratch.stages = b2ReserveStages(scratch.stages, stageCount);
		scratch.bodyBlocks = b2ReserveSyncBlocks(scratch.bodyBlocks, bodyDim.count);
		scratch.contactBlocks = b2ReserveSyncBlocks(scratch.contactBlocks, contactPrepareDim.count);
		scratch.jointBlocks = b2ReserveSyncBlocks(scratch.jointBlocks, jointPrepareDim.count);
		scratch.graphBlocks = b2ReserveSyncBlocks(scratch.graphBlocks, graphBlockCount);
		let stages = scratch.stages;
		let bodyBlocks = scratch.bodyBlocks;
		let contactBlocks = scratch.contactBlocks;
		let jointBlocks = scratch.jointBlocks;
		let graphBlocks = scratch.graphBlocks;

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		// Tasks run inline, as C's default enqueueTaskFcn does, which returns NULL.
		if (world.splitIslandId != B2_NULL_INDEX) {
			if (world.taskCount < B2_MAX_TASKS) {
				b2SplitIslandTask(world);
				world.taskCount += 1;
			} else {
				b2SplitIslandTask(world);
			}
		}

		// Prepare body blocks
		b2InitBlocks(bodyBlocks, 0, bodyDim, awakeBodyCount, <u8>b2_bodyBlock, <u8>UINT8_MAX);

		// Prepare blocks as a single flat parallel-for over the whole constraint range.
		// The task walks spans to decode flat slot indices back to per-color arrays.
		b2InitBlocks(contactBlocks, 0, contactPrepareDim, wideContactCount, <u8>b2_contactBlock, <u8>UINT8_MAX);
		b2InitBlocks(jointBlocks, 0, jointPrepareDim, jointCount, <u8>b2_jointBlock, <u8>UINT8_MAX);

		// Prepare graph work blocks. Each color gets joint blocks followed by contact blocks.
		let graphColorBlockStarts = scratch.graphColorBlockStarts;
		let graphBlockCounts = scratch.graphBlockCounts;
		let baseGraphBlock = 0;
		for (let i = 0; i < activeColorCount; ++i) {
			graphColorBlockStarts[i] = baseGraphBlock;

			let colorIndex = <u8>activeColorIndices[i];
			b2InitBlocks(graphBlocks, baseGraphBlock, graphJointDims[i], colorJointCounts[i], <u8>b2_graphJointBlock, colorIndex);
			baseGraphBlock += graphJointDims[i].count;

			b2InitBlocks(graphBlocks, baseGraphBlock, graphContactDims[i], colorContactCounts[i], <u8>b2_graphContactBlock, colorIndex);
			baseGraphBlock += graphContactDims[i].count;

			graphBlockCounts[i] = graphJointDims[i].count + graphContactDims[i].count;
		}

		B2_ASSERT(baseGraphBlock == graphBlockCount);

		let stage = 0;
		stage = b2InitStage(stages, stage, b2_stagePrepareJoints, jointBlocks, 0, jointPrepareDim.count, <u8>UINT8_MAX);
		stage = b2InitStage(stages, stage, b2_stagePrepareContacts, contactBlocks, 0, contactPrepareDim.count, <u8>UINT8_MAX);
		stage = b2InitStage(stages, stage, b2_stageIntegrateVelocities, bodyBlocks, 0, bodyDim.count, <u8>UINT8_MAX);
		stage = b2InitColorStages(
			stages,
			stage,
			b2_stageWarmStart,
			1,
			activeColorCount,
			graphBlocks,
			graphColorBlockStarts,
			graphBlockCounts,
			activeColorIndices,
		);
		stage = b2InitColorStages(
			stages,
			stage,
			b2_stageSolve,
			ITERATIONS,
			activeColorCount,
			graphBlocks,
			graphColorBlockStarts,
			graphBlockCounts,
			activeColorIndices,
		);
		stage = b2InitStage(stages, stage, b2_stageIntegratePositions, bodyBlocks, 0, bodyDim.count, <u8>UINT8_MAX);
		stage = b2InitColorStages(
			stages,
			stage,
			b2_stageRelax,
			RELAX_ITERATIONS,
			activeColorCount,
			graphBlocks,
			graphColorBlockStarts,
			graphBlockCounts,
			activeColorIndices,
		);
		stage = b2InitColorStages(
			stages,
			stage,
			b2_stageRestitution,
			1,
			activeColorCount,
			graphBlocks,
			graphColorBlockStarts,
			graphBlockCounts,
			activeColorIndices,
		);
		stage = b2InitStage(stages, stage, b2_stageStoreImpulses, contactBlocks, 0, contactPrepareDim.count, <u8>UINT8_MAX);

		B2_ASSERT(stage == stageCount);

		B2_ASSERT(workerCount <= B2_MAX_WORKERS);
		let workerContext = scratch.workerContexts;

		stepContext.graph = graph;
		stepContext.activeColorCount = activeColorCount;
		stepContext.workerCount = workerCount;
		stepContext.stageCount = stageCount;
		stepContext.stages = stages;
		stepContext.wideContactConstraints = wideContactConstraints;
		stepContext.contactPrepareSpans = contactPrepareSpans;
		stepContext.wideContactCount = wideContactCount;
		stepContext.jointPrepareSpans = jointPrepareSpans;
		b2AtomicStoreU32(b2StepAtomic(stepContext, SC_SYNCBITS), 0);
		b2AtomicStoreInt(b2StepAtomic(stepContext, SC_MAINCLAIMED), 0);
		b2AtomicStoreInt(b2StepAtomic(stepContext, SC_SLEEPINGWORKERS), 0);

		world.profile.solverSetup = b2MillisecondsAndReset(setupTicks);

		let constraintTicks: FixedArray<u64, 1>;
		constraintTicks[0] = b2GetTicks();

		let jointIdCapacity = b2GetIdCapacity(world.jointIdPool);
		let contactIdCapacity = b2GetIdCapacity(world.contactIdPool);
		// C clears each worker's bits as it queues that worker's task; no task writes another worker's
		for (let i = 0; i < workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			b2SetBitCountAndClear(taskContext.jointStateBitSet, <u32>jointIdCapacity);
			b2SetBitCountAndClear(taskContext.hitEventBitSet, <u32>contactIdCapacity);
			taskContext.hasHitEvents = false;
		}

		let parallelSolve = b2_parallelSolve;
		if (parallelSolve == null || parallelSolve(stepContext) == false) {
			for (let i = 0; i < workerCount; ++i) {
				let context = workerContext[i];
				context.context = stepContext;
				context.workerIndex = i;

				// Tasks run inline, as C's default enqueueTaskFcn does, which returns NULL
				context.userTask = 0;
				b2SolverTask(context);
				if (world.taskCount < B2_MAX_TASKS) {
					world.taskCount += 1;
				}
			}

			// The calling thread of b2World_Step also enters b2SolverTask as worker 0 and races for the
			// orchestrator slot via the CAS inside. This guarantees progress even when the user's task
			// system can't run the queued worker 0 promptly: it might schedule out of order, have fewer
			// threads than workerCount, or invert priority by parking the calling thread in finishTaskFcn.
			// Whoever wins the CAS becomes the orchestrator; the loser returns and lets the spinner-only
			// path handle workers >0.
			let callerContext = scratch.callerContext;
			callerContext.context = stepContext;
			callerContext.workerIndex = 0;
			callerContext.userTask = 0;
			b2SolverTask(callerContext);
		}

		// Finish constraint solve and island split: inline tasks have no handle to finish
		world.splitIslandId = B2_NULL_INDEX;

		world.profile.constraints = b2MillisecondsAndReset(constraintTicks);

		// C finishes the tree task here
		b2FinishTreeUpdate(world);

		let transformTicks = b2GetTicks();

		// Prepare contact, enlarged body, and island bit sets used in body finalization.
		let awakeIslandCount = awakeSet.islandSims.length;
		for (let i = 0; i < world.workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			taskContext.sensorHits.length = 0;
			b2SetBitCountAndClear(taskContext.enlargedSimBitSet, <u32>awakeBodyCount);
			b2SetBitCountAndClear(taskContext.awakeIslandBitSet, <u32>awakeIslandCount);
			taskContext.splitIslandId = B2_NULL_INDEX;
			taskContext.splitSleepTime = 0.0;
		}

		// The user tree task has finished, so the broad-phase is ready.

		b2ValidateNoEnlarged(world.broadPhase);

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		let parallelFinalize = b2_parallelFinalize;
		if (parallelFinalize == null || parallelFinalize(stepContext, awakeBodyCount) == false) {
			b2ParallelFor<b2StepContext>(world, b2FinalizeBodiesTask, awakeBodyCount, 64, stepContext);
		}

		world.profile.transforms = b2GetMilliseconds(transformTicks);
	}

	// Report joint events
	{
		let jointEventTicks = b2GetTicks();

		// Gather bits for all joints that have force/torque events
		let jointStateBitSet = world.taskContexts[0].jointStateBitSet;
		for (let i = 1; i < world.workerCount; ++i) {
			b2InPlaceUnion(jointStateBitSet, world.taskContexts[i].jointStateBitSet);
		}

		{
			let wordCount = jointStateBitSet.blockCount;
			let bits = jointStateBitSet.bits;

			let worldIndex0 = world.worldId;

			for (let k: u32 = 0; k < wordCount; ++k) {
				let word = bits[<i32>k];
				while (word != 0) {
					let ctz = b2CTZ64(word);
					let jointId = <i32>(64 * k + ctz);

					B2_ASSERT(jointId < world.joints.length);
					B2_ASSERT(world.joints[jointId].setIndex == b2_awakeSet);

					let event = b2JointEvent();
					event.jointId = b2JointId(jointId + 1, worldIndex0, world.joints[jointId].generation);
					event.userData = world.joints[jointId].userData;

					world.jointEvents.push(event);

					// Clear the smallest set bit
					word = word & (word - 1);
				}
			}
		}

		world.profile.jointEvents = b2GetMilliseconds(jointEventTicks);
	}

	// Report hit events
	{
		let hitTicks = b2GetTicks();

		B2_ASSERT(world.contactHitEvents.length == 0);

		// Fast path: skip if no worker flagged any hit-event candidates during impulse storing.
		let anyHitEvents = false;
		for (let i = 0; i < world.workerCount; ++i) {
			if (world.taskContexts[i].hasHitEvents) {
				anyHitEvents = true;
				break;
			}
		}

		if (anyHitEvents) {
			// Union per-worker bits into worker 0's bit set.
			let hitEventBitSet = world.taskContexts[0].hitEventBitSet;
			for (let i = 1; i < world.workerCount; ++i) {
				if (world.taskContexts[i].hasHitEvents) {
					b2InPlaceUnion(hitEventBitSet, world.taskContexts[i].hitEventBitSet);
				}
			}

			let threshold = world.hitEventThreshold;
			let colors = world.constraintGraph.colors;
			let worldId = world.worldId;

			let wordCount = hitEventBitSet.blockCount;
			let bits = hitEventBitSet.bits;
			for (let k: u32 = 0; k < wordCount; ++k) {
				let word = bits[<i32>k];
				while (word != 0) {
					let ctz = b2CTZ64(word);
					let contactId = <i32>(64 * k + ctz);

					let contactColorIndex = world.contacts[contactId].colorIndex;
					let contactLocalIndex = world.contacts[contactId].localIndex;
					B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet && contactColorIndex != B2_NULL_INDEX);

					let color = colors[contactColorIndex];
					let contactSims = color.contactSims;

					let event = b2ContactHitEvent();
					event.approachSpeed = threshold;

					let bestPoint = B2_NULL_INDEX;
					let pointCount = contactSims[contactLocalIndex].manifold.pointCount;
					for (let p = 0; p < pointCount; ++p) {
						let approachSpeed = -contactSims[contactLocalIndex].manifold.points[p].normalVelocity;

						// Need to check total impulse because the point may be speculative and not colliding
						if (approachSpeed > event.approachSpeed && contactSims[contactLocalIndex].manifold.points[p].totalNormalImpulse > 0.0) {
							event.approachSpeed = approachSpeed;
							bestPoint = p;
						}
					}

					if (bestPoint != B2_NULL_INDEX) {
						event.normal = contactSims[contactLocalIndex].manifold.normal;

						let shapeIdA = contactSims[contactLocalIndex].shapeIdA;
						let shapeIdB = contactSims[contactLocalIndex].shapeIdB;

						// World contact point reconstructed from a body center of mass and the matching
						// anchor. The anchors were built with the manifold, so a body that has moved since
						// drags the point with it. A static body has not moved, prefer one so the common
						// case of a fast body striking the world stays exact.
						let bodyIdA = world.shapes[shapeIdA].bodyId;
						let bodyIdB = world.shapes[shapeIdB].bodyId;
						if (world.bodies[bodyIdA].type != b2_staticBody && world.bodies[bodyIdB].type == b2_staticBody) {
							let centerB = b2GetBodySim(world, bodyIdB).center;
							event.point = b2OffsetPos(centerB, contactSims[contactLocalIndex].manifold.points[bestPoint].anchorB);
						} else {
							let centerA = b2GetBodySim(world, bodyIdA).center;
							event.point = b2OffsetPos(centerA, contactSims[contactLocalIndex].manifold.points[bestPoint].anchorA);
						}

						event.shapeIdA = b2ShapeId(shapeIdA + 1, worldId, world.shapes[shapeIdA].generation);
						event.shapeIdB = b2ShapeId(shapeIdB + 1, worldId, world.shapes[shapeIdB].generation);

						let id = b2ContactId();
						id.index1 = world.contacts[contactId].contactId + 1;
						id.world0 = worldId;
						id.padding = 0;
						id.generation = world.contacts[contactId].generation;
						event.contactId = id;

						world.contactHitEvents.push(event);
					}

					// Clear the smallest set bit
					word = word & (word - 1);
				}
			}
		}

		world.profile.hitEvents = b2GetMilliseconds(hitTicks);
	}

	{
		let refitTicks = b2GetTicks();

		let bp = world.broadPhase;
		b2DynamicTree_Refit(bp.trees[b2_kinematicBody]);
		b2DynamicTree_Refit(bp.trees[b2_dynamicBody]);

		b2ValidateBroadphase(world.broadPhase);

		world.profile.refit = b2GetMilliseconds(refitTicks);
	}

	// Bullets are processed after the broad-phase refit so they can query the
	// final non-bullet world.
	let bulletBodyCount = b2AtomicLoadInt(b2StepAtomic(stepContext, SC_BULLETBODYCOUNT));
	if (bulletBodyCount > 0) {
		let bulletTicks = b2GetTicks();

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		let minRange = 8;
		b2ParallelFor<b2StepContext>(world, b2BulletBodyTask, bulletBodyCount, minRange, stepContext);

		let broadPhase = world.broadPhase;
		let dynamicTree = broadPhase.trees[b2_dynamicBody];

		// Fast array access is important here
		let bodySimArray = b2ArrayData(awakeSet.bodySims);
		let bulletBodySimIndices = stepContext.bulletBodies;

		// Serially enlarge broad-phase proxies for bullet shapes.
		// This loop has non-deterministic order but it shouldn't affect the result.
		for (let i = 0; i < bulletBodyCount; ++i) {
			// The bullet indices are awake sims b2FinalizeBodiesTask recorded
			let bulletBodySim = bodySimArray + <usize>bulletBodySimIndices[i] * sizeof<b2BodySim>();

			// It is worth tracking this flag because bullets may be moving slowly.
			let simFlags = load<u32>(bulletBodySim + SIM_FLAGS);
			if ((simFlags & b2_enlargeBulletBounds) == 0) {
				continue;
			}

			// Clear flag
			store<u32>(bulletBodySim + SIM_FLAGS, simFlags & ~b2_enlargeBulletBounds);

			let bodyId = load<i32>(bulletBodySim + SIM_BODYID);
			B2_ASSERT(0 <= bodyId && bodyId < world.bodies.length);

			let shapeId = world.bodies[bodyId].headShapeId;
			while (shapeId != B2_NULL_INDEX) {
				let proxyKey = world.shapes[shapeId].proxyKey;
				let proxyId = B2_PROXY_ID(proxyKey);

				let treeAABB = b2DynamicTree_GetProxyBounds(dynamicTree, proxyId);
				let shapeFatAABB = world.fatAABBs[shapeId];

				// Double check containment because a bullet can have multiple
				// shapes and maybe just one needs an update.
				if (b2AABB_Contains(treeAABB, shapeFatAABB) == false) {
					b2DynamicTree_EnlargeProxy(dynamicTree, proxyId, shapeFatAABB);
				}

				shapeId = world.shapes[shapeId].nextShapeId;
			}
		}

		world.profile.bullets = b2GetMilliseconds(bulletTicks);
	}

	// Need to free this even if no bullets got processed.
	stepContext.bulletBodies = b2_emptyIndices;
	b2AtomicStoreInt(b2StepAtomic(stepContext, SC_BULLETBODYCOUNT), 0);

	// Report sensor hits. This may include bullets sensor hits.
	{
		let sensorHitTicks = b2GetTicks();

		let workerCount = world.workerCount;
		B2_ASSERT(workerCount == world.taskContexts.length);

		for (let i = 0; i < workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			let hitCount = taskContext.sensorHits.length;
			let hits = taskContext.sensorHits;

			for (let j = 0; j < hitCount; ++j) {
				let hit = hits[j];
				let sensorIndex = world.shapes[hit.sensorId].sensorIndex;

				let sensor = world.sensors[sensorIndex];
				let shapeRef = b2Visitor();
				shapeRef.shapeId = hit.visitorId;
				shapeRef.generation = world.shapes[hit.visitorId].generation;
				sensor.hits.push(shapeRef);
			}
		}

		world.profile.sensorHits = b2GetMilliseconds(sensorHitTicks);
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if (world.enableSleep == true) {
		let sleepTicks = b2GetTicks();

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		B2_ASSERT(world.splitIslandId == B2_NULL_INDEX);
		let splitSleepTimer: f32 = 0.0;
		for (let i = 0; i < world.workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			if (taskContext.splitIslandId != B2_NULL_INDEX && taskContext.splitSleepTime >= splitSleepTimer) {
				B2_ASSERT(taskContext.splitSleepTime > 0.0);

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if (taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId) {
					continue;
				}

				world.splitIslandId = taskContext.splitIslandId;
				splitSleepTimer = taskContext.splitSleepTime;
			}
		}

		let awakeIslandBitSet = world.taskContexts[0].awakeIslandBitSet;
		for (let i = 1; i < world.workerCount; ++i) {
			b2InPlaceUnion(awakeIslandBitSet, world.taskContexts[i].awakeIslandBitSet);
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		let islands = awakeSet.islandSims;
		let count = islands.length;
		for (let islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1) {
			if (b2GetBit(awakeIslandBitSet, <u32>islandIndex) == true) {
				// this island is still awake
				continue;
			}

			let islandId = islands[islandIndex].islandId;

			b2TrySleepIsland(world, islandId);
		}

		world.profile.sleepIslands = b2GetMilliseconds(sleepTicks);
	}
}
