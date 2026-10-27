// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/physics_world.c

import {
	B2_ASSERT,
	B2_ASSERT_ENABLED,
	B2_NULL_INDEX,
	B2_SECRET_COOKIE,
	FLT_EPSILON,
	FLT_MAX,
	b2GetByteCount,
	b2GetMilliseconds,
	b2GetTicks,
	b2Log,
} from "./core";
import {
	B2_CONTACT_RECYCLE_COS_ANGLE,
	B2_CONTACT_RECYCLE_DISTANCE,
	B2_GRAPH_COLOR_COUNT,
	B2_LINEAR_SLOP,
	B2_MAX_RESTITUTION_ITERATIONS,
	B2_MAX_WORKERS,
	B2_MAX_WORLDS,
	B2_SPECULATIVE_DISTANCE,
} from "./constants";
import { b2BitSet, b2CreateBitSet, b2InPlaceUnion, b2SetBit, b2SetBitCountAndClear } from "./bitset";
import { b2CTZ64 } from "./ctz";
import { b2AllocId, b2CreateIdPool, b2GetIdCapacity, b2GetIdCount, b2IdPool } from "./id_pool";
import { b2Array_RemoveSwap } from "./container";
import {
	b2AABB,
	b2AABB_Union,
	b2AbsFloat,
	b2Add,
	b2ClampFloat,
	b2ClampInt,
	b2Cross,
	b2Distance,
	b2Dot,
	b2InvMulRot,
	b2InvMulWorldTransforms,
	b2InvTransformWorldPoint,
	b2InvertRot,
	b2IsNormalized,
	b2IsValidFloat,
	b2IsValidPos,
	b2IsValidVec2,
	b2LeftPerp,
	b2LengthSquared,
	b2MakeAABB,
	b2Max,
	b2MaxFloat,
	b2MaxInt,
	b2Min,
	b2MinFloat,
	b2MulAdd,
	b2MulRot,
	b2MulSV,
	b2Normalize,
	b2OffsetPos,
	b2Pos,
	b2Rot,
	b2RotateVector,
	b2Sub,
	b2SubPos,
	b2ToRelativeTransform,
	b2ToVec2,
	b2Transform,
	b2Transform_identity,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2IsValidAABB, b2OffsetAABB } from "./aabb";
import {
	b2BoxCastInput,
	b2Capsule,
	b2DistanceInput,
	b2RayCastInput,
	b2ShapeCastInput,
	b2ShapeProxy,
	b2SimplexCache,
	b2TreeStats,
} from "./collision";
import { b2ComputeShapeDistance, b2MakePointProxy } from "./distance";
import {
	b2DynamicTree_CastBox,
	b2DynamicTree_CastRay,
	b2DynamicTree_GatherMovedProxies,
	b2DynamicTree_GetHeight,
	b2DynamicTree_GetProxyCount,
	b2DynamicTree_GetRootBounds,
	b2DynamicTree_MarkProxyMovedSerial,
	b2DynamicTree_OverlapAABB,
	b2DynamicTree_Rebuild,
	b2HasTreeMoved,
} from "./dynamic_tree";
import { b2BodyId, b2ChainId, b2ContactId, b2JointId, b2ShapeId, b2WorldId } from "./id";
import { b2GetSIMDWidth, b2OverlapV } from "./simd";
import {
	b2BodyEvents,
	b2BodyMoveEvent,
	b2Capacity,
	b2CastResultCallbackFcn,
	b2ContactBeginTouchEvent,
	b2ContactEndTouchEvent,
	b2ContactEvents,
	b2ContactHitEvent,
	b2Counters,
	b2CustomFilterCallbackFcn,
	b2ExplosionDef,
	b2FrictionCallbackFcn,
	b2JointEvent,
	b2JointEvents,
	b2OverlapResultCallbackFcn,
	b2PlaneResultCallbackFcn,
	b2PreContinuousCallbackFcn,
	b2PreSolveCallbackFcn,
	b2Profile,
	b2QueryFilter,
	b2RayResult,
	b2RestitutionCallbackFcn,
	b2SensorBeginTouchEvent,
	b2SensorEndTouchEvent,
	b2SensorEvents,
	b2WorldDef,
	b2_bodyTypeCount,
	b2_dynamicBody,
	b2_kinematicBody,
	b2_staticBody,
} from "./types";
import { b2BroadPhase, b2CreateBroadPhase, b2UpdateBroadPhasePairs } from "./broad_phase";
import { b2AddContactToGraph, b2ConstraintGraph, b2CreateGraph, b2RemoveContactFromGraph } from "./constraint_graph";
import { b2ArrayData, b2Body, b2BodySim, b2GetBodySimAddress, b2GetBodyTransformQuick, b2IsStaticSimIndex, b2WakeBody } from "./body";
import { b2SolverSet, b2WakeSolverSet, b2_awakeSet, b2_disabledSet, b2_firstSleepingSet, b2_staticSet } from "./solver_set";
import { b2Joint } from "./joint";
import {
	b2Contact,
	b2ContactSim,
	b2ContactSimView,
	b2DestroyContact,
	b2UpdateContact,
	b2_contactEnableContactEvents,
	b2_contactRecycleFlag,
	b2_contactTouchingFlag,
	b2_simDisjoint,
	b2_simRelativeTransformValid,
	b2_simStartedTouching,
	b2_simStoppedTouching,
	b2_simTouchingFlag,
} from "./contact";
import { b2Island, b2LinkContact, b2UnlinkContact } from "./island";
import {
	b2ChainShape,
	b2CollideMover,
	b2GetShapeCentroid,
	b2GetShapeProjectedPerimeter,
	b2MakeShapeDistanceProxy,
	b2RayCastShape,
	b2Shape,
	b2ShapeCastShape,
	b2ShouldQueryCollide,
} from "./shape";
import { b2OverlapSensors, b2Sensor, b2SensorHit, b2SensorTaskContext } from "./sensor";
import {
	b2ContactCollideSpan,
	b2MakeSoft,
	b2Softness,
	b2Solve,
	b2StepContext,
	b2_emptyBodySims,
	b2_emptyBodyStates,
	b2_emptyContactSims,
} from "./solver";
import { b2ParallelFor } from "./parallel_for";

// Per thread task storage
export class b2TaskContext {
	// Collect per thread sensor continuous hit events.
	sensorHits: Array<b2SensorHit> = new Array<b2SensorHit>();

	// Broad-phase pairs.
	pairKeys: Array<u64> = new Array<u64>();

	// These bits align with the contact id capacity and signal a change in contact status
	contactStateBitSet: b2BitSet = new b2BitSet();

	// These bits align with the contact id capacity and signal a hit event.
	hitEventBitSet: b2BitSet = new b2BitSet();

	// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
	hasHitEvents: bool = false;

	// These bits align with the joint id capacity and signal a change in contact status
	jointStateBitSet: b2BitSet = new b2BitSet();

	// Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
	// that is very large when there are many static shapes.
	enlargedSimBitSet: b2BitSet = new b2BitSet();

	// Used to put islands to sleep
	awakeIslandBitSet: b2BitSet = new b2BitSet();

	// Per worker split island candidate
	splitSleepTime: f32 = 0;
	splitIslandId: i32 = 0;

	// Number of contacts recycled this step (collide pass).
	recycledContactCount: i32 = 0;
}

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
export class b2World {
	broadPhase: b2BroadPhase = new b2BroadPhase();
	constraintGraph: b2ConstraintGraph = new b2ConstraintGraph();

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b2Body.
	bodyIdPool: b2IdPool = new b2IdPool();

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	bodies: Array<b2Body> = new Array<b2Body>();

	// Provides free list for solver sets.
	solverSetIdPool: b2IdPool = new b2IdPool();

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	solverSets: Array<b2SolverSet> = new Array<b2SolverSet>();

	// Used to create stable ids for joints
	jointIdPool: b2IdPool = new b2IdPool();

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	joints: Array<b2Joint> = new Array<b2Joint>();

	// Used to create stable ids for contacts
	contactIdPool: b2IdPool = new b2IdPool();

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	contacts: Array<b2Contact> = new Array<b2Contact>();

	// Used to create stable ids for islands
	islandIdPool: b2IdPool = new b2IdPool();

	// Persistent islands
	islands: Array<b2Island> = new Array<b2Island>();

	shapeIdPool: b2IdPool = new b2IdPool();
	chainIdPool: b2IdPool = new b2IdPool();

	// These are sparse arrays that point into the pools above
	shapes: Array<b2Shape> = new Array<b2Shape>();

	// Shape enlarged bounds (hot data split).
	fatAABBs: Array<b2AABB> = new Array<b2AABB>();

	chainShapes: Array<b2ChainShape> = new Array<b2ChainShape>();

	// This is a dense array of sensor data.
	sensors: Array<b2Sensor> = new Array<b2Sensor>();

	// Per thread storage
	taskContexts: Array<b2TaskContext> = new Array<b2TaskContext>();
	sensorTaskContexts: Array<b2SensorTaskContext> = new Array<b2SensorTaskContext>();

	bodyMoveEvents: Array<b2BodyMoveEvent> = new Array<b2BodyMoveEvent>();
	sensorBeginEvents: Array<b2SensorBeginTouchEvent> = new Array<b2SensorBeginTouchEvent>();
	contactBeginEvents: Array<b2ContactBeginTouchEvent> = new Array<b2ContactBeginTouchEvent>();

	// End events are double buffered so that the user doesn't need to flush events
	sensorEndEvents: StaticArray<Array<b2SensorEndTouchEvent>> = new StaticArray<Array<b2SensorEndTouchEvent>>(2);
	contactEndEvents: StaticArray<Array<b2ContactEndTouchEvent>> = new StaticArray<Array<b2ContactEndTouchEvent>>(2);
	endEventArrayIndex: i32 = 0;

	contactHitEvents: Array<b2ContactHitEvent> = new Array<b2ContactHitEvent>();
	jointEvents: Array<b2JointEvent> = new Array<b2JointEvent>();

	// Id that is incremented every time step
	stepIndex: u64 = 0;

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	splitIslandId: i32 = 0;

	gravity: b2Vec2 = b2Vec2();
	hitEventThreshold: f32 = 0;
	restitutionThreshold: f32 = 0;
	restitutionIterations: i32 = 0;
	enableRestitutionPropagation: bool = false;
	maxLinearSpeed: f32 = 0;
	contactSpeed: f32 = 0;
	contactHertz: f32 = 0;
	contactDampingRatio: f32 = 0;
	contactRecycleDistance: f32 = 0;

	// Never null after b2CreateWorld, which installs the defaults as C does.
	frictionCallback: b2FrictionCallbackFcn? = null;
	restitutionCallback: b2RestitutionCallbackFcn? = null;

	generation: u16 = 0;

	profile: b2Profile = b2Profile();

	maxCapacity: b2Capacity = b2Capacity();

	preSolveFcn: b2PreSolveCallbackFcn? = null;
	preContinuousFcn: b2PreContinuousCallbackFcn? = null;

	customFilterFcn: b2CustomFilterCallbackFcn? = null;

	workerCount: i32 = 0;
	simdWidth: i32 = 0;
	userTaskContext: u64 = 0;

	userData: u64 = 0;

	// Remember type step used for reporting forces and torques
	// inverse sub-step
	inv_h: f32 = 0;

	// inverse full-step
	inv_dt: f32 = 0;

	activeTaskCount: i32 = 0;
	taskCount: i32 = 0;

	worldId: u16 = 0;

	enableSleep: bool = false;
	locked: bool = false;
	enableWarmStarting: bool = false;
	enableContinuous: bool = false;
	inUse: bool = false;

	// C's step context lives on the stack of b2World_Step; this one is reused every step
	stepContext: b2StepContext? = null;

	// Returned by the event getters, refreshed on each call
	bodyEvents: b2BodyEvents = new b2BodyEvents();
	sensorEvents: b2SensorEvents = new b2SensorEvents();
	contactEvents: b2ContactEvents = new b2ContactEvents();
	jointEventsView: b2JointEvents = new b2JointEvents();

	constructor() {
		for (let i = 0; i < 2; ++i) {
			this.sensorEndEvents[i] = new Array<b2SensorEndTouchEvent>();
			this.contactEndEvents[i] = new Array<b2ContactEndTouchEvent>();
		}
	}
}

// functions from physics_world.c follow

const b2_worlds = new StaticArray<b2World>(B2_MAX_WORLDS);
for (let i = 0; i < B2_MAX_WORLDS; ++i) {
	b2_worlds[i] = new b2World();
}

function b2GetUnlockedWorldFromId(id: b2WorldId): b2World? {
	B2_ASSERT(1 <= id.index1 && id.index1 <= B2_MAX_WORLDS);
	let world = b2_worlds[id.index1 - 1];
	B2_ASSERT(id.index1 == world.worldId + 1);
	B2_ASSERT(id.generation == world.generation);

	// A world accessed from an id should not be locked
	if (world.locked) {
		B2_ASSERT(false);
		return null;
	}
	return world;
}

export function b2GetWorldFromId(id: b2WorldId): b2World {
	B2_ASSERT(1 <= id.index1 && id.index1 <= B2_MAX_WORLDS);
	let world = b2_worlds[id.index1 - 1];
	B2_ASSERT(id.index1 == world.worldId + 1);
	B2_ASSERT(id.generation == world.generation);
	return world;
}

export function b2GetWorld(index: i32): b2World {
	B2_ASSERT(0 <= index && index < B2_MAX_WORLDS);
	let world = b2_worlds[index];
	B2_ASSERT(world.worldId == index);
	return world;
}

export function b2GetWorldLocked(index: i32): b2World? {
	B2_ASSERT(0 <= index && index < B2_MAX_WORLDS);
	let world = b2_worlds[index];
	B2_ASSERT(world.worldId == index);
	if (world.locked) {
		B2_ASSERT(false);
		return null;
	}

	return world;
}

function b2CreateWorkerContexts(world: b2World): void {
	world.taskContexts = new Array<b2TaskContext>();
	world.sensorTaskContexts = new Array<b2SensorTaskContext>();

	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = new b2TaskContext();
		taskContext.contactStateBitSet = b2CreateBitSet(1024);
		taskContext.hitEventBitSet = b2CreateBitSet(1024);
		taskContext.hasHitEvents = false;
		taskContext.jointStateBitSet = b2CreateBitSet(1024);
		taskContext.enlargedSimBitSet = b2CreateBitSet(256);
		taskContext.awakeIslandBitSet = b2CreateBitSet(256);
		taskContext.splitIslandId = B2_NULL_INDEX;
		world.taskContexts.push(taskContext);

		let sensorTaskContext = new b2SensorTaskContext();
		sensorTaskContext.eventBits = b2CreateBitSet(128);
		world.sensorTaskContexts.push(sensorTaskContext);
	}
}

export function b2CreateWorld(def: b2WorldDef): b2WorldId {
	B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	if (
		!(b2IsValidVec2(def.gravity)) ||
		!(b2IsValidFloat(def.restitutionThreshold)) ||
		!(def.restitutionIterations >= 0) ||
		!(b2IsValidFloat(def.hitEventThreshold)) ||
		!(b2IsValidFloat(def.contactHertz)) ||
		!(b2IsValidFloat(def.contactDampingRatio)) ||
		!(b2IsValidFloat(def.contactSpeed)) ||
		!(b2IsValidFloat(def.maximumLinearSpeed) && def.maximumLinearSpeed > 0.0)
	) {
		b2Log("invalid world definition");
		B2_ASSERT(false);
		return b2WorldId();
	}

	let worldId = B2_NULL_INDEX;
	for (let i = 0; i < B2_MAX_WORLDS; ++i) {
		if (b2_worlds[i].inUse == false) {
			worldId = i;
			break;
		}
	}

	if (worldId == B2_NULL_INDEX) {
		return b2WorldId();
	}

	let generation = b2_worlds[worldId].generation;

	let world = new b2World();
	b2_worlds[worldId] = world;

	world.worldId = <u16>worldId;
	world.generation = generation;
	world.inUse = true;

	b2CreateBroadPhase(world.broadPhase, def.capacity);
	b2CreateGraph(world.constraintGraph, def.capacity);

	// pools
	world.bodyIdPool = b2CreateIdPool();

	// add empty static, active, and disabled body sets
	world.solverSetIdPool = b2CreateIdPool();

	// static set
	{
		let set = new b2SolverSet();
		set.setIndex = b2AllocId(world.solverSetIdPool);
		world.solverSets.push(set);
		B2_ASSERT(world.solverSets[b2_staticSet].setIndex == b2_staticSet);
	}

	// disabled set
	{
		let set = new b2SolverSet();
		set.setIndex = b2AllocId(world.solverSetIdPool);
		world.solverSets.push(set);
		B2_ASSERT(world.solverSets[b2_disabledSet].setIndex == b2_disabledSet);
	}

	// awake set
	{
		let set = new b2SolverSet();
		set.setIndex = b2AllocId(world.solverSetIdPool);
		world.solverSets.push(set);
		B2_ASSERT(world.solverSets[b2_awakeSet].setIndex == b2_awakeSet);
	}

	world.shapeIdPool = b2CreateIdPool();
	world.chainIdPool = b2CreateIdPool();
	world.contactIdPool = b2CreateIdPool();
	world.jointIdPool = b2CreateIdPool();
	world.islandIdPool = b2CreateIdPool();

	world.endEventArrayIndex = 0;

	world.stepIndex = 0;
	world.splitIslandId = B2_NULL_INDEX;
	world.activeTaskCount = 0;
	world.taskCount = 0;
	world.gravity = def.gravity;
	world.hitEventThreshold = b2ClampFloat(def.hitEventThreshold, 0.0, FLT_MAX);
	world.restitutionThreshold = b2ClampFloat(def.restitutionThreshold, 0.0, FLT_MAX);
	// Clamp this to avoid overflowing the solver sync flags.
	world.restitutionIterations = b2ClampInt(def.restitutionIterations, 0, B2_MAX_RESTITUTION_ITERATIONS);
	world.enableRestitutionPropagation = def.enableRestitutionPropagation;
	world.maxLinearSpeed = def.maximumLinearSpeed;
	world.contactSpeed = b2ClampFloat(def.contactSpeed, 0.0, FLT_MAX);
	world.contactHertz = b2ClampFloat(def.contactHertz, 0.0, FLT_MAX);
	world.contactDampingRatio = b2ClampFloat(def.contactDampingRatio, 0.0, FLT_MAX);
	world.contactRecycleDistance = B2_CONTACT_RECYCLE_DISTANCE();

	if (def.frictionCallback != null) {
		world.frictionCallback = def.frictionCallback;
	}

	if (def.restitutionCallback != null) {
		world.restitutionCallback = def.restitutionCallback;
	}

	world.enableSleep = def.enableSleep;
	world.locked = false;
	world.enableWarmStarting = true;
	world.enableContinuous = def.enableContinuous;
	world.simdWidth = b2GetSIMDWidth();
	world.userData = def.userData;

	// Tasks run inline on one worker; C's built in scheduler and external task systems are not ported
	world.workerCount = 1;
	world.userTaskContext = 0;

	b2CreateWorkerContexts(world);

	world.stepContext = new b2StepContext(world);

	// add one to worldId so that 0 represents a null b2WorldId
	let id = b2WorldId();
	id.index1 = <u16>(worldId + 1);
	id.generation = world.generation;
	return id;
}

export function b2DestroyWorld(worldId: b2WorldId): void {
	let world = b2GetWorldFromId(worldId);

	// Wipe world but preserve generation. The arrays go with the world object.
	let generation = world.generation;
	let wiped = new b2World();
	wiped.worldId = 0;
	wiped.generation = generation + 1;
	b2_worlds[world.worldId] = wiped;
}

@inline function b2RelativeCos(a: b2Rot, b: b2Rot): f32 {
	return a.c * b.c + a.s * b.s;
}

// C returns a pointer to the sim; here its address, so the collide task reads only the fields it needs.
// Valid during the collide task, which never grows a sim array.
@inline unsafe function b2ResolveContactBodySim(
	world: b2World,
	awakeSims: usize,
	staticSims: usize,
	encodedIndex: i32,
	bodyId: i32,
): usize {
	if (encodedIndex >= 0) {
		return awakeSims + <usize>encodedIndex * sizeof<b2BodySim>();
	}

	if (b2IsStaticSimIndex(encodedIndex)) {
		return staticSims + <usize>(-encodedIndex - 2) * sizeof<b2BodySim>();
	}

	return b2GetBodySimAddress(world, bodyId);
}

unsafe function b2CollideTask(startIndex: i32, endIndex: i32, workerIndex: i32, stepContext: b2StepContext): void {
	let world = stepContext.world;
	let taskContext = world.taskContexts[workerIndex];
	let spans = stepContext.collideSpans;
	let fatAABBs = world.fatAABBs;
	let awakeSims = b2ArrayData(world.solverSets[b2_awakeSet].bodySims);
	let staticSims = b2ArrayData(world.solverSets[b2_staticSet].bodySims);

	B2_ASSERT(startIndex < endIndex);

	let spanIndex = 0;
	while (spans[spanIndex + 1].start <= startIndex) {
		spanIndex += 1;
	}

	let spanStart = spans[spanIndex].start;
	let spanEnd = spans[spanIndex + 1].start;
	let spanBase = b2ContactSimView(spans[spanIndex].contacts);

	let recycleDistance = world.contactRecycleDistance;
	let speculativeDistance = B2_SPECULATIVE_DISTANCE();
	let recycleDistanceNonTouching = b2MinFloat(recycleDistance, speculativeDistance);

	for (let contactIndex = startIndex; contactIndex < endIndex; ++contactIndex) {
		if (contactIndex == spanEnd) {
			spanIndex += 1;
			spanStart = spans[spanIndex].start;
			spanEnd = spans[spanIndex + 1].start;
			spanBase = b2ContactSimView(spans[spanIndex].contacts);
		}

		ref contactSim = spanBase[contactIndex - spanStart];

		let contactId = contactSim.contactId;
		let shapeIdA = contactSim.shapeIdA;
		let shapeIdB = contactSim.shapeIdB;

		// Do proxies still overlap?
		let overlap = b2OverlapV(fatAABBs[shapeIdA], fatAABBs[shapeIdB]);
		if (overlap == false) {
			contactSim.simFlags |= b2_simDisjoint;
			contactSim.simFlags &= ~b2_simTouchingFlag;
			b2SetBit(taskContext.contactStateBitSet, <u32>contactId);
			continue;
		}

		let wasTouching = (contactSim.simFlags & b2_simTouchingFlag) != 0;

		// Update contact respecting shape/body order (A,B)
		let encodedA = contactSim.encodedBodySimA;
		let encodedB = contactSim.encodedBodySimB;
		let bodySimA = b2ResolveContactBodySim(world, awakeSims, staticSims, encodedA, world.shapes[shapeIdA].bodyId);
		let bodySimB = b2ResolveContactBodySim(world, awakeSims, staticSims, encodedB, world.shapes[shapeIdB].bodyId);
		let transformA = load<b2Transform>(bodySimA + offsetof<b2BodySim>("transform"));
		let transformB = load<b2Transform>(bodySimB + offsetof<b2BodySim>("transform"));

		// These may not be skipped by relative transform check below
		contactSim.invMassA = load<f32>(bodySimA + offsetof<b2BodySim>("invMass"));
		contactSim.invIA = load<f32>(bodySimA + offsetof<b2BodySim>("invInertia"));
		contactSim.invMassB = load<f32>(bodySimB + offsetof<b2BodySim>("invMass"));
		contactSim.invIB = load<f32>(bodySimB + offsetof<b2BodySim>("invInertia"));

		// Contact recycling optimization. Please cite this code if you use this optimization.
		// This is inspired by persistent contact manifolds used in some physics engines, such as PhysX.
		// However, this allows larger relative motion and has fewer tuning parameters (just one).
		if (
			recycleDistance > 0.0 &&
			(contactSim.simFlags & b2_simRelativeTransformValid) != 0 &&
			(contactSim.simFlags & b2_contactRecycleFlag) != 0
		) {
			let cachedQA = contactSim.cachedRotationA;
			let cachedQB = contactSim.cachedRotationB;
			let xfc = contactSim.cachedRelativePose;
			let xf = b2InvMulWorldTransforms(transformA, transformB);

			let cosA = b2RelativeCos(transformA.q, cachedQA);
			let cosB = b2RelativeCos(transformB.q, cachedQB);
			let minCos = b2MinFloat(cosA, cosB);

			let maxExtentA: f32 = b2IsStaticSimIndex(encodedA) ? 0.0 : load<f32>(bodySimA + offsetof<b2BodySim>("maxExtent"));
			let maxExtentB: f32 = b2IsStaticSimIndex(encodedB) ? 0.0 : load<f32>(bodySimB + offsetof<b2BodySim>("maxExtent"));
			let maxExtent = b2MaxFloat(maxExtentA, maxExtentB);
			let distance = b2Distance(xf.p, xfc.p);
			let qr = b2InvMulRot(xf.q, xfc.q);

			// This metric is used for fast bodies and sleeping. It comes from conservative advancement.
			// Note that qr.s == sin(theta) ~= theta for small angles.
			// Need a tighter tolerance for non-touching shapes so that contacts are not missed.
			let tolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

			if (minCos > B2_CONTACT_RECYCLE_COS_ANGLE && distance + maxExtent * b2AbsFloat(qr.s) < tolerance) {
				let dqA = b2MulRot(transformA.q, b2InvertRot(cachedQA));
				let dqB = b2MulRot(transformB.q, b2InvertRot(cachedQB));
				let normal = contactSim.manifold.normal;

				// Minimize round-off
				let dc = b2SubPos(
					load<b2Vec2>(bodySimB + offsetof<b2BodySim>("center")),
					load<b2Vec2>(bodySimA + offsetof<b2BodySim>("center")),
				);

				for (let i = 0; i < contactSim.manifold.pointCount; ++i) {
					// Keep anchors but update separation, same as sub-stepping. This eliminates jitter.
					ref mp = contactSim.manifold.points[i];
					let rA = b2RotateVector(dqA, mp.anchorA);
					let rB = b2RotateVector(dqB, mp.anchorB);
					let dp = b2Add(dc, b2Sub(rB, rA));

					mp.separation = mp.baseSeparation + b2Dot(dp, normal);
					mp.normalVelocity = 0.0;
					mp.persisted = true;
				}

				taskContext.recycledContactCount += 1;

				// Contact is recycled. This also skips updating other aspects of the contact
				// such as material parameters.
				continue;
			}
		}

		// Caching for contact recycling.
		contactSim.cachedRotationA = transformA.q;
		contactSim.cachedRotationB = transformB.q;
		contactSim.cachedRelativePose = b2InvMulWorldTransforms(transformA, transformB);
		contactSim.simFlags |= b2_simRelativeTransformValid;

		let centerOffsetA = b2RotateVector(transformA.q, load<b2Vec2>(bodySimA + offsetof<b2BodySim>("localCenter")));
		let centerOffsetB = b2RotateVector(transformB.q, load<b2Vec2>(bodySimB + offsetof<b2BodySim>("localCenter")));

		// This updates solid contacts
		let touching = b2UpdateContact(world, contactSim, shapeIdA, transformA, centerOffsetA, shapeIdB, transformB, centerOffsetB);

		// State changes that affect island connectivity. Also affects contact events.
		if (touching == true && wasTouching == false) {
			contactSim.simFlags |= b2_simStartedTouching;
			b2SetBit(taskContext.contactStateBitSet, <u32>contactId);
		} else if (touching == false && wasTouching == true) {
			contactSim.simFlags |= b2_simStoppedTouching;
			b2SetBit(taskContext.contactStateBitSet, <u32>contactId);
		}

		if (touching) {
			for (let i = 0; i < contactSim.manifold.pointCount; ++i) {
				// Cache separation
				ref mp = contactSim.manifold.points[i];
				mp.baseSeparation = mp.separation;
			}
		}
	}
}

function b2AddNonTouchingContact(world: b2World, contactId: i32, contactSim: b2ContactSim): void {
	B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet);
	let set = world.solverSets[b2_awakeSet];
	world.contacts[contactId].colorIndex = B2_NULL_INDEX;
	world.contacts[contactId].localIndex = set.contactSims.length;

	set.contactSims.push(contactSim);
}

function b2RemoveNonTouchingContact(world: b2World, setIndex: i32, localIndex: i32): void {
	let set = world.solverSets[setIndex];
	let movedIndex = b2Array_RemoveSwap(set.contactSims, localIndex);
	if (movedIndex != B2_NULL_INDEX) {
		let movedContactId = set.contactSims[localIndex].contactId;
		if (B2_ASSERT_ENABLED) {
			B2_ASSERT(world.contacts[movedContactId].setIndex == setIndex);
			B2_ASSERT(world.contacts[movedContactId].localIndex == movedIndex);
			B2_ASSERT(world.contacts[movedContactId].colorIndex == B2_NULL_INDEX);
		}
		world.contacts[movedContactId].localIndex = localIndex;
	}
}

function b2MakeContactShapeId(world: b2World, shapeId: i32): b2ShapeId {
	let id = b2ShapeId();
	id.index1 = world.shapes[shapeId].id + 1;
	id.world0 = world.worldId;
	id.generation = world.shapes[shapeId].generation;
	return id;
}

function b2MakeFullContactId(world: b2World, contactId: i32): b2ContactId {
	let id = b2ContactId();
	id.index1 = contactId + 1;
	id.world0 = world.worldId;
	id.padding = 0;
	id.generation = world.contacts[contactId].generation;
	return id;
}

// C's stack array of B2_GRAPH_COLOR_COUNT + 2 spans, kept across steps
const b2_collideSpans = b2MakeCollideSpans();

function b2MakeCollideSpans(): StaticArray<b2ContactCollideSpan> {
	let spans = new StaticArray<b2ContactCollideSpan>(B2_GRAPH_COLOR_COUNT + 2);
	for (let i = 0; i < spans.length; ++i) {
		spans[i] = new b2ContactCollideSpan();
	}
	return spans;
}

// Narrow-phase collision
function b2Collide(context: b2StepContext): void {
	let world = context.world;

	B2_ASSERT(world.workerCount > 0);

	// gather contacts into a single array for easier parallel-for
	let contactCount = 0;
	let graphColors = world.constraintGraph.colors;
	for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
		contactCount += graphColors[i].contactSims.length;
	}

	let nonTouchingCount = world.solverSets[b2_awakeSet].contactSims.length;
	contactCount += nonTouchingCount;

	if (contactCount == 0) {
		return;
	}

	let collideSpans = b2_collideSpans;
	let spanCount = 0;
	let contactIndex = 0;
	for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
		let color = graphColors[i];
		let count = color.contactSims.length;
		if (count > 0) {
			collideSpans[spanCount].start = contactIndex;
			collideSpans[spanCount].contacts = color.contactSims;
			spanCount += 1;
			contactIndex += count;
		}
	}

	if (nonTouchingCount > 0) {
		collideSpans[spanCount].start = contactIndex;
		collideSpans[spanCount].contacts = world.solverSets[b2_awakeSet].contactSims;
		spanCount += 1;
		contactIndex += nonTouchingCount;
	}

	B2_ASSERT(contactIndex == contactCount);
	B2_ASSERT(spanCount <= B2_GRAPH_COLOR_COUNT + 1);

	collideSpans[spanCount].start = contactCount;
	collideSpans[spanCount].contacts = b2_emptyContactSims;

	context.collideSpans = collideSpans;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	let contactIdCapacity = b2GetIdCapacity(world.contactIdPool);
	for (let i = 0; i < world.workerCount; ++i) {
		b2SetBitCountAndClear(world.taskContexts[i].contactStateBitSet, <u32>contactIdCapacity);
		world.taskContexts[i].recycledContactCount = 0;
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	let minRange = 64;
	b2ParallelFor<b2StepContext>(world, b2CollideTask, contactCount, minRange, context);

	// Drop the array references so a destroyed color array is not kept alive
	for (let i = 0; i <= spanCount; ++i) {
		collideSpans[i].contacts = b2_emptyContactSims;
	}

	// Serially update contact state

	// Bitwise OR all contact bits
	let bitSet = world.taskContexts[0].contactStateBitSet;
	for (let i = 1; i < world.workerCount; ++i) {
		b2InPlaceUnion(bitSet, world.taskContexts[i].contactStateBitSet);
	}

	let awakeSet = world.solverSets[b2_awakeSet];

	let endEventArrayIndex = world.endEventArrayIndex;

	// Process contact state changes. Iterate over set bits
	for (let k: u32 = 0; k < bitSet.blockCount; ++k) {
		let bits = bitSet.bits[k];
		while (bits != 0) {
			let ctz = b2CTZ64(bits);
			let contactId = <i32>(64 * k + ctz);

			B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet);

			let colorIndex = world.contacts[contactId].colorIndex;
			let localIndex = world.contacts[contactId].localIndex;

			let contactSims: Array<b2ContactSim>;
			if (colorIndex != B2_NULL_INDEX) {
				// contact lives in constraint graph
				B2_ASSERT(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
				contactSims = graphColors[colorIndex].contactSims;
			} else {
				contactSims = awakeSet.contactSims;
			}

			let flags = world.contacts[contactId].flags;
			let simFlags = contactSims[localIndex].simFlags;

			if ((simFlags & b2_simDisjoint) != 0) {
				// Bounding boxes no longer overlap
				b2DestroyContact(world, contactId);
			} else if ((simFlags & b2_simStartedTouching) != 0) {
				B2_ASSERT(world.contacts[contactId].islandId == B2_NULL_INDEX);

				if ((flags & b2_contactEnableContactEvents) != 0) {
					let event = b2ContactBeginTouchEvent();
					event.shapeIdA = b2MakeContactShapeId(world, world.contacts[contactId].shapeIdA);
					event.shapeIdB = b2MakeContactShapeId(world, world.contacts[contactId].shapeIdB);
					event.contactId = b2MakeFullContactId(world, contactId);
					world.contactBeginEvents.push(event);
				}

				if (B2_ASSERT_ENABLED) {
					B2_ASSERT(contactSims[localIndex].manifold.pointCount > 0);
					B2_ASSERT(world.contacts[contactId].setIndex == b2_awakeSet);
				}

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				world.contacts[contactId].flags |= b2_contactTouchingFlag;
				b2LinkContact(world, contactId);

				// Make sure these didn't change
				B2_ASSERT(world.contacts[contactId].colorIndex == B2_NULL_INDEX);
				B2_ASSERT(world.contacts[contactId].localIndex == localIndex);

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.
				let awakeSims = b2ContactSimView(awakeSet.contactSims);
				ref contactSim = awakeSims[localIndex];

				contactSim.simFlags &= ~b2_simStartedTouching;

				// Add first for memcpy
				b2AddContactToGraph(world, contactSim, contactId);

				// This destroys the contact sim
				b2RemoveNonTouchingContact(world, b2_awakeSet, localIndex);
			} else if ((simFlags & b2_simStoppedTouching) != 0) {
				let contactSim = contactSims[localIndex];
				contactSim.simFlags &= ~b2_simStoppedTouching;
				world.contacts[contactId].flags &= ~b2_contactTouchingFlag;

				if ((world.contacts[contactId].flags & b2_contactEnableContactEvents) != 0) {
					let event = b2ContactEndTouchEvent();
					event.shapeIdA = b2MakeContactShapeId(world, world.contacts[contactId].shapeIdA);
					event.shapeIdB = b2MakeContactShapeId(world, world.contacts[contactId].shapeIdB);
					event.contactId = b2MakeFullContactId(world, contactId);
					world.contactEndEvents[endEventArrayIndex].push(event);
				}

				B2_ASSERT(contactSim.manifold.pointCount == 0);

				b2UnlinkContact(world, contactId);
				let bodyIdA = world.contacts[contactId].edges[0].bodyId;
				let bodyIdB = world.contacts[contactId].edges[1].bodyId;

				// Add first for memcpy
				b2AddNonTouchingContact(world, contactId, contactSim);
				b2RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex);
			}

			// Clear the smallest set bit
			bits = bits & (bits - 1);
		}
	}
}

export function b2World_Step(worldId: b2WorldId, timeStep: f32, subStepCount: i32): void {
	if (!b2IsValidFloat(timeStep)) {
		b2Log("invalid time step");
		B2_ASSERT(false);
		return;
	}
	B2_ASSERT(0 < subStepCount);

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	world.bodyMoveEvents.length = 0;
	world.sensorBeginEvents.length = 0;
	world.contactBeginEvents.length = 0;
	world.contactHitEvents.length = 0;
	world.jointEvents.length = 0;

	world.profile = b2Profile();

	world.locked = true;
	world.activeTaskCount = 0;
	world.taskCount = 0;

	let stepTicks = b2GetTicks();

	{
		let staticShapeCount = world.broadPhase.trees[b2_staticBody].proxyCount;
		let dynamicShapeCount = world.broadPhase.trees[b2_dynamicBody].proxyCount;
		world.maxCapacity.staticShapeCount = b2MaxInt(world.maxCapacity.staticShapeCount, staticShapeCount);
		world.maxCapacity.dynamicShapeCount = b2MaxInt(world.maxCapacity.dynamicShapeCount, dynamicShapeCount);

		let staticBodyCount = world.solverSets[b2_staticSet].bodySims.length;
		world.maxCapacity.staticBodyCount = b2MaxInt(world.maxCapacity.staticBodyCount, staticBodyCount);

		// this includes kinematic bodies
		let totalBodyCount = b2GetIdCount(world.bodyIdPool);
		world.maxCapacity.dynamicBodyCount = b2MaxInt(world.maxCapacity.dynamicBodyCount, totalBodyCount - staticBodyCount);

		let totalContactCount = b2GetIdCount(world.contactIdPool);
		world.maxCapacity.contactCount = b2MaxInt(world.maxCapacity.contactCount, totalContactCount);
	}

	// Update collision pairs and create contacts
	{
		let pairTicks = b2GetTicks();
		b2UpdateBroadPhasePairs(world);
		world.profile.pairs = b2GetMilliseconds(pairTicks);
	}

	// C zero initializes a stack context; this one is reused across steps
	let context = world.stepContext!;
	b2ResetStepContext(context);
	context.dt = timeStep;
	context.subStepCount = b2MaxInt(1, subStepCount);

	if (timeStep > 0.0) {
		context.inv_dt = 1.0 / timeStep;
		context.h = timeStep / <f32>context.subStepCount;
		context.inv_h = <f32>context.subStepCount * context.inv_dt;
	} else {
		context.inv_dt = 0.0;
		context.h = 0.0;
		context.inv_h = 0.0;
	}

	world.inv_h = context.inv_h;
	world.inv_dt = context.inv_dt;

	// Hertz values get reduced for large time steps
	let contactHertz = b2MinFloat(world.contactHertz, 0.125 * context.inv_h);
	context.contactSoftness = b2MakeSoft(contactHertz, world.contactDampingRatio, context.h);
	context.staticSoftness = b2MakeSoft(2.0 * contactHertz, world.contactDampingRatio, context.h);

	context.restitutionThreshold = world.restitutionThreshold;
	context.maxLinearVelocity = world.maxLinearSpeed;
	context.enableWarmStarting = world.enableWarmStarting;

	// Narrow phase : update contacts
	{
		let collideTicks = b2GetTicks();
		b2Collide(context);
		world.profile.collide = b2GetMilliseconds(collideTicks);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (timeStep > 0.0) {
		let solveTicks = b2GetTicks();
		b2Solve(world, context);
		world.profile.solve = b2GetMilliseconds(solveTicks);
	}

	// Update sensors
	{
		let sensorTicks = b2GetTicks();
		b2OverlapSensors(world);
		world.profile.sensors = b2GetMilliseconds(sensorTicks);
	}

	world.profile.step = b2GetMilliseconds(stepTicks);

	// Make sure all tasks that were started were also finished
	B2_ASSERT(world.activeTaskCount == 0);

	// Swap end event array buffers
	world.endEventArrayIndex = 1 - world.endEventArrayIndex;
	world.sensorEndEvents[world.endEventArrayIndex].length = 0;
	world.contactEndEvents[world.endEventArrayIndex].length = 0;

	world.locked = false;
}

// The fields of C's zero initialized step context that are read before b2World_Step or b2Solve writes them
function b2ResetStepContext(context: b2StepContext): void {
	context.dt = 0.0;
	context.inv_dt = 0.0;
	context.h = 0.0;
	context.inv_h = 0.0;
	context.subStepCount = 0;
	context.contactSoftness = b2Softness();
	context.staticSoftness = b2Softness();
	context.restitutionThreshold = 0.0;
	context.maxLinearVelocity = 0.0;
	context.states = b2_emptyBodyStates;
	context.sims = b2_emptyBodySims;
	context.enlargedShapeCount = 0;
	context.bulletBodyCount.value = 0;
	context.anyRestitution.value = 0;
	context.wideContactCount = 0;
	context.jointCount = 0;
	context.activeColorCount = 0;
	context.workerCount = 0;
	context.stageCount = 0;
	context.enableWarmStarting = false;
	context.atomicSyncBits.value = 0;
	context.mainClaimed.value = 0;
}

export function b2ComputeWorldBounds(world: b2World, bounds: ref b2AABB): bool {
	let worldBounds = b2AABB();
	let haveBounds = false;

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		let tree = world.broadPhase.trees[i];
		if (b2DynamicTree_GetProxyCount(tree) == 0) {
			continue;
		}

		let treeBounds = b2DynamicTree_GetRootBounds(tree);
		worldBounds = haveBounds ? b2AABB_Union(worldBounds, treeBounds) : treeBounds;
		haveBounds = true;
	}

	bounds = worldBounds;
	return haveBounds;
}

export function b2World_GetBounds(worldId: b2WorldId): b2AABB {
	let world = b2GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return b2AABB();
	}

	let bounds = b2AABB();
	b2ComputeWorldBounds(world, bounds);
	return bounds;
}

// The returned object is the world's, refreshed by each call
export function b2World_GetBodyEvents(worldId: b2WorldId): b2BodyEvents {
	let world = b2GetWorldFromId(worldId);
	let events = world.bodyEvents;
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		events.moveEvents = world.bodyMoveEvents;
		events.moveCount = 0;
		return events;
	}

	events.moveEvents = world.bodyMoveEvents;
	events.moveCount = world.bodyMoveEvents.length;
	return events;
}

export function b2World_GetSensorEvents(worldId: b2WorldId): b2SensorEvents {
	let world = b2GetWorldFromId(worldId);
	let events = world.sensorEvents;
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		events.beginCount = 0;
		events.endCount = 0;
		return events;
	}

	// Careful to use previous buffer
	let endEventArrayIndex = 1 - world.endEventArrayIndex;

	events.beginEvents = world.sensorBeginEvents;
	events.endEvents = world.sensorEndEvents[endEventArrayIndex];
	events.beginCount = world.sensorBeginEvents.length;
	events.endCount = world.sensorEndEvents[endEventArrayIndex].length;
	return events;
}

export function b2World_GetContactEvents(worldId: b2WorldId): b2ContactEvents {
	let world = b2GetWorldFromId(worldId);
	let events = world.contactEvents;
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		events.beginCount = 0;
		events.endCount = 0;
		events.hitCount = 0;
		return events;
	}

	// Careful to use previous buffer
	let endEventArrayIndex = 1 - world.endEventArrayIndex;

	events.beginEvents = world.contactBeginEvents;
	events.endEvents = world.contactEndEvents[endEventArrayIndex];
	events.hitEvents = world.contactHitEvents;
	events.beginCount = world.contactBeginEvents.length;
	events.endCount = world.contactEndEvents[endEventArrayIndex].length;
	events.hitCount = world.contactHitEvents.length;
	return events;
}

export function b2World_GetJointEvents(worldId: b2WorldId): b2JointEvents {
	let world = b2GetWorldFromId(worldId);
	let events = world.jointEventsView;
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		events.count = 0;
		return events;
	}

	events.jointEvents = world.jointEvents;
	events.count = world.jointEvents.length;
	return events;
}

export function b2World_IsValid(id: b2WorldId): bool {
	if (id.index1 < 1 || B2_MAX_WORLDS < id.index1) {
		return false;
	}

	let world = b2_worlds[id.index1 - 1];

	if (world.worldId != id.index1 - 1) {
		// world is not allocated
		return false;
	}

	return id.generation == world.generation;
}

export function b2Body_IsValid(id: b2BodyId): bool {
	if (B2_MAX_WORLDS <= id.world0) {
		// invalid world
		return false;
	}

	let world = b2_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	if (id.index1 < 1 || world.bodies.length < id.index1) {
		// invalid index
		return false;
	}

	let index = id.index1 - 1;
	if (world.bodies[index].setIndex == B2_NULL_INDEX) {
		// this was freed
		return false;
	}

	B2_ASSERT(world.bodies[index].localIndex != B2_NULL_INDEX);

	if (world.bodies[index].generation != id.generation) {
		// this id is orphaned
		return false;
	}

	return true;
}

export function b2Shape_IsValid(id: b2ShapeId): bool {
	if (B2_MAX_WORLDS <= id.world0) {
		return false;
	}

	let world = b2_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let shapeId = id.index1 - 1;
	if (shapeId < 0 || world.shapes.length <= shapeId) {
		return false;
	}

	if (world.shapes[shapeId].id == B2_NULL_INDEX) {
		// shape is free
		return false;
	}

	B2_ASSERT(world.shapes[shapeId].id == shapeId);

	return id.generation == world.shapes[shapeId].generation;
}

export function b2Chain_IsValid(id: b2ChainId): bool {
	if (B2_MAX_WORLDS <= id.world0) {
		return false;
	}

	let world = b2_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let chainId = id.index1 - 1;
	if (chainId < 0 || world.chainShapes.length <= chainId) {
		return false;
	}

	let chain = world.chainShapes[chainId];
	if (chain.id == B2_NULL_INDEX) {
		// chain is free
		return false;
	}

	B2_ASSERT(chain.id == chainId);

	return id.generation == chain.generation;
}

export function b2Joint_IsValid(id: b2JointId): bool {
	if (B2_MAX_WORLDS <= id.world0) {
		return false;
	}

	let world = b2_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let jointId = id.index1 - 1;
	if (jointId < 0 || world.joints.length <= jointId) {
		return false;
	}

	if (world.joints[jointId].jointId == B2_NULL_INDEX) {
		// joint is free
		return false;
	}

	B2_ASSERT(world.joints[jointId].jointId == jointId);

	return id.generation == world.joints[jointId].generation;
}

export function b2Contact_IsValid(id: b2ContactId): bool {
	if (B2_MAX_WORLDS <= id.world0) {
		return false;
	}

	let world = b2_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let contactId = id.index1 - 1;
	if (contactId < 0 || world.contacts.length <= contactId) {
		return false;
	}

	if (world.contacts[contactId].contactId == B2_NULL_INDEX) {
		// contact is free
		return false;
	}

	B2_ASSERT(world.contacts[contactId].contactId == contactId);

	return id.generation == world.contacts[contactId].generation;
}

export function b2World_EnableSleeping(worldId: b2WorldId, flag: bool): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	if (flag == world.enableSleep) {
		return;
	}

	world.enableSleep = flag;

	if (flag == false) {
		let setCount = world.solverSets.length;
		for (let i = b2_firstSleepingSet; i < setCount; ++i) {
			let set = world.solverSets[i];
			if (set.bodySims.length > 0) {
				b2WakeSolverSet(world, i);
			}
		}
	}
}

export function b2World_IsSleepingEnabled(worldId: b2WorldId): bool {
	let world = b2GetWorldFromId(worldId);
	return world.enableSleep;
}

export function b2World_EnableWarmStarting(worldId: b2WorldId, flag: bool): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.enableWarmStarting = flag;
}

export function b2World_IsWarmStartingEnabled(worldId: b2WorldId): bool {
	let world = b2GetWorldFromId(worldId);
	return world.enableWarmStarting;
}

export function b2World_EnableSSE2Fallback(worldId: b2WorldId, flag: bool): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.simdWidth = flag ? 4 : b2GetSIMDWidth();
}

export function b2World_GetAwakeBodyCount(worldId: b2WorldId): i32 {
	let world = b2GetWorldFromId(worldId);
	let awakeSet = world.solverSets[b2_awakeSet];
	return awakeSet.bodySims.length;
}

export function b2World_EnableContinuous(worldId: b2WorldId, flag: bool): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.enableContinuous = flag;
}

export function b2World_IsContinuousEnabled(worldId: b2WorldId): bool {
	let world = b2GetWorldFromId(worldId);
	return world.enableContinuous;
}

export function b2World_SetRestitutionThreshold(worldId: b2WorldId, value: f32): void {
	if (!b2IsValidFloat(value)) {
		b2Log("invalid restitution threshold");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.restitutionThreshold = b2ClampFloat(value, 0.0, FLT_MAX);
}

export function b2World_GetRestitutionThreshold(worldId: b2WorldId): f32 {
	let world = b2GetWorldFromId(worldId);
	return world.restitutionThreshold;
}

export function b2World_SetRestitutionIterations(worldId: b2WorldId, iterations: i32): void {
	if (!(iterations >= 0)) {
		b2Log("invalid restitution iterations");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.restitutionIterations = b2ClampInt(iterations, 0, B2_MAX_RESTITUTION_ITERATIONS);
}

export function b2World_GetRestitutionIterations(worldId: b2WorldId): i32 {
	let world = b2GetWorldFromId(worldId);
	return world.restitutionIterations;
}

export function b2World_EnableRestitutionPropagation(worldId: b2WorldId, flag: bool): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.enableRestitutionPropagation = flag;
}

export function b2World_IsRestitutionPropagationEnabled(worldId: b2WorldId): bool {
	let world = b2GetWorldFromId(worldId);
	return world.enableRestitutionPropagation;
}

export function b2World_SetHitEventThreshold(worldId: b2WorldId, value: f32): void {
	if (!b2IsValidFloat(value)) {
		b2Log("invalid hit event threshold");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.hitEventThreshold = b2ClampFloat(value, 0.0, FLT_MAX);
}

export function b2World_GetHitEventThreshold(worldId: b2WorldId): f32 {
	let world = b2GetWorldFromId(worldId);
	return world.hitEventThreshold;
}

export function b2World_SetContactTuning(worldId: b2WorldId, hertz: f32, dampingRatio: f32, pushSpeed: f32): void {
	if (!b2IsValidFloat(hertz) || !b2IsValidFloat(dampingRatio) || !b2IsValidFloat(pushSpeed)) {
		b2Log("invalid contact tuning");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.contactHertz = b2ClampFloat(hertz, 0.0, FLT_MAX);
	world.contactDampingRatio = b2ClampFloat(dampingRatio, 0.0, FLT_MAX);
	world.contactSpeed = b2ClampFloat(pushSpeed, 0.0, FLT_MAX);
}

export function b2World_GetContactTuning(worldId: b2WorldId, hertz: ref f32, dampingRatio: ref f32, pushSpeed: ref f32): void {
	let world = b2GetWorldFromId(worldId);
	hertz = world.contactHertz;
	dampingRatio = world.contactDampingRatio;
	pushSpeed = world.contactSpeed;
}

export function b2World_SetContactRecycleDistance(worldId: b2WorldId, recycleDistance: f32): void {
	if (!b2IsValidFloat(recycleDistance)) {
		b2Log("invalid contact recycle distance");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.contactRecycleDistance = b2ClampFloat(recycleDistance, 0.0, FLT_MAX);
}

export function b2World_GetContactRecycleDistance(worldId: b2WorldId): f32 {
	let world = b2GetWorldFromId(worldId);
	return world.contactRecycleDistance;
}

export function b2World_SetMaxLinearSpeed(worldId: b2WorldId, maximumLinearSpeed: f32): void {
	if (!(b2IsValidFloat(maximumLinearSpeed) && maximumLinearSpeed > 0.0)) {
		b2Log("invalid maximum linear speed");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.maxLinearSpeed = maximumLinearSpeed;
}

export function b2World_GetMaxLinearSpeed(worldId: b2WorldId): f32 {
	let world = b2GetWorldFromId(worldId);
	return world.maxLinearSpeed;
}

export function b2World_GetProfile(worldId: b2WorldId): b2Profile {
	let world = b2GetWorldFromId(worldId);
	return world.profile;
}

export function b2World_GetCounters(worldId: b2WorldId): b2Counters {
	let world = b2GetWorldFromId(worldId);
	let s = b2Counters();
	s.bodyCount = b2GetIdCount(world.bodyIdPool);
	s.shapeCount = b2GetIdCount(world.shapeIdPool);
	s.contactCount = b2GetIdCount(world.contactIdPool);
	s.jointCount = b2GetIdCount(world.jointIdPool);
	s.islandCount = b2GetIdCount(world.islandIdPool);

	let staticTree = world.broadPhase.trees[b2_staticBody];
	s.staticTreeHeight = b2DynamicTree_GetHeight(staticTree);

	let dynamicTree = world.broadPhase.trees[b2_dynamicBody];
	let kinematicTree = world.broadPhase.trees[b2_kinematicBody];
	s.treeHeight = b2MaxInt(b2DynamicTree_GetHeight(dynamicTree), b2DynamicTree_GetHeight(kinematicTree));

	// No stack allocator: C's stack high water mark has no equivalent
	s.stackUsed = 0;
	s.byteCount = b2GetByteCount();
	s.taskCount = world.taskCount;

	s.recycledContactCount = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		s.recycledContactCount += world.taskContexts[i].recycledContactCount;
	}

	s.awakeContactCount = 0;
	for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
		let color = world.constraintGraph.colors[i];
		s.colorCounts[i] = color.contactSims.length + color.jointSims.length;
		s.awakeContactCount += color.contactSims.length;
	}
	s.awakeContactCount += world.solverSets[b2_awakeSet].contactSims.length;
	return s;
}

export function b2World_GetMaxCapacity(worldId: b2WorldId): b2Capacity {
	let world = b2GetWorldFromId(worldId);
	return world.maxCapacity;
}

export function b2World_SetUserData(worldId: b2WorldId, userData: u64): void {
	let world = b2GetWorldFromId(worldId);
	world.userData = userData;
}

export function b2World_GetUserData(worldId: b2WorldId): u64 {
	let world = b2GetWorldFromId(worldId);
	return world.userData;
}

export function b2World_SetFrictionCallback(worldId: b2WorldId, callback: b2FrictionCallbackFcn?): void {
	let world = b2GetWorldFromId(worldId);
	if (world.locked) {
		return;
	}

	world.frictionCallback = callback;
}

export function b2World_SetRestitutionCallback(worldId: b2WorldId, callback: b2RestitutionCallbackFcn?): void {
	let world = b2GetWorldFromId(worldId);
	if (world.locked) {
		return;
	}

	world.restitutionCallback = callback;
}

// Tasks always run inline here, so the count only sizes the per worker storage, as C's does
export function b2World_SetWorkerCount(worldId: b2WorldId, count: i32): void {
	let world = b2GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	if (count == world.workerCount) {
		return;
	}

	world.workerCount = b2ClampInt(count, 1, B2_MAX_WORKERS);
	b2CreateWorkerContexts(world);
}

export function b2World_GetWorkerCount(worldId: b2WorldId): i32 {
	let world = b2GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return 0;
	}

	return world.workerCount;
}

function b2MakeQueryShapeId(world: b2World, shapeId: i32): b2ShapeId {
	let id = b2ShapeId();
	id.index1 = shapeId + 1;
	id.world0 = world.worldId;
	id.generation = world.shapes[shapeId].generation;
	return id;
}

// C's query contexts live on the stack, so a callback may start another query. Each query type keeps
// one context per nesting depth here, reused so a query allocates nothing once its depth was reached.

class WorldQueryContext {
	world: b2World = b2_worlds[0];
	fcn: b2OverlapResultCallbackFcn? = null;
	filter: b2QueryFilter = b2QueryFilter();

	// TreeQueryCallback
	visit(proxyId: i32, userData: u64): bool {
		let shapeId = <i32>userData;

		let world = this.world;

		if (b2ShouldQueryCollide(world.shapes[shapeId].filter, this.filter) == false) {
			return true;
		}

		let id = b2MakeQueryShapeId(world, shapeId);
		let result = this.fcn!(id);
		return result;
	}
}

const b2_worldQueryContexts = new Array<WorldQueryContext>();
let b2_worldQueryDepth = 0;

export function b2World_OverlapAABB(
	worldId: b2WorldId,
	origin: b2Pos,
	aabb: b2AABB,
	filter: b2QueryFilter,
	fcn: b2OverlapResultCallbackFcn,
): b2TreeStats {
	let treeStats = b2TreeStats();

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return treeStats;
	}

	B2_ASSERT(b2IsValidPos(origin));
	B2_ASSERT(b2IsValidAABB(aabb));

	// Lift to a world float box with outward rounding so the conservative tree test never misses
	let worldBox = b2OffsetAABB(aabb, origin);

	if (b2_worldQueryDepth == b2_worldQueryContexts.length) {
		b2_worldQueryContexts.push(new WorldQueryContext());
	}
	let worldContext = b2_worldQueryContexts[b2_worldQueryDepth];
	b2_worldQueryDepth += 1;
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		let treeResult = b2DynamicTree_OverlapAABB(world.broadPhase.trees[i], worldBox, filter.maskBits, worldContext);

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	worldContext.fcn = null;
	b2_worldQueryDepth -= 1;
	return treeStats;
}

class WorldOverlapContext {
	world: b2World = b2_worlds[0];
	fcn: b2OverlapResultCallbackFcn? = null;
	filter: b2QueryFilter = b2QueryFilter();
	proxy: b2ShapeProxy = b2ShapeProxy();
	origin: b2Pos = b2Vec2();

	// TreeOverlapCallback
	visit(proxyId: i32, userData: u64): bool {
		let shapeId = <i32>userData;

		let world = this.world;

		if (b2ShouldQueryCollide(world.shapes[shapeId].filter, this.filter) == false) {
			return true;
		}

		// Re-center on the query origin so the distance test stays in float precision far from the world origin
		let transform = b2ToRelativeTransform(b2GetBodyTransformQuick(world, world.shapes[shapeId].bodyId), this.origin);

		let input = b2DistanceInput();
		input.proxyA = this.proxy;
		input.proxyB = b2MakeShapeDistanceProxy(world, shapeId);
		input.transform = transform;
		input.useRadii = true;

		let cache = b2SimplexCache();
		let output = b2ComputeShapeDistance(input, cache, null, 0);

		let tolerance: f32 = 0.1 * B2_LINEAR_SLOP();
		if (output.distance > tolerance) {
			return true;
		}

		let id = b2MakeQueryShapeId(world, world.shapes[shapeId].id);
		let result = this.fcn!(id);
		return result;
	}
}

const b2_worldOverlapContexts = new Array<WorldOverlapContext>();
let b2_worldOverlapDepth = 0;

export function b2World_OverlapShape(
	worldId: b2WorldId,
	origin: b2Pos,
	proxy: b2ShapeProxy,
	filter: b2QueryFilter,
	fcn: b2OverlapResultCallbackFcn,
): b2TreeStats {
	let treeStats = b2TreeStats();

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return treeStats;
	}

	B2_ASSERT(b2IsValidPos(origin));

	// Relative box lifted to world float with outward rounding, conservative for the tree
	let aabb = b2OffsetAABB(b2MakeAABB(proxy.points, proxy.count, proxy.radius), origin);

	if (b2_worldOverlapDepth == b2_worldOverlapContexts.length) {
		b2_worldOverlapContexts.push(new WorldOverlapContext());
	}
	let worldContext = b2_worldOverlapContexts[b2_worldOverlapDepth];
	b2_worldOverlapDepth += 1;
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.proxy = proxy;
	worldContext.origin = origin;

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		let treeResult = b2DynamicTree_OverlapAABB(world.broadPhase.trees[i], aabb, filter.maskBits, worldContext);

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	worldContext.fcn = null;
	b2_worldOverlapDepth -= 1;
	return treeStats;
}

class WorldRayCastContext {
	world: b2World = b2_worlds[0];
	fcn: b2CastResultCallbackFcn? = null;
	filter: b2QueryFilter = b2QueryFilter();
	fraction: f32 = 0;
	origin: b2Pos = b2Vec2();

	// b2World_CastRayClosest uses b2RayCastClosestFcn, which C passes the result through the context
	closest: bool = false;
	result: b2RayResult = b2RayResult();

	// This callback finds the closest hit. This is the most common callback used in games.
	b2RayCastClosestFcn(shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32): f32 {
		// Ignore initial overlap
		if (fraction == 0.0) {
			return -1.0;
		}

		this.result.shapeId = shapeId;
		this.result.point = point;
		this.result.normal = normal;
		this.result.fraction = fraction;
		this.result.hit = true;
		return fraction;
	}

	// RayCastCallback
	visit(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
		let shapeId = <i32>userData;

		let world = this.world;

		if (b2ShouldQueryCollide(world.shapes[shapeId].filter, this.filter) == false) {
			return input.maxFraction;
		}

		let bodyTransform = b2GetBodyTransformQuick(world, world.shapes[shapeId].bodyId);
		let transform = b2ToRelativeTransform(bodyTransform, this.origin);

		let localInput = input;
		localInput.origin = b2Vec2_zero;
		let output = b2RayCastShape(localInput, world, shapeId, transform);

		if (output.hit) {
			let id = b2MakeQueryShapeId(world, shapeId);
			let point = b2OffsetPos(this.origin, output.point);
			let fraction: f32;
			if (this.closest) {
				fraction = this.b2RayCastClosestFcn(id, point, output.normal, output.fraction);
			} else {
				fraction = this.fcn!(id, point, output.normal, output.fraction);
			}

			// The user may return -1 to skip this shape
			if (0.0 <= fraction && fraction <= 1.0) {
				this.fraction = fraction;
			}

			return fraction;
		}

		return input.maxFraction;
	}
}

const b2_worldRayCastContexts = new Array<WorldRayCastContext>();
let b2_worldRayCastDepth = 0;

function b2AcquireRayCastContext(): WorldRayCastContext {
	if (b2_worldRayCastDepth == b2_worldRayCastContexts.length) {
		b2_worldRayCastContexts.push(new WorldRayCastContext());
	}
	let worldContext = b2_worldRayCastContexts[b2_worldRayCastDepth];
	b2_worldRayCastDepth += 1;
	return worldContext;
}

export function b2World_CastRay(
	worldId: b2WorldId,
	origin: b2Pos,
	translation: b2Vec2,
	filter: b2QueryFilter,
	fcn: b2CastResultCallbackFcn,
): b2TreeStats {
	let treeStats = b2TreeStats();

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return treeStats;
	}

	B2_ASSERT(b2IsValidPos(origin));
	B2_ASSERT(b2IsValidVec2(translation));

	// Tree traversal sees the origin truncated to float, displacing the ray by up to one
	// coordinate ULP, a graze sized miss tolerance at extreme range. Per-shape casts
	// re-difference against the full precision origin carried on the context.
	let input = b2RayCastInput(b2ToVec2(origin), translation, 1.0);

	let worldContext = b2AcquireRayCastContext();
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = 1.0;
	worldContext.origin = origin;
	worldContext.closest = false;

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		let treeResult = b2DynamicTree_CastRay(world.broadPhase.trees[i], input, filter.maskBits, worldContext);
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if (worldContext.fraction == 0.0) {
			break;
		}

		input.maxFraction = worldContext.fraction;
	}

	worldContext.fcn = null;
	b2_worldRayCastDepth -= 1;
	return treeStats;
}

export function b2World_CastRayClosest(worldId: b2WorldId, origin: b2Pos, translation: b2Vec2, filter: b2QueryFilter): b2RayResult {
	let result = b2RayResult();

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return result;
	}

	B2_ASSERT(b2IsValidPos(origin));
	B2_ASSERT(b2IsValidVec2(translation));

	let input = b2RayCastInput(b2ToVec2(origin), translation, 1.0);

	let worldContext = b2AcquireRayCastContext();
	worldContext.world = world;
	worldContext.fcn = null;
	worldContext.filter = filter;
	worldContext.fraction = 1.0;
	worldContext.origin = origin;
	worldContext.closest = true;
	worldContext.result = result;

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		let treeResult = b2DynamicTree_CastRay(world.broadPhase.trees[i], input, filter.maskBits, worldContext);
		worldContext.result.nodeVisits += treeResult.nodeVisits;
		worldContext.result.leafVisits += treeResult.leafVisits;

		if (worldContext.fraction == 0.0) {
			break;
		}

		input.maxFraction = worldContext.fraction;
	}

	result = worldContext.result;
	b2_worldRayCastDepth -= 1;
	return result;
}

class WorldShapeCastContext {
	world: b2World = b2_worlds[0];
	fcn: b2CastResultCallbackFcn? = null;
	filter: b2QueryFilter = b2QueryFilter();
	fraction: f32 = 0;
	origin: b2Pos = b2Vec2();

	// origin relative input
	input: b2ShapeCastInput = b2ShapeCastInput();

	// ShapeCastCallback
	visit(input: b2BoxCastInput, proxyId: i32, userData: u64): f32 {
		let shapeId = <i32>userData;

		let world = this.world;

		if (b2ShouldQueryCollide(world.shapes[shapeId].filter, this.filter) == false) {
			return input.maxFraction;
		}

		// Rebuild from the origin relative input, taking only the advancing fraction from the tree.
		// The tree input is world float and would lose the cast far from the origin.
		let localInput = this.input;
		localInput.maxFraction = input.maxFraction;

		let transform = b2GetBodyTransformQuick(world, world.shapes[shapeId].bodyId);
		let localTransform = b2ToRelativeTransform(transform, this.origin);

		let output = b2ShapeCastShape(localInput, world, shapeId, localTransform);

		if (output.hit) {
			let id = b2MakeQueryShapeId(world, shapeId);
			let point = b2OffsetPos(this.origin, output.point);
			let fraction = this.fcn!(id, point, output.normal, output.fraction);

			// The user may return -1 to skip this shape
			if (0.0 <= fraction && fraction <= 1.0) {
				this.fraction = fraction;
			}

			return fraction;
		}

		return input.maxFraction;
	}
}

const b2_worldShapeCastContexts = new Array<WorldShapeCastContext>();
let b2_worldShapeCastDepth = 0;

export function b2World_CastShape(
	worldId: b2WorldId,
	origin: b2Pos,
	proxy: b2ShapeProxy,
	translation: b2Vec2,
	filter: b2QueryFilter,
	fcn: b2CastResultCallbackFcn,
): b2TreeStats {
	let treeStats = b2TreeStats();

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return treeStats;
	}

	B2_ASSERT(b2IsValidPos(origin));
	B2_ASSERT(b2IsValidVec2(translation));

	if (b2_worldShapeCastDepth == b2_worldShapeCastContexts.length) {
		b2_worldShapeCastContexts.push(new WorldShapeCastContext());
	}
	let worldContext = b2_worldShapeCastContexts[b2_worldShapeCastDepth];
	b2_worldShapeCastDepth += 1;
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = 1.0;
	worldContext.origin = origin;
	worldContext.input = b2ShapeCastInput();
	worldContext.input.proxy = proxy;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0;

	// Create a conservative world space box from the proxy and origin.
	let localBox = b2MakeAABB(proxy.points, proxy.count, proxy.radius);
	let box = b2OffsetAABB(localBox, origin);
	let treeInput = b2BoxCastInput(box, translation, 1.0);

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		let treeResult = b2DynamicTree_CastBox(world.broadPhase.trees[i], treeInput, filter.maskBits, worldContext);
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if (worldContext.fraction == 0.0) {
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}

	worldContext.fcn = null;
	b2_worldShapeCastDepth -= 1;
	return treeStats;
}

// No user callback runs inside, so one context serves every call
class WorldMoverCastContext {
	world: b2World = b2_worlds[0];
	filter: b2QueryFilter = b2QueryFilter();
	fraction: f32 = 0;
	origin: b2Pos = b2Vec2();

	// origin relative input
	input: b2ShapeCastInput = b2ShapeCastInput();

	// MoverCastCallback
	visit(input: b2BoxCastInput, proxyId: i32, userData: u64): f32 {
		let shapeId = <i32>userData;

		let world = this.world;

		if (b2ShouldQueryCollide(world.shapes[shapeId].filter, this.filter) == false) {
			return this.fraction;
		}

		// Rebuild from the origin relative input, taking only the advancing fraction from the tree
		let localInput = this.input;
		localInput.maxFraction = input.maxFraction;

		let bodyTransform = b2GetBodyTransformQuick(world, world.shapes[shapeId].bodyId);
		let transform = b2ToRelativeTransform(bodyTransform, this.origin);

		let output = b2ShapeCastShape(localInput, world, shapeId, transform);
		if (output.fraction == 0.0) {
			// Ignore overlapping shapes
			return this.fraction;
		}

		this.fraction = output.fraction;
		return output.fraction;
	}
}

const b2_worldMoverCastContext = new WorldMoverCastContext();

export function b2World_CastMover(worldId: b2WorldId, origin: b2Pos, mover: b2Capsule, translation: b2Vec2, filter: b2QueryFilter): f32 {
	B2_ASSERT(b2IsValidPos(origin));
	B2_ASSERT(b2IsValidVec2(translation));
	B2_ASSERT(mover.radius > 2.0 * B2_LINEAR_SLOP());

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return 1.0;
	}

	let worldContext = b2_worldMoverCastContext;
	worldContext.world = world;
	worldContext.filter = filter;
	worldContext.fraction = 1.0;
	worldContext.origin = origin;
	worldContext.input = b2ShapeCastInput();
	worldContext.input.proxy.points[0] = mover.center1;
	worldContext.input.proxy.points[1] = mover.center2;
	worldContext.input.proxy.count = 2;
	worldContext.input.proxy.radius = mover.radius;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0;
	worldContext.input.canEncroach = true;

	// Bound the capsule in origin relative space then lift to a conservative world float box
	let centers: FixedArray<b2Vec2, 8>;
	centers[0] = mover.center1;
	centers[1] = mover.center2;
	let box = b2OffsetAABB(b2MakeAABB(centers, 2, mover.radius), origin);
	let treeInput = b2BoxCastInput(box, translation, 1.0);

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		b2DynamicTree_CastBox(world.broadPhase.trees[i], treeInput, filter.maskBits, worldContext);

		if (worldContext.fraction == 0.0) {
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}

	return worldContext.fraction;
}

class WorldMoverContext {
	world: b2World = b2_worlds[0];
	fcn: b2PlaneResultCallbackFcn? = null;
	filter: b2QueryFilter = b2QueryFilter();
	mover: b2Capsule = b2Capsule();
	origin: b2Pos = b2Vec2();

	// TreeCollideCallback
	visit(proxyId: i32, userData: u64): bool {
		let shapeId = <i32>userData;

		let world = this.world;

		if (b2ShouldQueryCollide(world.shapes[shapeId].filter, this.filter) == false) {
			return true;
		}

		// Re-center on the query origin, the mover and the resulting planes are origin relative
		let transform = b2ToRelativeTransform(b2GetBodyTransformQuick(world, world.shapes[shapeId].bodyId), this.origin);

		let result = b2CollideMover(this.mover, world, shapeId, transform);

		// todo handle deep overlap
		if (result.hit && b2IsNormalized(result.plane.normal)) {
			let id = b2MakeQueryShapeId(world, world.shapes[shapeId].id);
			return this.fcn!(id, result);
		}

		return true;
	}
}

const b2_worldMoverContexts = new Array<WorldMoverContext>();
let b2_worldMoverDepth = 0;

// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
// not be worth it.
export function b2World_CollideMover(
	worldId: b2WorldId,
	origin: b2Pos,
	mover: b2Capsule,
	filter: b2QueryFilter,
	fcn: b2PlaneResultCallbackFcn,
): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	B2_ASSERT(b2IsValidPos(origin));

	let r = b2Vec2(mover.radius, mover.radius);

	// Relative box lifted to world float with outward rounding, conservative for the tree
	let relBox = b2AABB();
	relBox.lowerBound = b2Sub(b2Min(mover.center1, mover.center2), r);
	relBox.upperBound = b2Add(b2Max(mover.center1, mover.center2), r);
	let aabb = b2OffsetAABB(relBox, origin);

	if (b2_worldMoverDepth == b2_worldMoverContexts.length) {
		b2_worldMoverContexts.push(new WorldMoverContext());
	}
	let worldContext = b2_worldMoverContexts[b2_worldMoverDepth];
	b2_worldMoverDepth += 1;
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.mover = mover;
	worldContext.origin = origin;

	for (let i = 0; i < b2_bodyTypeCount; ++i) {
		b2DynamicTree_OverlapAABB(world.broadPhase.trees[i], aabb, filter.maskBits, worldContext);
	}

	worldContext.fcn = null;
	b2_worldMoverDepth -= 1;
}

export function b2World_SetCustomFilterCallback(worldId: b2WorldId, fcn: b2CustomFilterCallbackFcn?): void {
	let world = b2GetWorldFromId(worldId);
	world.customFilterFcn = fcn;
}

export function b2World_SetPreSolveCallback(
	worldId: b2WorldId,
	preSolveFcn: b2PreSolveCallbackFcn?,
	preContinuousFcn: b2PreContinuousCallbackFcn?,
): void {
	let world = b2GetWorldFromId(worldId);
	world.preSolveFcn = preSolveFcn;
	world.preContinuousFcn = preContinuousFcn;
}

export function b2World_SetGravity(worldId: b2WorldId, gravity: b2Vec2): void {
	if (!b2IsValidVec2(gravity)) {
		b2Log("invalid gravity");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	world.gravity = gravity;
}

export function b2World_GetGravity(worldId: b2WorldId): b2Vec2 {
	let world = b2GetWorldFromId(worldId);
	return world.gravity;
}

// No user callback runs inside, so one context serves every call
class ExplosionContext {
	world: b2World = b2_worlds[0];
	position: b2Pos = b2Vec2();
	radius: f32 = 0;
	falloff: f32 = 0;
	impulsePerLength: f32 = 0;

	// ExplosionCallback
	visit(proxyId: i32, userData: u64): bool {
		let shapeId = <i32>userData;

		let world = this.world;
		let bodyId = world.shapes[shapeId].bodyId;
		B2_ASSERT(world.bodies[bodyId].type == b2_dynamicBody);

		let xf = b2GetBodyTransformQuick(world, bodyId);

		// Re-center the explosion into the shape local frame so distance and direction stay precise
		// far from the origin. Everything below runs in that near-origin frame.
		let localPosition = b2InvTransformWorldPoint(xf, this.position);

		let input = b2DistanceInput();
		input.proxyA = b2MakeShapeDistanceProxy(world, shapeId);
		input.proxyB = b2MakePointProxy(localPosition, 0.0);
		input.transform = b2Transform_identity;
		input.useRadii = true;

		let cache = b2SimplexCache();
		let output = b2ComputeShapeDistance(input, cache, null, 0);

		let radius = this.radius;
		let falloff = this.falloff;
		if (output.distance > radius + falloff) {
			return true;
		}

		b2WakeBody(world, bodyId);

		if (world.bodies[bodyId].setIndex != b2_awakeSet) {
			return true;
		}

		let closestPoint = output.pointA;
		if (output.distance == 0.0) {
			closestPoint = b2GetShapeCentroid(world, shapeId);
		}

		let direction = b2Sub(closestPoint, localPosition);
		if (b2LengthSquared(direction) > 100.0 * FLT_EPSILON * FLT_EPSILON) {
			direction = b2Normalize(direction);
		} else {
			direction = b2Vec2(1.0, 0.0);
		}

		let localLine = b2LeftPerp(direction);
		let perimeter = b2GetShapeProjectedPerimeter(world, shapeId, localLine);
		let scale: f32 = 1.0;
		if (output.distance > radius && falloff > 0.0) {
			scale = b2ClampFloat((radius + falloff - output.distance) / falloff, 0.0, 1.0);
		}

		let magnitude = this.impulsePerLength * perimeter * scale;
		let impulse = b2MulSV(magnitude, b2RotateVector(xf.q, direction));

		let localIndex = world.bodies[bodyId].localIndex;
		let set = world.solverSets[b2_awakeSet];
		let invMass = set.bodySims[localIndex].invMass;
		let invInertia = set.bodySims[localIndex].invInertia;
		let localCenter = set.bodySims[localIndex].localCenter;
		set.bodyStates[localIndex].linearVelocity = b2MulAdd(set.bodyStates[localIndex].linearVelocity, invMass, impulse);

		// Lever arm from the center of mass to the closest point, rotated to world
		let r = b2RotateVector(xf.q, b2Sub(closestPoint, localCenter));
		set.bodyStates[localIndex].angularVelocity += invInertia * b2Cross(r, impulse);

		return true;
	}
}

const b2_explosionContext = new ExplosionContext();

export function b2World_Explode(worldId: b2WorldId, explosionDef: b2ExplosionDef): void {
	let maskBits = explosionDef.maskBits;
	let position = explosionDef.position;
	let radius = explosionDef.radius;
	let falloff = explosionDef.falloff;
	let impulsePerLength = explosionDef.impulsePerLength;

	if (
		!b2IsValidPos(position) ||
		!(b2IsValidFloat(radius) && radius >= 0.0) ||
		!(b2IsValidFloat(falloff) && falloff >= 0.0) ||
		!b2IsValidFloat(impulsePerLength)
	) {
		b2Log("invalid explosion");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	let explosionContext = b2_explosionContext;
	explosionContext.world = world;
	explosionContext.position = position;
	explosionContext.radius = radius;
	explosionContext.falloff = falloff;
	explosionContext.impulsePerLength = impulsePerLength;

	// The broad-phase tree is float, so translate a local query box out to world with outward rounding
	let extent = radius + falloff;
	let localBox = b2AABB(b2Vec2(-extent, -extent), b2Vec2(extent, extent));
	let aabb = b2OffsetAABB(localBox, position);
	b2DynamicTree_OverlapAABB(world.broadPhase.trees[b2_dynamicBody], aabb, maskBits, explosionContext);
}

// This is for internal testing. Not optimized.
export function b2World_RebuildStaticTree(worldId: b2WorldId): void {
	let world = b2GetWorldFromId(worldId);
	B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	let staticTree = world.broadPhase.trees[b2_staticBody];

	// Moved proxies must retain the flag in the rebuild.
	let movedCount = 0;
	let proxyCount = b2DynamicTree_GetProxyCount(staticTree);
	let movedProxies = new StaticArray<i32>(0);
	if (b2HasTreeMoved(staticTree)) {
		movedProxies = new StaticArray<i32>(proxyCount);
		movedCount = b2DynamicTree_GatherMovedProxies(staticTree, movedProxies);
	}

	b2DynamicTree_Rebuild(staticTree, true);

	for (let i = 0; i < movedCount; ++i) {
		b2DynamicTree_MarkProxyMovedSerial(staticTree, movedProxies[i]);
	}
}
