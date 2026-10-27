// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/physics_world.c

import { b2BitSet } from "./bitset";
import { b2IdPool } from "./id_pool";
import { b2AABB, b2Vec2 } from "./math_functions";
import {
	b2BodyMoveEvent,
	b2Capacity,
	b2ContactBeginTouchEvent,
	b2ContactEndTouchEvent,
	b2ContactHitEvent,
	b2CustomFilterCallbackFcn,
	b2FrictionCallbackFcn,
	b2JointEvent,
	b2PreContinuousCallbackFcn,
	b2PreSolveCallbackFcn,
	b2Profile,
	b2RestitutionCallbackFcn,
	b2SensorBeginTouchEvent,
	b2SensorEndTouchEvent,
} from "./types";
import { b2BroadPhase } from "./broad_phase";
import { b2ConstraintGraph } from "./constraint_graph";
import { b2Body } from "./body";
import { b2SolverSet } from "./solver_set";
import { b2Joint } from "./joint";
import { b2Contact } from "./contact";
import { b2Island } from "./island";
import { b2ChainShape, b2Shape } from "./shape";
import { b2Sensor, b2SensorHit, b2SensorTaskContext } from "./sensor";

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

	constructor() {
		for (let i = 0; i < 2; ++i) {
			this.sensorEndEvents[i] = new Array<b2SensorEndTouchEvent>();
			this.contactEndEvents[i] = new Array<b2ContactEndTouchEvent>();
		}
	}
}

// functions from physics_world.c follow
