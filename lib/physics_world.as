// Ported from box2d src/physics_world.h/.c. Single threaded: the default
// task callback runs tasks inline, worker count is always 1. Debug draw,
// the mover queries, and memory stats are omitted. Validation matches the
// NDEBUG reference build (no-ops).

import { B2_NULL_INDEX, B2_SECRET_COOKIE, b2CTZ64 } from "./core";
import { B2_GRAPH_COLOR_COUNT, B2_MAX_WORLDS, B2_LINEAR_SLOP } from "./constants";
import {
    b2Vec2, b2Transform, b2AABB, b2Sub, b2Add, b2Min, b2Max, b2MulSV, b2MulAdd, b2Cross,
    b2LengthSquared, b2Normalize, b2InvRotateVector, b2LeftPerp, b2RotateVector, b2ClampFloat,
    b2MinFloat, b2MaxFloat, b2MinInt, b2MaxInt, b2IsValidFloat, b2IsValidVec2, b2IsValidAABB,
    b2Transform_identity, b2AABB_Overlaps, FLT_MAX, FLT_EPSILON,
} from "./math_functions";
import { b2ShapeProxy, b2SimplexCache, b2Manifold, b2RayCastInput, b2ShapeCastInput, b2TreeStats,
    b2Capsule, b2DistanceInput, makeVecArray } from "./collision";
import { b2WorldId, b2BodyId, b2ShapeId, b2ChainId, b2JointId, b2ContactId } from "./id";
import { b2WorldDef, b2ExplosionDef, b2QueryFilter } from "./types";
import { b2ArenaAllocator, b2CreateArenaAllocator, b2GrowArena, b2GetArenaAllocation } from "./arena_allocator";
import {
    b2BroadPhase, b2CreateBroadPhase, b2DestroyBroadPhase, b2PairFilter,
    b2UpdateBroadPhasePairs as b2BroadPhaseUpdatePairs, b2BroadPhase_RebuildTrees,
    b2_staticBody, b2_kinematicBody, b2_dynamicBody, b2_bodyTypeCount,
} from "./broad_phase";
import { b2IdPool, b2CreateIdPool, b2AllocId, b2GetIdCapacity, b2GetIdCount } from "./id_pool";
import { b2Array, b2IntArray, b2IntArray_Create, b2IntArray_Clear } from "./array";
import {
    b2BitSet, b2CreateBitSet, b2SetBitCountAndClear, b2SetBit, b2GetBit, b2InPlaceUnion,
} from "./bitset";
import { b2ConstraintGraph, b2CreateGraph, b2GraphColor } from "./constraint_graph";
import { b2Body, b2BodySim, b2BodyState, b2GetBodySim, b2GetBodyTransformQuick, b2WakeBody } from "./body";
import { b2Shape, b2ChainShape, b2ShouldShapesCollide, b2ShouldQueryCollide, b2MakeShapeDistanceProxy,
    b2RayCastShape, b2ShapeCastShape, b2GetShapeCentroid, b2GetShapeProjectedPerimeter } from "./shape";
import {
    b2Contact, b2ContactSim, b2InitializeContactRegisters, b2CreateContact, b2DestroyContact,
    b2UpdateContact, b2_simTouchingFlag, b2_simDisjoint, b2_simStartedTouching, b2_simStoppedTouching,
    b2_contactTouchingFlag, b2_contactEnableContactEvents,
} from "./contact";
import { b2Joint, b2JointSim } from "./joint";
import { b2Island, b2IslandSim, b2LinkContact, b2UnlinkContact } from "./island";
import { b2SolverSet, b2WakeSolverSet } from "./solver_set";
import { b2AddContactToGraph, b2RemoveContactFromGraph } from "./constraint_graph";
import { b2Sensor, b2SensorHit, b2OverlapSensors } from "./sensor";
import { b2StepContext, b2MakeSoft, b2Solve } from "./solver";
import { b2ShouldBodiesCollide } from "./body";
import { b2ShapeDistance } from "./distance";
import { b2MakeProxy } from "./distance";
import {
    b2DynamicTree_Query, b2DynamicTree_RayCast, b2DynamicTree_ShapeCast, b2DynamicTree_Rebuild,
    b2DynamicTree_GetHeight, b2TreeQueryCallback, b2TreeRayCastCallback, b2TreeShapeCastCallback,
} from "./dynamic_tree";

// The solver set type by index (C solver_set.h)
export const b2_staticSet: i32 = 0;
export const b2_disabledSet: i32 = 1;
export const b2_awakeSet: i32 = 2;
export const b2_firstSleepingSet: i32 = 3;

// Events

export class b2SensorBeginTouchEvent {
    sensorShapeId: b2ShapeId = new b2ShapeId(0, 0, 0);
    visitorShapeId: b2ShapeId = new b2ShapeId(0, 0, 0);
}

export class b2SensorEndTouchEvent {
    sensorShapeId: b2ShapeId = new b2ShapeId(0, 0, 0);
    visitorShapeId: b2ShapeId = new b2ShapeId(0, 0, 0);
}

export class b2ContactBeginTouchEvent {
    shapeIdA: b2ShapeId = new b2ShapeId(0, 0, 0);
    shapeIdB: b2ShapeId = new b2ShapeId(0, 0, 0);
    contactId: b2ContactId = new b2ContactId(0, 0, 0);
}

export class b2ContactEndTouchEvent {
    shapeIdA: b2ShapeId = new b2ShapeId(0, 0, 0);
    shapeIdB: b2ShapeId = new b2ShapeId(0, 0, 0);
    contactId: b2ContactId = new b2ContactId(0, 0, 0);
}

export class b2ContactHitEvent {
    shapeIdA: b2ShapeId = new b2ShapeId(0, 0, 0);
    shapeIdB: b2ShapeId = new b2ShapeId(0, 0, 0);
    contactId: b2ContactId = new b2ContactId(0, 0, 0);
    point: b2Vec2 = new b2Vec2();
    normal: b2Vec2 = new b2Vec2();
    approachSpeed: f32 = 0;
}

export class b2BodyMoveEvent {
    userData: u64 = 0;
    transform: b2Transform = new b2Transform();
    bodyId: b2BodyId = new b2BodyId(0, 0, 0);
    fellAsleep: bool = false;
}

export class b2JointEvent {
    jointId: b2JointId = new b2JointId(0, 0, 0);
    userData: u64 = 0;
}

export class b2ContactData {
    contactId: b2ContactId = new b2ContactId(0, 0, 0);
    shapeIdA: b2ShapeId = new b2ShapeId(0, 0, 0);
    shapeIdB: b2ShapeId = new b2ShapeId(0, 0, 0);
    manifold: b2Manifold = new b2Manifold();
}

export class b2BodyEvents {
    moveEvents: Array<b2BodyMoveEvent | null> = new Array<b2BodyMoveEvent | null>();
    moveCount: i32 = 0;
}

export class b2SensorEvents {
    beginEvents: Array<b2SensorBeginTouchEvent | null> = new Array<b2SensorBeginTouchEvent | null>();
    endEvents: Array<b2SensorEndTouchEvent | null> = new Array<b2SensorEndTouchEvent | null>();
    beginCount: i32 = 0;
    endCount: i32 = 0;
}

export class b2ContactEvents {
    beginEvents: Array<b2ContactBeginTouchEvent | null> = new Array<b2ContactBeginTouchEvent | null>();
    endEvents: Array<b2ContactEndTouchEvent | null> = new Array<b2ContactEndTouchEvent | null>();
    hitEvents: Array<b2ContactHitEvent | null> = new Array<b2ContactHitEvent | null>();
    beginCount: i32 = 0;
    endCount: i32 = 0;
    hitCount: i32 = 0;
}

export class b2JointEvents {
    jointEvents: Array<b2JointEvent | null> = new Array<b2JointEvent | null>();
    count: i32 = 0;
}

export class b2RayResult {
    shapeId: b2ShapeId = new b2ShapeId(0, 0, 0);
    point: b2Vec2 = new b2Vec2();
    normal: b2Vec2 = new b2Vec2();
    fraction: f32 = 0;
    nodeVisits: i32 = 0;
    leafVisits: i32 = 0;
    hit: bool = false;
}

// Callback base classes (C function pointers); override to customize.

export class b2FrictionCallback {
    mix(frictionA: f32, materialA: u64, frictionB: f32, materialB: u64): f32 {
        return sqrt<f32>(frictionA * frictionB);
    }
}

export class b2RestitutionCallback {
    mix(restitutionA: f32, materialA: u64, restitutionB: f32, materialB: u64): f32 {
        return b2MaxFloat(restitutionA, restitutionB);
    }
}

export class b2CustomFilterFcn {
    shouldCollide(shapeIdA: b2ShapeId, shapeIdB: b2ShapeId): bool {
        return true;
    }
}

export class b2PreSolveFcn {
    preSolve(shapeIdA: b2ShapeId, shapeIdB: b2ShapeId, point: b2Vec2, normal: b2Vec2): bool {
        return true;
    }
}

export class b2OverlapResultFcn {
    report(shapeId: b2ShapeId): bool {
        return true;
    }
}

export class b2CastResultFcn {
    report(shapeId: b2ShapeId, point: b2Vec2, normal: b2Vec2, fraction: f32): f32 {
        return fraction;
    }
}

// Per thread task storage
export class b2TaskContext {
    sensorHits: b2Array<b2SensorHit> = new b2Array<b2SensorHit>();
    contactStateBitSet: b2BitSet = new b2BitSet();
    jointStateBitSet: b2BitSet = new b2BitSet();
    enlargedSimBitSet: b2BitSet = new b2BitSet();
    awakeIslandBitSet: b2BitSet = new b2BitSet();
    splitSleepTime: f32 = 0;
    splitIslandId: i32 = 0;
}

export class b2Profile {
    step: f32 = 0;
    pairs: f32 = 0;
    collide: f32 = 0;
    solve: f32 = 0;
}

export class b2Counters {
    bodyCount: i32 = 0;
    shapeCount: i32 = 0;
    contactCount: i32 = 0;
    jointCount: i32 = 0;
    islandCount: i32 = 0;
    stackUsed: i32 = 0;
    staticTreeHeight: i32 = 0;
    treeHeight: i32 = 0;
    byteCount: i32 = 0;
    taskCount: i32 = 0;
    colorCounts: StaticArray<i32> = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
}

export class b2World {
    arena: b2ArenaAllocator = new b2ArenaAllocator();
    broadPhase: b2BroadPhase = new b2BroadPhase();
    constraintGraph: b2ConstraintGraph = new b2ConstraintGraph();

    bodyIdPool: b2IdPool = new b2IdPool();
    bodies: b2Array<b2Body> = new b2Array<b2Body>();

    solverSetIdPool: b2IdPool = new b2IdPool();
    solverSets: b2Array<b2SolverSet> = new b2Array<b2SolverSet>();

    jointIdPool: b2IdPool = new b2IdPool();
    joints: b2Array<b2Joint> = new b2Array<b2Joint>();

    contactIdPool: b2IdPool = new b2IdPool();
    contacts: b2Array<b2Contact> = new b2Array<b2Contact>();

    islandIdPool: b2IdPool = new b2IdPool();
    islands: b2Array<b2Island> = new b2Array<b2Island>();

    shapeIdPool: b2IdPool = new b2IdPool();
    chainIdPool: b2IdPool = new b2IdPool();

    shapes: b2Array<b2Shape> = new b2Array<b2Shape>();
    chainShapes: b2Array<b2ChainShape> = new b2Array<b2ChainShape>();

    sensors: b2Array<b2Sensor> = new b2Array<b2Sensor>();

    taskContexts: b2Array<b2TaskContext> = new b2Array<b2TaskContext>();

    bodyMoveEvents: b2Array<b2BodyMoveEvent> = new b2Array<b2BodyMoveEvent>();
    sensorBeginEvents: b2Array<b2SensorBeginTouchEvent> = new b2Array<b2SensorBeginTouchEvent>();
    contactBeginEvents: b2Array<b2ContactBeginTouchEvent> = new b2Array<b2ContactBeginTouchEvent>();

    // End events are double buffered so that the user doesn't need to flush events
    sensorEndEvents: StaticArray<b2Array<b2SensorEndTouchEvent>> = new StaticArray<b2Array<b2SensorEndTouchEvent>>(2);
    contactEndEvents: StaticArray<b2Array<b2ContactEndTouchEvent>> = new StaticArray<b2Array<b2ContactEndTouchEvent>>(2);
    endEventArrayIndex: i32 = 0;

    contactHitEvents: b2Array<b2ContactHitEvent> = new b2Array<b2ContactHitEvent>();
    jointEvents: b2Array<b2JointEvent> = new b2Array<b2JointEvent>();

    stepIndex: u64 = 0;

    splitIslandId: i32 = 0;

    gravity: b2Vec2 = new b2Vec2();
    hitEventThreshold: f32 = 0;
    restitutionThreshold: f32 = 0;
    maxLinearSpeed: f32 = 0;
    contactSpeed: f32 = 0;
    contactHertz: f32 = 0;
    contactDampingRatio: f32 = 0;

    frictionCallback: b2FrictionCallback = new b2FrictionCallback();
    restitutionCallback: b2RestitutionCallback = new b2RestitutionCallback();

    generation: u16 = 0;

    profile: b2Profile = new b2Profile();

    preSolveFcn: b2PreSolveFcn | null = null;
    customFilterFcn: b2CustomFilterFcn | null = null;

    workerCount: i32 = 0;

    userData: u64 = 0;

    inv_h: f32 = 0;
    inv_dt: f32 = 0;

    activeTaskCount: i32 = 0;
    taskCount: i32 = 0;

    worldId: u16 = 0;

    enableSleep: bool = false;
    locked: bool = false;
    enableWarmStarting: bool = false;
    enableContactSoftening: bool = false;
    enableContinuous: bool = false;
    enableSpeculative: bool = false;
    inUse: bool = false;

    constructor() {
        this.sensorEndEvents[0] = new b2Array<b2SensorEndTouchEvent>();
        this.sensorEndEvents[1] = new b2Array<b2SensorEndTouchEvent>();
        this.contactEndEvents[0] = new b2Array<b2ContactEndTouchEvent>();
        this.contactEndEvents[1] = new b2Array<b2ContactEndTouchEvent>();
    }
}

function makeWorlds(): StaticArray<b2World> {
    let worlds = new StaticArray<b2World>(B2_MAX_WORLDS);
    for (let i = 0; i < B2_MAX_WORLDS; ++i) {
        worlds[i] = new b2World();
    }
    return worlds;
}

let b2_worlds: StaticArray<b2World> = makeWorlds();

export function b2GetWorldFromId(id: b2WorldId): b2World {
    assert(1 <= id.index1 && <i32>id.index1 <= B2_MAX_WORLDS);
    let world = b2_worlds[id.index1 - 1];
    assert(<i32>id.index1 == <i32>world.worldId + 1);
    assert(id.generation == world.generation);
    return world;
}

export function b2GetWorld(index: i32): b2World {
    assert(0 <= index && index < B2_MAX_WORLDS);
    let world = b2_worlds[index];
    assert(<i32>world.worldId == index);
    return world;
}

export function b2GetWorldLocked(index: i32): b2World | null {
    assert(0 <= index && index < B2_MAX_WORLDS);
    let world = b2_worlds[index];
    assert(<i32>world.worldId == index);
    if (world.locked) {
        assert(false);
        return null;
    }

    return world;
}

// Like C's direct b2_worlds access in the IsValid functions.
export function b2GetWorldMaybe(index: i32): b2World | null {
    if (index < 0 || index >= B2_MAX_WORLDS) {
        return null;
    }
    let world = b2_worlds[index];
    if (<i32>world.worldId != index) {
        return null;
    }
    return world;
}

// Bench-only: the stub-runtime bake rewinds the heap between runs, so the
// slot must return to an object allocated before the rewind mark.
export function b2BenchGetWorldSlot(index: i32): b2World {
    return b2_worlds[index];
}

export function b2BenchSetWorldSlot(index: i32, world: b2World): void {
    b2_worlds[index] = world;
}

// NDEBUG reference build: validation is compiled out.
export function b2ValidateConnectivity(world: b2World): void {
}

export function b2ValidateSolverSets(world: b2World): void {
}

export function b2ValidateContacts(world: b2World): void {
}

export function b2ValidateIsland(world: b2World, islandId: i32): void {
}

export function b2CreateWorld(def: b2WorldDef): b2WorldId {
    assert(def.internalValue == B2_SECRET_COOKIE);

    let worldId = B2_NULL_INDEX;
    for (let i = 0; i < B2_MAX_WORLDS; ++i) {
        if (b2_worlds[i].inUse == false) {
            worldId = i;
            break;
        }
    }

    if (worldId == B2_NULL_INDEX) {
        return new b2WorldId(0, 0);
    }

    b2InitializeContactRegisters();

    let generation = b2_worlds[worldId].generation;

    // Wipe the slot like C's zero-assignment, preserving generation.
    let world = new b2World();
    b2_worlds[worldId] = world;

    world.worldId = <u16>worldId;
    world.generation = generation;
    world.inUse = true;

    world.arena = b2CreateArenaAllocator(2048);
    b2CreateBroadPhase(world.broadPhase);
    b2CreateGraph(world.constraintGraph, 16);

    // pools
    world.bodyIdPool = b2CreateIdPool();
    world.bodies = new b2Array<b2Body>(16);
    world.solverSets = new b2Array<b2SolverSet>(8);

    // add empty static, active, and disabled body sets
    world.solverSetIdPool = b2CreateIdPool();

    // static set
    {
        let setId = b2AllocId(world.solverSetIdPool);
        let set = world.solverSets.add();
        set.setIndex = setId;
        assert(world.solverSets.get(b2_staticSet).setIndex == b2_staticSet);
    }

    // disabled set
    {
        let setId = b2AllocId(world.solverSetIdPool);
        let set = world.solverSets.add();
        set.setIndex = setId;
        assert(world.solverSets.get(b2_disabledSet).setIndex == b2_disabledSet);
    }

    // awake set
    {
        let setId = b2AllocId(world.solverSetIdPool);
        let set = world.solverSets.add();
        set.setIndex = setId;
        assert(world.solverSets.get(b2_awakeSet).setIndex == b2_awakeSet);
    }

    world.shapeIdPool = b2CreateIdPool();
    world.shapes = new b2Array<b2Shape>(16);

    world.chainIdPool = b2CreateIdPool();
    world.chainShapes = new b2Array<b2ChainShape>(4);

    world.contactIdPool = b2CreateIdPool();
    world.contacts = new b2Array<b2Contact>(16);

    world.jointIdPool = b2CreateIdPool();
    world.joints = new b2Array<b2Joint>(16);

    world.islandIdPool = b2CreateIdPool();
    world.islands = new b2Array<b2Island>(8);

    world.sensors = new b2Array<b2Sensor>(4);

    world.bodyMoveEvents = new b2Array<b2BodyMoveEvent>(4);
    world.sensorBeginEvents = new b2Array<b2SensorBeginTouchEvent>(4);
    world.sensorEndEvents[0] = new b2Array<b2SensorEndTouchEvent>(4);
    world.sensorEndEvents[1] = new b2Array<b2SensorEndTouchEvent>(4);
    world.contactBeginEvents = new b2Array<b2ContactBeginTouchEvent>(4);
    world.contactEndEvents[0] = new b2Array<b2ContactEndTouchEvent>(4);
    world.contactEndEvents[1] = new b2Array<b2ContactEndTouchEvent>(4);
    world.contactHitEvents = new b2Array<b2ContactHitEvent>(4);
    world.jointEvents = new b2Array<b2JointEvent>(4);
    world.endEventArrayIndex = 0;

    world.stepIndex = 0;
    world.splitIslandId = B2_NULL_INDEX;
    world.activeTaskCount = 0;
    world.taskCount = 0;
    world.gravity = def.gravity.clone();
    world.hitEventThreshold = def.hitEventThreshold;
    world.restitutionThreshold = def.restitutionThreshold;
    world.maxLinearSpeed = def.maximumLinearSpeed;
    world.contactSpeed = def.contactSpeed;
    world.contactHertz = def.contactHertz;
    world.contactDampingRatio = def.contactDampingRatio;

    world.frictionCallback = new b2FrictionCallback();
    world.restitutionCallback = new b2RestitutionCallback();

    world.enableSleep = def.enableSleep;
    world.locked = false;
    world.enableWarmStarting = true;
    world.enableContactSoftening = def.enableContactSoftening;
    world.enableContinuous = def.enableContinuous;
    world.enableSpeculative = true;
    world.userData = def.userData;

    // single threaded port
    world.workerCount = 1;

    world.taskContexts = new b2Array<b2TaskContext>(world.workerCount);
    world.taskContexts.resize(world.workerCount);

    for (let i = 0; i < world.workerCount; ++i) {
        let taskContext = new b2TaskContext();
        taskContext.sensorHits = new b2Array<b2SensorHit>(8);
        taskContext.contactStateBitSet = b2CreateBitSet(1024);
        taskContext.jointStateBitSet = b2CreateBitSet(1024);
        taskContext.enlargedSimBitSet = b2CreateBitSet(256);
        taskContext.awakeIslandBitSet = b2CreateBitSet(256);
        world.taskContexts.set(i, taskContext);
    }

    // add one to worldId so that 0 represents a null b2WorldId
    return new b2WorldId(<u16>(worldId + 1), world.generation);
}

export function b2DestroyWorld(worldId: b2WorldId): void {
    let world = b2GetWorldFromId(worldId);

    // Wipe world but preserve generation
    let generation = world.generation;
    let fresh = new b2World();
    b2_worlds[<i32>world.worldId] = fresh;
    fresh.worldId = 0;
    fresh.generation = generation + 1;
}

// World-dependent pair filtering for the Phase B broad phase hook. This is
// the tail of C's b2PairQueryCallback.
class b2WorldPairFilter extends b2PairFilter {
    world: b2World;

    constructor(world: b2World) {
        super();
        this.world = world;
    }

    shouldCollide(shapeIdA: i32, shapeIdB: i32): bool {
        let world = this.world;

        let shapeA = world.shapes.get(shapeIdA);
        let shapeB = world.shapes.get(shapeIdB);

        let bodyIdA = shapeA.bodyId;
        let bodyIdB = shapeB.bodyId;

        // Are the shapes on the same body?
        if (bodyIdA == bodyIdB) {
            return false;
        }

        // Sensors are handled elsewhere
        if (shapeA.sensorIndex != B2_NULL_INDEX || shapeB.sensorIndex != B2_NULL_INDEX) {
            return false;
        }

        if (b2ShouldShapesCollide(shapeA.filter, shapeB.filter) == false) {
            return false;
        }

        // Does a joint override collision?
        let bodyA = world.bodies.get(bodyIdA);
        let bodyB = world.bodies.get(bodyIdB);
        if (b2ShouldBodiesCollide(world, bodyA, bodyB) == false) {
            return false;
        }

        // Custom user filter
        if (shapeA.enableCustomFiltering || shapeB.enableCustomFiltering) {
            let customFilterFcn = world.customFilterFcn;
            if (customFilterFcn != null) {
                let idA = new b2ShapeId(shapeIdA + 1, world.worldId, shapeA.generation);
                let idB = new b2ShapeId(shapeIdB + 1, world.worldId, shapeB.generation);
                let shouldCollide = customFilterFcn.shouldCollide(idA, idB);
                if (shouldCollide == false) {
                    return false;
                }
            }
        }

        return true;
    }
}

let s_pairBuffer = new b2IntArray();

// C's b2UpdateBroadPhasePairs: pair sweep then contact creation in order.
function b2UpdateBroadPhasePairs(world: b2World): void {
    let filter = new b2WorldPairFilter(world);
    b2IntArray_Clear(s_pairBuffer);
    b2BroadPhaseUpdatePairs(world.broadPhase, filter, s_pairBuffer);

    let pairCount = s_pairBuffer.count / 2;
    for (let i = 0; i < pairCount; ++i) {
        let shapeIdA = s_pairBuffer.data[2 * i];
        let shapeIdB = s_pairBuffer.data[2 * i + 1];
        let shapeA = world.shapes.get(shapeIdA);
        let shapeB = world.shapes.get(shapeIdB);
        b2CreateContact(world, shapeA, shapeB);
    }
}

function b2CollideTask(startIndex: i32, endIndex: i32, threadIndex: u32, stepContext: b2StepContext): void {
    let world = stepContext.world!;
    assert(<i32>threadIndex < world.workerCount);
    let taskContext = world.taskContexts.get(threadIndex);
    let contactSims = stepContext.contacts;

    assert(startIndex < endIndex);

    for (let contactIndex = startIndex; contactIndex < endIndex; ++contactIndex) {
        let contactSim = contactSims[contactIndex]!;

        let contactId = contactSim.contactId;

        let shapeA = world.shapes.get(contactSim.shapeIdA);
        let shapeB = world.shapes.get(contactSim.shapeIdB);

        // Do proxies still overlap?
        let overlap = b2AABB_Overlaps(shapeA.fatAABB, shapeB.fatAABB);
        if (overlap == false) {
            contactSim.simFlags |= b2_simDisjoint;
            contactSim.simFlags &= ~b2_simTouchingFlag;
            b2SetBit(taskContext.contactStateBitSet, contactId);
        } else {
            let wasTouching = (contactSim.simFlags & b2_simTouchingFlag) != 0;

            // Update contact respecting shape/body order (A,B)
            let bodyA = world.bodies.get(shapeA.bodyId);
            let bodyB = world.bodies.get(shapeB.bodyId);
            let bodySimA = b2GetBodySim(world, bodyA);
            let bodySimB = b2GetBodySim(world, bodyB);

            // avoid cache misses in b2PrepareContactsTask
            contactSim.bodySimIndexA = bodyA.setIndex == b2_awakeSet ? bodyA.localIndex : B2_NULL_INDEX;
            contactSim.invMassA = bodySimA.invMass;
            contactSim.invIA = bodySimA.invInertia;

            contactSim.bodySimIndexB = bodyB.setIndex == b2_awakeSet ? bodyB.localIndex : B2_NULL_INDEX;
            contactSim.invMassB = bodySimB.invMass;
            contactSim.invIB = bodySimB.invInertia;

            let transformA = bodySimA.transform;
            let transformB = bodySimB.transform;

            let centerOffsetA = b2RotateVector(transformA.q, bodySimA.localCenter);
            let centerOffsetB = b2RotateVector(transformB.q, bodySimB.localCenter);

            // This updates solid contacts
            let touching =
                b2UpdateContact(world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB);

            // State changes that affect island connectivity. Also affects contact events.
            if (touching == true && wasTouching == false) {
                contactSim.simFlags |= b2_simStartedTouching;
                b2SetBit(taskContext.contactStateBitSet, contactId);
            } else if (touching == false && wasTouching == true) {
                contactSim.simFlags |= b2_simStoppedTouching;
                b2SetBit(taskContext.contactStateBitSet, contactId);
            }
        }
    }
}

function b2AddNonTouchingContact(world: b2World, contact: b2Contact, contactSim: b2ContactSim): void {
    assert(contact.setIndex == b2_awakeSet);
    let set = world.solverSets.get(b2_awakeSet);
    contact.colorIndex = B2_NULL_INDEX;
    contact.localIndex = set.contactSims.count;

    // move the reference; caller removes the graph slot next
    set.contactSims.push(contactSim);
}

function b2RemoveNonTouchingContact(world: b2World, setIndex: i32, localIndex: i32): void {
    let set = world.solverSets.get(setIndex);
    let movedIndex = set.contactSims.removeSwap(localIndex);
    if (movedIndex != B2_NULL_INDEX) {
        let movedContactSim = set.contactSims.get(localIndex);
        let movedContact = world.contacts.get(movedContactSim.contactId);
        assert(movedContact.setIndex == setIndex);
        assert(movedContact.localIndex == movedIndex);
        assert(movedContact.colorIndex == B2_NULL_INDEX);
        movedContact.localIndex = localIndex;
    }
}

// Narrow-phase collision
function b2Collide(context: b2StepContext): void {
    let world = context.world!;

    assert(world.workerCount > 0);

    // In C the tree rebuild task runs in parallel with the narrow-phase;
    // single threaded it runs inline here.
    b2BroadPhase_RebuildTrees(world.broadPhase);
    world.taskCount += 1;

    // gather contacts into a single array for easier parallel-for
    let contactCount = 0;
    let graphColors = world.constraintGraph.colors;
    for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
        contactCount += graphColors[i].contactSims.count;
    }

    let nonTouchingCount = world.solverSets.get(b2_awakeSet).contactSims.count;
    contactCount += nonTouchingCount;

    if (contactCount == 0) {
        return;
    }

    let contactSims = new Array<b2ContactSim | null>(contactCount);

    let contactIndex = 0;
    for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
        let color = graphColors[i];
        let count = color.contactSims.count;
        for (let j = 0; j < count; ++j) {
            contactSims[contactIndex] = color.contactSims.get(j);
            contactIndex += 1;
        }
    }

    {
        let awake = world.solverSets.get(b2_awakeSet);
        for (let i = 0; i < nonTouchingCount; ++i) {
            contactSims[contactIndex] = awake.contactSims.get(i);
            contactIndex += 1;
        }
    }

    assert(contactIndex == contactCount);

    context.contacts = contactSims;

    // Contact bit set on ids because contact references are unstable as they move between touching and not touching.
    let contactIdCapacity = b2GetIdCapacity(world.contactIdPool);
    for (let i = 0; i < world.workerCount; ++i) {
        b2SetBitCountAndClear(world.taskContexts.get(i).contactStateBitSet, contactIdCapacity);
    }

    // Task runs inline (single threaded)
    b2CollideTask(0, contactCount, 0, context);
    world.taskCount += 1;

    context.contacts = new Array<b2ContactSim | null>();

    // Serially update contact state

    // Bitwise OR all contact bits
    let bitSet = world.taskContexts.get(0).contactStateBitSet;
    for (let i = 1; i < world.workerCount; ++i) {
        b2InPlaceUnion(bitSet, world.taskContexts.get(i).contactStateBitSet);
    }

    let awakeSet = world.solverSets.get(b2_awakeSet);

    let endEventArrayIndex = world.endEventArrayIndex;

    let worldId = world.worldId;

    // Process contact state changes. Iterate over set bits
    for (let k: u32 = 0; k < bitSet.blockCount; ++k) {
        let bits = bitSet.bits[k];
        while (bits != 0) {
            let ctz = b2CTZ64(bits);
            let contactId = <i32>(64 * k + ctz);

            let contact = world.contacts.get(contactId);
            assert(contact.setIndex == b2_awakeSet);

            let colorIndex = contact.colorIndex;
            let localIndex = contact.localIndex;

            let contactSim: b2ContactSim;
            if (colorIndex != B2_NULL_INDEX) {
                // contact lives in constraint graph
                assert(0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT);
                let color = graphColors[colorIndex];
                contactSim = color.contactSims.get(localIndex);
            } else {
                contactSim = awakeSet.contactSims.get(localIndex);
            }

            let shapeA = world.shapes.get(contact.shapeIdA);
            let shapeB = world.shapes.get(contact.shapeIdB);
            let shapeIdA = new b2ShapeId(shapeA.id + 1, worldId, shapeA.generation);
            let shapeIdB = new b2ShapeId(shapeB.id + 1, worldId, shapeB.generation);
            let contactFullId = new b2ContactId(contactId + 1, worldId, contact.generation);
            let flags = contact.flags;
            let simFlags = contactSim.simFlags;

            if ((simFlags & b2_simDisjoint) != 0) {
                // Bounding boxes no longer overlap
                b2DestroyContact(world, contact, false);
            } else if ((simFlags & b2_simStartedTouching) != 0) {
                assert(contact.islandId == B2_NULL_INDEX);

                if ((flags & b2_contactEnableContactEvents) != 0) {
                    let event = new b2ContactBeginTouchEvent();
                    event.shapeIdA = shapeIdA;
                    event.shapeIdB = shapeIdB;
                    event.contactId = contactFullId;
                    world.contactBeginEvents.push(event);
                }

                assert(contactSim.manifold.pointCount > 0);
                assert(contact.setIndex == b2_awakeSet);

                // Link first because this wakes colliding bodies and ensures the body sims
                // are in the correct place.
                contact.flags |= b2_contactTouchingFlag;
                b2LinkContact(world, contact);

                // Make sure these didn't change
                assert(contact.colorIndex == B2_NULL_INDEX);
                assert(contact.localIndex == localIndex);

                // Refresh the sim reference (stable in AS, matches C refetch)
                contactSim = awakeSet.contactSims.get(localIndex);

                contactSim.simFlags &= ~b2_simStartedTouching;

                b2AddContactToGraph(world, contactSim, contact);
                b2RemoveNonTouchingContact(world, b2_awakeSet, localIndex);
            } else if ((simFlags & b2_simStoppedTouching) != 0) {
                contactSim.simFlags &= ~b2_simStoppedTouching;
                contact.flags &= ~b2_contactTouchingFlag;

                if ((contact.flags & b2_contactEnableContactEvents) != 0) {
                    let event = new b2ContactEndTouchEvent();
                    event.shapeIdA = shapeIdA;
                    event.shapeIdB = shapeIdB;
                    event.contactId = contactFullId;
                    world.contactEndEvents[endEventArrayIndex].push(event);
                }

                assert(contactSim.manifold.pointCount == 0);

                b2UnlinkContact(world, contact);
                let bodyIdA = contact.edges[0].bodyId;
                let bodyIdB = contact.edges[1].bodyId;

                b2AddNonTouchingContact(world, contact, contactSim);
                b2RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex);
            }

            // Clear the smallest set bit
            bits = bits & (bits - 1);
        }
    }

    b2ValidateSolverSets(world);
    b2ValidateContacts(world);
}

export function b2World_Step(worldId: b2WorldId, timeStep: f32, subStepCount: i32): void {
    assert(b2IsValidFloat(timeStep));
    assert(0 < subStepCount);

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    // Prepare to capture events
    // Ensure user does not access stale data if there is an early return
    world.bodyMoveEvents.clear();
    world.sensorBeginEvents.clear();
    world.contactBeginEvents.clear();
    world.contactHitEvents.clear();
    world.jointEvents.clear();

    world.profile = new b2Profile();

    if (timeStep == 0.0) {
        // Swap end event array buffers
        world.endEventArrayIndex = 1 - world.endEventArrayIndex;
        world.sensorEndEvents[world.endEventArrayIndex].clear();
        world.contactEndEvents[world.endEventArrayIndex].clear();
        return;
    }

    world.locked = true;
    world.activeTaskCount = 0;
    world.taskCount = 0;

    // Update collision pairs and create contacts
    b2UpdateBroadPhasePairs(world);

    let context = new b2StepContext();
    context.world = world;
    context.dt = timeStep;
    context.subStepCount = b2MaxInt(1, subStepCount);

    if (timeStep > 0.0) {
        context.inv_dt = <f32>1.0 / timeStep;
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
    let contactHertz = b2MinFloat(world.contactHertz, <f32>0.125 * context.inv_h);
    context.contactSoftness = b2MakeSoft(contactHertz, world.contactDampingRatio, context.h);
    context.staticSoftness = b2MakeSoft(<f32>2.0 * contactHertz, world.contactDampingRatio, context.h);

    context.restitutionThreshold = world.restitutionThreshold;
    context.maxLinearVelocity = world.maxLinearSpeed;
    context.enableWarmStarting = world.enableWarmStarting;

    // Update contacts
    b2Collide(context);

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (context.dt > 0.0) {
        b2Solve(world, context);
    }

    // Update sensors
    b2OverlapSensors(world);

    assert(b2GetArenaAllocation(world.arena) == 0);

    // Ensure stack is large enough
    b2GrowArena(world.arena);

    // Swap end event array buffers
    world.endEventArrayIndex = 1 - world.endEventArrayIndex;
    world.sensorEndEvents[world.endEventArrayIndex].clear();
    world.contactEndEvents[world.endEventArrayIndex].clear();
    world.locked = false;
}

export function b2World_GetBodyEvents(worldId: b2WorldId): b2BodyEvents {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return new b2BodyEvents();
    }

    let events = new b2BodyEvents();
    events.moveEvents = world.bodyMoveEvents.data;
    events.moveCount = world.bodyMoveEvents.count;
    return events;
}

export function b2World_GetSensorEvents(worldId: b2WorldId): b2SensorEvents {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return new b2SensorEvents();
    }

    // Careful to use previous buffer
    let endEventArrayIndex = 1 - world.endEventArrayIndex;

    let events = new b2SensorEvents();
    events.beginEvents = world.sensorBeginEvents.data;
    events.endEvents = world.sensorEndEvents[endEventArrayIndex].data;
    events.beginCount = world.sensorBeginEvents.count;
    events.endCount = world.sensorEndEvents[endEventArrayIndex].count;
    return events;
}

export function b2World_GetContactEvents(worldId: b2WorldId): b2ContactEvents {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return new b2ContactEvents();
    }

    // Careful to use previous buffer
    let endEventArrayIndex = 1 - world.endEventArrayIndex;

    let events = new b2ContactEvents();
    events.beginEvents = world.contactBeginEvents.data;
    events.endEvents = world.contactEndEvents[endEventArrayIndex].data;
    events.hitEvents = world.contactHitEvents.data;
    events.beginCount = world.contactBeginEvents.count;
    events.endCount = world.contactEndEvents[endEventArrayIndex].count;
    events.hitCount = world.contactHitEvents.count;

    return events;
}

export function b2World_GetJointEvents(worldId: b2WorldId): b2JointEvents {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return new b2JointEvents();
    }

    let events = new b2JointEvents();
    events.jointEvents = world.jointEvents.data;
    events.count = world.jointEvents.count;
    return events;
}

export function b2World_IsValid(id: b2WorldId): bool {
    if (id.index1 < 1 || B2_MAX_WORLDS < <i32>id.index1) {
        return false;
    }

    let world = b2_worlds[id.index1 - 1];

    if (<i32>world.worldId != <i32>id.index1 - 1) {
        // world is not allocated
        return false;
    }

    return id.generation == world.generation;
}

export function b2Shape_IsValid(id: b2ShapeId): bool {
    if (B2_MAX_WORLDS <= <i32>id.world0) {
        return false;
    }

    let world = b2GetWorldMaybe(id.world0);
    if (world == null) {
        return false;
    }

    let shapeId = id.index1 - 1;
    if (shapeId < 0 || world.shapes.count <= shapeId) {
        return false;
    }

    let shape = world.shapes.get(shapeId);
    if (shape.id == B2_NULL_INDEX) {
        // shape is free
        return false;
    }

    assert(shape.id == shapeId);

    return id.generation == shape.generation;
}

export function b2Chain_IsValid(id: b2ChainId): bool {
    if (B2_MAX_WORLDS <= <i32>id.world0) {
        return false;
    }

    let world = b2GetWorldMaybe(id.world0);
    if (world == null) {
        return false;
    }

    let chainId = id.index1 - 1;
    if (chainId < 0 || world.chainShapes.count <= chainId) {
        return false;
    }

    let chain = world.chainShapes.get(chainId);
    if (chain.id == B2_NULL_INDEX) {
        // chain is free
        return false;
    }

    assert(chain.id == chainId);

    return id.generation == chain.generation;
}

export function b2Joint_IsValid(id: b2JointId): bool {
    if (B2_MAX_WORLDS <= <i32>id.world0) {
        return false;
    }

    let world = b2GetWorldMaybe(id.world0);
    if (world == null) {
        return false;
    }

    let jointId = id.index1 - 1;
    if (jointId < 0 || world.joints.count <= jointId) {
        return false;
    }

    let joint = world.joints.get(jointId);
    if (joint.jointId == B2_NULL_INDEX) {
        // joint is free
        return false;
    }

    assert(joint.jointId == jointId);

    return id.generation == joint.generation;
}

export function b2Contact_IsValid(id: b2ContactId): bool {
    if (B2_MAX_WORLDS <= <i32>id.world0) {
        return false;
    }

    let world = b2GetWorldMaybe(id.world0);
    if (world == null) {
        return false;
    }

    let contactId = id.index1 - 1;
    if (contactId < 0 || world.contacts.count <= contactId) {
        return false;
    }

    let contact = world.contacts.get(contactId);
    if (contact.contactId == B2_NULL_INDEX) {
        // contact is free
        return false;
    }

    assert(contact.contactId == contactId);

    return id.generation == contact.generation;
}

export function b2World_EnableSleeping(worldId: b2WorldId, flag: bool): void {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    if (flag == world.enableSleep) {
        return;
    }

    world.enableSleep = flag;

    if (flag == false) {
        let setCount = world.solverSets.count;
        for (let i = b2_firstSleepingSet; i < setCount; ++i) {
            let set = world.solverSets.get(i);
            if (set.bodySims.count > 0) {
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
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    world.enableWarmStarting = flag;
}

export function b2World_IsWarmStartingEnabled(worldId: b2WorldId): bool {
    let world = b2GetWorldFromId(worldId);
    return world.enableWarmStarting;
}

export function b2World_GetAwakeBodyCount(worldId: b2WorldId): i32 {
    let world = b2GetWorldFromId(worldId);
    let awakeSet = world.solverSets.get(b2_awakeSet);
    return awakeSet.bodySims.count;
}

export function b2World_EnableContinuous(worldId: b2WorldId, flag: bool): void {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
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
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    world.restitutionThreshold = b2ClampFloat(value, 0.0, FLT_MAX);
}

export function b2World_GetRestitutionThreshold(worldId: b2WorldId): f32 {
    let world = b2GetWorldFromId(worldId);
    return world.restitutionThreshold;
}

export function b2World_SetHitEventThreshold(worldId: b2WorldId, value: f32): void {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
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
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    world.contactHertz = b2ClampFloat(hertz, 0.0, FLT_MAX);
    world.contactDampingRatio = b2ClampFloat(dampingRatio, 0.0, FLT_MAX);
    world.contactSpeed = b2ClampFloat(pushSpeed, 0.0, FLT_MAX);
}

export function b2World_SetMaximumLinearSpeed(worldId: b2WorldId, maximumLinearSpeed: f32): void {
    assert(b2IsValidFloat(maximumLinearSpeed) && maximumLinearSpeed > 0.0);

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    world.maxLinearSpeed = maximumLinearSpeed;
}

export function b2World_GetMaximumLinearSpeed(worldId: b2WorldId): f32 {
    let world = b2GetWorldFromId(worldId);
    return world.maxLinearSpeed;
}

export function b2World_GetCounters(worldId: b2WorldId): b2Counters {
    let world = b2GetWorldFromId(worldId);
    let s = new b2Counters();
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

    s.taskCount = world.taskCount;

    for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
        s.colorCounts[i] = world.constraintGraph.colors[i].contactSims.count +
            world.constraintGraph.colors[i].jointSims.count;
    }
    return s;
}

export function b2World_SetUserData(worldId: b2WorldId, userData: u64): void {
    let world = b2GetWorldFromId(worldId);
    world.userData = userData;
}

export function b2World_GetUserData(worldId: b2WorldId): u64 {
    let world = b2GetWorldFromId(worldId);
    return world.userData;
}

export function b2World_SetFrictionCallback(worldId: b2WorldId, callback: b2FrictionCallback | null): void {
    let world = b2GetWorldFromId(worldId);
    if (world.locked) {
        return;
    }

    if (callback != null) {
        world.frictionCallback = callback;
    } else {
        world.frictionCallback = new b2FrictionCallback();
    }
}

export function b2World_SetRestitutionCallback(worldId: b2WorldId, callback: b2RestitutionCallback | null): void {
    let world = b2GetWorldFromId(worldId);
    if (world.locked) {
        return;
    }

    if (callback != null) {
        world.restitutionCallback = callback;
    } else {
        world.restitutionCallback = new b2RestitutionCallback();
    }
}

// Queries

class WorldQueryTreeCallback extends b2TreeQueryCallback {
    world: b2World;
    fcn: b2OverlapResultFcn;
    filter: b2QueryFilter;

    constructor(world: b2World, fcn: b2OverlapResultFcn, filter: b2QueryFilter) {
        super();
        this.world = world;
        this.fcn = fcn;
        this.filter = filter;
    }

    queryCallback(proxyId: i32, userData: u64): bool {
        let shapeId = <i32>userData;
        let world = this.world;

        let shape = world.shapes.get(shapeId);

        if (b2ShouldQueryCollide(shape.filter, this.filter) == false) {
            return true;
        }

        let id = new b2ShapeId(shapeId + 1, world.worldId, shape.generation);
        return this.fcn.report(id);
    }
}

export function b2World_OverlapAABB(worldId: b2WorldId, aabb: b2AABB, filter: b2QueryFilter,
    fcn: b2OverlapResultFcn): b2TreeStats {
    let treeStats = new b2TreeStats();

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return treeStats;
    }

    assert(b2IsValidAABB(aabb));

    let callback = new WorldQueryTreeCallback(world, fcn, filter);

    for (let i = 0; i < b2_bodyTypeCount; ++i) {
        let treeResult = b2DynamicTree_Query(world.broadPhase.trees[i], aabb, filter.maskBits, callback);

        treeStats.nodeVisits += treeResult.nodeVisits;
        treeStats.leafVisits += treeResult.leafVisits;
    }

    return treeStats;
}

class WorldOverlapTreeCallback extends b2TreeQueryCallback {
    world: b2World;
    fcn: b2OverlapResultFcn;
    filter: b2QueryFilter;
    proxy: b2ShapeProxy;

    constructor(world: b2World, fcn: b2OverlapResultFcn, filter: b2QueryFilter, proxy: b2ShapeProxy) {
        super();
        this.world = world;
        this.fcn = fcn;
        this.filter = filter;
        this.proxy = proxy;
    }

    queryCallback(proxyId: i32, userData: u64): bool {
        let shapeId = <i32>userData;
        let world = this.world;

        let shape = world.shapes.get(shapeId);

        if (b2ShouldQueryCollide(shape.filter, this.filter) == false) {
            return true;
        }

        let body = world.bodies.get(shape.bodyId);
        let transform = b2GetBodyTransformQuick(world, body);

        let input = new b2DistanceInput();
        input.proxyA = this.proxy;
        input.proxyB = b2MakeShapeDistanceProxy(shape);
        input.transformA = b2Transform_identity();
        input.transformB = transform;
        input.useRadii = true;

        let cache = new b2SimplexCache();
        let output = b2ShapeDistance(input, cache);

        let tolerance = <f32>0.1 * B2_LINEAR_SLOP();
        if (output.distance > tolerance) {
            return true;
        }

        let id = new b2ShapeId(shape.id + 1, world.worldId, shape.generation);
        return this.fcn.report(id);
    }
}

export function b2World_OverlapShape(worldId: b2WorldId, proxy: b2ShapeProxy, filter: b2QueryFilter,
    fcn: b2OverlapResultFcn): b2TreeStats {
    let treeStats = new b2TreeStats();

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return treeStats;
    }

    let aabb = b2MakeAABBFromProxy(proxy);
    let callback = new WorldOverlapTreeCallback(world, fcn, filter, proxy);

    for (let i = 0; i < b2_bodyTypeCount; ++i) {
        let treeResult = b2DynamicTree_Query(world.broadPhase.trees[i], aabb, filter.maskBits, callback);

        treeStats.nodeVisits += treeResult.nodeVisits;
        treeStats.leafVisits += treeResult.leafVisits;
    }

    return treeStats;
}

function b2MakeAABBFromProxy(proxy: b2ShapeProxy): b2AABB {
    // b2MakeAABB(points, count, radius)
    return b2MakeAABBHelper(proxy.points, proxy.count, proxy.radius);
}

class WorldRayCastTreeCallback extends b2TreeRayCastCallback {
    world: b2World;
    fcn: b2CastResultFcn;
    filter: b2QueryFilter;
    fraction: f32 = 1.0;

    constructor(world: b2World, fcn: b2CastResultFcn, filter: b2QueryFilter) {
        super();
        this.world = world;
        this.fcn = fcn;
        this.filter = filter;
    }

    rayCastCallback(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
        let shapeId = <i32>userData;
        let world = this.world;

        let shape = world.shapes.get(shapeId);

        if (b2ShouldQueryCollide(shape.filter, this.filter) == false) {
            return input.maxFraction;
        }

        let body = world.bodies.get(shape.bodyId);
        let transform = b2GetBodyTransformQuick(world, body);
        let output = b2RayCastShape(input, shape, transform);

        if (output.hit) {
            let id = new b2ShapeId(shapeId + 1, world.worldId, shape.generation);
            let fraction = this.fcn.report(id, output.point, output.normal, output.fraction);

            // The user may return -1 to skip this shape
            if (0.0 <= fraction && fraction <= 1.0) {
                this.fraction = fraction;
            }

            return fraction;
        }

        return input.maxFraction;
    }
}

export function b2World_CastRay(worldId: b2WorldId, origin: b2Vec2, translation: b2Vec2, filter: b2QueryFilter,
    fcn: b2CastResultFcn): b2TreeStats {
    let treeStats = new b2TreeStats();

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return treeStats;
    }

    assert(b2IsValidVec2(origin));
    assert(b2IsValidVec2(translation));

    let input = new b2RayCastInput();
    input.origin = origin.clone();
    input.translation = translation.clone();
    input.maxFraction = 1.0;

    let callback = new WorldRayCastTreeCallback(world, fcn, filter);

    for (let i = 0; i < b2_bodyTypeCount; ++i) {
        let treeResult = b2DynamicTree_RayCast(world.broadPhase.trees[i], input, filter.maskBits, callback);
        treeStats.nodeVisits += treeResult.nodeVisits;
        treeStats.leafVisits += treeResult.leafVisits;

        if (callback.fraction == 0.0) {
            return treeStats;
        }

        input.maxFraction = callback.fraction;
    }

    return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
class b2RayCastClosestFcn extends b2CastResultFcn {
    rayResult: b2RayResult;

    constructor(rayResult: b2RayResult) {
        super();
        this.rayResult = rayResult;
    }

    report(shapeId: b2ShapeId, point: b2Vec2, normal: b2Vec2, fraction: f32): f32 {
        // Ignore initial overlap
        if (fraction == 0.0) {
            return -1.0;
        }

        let rayResult = this.rayResult;
        rayResult.shapeId = shapeId;
        rayResult.point = point.clone();
        rayResult.normal = normal.clone();
        rayResult.fraction = fraction;
        rayResult.hit = true;
        return fraction;
    }
}

export function b2World_CastRayClosest(worldId: b2WorldId, origin: b2Vec2, translation: b2Vec2,
    filter: b2QueryFilter): b2RayResult {
    let result = new b2RayResult();

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return result;
    }

    assert(b2IsValidVec2(origin));
    assert(b2IsValidVec2(translation));

    let input = new b2RayCastInput();
    input.origin = origin.clone();
    input.translation = translation.clone();
    input.maxFraction = 1.0;

    let fcn = new b2RayCastClosestFcn(result);
    let callback = new WorldRayCastTreeCallback(world, fcn, filter);

    for (let i = 0; i < b2_bodyTypeCount; ++i) {
        let treeResult = b2DynamicTree_RayCast(world.broadPhase.trees[i], input, filter.maskBits, callback);
        result.nodeVisits += treeResult.nodeVisits;
        result.leafVisits += treeResult.leafVisits;

        if (callback.fraction == 0.0) {
            return result;
        }

        input.maxFraction = callback.fraction;
    }

    return result;
}

class WorldShapeCastTreeCallback extends b2TreeShapeCastCallback {
    world: b2World;
    fcn: b2CastResultFcn;
    filter: b2QueryFilter;
    fraction: f32 = 1.0;

    constructor(world: b2World, fcn: b2CastResultFcn, filter: b2QueryFilter) {
        super();
        this.world = world;
        this.fcn = fcn;
        this.filter = filter;
    }

    shapeCastCallback(input: b2ShapeCastInput, proxyId: i32, userData: u64): f32 {
        let shapeId = <i32>userData;
        let world = this.world;

        let shape = world.shapes.get(shapeId);

        if (b2ShouldQueryCollide(shape.filter, this.filter) == false) {
            return input.maxFraction;
        }

        let body = world.bodies.get(shape.bodyId);
        let transform = b2GetBodyTransformQuick(world, body);

        let output = b2ShapeCastShape(input, shape, transform);

        if (output.hit) {
            let id = new b2ShapeId(shapeId + 1, world.worldId, shape.generation);
            let fraction = this.fcn.report(id, output.point, output.normal, output.fraction);

            // The user may return -1 to skip this shape
            if (0.0 <= fraction && fraction <= 1.0) {
                this.fraction = fraction;
            }

            return fraction;
        }

        return input.maxFraction;
    }
}

export function b2World_CastShape(worldId: b2WorldId, proxy: b2ShapeProxy, translation: b2Vec2, filter: b2QueryFilter,
    fcn: b2CastResultFcn): b2TreeStats {
    let treeStats = new b2TreeStats();

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return treeStats;
    }

    assert(b2IsValidVec2(translation));

    let input = new b2ShapeCastInput();
    input.proxy = proxy;
    input.translation = translation.clone();
    input.maxFraction = 1.0;

    let callback = new WorldShapeCastTreeCallback(world, fcn, filter);

    for (let i = 0; i < b2_bodyTypeCount; ++i) {
        let treeResult = b2DynamicTree_ShapeCast(world.broadPhase.trees[i], input, filter.maskBits, callback);
        treeStats.nodeVisits += treeResult.nodeVisits;
        treeStats.leafVisits += treeResult.leafVisits;

        if (callback.fraction == 0.0) {
            return treeStats;
        }

        input.maxFraction = callback.fraction;
    }

    return treeStats;
}

export function b2World_SetCustomFilterCallback(worldId: b2WorldId, fcn: b2CustomFilterFcn | null): void {
    let world = b2GetWorldFromId(worldId);
    world.customFilterFcn = fcn;
}

export function b2World_SetPreSolveCallback(worldId: b2WorldId, fcn: b2PreSolveFcn | null): void {
    let world = b2GetWorldFromId(worldId);
    world.preSolveFcn = fcn;
}

export function b2World_SetGravity(worldId: b2WorldId, gravity: b2Vec2): void {
    let world = b2GetWorldFromId(worldId);
    world.gravity = gravity.clone();
}

export function b2World_GetGravity(worldId: b2WorldId): b2Vec2 {
    let world = b2GetWorldFromId(worldId);
    return world.gravity.clone();
}

class ExplosionCallback extends b2TreeQueryCallback {
    world: b2World;
    position: b2Vec2;
    radius: f32;
    falloff: f32;
    impulsePerLength: f32;

    constructor(world: b2World, position: b2Vec2, radius: f32, falloff: f32, impulsePerLength: f32) {
        super();
        this.world = world;
        this.position = position;
        this.radius = radius;
        this.falloff = falloff;
        this.impulsePerLength = impulsePerLength;
    }

    queryCallback(proxyId: i32, userData: u64): bool {
        let shapeId = <i32>userData;
        let world = this.world;

        let shape = world.shapes.get(shapeId);

        let body = world.bodies.get(shape.bodyId);
        assert(body.type == b2_dynamicBody);

        let transform = b2GetBodyTransformQuick(world, body);

        let input = new b2DistanceInput();
        input.proxyA = b2MakeShapeDistanceProxy(shape);
        {
            let pts = makeVecArray(1);
            pts[0] = this.position.clone();
            input.proxyB = b2MakeProxy(pts, 1, 0.0);
        }
        input.transformA = transform;
        input.transformB = b2Transform_identity();
        input.useRadii = true;

        let cache = new b2SimplexCache();
        let output = b2ShapeDistance(input, cache);

        let radius = this.radius;
        let falloff = this.falloff;
        if (output.distance > radius + falloff) {
            return true;
        }

        b2WakeBody(world, body);

        if (body.setIndex != b2_awakeSet) {
            return true;
        }

        let closestPoint = output.pointA;
        if (output.distance == 0.0) {
            let localCentroid = b2GetShapeCentroid(shape);
            closestPoint = b2TransformPointHelper(transform, localCentroid);
        }

        let direction = b2Sub(closestPoint, this.position);
        if (b2LengthSquared(direction) > <f32>100.0 * FLT_EPSILON * FLT_EPSILON) {
            direction = b2Normalize(direction);
        } else {
            direction = new b2Vec2(1.0, 0.0);
        }

        let localLine = b2InvRotateVector(transform.q, b2LeftPerp(direction));
        let perimeter = b2GetShapeProjectedPerimeter(shape, localLine);
        let scale: f32 = 1.0;
        if (output.distance > radius && falloff > 0.0) {
            scale = b2ClampFloat((radius + falloff - output.distance) / falloff, 0.0, 1.0);
        }

        let magnitude = this.impulsePerLength * perimeter * scale;
        let impulse = b2MulSV(magnitude, direction);

        let localIndex = body.localIndex;
        let set = world.solverSets.get(b2_awakeSet);
        let state = set.bodyStates.get(localIndex);
        let bodySim = set.bodySims.get(localIndex);
        state.linearVelocity = b2MulAdd(state.linearVelocity, bodySim.invMass, impulse);
        state.angularVelocity += bodySim.invInertia * b2Cross(b2Sub(closestPoint, bodySim.center), impulse);

        return true;
    }
}

export function b2World_Explode(worldId: b2WorldId, explosionDef: b2ExplosionDef): void {
    let maskBits = explosionDef.maskBits;
    let position = explosionDef.position;
    let radius = explosionDef.radius;
    let falloff = explosionDef.falloff;
    let impulsePerLength = explosionDef.impulsePerLength;

    assert(b2IsValidVec2(position));
    assert(b2IsValidFloat(radius) && radius >= 0.0);
    assert(b2IsValidFloat(falloff) && falloff >= 0.0);
    assert(b2IsValidFloat(impulsePerLength));

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    let callback = new ExplosionCallback(world, position, radius, falloff, impulsePerLength);

    let aabb = new b2AABB();
    aabb.lowerBound.x = position.x - (radius + falloff);
    aabb.lowerBound.y = position.y - (radius + falloff);
    aabb.upperBound.x = position.x + (radius + falloff);
    aabb.upperBound.y = position.y + (radius + falloff);

    b2DynamicTree_Query(world.broadPhase.trees[b2_dynamicBody], aabb, maskBits, callback);
}

export function b2World_RebuildStaticTree(worldId: b2WorldId): void {
    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    let staticTree = world.broadPhase.trees[b2_staticBody];
    b2DynamicTree_Rebuild(staticTree, true);
}

export function b2World_EnableSpeculative(worldId: b2WorldId, flag: bool): void {
    let world = b2GetWorldFromId(worldId);
    world.enableSpeculative = flag;
}

import { b2MakeAABB as b2MakeAABBHelper, b2TransformPoint as b2TransformPointHelper } from "./math_functions";
