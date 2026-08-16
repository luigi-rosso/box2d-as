// Ported from box2d src/solver.h/.c. Single threaded: the stage/block
// machinery runs with one worker, atomics become plain ints, and enqueued
// tasks execute inline over their full range (C's default task callback).

import { b2Vec2, B2_PI } from "./math_functions";
import { b2IntArray } from "./array";
import { b2World } from "./physics_world";
import { b2ConstraintGraph } from "./constraint_graph";
import { b2BodyState, b2BodySim } from "./body";
import { b2ContactSim } from "./contact";
import { b2JointSim } from "./joint";
import { b2ContactConstraintSIMD } from "./contact_solver";

export class b2Softness {
    biasRate: f32 = 0;
    massScale: f32 = 0;
    impulseScale: f32 = 0;

    clone(): b2Softness {
        let s = new b2Softness();
        s.biasRate = this.biasRate;
        s.massScale = this.massScale;
        s.impulseScale = this.impulseScale;
        return s;
    }
}

// b2SolverStageType
export const b2_stagePrepareJoints: i32 = 0;
export const b2_stagePrepareContacts: i32 = 1;
export const b2_stageIntegrateVelocities: i32 = 2;
export const b2_stageWarmStart: i32 = 3;
export const b2_stageSolve: i32 = 4;
export const b2_stageIntegratePositions: i32 = 5;
export const b2_stageRelax: i32 = 6;
export const b2_stageRestitution: i32 = 7;
export const b2_stageStoreImpulses: i32 = 8;

// b2SolverBlockType
export const b2_bodyBlock: i32 = 0;
export const b2_jointBlock: i32 = 1;
export const b2_contactBlock: i32 = 2;
export const b2_graphJointBlock: i32 = 3;
export const b2_graphContactBlock: i32 = 4;

export class b2SolverBlock {
    startIndex: i32 = 0;
    count: i16 = 0;
    blockType: i16 = 0;
    syncIndex: i32 = 0;
}

export class b2SolverStage {
    type: i32 = 0;
    blocks: Array<b2SolverBlock> = new Array<b2SolverBlock>();
    blockCount: i32 = 0;
    colorIndex: i32 = 0;
    completionCount: i32 = 0;
}

// Context for a time step. Recreated each time step.
export class b2StepContext {
    dt: f32 = 0;
    inv_dt: f32 = 0;
    h: f32 = 0;
    inv_h: f32 = 0;
    subStepCount: i32 = 0;
    contactSoftness: b2Softness = new b2Softness();
    staticSoftness: b2Softness = new b2Softness();
    restitutionThreshold: f32 = 0;
    maxLinearVelocity: f32 = 0;
    world: b2World | null = null;
    graph: b2ConstraintGraph | null = null;
    // shortcut to body states from awake set
    states: Array<b2BodyState | null> = new Array<b2BodyState | null>();
    // shortcut to body sims from awake set
    sims: Array<b2BodySim | null> = new Array<b2BodySim | null>();
    // shape ids for shapes with enlarged AABBs
    enlargedShapes: b2IntArray = new b2IntArray();
    // bullet bodies that need continuous collision handling
    bulletBodies: b2IntArray = new b2IntArray();
    // joint pointers for simplified parallel-for access
    joints: Array<b2JointSim | null> = new Array<b2JointSim | null>();
    // contact pointers with NULL gaps for SIMD remainders, packed by color
    contacts: Array<b2ContactSim | null> = new Array<b2ContactSim | null>();
    simdContactConstraints: StaticArray<b2ContactConstraintSIMD> | null = null;
    activeColorCount: i32 = 0;
    workerCount: i32 = 0;
    stages: Array<b2SolverStage> = new Array<b2SolverStage>();
    stageCount: i32 = 0;
    enableWarmStarting: bool = false;
    atomicSyncBits: u32 = 0;
}

export function b2MakeSoft(hertz: f32, zeta: f32, h: f32): b2Softness {
    let out = new b2Softness();
    if (hertz == 0.0) {
        return out;
    }

    let omega = <f32>2.0 * B2_PI * hertz;
    let a1 = <f32>2.0 * zeta + h * omega;
    let a2 = h * omega * a1;
    let a3 = <f32>1.0 / (<f32>1.0 + a2);

    out.biasRate = omega / a1;
    out.massScale = a2 * a3;
    out.impulseScale = a3;
    return out;
}

// ---------------------------------------------------------------------------
// solver.c port
// ---------------------------------------------------------------------------

import { B2_NULL_INDEX, b2CTZ64 } from "./core";
import {
    B2_MAX_ROTATION, B2_TIME_TO_SLEEP, B2_GRAPH_COLOR_COUNT, B2_LINEAR_SLOP,
    B2_SPECULATIVE_DISTANCE, B2_AABB_MARGIN,
} from "./constants";
import {
    b2Rot, b2Transform, b2AABB, b2Add, b2Sub, b2MulSV, b2MulAdd, b2Dot, b2Cross, b2Length,
    b2AbsFloat, b2MaxFloat, b2MinInt, b2RotateVector, b2TransformPoint, b2NormalizeRot, b2MulRot,
    b2IntegrateRotation, b2NLerp, b2Lerp, b2Vec2_zero, b2Rot_identity, b2GetLengthAndNormalize,
    b2AABB_Union, b2AABB_Contains, b2IsValidVec2, b2IsValidFloat, FLT_MAX,
} from "./math_functions";
import { b2TOIInput, b2TOIOutput, b2Sweep, makeVecArray } from "./collision";
import { b2TimeOfImpact, b2MakeProxy } from "./distance";
import { b2ShapeId } from "./id";
import { b2IntArray_Create, b2IntArray_Push, b2IntArray_Clear } from "./array";
import { b2SetBit, b2GetBit, b2SetBitCountAndClear, b2InPlaceUnion, b2BitSet } from "./bitset";
import { b2GetIdCapacity } from "./id_pool";
import {
    b2GetWorld, b2ValidateSolverSets, b2TaskContext, b2BodyMoveEvent, b2JointEvent, b2ContactHitEvent,
    b2_awakeSet, b2_staticSet,
} from "./physics_world";
import {
    b2_lockLinearX, b2_lockLinearY, b2_lockAngularZ, b2_allowFastRotation, b2_isSpeedCapped,
    b2_isFast, b2_isBullet, b2_hadTimeOfImpact, b2_enlargeBounds, b2_dirtyMass,
    b2MakeSweep, b2GetBodySim,
} from "./body";
import { b2Shape, b2ComputeShapeAABB, b2MakeShapeDistanceProxy, b2ShouldShapesCollide,
    b2GetShapeCentroid, b2ComputeShapeExtent } from "./shape";
import { b2ShouldBodiesCollide } from "./body";
import { b2_simEnableHitEvent } from "./contact";
import { b2PrepareJoint, b2WarmStartJoint, b2SolveJoint, b2GetJointReaction,
    b2PrepareOverflowJoints, b2WarmStartOverflowJoints, b2SolveOverflowJoints } from "./joint";
import {
    b2PrepareOverflowContacts, b2WarmStartOverflowContacts, b2SolveOverflowContacts,
    b2ApplyOverflowRestitution, b2StoreOverflowImpulses,
    b2PrepareContactsTask, b2WarmStartContactsTask, b2SolveContactsTask, b2ApplyRestitutionTask,
    b2StoreImpulsesTask, b2ContactConstraint, b2ContactConstraintSIMD as b2SimdConstraint,
    b2AllocOverflowConstraints, b2AllocSimdConstraints,
} from "./contact_solver";
import { b2SplitIslandTask, b2IslandSim } from "./island";
import { b2TrySleepIsland } from "./solver_set";
import { b2SensorHit } from "./sensor";
import {
    b2_staticBody, b2_kinematicBody, b2_dynamicBody, B2_PROXY_ID, B2_PROXY_TYPE,
    b2BufferMove, b2BroadPhase_EnlargeProxy,
} from "./broad_phase";
import { b2DynamicTree_Query, b2DynamicTree_EnlargeProxy, b2TreeQueryCallback } from "./dynamic_tree";
import { b2ContainsKey } from "./table";
import { b2GetJointSim } from "./joint";
import { b2ContactId } from "./id";

const ITERATIONS: i32 = 1;
const RELAX_ITERATIONS: i32 = 1;

const B2_SIMD_WIDTH: i32 = 4;
const B2_SIMD_SHIFT: i32 = 2;

// Integrate velocities and apply damping
function b2IntegrateVelocitiesTask(startIndex: i32, endIndex: i32, context: b2StepContext): void {
    let states = context.states;
    let sims = context.sims;

    let gravity = context.world!.gravity;
    let h = context.h;
    let maxLinearSpeed = context.maxLinearVelocity;
    let maxAngularSpeed = B2_MAX_ROTATION * context.inv_dt;
    let maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
    let maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

    for (let i = startIndex; i < endIndex; ++i) {
        let sim = unchecked(sims[i])!;
        let state = unchecked(states[i])!;

        let v = state.linearVelocity;
        let w = state.angularVelocity;

        // v2 = v1 * 1 / (1 + c * dt)
        let linearDamping = <f32>1.0 / (<f32>1.0 + h * sim.linearDamping);
        let angularDamping = <f32>1.0 / (<f32>1.0 + h * sim.angularDamping);

        // Gravity scale will be zero for kinematic bodies
        let gravityScale = sim.invMass > 0.0 ? sim.gravityScale : <f32>0.0;

        // lvd = h * im * f + h * g; scalar form of the pure helpers, written
        // back in place to keep the substep loop allocation free.
        let hm: f32 = h * sim.invMass;
        let hg: f32 = h * gravityScale;
        let angularVelocityDelta = h * sim.invInertia * sim.torque;

        let vx: f32 = (hm * sim.force.x + hg * gravity.x) + linearDamping * v.x;
        let vy: f32 = (hm * sim.force.y + hg * gravity.y) + linearDamping * v.y;
        w = angularVelocityDelta + angularDamping * w;

        // Clamp to max linear speed
        if (vx * vx + vy * vy > maxLinearSpeedSquared) {
            let ratio = maxLinearSpeed / sqrt<f32>(vx * vx + vy * vy);
            vx = ratio * vx;
            vy = ratio * vy;
            sim.flags |= b2_isSpeedCapped;
        }

        // Clamp to max angular speed
        if (w * w > maxAngularSpeedSquared && (sim.flags & b2_allowFastRotation) == 0) {
            let ratio = maxAngularSpeed / b2AbsFloat(w);
            w *= ratio;
            sim.flags |= b2_isSpeedCapped;
        }

        if ((state.flags & b2_lockLinearX) != 0) {
            vx = 0.0;
        }

        if ((state.flags & b2_lockLinearY) != 0) {
            vy = 0.0;
        }

        if ((state.flags & b2_lockAngularZ) != 0) {
            w = 0.0;
        }

        v.x = vx;
        v.y = vy;
        state.angularVelocity = w;
    }
}

function b2PrepareJointsTask(startIndex: i32, endIndex: i32, context: b2StepContext): void {
    let joints = context.joints;

    for (let i = startIndex; i < endIndex; ++i) {
        let joint = joints[i]!;
        b2PrepareJoint(joint, context);
    }
}

function b2WarmStartJointsTask(startIndex: i32, endIndex: i32, context: b2StepContext, colorIndex: i32): void {
    let color = context.graph!.colors[colorIndex];
    assert(0 <= startIndex && startIndex < color.jointSims.count);
    assert(startIndex <= endIndex && endIndex <= color.jointSims.count);

    for (let i = startIndex; i < endIndex; ++i) {
        let joint = color.jointSims.get(i);
        b2WarmStartJoint(joint, context);
    }
}

function b2SolveJointsTask(startIndex: i32, endIndex: i32, context: b2StepContext, colorIndex: i32,
    useBias: bool, workerIndex: i32): void {
    let color = context.graph!.colors[colorIndex];
    assert(0 <= startIndex && startIndex < color.jointSims.count);
    assert(startIndex <= endIndex && endIndex <= color.jointSims.count);

    let jointStateBitSet = context.world!.taskContexts.get(workerIndex).jointStateBitSet;

    for (let i = startIndex; i < endIndex; ++i) {
        let joint = color.jointSims.get(i);
        b2SolveJoint(joint, context, useBias);

        if (useBias && (joint.forceThreshold < FLT_MAX || joint.torqueThreshold < FLT_MAX) &&
            b2GetBit(jointStateBitSet, joint.jointId) == false) {
            let reaction = b2GetJointReaction(joint, context.inv_h);

            // Check thresholds. A zero threshold means all awake joints get reported.
            if (reaction.force >= joint.forceThreshold || reaction.torque >= joint.torqueThreshold) {
                // Flag this joint for processing.
                b2SetBit(jointStateBitSet, joint.jointId);
            }
        }
    }
}

function b2IntegratePositionsTask(startIndex: i32, endIndex: i32, context: b2StepContext): void {
    let states = context.states;
    let h = context.h;

    assert(startIndex <= endIndex);

    for (let i = startIndex; i < endIndex; ++i) {
        let state = unchecked(states[i])!;

        if ((state.flags & b2_lockLinearX) != 0) {
            state.linearVelocity.x = 0.0;
        }

        if ((state.flags & b2_lockLinearY) != 0) {
            state.linearVelocity.y = 0.0;
        }

        if ((state.flags & b2_lockAngularZ) != 0) {
            state.angularVelocity = 0.0;
        }

        // In-place b2MulAdd / b2IntegrateRotation; nothing aliases these.
        let dp = state.deltaPosition;
        dp.x = dp.x + h * state.linearVelocity.x;
        dp.y = dp.y + h * state.linearVelocity.y;
        let dq = state.deltaRotation;
        let deltaAngle: f32 = h * state.angularVelocity;
        let q2c: f32 = dq.c - deltaAngle * dq.s;
        let q2s: f32 = dq.s + deltaAngle * dq.c;
        let mag: f32 = sqrt<f32>(q2s * q2s + q2c * q2c);
        let invMag: f32 = mag > 0.0 ? <f32>1.0 / mag : <f32>0.0;
        dq.c = q2c * invMag;
        dq.s = q2s * invMag;
    }
}

const B2_MAX_CONTINUOUS_SENSOR_HITS: i32 = 8;
const B2_CORE_FRACTION: f32 = 0.25;

class b2ContinuousContext extends b2TreeQueryCallback {
    world: b2World | null = null;
    fastBodySim: b2BodySim | null = null;
    fastShape: b2Shape | null = null;
    centroid1: b2Vec2 = new b2Vec2();
    centroid2: b2Vec2 = new b2Vec2();
    sweep: b2Sweep = new b2Sweep();
    fraction: f32 = 0;
    sensorHits: StaticArray<b2SensorHit> = initSensorHits();
    sensorFractions: StaticArray<f32> = new StaticArray<f32>(B2_MAX_CONTINUOUS_SENSOR_HITS);
    sensorCount: i32 = 0;

    // This is called from b2DynamicTree_Query for continuous collision
    queryCallback(proxyId: i32, userData: u64): bool {
        let shapeId = <i32>userData;

        let fastShape = this.fastShape!;
        let fastBodySim = this.fastBodySim!;

        assert(fastShape.sensorIndex == B2_NULL_INDEX);

        // Skip same shape
        if (shapeId == fastShape.id) {
            return true;
        }

        let world = this.world!;

        let shape = world.shapes.get(shapeId);

        // Skip same body
        if (shape.bodyId == fastShape.bodyId) {
            return true;
        }

        let isSensor = shape.sensorIndex != B2_NULL_INDEX;

        // Skip sensors unless the shapes want sensor events
        if (isSensor && (shape.enableSensorEvents == false || fastShape.enableSensorEvents == false)) {
            return true;
        }

        // Skip filtered shapes
        let canCollide = b2ShouldShapesCollide(fastShape.filter, shape.filter);
        if (canCollide == false) {
            return true;
        }

        let body = world.bodies.get(shape.bodyId);

        let bodySim = b2GetBodySim(world, body);
        assert(body.type == b2_staticBody || (fastBodySim.flags & b2_isBullet) != 0);

        // Skip bullets
        if ((bodySim.flags & b2_isBullet) != 0) {
            return true;
        }

        // Skip filtered bodies
        let fastBody = world.bodies.get(fastBodySim.bodyId);
        canCollide = b2ShouldBodiesCollide(world, fastBody, body);
        if (canCollide == false) {
            return true;
        }

        // Custom user filtering
        if (shape.enableCustomFiltering || fastShape.enableCustomFiltering) {
            let customFilterFcn = world.customFilterFcn;
            if (customFilterFcn != null) {
                let idA = new b2ShapeId(shape.id + 1, world.worldId, shape.generation);
                let idB = new b2ShapeId(fastShape.id + 1, world.worldId, fastShape.generation);
                canCollide = customFilterFcn.shouldCollide(idA, idB);
                if (canCollide == false) {
                    return true;
                }
            }
        }

        // Early out on fast parallel movement over a chain shape.
        if (shape.type == b2_chainSegmentShapeConst) {
            let transform = bodySim.transform;
            let p1 = b2TransformPoint(transform, shape.chainSegment!.segment.point1);
            let p2 = b2TransformPoint(transform, shape.chainSegment!.segment.point2);
            let e = b2Sub(p2, p1);
            let ln = b2GetLengthAndNormalize(e);
            let length = ln.length;
            e = ln.normal;
            if (length > B2_LINEAR_SLOP()) {
                let c1 = this.centroid1;
                let separation1 = b2Cross(b2Sub(c1, p1), e);
                let c2 = this.centroid2;
                let separation2 = b2Cross(b2Sub(c2, p1), e);

                let coreDistance = B2_CORE_FRACTION * fastBodySim.minExtent;

                if (separation1 < 0.0 || (separation1 - separation2 < coreDistance && separation2 > coreDistance)) {
                    // Minimal clipping
                    return true;
                }
            }
        }

        let input = new b2TOIInput();
        input.proxyA = b2MakeShapeDistanceProxy(shape);
        input.proxyB = b2MakeShapeDistanceProxy(fastShape);
        input.sweepA = b2MakeSweep(bodySim);
        input.sweepB = this.sweep.clone();
        input.maxFraction = this.fraction;

        let output = b2TimeOfImpact(input);
        if (isSensor) {
            // Only accept a sensor hit that is sooner than the current solid hit.
            if (output.fraction <= this.fraction && this.sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS) {
                let index = this.sensorCount;

                // The hit shape is a sensor
                let sensorHit = this.sensorHits[index];
                sensorHit.sensorId = shape.id;
                sensorHit.visitorId = fastShape.id;

                this.sensorFractions[index] = output.fraction;
                this.sensorCount += 1;
            }
        } else {
            let hitFraction = this.fraction;
            let didHit = false;

            if (0.0 < output.fraction && output.fraction < this.fraction) {
                hitFraction = output.fraction;
                didHit = true;
            } else if (0.0 == output.fraction) {
                // fallback to TOI of a small circle around the fast shape centroid
                let centroid = b2GetShapeCentroid(fastShape);
                let extent = b2ComputeShapeExtent(fastShape, centroid);
                let radius = B2_CORE_FRACTION * extent.minExtent;
                let pts = makeVecArray(1);
                pts[0] = centroid;
                input.proxyB = b2MakeProxy(pts, 1, radius);
                output = b2TimeOfImpact(input);
                if (0.0 < output.fraction && output.fraction < this.fraction) {
                    hitFraction = output.fraction;
                    didHit = true;
                }
            }

            if (didHit && (shape.enablePreSolveEvents || fastShape.enablePreSolveEvents) && world.preSolveFcn != null) {
                let shapeIdA = new b2ShapeId(shape.id + 1, world.worldId, shape.generation);
                let shapeIdB = new b2ShapeId(fastShape.id + 1, world.worldId, fastShape.generation);
                didHit = world.preSolveFcn!.preSolve(shapeIdA, shapeIdB, output.point, output.normal);
            }

            if (didHit) {
                fastBodySim.flags |= b2_hadTimeOfImpact;
                this.fraction = hitFraction;
            }
        }

        // Continue query
        return true;
    }
}

function initSensorHits(): StaticArray<b2SensorHit> {
    let a = new StaticArray<b2SensorHit>(B2_MAX_CONTINUOUS_SENSOR_HITS);
    for (let i = 0; i < B2_MAX_CONTINUOUS_SENSOR_HITS; ++i) {
        a[i] = new b2SensorHit();
    }
    return a;
}

function b2SolveContinuous(world: b2World, bodySimIndex: i32, taskContext: b2TaskContext): void {
    let awakeSet = world.solverSets.get(b2_awakeSet);
    let fastBodySim = awakeSet.bodySims.get(bodySimIndex);
    assert((fastBodySim.flags & b2_isFast) != 0);

    let sweep = b2MakeSweep(fastBodySim);

    let xf1 = new b2Transform();
    xf1.q = sweep.q1.clone();
    xf1.p = b2Sub(sweep.c1, b2RotateVector(sweep.q1, sweep.localCenter));

    let xf2 = new b2Transform();
    xf2.q = sweep.q2.clone();
    xf2.p = b2Sub(sweep.c2, b2RotateVector(sweep.q2, sweep.localCenter));

    let staticTree = world.broadPhase.trees[b2_staticBody];
    let kinematicTree = world.broadPhase.trees[b2_kinematicBody];
    let dynamicTree = world.broadPhase.trees[b2_dynamicBody];
    let fastBody = world.bodies.get(fastBodySim.bodyId);

    let context = new b2ContinuousContext();
    context.world = world;
    context.sweep = sweep;
    context.fastBodySim = fastBodySim;
    context.fraction = 1.0;

    let isBullet = (fastBodySim.flags & b2_isBullet) != 0;

    let shapeId = fastBody.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let fastShape = world.shapes.get(shapeId);
        shapeId = fastShape.nextShapeId;

        context.fastShape = fastShape;
        context.centroid1 = b2TransformPoint(xf1, fastShape.localCentroid);
        context.centroid2 = b2TransformPoint(xf2, fastShape.localCentroid);

        let box1 = fastShape.aabb.clone();
        let box2 = b2ComputeShapeAABB(fastShape, xf2);

        // Store this to avoid double computation in the case there is no impact event
        fastShape.aabb = box2;

        // No continuous collision for sensors (but still need the updated bounds)
        if (fastShape.sensorIndex != B2_NULL_INDEX) {
            continue;
        }

        let sweptBox = b2AABB_Union(box1, box2);

        b2DynamicTree_Query(staticTree, sweptBox, 0xffffffffffffffff, context);

        if (isBullet) {
            b2DynamicTree_Query(kinematicTree, sweptBox, 0xffffffffffffffff, context);
            b2DynamicTree_Query(dynamicTree, sweptBox, 0xffffffffffffffff, context);
        }
    }

    let speculativeDistance = B2_SPECULATIVE_DISTANCE();
    let aabbMargin = B2_AABB_MARGIN();

    if (context.fraction < 1.0) {
        // Handle time of impact event
        let q = b2NLerp(sweep.q1, sweep.q2, context.fraction);
        let c = b2Lerp(sweep.c1, sweep.c2, context.fraction);
        let origin = b2Sub(c, b2RotateVector(q, sweep.localCenter));

        // Advance body
        let transform = new b2Transform(origin, q);
        fastBodySim.transform = transform;
        fastBodySim.center = c;
        fastBodySim.rotation0 = q.clone();
        fastBodySim.center0 = c.clone();

        // Update body move event
        let event = world.bodyMoveEvents.get(bodySimIndex);
        event.transform = transform.clone();

        // Prepare AABBs for broad-phase.
        shapeId = fastBody.headShapeId;
        while (shapeId != B2_NULL_INDEX) {
            let shape = world.shapes.get(shapeId);

            // Must recompute aabb at the interpolated transform
            let aabb = b2ComputeShapeAABB(shape, transform);
            aabb.lowerBound.x -= speculativeDistance;
            aabb.lowerBound.y -= speculativeDistance;
            aabb.upperBound.x += speculativeDistance;
            aabb.upperBound.y += speculativeDistance;
            shape.aabb = aabb;

            if (b2AABB_Contains(shape.fatAABB, aabb) == false) {
                let fatAABB = new b2AABB();
                fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
                fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
                fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
                fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
                shape.fatAABB = fatAABB;

                shape.enlargedAABB = true;
                fastBodySim.flags |= b2_enlargeBounds;
            }

            shapeId = shape.nextShapeId;
        }
    } else {
        // No time of impact event

        // Advance body
        fastBodySim.rotation0 = fastBodySim.transform.q.clone();
        fastBodySim.center0 = fastBodySim.center.clone();

        // Prepare AABBs for broad-phase
        shapeId = fastBody.headShapeId;
        while (shapeId != B2_NULL_INDEX) {
            let shape = world.shapes.get(shapeId);

            // shape.aabb is still valid from above

            if (b2AABB_Contains(shape.fatAABB, shape.aabb) == false) {
                let fatAABB = new b2AABB();
                fatAABB.lowerBound.x = shape.aabb.lowerBound.x - aabbMargin;
                fatAABB.lowerBound.y = shape.aabb.lowerBound.y - aabbMargin;
                fatAABB.upperBound.x = shape.aabb.upperBound.x + aabbMargin;
                fatAABB.upperBound.y = shape.aabb.upperBound.y + aabbMargin;
                shape.fatAABB = fatAABB;

                shape.enlargedAABB = true;
                fastBodySim.flags |= b2_enlargeBounds;
            }

            shapeId = shape.nextShapeId;
        }
    }

    // Push sensor hits on the the task context for serial processing.
    for (let i = 0; i < context.sensorCount; ++i) {
        // Skip any sensor hits that occurred after a solid hit
        if (context.sensorFractions[i] < context.fraction) {
            let hit = new b2SensorHit();
            hit.sensorId = context.sensorHits[i].sensorId;
            hit.visitorId = context.sensorHits[i].visitorId;
            taskContext.sensorHits.push(hit);
        }
    }
}

function b2FinalizeBodiesTask(startIndex: i32, endIndex: i32, threadIndex: u32, stepContext: b2StepContext): void {
    let world = stepContext.world!;

    assert(<i32>threadIndex < world.workerCount);

    let enableSleep = world.enableSleep;
    let states = stepContext.states;
    let sims = stepContext.sims;
    let timeStep = stepContext.dt;
    let invTimeStep = stepContext.inv_dt;

    let worldId = world.worldId;

    // The body move event array should already have the correct size
    assert(endIndex <= world.bodyMoveEvents.count);

    let taskContext = world.taskContexts.get(threadIndex);
    let enlargedSimBitSet = taskContext.enlargedSimBitSet;
    let awakeIslandBitSet = taskContext.awakeIslandBitSet;

    let enableContinuous = world.enableContinuous;

    let speculativeDistance = B2_SPECULATIVE_DISTANCE();
    let aabbMargin = B2_AABB_MARGIN();

    assert(startIndex <= endIndex);

    for (let simIndex = startIndex; simIndex < endIndex; ++simIndex) {
        let state = unchecked(states[simIndex])!;
        let sim = unchecked(sims[simIndex])!;

        if ((state.flags & b2_lockLinearX) != 0) {
            state.linearVelocity.x = 0.0;
        }

        if ((state.flags & b2_lockLinearY) != 0) {
            state.linearVelocity.y = 0.0;
        }

        if ((state.flags & b2_lockAngularZ) != 0) {
            state.angularVelocity = 0.0;
        }

        let v = state.linearVelocity;
        let w = state.angularVelocity;

        assert(b2IsValidVec2(v));
        assert(b2IsValidFloat(w));

        sim.center = b2Add(sim.center, state.deltaPosition);
        sim.transform.q = b2NormalizeRot(b2MulRot(state.deltaRotation, sim.transform.q));

        // Use the velocity of the farthest point on the body to account for rotation.
        let maxVelocity = b2Length(v) + b2AbsFloat(w) * sim.maxExtent;

        // Sleep needs to observe position correction as well as true velocity.
        let maxDeltaPosition = b2Length(state.deltaPosition) + b2AbsFloat(state.deltaRotation.s) * sim.maxExtent;

        // Position correction is not as important for sleep as true velocity.
        let positionSleepFactor: f32 = 0.5;

        let sleepVelocity = b2MaxFloat(maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition);

        // reset state deltas
        state.deltaPosition = b2Vec2_zero();
        state.deltaRotation = b2Rot_identity();

        sim.transform.p = b2Sub(sim.center, b2RotateVector(sim.transform.q, sim.localCenter));

        // cache miss here, however I need the shape list below
        let body = world.bodies.get(sim.bodyId);
        body.bodyMoveIndex = simIndex;
        let moveEvent = world.bodyMoveEvents.get(simIndex);
        moveEvent.transform = sim.transform.clone();
        moveEvent.bodyId = new b2BodyIdLocal(sim.bodyId + 1, worldId, body.generation);
        moveEvent.userData = body.userData;
        moveEvent.fellAsleep = false;

        // reset applied force and torque
        sim.force = b2Vec2_zero();
        sim.torque = 0.0;

        // If you hit this then it means you deferred mass computation but never called b2Body_ApplyMassFromShapes
        assert((body.flags & b2_dirtyMass) == 0);

        body.flags &= ~(b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact);
        body.flags |= (sim.flags & (b2_isSpeedCapped | b2_hadTimeOfImpact));
        sim.flags &= ~(b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact);

        if (enableSleep == false || body.enableSleep == false || sleepVelocity > body.sleepThreshold) {
            // Body is not sleepy
            body.sleepTime = 0.0;

            if (body.type == b2_dynamicBody && enableContinuous && maxVelocity * timeStep > <f32>0.5 * sim.minExtent) {
                // This flag is only retained for debug draw
                sim.flags |= b2_isFast;

                // Store in fast array for the continuous collision stage
                if ((sim.flags & b2_isBullet) != 0) {
                    b2IntArray_Push(stepContext.bulletBodies, simIndex);
                } else {
                    b2SolveContinuous(world, simIndex, taskContext);
                }
            } else {
                // Body is safe to advance
                sim.center0 = sim.center.clone();
                sim.rotation0 = sim.transform.q.clone();
            }
        } else {
            // Body is safe to advance and is falling asleep
            sim.center0 = sim.center.clone();
            sim.rotation0 = sim.transform.q.clone();
            body.sleepTime += timeStep;
        }

        // Any single body in an island can keep it awake
        let island = world.islands.get(body.islandId);
        if (body.sleepTime < B2_TIME_TO_SLEEP) {
            // keep island awake
            let islandIndex = island.localIndex;
            b2SetBit(awakeIslandBitSet, islandIndex);
        } else if (island.constraintRemoveCount > 0) {
            // body wants to sleep but its island needs splitting first
            if (body.sleepTime > taskContext.splitSleepTime) {
                // pick the sleepiest candidate
                taskContext.splitIslandId = body.islandId;
                taskContext.splitSleepTime = body.sleepTime;
            }
        }

        // Update shapes AABBs
        let transform = sim.transform;
        let isFast = (sim.flags & b2_isFast) != 0;
        let shapeId = body.headShapeId;
        while (shapeId != B2_NULL_INDEX) {
            let shape = world.shapes.get(shapeId);

            if (isFast) {
                // For fast non-bullet bodies the AABB has already been updated in b2SolveContinuous
                // For fast bullet bodies the AABB will be updated at a later stage

                // Add to enlarged shapes regardless of AABB changes.
                // Bit-set to keep the move array sorted
                b2SetBit(enlargedSimBitSet, simIndex);
            } else {
                let aabb = b2ComputeShapeAABB(shape, transform);
                aabb.lowerBound.x -= speculativeDistance;
                aabb.lowerBound.y -= speculativeDistance;
                aabb.upperBound.x += speculativeDistance;
                aabb.upperBound.y += speculativeDistance;
                shape.aabb = aabb;

                assert(shape.enlargedAABB == false);

                if (b2AABB_Contains(shape.fatAABB, aabb) == false) {
                    let fatAABB = new b2AABB();
                    fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
                    fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
                    fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
                    fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
                    shape.fatAABB = fatAABB;

                    shape.enlargedAABB = true;

                    // Bit-set to keep the move array sorted
                    b2SetBit(enlargedSimBitSet, simIndex);
                }
            }

            shapeId = shape.nextShapeId;
        }
    }
}

function b2ExecuteBlock(stage: b2SolverStage, context: b2StepContext, block: b2SolverBlock, workerIndex: i32): void {
    let stageType = stage.type;
    let blockType = <i32>block.blockType;
    let startIndex = block.startIndex;
    let endIndex = startIndex + <i32>block.count;

    switch (stageType) {
        case b2_stagePrepareJoints:
            b2PrepareJointsTask(startIndex, endIndex, context);
            break;

        case b2_stagePrepareContacts:
            b2PrepareContactsTask(startIndex, endIndex, context);
            break;

        case b2_stageIntegrateVelocities:
            b2IntegrateVelocitiesTask(startIndex, endIndex, context);
            break;

        case b2_stageWarmStart:
            if (blockType == b2_graphContactBlock) {
                b2WarmStartContactsTask(startIndex, endIndex, context, stage.colorIndex);
            } else if (blockType == b2_graphJointBlock) {
                b2WarmStartJointsTask(startIndex, endIndex, context, stage.colorIndex);
            }
            break;

        case b2_stageSolve:
            if (blockType == b2_graphContactBlock) {
                b2SolveContactsTask(startIndex, endIndex, context, stage.colorIndex, true);
            } else if (blockType == b2_graphJointBlock) {
                b2SolveJointsTask(startIndex, endIndex, context, stage.colorIndex, true, workerIndex);
            }
            break;

        case b2_stageIntegratePositions:
            b2IntegratePositionsTask(startIndex, endIndex, context);
            break;

        case b2_stageRelax:
            if (blockType == b2_graphContactBlock) {
                b2SolveContactsTask(startIndex, endIndex, context, stage.colorIndex, false);
            } else if (blockType == b2_graphJointBlock) {
                b2SolveJointsTask(startIndex, endIndex, context, stage.colorIndex, false, workerIndex);
            }
            break;

        case b2_stageRestitution:
            if (blockType == b2_graphContactBlock) {
                b2ApplyRestitutionTask(startIndex, endIndex, context, stage.colorIndex);
            }
            break;

        case b2_stageStoreImpulses:
            b2StoreImpulsesTask(startIndex, endIndex, context);
            break;

        default:
            break;
    }
}

function GetWorkerStartIndex(workerIndex: i32, blockCount: i32, workerCount: i32): i32 {
    if (blockCount <= workerCount) {
        return workerIndex < blockCount ? workerIndex : B2_NULL_INDEX;
    }

    let blocksPerWorker = blockCount / workerCount;
    let remainder = blockCount - blocksPerWorker * workerCount;
    return blocksPerWorker * workerIndex + b2MinInt(remainder, workerIndex);
}

// Single threaded: the CAS on the block sync index reduces to a plain
// compare and set; worker 0 claims every block in forward order.
function b2ExecuteStage(stage: b2SolverStage, context: b2StepContext, previousSyncIndex: i32, syncIndex: i32,
    workerIndex: i32): void {
    let completedCount = 0;
    let blocks = stage.blocks;
    let blockCount = stage.blockCount;

    let startIndex = GetWorkerStartIndex(workerIndex, blockCount, context.workerCount);
    if (startIndex == B2_NULL_INDEX) {
        return;
    }

    assert(0 <= startIndex && startIndex < blockCount);

    let blockIndex = startIndex;

    while (blocks[blockIndex].syncIndex == previousSyncIndex) {
        blocks[blockIndex].syncIndex = syncIndex;

        assert(completedCount < blockCount);

        b2ExecuteBlock(stage, context, blocks[blockIndex], workerIndex);

        completedCount += 1;
        blockIndex += 1;
        if (blockIndex >= blockCount) {
            // Keep looking for work
            blockIndex = 0;
        }
    }

    // Search backwards for blocks
    blockIndex = startIndex - 1;
    while (true) {
        if (blockIndex < 0) {
            blockIndex = blockCount - 1;
        }

        if (blocks[blockIndex].syncIndex != previousSyncIndex) {
            break;
        }

        blocks[blockIndex].syncIndex = syncIndex;

        b2ExecuteBlock(stage, context, blocks[blockIndex], workerIndex);
        completedCount += 1;
        blockIndex -= 1;
    }

    stage.completionCount += completedCount;
}

function b2ExecuteMainStage(stage: b2SolverStage, context: b2StepContext, syncBits: u32): void {
    let blockCount = stage.blockCount;
    if (blockCount == 0) {
        return;
    }

    let workerIndex = 0;

    if (blockCount == 1) {
        b2ExecuteBlock(stage, context, stage.blocks[0], workerIndex);
    } else {
        context.atomicSyncBits = syncBits;

        let syncIndex = <i32>((syncBits >> 16) & 0xFFFF);
        assert(syncIndex > 0);
        let previousSyncIndex = syncIndex - 1;

        b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

        assert(stage.completionCount == blockCount);
        stage.completionCount = 0;
    }
}

// Main solver task body for worker 0 (the only worker in this port).
function b2SolverTask(context: b2StepContext): void {
    let workerIndex = 0;
    let activeColorCount = context.activeColorCount;
    let stages = context.stages;

    // Main thread synchronizes the workers and does work itself.
    let bodySyncIndex = 1;
    let stageIndex = 0;

    // This stage loops over all awake joints
    let jointSyncIndex: u32 = 1;
    let syncBits: u32 = (jointSyncIndex << 16) | <u32>stageIndex;
    assert(stages[stageIndex].type == b2_stagePrepareJoints);
    b2ExecuteMainStage(stages[stageIndex], context, syncBits);
    stageIndex += 1;
    jointSyncIndex += 1;

    // This stage loops over all contact constraints
    let contactSyncIndex: u32 = 1;
    syncBits = (contactSyncIndex << 16) | <u32>stageIndex;
    assert(stages[stageIndex].type == b2_stagePrepareContacts);
    b2ExecuteMainStage(stages[stageIndex], context, syncBits);
    stageIndex += 1;
    contactSyncIndex += 1;

    let graphSyncIndex: u32 = 1;

    // Single-threaded overflow work. These constraints don't fit in the graph coloring.
    b2PrepareOverflowJoints(context);
    b2PrepareOverflowContacts(context);

    let subStepCount = context.subStepCount;
    for (let i = 0; i < subStepCount; ++i) {
        // stage index restarted each iteration
        let iterStageIndex = stageIndex;

        // integrate velocities
        syncBits = (<u32>bodySyncIndex << 16) | <u32>iterStageIndex;
        assert(stages[iterStageIndex].type == b2_stageIntegrateVelocities);
        b2ExecuteMainStage(stages[iterStageIndex], context, syncBits);
        iterStageIndex += 1;
        bodySyncIndex += 1;

        // warm start constraints
        b2WarmStartOverflowJoints(context);
        b2WarmStartOverflowContacts(context);

        for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
            syncBits = (graphSyncIndex << 16) | <u32>iterStageIndex;
            assert(stages[iterStageIndex].type == b2_stageWarmStart);
            b2ExecuteMainStage(stages[iterStageIndex], context, syncBits);
            iterStageIndex += 1;
        }
        graphSyncIndex += 1;

        // solve constraints
        let useBias = true;

        for (let j = 0; j < ITERATIONS; ++j) {
            // Overflow constraints have lower priority
            b2SolveOverflowJoints(context, useBias);
            b2SolveOverflowContacts(context, useBias);

            for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
                syncBits = (graphSyncIndex << 16) | <u32>iterStageIndex;
                assert(stages[iterStageIndex].type == b2_stageSolve);
                b2ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                iterStageIndex += 1;
            }
            graphSyncIndex += 1;
        }

        // integrate positions
        assert(stages[iterStageIndex].type == b2_stageIntegratePositions);
        syncBits = (<u32>bodySyncIndex << 16) | <u32>iterStageIndex;
        b2ExecuteMainStage(stages[iterStageIndex], context, syncBits);
        iterStageIndex += 1;
        bodySyncIndex += 1;

        // relax constraints
        useBias = false;
        for (let j = 0; j < RELAX_ITERATIONS; ++j) {
            b2SolveOverflowJoints(context, useBias);
            b2SolveOverflowContacts(context, useBias);

            for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
                syncBits = (graphSyncIndex << 16) | <u32>iterStageIndex;
                assert(stages[iterStageIndex].type == b2_stageRelax);
                b2ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                iterStageIndex += 1;
            }
            graphSyncIndex += 1;
        }
    }

    // advance the stage according to the sub-stepping tasks just completed
    stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

    // Restitution
    {
        b2ApplyOverflowRestitution(context);

        let iterStageIndex = stageIndex;
        for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
            syncBits = (graphSyncIndex << 16) | <u32>iterStageIndex;
            assert(stages[iterStageIndex].type == b2_stageRestitution);
            b2ExecuteMainStage(stages[iterStageIndex], context, syncBits);
            iterStageIndex += 1;
        }
        stageIndex += activeColorCount;
    }

    b2StoreOverflowImpulses(context);

    syncBits = (contactSyncIndex << 16) | <u32>stageIndex;
    assert(stages[stageIndex].type == b2_stageStoreImpulses);
    b2ExecuteMainStage(stages[stageIndex], context, syncBits);

    // Signal workers to finish
    context.atomicSyncBits = 0xffffffff;

    assert(stageIndex + 1 == context.stageCount);
}

function b2BulletBodyTask(startIndex: i32, endIndex: i32, threadIndex: u32, stepContext: b2StepContext): void {
    let taskContext = stepContext.world!.taskContexts.get(threadIndex);

    assert(startIndex <= endIndex);

    for (let i = startIndex; i < endIndex; ++i) {
        let simIndex = stepContext.bulletBodies.data[i];
        b2SolveContinuous(stepContext.world!, simIndex, taskContext);
    }
}

// Solve with graph coloring
export function b2Solve(world: b2World, stepContext: b2StepContext): void {
    world.stepIndex += 1;

    // Are there any awake bodies? This scenario should not be important for profiling.
    let awakeSet = world.solverSets.get(b2_awakeSet);
    let awakeBodyCount = awakeSet.bodySims.count;
    if (awakeBodyCount == 0) {
        // Nothing to simulate (tree rebuild already ran inline in b2Collide).
        return;
    }

    // Solve constraints using graph coloring
    {
        // Prepare buffers for bullets
        stepContext.bulletBodies = b2IntArray_Create(awakeBodyCount);

        let graph = world.constraintGraph;
        let colors = graph.colors;

        stepContext.sims = awakeSet.bodySims.data;
        stepContext.states = awakeSet.bodyStates.data;

        // count contacts, joints, and colors
        let awakeJointCount = 0;
        let activeColorCount = 0;
        for (let i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i) {
            let perColorContactCount = colors[i].contactSims.count;
            let perColorJointCount = colors[i].jointSims.count;
            let occupancyCount = perColorContactCount + perColorJointCount;
            activeColorCount += occupancyCount > 0 ? 1 : 0;
            awakeJointCount += perColorJointCount;
        }

        // prepare for move events
        world.bodyMoveEvents.resize(awakeBodyCount);
        for (let i = 0; i < awakeBodyCount; ++i) {
            if (world.bodyMoveEvents.data[i] == null) {
                world.bodyMoveEvents.data[i] = new b2BodyMoveEvent();
            }
        }

        let workerCount = world.workerCount;
        const blocksPerWorker = 4;
        let maxBlockCount = blocksPerWorker * workerCount;

        // Configure blocks for tasks that parallel-for bodies
        let bodyBlockSize = 1 << 5;
        let bodyBlockCount: i32;
        if (awakeBodyCount > bodyBlockSize * maxBlockCount) {
            // Too many blocks, increase block size
            bodyBlockSize = awakeBodyCount / maxBlockCount;
            bodyBlockCount = maxBlockCount;
        } else {
            bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1;
        }

        // Configure blocks for tasks parallel-for each active graph color
        let activeColorIndices = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);

        let colorContactCounts = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
        let colorContactBlockSizes = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
        let colorContactBlockCounts = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);

        let colorJointCounts = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
        let colorJointBlockSizes = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);
        let colorJointBlockCounts = new StaticArray<i32>(B2_GRAPH_COLOR_COUNT);

        let graphBlockCount = 0;

        // c is the active color index
        let simdContactCount = 0;
        let c = 0;
        for (let i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i) {
            let colorContactCount = colors[i].contactSims.count;
            let colorJointCount = colors[i].jointSims.count;

            if (colorContactCount + colorJointCount > 0) {
                activeColorIndices[c] = i;

                // 4-way SIMD
                let colorContactCountSIMD = colorContactCount > 0 ? ((colorContactCount - 1) >> B2_SIMD_SHIFT) + 1 : 0;

                colorContactCounts[c] = colorContactCountSIMD;

                // determine the number of contact work blocks for this color
                if (colorContactCountSIMD > blocksPerWorker * maxBlockCount) {
                    // too many contact blocks per worker, so make bigger blocks
                    colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
                    colorContactBlockCounts[c] = maxBlockCount;
                } else if (colorContactCountSIMD > 0) {
                    colorContactBlockSizes[c] = blocksPerWorker;
                    colorContactBlockCounts[c] = ((colorContactCountSIMD - 1) / blocksPerWorker) + 1;
                } else {
                    colorContactBlockSizes[c] = 0;
                    colorContactBlockCounts[c] = 0;
                }

                colorJointCounts[c] = colorJointCount;

                // determine number of joint work blocks for this color
                if (colorJointCount > blocksPerWorker * maxBlockCount) {
                    // too many joint blocks
                    colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
                    colorJointBlockCounts[c] = maxBlockCount;
                } else if (colorJointCount > 0) {
                    colorJointBlockSizes[c] = blocksPerWorker;
                    colorJointBlockCounts[c] = ((colorJointCount - 1) / 4) + 1;
                } else {
                    colorJointBlockSizes[c] = 0;
                    colorJointBlockCounts[c] = 0;
                }

                graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
                simdContactCount += colorContactCountSIMD;
                c += 1;
            }
        }
        activeColorCount = c;

        // Gather contact pointers for easy parallel-for traversal. Some may be null due to SIMD remainders.
        let contacts = new Array<b2ContactSim | null>(B2_SIMD_WIDTH * simdContactCount);

        // Gather joint pointers for easy parallel-for traversal.
        let joints = new Array<b2JointSim | null>(awakeJointCount);

        let simdContactConstraints = b2AllocSimdConstraints(simdContactCount);

        let overflowContactCount = colors[B2_OVERFLOW_INDEX].contactSims.count;
        let overflowContactConstraints = b2AllocOverflowConstraints(overflowContactCount);

        graph.colors[B2_OVERFLOW_INDEX].overflowConstraints = overflowContactConstraints;

        // Distribute transient constraints to each graph color and build flat arrays of contact and joint pointers
        {
            let contactBase = 0;
            let jointBase = 0;
            for (let i = 0; i < activeColorCount; ++i) {
                let j = activeColorIndices[i];
                let color = colors[j];

                let colorContactCount = color.contactSims.count;

                if (colorContactCount == 0) {
                    color.simdConstraints = null;
                } else {
                    // C slices the shared allocation with pointer arithmetic;
                    // copy references so the color slice aliases the same
                    // constraint objects.
                    let colorContactCountSIMD = ((colorContactCount - 1) >> B2_SIMD_SHIFT) + 1;
                    let slice = new StaticArray<b2SimdConstraint>(colorContactCountSIMD);
                    for (let k = 0; k < colorContactCountSIMD; ++k) {
                        slice[k] = simdContactConstraints[contactBase + k];
                    }
                    color.simdConstraints = slice;

                    for (let k = 0; k < colorContactCount; ++k) {
                        contacts[B2_SIMD_WIDTH * contactBase + k] = color.contactSims.get(k);
                    }

                    // remainder
                    for (let k = colorContactCount; k < B2_SIMD_WIDTH * colorContactCountSIMD; ++k) {
                        contacts[B2_SIMD_WIDTH * contactBase + k] = null;
                    }

                    contactBase += colorContactCountSIMD;
                }

                let colorJointCount = color.jointSims.count;
                for (let k = 0; k < colorJointCount; ++k) {
                    joints[jointBase + k] = color.jointSims.get(k);
                }
                jointBase += colorJointCount;
            }

            assert(contactBase == simdContactCount);
            assert(jointBase == awakeJointCount);
        }

        // Define work blocks for preparing contacts and storing contact impulses
        let contactBlockSize = blocksPerWorker;
        let contactBlockCount = simdContactCount > 0 ? ((simdContactCount - 1) / blocksPerWorker) + 1 : 0;
        if (simdContactCount > contactBlockSize * maxBlockCount) {
            // Too many blocks, increase block size
            contactBlockSize = simdContactCount / maxBlockCount;
            contactBlockCount = maxBlockCount;
        }

        // Define work blocks for preparing joints
        let jointBlockSize = blocksPerWorker;
        let jointBlockCount = awakeJointCount > 0 ? ((awakeJointCount - 1) / blocksPerWorker) + 1 : 0;
        if (awakeJointCount > jointBlockSize * maxBlockCount) {
            // Too many blocks, increase block size
            jointBlockSize = awakeJointCount / maxBlockCount;
            jointBlockCount = maxBlockCount;
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

        let stages = new Array<b2SolverStage>(stageCount);
        for (let i = 0; i < stageCount; ++i) {
            stages[i] = new b2SolverStage();
        }

        let bodyBlocks = makeBlocks(bodyBlockCount);
        let contactBlocks = makeBlocks(contactBlockCount);
        let jointBlocks = makeBlocks(jointBlockCount);
        let graphBlocks = makeBlocks(graphBlockCount);

        // Split an awake island. This may be expensive and this is a safe place to put it.
        if (world.splitIslandId != B2_NULL_INDEX) {
            b2SplitIslandTask(0, 1, 0, world);
            world.taskCount += 1;
        }

        // Prepare body work blocks
        for (let i = 0; i < bodyBlockCount; ++i) {
            let block = bodyBlocks[i];
            block.startIndex = i * bodyBlockSize;
            block.count = <i16>bodyBlockSize;
            block.blockType = <i16>b2_bodyBlock;
            block.syncIndex = 0;
        }
        bodyBlocks[bodyBlockCount - 1].count = <i16>(awakeBodyCount - (bodyBlockCount - 1) * bodyBlockSize);

        // Prepare joint work blocks
        for (let i = 0; i < jointBlockCount; ++i) {
            let block = jointBlocks[i];
            block.startIndex = i * jointBlockSize;
            block.count = <i16>jointBlockSize;
            block.blockType = <i16>b2_jointBlock;
            block.syncIndex = 0;
        }

        if (jointBlockCount > 0) {
            jointBlocks[jointBlockCount - 1].count = <i16>(awakeJointCount - (jointBlockCount - 1) * jointBlockSize);
        }

        // Prepare contact work blocks
        for (let i = 0; i < contactBlockCount; ++i) {
            let block = contactBlocks[i];
            block.startIndex = i * contactBlockSize;
            block.count = <i16>contactBlockSize;
            block.blockType = <i16>b2_contactBlock;
            block.syncIndex = 0;
        }

        if (contactBlockCount > 0) {
            contactBlocks[contactBlockCount - 1].count =
                <i16>(simdContactCount - (contactBlockCount - 1) * contactBlockSize);
        }

        // Prepare graph work blocks
        let graphColorBlocks = new Array<Array<b2SolverBlock>>(B2_GRAPH_COLOR_COUNT);
        let baseGraphBlock = 0;

        for (let i = 0; i < activeColorCount; ++i) {
            let colorBlockStart = baseGraphBlock;

            let colorJointBlockCount = colorJointBlockCounts[i];
            let colorJointBlockSize = colorJointBlockSizes[i];
            for (let j = 0; j < colorJointBlockCount; ++j) {
                let block = graphBlocks[baseGraphBlock + j];
                block.startIndex = j * colorJointBlockSize;
                block.count = <i16>colorJointBlockSize;
                block.blockType = <i16>b2_graphJointBlock;
                block.syncIndex = 0;
            }

            if (colorJointBlockCount > 0) {
                graphBlocks[baseGraphBlock + colorJointBlockCount - 1].count =
                    <i16>(colorJointCounts[i] - (colorJointBlockCount - 1) * colorJointBlockSize);
                baseGraphBlock += colorJointBlockCount;
            }

            let colorContactBlockCount = colorContactBlockCounts[i];
            let colorContactBlockSize = colorContactBlockSizes[i];
            for (let j = 0; j < colorContactBlockCount; ++j) {
                let block = graphBlocks[baseGraphBlock + j];
                block.startIndex = j * colorContactBlockSize;
                block.count = <i16>colorContactBlockSize;
                block.blockType = <i16>b2_graphContactBlock;
                block.syncIndex = 0;
            }

            if (colorContactBlockCount > 0) {
                graphBlocks[baseGraphBlock + colorContactBlockCount - 1].count =
                    <i16>(colorContactCounts[i] - (colorContactBlockCount - 1) * colorContactBlockSize);
                baseGraphBlock += colorContactBlockCount;
            }

            // Slice of graphBlocks for this color (shared block objects)
            let span = new Array<b2SolverBlock>(baseGraphBlock - colorBlockStart);
            for (let j = colorBlockStart; j < baseGraphBlock; ++j) {
                span[j - colorBlockStart] = graphBlocks[j];
            }
            graphColorBlocks[i] = span;
        }

        assert(baseGraphBlock == graphBlockCount);

        let stageIdx = 0;

        // Prepare joints
        {
            let stage = stages[stageIdx];
            stage.type = b2_stagePrepareJoints;
            stage.blocks = jointBlocks;
            stage.blockCount = jointBlockCount;
            stage.colorIndex = -1;
            stage.completionCount = 0;
            stageIdx += 1;
        }

        // Prepare contacts
        {
            let stage = stages[stageIdx];
            stage.type = b2_stagePrepareContacts;
            stage.blocks = contactBlocks;
            stage.blockCount = contactBlockCount;
            stage.colorIndex = -1;
            stage.completionCount = 0;
            stageIdx += 1;
        }

        // Integrate velocities
        {
            let stage = stages[stageIdx];
            stage.type = b2_stageIntegrateVelocities;
            stage.blocks = bodyBlocks;
            stage.blockCount = bodyBlockCount;
            stage.colorIndex = -1;
            stage.completionCount = 0;
            stageIdx += 1;
        }

        // Warm start
        for (let i = 0; i < activeColorCount; ++i) {
            let stage = stages[stageIdx];
            stage.type = b2_stageWarmStart;
            stage.blocks = graphColorBlocks[i];
            stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
            stage.colorIndex = activeColorIndices[i];
            stage.completionCount = 0;
            stageIdx += 1;
        }

        // Solve graph
        for (let j = 0; j < ITERATIONS; ++j) {
            for (let i = 0; i < activeColorCount; ++i) {
                let stage = stages[stageIdx];
                stage.type = b2_stageSolve;
                stage.blocks = graphColorBlocks[i];
                stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                stage.colorIndex = activeColorIndices[i];
                stage.completionCount = 0;
                stageIdx += 1;
            }
        }

        // Integrate positions
        {
            let stage = stages[stageIdx];
            stage.type = b2_stageIntegratePositions;
            stage.blocks = bodyBlocks;
            stage.blockCount = bodyBlockCount;
            stage.colorIndex = -1;
            stage.completionCount = 0;
            stageIdx += 1;
        }

        // Relax constraints
        for (let j = 0; j < RELAX_ITERATIONS; ++j) {
            for (let i = 0; i < activeColorCount; ++i) {
                let stage = stages[stageIdx];
                stage.type = b2_stageRelax;
                stage.blocks = graphColorBlocks[i];
                stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                stage.colorIndex = activeColorIndices[i];
                stage.completionCount = 0;
                stageIdx += 1;
            }
        }

        // Restitution
        // Note: joint blocks mixed in, could have joint limit restitution
        for (let i = 0; i < activeColorCount; ++i) {
            let stage = stages[stageIdx];
            stage.type = b2_stageRestitution;
            stage.blocks = graphColorBlocks[i];
            stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
            stage.colorIndex = activeColorIndices[i];
            stage.completionCount = 0;
            stageIdx += 1;
        }

        // Store impulses
        {
            let stage = stages[stageIdx];
            stage.type = b2_stageStoreImpulses;
            stage.blocks = contactBlocks;
            stage.blockCount = contactBlockCount;
            stage.colorIndex = -1;
            stage.completionCount = 0;
            stageIdx += 1;
        }

        assert(stageIdx == stageCount);

        stepContext.graph = graph;
        stepContext.joints = joints;
        stepContext.contacts = contacts;
        stepContext.simdContactConstraints = simdContactConstraints;
        stepContext.activeColorCount = activeColorCount;
        stepContext.workerCount = workerCount;
        stepContext.stageCount = stageCount;
        stepContext.stages = stages;
        stepContext.atomicSyncBits = 0;

        // Single worker runs the whole stage pipeline inline.
        let jointIdCapacity = b2GetIdCapacity(world.jointIdPool);
        for (let i = 0; i < workerCount; ++i) {
            let taskContext = world.taskContexts.get(i);
            b2SetBitCountAndClear(taskContext.jointStateBitSet, jointIdCapacity);
        }

        b2SolverTask(stepContext);
        world.taskCount += 1;

        world.splitIslandId = B2_NULL_INDEX;

        // Prepare contact, enlarged body, and island bit sets used in body finalization.
        let awakeIslandCount = awakeSet.islandSims.count;
        for (let i = 0; i < world.workerCount; ++i) {
            let taskContext = world.taskContexts.get(i);
            taskContext.sensorHits.clear();
            b2SetBitCountAndClear(taskContext.enlargedSimBitSet, awakeBodyCount);
            b2SetBitCountAndClear(taskContext.awakeIslandBitSet, awakeIslandCount);
            taskContext.splitIslandId = B2_NULL_INDEX;
            taskContext.splitSleepTime = 0.0;
        }

        // Finalize bodies. Must happen after the constraint solver and after island splitting.
        b2FinalizeBodiesTask(0, awakeBodyCount, 0, stepContext);
        world.taskCount += 1;

        // release transient references like C's arena frees
        graph.colors[B2_OVERFLOW_INDEX].overflowConstraints = null;
        for (let i = 0; i < activeColorCount; ++i) {
            let color = colors[activeColorIndices[i]];
            color.simdConstraints = null;
        }
        stepContext.simdContactConstraints = null;
        stepContext.joints = new Array<b2JointSim | null>();
        stepContext.contacts = new Array<b2ContactSim | null>();
    }

    // Report joint events
    {
        // Gather bits for all joints that have force/torque events
        let jointStateBitSet = world.taskContexts.get(0).jointStateBitSet;
        for (let i = 1; i < world.workerCount; ++i) {
            b2InPlaceUnion(jointStateBitSet, world.taskContexts.get(i).jointStateBitSet);
        }

        {
            let wordCount = jointStateBitSet.blockCount;
            let bits = jointStateBitSet.bits;

            let worldIndex0 = world.worldId;

            for (let k: u32 = 0; k < wordCount; ++k) {
                let word = bits[k];
                while (word != 0) {
                    let ctz = b2CTZ64(word);
                    let jointId = <i32>(64 * k + ctz);

                    let joint = world.joints.get(jointId);

                    assert(joint.setIndex == b2_awakeSet);

                    let event = new b2JointEvent();
                    event.jointId = new b2JointIdLocal(jointId + 1, worldIndex0, joint.generation);
                    event.userData = joint.userData;

                    world.jointEvents.push(event);

                    // Clear the smallest set bit
                    word = word & (word - 1);
                }
            }
        }
    }

    // Report hit events
    {
        assert(world.contactHitEvents.count == 0);

        let threshold = world.hitEventThreshold;
        let colors = world.constraintGraph.colors;
        for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
            let color = colors[i];
            let contactCount = color.contactSims.count;
            for (let j = 0; j < contactCount; ++j) {
                let contactSim = color.contactSims.get(j);
                if ((contactSim.simFlags & b2_simEnableHitEvent) == 0) {
                    continue;
                }

                let event = new b2ContactHitEvent();
                event.approachSpeed = threshold;

                let hit = false;
                let pointCount = contactSim.manifold.pointCount;
                for (let k = 0; k < pointCount; ++k) {
                    let mp = contactSim.manifold.points[k];
                    let approachSpeed = -mp.normalVelocity;

                    // Need to check total impulse because the point may be speculative and not colliding
                    if (approachSpeed > event.approachSpeed && mp.totalNormalImpulse > 0.0) {
                        event.approachSpeed = approachSpeed;
                        event.point = mp.point.clone();
                        hit = true;
                    }
                }

                if (hit == true) {
                    event.normal = contactSim.manifold.normal.clone();

                    let shapeA = world.shapes.get(contactSim.shapeIdA);
                    let shapeB = world.shapes.get(contactSim.shapeIdB);

                    event.shapeIdA = new b2ShapeId(shapeA.id + 1, world.worldId, shapeA.generation);
                    event.shapeIdB = new b2ShapeId(shapeB.id + 1, world.worldId, shapeB.generation);

                    let contact = world.contacts.get(contactSim.contactId);

                    event.contactId = new b2ContactId(contact.contactId + 1, world.worldId, contact.generation);

                    world.contactHitEvents.push(event);
                }
            }
        }
    }

    {
        // Gather bits for all sim bodies that have enlarged AABBs
        let enlargedBodyBitSet = world.taskContexts.get(0).enlargedSimBitSet;
        for (let i = 1; i < world.workerCount; ++i) {
            b2InPlaceUnion(enlargedBodyBitSet, world.taskContexts.get(i).enlargedSimBitSet);
        }

        // Enlarge broad-phase proxies and build move array
        {
            let broadPhase = world.broadPhase;
            let wordCount = enlargedBodyBitSet.blockCount;
            let bits = enlargedBodyBitSet.bits;

            for (let k: u32 = 0; k < wordCount; ++k) {
                let word = bits[k];
                while (word != 0) {
                    let ctz = b2CTZ64(word);
                    let bodySimIndex = <i32>(64 * k + ctz);

                    let bodySim = awakeSet.bodySims.get(bodySimIndex);

                    let body = world.bodies.get(bodySim.bodyId);

                    let shapeId = body.headShapeId;
                    if ((bodySim.flags & (b2_isBullet | b2_isFast)) == (b2_isBullet | b2_isFast)) {
                        // Fast bullet bodies don't have their final AABB yet
                        while (shapeId != B2_NULL_INDEX) {
                            let shape = world.shapes.get(shapeId);

                            // Update the move array here for determinism because bullets are processed
                            // below in non-deterministic order.
                            b2BufferMove(broadPhase, shape.proxyKey);

                            shapeId = shape.nextShapeId;
                        }
                    } else {
                        while (shapeId != B2_NULL_INDEX) {
                            let shape = world.shapes.get(shapeId);

                            // The AABB may not have been enlarged, despite the body being flagged as enlarged.
                            if (shape.enlargedAABB) {
                                b2BroadPhase_EnlargeProxy(broadPhase, shape.proxyKey, shape.fatAABB);
                                shape.enlargedAABB = false;
                            }

                            shapeId = shape.nextShapeId;
                        }
                    }

                    // Clear the smallest set bit
                    word = word & (word - 1);
                }
            }
        }
    }

    let bulletBodyCount = stepContext.bulletBodies.count;
    if (bulletBodyCount > 0) {
        // Fast bullet bodies
        b2BulletBodyTask(0, bulletBodyCount, 0, stepContext);
        world.taskCount += 1;

        // Serially enlarge broad-phase proxies for bullet shapes
        let broadPhase = world.broadPhase;
        let dynamicTree = broadPhase.trees[b2_dynamicBody];

        let bulletBodySimIndices = stepContext.bulletBodies;

        // This loop has non-deterministic order in C but it shouldn't affect the result
        for (let i = 0; i < bulletBodyCount; ++i) {
            let bulletBodySim = awakeSet.bodySims.get(bulletBodySimIndices.data[i]);
            if ((bulletBodySim.flags & b2_enlargeBounds) == 0) {
                continue;
            }

            // Clear flag
            bulletBodySim.flags &= ~b2_enlargeBounds;

            let bodyId = bulletBodySim.bodyId;
            assert(0 <= bodyId && bodyId < world.bodies.count);
            let bulletBody = world.bodies.get(bodyId);

            let shapeId = bulletBody.headShapeId;
            while (shapeId != B2_NULL_INDEX) {
                let shape = world.shapes.get(shapeId);
                if (shape.enlargedAABB == false) {
                    shapeId = shape.nextShapeId;
                    continue;
                }

                // Clear flag
                shape.enlargedAABB = false;

                let proxyKey = shape.proxyKey;
                let proxyId = B2_PROXY_ID(proxyKey);
                assert(B2_PROXY_TYPE(proxyKey) == b2_dynamicBody);

                // all fast bullet shapes should already be in the move buffer
                assert(b2ContainsKey(broadPhase.moveSet, <u64>(proxyKey + 1)));

                b2DynamicTree_EnlargeProxy(dynamicTree, proxyId, shape.fatAABB);

                shapeId = shape.nextShapeId;
            }
        }
    }

    stepContext.bulletBodies = b2IntArray_Create(0);

    // Report sensor hits. This may include bullet sensor hits.
    {
        let workerCount = world.workerCount;
        assert(workerCount == world.taskContexts.count);

        for (let i = 0; i < workerCount; ++i) {
            let taskContext = world.taskContexts.get(i);
            let hitCount = taskContext.sensorHits.count;

            for (let j = 0; j < hitCount; ++j) {
                let hit = taskContext.sensorHits.get(j);
                let visitor = world.shapes.get(hit.visitorId);
                let sensorShape = world.shapes.get(hit.sensorId);

                let sensor = world.sensors.get(sensorShape.sensorIndex);
                let shapeRef = sensor.hits.add();
                shapeRef.shapeId = hit.visitorId;
                shapeRef.generation = visitor.generation;
            }
        }
    }

    // Island sleeping
    // This must be done last because putting islands to sleep invalidates the enlarged body bits.
    if (world.enableSleep == true) {
        // Collect split island candidate for the next time step. No need to split if sleeping is disabled.
        assert(world.splitIslandId == B2_NULL_INDEX);
        let splitSleepTimer: f32 = 0.0;
        for (let i = 0; i < world.workerCount; ++i) {
            let taskContext = world.taskContexts.get(i);
            if (taskContext.splitIslandId != B2_NULL_INDEX && taskContext.splitSleepTime >= splitSleepTimer) {
                assert(taskContext.splitSleepTime > 0.0);

                // Tie breaking for determinism. Largest island id wins.
                if (taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId) {
                    continue;
                }

                world.splitIslandId = taskContext.splitIslandId;
                splitSleepTimer = taskContext.splitSleepTime;
            }
        }

        let awakeIslandBitSet = world.taskContexts.get(0).awakeIslandBitSet;
        for (let i = 1; i < world.workerCount; ++i) {
            b2InPlaceUnion(awakeIslandBitSet, world.taskContexts.get(i).awakeIslandBitSet);
        }

        // Need to process in reverse because this moves islands to sleeping solver sets.
        let count = awakeSet.islandSims.count;
        for (let islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1) {
            if (b2GetBit(awakeIslandBitSet, islandIndex) == true) {
                // this island is still awake
                continue;
            }

            let island = awakeSet.islandSims.get(islandIndex);
            let islandId = island.islandId;

            b2TrySleepIsland(world, islandId);
        }

        b2ValidateSolverSets(world);
    }
}

function makeBlocks(count: i32): Array<b2SolverBlock> {
    let blocks = new Array<b2SolverBlock>(count);
    for (let i = 0; i < count; ++i) {
        blocks[i] = new b2SolverBlock();
    }
    return blocks;
}

import { b2BodyId as b2BodyIdLocal, b2JointId as b2JointIdLocal } from "./id";
import { B2_OVERFLOW_INDEX } from "./constraint_graph";
import { b2_chainSegmentShape as b2_chainSegmentShapeConst } from "./types";
