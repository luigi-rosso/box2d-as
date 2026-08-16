// Ported from box2d src/joint.h/.c. Joint def structs and defaults live here
// like C's joint.c. Debug draw omitted. The C union of per-type joint data
// becomes nullable fields with exactly one set.

import { B2_NULL_INDEX, B2_SECRET_COOKIE, b2_lengthUnitsPerMeter } from "./core";
import { B2_LINEAR_SLOP, B2_HUGE, B2_GRAPH_COLOR_COUNT } from "./constants";
import {
    b2Vec2, b2Rot, b2Transform, b2Vec2_zero, b2Sub, b2Add, b2Length, b2Lerp, b2Dot, b2AbsFloat,
    b2MaxFloat, b2MinFloat, b2ClampFloat, b2MaxInt, b2TransformPoint, b2RotateVector, b2LeftPerp,
    b2RelativeAngle, b2IsValidFloat, b2IsValidTransform, b2Mat22, FLT_MAX, B2_PI,
} from "./math_functions";
import { b2JointId, b2BodyId, b2WorldId } from "./id";
import {
    b2_distanceJoint, b2_filterJoint, b2_motorJoint, b2_prismaticJoint, b2_revoluteJoint,
    b2_weldJoint, b2_wheelJoint,
} from "./types";
import { b2AllocId, b2FreeId } from "./id_pool";
import {
    b2World, b2GetWorld, b2GetWorldFromId, b2GetWorldLocked, b2ValidateSolverSets,
    b2_awakeSet, b2_disabledSet, b2_staticSet, b2_firstSleepingSet,
} from "./physics_world";
import { b2Body, b2GetBodyFullId, b2GetBodyTransform, b2WakeBody, b2MakeBodyId } from "./body";
import { b2Contact, b2DestroyContact } from "./contact";
import { b2SolverSet, b2WakeSolverSet, b2MergeSolverSets } from "./solver_set";
import { b2CreateJointInGraph, b2RemoveJointFromGraph } from "./constraint_graph";
import { b2LinkJoint, b2UnlinkJoint } from "./island";
import { b2Softness, b2StepContext, b2MakeSoft } from "./solver";
import { b2BufferMove } from "./broad_phase";
import { b2_dynamicBody } from "./broad_phase";
import {
    b2PrepareDistanceJoint, b2WarmStartDistanceJoint, b2SolveDistanceJoint, b2GetDistanceJointForce,
} from "./distance_joint";
import {
    b2PrepareMotorJoint, b2WarmStartMotorJoint, b2SolveMotorJoint, b2GetMotorJointForce, b2GetMotorJointTorque,
} from "./motor_joint";
import {
    b2PreparePrismaticJoint, b2WarmStartPrismaticJoint, b2SolvePrismaticJoint, b2GetPrismaticJointForce,
    b2GetPrismaticJointTorque,
} from "./prismatic_joint";
import {
    b2PrepareRevoluteJoint, b2WarmStartRevoluteJoint, b2SolveRevoluteJoint, b2GetRevoluteJointForce,
    b2GetRevoluteJointTorque,
} from "./revolute_joint";
import {
    b2PrepareWeldJoint, b2WarmStartWeldJoint, b2SolveWeldJoint, b2GetWeldJointForce, b2GetWeldJointTorque,
} from "./weld_joint";
import {
    b2PrepareWheelJoint, b2WarmStartWheelJoint, b2SolveWheelJoint, b2GetWheelJointForce, b2GetWheelJointTorque,
} from "./wheel_joint";
import { B2_OVERFLOW_INDEX } from "./constraint_graph";

export class b2JointEdge {
    bodyId: i32 = 0;
    prevKey: i32 = 0;
    nextKey: i32 = 0;
}

function makeJointEdges(): StaticArray<b2JointEdge> {
    let a = new StaticArray<b2JointEdge>(2);
    a[0] = new b2JointEdge();
    a[1] = new b2JointEdge();
    return a;
}

// Map from b2JointId to b2Joint in the solver sets
export class b2Joint {
    userData: u64 = 0;
    setIndex: i32 = 0;
    colorIndex: i32 = 0;
    localIndex: i32 = 0;
    edges: StaticArray<b2JointEdge> = makeJointEdges();
    jointId: i32 = B2_NULL_INDEX;
    islandId: i32 = 0;
    islandPrev: i32 = 0;
    islandNext: i32 = 0;
    drawScale: f32 = 0;
    type: i32 = 0;
    generation: u16 = 0;
    collideConnected: bool = false;
}

export class b2DistanceJoint {
    length: f32 = 0;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    lowerSpringForce: f32 = 0;
    upperSpringForce: f32 = 0;
    minLength: f32 = 0;
    maxLength: f32 = 0;
    maxMotorForce: f32 = 0;
    motorSpeed: f32 = 0;
    impulse: f32 = 0;
    lowerImpulse: f32 = 0;
    upperImpulse: f32 = 0;
    motorImpulse: f32 = 0;
    indexA: i32 = 0;
    indexB: i32 = 0;
    anchorA: b2Vec2 = new b2Vec2();
    anchorB: b2Vec2 = new b2Vec2();
    deltaCenter: b2Vec2 = new b2Vec2();
    distanceSoftness: b2Softness = new b2Softness();
    axialMass: f32 = 0;
    enableSpring: bool = false;
    enableLimit: bool = false;
    enableMotor: bool = false;
}

export class b2MotorJoint {
    linearVelocity: b2Vec2 = new b2Vec2();
    maxVelocityForce: f32 = 0;
    angularVelocity: f32 = 0;
    maxVelocityTorque: f32 = 0;
    linearHertz: f32 = 0;
    linearDampingRatio: f32 = 0;
    maxSpringForce: f32 = 0;
    angularHertz: f32 = 0;
    angularDampingRatio: f32 = 0;
    maxSpringTorque: f32 = 0;
    linearVelocityImpulse: b2Vec2 = new b2Vec2();
    angularVelocityImpulse: f32 = 0;
    linearSpringImpulse: b2Vec2 = new b2Vec2();
    angularSpringImpulse: f32 = 0;
    linearSpring: b2Softness = new b2Softness();
    angularSpring: b2Softness = new b2Softness();
    indexA: i32 = 0;
    indexB: i32 = 0;
    frameA: b2Transform = new b2Transform();
    frameB: b2Transform = new b2Transform();
    deltaCenter: b2Vec2 = new b2Vec2();
    linearMass: b2Mat22 = new b2Mat22();
    angularMass: f32 = 0;
}

export class b2PrismaticJoint {
    impulse: b2Vec2 = new b2Vec2();
    springImpulse: f32 = 0;
    motorImpulse: f32 = 0;
    lowerImpulse: f32 = 0;
    upperImpulse: f32 = 0;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    targetTranslation: f32 = 0;
    maxMotorForce: f32 = 0;
    motorSpeed: f32 = 0;
    lowerTranslation: f32 = 0;
    upperTranslation: f32 = 0;
    indexA: i32 = 0;
    indexB: i32 = 0;
    frameA: b2Transform = new b2Transform();
    frameB: b2Transform = new b2Transform();
    deltaCenter: b2Vec2 = new b2Vec2();
    springSoftness: b2Softness = new b2Softness();
    enableSpring: bool = false;
    enableLimit: bool = false;
    enableMotor: bool = false;
}

export class b2RevoluteJoint {
    linearImpulse: b2Vec2 = new b2Vec2();
    springImpulse: f32 = 0;
    motorImpulse: f32 = 0;
    lowerImpulse: f32 = 0;
    upperImpulse: f32 = 0;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    targetAngle: f32 = 0;
    maxMotorTorque: f32 = 0;
    motorSpeed: f32 = 0;
    lowerAngle: f32 = 0;
    upperAngle: f32 = 0;
    indexA: i32 = 0;
    indexB: i32 = 0;
    frameA: b2Transform = new b2Transform();
    frameB: b2Transform = new b2Transform();
    deltaCenter: b2Vec2 = new b2Vec2();
    axialMass: f32 = 0;
    springSoftness: b2Softness = new b2Softness();
    enableSpring: bool = false;
    enableMotor: bool = false;
    enableLimit: bool = false;
}

export class b2WeldJoint {
    linearHertz: f32 = 0;
    linearDampingRatio: f32 = 0;
    angularHertz: f32 = 0;
    angularDampingRatio: f32 = 0;
    linearSpring: b2Softness = new b2Softness();
    angularSpring: b2Softness = new b2Softness();
    linearImpulse: b2Vec2 = new b2Vec2();
    angularImpulse: f32 = 0;
    indexA: i32 = 0;
    indexB: i32 = 0;
    frameA: b2Transform = new b2Transform();
    frameB: b2Transform = new b2Transform();
    deltaCenter: b2Vec2 = new b2Vec2();
    axialMass: f32 = 0;
}

export class b2WheelJoint {
    perpImpulse: f32 = 0;
    motorImpulse: f32 = 0;
    springImpulse: f32 = 0;
    lowerImpulse: f32 = 0;
    upperImpulse: f32 = 0;
    maxMotorTorque: f32 = 0;
    motorSpeed: f32 = 0;
    lowerTranslation: f32 = 0;
    upperTranslation: f32 = 0;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    indexA: i32 = 0;
    indexB: i32 = 0;
    frameA: b2Transform = new b2Transform();
    frameB: b2Transform = new b2Transform();
    deltaCenter: b2Vec2 = new b2Vec2();
    perpMass: f32 = 0;
    motorMass: f32 = 0;
    axialMass: f32 = 0;
    springSoftness: b2Softness = new b2Softness();
    enableSpring: bool = false;
    enableMotor: bool = false;
    enableLimit: bool = false;
}

// The base joint class.
export class b2JointSim {
    jointId: i32 = 0;
    bodyIdA: i32 = 0;
    bodyIdB: i32 = 0;
    type: i32 = 0;
    localFrameA: b2Transform = new b2Transform();
    localFrameB: b2Transform = new b2Transform();
    invMassA: f32 = 0;
    invMassB: f32 = 0;
    invIA: f32 = 0;
    invIB: f32 = 0;
    constraintHertz: f32 = 0;
    constraintDampingRatio: f32 = 0;
    constraintSoftness: b2Softness = new b2Softness();
    forceThreshold: f32 = 0;
    torqueThreshold: f32 = 0;

    // C union: exactly one is non-null
    distanceJoint: b2DistanceJoint | null = null;
    motorJoint: b2MotorJoint | null = null;
    revoluteJoint: b2RevoluteJoint | null = null;
    prismaticJoint: b2PrismaticJoint | null = null;
    weldJoint: b2WeldJoint | null = null;
    wheelJoint: b2WheelJoint | null = null;
}

// Joint definitions (C keeps these in types.h; defaults in joint.c)
export class b2JointDef {
    userData: u64 = 0;
    bodyIdA: b2BodyId = new b2BodyId(0, 0, 0);
    bodyIdB: b2BodyId = new b2BodyId(0, 0, 0);
    localFrameA: b2Transform = new b2Transform();
    localFrameB: b2Transform = new b2Transform();
    forceThreshold: f32 = 0;
    torqueThreshold: f32 = 0;
    constraintHertz: f32 = 0;
    constraintDampingRatio: f32 = 0;
    drawScale: f32 = 0;
    collideConnected: bool = false;
}

function b2DefaultJointDef(): b2JointDef {
    let def = new b2JointDef();
    def.localFrameA.q = new b2Rot(1, 0);
    def.localFrameB.q = new b2Rot(1, 0);
    def.forceThreshold = FLT_MAX;
    def.torqueThreshold = FLT_MAX;
    def.constraintHertz = 60.0;
    def.constraintDampingRatio = 2.0;
    def.drawScale = b2_lengthUnitsPerMeter;
    return def;
}

export class b2DistanceJointDef {
    base: b2JointDef = new b2JointDef();
    length: f32 = 0;
    enableSpring: bool = false;
    lowerSpringForce: f32 = 0;
    upperSpringForce: f32 = 0;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    enableLimit: bool = false;
    minLength: f32 = 0;
    maxLength: f32 = 0;
    enableMotor: bool = false;
    maxMotorForce: f32 = 0;
    motorSpeed: f32 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultDistanceJointDef(): b2DistanceJointDef {
    let def = new b2DistanceJointDef();
    def.base = b2DefaultJointDef();
    def.lowerSpringForce = -FLT_MAX;
    def.upperSpringForce = FLT_MAX;
    def.length = 1.0;
    def.maxLength = B2_HUGE();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2MotorJointDef {
    base: b2JointDef = new b2JointDef();
    linearVelocity: b2Vec2 = new b2Vec2();
    maxVelocityForce: f32 = 0;
    angularVelocity: f32 = 0;
    maxVelocityTorque: f32 = 0;
    linearHertz: f32 = 0;
    linearDampingRatio: f32 = 0;
    maxSpringForce: f32 = 0;
    angularHertz: f32 = 0;
    angularDampingRatio: f32 = 0;
    maxSpringTorque: f32 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultMotorJointDef(): b2MotorJointDef {
    let def = new b2MotorJointDef();
    def.base = b2DefaultJointDef();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2FilterJointDef {
    base: b2JointDef = new b2JointDef();
    internalValue: i32 = 0;
}

export function b2DefaultFilterJointDef(): b2FilterJointDef {
    let def = new b2FilterJointDef();
    def.base = b2DefaultJointDef();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2PrismaticJointDef {
    base: b2JointDef = new b2JointDef();
    enableSpring: bool = false;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    targetTranslation: f32 = 0;
    enableLimit: bool = false;
    lowerTranslation: f32 = 0;
    upperTranslation: f32 = 0;
    enableMotor: bool = false;
    maxMotorForce: f32 = 0;
    motorSpeed: f32 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultPrismaticJointDef(): b2PrismaticJointDef {
    let def = new b2PrismaticJointDef();
    def.base = b2DefaultJointDef();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2RevoluteJointDef {
    base: b2JointDef = new b2JointDef();
    targetAngle: f32 = 0;
    enableSpring: bool = false;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    enableLimit: bool = false;
    lowerAngle: f32 = 0;
    upperAngle: f32 = 0;
    enableMotor: bool = false;
    maxMotorTorque: f32 = 0;
    motorSpeed: f32 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultRevoluteJointDef(): b2RevoluteJointDef {
    let def = new b2RevoluteJointDef();
    def.base = b2DefaultJointDef();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2WeldJointDef {
    base: b2JointDef = new b2JointDef();
    linearHertz: f32 = 0;
    angularHertz: f32 = 0;
    linearDampingRatio: f32 = 0;
    angularDampingRatio: f32 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultWeldJointDef(): b2WeldJointDef {
    let def = new b2WeldJointDef();
    def.base = b2DefaultJointDef();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2WheelJointDef {
    base: b2JointDef = new b2JointDef();
    enableSpring: bool = false;
    hertz: f32 = 0;
    dampingRatio: f32 = 0;
    enableLimit: bool = false;
    lowerTranslation: f32 = 0;
    upperTranslation: f32 = 0;
    enableMotor: bool = false;
    maxMotorTorque: f32 = 0;
    motorSpeed: f32 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultWheelJointDef(): b2WheelJointDef {
    let def = new b2WheelJointDef();
    def.base = b2DefaultJointDef();
    def.enableSpring = true;
    def.hertz = 1.0;
    def.dampingRatio = 0.7;
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export function b2GetJointFullId(world: b2World, jointId: b2JointId): b2Joint {
    let id = jointId.index1 - 1;
    let joint = world.joints.get(id);
    assert(joint.jointId == id && joint.generation == jointId.generation);
    return joint;
}

export function b2GetJointSim(world: b2World, joint: b2Joint): b2JointSim {
    if (joint.setIndex == b2_awakeSet) {
        assert(0 <= joint.colorIndex && joint.colorIndex < B2_GRAPH_COLOR_COUNT);
        let color = world.constraintGraph.colors[joint.colorIndex];
        return color.jointSims.get(joint.localIndex);
    }

    let set = world.solverSets.get(joint.setIndex);
    return set.jointSims.get(joint.localIndex);
}

export function b2GetJointSimCheckType(jointId: b2JointId, type: i32): b2JointSim | null {
    let world = b2GetWorld(jointId.world0);
    assert(world.locked == false);
    if (world.locked) {
        return null;
    }

    let joint = b2GetJointFullId(world, jointId);
    assert(joint.type == type);
    let jointSim = b2GetJointSim(world, joint);
    assert(jointSim.type == type);
    return jointSim;
}

function b2DestroyContactsBetweenBodies(world: b2World, bodyA: b2Body, bodyB: b2Body): void {
    let contactKey: i32;
    let otherBodyId: i32;

    // use the smaller of the two contact lists
    if (bodyA.contactCount < bodyB.contactCount) {
        contactKey = bodyA.headContactKey;
        otherBodyId = bodyB.id;
    } else {
        contactKey = bodyB.headContactKey;
        otherBodyId = bodyA.id;
    }

    // no need to wake bodies when a joint removes collision between them
    let wakeBodies = false;

    // destroy the contacts
    while (contactKey != B2_NULL_INDEX) {
        let contactId = contactKey >> 1;
        let edgeIndex = contactKey & 1;

        let contact = world.contacts.get(contactId);
        contactKey = contact.edges[edgeIndex].nextKey;

        let otherEdgeIndex = edgeIndex ^ 1;
        if (contact.edges[otherEdgeIndex].bodyId == otherBodyId) {
            // Careful, this removes the contact from the current doubly linked list
            b2DestroyContact(world, contact, wakeBodies);
        }
    }

    b2ValidateSolverSets(world);
}

export class b2JointPair {
    joint: b2Joint;
    jointSim: b2JointSim;

    constructor(joint: b2Joint, jointSim: b2JointSim) {
        this.joint = joint;
        this.jointSim = jointSim;
    }
}

function b2CreateJoint(world: b2World, def: b2JointDef, type: i32): b2JointPair {
    assert(b2IsValidTransform(def.localFrameA));
    assert(b2IsValidTransform(def.localFrameB));
    assert(world.worldId == def.bodyIdA.world0);
    assert(world.worldId == def.bodyIdB.world0);

    let bodyA = b2GetBodyFullId(world, def.bodyIdA);
    let bodyB = b2GetBodyFullId(world, def.bodyIdB);

    let bodyIdA = bodyA.id;
    let bodyIdB = bodyB.id;
    let maxSetIndex = b2MaxInt(bodyA.setIndex, bodyB.setIndex);

    // Create joint id and joint
    let jointId = b2AllocId(world.jointIdPool);
    if (jointId == world.joints.count) {
        world.joints.add();
    }

    let joint = world.joints.get(jointId);
    joint.jointId = jointId;
    joint.userData = def.userData;
    joint.generation += 1;
    joint.setIndex = B2_NULL_INDEX;
    joint.colorIndex = B2_NULL_INDEX;
    joint.localIndex = B2_NULL_INDEX;
    joint.islandId = B2_NULL_INDEX;
    joint.islandPrev = B2_NULL_INDEX;
    joint.islandNext = B2_NULL_INDEX;
    joint.drawScale = def.drawScale;
    joint.type = type;
    joint.collideConnected = def.collideConnected;

    // Doubly linked list on bodyA
    joint.edges[0].bodyId = bodyIdA;
    joint.edges[0].prevKey = B2_NULL_INDEX;
    joint.edges[0].nextKey = bodyA.headJointKey;

    let keyA = (jointId << 1) | 0;
    if (bodyA.headJointKey != B2_NULL_INDEX) {
        let jointA = world.joints.get(bodyA.headJointKey >> 1);
        let edgeA = jointA.edges[bodyA.headJointKey & 1];
        edgeA.prevKey = keyA;
    }
    bodyA.headJointKey = keyA;
    bodyA.jointCount += 1;

    // Doubly linked list on bodyB
    joint.edges[1].bodyId = bodyIdB;
    joint.edges[1].prevKey = B2_NULL_INDEX;
    joint.edges[1].nextKey = bodyB.headJointKey;

    let keyB = (jointId << 1) | 1;
    if (bodyB.headJointKey != B2_NULL_INDEX) {
        let jointB = world.joints.get(bodyB.headJointKey >> 1);
        let edgeB = jointB.edges[bodyB.headJointKey & 1];
        edgeB.prevKey = keyB;
    }
    bodyB.headJointKey = keyB;
    bodyB.jointCount += 1;

    let jointSim: b2JointSim;

    if (bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet) {
        // if either body is disabled, create in disabled set
        let set = world.solverSets.get(b2_disabledSet);
        joint.setIndex = b2_disabledSet;
        joint.localIndex = set.jointSims.count;

        jointSim = set.jointSims.add();

        jointSim.jointId = jointId;
        jointSim.bodyIdA = bodyIdA;
        jointSim.bodyIdB = bodyIdB;
    } else if (bodyA.type != b2_dynamicBody && bodyB.type != b2_dynamicBody) {
        // joint is not attached to a dynamic body
        let set = world.solverSets.get(b2_staticSet);
        joint.setIndex = b2_staticSet;
        joint.localIndex = set.jointSims.count;

        jointSim = set.jointSims.add();

        jointSim.jointId = jointId;
        jointSim.bodyIdA = bodyIdA;
        jointSim.bodyIdB = bodyIdB;
    } else if (bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet) {
        // if either body is sleeping, wake it
        if (maxSetIndex >= b2_firstSleepingSet) {
            b2WakeSolverSet(world, maxSetIndex);
        }

        joint.setIndex = b2_awakeSet;

        jointSim = b2CreateJointInGraph(world, joint);
        jointSim.jointId = jointId;
        jointSim.bodyIdA = bodyIdA;
        jointSim.bodyIdB = bodyIdB;
    } else {
        // joint connected between sleeping and/or static bodies
        assert(bodyA.setIndex >= b2_firstSleepingSet || bodyB.setIndex >= b2_firstSleepingSet);
        assert(bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet);

        // joint should go into the sleeping set (not static set)
        let setIndex = maxSetIndex;

        let set = world.solverSets.get(setIndex);
        joint.setIndex = setIndex;
        joint.localIndex = set.jointSims.count;

        jointSim = set.jointSims.add();

        // These must be set to accommodate the merge below
        jointSim.jointId = jointId;
        jointSim.bodyIdA = bodyIdA;
        jointSim.bodyIdB = bodyIdB;

        if (bodyA.setIndex != bodyB.setIndex && bodyA.setIndex >= b2_firstSleepingSet &&
            bodyB.setIndex >= b2_firstSleepingSet) {
            // merge sleeping sets
            b2MergeSolverSets(world, bodyA.setIndex, bodyB.setIndex);
            assert(bodyA.setIndex == bodyB.setIndex);

            // fix potentially invalid set index
            setIndex = bodyA.setIndex;

            let mergedSet = world.solverSets.get(setIndex);

            // Careful! The joint sim reference was orphaned by the set merge.
            jointSim = mergedSet.jointSims.get(joint.localIndex);
        }

        assert(joint.setIndex == setIndex);
    }

    jointSim.localFrameA = def.localFrameA.clone();
    jointSim.localFrameB = def.localFrameB.clone();
    jointSim.type = type;
    jointSim.constraintHertz = def.constraintHertz;
    jointSim.constraintDampingRatio = def.constraintDampingRatio;
    jointSim.constraintSoftness = new b2Softness();
    jointSim.constraintSoftness.biasRate = 0.0;
    jointSim.constraintSoftness.massScale = 1.0;
    jointSim.constraintSoftness.impulseScale = 0.0;

    assert(b2IsValidFloat(def.forceThreshold) && def.forceThreshold >= 0.0);
    assert(b2IsValidFloat(def.torqueThreshold) && def.torqueThreshold >= 0.0);

    jointSim.forceThreshold = def.forceThreshold;
    jointSim.torqueThreshold = def.torqueThreshold;

    assert(jointSim.jointId == jointId);
    assert(jointSim.bodyIdA == bodyIdA);
    assert(jointSim.bodyIdB == bodyIdB);

    if (joint.setIndex > b2_disabledSet) {
        // Add edge to island graph
        b2LinkJoint(world, joint);
    }

    // If the joint prevents collisions, then destroy all contacts between attached bodies
    if (def.collideConnected == false) {
        b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
    }

    b2ValidateSolverSets(world);

    return new b2JointPair(joint, jointSim);
}

export function b2CreateDistanceJoint(worldId: b2WorldId, def: b2DistanceJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    assert(b2IsValidFloat(def.length) && def.length > 0.0);
    assert(def.lowerSpringForce <= def.upperSpringForce);

    let pair = b2CreateJoint(world, def.base, b2_distanceJoint);

    let joint = pair.jointSim;

    let dj = new b2DistanceJoint();
    joint.distanceJoint = dj;
    dj.length = b2MaxFloat(def.length, B2_LINEAR_SLOP());
    dj.hertz = def.hertz;
    dj.dampingRatio = def.dampingRatio;
    dj.minLength = b2MaxFloat(def.minLength, B2_LINEAR_SLOP());
    dj.maxLength = b2MaxFloat(def.minLength, def.maxLength);
    dj.maxMotorForce = def.maxMotorForce;
    dj.motorSpeed = def.motorSpeed;
    dj.enableSpring = def.enableSpring;
    dj.lowerSpringForce = def.lowerSpringForce;
    dj.upperSpringForce = def.upperSpringForce;
    dj.enableLimit = def.enableLimit;
    dj.enableMotor = def.enableMotor;
    dj.impulse = 0.0;
    dj.lowerImpulse = 0.0;
    dj.upperImpulse = 0.0;
    dj.motorImpulse = 0.0;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2CreateMotorJoint(worldId: b2WorldId, def: b2MotorJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    let pair = b2CreateJoint(world, def.base, b2_motorJoint);
    let joint = pair.jointSim;

    let mj = new b2MotorJoint();
    joint.motorJoint = mj;
    mj.linearVelocity = def.linearVelocity.clone();
    mj.maxVelocityForce = def.maxVelocityForce;
    mj.angularVelocity = def.angularVelocity;
    mj.maxVelocityTorque = def.maxVelocityTorque;
    mj.linearHertz = def.linearHertz;
    mj.linearDampingRatio = def.linearDampingRatio;
    mj.maxSpringForce = def.maxSpringForce;
    mj.angularHertz = def.angularHertz;
    mj.angularDampingRatio = def.angularDampingRatio;
    mj.maxSpringTorque = def.maxSpringTorque;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2CreateFilterJoint(worldId: b2WorldId, def: b2FilterJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    let pair = b2CreateJoint(world, def.base, b2_filterJoint);

    let joint = pair.jointSim;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2CreatePrismaticJoint(worldId: b2WorldId, def: b2PrismaticJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    assert(def.lowerTranslation <= def.upperTranslation);

    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    let pair = b2CreateJoint(world, def.base, b2_prismaticJoint);

    let joint = pair.jointSim;

    let pj = new b2PrismaticJoint();
    joint.prismaticJoint = pj;
    pj.hertz = def.hertz;
    pj.dampingRatio = def.dampingRatio;
    pj.targetTranslation = def.targetTranslation;
    pj.lowerTranslation = def.lowerTranslation;
    pj.upperTranslation = def.upperTranslation;
    pj.maxMotorForce = def.maxMotorForce;
    pj.motorSpeed = def.motorSpeed;
    pj.enableSpring = def.enableSpring;
    pj.enableLimit = def.enableLimit;
    pj.enableMotor = def.enableMotor;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2CreateRevoluteJoint(worldId: b2WorldId, def: b2RevoluteJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    assert(def.lowerAngle <= def.upperAngle);
    assert(def.lowerAngle >= <f32>-0.99 * B2_PI);
    assert(def.upperAngle <= <f32>0.99 * B2_PI);

    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    let pair = b2CreateJoint(world, def.base, b2_revoluteJoint);

    let joint = pair.jointSim;

    let rj = new b2RevoluteJoint();
    joint.revoluteJoint = rj;

    rj.targetAngle = b2ClampFloat(def.targetAngle, -B2_PI, B2_PI);
    rj.hertz = def.hertz;
    rj.dampingRatio = def.dampingRatio;
    rj.lowerAngle = def.lowerAngle;
    rj.upperAngle = def.upperAngle;
    rj.maxMotorTorque = def.maxMotorTorque;
    rj.motorSpeed = def.motorSpeed;
    rj.enableSpring = def.enableSpring;
    rj.enableLimit = def.enableLimit;
    rj.enableMotor = def.enableMotor;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2CreateWeldJoint(worldId: b2WorldId, def: b2WeldJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    let pair = b2CreateJoint(world, def.base, b2_weldJoint);

    let joint = pair.jointSim;

    let wj = new b2WeldJoint();
    joint.weldJoint = wj;
    wj.linearHertz = def.linearHertz;
    wj.linearDampingRatio = def.linearDampingRatio;
    wj.angularHertz = def.angularHertz;
    wj.angularDampingRatio = def.angularDampingRatio;
    wj.linearImpulse = b2Vec2_zero();
    wj.angularImpulse = 0.0;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2CreateWheelJoint(worldId: b2WorldId, def: b2WheelJointDef): b2JointId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    assert(def.lowerTranslation <= def.upperTranslation);

    let world = b2GetWorldFromId(worldId);

    assert(world.locked == false);

    if (world.locked) {
        return new b2JointId(0, 0, 0);
    }

    let pair = b2CreateJoint(world, def.base, b2_wheelJoint);

    let joint = pair.jointSim;

    let wj = new b2WheelJoint();
    joint.wheelJoint = wj;
    wj.perpMass = 0.0;
    wj.axialMass = 0.0;
    wj.motorImpulse = 0.0;
    wj.lowerImpulse = 0.0;
    wj.upperImpulse = 0.0;
    wj.lowerTranslation = def.lowerTranslation;
    wj.upperTranslation = def.upperTranslation;
    wj.maxMotorTorque = def.maxMotorTorque;
    wj.motorSpeed = def.motorSpeed;
    wj.hertz = def.hertz;
    wj.dampingRatio = def.dampingRatio;
    wj.enableSpring = def.enableSpring;
    wj.enableLimit = def.enableLimit;
    wj.enableMotor = def.enableMotor;

    return new b2JointId(joint.jointId + 1, world.worldId, pair.joint.generation);
}

export function b2DestroyJointInternal(world: b2World, joint: b2Joint, wakeBodies: bool): void {
    let jointId = joint.jointId;

    let edgeA = joint.edges[0];
    let edgeB = joint.edges[1];

    let idA = edgeA.bodyId;
    let idB = edgeB.bodyId;
    let bodyA = world.bodies.get(idA);
    let bodyB = world.bodies.get(idB);

    // Remove from body A
    if (edgeA.prevKey != B2_NULL_INDEX) {
        let prevJoint = world.joints.get(edgeA.prevKey >> 1);
        let prevEdge = prevJoint.edges[edgeA.prevKey & 1];
        prevEdge.nextKey = edgeA.nextKey;
    }

    if (edgeA.nextKey != B2_NULL_INDEX) {
        let nextJoint = world.joints.get(edgeA.nextKey >> 1);
        let nextEdge = nextJoint.edges[edgeA.nextKey & 1];
        nextEdge.prevKey = edgeA.prevKey;
    }

    let edgeKeyA = (jointId << 1) | 0;
    if (bodyA.headJointKey == edgeKeyA) {
        bodyA.headJointKey = edgeA.nextKey;
    }

    bodyA.jointCount -= 1;

    // Remove from body B
    if (edgeB.prevKey != B2_NULL_INDEX) {
        let prevJoint = world.joints.get(edgeB.prevKey >> 1);
        let prevEdge = prevJoint.edges[edgeB.prevKey & 1];
        prevEdge.nextKey = edgeB.nextKey;
    }

    if (edgeB.nextKey != B2_NULL_INDEX) {
        let nextJoint = world.joints.get(edgeB.nextKey >> 1);
        let nextEdge = nextJoint.edges[edgeB.nextKey & 1];
        nextEdge.prevKey = edgeB.prevKey;
    }

    let edgeKeyB = (jointId << 1) | 1;
    if (bodyB.headJointKey == edgeKeyB) {
        bodyB.headJointKey = edgeB.nextKey;
    }

    bodyB.jointCount -= 1;

    if (joint.islandId != B2_NULL_INDEX) {
        assert(joint.setIndex > b2_disabledSet);
        b2UnlinkJoint(world, joint);
    } else {
        assert(joint.setIndex <= b2_disabledSet);
    }

    // Remove joint from solver set that owns it
    let setIndex = joint.setIndex;
    let localIndex = joint.localIndex;

    if (setIndex == b2_awakeSet) {
        b2RemoveJointFromGraph(world, joint.edges[0].bodyId, joint.edges[1].bodyId, joint.colorIndex, localIndex);
    } else {
        let set = world.solverSets.get(setIndex);
        let movedIndex = set.jointSims.removeSwap(localIndex);
        if (movedIndex != B2_NULL_INDEX) {
            // Fix moved joint
            let movedJointSim = set.jointSims.get(localIndex);
            let movedId = movedJointSim.jointId;
            let movedJoint = world.joints.get(movedId);
            assert(movedJoint.localIndex == movedIndex);
            movedJoint.localIndex = localIndex;
        }
    }

    // Free joint and id (preserve joint generation)
    joint.setIndex = B2_NULL_INDEX;
    joint.localIndex = B2_NULL_INDEX;
    joint.colorIndex = B2_NULL_INDEX;
    joint.jointId = B2_NULL_INDEX;
    b2FreeId(world.jointIdPool, jointId);

    if (wakeBodies) {
        b2WakeBody(world, bodyA);
        b2WakeBody(world, bodyB);
    }

    b2ValidateSolverSets(world);
}

export function b2DestroyJoint(jointId: b2JointId, wakeAttached: bool): void {
    let world = b2GetWorld(jointId.world0);
    assert(world.locked == false);

    if (world.locked) {
        return;
    }

    let joint = b2GetJointFullId(world, jointId);

    b2DestroyJointInternal(world, joint, wakeAttached);
}

export function b2Joint_GetType(jointId: b2JointId): i32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return joint.type;
}

export function b2Joint_GetBodyA(jointId: b2JointId): b2BodyId {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return b2MakeBodyId(world, joint.edges[0].bodyId);
}

export function b2Joint_GetBodyB(jointId: b2JointId): b2BodyId {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return b2MakeBodyId(world, joint.edges[1].bodyId);
}

export function b2Joint_GetWorld(jointId: b2JointId): b2WorldId {
    let world = b2GetWorld(jointId.world0);
    return new b2WorldId(jointId.world0 + 1, world.generation);
}

export function b2Joint_SetLocalFrameA(jointId: b2JointId, localFrame: b2Transform): void {
    assert(b2IsValidTransform(localFrame));

    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let jointSim = b2GetJointSim(world, joint);
    jointSim.localFrameA = localFrame.clone();
}

export function b2Joint_GetLocalFrameA(jointId: b2JointId): b2Transform {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let jointSim = b2GetJointSim(world, joint);
    return jointSim.localFrameA.clone();
}

export function b2Joint_SetLocalFrameB(jointId: b2JointId, localFrame: b2Transform): void {
    assert(b2IsValidTransform(localFrame));

    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let jointSim = b2GetJointSim(world, joint);
    jointSim.localFrameB = localFrame.clone();
}

export function b2Joint_GetLocalFrameB(jointId: b2JointId): b2Transform {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let jointSim = b2GetJointSim(world, joint);
    return jointSim.localFrameB.clone();
}

export function b2Joint_SetCollideConnected(jointId: b2JointId, shouldCollide: bool): void {
    let world = b2GetWorldLocked(jointId.world0);
    if (world == null) {
        return;
    }

    let joint = b2GetJointFullId(world, jointId);
    if (joint.collideConnected == shouldCollide) {
        return;
    }

    joint.collideConnected = shouldCollide;

    let bodyA = world.bodies.get(joint.edges[0].bodyId);
    let bodyB = world.bodies.get(joint.edges[1].bodyId);

    if (shouldCollide) {
        // need to tell the broad-phase to look for new pairs for one of the
        // two bodies. Pick the one with the fewest shapes.
        let shapeCountA = bodyA.shapeCount;
        let shapeCountB = bodyB.shapeCount;

        let shapeId = shapeCountA < shapeCountB ? bodyA.headShapeId : bodyB.headShapeId;
        while (shapeId != B2_NULL_INDEX) {
            let shape = world.shapes.get(shapeId);

            if (shape.proxyKey != B2_NULL_INDEX) {
                b2BufferMove(world.broadPhase, shape.proxyKey);
            }

            shapeId = shape.nextShapeId;
        }
    } else {
        b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
    }
}

export function b2Joint_GetCollideConnected(jointId: b2JointId): bool {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return joint.collideConnected;
}

export function b2Joint_SetUserData(jointId: b2JointId, userData: u64): void {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    joint.userData = userData;
}

export function b2Joint_GetUserData(jointId: b2JointId): u64 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return joint.userData;
}

export function b2Joint_WakeBodies(jointId: b2JointId): void {
    let world = b2GetWorldLocked(jointId.world0);
    if (world == null) {
        return;
    }

    let joint = b2GetJointFullId(world, jointId);
    let bodyA = world.bodies.get(joint.edges[0].bodyId);
    let bodyB = world.bodies.get(joint.edges[1].bodyId);

    b2WakeBody(world, bodyA);
    b2WakeBody(world, bodyB);
}

export class b2JointReaction {
    force: f32 = 0;
    torque: f32 = 0;
}

export function b2GetJointReaction(sim: b2JointSim, invTimeStep: f32): b2JointReaction {
    let linearImpulse: f32 = 0.0;
    let angularImpulse: f32 = 0.0;

    switch (sim.type) {
        case b2_distanceJoint: {
            let joint = sim.distanceJoint!;
            linearImpulse = b2AbsFloat(joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse);
            break;
        }

        case b2_motorJoint: {
            let joint = sim.motorJoint!;
            linearImpulse = b2Length(b2Add(joint.linearVelocityImpulse, joint.linearSpringImpulse));
            angularImpulse = b2AbsFloat(joint.angularVelocityImpulse + joint.angularSpringImpulse);
            break;
        }

        case b2_prismaticJoint: {
            let joint = sim.prismaticJoint!;
            let perpImpulse = joint.impulse.x;
            let axialImpulse = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
            linearImpulse = sqrt<f32>(perpImpulse * perpImpulse + axialImpulse * axialImpulse);
            angularImpulse = b2AbsFloat(joint.impulse.y);
            break;
        }

        case b2_revoluteJoint: {
            let joint = sim.revoluteJoint!;
            linearImpulse = b2Length(joint.linearImpulse);
            angularImpulse = b2AbsFloat(joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse);
            break;
        }

        case b2_weldJoint: {
            let joint = sim.weldJoint!;
            linearImpulse = b2Length(joint.linearImpulse);
            angularImpulse = b2AbsFloat(joint.angularImpulse);
            break;
        }

        case b2_wheelJoint: {
            let joint = sim.wheelJoint!;
            let perpImpulse = joint.perpImpulse;
            let axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;
            linearImpulse = sqrt<f32>(perpImpulse * perpImpulse + axialImpulse * axialImpulse);
            angularImpulse = b2AbsFloat(joint.motorImpulse);
            break;
        }

        default:
            break;
    }

    let reaction = new b2JointReaction();
    reaction.force = linearImpulse * invTimeStep;
    reaction.torque = angularImpulse * invTimeStep;
    return reaction;
}

function b2GetJointConstraintForce(world: b2World, joint: b2Joint): b2Vec2 {
    let base = b2GetJointSim(world, joint);

    switch (joint.type) {
        case b2_distanceJoint:
            return b2GetDistanceJointForce(world, base);

        case b2_motorJoint:
            return b2GetMotorJointForce(world, base);

        case b2_filterJoint:
            return b2Vec2_zero();

        case b2_prismaticJoint:
            return b2GetPrismaticJointForce(world, base);

        case b2_revoluteJoint:
            return b2GetRevoluteJointForce(world, base);

        case b2_weldJoint:
            return b2GetWeldJointForce(world, base);

        case b2_wheelJoint:
            return b2GetWheelJointForce(world, base);

        default:
            assert(false);
            return b2Vec2_zero();
    }
}

function b2GetJointConstraintTorque(world: b2World, joint: b2Joint): f32 {
    let base = b2GetJointSim(world, joint);

    switch (joint.type) {
        case b2_distanceJoint:
            return 0.0;

        case b2_motorJoint:
            return b2GetMotorJointTorque(world, base);

        case b2_filterJoint:
            return 0.0;

        case b2_prismaticJoint:
            return b2GetPrismaticJointTorque(world, base);

        case b2_revoluteJoint:
            return b2GetRevoluteJointTorque(world, base);

        case b2_weldJoint:
            return b2GetWeldJointTorque(world, base);

        case b2_wheelJoint:
            return b2GetWheelJointTorque(world, base);

        default:
            assert(false);
            return 0.0;
    }
}

export function b2Joint_GetConstraintForce(jointId: b2JointId): b2Vec2 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return b2GetJointConstraintForce(world, joint);
}

export function b2Joint_GetConstraintTorque(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    return b2GetJointConstraintTorque(world, joint);
}

export function b2Joint_GetLinearSeparation(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);

    let xfA = b2GetBodyTransform(world, joint.edges[0].bodyId);
    let xfB = b2GetBodyTransform(world, joint.edges[1].bodyId);

    let pA = b2TransformPoint(xfA, base.localFrameA.p);
    let pB = b2TransformPoint(xfB, base.localFrameB.p);
    let dp = b2Sub(pB, pA);

    switch (joint.type) {
        case b2_distanceJoint: {
            let distanceJoint = base.distanceJoint!;
            let length = b2Length(dp);
            if (distanceJoint.enableSpring) {
                if (distanceJoint.enableLimit) {
                    if (length < distanceJoint.minLength) {
                        return distanceJoint.minLength - length;
                    }

                    if (length > distanceJoint.maxLength) {
                        return length - distanceJoint.maxLength;
                    }

                    return 0.0;
                }

                return 0.0;
            }

            return b2AbsFloat(length - distanceJoint.length);
        }

        case b2_motorJoint:
            return 0.0;

        case b2_filterJoint:
            return 0.0;

        case b2_prismaticJoint: {
            let prismaticJoint = base.prismaticJoint!;
            let axisA = b2RotateVector(xfA.q, new b2Vec2(1.0, 0.0));
            let perpA = b2LeftPerp(axisA);
            let perpendicularSeparation = b2AbsFloat(b2Dot(perpA, dp));
            let limitSeparation: f32 = 0.0;

            if (prismaticJoint.enableLimit) {
                let translation = b2Dot(axisA, dp);
                if (translation < prismaticJoint.lowerTranslation) {
                    limitSeparation = prismaticJoint.lowerTranslation - translation;
                }

                if (prismaticJoint.upperTranslation < translation) {
                    limitSeparation = translation - prismaticJoint.upperTranslation;
                }
            }

            return sqrt<f32>(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
        }

        case b2_revoluteJoint:
            return b2Length(dp);

        case b2_weldJoint: {
            let weldJoint = base.weldJoint!;
            if (weldJoint.linearHertz == 0.0) {
                return b2Length(dp);
            }

            return 0.0;
        }

        case b2_wheelJoint: {
            let wheelJoint = base.wheelJoint!;
            let axisA = b2RotateVector(xfA.q, new b2Vec2(1.0, 0.0));
            let perpA = b2LeftPerp(axisA);
            let perpendicularSeparation = b2AbsFloat(b2Dot(perpA, dp));
            let limitSeparation: f32 = 0.0;

            if (wheelJoint.enableLimit) {
                let translation = b2Dot(axisA, dp);
                if (translation < wheelJoint.lowerTranslation) {
                    limitSeparation = wheelJoint.lowerTranslation - translation;
                }

                if (wheelJoint.upperTranslation < translation) {
                    limitSeparation = translation - wheelJoint.upperTranslation;
                }
            }

            return sqrt<f32>(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
        }

        default:
            assert(false);
            return 0.0;
    }
}

export function b2Joint_GetAngularSeparation(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);

    let xfA = b2GetBodyTransform(world, joint.edges[0].bodyId);
    let xfB = b2GetBodyTransform(world, joint.edges[1].bodyId);
    let relativeAngle = b2RelativeAngle(xfA.q, xfB.q);

    switch (joint.type) {
        case b2_distanceJoint:
            return 0.0;

        case b2_motorJoint:
            return 0.0;

        case b2_filterJoint:
            return 0.0;

        case b2_prismaticJoint: {
            return relativeAngle;
        }

        case b2_revoluteJoint: {
            let revoluteJoint = base.revoluteJoint!;
            if (revoluteJoint.enableLimit) {
                let angle = relativeAngle;
                if (angle < revoluteJoint.lowerAngle) {
                    return revoluteJoint.lowerAngle - angle;
                }

                if (revoluteJoint.upperAngle < angle) {
                    return angle - revoluteJoint.upperAngle;
                }
            }

            return 0.0;
        }

        case b2_weldJoint: {
            let weldJoint = base.weldJoint!;
            if (weldJoint.angularHertz == 0.0) {
                return relativeAngle;
            }

            return 0.0;
        }

        case b2_wheelJoint:
            return 0.0;

        default:
            assert(false);
            return 0.0;
    }
}

export function b2Joint_SetConstraintTuning(jointId: b2JointId, hertz: f32, dampingRatio: f32): void {
    assert(b2IsValidFloat(hertz) && hertz >= 0.0);
    assert(b2IsValidFloat(dampingRatio) && dampingRatio >= 0.0);

    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);
    base.constraintHertz = hertz;
    base.constraintDampingRatio = dampingRatio;
}

export function b2Joint_SetForceThreshold(jointId: b2JointId, threshold: f32): void {
    assert(b2IsValidFloat(threshold) && threshold >= 0.0);

    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);
    base.forceThreshold = threshold;
}

export function b2Joint_GetForceThreshold(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);
    return base.forceThreshold;
}

export function b2Joint_SetTorqueThreshold(jointId: b2JointId, threshold: f32): void {
    assert(b2IsValidFloat(threshold) && threshold >= 0.0);

    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);
    base.torqueThreshold = threshold;
}

export function b2Joint_GetTorqueThreshold(jointId: b2JointId): f32 {
    let world = b2GetWorld(jointId.world0);
    let joint = b2GetJointFullId(world, jointId);
    let base = b2GetJointSim(world, joint);
    return base.torqueThreshold;
}

export function b2PrepareJoint(joint: b2JointSim, context: b2StepContext): void {
    // Clamp joint hertz based on the time step to reduce jitter.
    let hertz = b2MinFloat(joint.constraintHertz, <f32>0.25 * context.inv_h);
    joint.constraintSoftness = b2MakeSoft(hertz, joint.constraintDampingRatio, context.h);

    switch (joint.type) {
        case b2_distanceJoint:
            b2PrepareDistanceJoint(joint, context);
            break;

        case b2_motorJoint:
            b2PrepareMotorJoint(joint, context);
            break;

        case b2_filterJoint:
            break;

        case b2_prismaticJoint:
            b2PreparePrismaticJoint(joint, context);
            break;

        case b2_revoluteJoint:
            b2PrepareRevoluteJoint(joint, context);
            break;

        case b2_weldJoint:
            b2PrepareWeldJoint(joint, context);
            break;

        case b2_wheelJoint:
            b2PrepareWheelJoint(joint, context);
            break;

        default:
            assert(false);
    }
}

export function b2WarmStartJoint(joint: b2JointSim, context: b2StepContext): void {
    switch (joint.type) {
        case b2_distanceJoint:
            b2WarmStartDistanceJoint(joint, context);
            break;

        case b2_motorJoint:
            b2WarmStartMotorJoint(joint, context);
            break;

        case b2_filterJoint:
            break;

        case b2_prismaticJoint:
            b2WarmStartPrismaticJoint(joint, context);
            break;

        case b2_revoluteJoint:
            b2WarmStartRevoluteJoint(joint, context);
            break;

        case b2_weldJoint:
            b2WarmStartWeldJoint(joint, context);
            break;

        case b2_wheelJoint:
            b2WarmStartWheelJoint(joint, context);
            break;

        default:
            assert(false);
    }
}

export function b2SolveJoint(joint: b2JointSim, context: b2StepContext, useBias: bool): void {
    switch (joint.type) {
        case b2_distanceJoint:
            b2SolveDistanceJoint(joint, context, useBias);
            break;

        case b2_motorJoint:
            b2SolveMotorJoint(joint, context);
            break;

        case b2_filterJoint:
            break;

        case b2_prismaticJoint:
            b2SolvePrismaticJoint(joint, context, useBias);
            break;

        case b2_revoluteJoint:
            b2SolveRevoluteJoint(joint, context, useBias);
            break;

        case b2_weldJoint:
            b2SolveWeldJoint(joint, context, useBias);
            break;

        case b2_wheelJoint:
            b2SolveWheelJoint(joint, context, useBias);
            break;

        default:
            assert(false);
    }
}

export function b2PrepareOverflowJoints(context: b2StepContext): void {
    let graph = context.graph!;
    let jointSims = graph.colors[B2_OVERFLOW_INDEX].jointSims;
    let jointCount = jointSims.count;

    for (let i = 0; i < jointCount; ++i) {
        let joint = jointSims.get(i);
        b2PrepareJoint(joint, context);
    }
}

export function b2WarmStartOverflowJoints(context: b2StepContext): void {
    let graph = context.graph!;
    let jointSims = graph.colors[B2_OVERFLOW_INDEX].jointSims;
    let jointCount = jointSims.count;

    for (let i = 0; i < jointCount; ++i) {
        let joint = jointSims.get(i);
        b2WarmStartJoint(joint, context);
    }
}

export function b2SolveOverflowJoints(context: b2StepContext, useBias: bool): void {
    let graph = context.graph!;
    let jointSims = graph.colors[B2_OVERFLOW_INDEX].jointSims;
    let jointCount = jointSims.count;

    for (let i = 0; i < jointCount; ++i) {
        let joint = jointSims.get(i);
        b2SolveJoint(joint, context, useBias);
    }
}
