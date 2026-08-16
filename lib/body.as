// Ported from box2d src/body.h/.c. Debug draw omitted. The name char array
// becomes a string truncated to 31 chars.

import {
    b2Vec2, b2Rot, b2Transform, b2AABB, b2Sub, b2Add, b2MulSV, b2MulAdd, b2Cross, b2CrossSV,
    b2LengthSquared, b2Length, b2TransformPoint, b2InvTransformPoint, b2RotateVector,
    b2InvRotateVector, b2RelativeAngle, b2MinFloat, b2MaxFloat, b2AbsFloat, b2Dot,
    b2Vec2_zero, b2IsValidVec2, b2IsValidRotation, b2IsValidFloat, b2AABB_Union, b2AABB_Contains,
} from "./math_functions";
import { B2_NULL_INDEX, B2_SECRET_COOKIE } from "./core";
import { B2_HUGE, B2_AABB_MARGIN, B2_SPECULATIVE_DISTANCE } from "./constants";
import { b2Sweep, b2MassData } from "./collision";
import { b2BodyId, b2ShapeId, b2JointId, b2ContactId, b2WorldId } from "./id";
import { b2_staticBody, b2_kinematicBody, b2_dynamicBody } from "./broad_phase";
import { b2BodyDef, b2MotionLocks } from "./types";
import { b2AllocId, b2FreeId } from "./id_pool";
import {
    b2World, b2GetWorld, b2GetWorldFromId, b2GetWorldLocked, b2ValidateSolverSets,
    b2ValidateConnectivity, b2ValidateIsland, b2BodyMoveEvent, b2ContactData,
    b2_staticSet, b2_disabledSet, b2_awakeSet, b2_firstSleepingSet,
} from "./physics_world";
import { b2SolverSet, b2WakeSolverSet, b2TrySleepIsland, b2TransferBody, b2TransferJoint, b2DestroySolverSet } from "./solver_set";
import { b2Contact, b2ContactSim, b2DestroyContact, b2GetContactSim } from "./contact";
import { b2Shape, b2ChainShape, b2ComputeShapeAABB, b2ComputeShapeMass, b2ComputeShapeExtent,
    b2CreateShapeProxy, b2DestroyShapeProxy, b2FreeChainData } from "./shape";
import { b2Joint, b2DestroyJointInternal, b2GetJointSim } from "./joint";
import { b2Island, b2CreateIsland, b2DestroyIsland, b2LinkJoint, b2UnlinkJoint, b2SplitIsland } from "./island";
import { b2DestroySensor } from "./sensor";
import { b2BroadPhase_MoveProxy } from "./broad_phase";

export const B2_NAME_LENGTH: i32 = 32;

// enum b2BodyFlags
export const b2_lockLinearX: u32 = 0x00000001;
export const b2_lockLinearY: u32 = 0x00000002;
export const b2_lockAngularZ: u32 = 0x00000004;
export const b2_isFast: u32 = 0x00000008;
export const b2_isBullet: u32 = 0x00000010;
export const b2_isSpeedCapped: u32 = 0x00000020;
export const b2_hadTimeOfImpact: u32 = 0x00000040;
export const b2_allowFastRotation: u32 = 0x00000080;
export const b2_enlargeBounds: u32 = 0x00000100;
export const b2_dynamicFlag: u32 = 0x00000200;
export const b2_dirtyMass: u32 = 0x00000400;
export const b2_allLocks: u32 = b2_lockAngularZ | b2_lockLinearX | b2_lockLinearY;

export class b2Body {
    name: string = "";
    userData: u64 = 0;
    setIndex: i32 = 0;
    localIndex: i32 = 0;
    headContactKey: i32 = 0;
    contactCount: i32 = 0;
    headShapeId: i32 = 0;
    shapeCount: i32 = 0;
    headChainId: i32 = 0;
    headJointKey: i32 = 0;
    jointCount: i32 = 0;
    islandId: i32 = 0;
    islandPrev: i32 = 0;
    islandNext: i32 = 0;
    mass: f32 = 0;
    inertia: f32 = 0;
    sleepThreshold: f32 = 0;
    sleepTime: f32 = 0;
    bodyMoveIndex: i32 = 0;
    id: i32 = B2_NULL_INDEX;
    flags: u32 = 0;
    type: i32 = 0;
    generation: u16 = 0;
    enableSleep: bool = false;
}

// A default-constructed state IS the C identity body state (deltaRotation (1,0)).
export class b2BodyState {
    linearVelocity: b2Vec2 = new b2Vec2();
    angularVelocity: f32 = 0;
    flags: u32 = 0;
    deltaPosition: b2Vec2 = new b2Vec2();
    deltaRotation: b2Rot = new b2Rot(1, 0);
}

export class b2BodySim {
    transform: b2Transform = new b2Transform();
    center: b2Vec2 = new b2Vec2();
    rotation0: b2Rot = new b2Rot(1, 0);
    center0: b2Vec2 = new b2Vec2();
    localCenter: b2Vec2 = new b2Vec2();
    force: b2Vec2 = new b2Vec2();
    torque: f32 = 0;
    invMass: f32 = 0;
    invInertia: f32 = 0;
    minExtent: f32 = 0;
    maxExtent: f32 = 0;
    linearDamping: f32 = 0;
    angularDamping: f32 = 0;
    gravityScale: f32 = 0;
    bodyId: i32 = 0;
    flags: u32 = 0;
}

export function b2MakeSweep(bodySim: b2BodySim): b2Sweep {
    let s = new b2Sweep();
    s.c1 = bodySim.center0.clone();
    s.c2 = bodySim.center.clone();
    s.q1 = bodySim.rotation0.clone();
    s.q2 = bodySim.transform.q.clone();
    s.localCenter = bodySim.localCenter.clone();
    return s;
}

function b2LimitVelocity(state: b2BodyState, maxLinearSpeed: f32): void {
    let v2 = b2LengthSquared(state.linearVelocity);
    if (v2 > maxLinearSpeed * maxLinearSpeed) {
        state.linearVelocity = b2MulSV(maxLinearSpeed / sqrt<f32>(v2), state.linearVelocity);
    }
}

export function b2GetBodyFullId(world: b2World, bodyId: b2BodyId): b2Body {
    assert(b2Body_IsValid(bodyId));
    // id index starts at one so that zero can represent null
    return world.bodies.get(bodyId.index1 - 1);
}

export function b2GetBodyTransformQuick(world: b2World, body: b2Body): b2Transform {
    let set = world.solverSets.get(body.setIndex);
    let bodySim = set.bodySims.get(body.localIndex);
    return bodySim.transform.clone();
}

export function b2GetBodyTransform(world: b2World, bodyId: i32): b2Transform {
    let body = world.bodies.get(bodyId);
    return b2GetBodyTransformQuick(world, body);
}

export function b2MakeBodyId(world: b2World, bodyId: i32): b2BodyId {
    let body = world.bodies.get(bodyId);
    return new b2BodyId(bodyId + 1, world.worldId, body.generation);
}

export function b2GetBodySim(world: b2World, body: b2Body): b2BodySim {
    let set = world.solverSets.get(body.setIndex);
    return set.bodySims.get(body.localIndex);
}

export function b2GetBodyState(world: b2World, body: b2Body): b2BodyState | null {
    if (body.setIndex == b2_awakeSet) {
        let set = world.solverSets.get(b2_awakeSet);
        return set.bodyStates.get(body.localIndex);
    }

    return null;
}

function b2CreateIslandForBody(world: b2World, setIndex: i32, body: b2Body): void {
    assert(body.islandId == B2_NULL_INDEX);
    assert(body.islandPrev == B2_NULL_INDEX);
    assert(body.islandNext == B2_NULL_INDEX);
    assert(setIndex != b2_disabledSet);

    let island = b2CreateIsland(world, setIndex);

    body.islandId = island.islandId;
    island.headBody = body.id;
    island.tailBody = body.id;
    island.bodyCount = 1;
}

function b2RemoveBodyFromIsland(world: b2World, body: b2Body): void {
    if (body.islandId == B2_NULL_INDEX) {
        assert(body.islandPrev == B2_NULL_INDEX);
        assert(body.islandNext == B2_NULL_INDEX);
        return;
    }

    let islandId = body.islandId;
    let island = world.islands.get(islandId);

    // Fix the island's linked list of sims
    if (body.islandPrev != B2_NULL_INDEX) {
        let prevBody = world.bodies.get(body.islandPrev);
        prevBody.islandNext = body.islandNext;
    }

    if (body.islandNext != B2_NULL_INDEX) {
        let nextBody = world.bodies.get(body.islandNext);
        nextBody.islandPrev = body.islandPrev;
    }

    assert(island.bodyCount > 0);
    island.bodyCount -= 1;
    let islandDestroyed = false;

    if (island.headBody == body.id) {
        island.headBody = body.islandNext;

        if (island.headBody == B2_NULL_INDEX) {
            // Destroy empty island
            assert(island.tailBody == body.id);
            assert(island.bodyCount == 0);
            assert(island.contactCount == 0);
            assert(island.jointCount == 0);

            b2DestroyIsland(world, island.islandId);
            islandDestroyed = true;
        }
    } else if (island.tailBody == body.id) {
        island.tailBody = body.islandPrev;
    }

    if (islandDestroyed == false) {
        b2ValidateIsland(world, islandId);
    }

    body.islandId = B2_NULL_INDEX;
    body.islandPrev = B2_NULL_INDEX;
    body.islandNext = B2_NULL_INDEX;
}

function b2DestroyBodyContacts(world: b2World, body: b2Body, wakeBodies: bool): void {
    // Destroy the attached contacts
    let edgeKey = body.headContactKey;
    while (edgeKey != B2_NULL_INDEX) {
        let contactId = edgeKey >> 1;
        let edgeIndex = edgeKey & 1;

        let contact = world.contacts.get(contactId);
        edgeKey = contact.edges[edgeIndex].nextKey;
        b2DestroyContact(world, contact, wakeBodies);
    }

    b2ValidateSolverSets(world);
}

export function b2CreateBody(worldId: b2WorldId, def: b2BodyDef): b2BodyId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    assert(b2IsValidVec2(def.position));
    assert(b2IsValidRotation(def.rotation));
    assert(b2IsValidVec2(def.linearVelocity));
    assert(b2IsValidFloat(def.angularVelocity));
    assert(b2IsValidFloat(def.linearDamping) && def.linearDamping >= 0.0);
    assert(b2IsValidFloat(def.angularDamping) && def.angularDamping >= 0.0);
    assert(b2IsValidFloat(def.sleepThreshold) && def.sleepThreshold >= 0.0);
    assert(b2IsValidFloat(def.gravityScale));

    let world = b2GetWorldFromId(worldId);
    assert(world.locked == false);

    if (world.locked) {
        return new b2BodyId(0, 0, 0);
    }

    let isAwake = (def.isAwake || def.enableSleep == false) && def.isEnabled;

    // determine the solver set
    let setId: i32;
    if (def.isEnabled == false) {
        // any body type can be disabled
        setId = b2_disabledSet;
    } else if (def.type == b2_staticBody) {
        setId = b2_staticSet;
    } else if (isAwake == true) {
        setId = b2_awakeSet;
    } else {
        // new set for a sleeping body in its own island
        setId = b2AllocId(world.solverSetIdPool);
        if (setId == world.solverSets.count) {
            // Create a zero initialized solver set. All sub-arrays are also zero initialized.
            let newSet = world.solverSets.add();
            newSet.setIndex = 0;
        } else {
            assert(world.solverSets.get(setId).setIndex == B2_NULL_INDEX);
        }

        world.solverSets.get(setId).setIndex = setId;
    }

    assert(0 <= setId && setId < world.solverSets.count);

    let bodyId = b2AllocId(world.bodyIdPool);

    let lockFlags: u32 = 0;
    lockFlags |= def.motionLocks.linearX ? b2_lockLinearX : 0;
    lockFlags |= def.motionLocks.linearY ? b2_lockLinearY : 0;
    lockFlags |= def.motionLocks.angularZ ? b2_lockAngularZ : 0;

    let set = world.solverSets.get(setId);
    let bodySim = set.bodySims.add();
    bodySim.transform.p = def.position.clone();
    bodySim.transform.q = def.rotation.clone();
    bodySim.center = def.position.clone();
    bodySim.rotation0 = bodySim.transform.q.clone();
    bodySim.center0 = bodySim.center.clone();
    bodySim.localCenter = new b2Vec2();
    bodySim.force = new b2Vec2();
    bodySim.torque = 0.0;
    bodySim.invMass = 0.0;
    bodySim.invInertia = 0.0;
    bodySim.minExtent = B2_HUGE();
    bodySim.maxExtent = 0.0;
    bodySim.linearDamping = def.linearDamping;
    bodySim.angularDamping = def.angularDamping;
    bodySim.gravityScale = def.gravityScale;
    bodySim.bodyId = bodyId;
    bodySim.flags = lockFlags;
    bodySim.flags |= def.isBullet ? b2_isBullet : 0;
    bodySim.flags |= def.allowFastRotation ? b2_allowFastRotation : 0;
    bodySim.flags |= def.type == b2_dynamicBody ? b2_dynamicFlag : 0;

    if (setId == b2_awakeSet) {
        let bodyState = set.bodyStates.add();
        bodyState.linearVelocity = def.linearVelocity.clone();
        bodyState.angularVelocity = def.angularVelocity;
        bodyState.deltaRotation = new b2Rot(1, 0);
        bodyState.flags = bodySim.flags;
    }

    if (bodyId == world.bodies.count) {
        world.bodies.add();
    } else {
        assert(world.bodies.get(bodyId).id == B2_NULL_INDEX);
    }

    let body = world.bodies.get(bodyId);

    if (def.name.length > 0) {
        body.name = def.name.length > B2_NAME_LENGTH - 1 ? def.name.substring(0, B2_NAME_LENGTH - 1) : def.name;
    } else {
        body.name = "";
    }

    body.userData = def.userData;
    body.setIndex = setId;
    body.localIndex = set.bodySims.count - 1;
    body.generation += 1;
    body.headShapeId = B2_NULL_INDEX;
    body.shapeCount = 0;
    body.headChainId = B2_NULL_INDEX;
    body.headContactKey = B2_NULL_INDEX;
    body.contactCount = 0;
    body.headJointKey = B2_NULL_INDEX;
    body.jointCount = 0;
    body.islandId = B2_NULL_INDEX;
    body.islandPrev = B2_NULL_INDEX;
    body.islandNext = B2_NULL_INDEX;
    body.bodyMoveIndex = B2_NULL_INDEX;
    body.id = bodyId;
    body.mass = 0.0;
    body.inertia = 0.0;
    body.sleepThreshold = def.sleepThreshold;
    body.sleepTime = 0.0;
    body.type = def.type;
    body.flags = bodySim.flags;
    body.enableSleep = def.enableSleep;

    // dynamic and kinematic bodies that are enabled need a island
    if (setId >= b2_awakeSet) {
        b2CreateIslandForBody(world, setId, body);
    }

    b2ValidateSolverSets(world);

    return new b2BodyId(bodyId + 1, world.worldId, body.generation);
}

// careful calling this because it can invalidate body, state, joint, and contact references
export function b2WakeBody(world: b2World, body: b2Body): bool {
    if (body.setIndex >= b2_firstSleepingSet) {
        b2WakeSolverSet(world, body.setIndex);
        b2ValidateSolverSets(world);
        return true;
    }

    return false;
}

export function b2DestroyBody(bodyId: b2BodyId): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);

    // Wake bodies attached to this body, even if this body is static.
    let wakeBodies = true;

    // Destroy the attached joints
    let edgeKey = body.headJointKey;
    while (edgeKey != B2_NULL_INDEX) {
        let jointId = edgeKey >> 1;
        let edgeIndex = edgeKey & 1;

        let joint = world.joints.get(jointId);
        edgeKey = joint.edges[edgeIndex].nextKey;

        // Careful because this modifies the list being traversed
        b2DestroyJointInternal(world, joint, wakeBodies);
    }

    // Destroy all contacts attached to this body.
    b2DestroyBodyContacts(world, body, wakeBodies);

    // Destroy the attached shapes and their broad-phase proxies.
    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);

        if (shape.sensorIndex != B2_NULL_INDEX) {
            b2DestroySensor(world, shape);
        }

        b2DestroyShapeProxy(shape, world.broadPhase);

        // Return shape to free list.
        b2FreeId(world.shapeIdPool, shapeId);
        shape.id = B2_NULL_INDEX;

        shapeId = shape.nextShapeId;
    }

    // Destroy the attached chains. The associated shapes have already been destroyed above.
    let chainId = body.headChainId;
    while (chainId != B2_NULL_INDEX) {
        let chain = world.chainShapes.get(chainId);

        b2FreeChainData(chain);

        // Return chain to free list.
        b2FreeId(world.chainIdPool, chainId);
        chain.id = B2_NULL_INDEX;

        chainId = chain.nextChainId;
    }

    b2RemoveBodyFromIsland(world, body);

    // Remove body sim from solver set that owns it
    let set = world.solverSets.get(body.setIndex);
    let movedIndex = set.bodySims.removeSwap(body.localIndex);
    if (movedIndex != B2_NULL_INDEX) {
        // Fix moved body index
        let movedSim = set.bodySims.get(body.localIndex);
        let movedId = movedSim.bodyId;
        let movedBody = world.bodies.get(movedId);
        assert(movedBody.localIndex == movedIndex);
        movedBody.localIndex = body.localIndex;
    }

    // Remove body state from awake set
    if (body.setIndex == b2_awakeSet) {
        let result = set.bodyStates.removeSwap(body.localIndex);
        assert(result == movedIndex);
    } else if (set.setIndex >= b2_firstSleepingSet && set.bodySims.count == 0) {
        // Remove solver set if it's now an orphan.
        b2DestroySolverSet(world, set.setIndex);
    }

    // Free body and id (preserve body generation)
    b2FreeId(world.bodyIdPool, body.id);

    body.setIndex = B2_NULL_INDEX;
    body.localIndex = B2_NULL_INDEX;
    body.id = B2_NULL_INDEX;

    b2ValidateSolverSets(world);
}

export function b2Body_GetContactCapacity(bodyId: b2BodyId): i32 {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return 0;
    }

    let body = b2GetBodyFullId(world, bodyId);
    // Conservative and fast
    return body.contactCount;
}

export function b2Body_GetContactData(bodyId: b2BodyId, contactData: Array<b2ContactData>, capacity: i32): i32 {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return 0;
    }

    let body = b2GetBodyFullId(world, bodyId);

    let contactKey = body.headContactKey;
    let index = 0;
    while (contactKey != B2_NULL_INDEX && index < capacity) {
        let contactId = contactKey >> 1;
        let edgeIndex = contactKey & 1;

        let contact = world.contacts.get(contactId);

        // Is contact touching?
        if ((contact.flags & 0x00000001) != 0) { // b2_contactTouchingFlag
            let shapeA = world.shapes.get(contact.shapeIdA);
            let shapeB = world.shapes.get(contact.shapeIdB);

            let data = contactData[index];
            data.contactId = new b2ContactId(contact.contactId + 1, bodyId.world0, contact.generation);
            data.shapeIdA = new b2ShapeId(shapeA.id + 1, bodyId.world0, shapeA.generation);
            data.shapeIdB = new b2ShapeId(shapeB.id + 1, bodyId.world0, shapeB.generation);

            let contactSim = b2GetContactSim(world, contact);
            data.manifold = contactSim.manifold;

            index += 1;
        }

        contactKey = contact.edges[edgeIndex].nextKey;
    }

    assert(index <= capacity);

    return index;
}

export function b2Body_ComputeAABB(bodyId: b2BodyId): b2AABB {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return new b2AABB();
    }

    let body = b2GetBodyFullId(world, bodyId);
    if (body.headShapeId == B2_NULL_INDEX) {
        let transform = b2GetBodyTransform(world, body.id);
        return new b2AABB(transform.p.clone(), transform.p.clone());
    }

    let shape = world.shapes.get(body.headShapeId);
    let aabb = shape.aabb.clone();
    while (shape.nextShapeId != B2_NULL_INDEX) {
        shape = world.shapes.get(shape.nextShapeId);
        aabb = b2AABB_Union(aabb, shape.aabb);
    }

    return aabb;
}

export function b2UpdateBodyMassData(world: b2World, body: b2Body): void {
    let bodySim = b2GetBodySim(world, body);

    // Mass is no longer dirty
    body.flags &= ~b2_dirtyMass;

    // Compute mass data from shapes. Each shape has its own density.
    body.mass = 0.0;
    body.inertia = 0.0;

    bodySim.invMass = 0.0;
    bodySim.invInertia = 0.0;
    bodySim.localCenter = b2Vec2_zero();
    bodySim.minExtent = B2_HUGE();
    bodySim.maxExtent = 0.0;

    // Static and kinematic sims have zero mass.
    if (body.type != b2_dynamicBody) {
        bodySim.center = bodySim.transform.p.clone();
        bodySim.center0 = bodySim.center.clone();

        // Need extents for kinematic bodies for sleeping to work correctly.
        if (body.type == b2_kinematicBody) {
            let shapeId = body.headShapeId;
            while (shapeId != B2_NULL_INDEX) {
                let s = world.shapes.get(shapeId);

                let extent = b2ComputeShapeExtent(s, b2Vec2_zero());
                bodySim.minExtent = b2MinFloat(bodySim.minExtent, extent.minExtent);
                bodySim.maxExtent = b2MaxFloat(bodySim.maxExtent, extent.maxExtent);

                shapeId = s.nextShapeId;
            }
        }

        return;
    }

    let shapeCount = body.shapeCount;
    let masses = new Array<b2MassData>(shapeCount);

    // Accumulate mass over all shapes.
    let localCenter = b2Vec2_zero();
    let shapeId = body.headShapeId;
    let shapeIndex = 0;
    while (shapeId != B2_NULL_INDEX) {
        let s = world.shapes.get(shapeId);
        shapeId = s.nextShapeId;

        if (s.density == 0.0) {
            masses[shapeIndex] = new b2MassData();
            shapeIndex += 1;
            continue;
        }

        let massData = b2ComputeShapeMass(s);
        body.mass += massData.mass;
        localCenter = b2MulAdd(localCenter, massData.mass, massData.center);

        masses[shapeIndex] = massData;
        shapeIndex += 1;
    }

    // Compute center of mass.
    if (body.mass > 0.0) {
        bodySim.invMass = <f32>1.0 / body.mass;
        localCenter = b2MulSV(bodySim.invMass, localCenter);
    }

    // Second loop to accumulate the rotational inertia about the center of mass
    for (shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex) {
        let massData = masses[shapeIndex];
        if (massData.mass == 0.0) {
            continue;
        }

        // Shift to center of mass. This is safe because it can only increase.
        let offset = b2Sub(localCenter, massData.center);
        let inertia = massData.rotationalInertia + massData.mass * b2Dot(offset, offset);
        body.inertia += inertia;
    }

    assert(body.inertia >= 0.0);

    if (body.inertia > 0.0) {
        bodySim.invInertia = <f32>1.0 / body.inertia;
    } else {
        body.inertia = 0.0;
        bodySim.invInertia = 0.0;
    }

    // Move center of mass.
    let oldCenter = bodySim.center.clone();
    bodySim.localCenter = localCenter;
    bodySim.center = b2TransformPoint(bodySim.transform, bodySim.localCenter);
    bodySim.center0 = bodySim.center.clone();

    // Update center of mass velocity
    let state = b2GetBodyState(world, body);
    if (state != null) {
        let deltaLinear = b2CrossSV(state.angularVelocity, b2Sub(bodySim.center, oldCenter));
        state.linearVelocity = b2Add(state.linearVelocity, deltaLinear);
    }

    // Compute body extents relative to center of mass
    shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let s = world.shapes.get(shapeId);

        let extent = b2ComputeShapeExtent(s, localCenter);
        bodySim.minExtent = b2MinFloat(bodySim.minExtent, extent.minExtent);
        bodySim.maxExtent = b2MaxFloat(bodySim.maxExtent, extent.maxExtent);

        shapeId = s.nextShapeId;
    }
}

export function b2Body_GetPosition(bodyId: b2BodyId): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);
    return transform.p;
}

export function b2Body_GetRotation(bodyId: b2BodyId): b2Rot {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);
    return transform.q;
}

export function b2Body_GetTransform(bodyId: b2BodyId): b2Transform {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return b2GetBodyTransformQuick(world, body);
}

export function b2Body_GetLocalPoint(bodyId: b2BodyId, worldPoint: b2Vec2): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);
    return b2InvTransformPoint(transform, worldPoint);
}

export function b2Body_GetWorldPoint(bodyId: b2BodyId, localPoint: b2Vec2): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);
    return b2TransformPoint(transform, localPoint);
}

export function b2Body_GetLocalVector(bodyId: b2BodyId, worldVector: b2Vec2): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);
    return b2InvRotateVector(transform.q, worldVector);
}

export function b2Body_GetWorldVector(bodyId: b2BodyId, localVector: b2Vec2): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);
    return b2RotateVector(transform.q, localVector);
}

export function b2Body_SetTransform(bodyId: b2BodyId, position: b2Vec2, rotation: b2Rot): void {
    assert(b2IsValidVec2(position));
    assert(b2IsValidRotation(rotation));
    assert(b2Body_IsValid(bodyId));
    let world = b2GetWorld(bodyId.world0);
    assert(world.locked == false);

    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);

    bodySim.transform.p = position.clone();
    bodySim.transform.q = rotation.clone();
    bodySim.center = b2TransformPoint(bodySim.transform, bodySim.localCenter);

    bodySim.rotation0 = bodySim.transform.q.clone();
    bodySim.center0 = bodySim.center.clone();

    let broadPhase = world.broadPhase;

    let transform = bodySim.transform;
    let margin = B2_AABB_MARGIN();
    let speculativeDistance = B2_SPECULATIVE_DISTANCE();

    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);
        let aabb = b2ComputeShapeAABB(shape, transform);
        aabb.lowerBound.x -= speculativeDistance;
        aabb.lowerBound.y -= speculativeDistance;
        aabb.upperBound.x += speculativeDistance;
        aabb.upperBound.y += speculativeDistance;
        shape.aabb = aabb;

        if (b2AABB_Contains(shape.fatAABB, aabb) == false) {
            let fatAABB = new b2AABB();
            fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
            fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
            fatAABB.upperBound.x = aabb.upperBound.x + margin;
            fatAABB.upperBound.y = aabb.upperBound.y + margin;
            shape.fatAABB = fatAABB;

            // The body could be disabled
            if (shape.proxyKey != B2_NULL_INDEX) {
                b2BroadPhase_MoveProxy(broadPhase, shape.proxyKey, fatAABB);
            }
        }

        shapeId = shape.nextShapeId;
    }
}

export function b2Body_GetLinearVelocity(bodyId: b2BodyId): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let state = b2GetBodyState(world, body);
    if (state != null) {
        return state.linearVelocity.clone();
    }
    return b2Vec2_zero();
}

export function b2Body_GetAngularVelocity(bodyId: b2BodyId): f32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let state = b2GetBodyState(world, body);
    if (state != null) {
        return state.angularVelocity;
    }
    return 0.0;
}

export function b2Body_SetLinearVelocity(bodyId: b2BodyId, linearVelocity: b2Vec2): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type == b2_staticBody) {
        return;
    }

    if (b2LengthSquared(linearVelocity) > 0.0) {
        b2WakeBody(world, body);
    }

    let state = b2GetBodyState(world, body);
    if (state == null) {
        return;
    }

    state.linearVelocity = linearVelocity.clone();
}

export function b2Body_SetAngularVelocity(bodyId: b2BodyId, angularVelocity: f32): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type == b2_staticBody || (body.flags & b2_lockAngularZ) != 0) {
        return;
    }

    if (angularVelocity != 0.0) {
        b2WakeBody(world, body);
    }

    let state = b2GetBodyState(world, body);
    if (state == null) {
        return;
    }

    state.angularVelocity = angularVelocity;
}

export function b2Body_SetTargetTransform(bodyId: b2BodyId, target: b2Transform, timeStep: f32, wake: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.setIndex == b2_disabledSet) {
        return;
    }

    if (body.type == b2_staticBody || timeStep <= 0.0) {
        return;
    }

    if (body.setIndex != b2_awakeSet && wake == false) {
        return;
    }

    let sim = b2GetBodySim(world, body);

    // Compute linear velocity
    let center1 = sim.center;
    let center2 = b2TransformPoint(target, sim.localCenter);
    let invTimeStep = <f32>1.0 / timeStep;
    let linearVelocity = b2MulSV(invTimeStep, b2Sub(center2, center1));

    // Compute angular velocity
    let q1 = sim.transform.q;
    let q2 = target.q;
    let deltaAngle = b2RelativeAngle(q1, q2);
    let angularVelocity = invTimeStep * deltaAngle;

    // Early out if the body is asleep already and the desired movement is small
    if (body.setIndex != b2_awakeSet) {
        let maxVelocity = b2Length(linearVelocity) + b2AbsFloat(angularVelocity) * sim.maxExtent;

        // Return if velocity would be sleepy
        if (maxVelocity < body.sleepThreshold) {
            return;
        }

        // Must wake for state to exist
        b2WakeBody(world, body);
    }

    assert(body.setIndex == b2_awakeSet);

    let state = b2GetBodyState(world, body)!;
    state.linearVelocity = linearVelocity;
    state.angularVelocity = angularVelocity;
}

export function b2Body_GetLocalPointVelocity(bodyId: b2BodyId, localPoint: b2Vec2): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let state = b2GetBodyState(world, body);
    if (state == null) {
        return b2Vec2_zero();
    }

    let set = world.solverSets.get(body.setIndex);
    let bodySim = set.bodySims.get(body.localIndex);

    let r = b2RotateVector(bodySim.transform.q, b2Sub(localPoint, bodySim.localCenter));
    let v = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, r));
    return v;
}

export function b2Body_GetWorldPointVelocity(bodyId: b2BodyId, worldPoint: b2Vec2): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let state = b2GetBodyState(world, body);
    if (state == null) {
        return b2Vec2_zero();
    }

    let set = world.solverSets.get(body.setIndex);
    let bodySim = set.bodySims.get(body.localIndex);

    let r = b2Sub(worldPoint, bodySim.center);
    let v = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, r));
    return v;
}

export function b2Body_ApplyForce(bodyId: b2BodyId, force: b2Vec2, point: b2Vec2, wake: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type != b2_dynamicBody || body.setIndex == b2_disabledSet) {
        return;
    }

    if (wake && body.setIndex >= b2_firstSleepingSet) {
        b2WakeBody(world, body);
    }

    if (body.setIndex == b2_awakeSet) {
        let bodySim = b2GetBodySim(world, body);
        bodySim.force = b2Add(bodySim.force, force);
        bodySim.torque += b2Cross(b2Sub(point, bodySim.center), force);
    }
}

export function b2Body_ApplyForceToCenter(bodyId: b2BodyId, force: b2Vec2, wake: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type != b2_dynamicBody || body.setIndex == b2_disabledSet) {
        return;
    }

    if (wake && body.setIndex >= b2_firstSleepingSet) {
        b2WakeBody(world, body);
    }

    if (body.setIndex == b2_awakeSet) {
        let bodySim = b2GetBodySim(world, body);
        bodySim.force = b2Add(bodySim.force, force);
    }
}

export function b2Body_ApplyTorque(bodyId: b2BodyId, torque: f32, wake: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type != b2_dynamicBody || body.setIndex == b2_disabledSet) {
        return;
    }

    if (wake && body.setIndex >= b2_firstSleepingSet) {
        b2WakeBody(world, body);
    }

    if (body.setIndex == b2_awakeSet) {
        let bodySim = b2GetBodySim(world, body);
        bodySim.torque += torque;
    }
}

export function b2Body_ClearForces(bodyId: b2BodyId): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    bodySim.force = b2Vec2_zero();
    bodySim.torque = 0.0;
}

export function b2Body_ApplyLinearImpulse(bodyId: b2BodyId, impulse: b2Vec2, point: b2Vec2, wake: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type != b2_dynamicBody || body.setIndex == b2_disabledSet) {
        return;
    }

    if (wake && body.setIndex >= b2_firstSleepingSet) {
        b2WakeBody(world, body);
    }

    if (body.setIndex == b2_awakeSet) {
        let localIndex = body.localIndex;
        let set = world.solverSets.get(b2_awakeSet);
        let state = set.bodyStates.get(localIndex);
        let bodySim = set.bodySims.get(localIndex);
        state.linearVelocity = b2MulAdd(state.linearVelocity, bodySim.invMass, impulse);
        state.angularVelocity += bodySim.invInertia * b2Cross(b2Sub(point, bodySim.center), impulse);

        b2LimitVelocity(state, world.maxLinearSpeed);
    }
}

export function b2Body_ApplyLinearImpulseToCenter(bodyId: b2BodyId, impulse: b2Vec2, wake: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type != b2_dynamicBody || body.setIndex == b2_disabledSet) {
        return;
    }

    if (wake && body.setIndex >= b2_firstSleepingSet) {
        b2WakeBody(world, body);
    }

    if (body.setIndex == b2_awakeSet) {
        let localIndex = body.localIndex;
        let set = world.solverSets.get(b2_awakeSet);
        let state = set.bodyStates.get(localIndex);
        let bodySim = set.bodySims.get(localIndex);
        state.linearVelocity = b2MulAdd(state.linearVelocity, bodySim.invMass, impulse);

        b2LimitVelocity(state, world.maxLinearSpeed);
    }
}

export function b2Body_ApplyAngularImpulse(bodyId: b2BodyId, impulse: f32, wake: bool): void {
    assert(b2Body_IsValid(bodyId));
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (body.type != b2_dynamicBody || body.setIndex == b2_disabledSet) {
        return;
    }

    if (wake && body.setIndex >= b2_firstSleepingSet) {
        // this will not invalidate body reference
        b2WakeBody(world, body);
    }

    if (body.setIndex == b2_awakeSet) {
        let localIndex = body.localIndex;
        let set = world.solverSets.get(b2_awakeSet);
        let state = set.bodyStates.get(localIndex);
        let bodySim = set.bodySims.get(localIndex);
        state.angularVelocity += bodySim.invInertia * impulse;
    }
}

export function b2Body_GetType(bodyId: b2BodyId): i32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.type;
}

export function b2Body_SetType(bodyId: b2BodyId, type: i32): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    let originalType = body.type;
    if (originalType == type) {
        return;
    }

    // Stage 1: skip disabled bodies
    if (body.setIndex == b2_disabledSet) {
        // Disabled bodies don't change solver sets or islands when they change type.
        body.type = type;

        if (type == b2_dynamicBody) {
            body.flags |= b2_dynamicFlag;
        } else {
            body.flags &= ~b2_dynamicFlag;
        }

        // Body type affects the mass properties
        b2UpdateBodyMassData(world, body);
        return;
    }

    // Stage 2: destroy all contacts but don't wake bodies (because we don't need to)
    let wakeBodies = false;
    b2DestroyBodyContacts(world, body, wakeBodies);

    // Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
    // all bodies in the same sleeping solver set.
    b2WakeBody(world, body);

    // Stage 4: move joints to temporary storage
    let staticSet = world.solverSets.get(b2_staticSet);

    let jointKey = body.headJointKey;
    while (jointKey != B2_NULL_INDEX) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;

        let joint = world.joints.get(jointId);
        jointKey = joint.edges[edgeIndex].nextKey;

        // Joint may be disabled by other body
        if (joint.setIndex == b2_disabledSet) {
            continue;
        }

        // Wake attached bodies.
        let bodyA = world.bodies.get(joint.edges[0].bodyId);
        let bodyB = world.bodies.get(joint.edges[1].bodyId);
        b2WakeBody(world, bodyA);
        b2WakeBody(world, bodyB);

        // Remove joint from island
        b2UnlinkJoint(world, joint);

        // It is necessary to transfer all joints to the static set
        // so they can be added to the constraint graph below and acquire consistent colors.
        let jointSourceSet = world.solverSets.get(joint.setIndex);
        b2TransferJoint(world, staticSet, jointSourceSet, joint);
    }

    // Stage 5: change the body type and transfer body
    body.type = type;

    if (type == b2_dynamicBody) {
        body.flags |= b2_dynamicFlag;
    } else {
        body.flags &= ~b2_dynamicFlag;
    }

    let awakeSet = world.solverSets.get(b2_awakeSet);
    let sourceSet = world.solverSets.get(body.setIndex);
    let targetSet = type == b2_staticBody ? staticSet : awakeSet;

    // Transfer body
    b2TransferBody(world, targetSet, sourceSet, body);

    // Stage 6: update island participation for the body
    if (originalType == b2_staticBody) {
        // Create island for body
        b2CreateIslandForBody(world, b2_awakeSet, body);
    } else if (type == b2_staticBody) {
        // Remove body from island.
        b2RemoveBodyFromIsland(world, body);
    }

    // Stage 7: Transfer joints to the target set
    jointKey = body.headJointKey;
    while (jointKey != B2_NULL_INDEX) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;

        let joint = world.joints.get(jointId);

        jointKey = joint.edges[edgeIndex].nextKey;

        // Joint may be disabled by other body
        if (joint.setIndex == b2_disabledSet) {
            continue;
        }

        // All joints were transferred to the static set in an earlier stage
        assert(joint.setIndex == b2_staticSet);

        let bodyA = world.bodies.get(joint.edges[0].bodyId);
        let bodyB = world.bodies.get(joint.edges[1].bodyId);
        assert(bodyA.setIndex == b2_staticSet || bodyA.setIndex == b2_awakeSet);
        assert(bodyB.setIndex == b2_staticSet || bodyB.setIndex == b2_awakeSet);

        if (bodyA.type == b2_dynamicBody || bodyB.type == b2_dynamicBody) {
            b2TransferJoint(world, awakeSet, staticSet, joint);
        }
    }

    // Recreate shape proxies in broadphase
    let transform = b2GetBodyTransformQuick(world, body);
    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);
        shapeId = shape.nextShapeId;
        b2DestroyShapeProxy(shape, world.broadPhase);
        let forcePairCreation = true;
        b2CreateShapeProxy(shape, world.broadPhase, type, transform, forcePairCreation);
    }

    // Relink all joints
    jointKey = body.headJointKey;
    while (jointKey != B2_NULL_INDEX) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;

        let joint = world.joints.get(jointId);
        jointKey = joint.edges[edgeIndex].nextKey;

        let otherEdgeIndex = edgeIndex ^ 1;
        let otherBodyId = joint.edges[otherEdgeIndex].bodyId;
        let otherBody = world.bodies.get(otherBodyId);

        if (otherBody.setIndex == b2_disabledSet) {
            continue;
        }

        if (body.type != b2_dynamicBody && otherBody.type != b2_dynamicBody) {
            continue;
        }

        b2LinkJoint(world, joint);
    }

    // Body type affects the mass
    b2UpdateBodyMassData(world, body);

    let state = b2GetBodyState(world, body);
    if (state != null) {
        // Ensure flags are in sync
        state.flags = body.flags;
    }

    b2ValidateSolverSets(world);
    b2ValidateIsland(world, body.islandId);
}

export function b2Body_SetName(bodyId: b2BodyId, name: string): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    if (name.length > 0) {
        body.name = name.length > B2_NAME_LENGTH - 1 ? name.substring(0, B2_NAME_LENGTH - 1) : name;
    } else {
        body.name = "";
    }
}

export function b2Body_GetName(bodyId: b2BodyId): string {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.name;
}

export function b2Body_SetUserData(bodyId: b2BodyId, userData: u64): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    body.userData = userData;
}

export function b2Body_GetUserData(bodyId: b2BodyId): u64 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.userData;
}

export function b2Body_GetMass(bodyId: b2BodyId): f32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.mass;
}

export function b2Body_GetRotationalInertia(bodyId: b2BodyId): f32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.inertia;
}

export function b2Body_GetLocalCenterOfMass(bodyId: b2BodyId): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    return bodySim.localCenter.clone();
}

export function b2Body_GetWorldCenterOfMass(bodyId: b2BodyId): b2Vec2 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    return bodySim.center.clone();
}

export function b2Body_SetMassData(bodyId: b2BodyId, massData: b2MassData): void {
    assert(b2IsValidFloat(massData.mass) && massData.mass >= 0.0);
    assert(b2IsValidFloat(massData.rotationalInertia) && massData.rotationalInertia >= 0.0);
    assert(b2IsValidVec2(massData.center));

    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);

    body.mass = massData.mass;
    body.inertia = massData.rotationalInertia;
    bodySim.localCenter = massData.center.clone();

    let center = b2TransformPoint(bodySim.transform, massData.center);
    bodySim.center = center;
    bodySim.center0 = center.clone();

    bodySim.invMass = body.mass > 0.0 ? <f32>1.0 / body.mass : 0.0;
    bodySim.invInertia = body.inertia > 0.0 ? <f32>1.0 / body.inertia : 0.0;
}

export function b2Body_GetMassData(bodyId: b2BodyId): b2MassData {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    let massData = new b2MassData();
    massData.mass = body.mass;
    massData.center = bodySim.localCenter.clone();
    massData.rotationalInertia = body.inertia;
    return massData;
}

export function b2Body_ApplyMassFromShapes(bodyId: b2BodyId): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    b2UpdateBodyMassData(world, body);
}

export function b2Body_SetLinearDamping(bodyId: b2BodyId, linearDamping: f32): void {
    assert(b2IsValidFloat(linearDamping) && linearDamping >= 0.0);

    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    bodySim.linearDamping = linearDamping;
}

export function b2Body_GetLinearDamping(bodyId: b2BodyId): f32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    return bodySim.linearDamping;
}

export function b2Body_SetAngularDamping(bodyId: b2BodyId, angularDamping: f32): void {
    assert(b2IsValidFloat(angularDamping) && angularDamping >= 0.0);

    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    bodySim.angularDamping = angularDamping;
}

export function b2Body_GetAngularDamping(bodyId: b2BodyId): f32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    return bodySim.angularDamping;
}

export function b2Body_SetGravityScale(bodyId: b2BodyId, gravityScale: f32): void {
    assert(b2Body_IsValid(bodyId));
    assert(b2IsValidFloat(gravityScale));

    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    bodySim.gravityScale = gravityScale;
}

export function b2Body_GetGravityScale(bodyId: b2BodyId): f32 {
    assert(b2Body_IsValid(bodyId));
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    return bodySim.gravityScale;
}

export function b2Body_IsAwake(bodyId: b2BodyId): bool {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.setIndex == b2_awakeSet;
}

export function b2Body_SetAwake(bodyId: b2BodyId, awake: bool): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);

    if (awake && body.setIndex >= b2_firstSleepingSet) {
        b2WakeBody(world, body);
    } else if (awake == false && body.setIndex == b2_awakeSet) {
        let island = world.islands.get(body.islandId);
        if (island.constraintRemoveCount > 0) {
            // Must split the island before sleeping. This is expensive.
            b2SplitIsland(world, body.islandId);
        }

        b2TrySleepIsland(world, body.islandId);
    }
}

export function b2Body_WakeTouching(bodyId: b2BodyId): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    let contactKey = body.headContactKey;
    while (contactKey != B2_NULL_INDEX) {
        let contactId = contactKey >> 1;
        let edgeIndex = contactKey & 1;

        let contact = world.contacts.get(contactId);
        let shapeA = world.shapes.get(contact.shapeIdA);
        let shapeB = world.shapes.get(contact.shapeIdB);

        if (shapeA.bodyId == bodyId.index1 - 1) {
            let otherBody = world.bodies.get(shapeB.bodyId);
            b2WakeBody(world, otherBody);
        } else {
            let otherBody = world.bodies.get(shapeA.bodyId);
            b2WakeBody(world, otherBody);
        }

        contactKey = contact.edges[edgeIndex].nextKey;
    }
}

export function b2Body_IsEnabled(bodyId: b2BodyId): bool {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.setIndex != b2_disabledSet;
}

export function b2Body_IsSleepEnabled(bodyId: b2BodyId): bool {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.enableSleep;
}

export function b2Body_SetSleepThreshold(bodyId: b2BodyId, sleepThreshold: f32): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    body.sleepThreshold = sleepThreshold;
}

export function b2Body_GetSleepThreshold(bodyId: b2BodyId): f32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.sleepThreshold;
}

export function b2Body_EnableSleep(bodyId: b2BodyId, enableSleep: bool): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    body.enableSleep = enableSleep;

    if (enableSleep == false) {
        b2WakeBody(world, body);
    }
}

export function b2Body_Disable(bodyId: b2BodyId): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    if (body.setIndex == b2_disabledSet) {
        return;
    }

    // Destroy contacts and wake bodies touching this body. This avoid floating bodies.
    // This is necessary even for static bodies.
    let wakeBodies = true;
    b2DestroyBodyContacts(world, body, wakeBodies);

    // The current solver set of the body
    let set = world.solverSets.get(body.setIndex);

    // Disabled bodies and connected joints are moved to the disabled set
    let disabledSet = world.solverSets.get(b2_disabledSet);

    // Unlink joints and transfer them to the disabled set
    let jointKey = body.headJointKey;
    while (jointKey != B2_NULL_INDEX) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;

        let joint = world.joints.get(jointId);
        jointKey = joint.edges[edgeIndex].nextKey;

        // joint may already be disabled by other body
        if (joint.setIndex == b2_disabledSet) {
            continue;
        }

        assert(joint.setIndex == set.setIndex || set.setIndex == b2_staticSet);

        // Remove joint from island
        b2UnlinkJoint(world, joint);

        // Transfer joint to disabled set
        let jointSet = world.solverSets.get(joint.setIndex);
        b2TransferJoint(world, disabledSet, jointSet, joint);
    }

    // Remove shapes from broad-phase
    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);
        shapeId = shape.nextShapeId;
        b2DestroyShapeProxy(shape, world.broadPhase);
    }

    // Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
    b2RemoveBodyFromIsland(world, body);

    // Transfer body sim
    b2TransferBody(world, disabledSet, set, body);

    b2ValidateConnectivity(world);
    b2ValidateSolverSets(world);
}

export function b2Body_Enable(bodyId: b2BodyId): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    if (body.setIndex != b2_disabledSet) {
        return;
    }

    let disabledSet = world.solverSets.get(b2_disabledSet);
    let setId = body.type == b2_staticBody ? b2_staticSet : b2_awakeSet;
    let targetSet = world.solverSets.get(setId);

    b2TransferBody(world, targetSet, disabledSet, body);

    let transform = b2GetBodyTransformQuick(world, body);

    // Add shapes to broad-phase
    let proxyType = body.type;
    let forcePairCreation = true;
    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);
        shapeId = shape.nextShapeId;

        b2CreateShapeProxy(shape, world.broadPhase, proxyType, transform, forcePairCreation);
    }

    if (setId != b2_staticSet) {
        b2CreateIslandForBody(world, setId, body);
    }

    // Transfer joints. If the other body is disabled, don't transfer.
    // If the other body is sleeping, wake it.
    let jointKey = body.headJointKey;
    while (jointKey != B2_NULL_INDEX) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;

        let joint = world.joints.get(jointId);
        assert(joint.setIndex == b2_disabledSet);
        assert(joint.islandId == B2_NULL_INDEX);

        jointKey = joint.edges[edgeIndex].nextKey;

        let bodyA = world.bodies.get(joint.edges[0].bodyId);
        let bodyB = world.bodies.get(joint.edges[1].bodyId);

        if (bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet) {
            // one body is still disabled
            continue;
        }

        // Transfer joint first
        let jointSetId: i32;
        if (bodyA.setIndex == b2_staticSet && bodyB.setIndex == b2_staticSet) {
            jointSetId = b2_staticSet;
        } else if (bodyA.setIndex == b2_staticSet) {
            jointSetId = bodyB.setIndex;
        } else {
            jointSetId = bodyA.setIndex;
        }

        let jointSet = world.solverSets.get(jointSetId);
        b2TransferJoint(world, jointSet, disabledSet, joint);

        // Now that the joint is in the correct set, I can link the joint in the island.
        if (jointSetId != b2_staticSet) {
            b2LinkJoint(world, joint);
        }
    }

    b2ValidateSolverSets(world);
}

export function b2Body_SetMotionLocks(bodyId: b2BodyId, locks: b2MotionLocks): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let newFlags: u32 = 0;
    newFlags |= locks.linearX ? b2_lockLinearX : 0;
    newFlags |= locks.linearY ? b2_lockLinearY : 0;
    newFlags |= locks.angularZ ? b2_lockAngularZ : 0;

    let body = b2GetBodyFullId(world, bodyId);
    if ((body.flags & b2_allLocks) != newFlags) {
        body.flags &= ~b2_allLocks;
        body.flags |= newFlags;

        let bodySim = b2GetBodySim(world, body);
        bodySim.flags &= ~b2_allLocks;
        bodySim.flags |= newFlags;

        let state = b2GetBodyState(world, body);

        if (state != null) {
            state.flags = bodySim.flags;

            if (locks.linearX) {
                state.linearVelocity.x = 0.0;
            }

            if (locks.linearY) {
                state.linearVelocity.y = 0.0;
            }

            if (locks.angularZ) {
                state.angularVelocity = 0.0;
            }
        }
    }
}

export function b2Body_GetMotionLocks(bodyId: b2BodyId): b2MotionLocks {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);

    let locks = new b2MotionLocks();
    locks.linearX = (body.flags & b2_lockLinearX) != 0;
    locks.linearY = (body.flags & b2_lockLinearY) != 0;
    locks.angularZ = (body.flags & b2_lockAngularZ) != 0;
    return locks;
}

export function b2Body_SetBullet(bodyId: b2BodyId, flag: bool): void {
    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return;
    }

    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);

    if (flag) {
        bodySim.flags |= b2_isBullet;
    } else {
        bodySim.flags &= ~b2_isBullet;
    }
}

export function b2Body_IsBullet(bodyId: b2BodyId): bool {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let bodySim = b2GetBodySim(world, body);
    return (bodySim.flags & b2_isBullet) != 0;
}

export function b2Body_EnableContactEvents(bodyId: b2BodyId, flag: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);
        shape.enableContactEvents = flag;
        shapeId = shape.nextShapeId;
    }
}

export function b2Body_EnableHitEvents(bodyId: b2BodyId, flag: bool): void {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let shapeId = body.headShapeId;
    while (shapeId != B2_NULL_INDEX) {
        let shape = world.shapes.get(shapeId);
        shape.enableHitEvents = flag;
        shapeId = shape.nextShapeId;
    }
}

export function b2Body_GetWorld(bodyId: b2BodyId): b2WorldId {
    let world = b2GetWorld(bodyId.world0);
    return new b2WorldId(bodyId.world0 + 1, world.generation);
}

export function b2Body_GetShapeCount(bodyId: b2BodyId): i32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.shapeCount;
}

export function b2Body_GetShapes(bodyId: b2BodyId, shapeArray: Array<b2ShapeId>, capacity: i32): i32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let shapeId = body.headShapeId;
    let shapeCount = 0;
    while (shapeId != B2_NULL_INDEX && shapeCount < capacity) {
        let shape = world.shapes.get(shapeId);
        shapeArray[shapeCount] = new b2ShapeId(shape.id + 1, bodyId.world0, shape.generation);
        shapeCount += 1;

        shapeId = shape.nextShapeId;
    }

    return shapeCount;
}

export function b2Body_GetJointCount(bodyId: b2BodyId): i32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    return body.jointCount;
}

export function b2Body_GetJoints(bodyId: b2BodyId, jointArray: Array<b2JointId>, capacity: i32): i32 {
    let world = b2GetWorld(bodyId.world0);
    let body = b2GetBodyFullId(world, bodyId);
    let jointKey = body.headJointKey;

    let jointCount = 0;
    while (jointKey != B2_NULL_INDEX && jointCount < capacity) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;

        let joint = world.joints.get(jointId);

        jointArray[jointCount] = new b2JointId(jointId + 1, bodyId.world0, joint.generation);
        jointCount += 1;

        jointKey = joint.edges[edgeIndex].nextKey;
    }

    return jointCount;
}

export function b2ShouldBodiesCollide(world: b2World, bodyA: b2Body, bodyB: b2Body): bool {
    if (bodyA.type != b2_dynamicBody && bodyB.type != b2_dynamicBody) {
        return false;
    }

    let jointKey: i32;
    let otherBodyId: i32;
    if (bodyA.jointCount < bodyB.jointCount) {
        jointKey = bodyA.headJointKey;
        otherBodyId = bodyB.id;
    } else {
        jointKey = bodyB.headJointKey;
        otherBodyId = bodyA.id;
    }

    while (jointKey != B2_NULL_INDEX) {
        let jointId = jointKey >> 1;
        let edgeIndex = jointKey & 1;
        let otherEdgeIndex = edgeIndex ^ 1;

        let joint = world.joints.get(jointId);
        if (joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId) {
            return false;
        }

        jointKey = joint.edges[edgeIndex].nextKey;
    }

    return true;
}

// b2Body_IsValid lives in physics_world.c; ported here to avoid a cycle-only import.
export function b2Body_IsValid(id: b2BodyId): bool {
    if (id.world0 >= <u16>128) {
        return false;
    }

    let world = b2GetWorldMaybe(id.world0);
    if (world == null) {
        return false;
    }

    if (id.index1 < 1 || world.bodies.count < id.index1) {
        return false;
    }

    let body = world.bodies.get(id.index1 - 1);
    if (body.setIndex == B2_NULL_INDEX) {
        // this was freed
        return false;
    }

    assert(body.localIndex != B2_NULL_INDEX);

    if (body.generation != id.generation) {
        // this id is orphaned
        return false;
    }

    return true;
}

import { b2GetWorldMaybe } from "./physics_world";
