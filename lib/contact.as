// Ported from box2d src/contact.h/.c. The C function-pointer register table
// becomes two flag tables plus a switch dispatch over the primary pairs.

import { B2_NULL_INDEX } from "./core";
import { B2_LINEAR_SLOP, B2_GRAPH_COLOR_COUNT } from "./constants";
import { b2Transform, b2Vec2, b2Sub, b2MaxFloat } from "./math_functions";
import { b2Manifold, b2SimplexCache } from "./collision";
import { b2ShapeId, b2ContactId } from "./id";
import {
    b2CollideCircles, b2CollideCapsuleAndCircle, b2CollideCapsules, b2CollidePolygonAndCircle,
    b2CollidePolygonAndCapsule, b2CollidePolygons, b2CollideSegmentAndCircle, b2CollideSegmentAndCapsule,
    b2CollideSegmentAndPolygon, b2CollideChainSegmentAndCircle, b2CollideChainSegmentAndCapsule,
    b2CollideChainSegmentAndPolygon,
} from "./manifold";
import { B2_SHAPE_PAIR_KEY, b2AddKey, b2RemoveKey } from "./table";
import { b2AllocId, b2FreeId } from "./id_pool";
import {
    b2World, b2ValidateSolverSets, b2ContactEndTouchEvent, b2ContactData,
    b2_awakeSet, b2_disabledSet, b2_staticSet,
} from "./physics_world";
import {
    b2_circleShape, b2_capsuleShape, b2_segmentShape, b2_polygonShape, b2_chainSegmentShape,
    b2_shapeTypeCount,
} from "./types";
import { b2Shape, b2GetShapeRadius } from "./shape";
import { b2WakeBody } from "./body";
import { b2UnlinkContact } from "./island";
import { b2RemoveContactFromGraph } from "./constraint_graph";

// enum b2ContactFlags
export const b2_contactTouchingFlag: u32 = 0x00000001;
export const b2_contactHitEventFlag: u32 = 0x00000002;
export const b2_contactEnableContactEvents: u32 = 0x00000004;

// enum b2ContactSimFlags (shifted to be distinct from b2ContactFlags)
export const b2_simTouchingFlag: u32 = 0x00010000;
export const b2_simDisjoint: u32 = 0x00020000;
export const b2_simStartedTouching: u32 = 0x00040000;
export const b2_simStoppedTouching: u32 = 0x00080000;
export const b2_simEnableHitEvent: u32 = 0x00100000;
export const b2_simEnablePreSolveEvents: u32 = 0x00200000;

export class b2ContactEdge {
    bodyId: i32 = 0;
    prevKey: i32 = 0;
    nextKey: i32 = 0;
}

function makeEdges(): StaticArray<b2ContactEdge> {
    let a = new StaticArray<b2ContactEdge>(2);
    a[0] = new b2ContactEdge();
    a[1] = new b2ContactEdge();
    return a;
}

// Cold contact data. Used as a persistent handle and for persistent island connectivity.
export class b2Contact {
    setIndex: i32 = 0;
    colorIndex: i32 = 0;
    localIndex: i32 = 0;
    shapeIdA: i32 = 0;
    shapeIdB: i32 = 0;
    contactId: i32 = B2_NULL_INDEX;
    edges: StaticArray<b2ContactEdge> = makeEdges();
    islandPrev: i32 = 0;
    islandNext: i32 = 0;
    islandId: i32 = 0;
    flags: u32 = 0;
    generation: u32 = 0;
}

// The manages contact between two shapes.
export class b2ContactSim {
    contactId: i32 = 0;
    bodySimIndexA: i32 = 0;
    bodySimIndexB: i32 = 0;
    shapeIdA: i32 = 0;
    shapeIdB: i32 = 0;
    invMassA: f32 = 0;
    invIA: f32 = 0;
    invMassB: f32 = 0;
    invIB: f32 = 0;
    manifold: b2Manifold = new b2Manifold();
    // Mixed friction and restitution
    friction: f32 = 0;
    restitution: f32 = 0;
    rollingResistance: f32 = 0;
    tangentSpeed: f32 = 0;
    simFlags: u32 = 0;
    cache: b2SimplexCache = new b2SimplexCache();
}

// Register tables: fcn presence and primary orientation, [typeA][typeB].
let s_registerHasFcn = new StaticArray<bool>(b2_shapeTypeCount * b2_shapeTypeCount);
let s_registerPrimary = new StaticArray<bool>(b2_shapeTypeCount * b2_shapeTypeCount);
let s_initialized = false;

function b2AddType(type1: i32, type2: i32): void {
    assert(0 <= type1 && type1 < b2_shapeTypeCount);
    assert(0 <= type2 && type2 < b2_shapeTypeCount);

    s_registerHasFcn[type1 * b2_shapeTypeCount + type2] = true;
    s_registerPrimary[type1 * b2_shapeTypeCount + type2] = true;

    if (type1 != type2) {
        s_registerHasFcn[type2 * b2_shapeTypeCount + type1] = true;
        s_registerPrimary[type2 * b2_shapeTypeCount + type1] = false;
    }
}

export function b2InitializeContactRegisters(): void {
    if (s_initialized == false) {
        b2AddType(b2_circleShape, b2_circleShape);
        b2AddType(b2_capsuleShape, b2_circleShape);
        b2AddType(b2_capsuleShape, b2_capsuleShape);
        b2AddType(b2_polygonShape, b2_circleShape);
        b2AddType(b2_polygonShape, b2_capsuleShape);
        b2AddType(b2_polygonShape, b2_polygonShape);
        b2AddType(b2_segmentShape, b2_circleShape);
        b2AddType(b2_segmentShape, b2_capsuleShape);
        b2AddType(b2_segmentShape, b2_polygonShape);
        b2AddType(b2_chainSegmentShape, b2_circleShape);
        b2AddType(b2_chainSegmentShape, b2_capsuleShape);
        b2AddType(b2_chainSegmentShape, b2_polygonShape);
        s_initialized = true;
    }
}

// Dispatch over the primary pairs; mirrors the C s_registers fcn pointers.
function b2ComputeManifold(shapeA: b2Shape, xfA: b2Transform, shapeB: b2Shape, xfB: b2Transform,
    cache: b2SimplexCache): b2Manifold {
    let typeA = shapeA.type;
    let typeB = shapeB.type;

    if (typeA == b2_circleShape && typeB == b2_circleShape) {
        return b2CollideCircles(shapeA.circle!, xfA, shapeB.circle!, xfB);
    }
    if (typeA == b2_capsuleShape && typeB == b2_circleShape) {
        return b2CollideCapsuleAndCircle(shapeA.capsule!, xfA, shapeB.circle!, xfB);
    }
    if (typeA == b2_capsuleShape && typeB == b2_capsuleShape) {
        return b2CollideCapsules(shapeA.capsule!, xfA, shapeB.capsule!, xfB);
    }
    if (typeA == b2_polygonShape && typeB == b2_circleShape) {
        return b2CollidePolygonAndCircle(shapeA.polygon!, xfA, shapeB.circle!, xfB);
    }
    if (typeA == b2_polygonShape && typeB == b2_capsuleShape) {
        return b2CollidePolygonAndCapsule(shapeA.polygon!, xfA, shapeB.capsule!, xfB);
    }
    if (typeA == b2_polygonShape && typeB == b2_polygonShape) {
        return b2CollidePolygons(shapeA.polygon!, xfA, shapeB.polygon!, xfB);
    }
    if (typeA == b2_segmentShape && typeB == b2_circleShape) {
        return b2CollideSegmentAndCircle(shapeA.segment!, xfA, shapeB.circle!, xfB);
    }
    if (typeA == b2_segmentShape && typeB == b2_capsuleShape) {
        return b2CollideSegmentAndCapsule(shapeA.segment!, xfA, shapeB.capsule!, xfB);
    }
    if (typeA == b2_segmentShape && typeB == b2_polygonShape) {
        return b2CollideSegmentAndPolygon(shapeA.segment!, xfA, shapeB.polygon!, xfB);
    }
    if (typeA == b2_chainSegmentShape && typeB == b2_circleShape) {
        return b2CollideChainSegmentAndCircle(shapeA.chainSegment!, xfA, shapeB.circle!, xfB);
    }
    if (typeA == b2_chainSegmentShape && typeB == b2_capsuleShape) {
        return b2CollideChainSegmentAndCapsule(shapeA.chainSegment!, xfA, shapeB.capsule!, xfB, cache);
    }
    if (typeA == b2_chainSegmentShape && typeB == b2_polygonShape) {
        return b2CollideChainSegmentAndPolygon(shapeA.chainSegment!, xfA, shapeB.polygon!, xfB, cache);
    }

    assert(false);
    return new b2Manifold();
}

export function b2CreateContact(world: b2World, shapeA: b2Shape, shapeB: b2Shape): void {
    let type1 = shapeA.type;
    let type2 = shapeB.type;

    assert(0 <= type1 && type1 < b2_shapeTypeCount);
    assert(0 <= type2 && type2 < b2_shapeTypeCount);

    if (s_registerHasFcn[type1 * b2_shapeTypeCount + type2] == false) {
        // For example, no segment vs segment collision
        return;
    }

    if (s_registerPrimary[type1 * b2_shapeTypeCount + type2] == false) {
        // flip order
        b2CreateContact(world, shapeB, shapeA);
        return;
    }

    let bodyA = world.bodies.get(shapeA.bodyId);
    let bodyB = world.bodies.get(shapeB.bodyId);

    assert(bodyA.setIndex != b2_disabledSet && bodyB.setIndex != b2_disabledSet);
    assert(bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet);

    let setIndex: i32;
    if (bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet) {
        setIndex = b2_awakeSet;
    } else {
        // sleeping and non-touching contacts live in the disabled set
        setIndex = b2_disabledSet;
    }

    let set = world.solverSets.get(setIndex);

    // Create contact key and contact
    let contactId = b2AllocId(world.contactIdPool);
    if (contactId == world.contacts.count) {
        world.contacts.add();
    }

    let shapeIdA = shapeA.id;
    let shapeIdB = shapeB.id;

    let contact = world.contacts.get(contactId);
    contact.contactId = contactId;
    contact.generation += 1;
    contact.setIndex = setIndex;
    contact.colorIndex = B2_NULL_INDEX;
    contact.localIndex = set.contactSims.count;
    contact.islandId = B2_NULL_INDEX;
    contact.islandPrev = B2_NULL_INDEX;
    contact.islandNext = B2_NULL_INDEX;
    contact.shapeIdA = shapeIdA;
    contact.shapeIdB = shapeIdB;
    contact.flags = 0;

    assert(shapeA.sensorIndex == B2_NULL_INDEX && shapeB.sensorIndex == B2_NULL_INDEX);

    if (shapeA.enableContactEvents || shapeB.enableContactEvents) {
        contact.flags |= b2_contactEnableContactEvents;
    }

    // Connect to body A
    {
        contact.edges[0].bodyId = shapeA.bodyId;
        contact.edges[0].prevKey = B2_NULL_INDEX;
        contact.edges[0].nextKey = bodyA.headContactKey;

        let keyA = (contactId << 1) | 0;
        let headContactKey = bodyA.headContactKey;
        if (headContactKey != B2_NULL_INDEX) {
            let headContact = world.contacts.get(headContactKey >> 1);
            headContact.edges[headContactKey & 1].prevKey = keyA;
        }
        bodyA.headContactKey = keyA;
        bodyA.contactCount += 1;
    }

    // Connect to body B
    {
        contact.edges[1].bodyId = shapeB.bodyId;
        contact.edges[1].prevKey = B2_NULL_INDEX;
        contact.edges[1].nextKey = bodyB.headContactKey;

        let keyB = (contactId << 1) | 1;
        let headContactKey = bodyB.headContactKey;
        if (bodyB.headContactKey != B2_NULL_INDEX) {
            let headContact = world.contacts.get(headContactKey >> 1);
            headContact.edges[headContactKey & 1].prevKey = keyB;
        }
        bodyB.headContactKey = keyB;
        bodyB.contactCount += 1;
    }

    // Add to pair set for fast lookup
    let pairKey = B2_SHAPE_PAIR_KEY(shapeIdA, shapeIdB);
    b2AddKey(world.broadPhase.pairSet, pairKey);

    // Contacts are created as non-touching. Later if they are found to be touching
    // they will link islands and be moved into the constraint graph.
    let contactSim = set.contactSims.add();
    contactSim.contactId = contactId;

    contactSim.bodySimIndexA = B2_NULL_INDEX;
    contactSim.bodySimIndexB = B2_NULL_INDEX;
    contactSim.invMassA = 0.0;
    contactSim.invIA = 0.0;
    contactSim.invMassB = 0.0;
    contactSim.invIB = 0.0;
    contactSim.shapeIdA = shapeIdA;
    contactSim.shapeIdB = shapeIdB;
    contactSim.cache = new b2SimplexCache();
    contactSim.manifold = new b2Manifold();

    // These also get updated in the narrow phase
    contactSim.friction = world.frictionCallback.mix(shapeA.material.friction, shapeA.material.userMaterialId,
        shapeB.material.friction, shapeB.material.userMaterialId);
    contactSim.restitution = world.restitutionCallback.mix(shapeA.material.restitution, shapeA.material.userMaterialId,
        shapeB.material.restitution, shapeB.material.userMaterialId);

    contactSim.tangentSpeed = 0.0;
    contactSim.simFlags = 0;

    if (shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents) {
        contactSim.simFlags |= b2_simEnablePreSolveEvents;
    }
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
export function b2DestroyContact(world: b2World, contact: b2Contact, wakeBodies: bool): void {
    // Remove pair from set
    let pairKey = B2_SHAPE_PAIR_KEY(contact.shapeIdA, contact.shapeIdB);
    b2RemoveKey(world.broadPhase.pairSet, pairKey);

    let edgeA = contact.edges[0];
    let edgeB = contact.edges[1];

    let bodyIdA = edgeA.bodyId;
    let bodyIdB = edgeB.bodyId;
    let bodyA = world.bodies.get(bodyIdA);
    let bodyB = world.bodies.get(bodyIdB);

    let flags = contact.flags;
    let touching = (flags & b2_contactTouchingFlag) != 0;

    // End touch event
    if (touching && (flags & b2_contactEnableContactEvents) != 0) {
        let worldId = world.worldId;
        let shapeA = world.shapes.get(contact.shapeIdA);
        let shapeB = world.shapes.get(contact.shapeIdB);

        let event = new b2ContactEndTouchEvent();
        event.shapeIdA = new b2ShapeId(shapeA.id + 1, worldId, shapeA.generation);
        event.shapeIdB = new b2ShapeId(shapeB.id + 1, worldId, shapeB.generation);
        event.contactId = new b2ContactId(contact.contactId + 1, world.worldId, contact.generation);

        world.contactEndEvents[world.endEventArrayIndex].push(event);
    }

    // Remove from body A
    if (edgeA.prevKey != B2_NULL_INDEX) {
        let prevContact = world.contacts.get(edgeA.prevKey >> 1);
        let prevEdge = prevContact.edges[edgeA.prevKey & 1];
        prevEdge.nextKey = edgeA.nextKey;
    }

    if (edgeA.nextKey != B2_NULL_INDEX) {
        let nextContact = world.contacts.get(edgeA.nextKey >> 1);
        let nextEdge = nextContact.edges[edgeA.nextKey & 1];
        nextEdge.prevKey = edgeA.prevKey;
    }

    let contactId = contact.contactId;

    let edgeKeyA = (contactId << 1) | 0;
    if (bodyA.headContactKey == edgeKeyA) {
        bodyA.headContactKey = edgeA.nextKey;
    }

    bodyA.contactCount -= 1;

    // Remove from body B
    if (edgeB.prevKey != B2_NULL_INDEX) {
        let prevContact = world.contacts.get(edgeB.prevKey >> 1);
        let prevEdge = prevContact.edges[edgeB.prevKey & 1];
        prevEdge.nextKey = edgeB.nextKey;
    }

    if (edgeB.nextKey != B2_NULL_INDEX) {
        let nextContact = world.contacts.get(edgeB.nextKey >> 1);
        let nextEdge = nextContact.edges[edgeB.nextKey & 1];
        nextEdge.prevKey = edgeB.prevKey;
    }

    let edgeKeyB = (contactId << 1) | 1;
    if (bodyB.headContactKey == edgeKeyB) {
        bodyB.headContactKey = edgeB.nextKey;
    }

    bodyB.contactCount -= 1;

    // Remove contact from the array that owns it
    if (contact.islandId != B2_NULL_INDEX) {
        b2UnlinkContact(world, contact);
    }

    if (contact.colorIndex != B2_NULL_INDEX) {
        // contact is an active constraint
        assert(contact.setIndex == b2_awakeSet);
        b2RemoveContactFromGraph(world, bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex);
    } else {
        // contact is non-touching or is sleeping
        assert(contact.setIndex != b2_awakeSet || (contact.flags & b2_contactTouchingFlag) == 0);
        let set = world.solverSets.get(contact.setIndex);

        let movedIndex = set.contactSims.removeSwap(contact.localIndex);
        if (movedIndex != B2_NULL_INDEX) {
            let movedContactSim = set.contactSims.get(contact.localIndex);
            let movedContact = world.contacts.get(movedContactSim.contactId);
            movedContact.localIndex = contact.localIndex;
        }
    }

    // Free contact and id (preserve generation)
    contact.contactId = B2_NULL_INDEX;
    contact.setIndex = B2_NULL_INDEX;
    contact.colorIndex = B2_NULL_INDEX;
    contact.localIndex = B2_NULL_INDEX;
    b2FreeId(world.contactIdPool, contactId);

    if (wakeBodies && touching) {
        b2WakeBody(world, bodyA);
        b2WakeBody(world, bodyB);
    }
}

export function b2GetContactSim(world: b2World, contact: b2Contact): b2ContactSim {
    if (contact.setIndex == b2_awakeSet && contact.colorIndex != B2_NULL_INDEX) {
        // contact lives in constraint graph
        assert(0 <= contact.colorIndex && contact.colorIndex < B2_GRAPH_COLOR_COUNT);
        let color = world.constraintGraph.colors[contact.colorIndex];
        return color.contactSims.get(contact.localIndex);
    }

    let set = world.solverSets.get(contact.setIndex);
    return set.contactSims.get(contact.localIndex);
}

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
export function b2UpdateContact(world: b2World, contactSim: b2ContactSim, shapeA: b2Shape, transformA: b2Transform,
    centerOffsetA: b2Vec2, shapeB: b2Shape, transformB: b2Transform, centerOffsetB: b2Vec2): bool {
    // Save old manifold (the new manifold below is a fresh object, so the old
    // reference keeps the previous contents like C's by-value copy)
    let oldManifold = contactSim.manifold;

    // Compute new manifold
    contactSim.manifold = b2ComputeManifold(shapeA, transformA, shapeB, transformB, contactSim.cache);

    // Keep these updated in case the values on the shapes are modified
    contactSim.friction = world.frictionCallback.mix(shapeA.material.friction, shapeA.material.userMaterialId,
        shapeB.material.friction, shapeB.material.userMaterialId);
    contactSim.restitution = world.restitutionCallback.mix(shapeA.material.restitution, shapeA.material.userMaterialId,
        shapeB.material.restitution, shapeB.material.userMaterialId);

    if (shapeA.material.rollingResistance > 0.0 || shapeB.material.rollingResistance > 0.0) {
        let radiusA = b2GetShapeRadius(shapeA);
        let radiusB = b2GetShapeRadius(shapeB);
        let maxRadius = b2MaxFloat(radiusA, radiusB);
        contactSim.rollingResistance =
            b2MaxFloat(shapeA.material.rollingResistance, shapeB.material.rollingResistance) * maxRadius;
    } else {
        contactSim.rollingResistance = 0.0;
    }

    contactSim.tangentSpeed = shapeA.material.tangentSpeed + shapeB.material.tangentSpeed;

    let pointCount = contactSim.manifold.pointCount;
    let touching = pointCount > 0;

    if (touching && world.preSolveFcn != null && (contactSim.simFlags & b2_simEnablePreSolveEvents) != 0) {
        let shapeIdA = new b2ShapeId(shapeA.id + 1, world.worldId, shapeA.generation);
        let shapeIdB = new b2ShapeId(shapeB.id + 1, world.worldId, shapeB.generation);

        let manifold = contactSim.manifold;
        let bestSeparation = manifold.points[0].separation;
        let bestPoint = manifold.points[0].point;

        // Get deepest point
        for (let i = 1; i < manifold.pointCount; ++i) {
            let separation = manifold.points[i].separation;
            if (separation < bestSeparation) {
                bestSeparation = separation;
                bestPoint = manifold.points[i].point;
            }
        }

        // this call assumes thread safety
        touching = world.preSolveFcn!.preSolve(shapeIdA, shapeIdB, bestPoint, manifold.normal);
        if (touching == false) {
            // disable contact
            pointCount = 0;
            manifold.pointCount = 0;
        }
    }

    // This flag is for testing
    if (world.enableSpeculative == false && pointCount == 2) {
        if (contactSim.manifold.points[0].separation > <f32>1.5 * B2_LINEAR_SLOP()) {
            contactSim.manifold.points[0] = contactSim.manifold.points[1];
            contactSim.manifold.pointCount = 1;
        } else if (contactSim.manifold.points[0].separation > <f32>1.5 * B2_LINEAR_SLOP()) {
            contactSim.manifold.pointCount = 1;
        }

        pointCount = contactSim.manifold.pointCount;
    }

    if (touching && (shapeA.enableHitEvents || shapeB.enableHitEvents)) {
        contactSim.simFlags |= b2_simEnableHitEvent;
    } else {
        contactSim.simFlags &= ~b2_simEnableHitEvent;
    }

    if (pointCount > 0) {
        contactSim.manifold.rollingImpulse = oldManifold.rollingImpulse;
    }

    // Match old contact ids to new contact ids and copy the
    // stored impulses to warm start the solver.
    let unmatchedCount = 0;
    for (let i = 0; i < pointCount; ++i) {
        let mp2 = contactSim.manifold.points[i];

        // shift anchors to be center of mass relative
        mp2.anchorA = b2Sub(mp2.anchorA, centerOffsetA);
        mp2.anchorB = b2Sub(mp2.anchorB, centerOffsetB);

        mp2.normalImpulse = 0.0;
        mp2.tangentImpulse = 0.0;
        mp2.totalNormalImpulse = 0.0;
        mp2.normalVelocity = 0.0;
        mp2.persisted = false;

        let id2 = mp2.id;

        for (let j = 0; j < oldManifold.pointCount; ++j) {
            let mp1 = oldManifold.points[j];

            if (mp1.id == id2) {
                mp2.normalImpulse = mp1.normalImpulse;
                mp2.tangentImpulse = mp1.tangentImpulse;
                mp2.persisted = true;

                // clear old impulse
                mp1.normalImpulse = 0.0;
                mp1.tangentImpulse = 0.0;
                break;
            }
        }

        unmatchedCount += mp2.persisted ? 0 : 1;
    }

    if (touching) {
        contactSim.simFlags |= b2_simTouchingFlag;
    } else {
        contactSim.simFlags &= ~b2_simTouchingFlag;
    }

    return touching;
}
