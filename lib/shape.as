// Ported from box2d src/shape.h/.c. The geometry union becomes nullable
// fields with exactly one set; setters clone like C's by-value copy. The
// mover collide functions are not ported (outside the subset).

import { B2_NULL_INDEX, B2_SECRET_COOKIE } from "./core";
import { B2_LINEAR_SLOP, B2_SPECULATIVE_DISTANCE, B2_AABB_MARGIN, B2_HUGE } from "./constants";
import {
    b2Vec2, b2Rot, b2Transform, b2AABB, b2Transform_identity, b2Vec2_zero, b2Lerp, b2Sub, b2Add, b2Neg,
    b2Dot, b2Cross, b2CrossSV, b2Length, b2LengthSquared, b2DistanceSquared, b2MulSV, b2MulAdd, b2MulSub,
    b2MinFloat, b2MaxFloat, b2AbsFloat, b2MinInt, b2TransformPoint, b2InvTransformPoint, b2RotateVector,
    b2InvRotateVector, b2LeftPerp, b2RightPerp, b2Normalize, b2GetLengthAndNormalize, b2IsValidFloat, B2_PI,
} from "./math_functions";
import {
    b2Circle, b2Capsule, b2Polygon, b2Segment, b2ChainSegment, b2MassData, b2ShapeProxy,
    b2RayCastInput, b2ShapeCastInput, b2CastOutput, b2DistanceInput, b2SimplexCache, makeVecArray,
} from "./collision";
import {
    b2ComputeCircleAABB, b2ComputeCapsuleAABB, b2ComputePolygonAABB, b2ComputeSegmentAABB,
    b2ComputeCircleMass, b2ComputeCapsuleMass, b2ComputePolygonMass,
    b2PointInCircle, b2PointInCapsule, b2PointInPolygon,
    b2RayCastCircle, b2RayCastCapsule, b2RayCastPolygon, b2RayCastSegment,
    b2ShapeCastCircle, b2ShapeCastCapsule, b2ShapeCastPolygon, b2ShapeCastSegment,
} from "./geometry";
import { b2MakeProxy, b2ShapeDistance } from "./distance";
import { b2ShapeId, b2ChainId, b2BodyId, b2WorldId, b2ContactId } from "./id";
import { b2Filter, b2QueryFilter, b2SurfaceMaterial, b2ShapeDef, b2ChainDef, b2DefaultShapeDef,
    b2_circleShape, b2_capsuleShape, b2_segmentShape, b2_polygonShape, b2_chainSegmentShape } from "./types";
import {
    b2World, b2GetWorld, b2GetWorldLocked, b2ValidateSolverSets, b2ContactData,
    b2_awakeSet, b2_disabledSet, b2_staticSet, b2_firstSleepingSet,
} from "./physics_world";
import { b2AllocId, b2FreeId } from "./id_pool";
import {
    b2BroadPhase, b2BroadPhase_CreateProxy, b2BroadPhase_DestroyProxy, b2BroadPhase_MoveProxy,
    b2BufferMove, B2_PROXY_TYPE, b2_staticBody, b2_dynamicBody, b2_bodyTypeCount,
} from "./broad_phase";
import { b2Body, b2GetBodyFullId, b2GetBodyTransformQuick, b2GetBodyTransform, b2MakeBodyId,
    b2UpdateBodyMassData, b2WakeBody, b2GetBodySim, b2GetBodyState, b2_dirtyMass } from "./body";
import { b2Contact, b2ContactSim, b2DestroyContact, b2GetContactSim, b2_contactTouchingFlag } from "./contact";
import { b2Sensor, b2Visitor, b2DestroySensor } from "./sensor";
import { b2_lengthUnitsPerMeter } from "./core";

export class b2Shape {
    id: i32 = B2_NULL_INDEX;
    bodyId: i32 = 0;
    prevShapeId: i32 = 0;
    nextShapeId: i32 = 0;
    sensorIndex: i32 = 0;
    type: i32 = 0;
    material: b2SurfaceMaterial = new b2SurfaceMaterial();
    density: f32 = 0;

    aabb: b2AABB = new b2AABB();
    fatAABB: b2AABB = new b2AABB();
    localCentroid: b2Vec2 = new b2Vec2();
    proxyKey: i32 = 0;

    filter: b2Filter = new b2Filter();
    userData: u64 = 0;

    // C union: exactly one is non-null
    capsule: b2Capsule | null = null;
    circle: b2Circle | null = null;
    polygon: b2Polygon | null = null;
    segment: b2Segment | null = null;
    chainSegment: b2ChainSegment | null = null;

    generation: u16 = 0;
    enableSensorEvents: bool = false;
    enableContactEvents: bool = false;
    enableCustomFiltering: bool = false;
    enableHitEvents: bool = false;
    enablePreSolveEvents: bool = false;
    enlargedAABB: bool = false;
}

export class b2ChainShape {
    id: i32 = B2_NULL_INDEX;
    bodyId: i32 = 0;
    nextChainId: i32 = 0;
    count: i32 = 0;
    materialCount: i32 = 0;
    shapeIndices: Int32Array = new Int32Array(0);
    materials: Array<b2SurfaceMaterial> = new Array<b2SurfaceMaterial>();
    generation: u16 = 0;
}

export class b2ShapeExtent {
    minExtent: f32 = 0;
    maxExtent: f32 = 0;
}

function cloneCircle(c: b2Circle): b2Circle {
    return new b2Circle(c.center.clone(), c.radius);
}

function cloneCapsule(c: b2Capsule): b2Capsule {
    return new b2Capsule(c.center1.clone(), c.center2.clone(), c.radius);
}

function clonePolygon(p: b2Polygon): b2Polygon {
    let out = new b2Polygon();
    for (let i = 0; i < 8; ++i) {
        out.vertices[i] = p.vertices[i].clone();
        out.normals[i] = p.normals[i].clone();
    }
    out.centroid = p.centroid.clone();
    out.radius = p.radius;
    out.count = p.count;
    return out;
}

function cloneSegment(s: b2Segment): b2Segment {
    return new b2Segment(s.point1.clone(), s.point2.clone());
}

function cloneChainSegment(s: b2ChainSegment): b2ChainSegment {
    let out = new b2ChainSegment();
    out.ghost1 = s.ghost1.clone();
    out.segment = cloneSegment(s.segment);
    out.ghost2 = s.ghost2.clone();
    out.chainId = s.chainId;
    return out;
}

function b2GetShape(world: b2World, shapeId: b2ShapeId): b2Shape {
    let id = shapeId.index1 - 1;
    let shape = world.shapes.get(id);
    assert(shape.id == id && shape.generation == shapeId.generation);
    return shape;
}

function b2GetChainShape(world: b2World, chainId: b2ChainId): b2ChainShape {
    let id = chainId.index1 - 1;
    let chain = world.chainShapes.get(id);
    assert(chain.id == id && chain.generation == chainId.generation);
    return chain;
}

function b2UpdateShapeAABBs(shape: b2Shape, transform: b2Transform, proxyType: i32): void {
    // Compute a bounding box with a speculative margin
    let speculativeDistance = B2_SPECULATIVE_DISTANCE();
    let aabbMargin = B2_AABB_MARGIN();

    let aabb = b2ComputeShapeAABB(shape, transform);
    aabb.lowerBound.x -= speculativeDistance;
    aabb.lowerBound.y -= speculativeDistance;
    aabb.upperBound.x += speculativeDistance;
    aabb.upperBound.y += speculativeDistance;
    shape.aabb = aabb;

    // Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
    let margin = proxyType == b2_staticBody ? speculativeDistance : aabbMargin;
    let fatAABB = new b2AABB();
    fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
    fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
    fatAABB.upperBound.x = aabb.upperBound.x + margin;
    fatAABB.upperBound.y = aabb.upperBound.y + margin;
    shape.fatAABB = fatAABB;
}

// geometry union payload for shape creation
export class b2ShapeGeometry {
    circle: b2Circle | null = null;
    capsule: b2Capsule | null = null;
    polygon: b2Polygon | null = null;
    segment: b2Segment | null = null;
    chainSegment: b2ChainSegment | null = null;
}

export function b2CreateShapeInternal(world: b2World, body: b2Body, transform: b2Transform, def: b2ShapeDef,
    geometry: b2ShapeGeometry, shapeType: i32): b2Shape {
    let shapeId = b2AllocId(world.shapeIdPool);

    if (shapeId == world.shapes.count) {
        world.shapes.add();
    } else {
        assert(world.shapes.get(shapeId).id == B2_NULL_INDEX);
    }

    let shape = world.shapes.get(shapeId);

    shape.circle = null;
    shape.capsule = null;
    shape.polygon = null;
    shape.segment = null;
    shape.chainSegment = null;

    switch (shapeType) {
        case b2_capsuleShape:
            shape.capsule = cloneCapsule(geometry.capsule!);
            break;

        case b2_circleShape:
            shape.circle = cloneCircle(geometry.circle!);
            break;

        case b2_polygonShape:
            shape.polygon = clonePolygon(geometry.polygon!);
            break;

        case b2_segmentShape:
            shape.segment = cloneSegment(geometry.segment!);
            break;

        case b2_chainSegmentShape:
            shape.chainSegment = cloneChainSegment(geometry.chainSegment!);
            break;

        default:
            assert(false);
            break;
    }

    shape.id = shapeId;
    shape.bodyId = body.id;
    shape.type = shapeType;
    shape.density = def.density;
    shape.material = def.material.clone();
    shape.filter = def.filter.clone();
    shape.userData = def.userData;
    shape.enlargedAABB = false;
    shape.enableSensorEvents = def.enableSensorEvents;
    shape.enableContactEvents = def.enableContactEvents;
    shape.enableCustomFiltering = def.enableCustomFiltering;
    shape.enableHitEvents = def.enableHitEvents;
    shape.enablePreSolveEvents = def.enablePreSolveEvents;
    shape.proxyKey = B2_NULL_INDEX;
    shape.localCentroid = b2GetShapeCentroid(shape);
    shape.aabb = new b2AABB();
    shape.fatAABB = new b2AABB();
    shape.generation += 1;

    if (body.setIndex != b2_disabledSet) {
        let proxyType = body.type;
        b2CreateShapeProxy(shape, world.broadPhase, proxyType, transform, def.invokeContactCreation || def.isSensor);
    }

    // Add to shape doubly linked list
    if (body.headShapeId != B2_NULL_INDEX) {
        let headShape = world.shapes.get(body.headShapeId);
        headShape.prevShapeId = shapeId;
    }

    shape.prevShapeId = B2_NULL_INDEX;
    shape.nextShapeId = body.headShapeId;
    body.headShapeId = shapeId;
    body.shapeCount += 1;

    if (def.isSensor) {
        shape.sensorIndex = world.sensors.count;
        let sensor = world.sensors.add();
        sensor.shapeId = shapeId;
    } else {
        shape.sensorIndex = B2_NULL_INDEX;
    }

    b2ValidateSolverSets(world);

    return shape;
}

function b2CreateShape(bodyId: b2BodyId, def: b2ShapeDef, geometry: b2ShapeGeometry, shapeType: i32): b2ShapeId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    assert(b2IsValidFloat(def.density) && def.density >= 0.0);
    assert(b2IsValidFloat(def.material.friction) && def.material.friction >= 0.0);
    assert(b2IsValidFloat(def.material.restitution) && def.material.restitution >= 0.0);
    assert(b2IsValidFloat(def.material.rollingResistance) && def.material.rollingResistance >= 0.0);
    assert(b2IsValidFloat(def.material.tangentSpeed));

    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return new b2ShapeId(0, 0, 0);
    }

    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);

    let shape = b2CreateShapeInternal(world, body, transform, def, geometry, shapeType);

    if (def.updateBodyMass == true) {
        b2UpdateBodyMassData(world, body);
    } else {
        body.flags |= b2_dirtyMass;
    }

    b2ValidateSolverSets(world);

    return new b2ShapeId(shape.id + 1, bodyId.world0, shape.generation);
}

export function b2CreateCircleShape(bodyId: b2BodyId, def: b2ShapeDef, circle: b2Circle): b2ShapeId {
    let geometry = new b2ShapeGeometry();
    geometry.circle = circle;
    return b2CreateShape(bodyId, def, geometry, b2_circleShape);
}

export function b2CreateCapsuleShape(bodyId: b2BodyId, def: b2ShapeDef, capsule: b2Capsule): b2ShapeId {
    let lengthSqr = b2DistanceSquared(capsule.center1, capsule.center2);
    if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
        return new b2ShapeId(0, 0, 0);
    }

    let geometry = new b2ShapeGeometry();
    geometry.capsule = capsule;
    return b2CreateShape(bodyId, def, geometry, b2_capsuleShape);
}

export function b2CreatePolygonShape(bodyId: b2BodyId, def: b2ShapeDef, polygon: b2Polygon): b2ShapeId {
    assert(b2IsValidFloat(polygon.radius) && polygon.radius >= 0.0);
    let geometry = new b2ShapeGeometry();
    geometry.polygon = polygon;
    return b2CreateShape(bodyId, def, geometry, b2_polygonShape);
}

export function b2CreateSegmentShape(bodyId: b2BodyId, def: b2ShapeDef, segment: b2Segment): b2ShapeId {
    let lengthSqr = b2DistanceSquared(segment.point1, segment.point2);
    if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
        assert(false);
        return new b2ShapeId(0, 0, 0);
    }

    let geometry = new b2ShapeGeometry();
    geometry.segment = segment;
    return b2CreateShape(bodyId, def, geometry, b2_segmentShape);
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
function b2DestroyShapeInternal(world: b2World, shape: b2Shape, body: b2Body, wakeBodies: bool): void {
    let shapeId = shape.id;

    // Remove the shape from the body's doubly linked list.
    if (shape.prevShapeId != B2_NULL_INDEX) {
        let prevShape = world.shapes.get(shape.prevShapeId);
        prevShape.nextShapeId = shape.nextShapeId;
    }

    if (shape.nextShapeId != B2_NULL_INDEX) {
        let nextShape = world.shapes.get(shape.nextShapeId);
        nextShape.prevShapeId = shape.prevShapeId;
    }

    if (shapeId == body.headShapeId) {
        body.headShapeId = shape.nextShapeId;
    }

    body.shapeCount -= 1;

    // Remove from broad-phase.
    b2DestroyShapeProxy(shape, world.broadPhase);

    // Destroy any contacts associated with the shape.
    let contactKey = body.headContactKey;
    while (contactKey != B2_NULL_INDEX) {
        let contactId = contactKey >> 1;
        let edgeIndex = contactKey & 1;

        let contact = world.contacts.get(contactId);
        contactKey = contact.edges[edgeIndex].nextKey;

        if (contact.shapeIdA == shapeId || contact.shapeIdB == shapeId) {
            b2DestroyContact(world, contact, wakeBodies);
        }
    }

    if (shape.sensorIndex != B2_NULL_INDEX) {
        b2DestroySensor(world, shape);
    }

    // Return shape to free list.
    b2FreeId(world.shapeIdPool, shapeId);
    shape.id = B2_NULL_INDEX;

    b2ValidateSolverSets(world);
}

export function b2DestroyShape(shapeId: b2ShapeId, updateBodyMass: bool): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);

    // need to wake bodies because this might be a static body
    let wakeBodies = true;
    let body = world.bodies.get(shape.bodyId);
    b2DestroyShapeInternal(world, shape, body, wakeBodies);

    if (updateBodyMass == true) {
        b2UpdateBodyMassData(world, body);
    }
}

export function b2CreateChain(bodyId: b2BodyId, def: b2ChainDef): b2ChainId {
    assert(def.internalValue == B2_SECRET_COOKIE);
    assert(def.count >= 4);
    assert(def.materialCount == 1 || def.materialCount == def.count);

    let world = b2GetWorldLocked(bodyId.world0);
    if (world == null) {
        return new b2ChainId(0, 0, 0);
    }

    let body = b2GetBodyFullId(world, bodyId);
    let transform = b2GetBodyTransformQuick(world, body);

    let chainId = b2AllocId(world.chainIdPool);

    if (chainId == world.chainShapes.count) {
        world.chainShapes.add();
    } else {
        assert(world.chainShapes.get(chainId).id == B2_NULL_INDEX);
    }

    let chainShape = world.chainShapes.get(chainId);

    chainShape.id = chainId;
    chainShape.bodyId = body.id;
    chainShape.nextChainId = body.headChainId;
    chainShape.generation += 1;

    let materialCount = def.materialCount;
    chainShape.materialCount = materialCount;
    chainShape.materials = new Array<b2SurfaceMaterial>(materialCount);

    for (let i = 0; i < materialCount; ++i) {
        let material = def.materials[i];
        assert(b2IsValidFloat(material.friction) && material.friction >= 0.0);
        assert(b2IsValidFloat(material.restitution) && material.restitution >= 0.0);
        assert(b2IsValidFloat(material.rollingResistance) && material.rollingResistance >= 0.0);
        assert(b2IsValidFloat(material.tangentSpeed));

        chainShape.materials[i] = material.clone();
    }

    body.headChainId = chainId;

    let shapeDef = b2DefaultShapeDef();
    shapeDef.userData = def.userData;
    shapeDef.filter = def.filter.clone();
    shapeDef.enableSensorEvents = def.enableSensorEvents;
    shapeDef.enableContactEvents = false;
    shapeDef.enableHitEvents = false;

    let points = def.points;
    let n = def.count;

    let geometry = new b2ShapeGeometry();
    let chainSegment = new b2ChainSegment();
    geometry.chainSegment = chainSegment;

    if (def.isLoop) {
        chainShape.count = n;
        chainShape.shapeIndices = new Int32Array(chainShape.count);

        let prevIndex = n - 1;
        for (let i = 0; i < n - 2; ++i) {
            chainSegment.ghost1 = points[prevIndex].clone();
            chainSegment.segment.point1 = points[i].clone();
            chainSegment.segment.point2 = points[i + 1].clone();
            chainSegment.ghost2 = points[i + 2].clone();
            chainSegment.chainId = chainId;
            prevIndex = i;

            let materialIndex = materialCount == 1 ? 0 : i;
            shapeDef.material = def.materials[materialIndex].clone();

            let shape = b2CreateShapeInternal(world, body, transform, shapeDef, geometry, b2_chainSegmentShape);
            chainShape.shapeIndices[i] = shape.id;
        }

        {
            chainSegment.ghost1 = points[n - 3].clone();
            chainSegment.segment.point1 = points[n - 2].clone();
            chainSegment.segment.point2 = points[n - 1].clone();
            chainSegment.ghost2 = points[0].clone();
            chainSegment.chainId = chainId;

            let materialIndex = materialCount == 1 ? 0 : n - 2;
            shapeDef.material = def.materials[materialIndex].clone();

            let shape = b2CreateShapeInternal(world, body, transform, shapeDef, geometry, b2_chainSegmentShape);
            chainShape.shapeIndices[n - 2] = shape.id;
        }

        {
            chainSegment.ghost1 = points[n - 2].clone();
            chainSegment.segment.point1 = points[n - 1].clone();
            chainSegment.segment.point2 = points[0].clone();
            chainSegment.ghost2 = points[1].clone();
            chainSegment.chainId = chainId;

            let materialIndex = materialCount == 1 ? 0 : n - 1;
            shapeDef.material = def.materials[materialIndex].clone();

            let shape = b2CreateShapeInternal(world, body, transform, shapeDef, geometry, b2_chainSegmentShape);
            chainShape.shapeIndices[n - 1] = shape.id;
        }
    } else {
        chainShape.count = n - 3;
        chainShape.shapeIndices = new Int32Array(chainShape.count);

        for (let i = 0; i < n - 3; ++i) {
            chainSegment.ghost1 = points[i].clone();
            chainSegment.segment.point1 = points[i + 1].clone();
            chainSegment.segment.point2 = points[i + 2].clone();
            chainSegment.ghost2 = points[i + 3].clone();
            chainSegment.chainId = chainId;

            // Material is associated with leading point of solid segment
            let materialIndex = materialCount == 1 ? 0 : i + 1;
            shapeDef.material = def.materials[materialIndex].clone();

            let shape = b2CreateShapeInternal(world, body, transform, shapeDef, geometry, b2_chainSegmentShape);
            chainShape.shapeIndices[i] = shape.id;
        }
    }

    return new b2ChainId(chainId + 1, world.worldId, chainShape.generation);
}

export function b2FreeChainData(chain: b2ChainShape): void {
    chain.shapeIndices = new Int32Array(0);
    chain.materials = new Array<b2SurfaceMaterial>();
}

export function b2DestroyChain(chainId: b2ChainId): void {
    let world = b2GetWorldLocked(chainId.world0);
    if (world == null) {
        return;
    }

    let chain = b2GetChainShape(world, chainId);

    let body = world.bodies.get(chain.bodyId);

    // Remove the chain from the body's singly linked list.
    let found = false;
    if (body.headChainId == chain.id) {
        body.headChainId = chain.nextChainId;
        found = true;
    } else {
        let chainIdIter = body.headChainId;
        while (chainIdIter != B2_NULL_INDEX) {
            let prevChain = world.chainShapes.get(chainIdIter);
            if (prevChain.nextChainId == chain.id) {
                prevChain.nextChainId = chain.nextChainId;
                found = true;
                break;
            }
            chainIdIter = prevChain.nextChainId;
        }
    }

    assert(found == true);
    if (found == false) {
        return;
    }

    let count = chain.count;
    for (let i = 0; i < count; ++i) {
        let shapeId = chain.shapeIndices[i];
        let shape = world.shapes.get(shapeId);
        let wakeBodies = true;
        b2DestroyShapeInternal(world, shape, body, wakeBodies);
    }

    b2FreeChainData(chain);

    // Return chain to free list.
    b2FreeId(world.chainIdPool, chain.id);
    chain.id = B2_NULL_INDEX;

    b2ValidateSolverSets(world);
}

export function b2Chain_GetWorld(chainId: b2ChainId): b2WorldId {
    let world = b2GetWorld(chainId.world0);
    return new b2WorldId(chainId.world0 + 1, world.generation);
}

export function b2Chain_GetSegmentCount(chainId: b2ChainId): i32 {
    let world = b2GetWorldLocked(chainId.world0);
    if (world == null) {
        return 0;
    }

    let chain = b2GetChainShape(world, chainId);
    return chain.count;
}

export function b2Chain_GetSegments(chainId: b2ChainId, segmentArray: Array<b2ShapeId>, capacity: i32): i32 {
    let world = b2GetWorldLocked(chainId.world0);
    if (world == null) {
        return 0;
    }

    let chain = b2GetChainShape(world, chainId);

    let count = b2MinInt(chain.count, capacity);
    for (let i = 0; i < count; ++i) {
        let shapeId = chain.shapeIndices[i];
        let shape = world.shapes.get(shapeId);
        segmentArray[i] = new b2ShapeId(shapeId + 1, chainId.world0, shape.generation);
    }

    return count;
}

export function b2ComputeShapeAABB(shape: b2Shape, xf: b2Transform): b2AABB {
    switch (shape.type) {
        case b2_capsuleShape:
            return b2ComputeCapsuleAABB(shape.capsule!, xf);
        case b2_circleShape:
            return b2ComputeCircleAABB(shape.circle!, xf);
        case b2_polygonShape:
            return b2ComputePolygonAABB(shape.polygon!, xf);
        case b2_segmentShape:
            return b2ComputeSegmentAABB(shape.segment!, xf);
        case b2_chainSegmentShape:
            return b2ComputeSegmentAABB(shape.chainSegment!.segment, xf);
        default: {
            assert(false);
            return new b2AABB(xf.p.clone(), xf.p.clone());
        }
    }
}

export function b2GetShapeCentroid(shape: b2Shape): b2Vec2 {
    switch (shape.type) {
        case b2_capsuleShape:
            return b2Lerp(shape.capsule!.center1, shape.capsule!.center2, 0.5);
        case b2_circleShape:
            return shape.circle!.center.clone();
        case b2_polygonShape:
            return shape.polygon!.centroid.clone();
        case b2_segmentShape:
            return b2Lerp(shape.segment!.point1, shape.segment!.point2, 0.5);
        case b2_chainSegmentShape:
            return b2Lerp(shape.chainSegment!.segment.point1, shape.chainSegment!.segment.point2, 0.5);
        default:
            return b2Vec2_zero();
    }
}

export function b2GetShapePerimeter(shape: b2Shape): f32 {
    switch (shape.type) {
        case b2_capsuleShape:
            return <f32>2.0 * b2Length(b2Sub(shape.capsule!.center1, shape.capsule!.center2)) +
                <f32>2.0 * B2_PI * shape.capsule!.radius;
        case b2_circleShape:
            return <f32>2.0 * B2_PI * shape.circle!.radius;
        case b2_polygonShape: {
            let points = shape.polygon!.vertices;
            let count = shape.polygon!.count;
            let perimeter = <f32>2.0 * B2_PI * shape.polygon!.radius;
            assert(count > 0);
            let prev = points[count - 1];
            for (let i = 0; i < count; ++i) {
                let next = points[i];
                perimeter += b2Length(b2Sub(next, prev));
                prev = next;
            }

            return perimeter;
        }
        case b2_segmentShape:
            return <f32>2.0 * b2Length(b2Sub(shape.segment!.point1, shape.segment!.point2));
        case b2_chainSegmentShape:
            return <f32>2.0 * b2Length(b2Sub(shape.chainSegment!.segment.point1, shape.chainSegment!.segment.point2));
        default:
            return 0.0;
    }
}

// This projects the shape perimeter onto an infinite line
export function b2GetShapeProjectedPerimeter(shape: b2Shape, line: b2Vec2): f32 {
    switch (shape.type) {
        case b2_capsuleShape: {
            let axis = b2Sub(shape.capsule!.center2, shape.capsule!.center1);
            let projectedLength = b2AbsFloat(b2Dot(axis, line));
            return projectedLength + <f32>2.0 * shape.capsule!.radius;
        }

        case b2_circleShape:
            return <f32>2.0 * shape.circle!.radius;

        case b2_polygonShape: {
            let points = shape.polygon!.vertices;
            let count = shape.polygon!.count;
            assert(count > 0);
            let value = b2Dot(points[0], line);
            let lower = value;
            let upper = value;
            for (let i = 1; i < count; ++i) {
                value = b2Dot(points[i], line);
                lower = b2MinFloat(lower, value);
                upper = b2MaxFloat(upper, value);
            }

            return (upper - lower) + <f32>2.0 * shape.polygon!.radius;
        }

        case b2_segmentShape: {
            let value1 = b2Dot(shape.segment!.point1, line);
            let value2 = b2Dot(shape.segment!.point2, line);
            return b2AbsFloat(value2 - value1);
        }

        case b2_chainSegmentShape: {
            let value1 = b2Dot(shape.chainSegment!.segment.point1, line);
            let value2 = b2Dot(shape.chainSegment!.segment.point2, line);
            return b2AbsFloat(value2 - value1);
        }

        default:
            return 0.0;
    }
}

export function b2ComputeShapeMass(shape: b2Shape): b2MassData {
    switch (shape.type) {
        case b2_capsuleShape:
            return b2ComputeCapsuleMass(shape.capsule!, shape.density);
        case b2_circleShape:
            return b2ComputeCircleMass(shape.circle!, shape.density);
        case b2_polygonShape:
            return b2ComputePolygonMass(shape.polygon!, shape.density);
        default:
            return new b2MassData();
    }
}

export function b2ComputeShapeExtent(shape: b2Shape, localCenter: b2Vec2): b2ShapeExtent {
    let extent = new b2ShapeExtent();

    switch (shape.type) {
        case b2_capsuleShape: {
            let radius = shape.capsule!.radius;
            extent.minExtent = radius;
            let c1 = b2Sub(shape.capsule!.center1, localCenter);
            let c2 = b2Sub(shape.capsule!.center2, localCenter);
            extent.maxExtent = sqrt<f32>(b2MaxFloat(b2LengthSquared(c1), b2LengthSquared(c2))) + radius;
            break;
        }

        case b2_circleShape: {
            let radius = shape.circle!.radius;
            extent.minExtent = radius;
            extent.maxExtent = b2Length(b2Sub(shape.circle!.center, localCenter)) + radius;
            break;
        }

        case b2_polygonShape: {
            let poly = shape.polygon!;
            let minExtent = B2_HUGE();
            let maxExtentSqr: f32 = 0.0;
            let count = poly.count;
            for (let i = 0; i < count; ++i) {
                let v = poly.vertices[i];
                let planeOffset = b2Dot(poly.normals[i], b2Sub(v, poly.centroid));
                minExtent = b2MinFloat(minExtent, planeOffset);

                let distanceSqr = b2LengthSquared(b2Sub(v, localCenter));
                maxExtentSqr = b2MaxFloat(maxExtentSqr, distanceSqr);
            }

            extent.minExtent = minExtent + poly.radius;
            extent.maxExtent = sqrt<f32>(maxExtentSqr) + poly.radius;
            break;
        }

        case b2_segmentShape: {
            extent.minExtent = 0.0;
            let c1 = b2Sub(shape.segment!.point1, localCenter);
            let c2 = b2Sub(shape.segment!.point2, localCenter);
            extent.maxExtent = sqrt<f32>(b2MaxFloat(b2LengthSquared(c1), b2LengthSquared(c2)));
            break;
        }

        case b2_chainSegmentShape: {
            extent.minExtent = 0.0;
            let c1 = b2Sub(shape.chainSegment!.segment.point1, localCenter);
            let c2 = b2Sub(shape.chainSegment!.segment.point2, localCenter);
            extent.maxExtent = sqrt<f32>(b2MaxFloat(b2LengthSquared(c1), b2LengthSquared(c2)));
            break;
        }

        default:
            break;
    }

    return extent;
}

export function b2RayCastShape(input: b2RayCastInput, shape: b2Shape, transform: b2Transform): b2CastOutput {
    let localInput = new b2RayCastInput();
    localInput.origin = b2InvTransformPoint(transform, input.origin);
    localInput.translation = b2InvRotateVector(transform.q, input.translation);
    localInput.maxFraction = input.maxFraction;

    let output = new b2CastOutput();
    switch (shape.type) {
        case b2_capsuleShape:
            output = b2RayCastCapsule(shape.capsule!, localInput);
            break;
        case b2_circleShape:
            output = b2RayCastCircle(shape.circle!, localInput);
            break;
        case b2_polygonShape:
            output = b2RayCastPolygon(shape.polygon!, localInput);
            break;
        case b2_segmentShape:
            output = b2RayCastSegment(shape.segment!, localInput, false);
            break;
        case b2_chainSegmentShape:
            output = b2RayCastSegment(shape.chainSegment!.segment, localInput, true);
            break;
        default:
            return output;
    }

    output.point = b2TransformPoint(transform, output.point);
    output.normal = b2RotateVector(transform.q, output.normal);
    return output;
}

export function b2ShapeCastShape(input: b2ShapeCastInput, shape: b2Shape, transform: b2Transform): b2CastOutput {
    let output = new b2CastOutput();

    if (input.proxy.count == 0) {
        return output;
    }

    let localInput = new b2ShapeCastInput();
    localInput.proxy.count = input.proxy.count;
    localInput.proxy.radius = input.proxy.radius;
    localInput.maxFraction = input.maxFraction;
    localInput.canEncroach = input.canEncroach;

    for (let i = 0; i < localInput.proxy.count; ++i) {
        localInput.proxy.points[i] = b2InvTransformPoint(transform, input.proxy.points[i]);
    }

    localInput.translation = b2InvRotateVector(transform.q, input.translation);

    switch (shape.type) {
        case b2_capsuleShape:
            output = b2ShapeCastCapsule(shape.capsule!, localInput);
            break;
        case b2_circleShape:
            output = b2ShapeCastCircle(shape.circle!, localInput);
            break;
        case b2_polygonShape:
            output = b2ShapeCastPolygon(shape.polygon!, localInput);
            break;
        case b2_segmentShape:
            output = b2ShapeCastSegment(shape.segment!, localInput);
            break;
        case b2_chainSegmentShape: {
            // Check for back side collision
            let approximateCentroid = localInput.proxy.points[0].clone();
            for (let i = 1; i < localInput.proxy.count; ++i) {
                approximateCentroid = b2Add(approximateCentroid, localInput.proxy.points[i]);
            }

            approximateCentroid = b2MulSV(<f32>1.0 / <f32>localInput.proxy.count, approximateCentroid);

            let edge = b2Sub(shape.chainSegment!.segment.point2, shape.chainSegment!.segment.point1);
            let r = b2Sub(approximateCentroid, shape.chainSegment!.segment.point1);

            if (b2Cross(r, edge) < 0.0) {
                // Shape cast starts behind
                return output;
            }

            output = b2ShapeCastSegment(shape.chainSegment!.segment, localInput);
            break;
        }
        default:
            return output;
    }

    output.point = b2TransformPoint(transform, output.point);
    output.normal = b2RotateVector(transform.q, output.normal);
    return output;
}

export function b2CreateShapeProxy(shape: b2Shape, bp: b2BroadPhase, type: i32, transform: b2Transform,
    forcePairCreation: bool): void {
    assert(shape.proxyKey == B2_NULL_INDEX);

    b2UpdateShapeAABBs(shape, transform, type);

    // Create proxies in the broad-phase.
    shape.proxyKey =
        b2BroadPhase_CreateProxy(bp, type, shape.fatAABB, shape.filter.categoryBits, shape.id, forcePairCreation);
    assert(B2_PROXY_TYPE(shape.proxyKey) < b2_bodyTypeCount);
}

export function b2DestroyShapeProxy(shape: b2Shape, bp: b2BroadPhase): void {
    if (shape.proxyKey != B2_NULL_INDEX) {
        b2BroadPhase_DestroyProxy(bp, shape.proxyKey);
        shape.proxyKey = B2_NULL_INDEX;
    }
}

export function b2MakeShapeDistanceProxy(shape: b2Shape): b2ShapeProxy {
    switch (shape.type) {
        case b2_capsuleShape: {
            let pts = makeVecArray(2);
            pts[0] = shape.capsule!.center1.clone();
            pts[1] = shape.capsule!.center2.clone();
            return b2MakeProxy(pts, 2, shape.capsule!.radius);
        }
        case b2_circleShape: {
            let pts = makeVecArray(1);
            pts[0] = shape.circle!.center.clone();
            return b2MakeProxy(pts, 1, shape.circle!.radius);
        }
        case b2_polygonShape:
            return b2MakeProxy(shape.polygon!.vertices, shape.polygon!.count, shape.polygon!.radius);
        case b2_segmentShape: {
            let pts = makeVecArray(2);
            pts[0] = shape.segment!.point1.clone();
            pts[1] = shape.segment!.point2.clone();
            return b2MakeProxy(pts, 2, 0.0);
        }
        case b2_chainSegmentShape: {
            let pts = makeVecArray(2);
            pts[0] = shape.chainSegment!.segment.point1.clone();
            pts[1] = shape.chainSegment!.segment.point2.clone();
            return b2MakeProxy(pts, 2, 0.0);
        }
        default: {
            assert(false);
            return new b2ShapeProxy();
        }
    }
}

export function b2GetShapeRadius(shape: b2Shape): f32 {
    switch (shape.type) {
        case b2_capsuleShape:
            return shape.capsule!.radius;
        case b2_circleShape:
            return shape.circle!.radius;
        case b2_polygonShape:
            return shape.polygon!.radius;
        default:
            return 0.0;
    }
}

export function b2ShouldShapesCollide(filterA: b2Filter, filterB: b2Filter): bool {
    if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
        return filterA.groupIndex > 0;
    }

    return (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
}

export function b2ShouldQueryCollide(shapeFilter: b2Filter, queryFilter: b2QueryFilter): bool {
    return (shapeFilter.categoryBits & queryFilter.maskBits) != 0 && (shapeFilter.maskBits & queryFilter.categoryBits) != 0;
}

export function b2Shape_GetBody(shapeId: b2ShapeId): b2BodyId {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return b2MakeBodyId(world, shape.bodyId);
}

export function b2Shape_GetWorld(shapeId: b2ShapeId): b2WorldId {
    let world = b2GetWorld(shapeId.world0);
    return new b2WorldId(shapeId.world0 + 1, world.generation);
}

export function b2Shape_SetUserData(shapeId: b2ShapeId, userData: u64): void {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    shape.userData = userData;
}

export function b2Shape_GetUserData(shapeId: b2ShapeId): u64 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.userData;
}

export function b2Shape_IsSensor(shapeId: b2ShapeId): bool {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.sensorIndex != B2_NULL_INDEX;
}

export function b2Shape_TestPoint(shapeId: b2ShapeId, point: b2Vec2): bool {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);

    let transform = b2GetBodyTransform(world, shape.bodyId);
    let localPoint = b2InvTransformPoint(transform, point);

    switch (shape.type) {
        case b2_capsuleShape:
            return b2PointInCapsule(shape.capsule!, localPoint);

        case b2_circleShape:
            return b2PointInCircle(shape.circle!, localPoint);

        case b2_polygonShape:
            return b2PointInPolygon(shape.polygon!, localPoint);

        default:
            return false;
    }
}

export function b2Shape_RayCast(shapeId: b2ShapeId, input: b2RayCastInput): b2CastOutput {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);

    let transform = b2GetBodyTransform(world, shape.bodyId);

    // input in local coordinates
    let localInput = new b2RayCastInput();
    localInput.origin = b2InvTransformPoint(transform, input.origin);
    localInput.translation = b2InvRotateVector(transform.q, input.translation);
    localInput.maxFraction = input.maxFraction;

    let output = new b2CastOutput();
    switch (shape.type) {
        case b2_capsuleShape:
            output = b2RayCastCapsule(shape.capsule!, localInput);
            break;

        case b2_circleShape:
            output = b2RayCastCircle(shape.circle!, localInput);
            break;

        case b2_segmentShape:
            output = b2RayCastSegment(shape.segment!, localInput, false);
            break;

        case b2_polygonShape:
            output = b2RayCastPolygon(shape.polygon!, localInput);
            break;

        case b2_chainSegmentShape:
            output = b2RayCastSegment(shape.chainSegment!.segment, localInput, true);
            break;

        default:
            assert(false);
            return output;
    }

    if (output.hit) {
        // convert to world coordinates
        output.normal = b2RotateVector(transform.q, output.normal);
        output.point = b2TransformPoint(transform, output.point);
    }

    return output;
}

export function b2Shape_SetDensity(shapeId: b2ShapeId, density: f32, updateBodyMass: bool): void {
    assert(b2IsValidFloat(density) && density >= 0.0);

    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    if (density == shape.density) {
        // early return to avoid expensive function
        return;
    }

    shape.density = density;

    if (updateBodyMass == true) {
        let body = world.bodies.get(shape.bodyId);
        b2UpdateBodyMassData(world, body);
    }
}

export function b2Shape_GetDensity(shapeId: b2ShapeId): f32 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.density;
}

export function b2Shape_SetFriction(shapeId: b2ShapeId, friction: f32): void {
    assert(b2IsValidFloat(friction) && friction >= 0.0);

    let world = b2GetWorld(shapeId.world0);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.material.friction = friction;
}

export function b2Shape_GetFriction(shapeId: b2ShapeId): f32 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.material.friction;
}

export function b2Shape_SetRestitution(shapeId: b2ShapeId, restitution: f32): void {
    assert(b2IsValidFloat(restitution) && restitution >= 0.0);

    let world = b2GetWorld(shapeId.world0);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.material.restitution = restitution;
}

export function b2Shape_GetRestitution(shapeId: b2ShapeId): f32 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.material.restitution;
}

export function b2Shape_SetUserMaterial(shapeId: b2ShapeId, material: u64): void {
    let world = b2GetWorld(shapeId.world0);
    assert(world.locked == false);
    if (world.locked) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.material.userMaterialId = material;
}

export function b2Shape_GetUserMaterial(shapeId: b2ShapeId): u64 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.material.userMaterialId;
}

export function b2Shape_GetSurfaceMaterial(shapeId: b2ShapeId): b2SurfaceMaterial {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.material.clone();
}

export function b2Shape_SetSurfaceMaterial(shapeId: b2ShapeId, surfaceMaterial: b2SurfaceMaterial): void {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    shape.material = surfaceMaterial.clone();
}

export function b2Shape_GetFilter(shapeId: b2ShapeId): b2Filter {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.filter.clone();
}

function b2ResetProxy(world: b2World, shape: b2Shape, wakeBodies: bool, destroyProxy: bool): void {
    let body = world.bodies.get(shape.bodyId);

    let shapeId = shape.id;

    // destroy all contacts associated with this shape
    let contactKey = body.headContactKey;
    while (contactKey != B2_NULL_INDEX) {
        let contactId = contactKey >> 1;
        let edgeIndex = contactKey & 1;

        let contact = world.contacts.get(contactId);
        contactKey = contact.edges[edgeIndex].nextKey;

        if (contact.shapeIdA == shapeId || contact.shapeIdB == shapeId) {
            b2DestroyContact(world, contact, wakeBodies);
        }
    }

    let transform = b2GetBodyTransformQuick(world, body);
    if (shape.proxyKey != B2_NULL_INDEX) {
        let proxyType = B2_PROXY_TYPE(shape.proxyKey);
        b2UpdateShapeAABBs(shape, transform, proxyType);

        if (destroyProxy) {
            b2BroadPhase_DestroyProxy(world.broadPhase, shape.proxyKey);

            let forcePairCreation = true;
            shape.proxyKey = b2BroadPhase_CreateProxy(world.broadPhase, proxyType, shape.fatAABB,
                shape.filter.categoryBits, shapeId, forcePairCreation);
        } else {
            b2BroadPhase_MoveProxy(world.broadPhase, shape.proxyKey, shape.fatAABB);
        }
    } else {
        let proxyType = body.type;
        b2UpdateShapeAABBs(shape, transform, proxyType);
    }

    b2ValidateSolverSets(world);
}

export function b2Shape_SetFilter(shapeId: b2ShapeId, filter: b2Filter): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    if (filter.maskBits == shape.filter.maskBits && filter.categoryBits == shape.filter.categoryBits &&
        filter.groupIndex == shape.filter.groupIndex) {
        return;
    }

    // If the category bits change, I need to destroy the proxy because it affects the tree sorting.
    let destroyProxy = filter.categoryBits != shape.filter.categoryBits;

    shape.filter = filter.clone();

    // need to wake bodies because a filter change may destroy contacts
    let wakeBodies = true;
    b2ResetProxy(world, shape, wakeBodies, destroyProxy);
}

export function b2Shape_EnableSensorEvents(shapeId: b2ShapeId, flag: bool): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.enableSensorEvents = flag;
}

export function b2Shape_AreSensorEventsEnabled(shapeId: b2ShapeId): bool {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.enableSensorEvents;
}

export function b2Shape_EnableContactEvents(shapeId: b2ShapeId, flag: bool): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.enableContactEvents = flag;
}

export function b2Shape_AreContactEventsEnabled(shapeId: b2ShapeId): bool {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.enableContactEvents;
}

export function b2Shape_EnablePreSolveEvents(shapeId: b2ShapeId, flag: bool): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.enablePreSolveEvents = flag;
}

export function b2Shape_ArePreSolveEventsEnabled(shapeId: b2ShapeId): bool {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.enablePreSolveEvents;
}

export function b2Shape_EnableHitEvents(shapeId: b2ShapeId, flag: bool): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.enableHitEvents = flag;
}

export function b2Shape_AreHitEventsEnabled(shapeId: b2ShapeId): bool {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.enableHitEvents;
}

export function b2Shape_GetType(shapeId: b2ShapeId): i32 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.type;
}

export function b2Shape_GetCircle(shapeId: b2ShapeId): b2Circle {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    assert(shape.type == b2_circleShape);
    return cloneCircle(shape.circle!);
}

export function b2Shape_GetSegment(shapeId: b2ShapeId): b2Segment {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    assert(shape.type == b2_segmentShape);
    return cloneSegment(shape.segment!);
}

export function b2Shape_GetChainSegment(shapeId: b2ShapeId): b2ChainSegment {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    assert(shape.type == b2_chainSegmentShape);
    return cloneChainSegment(shape.chainSegment!);
}

export function b2Shape_GetCapsule(shapeId: b2ShapeId): b2Capsule {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    assert(shape.type == b2_capsuleShape);
    return cloneCapsule(shape.capsule!);
}

export function b2Shape_GetPolygon(shapeId: b2ShapeId): b2Polygon {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    assert(shape.type == b2_polygonShape);
    return clonePolygon(shape.polygon!);
}

export function b2Shape_SetCircle(shapeId: b2ShapeId, circle: b2Circle): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.circle = cloneCircle(circle);
    shape.capsule = null;
    shape.polygon = null;
    shape.segment = null;
    shape.chainSegment = null;
    shape.type = b2_circleShape;

    // need to wake bodies so they can react to the shape change
    let wakeBodies = true;
    let destroyProxy = true;
    b2ResetProxy(world, shape, wakeBodies, destroyProxy);
}

export function b2Shape_SetCapsule(shapeId: b2ShapeId, capsule: b2Capsule): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let lengthSqr = b2DistanceSquared(capsule.center1, capsule.center2);
    if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.capsule = cloneCapsule(capsule);
    shape.circle = null;
    shape.polygon = null;
    shape.segment = null;
    shape.chainSegment = null;
    shape.type = b2_capsuleShape;

    let wakeBodies = true;
    let destroyProxy = true;
    b2ResetProxy(world, shape, wakeBodies, destroyProxy);
}

export function b2Shape_SetSegment(shapeId: b2ShapeId, segment: b2Segment): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.segment = cloneSegment(segment);
    shape.circle = null;
    shape.capsule = null;
    shape.polygon = null;
    shape.chainSegment = null;
    shape.type = b2_segmentShape;

    let wakeBodies = true;
    let destroyProxy = true;
    b2ResetProxy(world, shape, wakeBodies, destroyProxy);
}

export function b2Shape_SetPolygon(shapeId: b2ShapeId, polygon: b2Polygon): void {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return;
    }

    let shape = b2GetShape(world, shapeId);
    shape.polygon = clonePolygon(polygon);
    shape.circle = null;
    shape.capsule = null;
    shape.segment = null;
    shape.chainSegment = null;
    shape.type = b2_polygonShape;

    let wakeBodies = true;
    let destroyProxy = true;
    b2ResetProxy(world, shape, wakeBodies, destroyProxy);
}

export function b2Shape_GetParentChain(shapeId: b2ShapeId): b2ChainId {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    if (shape.type == b2_chainSegmentShape) {
        let chainId = shape.chainSegment!.chainId;
        if (chainId != B2_NULL_INDEX) {
            let chain = world.chainShapes.get(chainId);
            return new b2ChainId(chainId + 1, shapeId.world0, chain.generation);
        }
    }

    return new b2ChainId(0, 0, 0);
}

export function b2Chain_GetSurfaceMaterialCount(chainId: b2ChainId): i32 {
    let world = b2GetWorld(chainId.world0);
    let chainShape = b2GetChainShape(world, chainId);
    return chainShape.materialCount;
}

export function b2Chain_SetSurfaceMaterial(chainId: b2ChainId, material: b2SurfaceMaterial, materialIndex: i32): void {
    let world = b2GetWorldLocked(chainId.world0);
    if (world == null) {
        return;
    }

    let chainShape = b2GetChainShape(world, chainId);
    assert(0 <= materialIndex && materialIndex < chainShape.materialCount);
    chainShape.materials[materialIndex] = material.clone();

    assert(chainShape.materialCount == 1 || chainShape.materialCount == chainShape.count);
    let count = chainShape.count;

    if (chainShape.materialCount == 1) {
        for (let i = 0; i < count; ++i) {
            let shapeId = chainShape.shapeIndices[i];
            let shape = world.shapes.get(shapeId);
            shape.material = material.clone();
        }
    } else {
        let shapeId = chainShape.shapeIndices[materialIndex];
        let shape = world.shapes.get(shapeId);
        shape.material = material.clone();
    }
}

export function b2Chain_GetSurfaceMaterial(chainId: b2ChainId, segmentIndex: i32): b2SurfaceMaterial {
    let world = b2GetWorld(chainId.world0);
    let chainShape = b2GetChainShape(world, chainId);
    assert(0 <= segmentIndex && segmentIndex < chainShape.count);
    return chainShape.materials[segmentIndex].clone();
}

export function b2Shape_GetContactCapacity(shapeId: b2ShapeId): i32 {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return 0;
    }

    let shape = b2GetShape(world, shapeId);
    if (shape.sensorIndex != B2_NULL_INDEX) {
        return 0;
    }

    let body = world.bodies.get(shape.bodyId);

    // Conservative and fast
    return body.contactCount;
}

export function b2Shape_GetContactData(shapeId: b2ShapeId, contactData: Array<b2ContactData>, capacity: i32): i32 {
    let world = b2GetWorldLocked(shapeId.world0);
    if (world == null) {
        return 0;
    }

    let shape = b2GetShape(world, shapeId);
    if (shape.sensorIndex != B2_NULL_INDEX) {
        return 0;
    }

    let body = world.bodies.get(shape.bodyId);
    let contactKey = body.headContactKey;
    let index = 0;
    while (contactKey != B2_NULL_INDEX && index < capacity) {
        let contactId = contactKey >> 1;
        let edgeIndex = contactKey & 1;

        let contact = world.contacts.get(contactId);

        // Does contact involve this shape and is it touching?
        if ((contact.shapeIdA == shapeId.index1 - 1 || contact.shapeIdB == shapeId.index1 - 1) &&
            (contact.flags & b2_contactTouchingFlag) != 0) {
            let shapeA = world.shapes.get(contact.shapeIdA);
            let shapeB = world.shapes.get(contact.shapeIdB);

            let data = contactData[index];
            data.contactId = new b2ContactId(contact.contactId + 1, shapeId.world0, contact.generation);
            data.shapeIdA = new b2ShapeId(shapeA.id + 1, shapeId.world0, shapeA.generation);
            data.shapeIdB = new b2ShapeId(shapeB.id + 1, shapeId.world0, shapeB.generation);

            let contactSim = b2GetContactSim(world, contact);
            data.manifold = contactSim.manifold;
            index += 1;
        }

        contactKey = contact.edges[edgeIndex].nextKey;
    }

    assert(index <= capacity);

    return index;
}

export function b2Shape_GetAABB(shapeId: b2ShapeId): b2AABB {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return shape.aabb.clone();
}

export function b2Shape_ComputeMassData(shapeId: b2ShapeId): b2MassData {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    return b2ComputeShapeMass(shape);
}

export function b2Shape_GetClosestPoint(shapeId: b2ShapeId, target: b2Vec2): b2Vec2 {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);
    let body = world.bodies.get(shape.bodyId);
    let transform = b2GetBodyTransformQuick(world, body);

    let input = new b2DistanceInput();
    input.proxyA = b2MakeShapeDistanceProxy(shape);
    {
        let pts = makeVecArray(1);
        pts[0] = target.clone();
        input.proxyB = b2MakeProxy(pts, 1, 0.0);
    }
    input.transformA = transform;
    input.transformB = b2Transform_identity();
    input.useRadii = true;

    let cache = new b2SimplexCache();
    let output = b2ShapeDistance(input, cache);

    return output.pointA;
}

// force = 0.5 * air_density * velocity^2 * area
export function b2Shape_ApplyWind(shapeId: b2ShapeId, wind: b2Vec2, drag: f32, lift: f32, wake: bool): void {
    let world = b2GetWorld(shapeId.world0);
    let shape = b2GetShape(world, shapeId);

    let shapeType = shape.type;
    if (shapeType != b2_circleShape && shapeType != b2_capsuleShape && shapeType != b2_polygonShape) {
        return;
    }

    let body = world.bodies.get(shape.bodyId);

    if (body.type != b2_dynamicBody) {
        return;
    }

    if (body.setIndex >= b2_firstSleepingSet && wake == false) {
        return;
    }

    let sim = b2GetBodySim(world, body);

    if (body.setIndex != b2_awakeSet) {
        // Must wake for state to exist
        b2WakeBody(world, body);
    }

    assert(body.setIndex == b2_awakeSet);

    let state = b2GetBodyState(world, body)!;
    let transform = sim.transform;

    let lengthUnits = b2_lengthUnitsPerMeter;
    let volumeUnits = lengthUnits * lengthUnits * lengthUnits;

    // In 2D I'm assuming unit depth
    let airDensity = <f32>1.2250 / volumeUnits;

    let force = new b2Vec2();
    let torque: f32 = 0.0;

    switch (shape.type) {
        case b2_circleShape: {
            let radius = shape.circle!.radius;
            let centroid = shape.localCentroid;
            let lever = b2RotateVector(transform.q, b2Sub(centroid, sim.localCenter));
            let shapeVelocity = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, lever));
            let relativeVelocity = b2MulSub(wind, drag, shapeVelocity);
            let ln = b2GetLengthAndNormalize(relativeVelocity);
            let speed = ln.length;
            let direction = ln.normal;
            let projectedArea = <f32>2.0 * radius;
            force = b2MulSV(<f32>0.5 * airDensity * projectedArea * speed * speed, direction);
            torque = b2Cross(lever, force);
            break;
        }

        case b2_capsuleShape: {
            let centroid = shape.localCentroid;
            let lever = b2RotateVector(transform.q, b2Sub(centroid, sim.localCenter));
            let shapeVelocity = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, lever));
            let relativeVelocity = b2MulSub(wind, drag, shapeVelocity);
            let ln = b2GetLengthAndNormalize(relativeVelocity);
            let speed = ln.length;
            let direction = ln.normal;

            let d = b2Sub(shape.capsule!.center2, shape.capsule!.center1);
            d = b2RotateVector(transform.q, d);

            let radius = shape.capsule!.radius;
            let projectedArea = <f32>2.0 * radius + b2AbsFloat(b2Cross(d, direction));

            // Normal that opposes the wind
            let normal = b2LeftPerp(b2Normalize(d));
            normal = b2Dot(normal, direction) > 0.0 ? b2Neg(normal) : normal;

            // portion of wind that is perpendicular to surface
            let liftDirection = b2CrossSV(b2Cross(normal, direction), direction);

            let forceMagnitude = <f32>0.5 * airDensity * projectedArea * speed * speed;
            force = b2MulSV(forceMagnitude, b2MulAdd(direction, lift, liftDirection));

            let edgeLever = b2MulAdd(lever, radius, normal);
            torque = b2Cross(edgeLever, force);
            break;
        }

        case b2_polygonShape: {
            let centroid = shape.localCentroid;
            let lever = b2RotateVector(transform.q, b2Sub(centroid, sim.localCenter));
            let shapeVelocity = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, lever));
            let relativeVelocity = b2MulSub(wind, drag, shapeVelocity);
            let ln = b2GetLengthAndNormalize(relativeVelocity);
            let speed = ln.length;
            let direction = ln.normal;

            // polygon radius is ignored for simplicity
            let count = shape.polygon!.count;
            let vertices = shape.polygon!.vertices;

            let v1 = vertices[count - 1];
            for (let i = 0; i < count; ++i) {
                let v2 = vertices[i];
                let d = b2Sub(v2, v1);
                let edgeCenter = b2Lerp(v1, v2, 0.5);
                v1 = v2;

                d = b2RotateVector(transform.q, d);

                let projectedArea = b2Cross(d, direction);
                if (projectedArea <= 0.0) {
                    // back facing
                    continue;
                }

                let normal = b2RightPerp(b2Normalize(d));

                // portion of wind that is perpendicular to surface
                let liftDirection = b2CrossSV(b2Cross(normal, direction), direction);

                let forceMagnitude = <f32>0.5 * airDensity * projectedArea * speed * speed;
                let f = b2MulSV(forceMagnitude, b2MulAdd(direction, lift, liftDirection));

                let edgeLever = b2RotateVector(transform.q, b2Sub(edgeCenter, sim.localCenter));

                force = b2Add(force, f);
                torque += b2Cross(edgeLever, f);
            }
            break;
        }

        default:
            break;
    }

    sim.force = b2Add(sim.force, force);
    sim.torque += torque;
}
