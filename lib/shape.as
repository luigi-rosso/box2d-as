// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/shape.c

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX, B2_SECRET_COOKIE, b2GetLengthUnitsPerMeter, b2Log } from "./core";
import { B2_AABB_MARGIN_FRACTION, B2_HUGE, B2_LINEAR_SLOP, B2_MAX_AABB_MARGIN, B2_SPECULATIVE_DISTANCE } from "./constants";
import {
	B2_PI,
	b2AABB,
	b2AbsFloat,
	b2Add,
	b2Cross,
	b2CrossSV,
	b2Distance,
	b2DistanceSquared,
	b2Dot,
	b2GetLengthAndNormalize,
	b2InvMulWorldTransforms,
	b2InvRotateVector,
	b2InvTransformPoint,
	b2InvTransformWorldPoint,
	b2IsValidFloat,
	b2IsValidPos,
	b2IsValidVec2,
	b2LeftPerp,
	b2Length,
	b2LengthSquared,
	b2Lerp,
	b2MaxFloat,
	b2MinFloat,
	b2MinInt,
	b2MulAdd,
	b2MulSV,
	b2MulSub,
	b2Neg,
	b2Normalize,
	b2OffsetPos,
	b2Pos,
	b2RightPerp,
	b2Rot_identity,
	b2RotateVector,
	b2Sub,
	b2ToRelativeTransform,
	b2Transform,
	b2TransformPoint,
	b2TransformWorldPoint,
	b2Vec2,
	b2Vec2_zero,
	b2WorldTransform,
} from "./math_functions";
import {
	B2_MAX_POLYGON_VERTICES,
	b2Capsule,
	b2CastOutput,
	b2ChainSegment,
	b2Circle,
	b2DistanceInput,
	b2MassData,
	b2PlaneResult,
	b2Polygon,
	b2RayCastInput,
	b2Segment,
	b2ShapeCastInput,
	b2ShapeProxy,
	b2SimplexCache,
	b2WorldCastOutput,
} from "./collision";
import { b2BodyId, b2ChainId, b2ContactId, b2ShapeId, b2WorldId, b2_nullShapeId } from "./id";
import {
	b2BodyType,
	b2ChainDef,
	b2ContactData,
	b2DefaultShapeDef,
	b2Filter,
	b2QueryFilter,
	b2SensorEndTouchEvent,
	b2ShapeDef,
	b2ShapeType,
	b2SurfaceMaterial,
	b2_bodyTypeCount,
	b2_capsuleShape,
	b2_chainSegmentShape,
	b2_circleShape,
	b2_dynamicBody,
	b2_polygonShape,
	b2_segmentShape,
	b2_staticBody,
} from "./types";
import {
	b2CastRayCapsule,
	b2CastRayCircle,
	b2CastRayPolygon,
	b2CastRaySegment,
	b2CastShapeCapsule,
	b2CastShapeCircle,
	b2CastShapePolygon,
	b2CastShapeSegment,
	b2CollideMoverAndCapsule,
	b2CollideMoverAndCircle,
	b2CollideMoverAndPolygon,
	b2CollideMoverAndSegment,
	b2ComputeCapsuleBounds,
	b2ComputeCapsuleFatAABB,
	b2ComputeCapsuleMass,
	b2ComputeCircleBounds,
	b2ComputeCircleFatAABB,
	b2ComputeCircleMass,
	b2ComputePolygonBounds,
	b2ComputePolygonFatAABB,
	b2ComputePolygonMass,
	b2ComputeSegmentBounds,
	b2ComputeSegmentFatAABB,
	b2PointInCapsule,
	b2PointInCircle,
	b2PointInPolygon,
} from "./geometry";
import { b2ComputeShapeDistance, b2MakeProxy } from "./distance";
import { B2_PROXY_TYPE, b2BroadPhase_CreateProxy, b2BroadPhase_DestroyProxy, b2BroadPhase_MoveProxy } from "./broad_phase";
import { b2AllocId, b2FreeId } from "./id_pool";
import { b2Array_RemoveSwap } from "./container";
import { b2GetWorld, b2GetWorldLocked, b2World } from "./physics_world";
import {
	b2GetBodyFullId,
	b2GetBodySim,
	b2GetBodyState,
	b2GetBodyTransform,
	b2GetBodyTransformQuick,
	b2MakeBodyId,
	b2SyncBodyFlags,
	b2UpdateBodyMassData,
	b2WakeBody,
	b2_dirtyMass,
	b2ArrayView,
} from "./body";
import { b2_awakeSet, b2_disabledSet, b2_firstSleepingSet } from "./solver_set";
import { b2DestroyContact, b2GetContactSimArray, b2_contactTouchingFlag } from "./contact";
import { b2Sensor } from "./sensor";

export struct b2Shape {
	id: i32 = 0;
	bodyId: i32 = 0;
	prevShapeId: i32 = 0;
	nextShapeId: i32 = 0;
	sensorIndex: i32 = 0;
	type: b2ShapeType = 0;
	material: b2SurfaceMaterial = b2SurfaceMaterial();
	density: f32 = 0;
	aabbMargin: f32 = 0;
	aabb: b2AABB = b2AABB();
	localCentroid: b2Vec2 = b2Vec2();
	proxyKey: i32 = 0;

	filter: b2Filter = b2Filter();
	userData: u64 = 0;

	// C's union, one field per member
	capsule: b2Capsule = b2Capsule();
	circle: b2Circle = b2Circle();
	polygon: b2Polygon = b2Polygon();
	segment: b2Segment = b2Segment();
	chainSegment: b2ChainSegment = b2ChainSegment();

	generation: u16 = 0;
	enableSensorEvents: bool = false;
	enableContactEvents: bool = false;
	enableCustomFiltering: bool = false;
	enableHitEvents: bool = false;
	enablePreSolveEvents: bool = false;
}

export class b2ChainShape {
	id: i32 = 0;
	bodyId: i32 = 0;
	nextChainId: i32 = 0;
	segmentCount: i32 = 0;
	generation: u16 = 0;
	shapeIndices: StaticArray<i32> = new StaticArray<i32>(0);
}

export struct b2ShapeExtent {
	minExtent: f32 = 0;
	maxExtent: f32 = 0;
}

// Sensors are shapes that live in the broad-phase but never have contacts.
// At the end of the time step all sensors are queried for overlap with any other shapes.
// Sensors ignore body type and sleeping.
// Sensors generate events when there is a new overlap or and overlap disappears.
// The sensor overlaps don't get cleared until the next time step regardless of the overlapped
// shapes being destroyed.
// When a sensor is destroyed.
export class b2SensorOverlaps {
	overlaps: Array<i32> = new Array<i32>();
}

export function b2GetShapeRadius(shape: ref b2Shape): f32 {
	switch (shape.type) {
		case b2_capsuleShape:
			return shape.capsule.radius;
		case b2_circleShape:
			return shape.circle.radius;
		case b2_polygonShape:
			return shape.polygon.radius;
		default:
			return 0.0;
	}
}

@inline export function b2ShouldShapesCollide(filterA: b2Filter, filterB: b2Filter): bool {
	if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
		return filterA.groupIndex > 0;
	}

	return (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
}

@inline export function b2ShouldQueryCollide(shapeFilter: b2Filter, queryFilter: b2QueryFilter): bool {
	return (shapeFilter.categoryBits & queryFilter.maskBits) != 0 && (shapeFilter.maskBits & queryFilter.categoryBits) != 0;
}

// functions from shape.c follow

// Returns the shape index, C returns the pointer.
function b2GetShape(world: b2World, shapeId: b2ShapeId): i32 {
	let id = shapeId.index1 - 1;
	if (B2_ASSERT_ENABLED) {
		B2_ASSERT(0 <= id && id < world.shapes.length);
		B2_ASSERT(world.shapes[id].id == id && world.shapes[id].generation == shapeId.generation);
	}
	return id;
}

function b2GetChainShape(world: b2World, chainId: b2ChainId): b2ChainShape {
	let id = chainId.index1 - 1;
	let chain = world.chainShapes[id];
	if (B2_ASSERT_ENABLED) B2_ASSERT(chain.id == id && chain.generation == chainId.generation);
	return chain;
}

function b2ComputeShapeMargin(world: b2World, shapeId: i32): f32 {
	let shapes = world.shapes;
	let margin: f32 = 0.0;

	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			{
				margin = 0.5 * b2Distance(shapes[shapeId].capsule.center2, shapes[shapeId].capsule.center1) + shapes[shapeId].capsule.radius;
			}
			break;

		case b2_circleShape:
			{
				margin = shapes[shapeId].circle.radius;
			}
			break;

		case b2_polygonShape:
			{
				let centroid = shapes[shapeId].polygon.centroid;
				let maxExtentSqr: f32 = 0.0;
				let count = shapes[shapeId].polygon.count;
				for (let i = 0; i < count; ++i) {
					let distanceSqr = b2DistanceSquared(shapes[shapeId].polygon.vertices[i], centroid);
					maxExtentSqr = b2MaxFloat(maxExtentSqr, distanceSqr);
				}

				margin = Mathf.sqrt(maxExtentSqr);
			}
			break;

		case b2_segmentShape:
			{
				margin = 0.5 * b2Distance(shapes[shapeId].segment.point1, shapes[shapeId].segment.point2);
			}
			break;

		case b2_chainSegmentShape:
			{
				margin = 0.5 * b2Distance(shapes[shapeId].chainSegment.segment.point1, shapes[shapeId].chainSegment.segment.point2);
			}
			break;

		default:
			return B2_MAX_AABB_MARGIN();
	}

	return b2MinFloat(B2_MAX_AABB_MARGIN(), B2_AABB_MARGIN_FRACTION * margin);
}

// C writes the fat AABB through a pointer that is always world->fatAABBs.data + shape->id.
function b2UpdateShapeAABBs(world: b2World, shapeId: i32, transform: b2WorldTransform, proxyType: b2BodyType): void {
	// Compute a bounding box with a speculative margin
	let speculativeDistance = B2_SPECULATIVE_DISTANCE();
	let aabbMargin = world.shapes[shapeId].aabbMargin;

	let aabb = b2ComputeFatShapeAABB(world, shapeId, transform, speculativeDistance);
	world.shapes[shapeId].aabb = aabb;

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	let margin = proxyType == b2_staticBody ? speculativeDistance : aabbMargin;
	let fatAABB = b2AABB();
	fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
	fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
	fatAABB.upperBound.x = aabb.upperBound.x + margin;
	fatAABB.upperBound.y = aabb.upperBound.y + margin;
	world.fatAABBs[shapeId] = fatAABB;
}

// C's const void* geometry: the creator fills the member that matches the shape type.
struct b2ShapeGeometry {
	capsule: b2Capsule = b2Capsule();
	circle: b2Circle = b2Circle();
	polygon: b2Polygon = b2Polygon();
	segment: b2Segment = b2Segment();
	chainSegment: b2ChainSegment = b2ChainSegment();
}

// Returns the shape index, C returns the pointer.
function b2CreateShapeInternal(
	world: b2World,
	bodyId: i32,
	transform: b2WorldTransform,
	def: b2ShapeDef,
	geometry: ref b2ShapeGeometry,
	shapeType: b2ShapeType,
): i32 {
	let shapeId = b2AllocId(world.shapeIdPool);
	let shapes = world.shapes;

	if (shapeId == shapes.length) {
		shapes.push(b2Shape());
		world.fatAABBs.push(b2AABB());
	} else {
		if (B2_ASSERT_ENABLED) B2_ASSERT(shapes[shapeId].id == B2_NULL_INDEX);
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(world.fatAABBs.length == shapes.length);

	switch (shapeType) {
		case b2_capsuleShape:
			shapes[shapeId].capsule = geometry.capsule;
			break;

		case b2_circleShape:
			shapes[shapeId].circle = geometry.circle;
			break;

		case b2_polygonShape:
			shapes[shapeId].polygon = geometry.polygon;
			break;

		case b2_segmentShape:
			shapes[shapeId].segment = geometry.segment;
			break;

		case b2_chainSegmentShape:
			shapes[shapeId].chainSegment = geometry.chainSegment;
			break;

		default:
			B2_ASSERT(false);
			break;
	}

	shapes[shapeId].id = shapeId;
	shapes[shapeId].bodyId = world.bodies[bodyId].id;
	shapes[shapeId].type = shapeType;
	shapes[shapeId].density = def.density;
	shapes[shapeId].material = def.material;
	shapes[shapeId].filter = def.filter;
	shapes[shapeId].userData = def.userData;
	shapes[shapeId].enableSensorEvents = def.enableSensorEvents;
	shapes[shapeId].enableContactEvents = def.enableContactEvents;
	shapes[shapeId].enableCustomFiltering = def.enableCustomFiltering;
	shapes[shapeId].enableHitEvents = def.enableHitEvents;
	shapes[shapeId].enablePreSolveEvents = def.enablePreSolveEvents;
	shapes[shapeId].proxyKey = B2_NULL_INDEX;
	shapes[shapeId].localCentroid = b2GetShapeCentroid(world, shapeId);
	shapes[shapeId].aabbMargin = b2ComputeShapeMargin(world, shapeId);
	shapes[shapeId].aabb = b2AABB();
	world.fatAABBs[shapeId] = b2AABB();
	shapes[shapeId].generation += 1;

	if (world.bodies[bodyId].setIndex != b2_disabledSet) {
		let proxyType = world.bodies[bodyId].type;
		b2CreateShapeProxy(world, shapeId, proxyType, transform, def.invokeContactCreation || def.isSensor);
	}

	// Add to shape doubly linked list
	let headShapeId = world.bodies[bodyId].headShapeId;
	if (headShapeId != B2_NULL_INDEX) {
		shapes[headShapeId].prevShapeId = shapeId;
	}

	shapes[shapeId].prevShapeId = B2_NULL_INDEX;
	shapes[shapeId].nextShapeId = headShapeId;
	world.bodies[bodyId].headShapeId = shapeId;
	world.bodies[bodyId].shapeCount += 1;

	if (def.isSensor) {
		shapes[shapeId].sensorIndex = world.sensors.length;
		let sensor = new b2Sensor();
		sensor.shapeId = shapeId;
		world.sensors.push(sensor);
	} else {
		shapes[shapeId].sensorIndex = B2_NULL_INDEX;
	}

	return shapeId;
}

function b2IsValidMaterial(material: b2SurfaceMaterial): bool {
	return (
		b2IsValidFloat(material.friction) &&
		material.friction >= 0.0 &&
		b2IsValidFloat(material.restitution) &&
		material.restitution >= 0.0 &&
		b2IsValidFloat(material.rollingResistance) &&
		material.rollingResistance >= 0.0 &&
		b2IsValidFloat(material.tangentSpeed)
	);
}

function b2IsValidCircle(circle: b2Circle): bool {
	return b2IsValidVec2(circle.center) && b2IsValidFloat(circle.radius) && circle.radius >= 0.0;
}

function b2IsValidCapsule(capsule: b2Capsule): bool {
	return b2IsValidVec2(capsule.center1) && b2IsValidVec2(capsule.center2) && b2IsValidFloat(capsule.radius) && capsule.radius >= 0.0;
}

function b2IsValidSegment(segment: b2Segment): bool {
	return b2IsValidVec2(segment.point1) && b2IsValidVec2(segment.point2);
}

function b2IsValidChainSegment(chainSegment: b2ChainSegment): bool {
	return b2IsValidVec2(chainSegment.ghost1) && b2IsValidSegment(chainSegment.segment) && b2IsValidVec2(chainSegment.ghost2);
}

function b2IsValidPolygon(polygon: ref b2Polygon): bool {
	if (polygon.count < 1 || polygon.count > B2_MAX_POLYGON_VERTICES) {
		return false;
	}

	if (b2IsValidFloat(polygon.radius) == false || polygon.radius < 0.0) {
		return false;
	}

	if (b2IsValidVec2(polygon.centroid) == false) {
		return false;
	}

	for (let i = 0; i < polygon.count; ++i) {
		if (b2IsValidVec2(polygon.vertices[i]) == false || b2IsValidVec2(polygon.normals[i]) == false) {
			return false;
		}
	}

	return true;
}

function b2CreateShape(bodyId: b2BodyId, def: b2ShapeDef, geometry: ref b2ShapeGeometry, shapeType: b2ShapeType): b2ShapeId {
	if (B2_ASSERT_ENABLED) B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	if ((b2IsValidFloat(def.density) && def.density >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( def->density ) && def->density >= 0.0f in b2CreateShape\n");
		B2_ASSERT(false);
		return b2ShapeId();
	}
	if (b2IsValidMaterial(def.material) == false) {
		b2Log("invalid input: b2IsValidMaterial( &def->material ) in b2CreateShape\n");
		B2_ASSERT(false);
		return b2ShapeId();
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return b2ShapeId();
	}

	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);

	let shapeId = b2CreateShapeInternal(world, body, transform, def, geometry, shapeType);

	if (def.updateBodyMass == true) {
		b2UpdateBodyMassData(world, body);
	} else if ((world.bodies[body].flags & b2_dirtyMass) == 0) {
		world.bodies[body].flags |= b2_dirtyMass;
		b2SyncBodyFlags(world, body);
	}

	return b2ShapeId(world.shapes[shapeId].id + 1, bodyId.world0, world.shapes[shapeId].generation);
}

export function b2CreateCircleShape(bodyId: b2BodyId, def: b2ShapeDef, circle: b2Circle): b2ShapeId {
	if (b2IsValidCircle(circle) == false) {
		b2Log("invalid input: b2IsValidCircle( circle ) in b2CreateCircleShape\n");
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	let geometry = b2ShapeGeometry();
	geometry.circle = circle;
	return b2CreateShape(bodyId, def, geometry, b2_circleShape);
}

export function b2CreateCapsuleShape(bodyId: b2BodyId, def: b2ShapeDef, capsule: b2Capsule): b2ShapeId {
	if (b2IsValidCapsule(capsule) == false) {
		b2Log("invalid input: b2IsValidCapsule( capsule ) in b2CreateCapsuleShape\n");
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	let lengthSqr = b2DistanceSquared(capsule.center1, capsule.center2);
	if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
		return b2_nullShapeId;
	}

	let geometry = b2ShapeGeometry();
	geometry.capsule = capsule;
	return b2CreateShape(bodyId, def, geometry, b2_capsuleShape);
}

export function b2CreatePolygonShape(bodyId: b2BodyId, def: b2ShapeDef, polygon: b2Polygon): b2ShapeId {
	let geometry = b2ShapeGeometry();
	geometry.polygon = polygon;

	if (b2IsValidPolygon(geometry.polygon) == false) {
		b2Log("invalid input: b2IsValidPolygon( polygon ) in b2CreatePolygonShape\n");
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	return b2CreateShape(bodyId, def, geometry, b2_polygonShape);
}

export function b2CreateSegmentShape(bodyId: b2BodyId, def: b2ShapeDef, segment: b2Segment): b2ShapeId {
	if (b2IsValidSegment(segment) == false) {
		b2Log("invalid input: b2IsValidSegment( segment ) in b2CreateSegmentShape\n");
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	let lengthSqr = b2DistanceSquared(segment.point1, segment.point2);
	if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	let geometry = b2ShapeGeometry();
	geometry.segment = segment;
	return b2CreateShape(bodyId, def, geometry, b2_segmentShape);
}

export function b2CreateChainSegmentShape(bodyId: b2BodyId, def: b2ShapeDef, chainSegment: b2ChainSegment): b2ShapeId {
	if (b2IsValidChainSegment(chainSegment) == false) {
		b2Log("invalid input: b2IsValidChainSegment( chainSegment ) in b2CreateChainSegmentShape\n");
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	let lengthSqr = b2DistanceSquared(chainSegment.segment.point1, chainSegment.segment.point2);
	if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	// No parent chain shape
	let geometry = b2ShapeGeometry();
	geometry.chainSegment = chainSegment;
	geometry.chainSegment.chainId = B2_NULL_INDEX;

	return b2CreateShape(bodyId, def, geometry, b2_chainSegmentShape);
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
function b2DestroyShapeInternal(world: b2World, shapeId: i32, bodyId: i32): void {
	let shapes = world.shapes;
	let prevShapeId = shapes[shapeId].prevShapeId;
	let nextShapeId = shapes[shapeId].nextShapeId;

	// Remove the shape from the body's doubly linked list.
	if (prevShapeId != B2_NULL_INDEX) {
		shapes[prevShapeId].nextShapeId = nextShapeId;
	}

	if (nextShapeId != B2_NULL_INDEX) {
		shapes[nextShapeId].prevShapeId = prevShapeId;
	}

	if (shapeId == world.bodies[bodyId].headShapeId) {
		world.bodies[bodyId].headShapeId = nextShapeId;
	}

	world.bodies[bodyId].shapeCount -= 1;

	// Remove from broad-phase.
	b2DestroyShapeProxy(world, shapeId);

	// Destroy any contacts associated with the shape.
	b2DestroyShapeContacts(world, shapeId, bodyId);

	let sensorIndex = shapes[shapeId].sensorIndex;
	if (sensorIndex != B2_NULL_INDEX) {
		let sensor = world.sensors[sensorIndex];
		let endEvents = world.sensorEndEvents[world.endEventArrayIndex];
		for (let i = 0; i < sensor.overlaps2.length; ++i) {
			let visitor = sensor.overlaps2[i];
			let event = b2SensorEndTouchEvent();
			event.sensorShapeId = b2ShapeId(shapeId + 1, world.worldId, shapes[shapeId].generation);
			event.visitorShapeId = b2ShapeId(visitor.shapeId + 1, world.worldId, visitor.generation);

			endEvents.push(event);
		}

		// Destroy sensor
		let movedIndex = b2Array_RemoveSwap(world.sensors, sensorIndex);
		if (movedIndex != B2_NULL_INDEX) {
			// Fixup moved sensor
			let movedSensor = world.sensors[sensorIndex];
			shapes[movedSensor.shapeId].sensorIndex = sensorIndex;
		}
	}

	// Return shape to free list.
	b2FreeId(world.shapeIdPool, shapeId);
	shapes[shapeId].id = B2_NULL_INDEX;
}

// The contact loop shared by b2DestroyShapeInternal and b2ResetProxy.
function b2DestroyShapeContacts(world: b2World, shapeId: i32, bodyId: i32): void {
	let contacts = world.contacts;
	let contactKey = world.bodies[bodyId].headContactKey;
	while (contactKey != B2_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		contactKey = contacts[contactId].edges[edgeIndex].nextKey;

		if (contacts[contactId].shapeIdA == shapeId || contacts[contactId].shapeIdB == shapeId) {
			b2DestroyContact(world, contactId);
		}
	}
}

export function b2DestroyShape(shapeId: b2ShapeId, updateBodyMass: bool): void {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);

	// Cannot destroy a chain segment that has a parent chain shape
	if (world.shapes[shape].type == b2_chainSegmentShape && world.shapes[shape].chainSegment.chainId != B2_NULL_INDEX) {
		B2_ASSERT(false);
		return;
	}

	let body = world.shapes[shape].bodyId;
	b2DestroyShapeInternal(world, shape, body);

	if (updateBodyMass == true) {
		b2UpdateBodyMassData(world, body);
	}
}

export function b2CreateChain(bodyId: b2BodyId, def: b2ChainDef): b2ChainId {
	if (B2_ASSERT_ENABLED) B2_ASSERT(def.internalValue == B2_SECRET_COOKIE);
	if ((def.isLoop ? def.pointCount >= 3 : def.pointCount >= 2) == false) {
		b2Log("invalid input: def->isLoop ? def->pointCount >= 3 : def->pointCount >= 2 in b2CreateChain\n");
		B2_ASSERT(false);
		return b2ChainId();
	}

	let segmentCount = def.isLoop ? def.pointCount : def.pointCount - 1;

	if ((def.materialCount == 1 || def.materialCount == segmentCount) == false) {
		b2Log("invalid input: def->materialCount == 1 || def->materialCount == segmentCount in b2CreateChain\n");
		B2_ASSERT(false);
		return b2ChainId();
	}

	for (let i = 0; i < def.pointCount; ++i) {
		if (b2IsValidVec2(def.points[i]) == false) {
			b2Log("invalid input: b2IsValidVec2( def->points[i] ) in b2CreateChain\n");
			B2_ASSERT(false);
			return b2ChainId();
		}
	}

	for (let i = 0; i < def.materialCount; ++i) {
		if (b2IsValidMaterial(def.materials[i]) == false) {
			b2Log("invalid input: b2IsValidMaterial( def->materials + i ) in b2CreateChain\n");
			B2_ASSERT(false);
			return b2ChainId();
		}
	}

	if (def.isLoop == false) {
		// These are set to INFINITY in the default def as a sentinel to ensure they are set.
		if (b2IsValidVec2(def.ghost1) == false) {
			b2Log("invalid input: b2IsValidVec2( def->ghost1 ) in b2CreateChain\n");
			B2_ASSERT(false);
			return b2ChainId();
		}
		if (b2IsValidVec2(def.ghost2) == false) {
			b2Log("invalid input: b2IsValidVec2( def->ghost2 ) in b2CreateChain\n");
			B2_ASSERT(false);
			return b2ChainId();
		}
	}

	let world = b2GetWorldLocked(bodyId.world0);
	if (world == null) {
		return b2ChainId();
	}

	let body = b2GetBodyFullId(world, bodyId);
	let transform = b2GetBodyTransformQuick(world, body);

	let chainId = b2AllocId(world.chainIdPool);

	if (chainId == world.chainShapes.length) {
		world.chainShapes.push(new b2ChainShape());
	} else {
		if (B2_ASSERT_ENABLED) B2_ASSERT(world.chainShapes[chainId].id == B2_NULL_INDEX);
	}

	let chainShape = world.chainShapes[chainId];

	chainShape.id = chainId;
	chainShape.bodyId = world.bodies[body].id;
	chainShape.nextChainId = world.bodies[body].headChainId;
	chainShape.segmentCount = segmentCount;
	chainShape.generation += 1;

	let materialCount = def.materialCount;

	world.bodies[body].headChainId = chainId;

	let shapeDef = b2DefaultShapeDef();
	shapeDef.userData = def.userData;
	shapeDef.filter = def.filter;
	shapeDef.enableSensorEvents = def.enableSensorEvents;
	shapeDef.enableContactEvents = false;
	shapeDef.enableHitEvents = false;

	let points = def.points;
	let n = segmentCount;
	chainShape.shapeIndices = new StaticArray<i32>(n);

	let tolSqr: f32 = B2_LINEAR_SLOP() * B2_LINEAR_SLOP();

	let geometry = b2ShapeGeometry();

	if (def.isLoop) {
		let prevIndex = n - 1;
		for (let i = 0; i < n; ++i) {
			geometry.chainSegment.ghost1 = points[prevIndex];
			geometry.chainSegment.segment.point1 = points[i];
			geometry.chainSegment.segment.point2 = points[(i + 1) % n];
			geometry.chainSegment.ghost2 = points[(i + 2) % n];

			if (B2_ASSERT_ENABLED) {
				B2_ASSERT(b2DistanceSquared(geometry.chainSegment.ghost1, geometry.chainSegment.segment.point1) > tolSqr);
				B2_ASSERT(b2DistanceSquared(geometry.chainSegment.segment.point1, geometry.chainSegment.segment.point2) > tolSqr);
			}

			geometry.chainSegment.chainId = chainId;
			prevIndex = i;

			let materialIndex = materialCount == 1 ? 0 : i;
			shapeDef.material = def.materials[materialIndex];

			let shapeId = b2CreateShapeInternal(world, body, transform, shapeDef, geometry, b2_chainSegmentShape);
			chainShape.shapeIndices[i] = world.shapes[shapeId].id;
		}
	} else {
		for (let i = 0; i < n; ++i) {
			if (B2_ASSERT_ENABLED) B2_ASSERT(i + 1 < def.pointCount);
			geometry.chainSegment.ghost1 = i == 0 ? def.ghost1 : points[i - 1];
			geometry.chainSegment.segment.point1 = points[i + 0];
			geometry.chainSegment.segment.point2 = points[i + 1];
			geometry.chainSegment.ghost2 = i == n - 1 ? def.ghost2 : points[i + 2];

			if (B2_ASSERT_ENABLED) {
				B2_ASSERT(b2DistanceSquared(geometry.chainSegment.ghost1, geometry.chainSegment.segment.point1) > tolSqr);
				B2_ASSERT(b2DistanceSquared(geometry.chainSegment.segment.point1, geometry.chainSegment.segment.point2) > tolSqr);
				B2_ASSERT(b2DistanceSquared(geometry.chainSegment.segment.point2, geometry.chainSegment.ghost2) > tolSqr);
			}

			geometry.chainSegment.chainId = chainId;

			let materialIndex = materialCount == 1 ? 0 : i;
			shapeDef.material = def.materials[materialIndex];

			let shapeId = b2CreateShapeInternal(world, body, transform, shapeDef, geometry, b2_chainSegmentShape);
			chainShape.shapeIndices[i] = world.shapes[shapeId].id;
		}
	}

	return b2ChainId(chainId + 1, world.worldId, chainShape.generation);
}

const b2_emptyShapeIndices = new StaticArray<i32>(0);

export function b2FreeChainData(chain: b2ChainShape): void {
	chain.shapeIndices = b2_emptyShapeIndices;
}

export function b2DestroyChain(chainId: b2ChainId): void {
	let world = b2GetWorldLocked(chainId.world0);
	if (world == null) {
		return;
	}

	let chain = b2GetChainShape(world, chainId);

	let body = chain.bodyId;

	// Remove the chain from the body's singly linked list.
	let found = false;
	let prevChainId = B2_NULL_INDEX;
	let currentId = world.bodies[body].headChainId;
	while (currentId != B2_NULL_INDEX) {
		if (currentId == chain.id) {
			if (prevChainId == B2_NULL_INDEX) {
				world.bodies[body].headChainId = chain.nextChainId;
			} else {
				world.chainShapes[prevChainId].nextChainId = chain.nextChainId;
			}
			found = true;
			break;
		}

		prevChainId = currentId;
		currentId = world.chainShapes[currentId].nextChainId;
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(found == true);
	if (found == false) {
		return;
	}

	let count = chain.segmentCount;
	for (let i = 0; i < count; ++i) {
		let shapeId = chain.shapeIndices[i];
		b2DestroyShapeInternal(world, shapeId, body);
	}

	b2FreeChainData(chain);

	// Return chain to free list.
	b2FreeId(world.chainIdPool, chain.id);
	chain.id = B2_NULL_INDEX;
}

export function b2Chain_GetWorld(chainId: b2ChainId): b2WorldId {
	let world = b2GetWorld(chainId.world0);
	return b2WorldId(<u16>(chainId.world0 + 1), world.generation);
}

export function b2Chain_GetSegmentCount(chainId: b2ChainId): i32 {
	let world = b2GetWorldLocked(chainId.world0);
	if (world == null) {
		return 0;
	}

	let chain = b2GetChainShape(world, chainId);
	return chain.segmentCount;
}

export function b2Chain_GetSegments(chainId: b2ChainId, segmentArray: StaticArray<b2ShapeId>, capacity: i32): i32 {
	let world = b2GetWorldLocked(chainId.world0);
	if (world == null) {
		return 0;
	}

	let chain = b2GetChainShape(world, chainId);

	let count = b2MinInt(chain.segmentCount, capacity);
	for (let i = 0; i < count; ++i) {
		let shapeId = chain.shapeIndices[i];
		segmentArray[i] = b2ShapeId(shapeId + 1, chainId.world0, world.shapes[shapeId].generation);
	}

	return count;
}

export function b2Chain_SetSurfaceMaterial(chainId: b2ChainId, material: b2SurfaceMaterial, segmentIndex: i32): void {
	if (b2IsValidMaterial(material) == false) {
		b2Log("invalid input: b2IsValidMaterial( material ) in b2Chain_SetSurfaceMaterial\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(chainId.world0);
	if (world == null) {
		return;
	}

	let chainShape = b2GetChainShape(world, chainId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= segmentIndex && segmentIndex < chainShape.segmentCount);
	let shapeId = chainShape.shapeIndices[segmentIndex];
	world.shapes[shapeId].material = material;
}

export function b2Chain_SetAllSurfaceMaterials(chainId: b2ChainId, material: b2SurfaceMaterial): void {
	if (b2IsValidMaterial(material) == false) {
		b2Log("invalid input: b2IsValidMaterial( material ) in b2Chain_SetAllSurfaceMaterials\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(chainId.world0);
	if (world == null) {
		return;
	}

	let chainShape = b2GetChainShape(world, chainId);

	for (let i = 0; i < chainShape.segmentCount; ++i) {
		let shapeId = chainShape.shapeIndices[i];
		world.shapes[shapeId].material = material;
	}
}

export function b2Chain_GetSurfaceMaterial(chainId: b2ChainId, segmentIndex: i32): b2SurfaceMaterial {
	let world = b2GetWorld(chainId.world0);
	let chainShape = b2GetChainShape(world, chainId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= segmentIndex && segmentIndex < chainShape.segmentCount);
	let shapeId = chainShape.shapeIndices[segmentIndex];
	return world.shapes[shapeId].material;
}

export function b2ComputeShapeAABB(world: b2World, shapeId: i32, xf: b2WorldTransform): b2AABB {
	let shapes = b2ArrayView(world.shapes);
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			return b2ComputeCapsuleBounds(shapes[shapeId].capsule, xf);
		case b2_circleShape:
			return b2ComputeCircleBounds(shapes[shapeId].circle, xf);
		case b2_polygonShape:
			return b2ComputePolygonBounds(shapes[shapeId].polygon, xf);
		case b2_segmentShape:
			return b2ComputeSegmentBounds(shapes[shapeId].segment, xf);
		case b2_chainSegmentShape:
			return b2ComputeSegmentBounds(shapes[shapeId].chainSegment.segment, xf);
		default: {
			B2_ASSERT(false);
			return b2AABB();
		}
	}
}

// geometry.c in C, which reads b2Shape.
// Conservative world AABB for a shape, inflated by extra margin. In large world mode this is
// computed in double and rounded outward so the inflation is not lost far from the origin.
@inline
export function b2ComputeFatShapeAABB(world: b2World, shapeId: i32, xf: b2WorldTransform, extra: f32): b2AABB {
	let shapes = b2ArrayView(world.shapes);
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			return b2ComputeCapsuleFatAABB(shapes[shapeId].capsule, xf, extra);
		case b2_circleShape:
			return b2ComputeCircleFatAABB(shapes[shapeId].circle, xf, extra);
		case b2_polygonShape:
			return b2ComputePolygonFatAABB(shapes[shapeId].polygon, xf, extra);
		case b2_segmentShape:
			return b2ComputeSegmentFatAABB(shapes[shapeId].segment, xf, extra);
		case b2_chainSegmentShape:
			return b2ComputeSegmentFatAABB(shapes[shapeId].chainSegment.segment, xf, extra);
		default: {
			B2_ASSERT(false);
			return b2AABB();
		}
	}
}

export function b2GetShapeCentroid(world: b2World, shapeId: i32): b2Vec2 {
	let shapes = world.shapes;
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			return b2Lerp(shapes[shapeId].capsule.center1, shapes[shapeId].capsule.center2, 0.5);
		case b2_circleShape:
			return shapes[shapeId].circle.center;
		case b2_polygonShape:
			return shapes[shapeId].polygon.centroid;
		case b2_segmentShape:
			return b2Lerp(shapes[shapeId].segment.point1, shapes[shapeId].segment.point2, 0.5);
		case b2_chainSegmentShape:
			return b2Lerp(shapes[shapeId].chainSegment.segment.point1, shapes[shapeId].chainSegment.segment.point2, 0.5);
		default:
			return b2Vec2_zero;
	}
}

// todo_erin maybe compute this on shape creation
export function b2GetShapePerimeter(world: b2World, shapeId: i32): f32 {
	let shapes = world.shapes;
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			return (
				2.0 * b2Length(b2Sub(shapes[shapeId].capsule.center1, shapes[shapeId].capsule.center2)) +
				2.0 * B2_PI * shapes[shapeId].capsule.radius
			);
		case b2_circleShape:
			return 2.0 * B2_PI * shapes[shapeId].circle.radius;
		case b2_polygonShape: {
			let count = shapes[shapeId].polygon.count;
			let perimeter: f32 = 2.0 * B2_PI * shapes[shapeId].polygon.radius;
			if (B2_ASSERT_ENABLED) B2_ASSERT(count > 0);
			let prev = shapes[shapeId].polygon.vertices[count - 1];
			for (let i = 0; i < count; ++i) {
				let next = shapes[shapeId].polygon.vertices[i];
				perimeter += b2Length(b2Sub(next, prev));
				prev = next;
			}

			return perimeter;
		}
		case b2_segmentShape:
			return 2.0 * b2Length(b2Sub(shapes[shapeId].segment.point1, shapes[shapeId].segment.point2));
		case b2_chainSegmentShape:
			return 2.0 * b2Length(b2Sub(shapes[shapeId].chainSegment.segment.point1, shapes[shapeId].chainSegment.segment.point2));
		default:
			return 0.0;
	}
}

// This projects the shape perimeter onto an infinite line
export function b2GetShapeProjectedPerimeter(world: b2World, shapeId: i32, line: b2Vec2): f32 {
	let shapes = world.shapes;
	switch (shapes[shapeId].type) {
		case b2_capsuleShape: {
			let axis = b2Sub(shapes[shapeId].capsule.center2, shapes[shapeId].capsule.center1);
			let projectedLength = b2AbsFloat(b2Dot(axis, line));
			return projectedLength + 2.0 * shapes[shapeId].capsule.radius;
		}

		case b2_circleShape:
			return 2.0 * shapes[shapeId].circle.radius;

		case b2_polygonShape: {
			let count = shapes[shapeId].polygon.count;
			if (B2_ASSERT_ENABLED) B2_ASSERT(count > 0);
			let value = b2Dot(shapes[shapeId].polygon.vertices[0], line);
			let lower = value;
			let upper = value;
			for (let i = 1; i < count; ++i) {
				value = b2Dot(shapes[shapeId].polygon.vertices[i], line);
				lower = b2MinFloat(lower, value);
				upper = b2MaxFloat(upper, value);
			}

			return upper - lower + 2.0 * shapes[shapeId].polygon.radius;
		}

		case b2_segmentShape: {
			let value1 = b2Dot(shapes[shapeId].segment.point1, line);
			let value2 = b2Dot(shapes[shapeId].segment.point2, line);
			return b2AbsFloat(value2 - value1);
		}

		case b2_chainSegmentShape: {
			let value1 = b2Dot(shapes[shapeId].chainSegment.segment.point1, line);
			let value2 = b2Dot(shapes[shapeId].chainSegment.segment.point2, line);
			return b2AbsFloat(value2 - value1);
		}

		default:
			return 0.0;
	}
}

export function b2ComputeShapeMass(world: b2World, shapeId: i32): b2MassData {
	let shapes = world.shapes;
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			return b2ComputeCapsuleMass(shapes[shapeId].capsule, shapes[shapeId].density);
		case b2_circleShape:
			return b2ComputeCircleMass(shapes[shapeId].circle, shapes[shapeId].density);
		case b2_polygonShape:
			return b2ComputePolygonMass(shapes[shapeId].polygon, shapes[shapeId].density);
		default:
			return b2MassData();
	}
}

export function b2ComputeShapeExtent(world: b2World, shapeId: i32, localCenter: b2Vec2): b2ShapeExtent {
	let shapes = world.shapes;
	let extent = b2ShapeExtent();

	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			{
				let radius = shapes[shapeId].capsule.radius;
				extent.minExtent = radius;
				let c1 = b2Sub(shapes[shapeId].capsule.center1, localCenter);
				let c2 = b2Sub(shapes[shapeId].capsule.center2, localCenter);
				extent.maxExtent = Mathf.sqrt(b2MaxFloat(b2LengthSquared(c1), b2LengthSquared(c2))) + radius;
			}
			break;

		case b2_circleShape:
			{
				let radius = shapes[shapeId].circle.radius;
				extent.minExtent = radius;
				extent.maxExtent = b2Length(b2Sub(shapes[shapeId].circle.center, localCenter)) + radius;
			}
			break;

		case b2_polygonShape:
			{
				let centroid = shapes[shapeId].polygon.centroid;
				let minExtent = B2_HUGE();
				let maxExtentSqr: f32 = 0.0;
				let count = shapes[shapeId].polygon.count;
				for (let i = 0; i < count; ++i) {
					let v = shapes[shapeId].polygon.vertices[i];
					let planeOffset = b2Dot(shapes[shapeId].polygon.normals[i], b2Sub(v, centroid));
					minExtent = b2MinFloat(minExtent, planeOffset);

					let distanceSqr = b2LengthSquared(b2Sub(v, localCenter));
					maxExtentSqr = b2MaxFloat(maxExtentSqr, distanceSqr);
				}

				let radius = shapes[shapeId].polygon.radius;
				extent.minExtent = minExtent + radius;
				extent.maxExtent = Mathf.sqrt(maxExtentSqr) + radius;
			}
			break;

		case b2_segmentShape:
			{
				extent.minExtent = 0.0;
				let c1 = b2Sub(shapes[shapeId].segment.point1, localCenter);
				let c2 = b2Sub(shapes[shapeId].segment.point2, localCenter);
				extent.maxExtent = Mathf.sqrt(b2MaxFloat(b2LengthSquared(c1), b2LengthSquared(c2)));
			}
			break;

		case b2_chainSegmentShape:
			{
				extent.minExtent = 0.0;
				let c1 = b2Sub(shapes[shapeId].chainSegment.segment.point1, localCenter);
				let c2 = b2Sub(shapes[shapeId].chainSegment.segment.point2, localCenter);
				extent.maxExtent = Mathf.sqrt(b2MaxFloat(b2LengthSquared(c1), b2LengthSquared(c2)));
			}
			break;

		default:
			break;
	}

	return extent;
}

export function b2RayCastShape(input: b2RayCastInput, world: b2World, shapeId: i32, transform: b2Transform): b2CastOutput {
	let shapes = world.shapes;
	let localInput = input;
	localInput.origin = b2InvTransformPoint(transform, input.origin);
	localInput.translation = b2InvRotateVector(transform.q, input.translation);

	let output = b2CastOutput();
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			output = b2CastRayCapsule(shapes[shapeId].capsule, localInput);
			break;
		case b2_circleShape:
			output = b2CastRayCircle(shapes[shapeId].circle, localInput);
			break;
		case b2_polygonShape:
			output = b2CastRayPolygon(shapes[shapeId].polygon, localInput);
			break;
		case b2_segmentShape:
			output = b2CastRaySegment(shapes[shapeId].segment, localInput, false);
			break;
		case b2_chainSegmentShape:
			output = b2CastRaySegment(shapes[shapeId].chainSegment.segment, localInput, true);
			break;
		default:
			return output;
	}

	// The output point stays in the frame of the input transform, a caller chosen frame that is
	// typically re-centered near the origin.
	output.point = b2TransformPoint(transform, output.point);
	output.normal = b2RotateVector(transform.q, output.normal);
	return output;
}

export function b2ShapeCastShape(input: b2ShapeCastInput, world: b2World, shapeId: i32, transform: b2Transform): b2CastOutput {
	let shapes = world.shapes;
	let output = b2CastOutput();

	if (input.proxy.count == 0) {
		return output;
	}

	let localInput = input;

	for (let i = 0; i < localInput.proxy.count; ++i) {
		localInput.proxy.points[i] = b2InvTransformPoint(transform, input.proxy.points[i]);
	}

	localInput.translation = b2InvRotateVector(transform.q, input.translation);

	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			output = b2CastShapeCapsule(shapes[shapeId].capsule, localInput);
			break;
		case b2_circleShape:
			output = b2CastShapeCircle(shapes[shapeId].circle, localInput);
			break;
		case b2_polygonShape:
			output = b2CastShapePolygon(shapes[shapeId].polygon, localInput);
			break;
		case b2_segmentShape:
			output = b2CastShapeSegment(shapes[shapeId].segment, localInput);
			break;
		case b2_chainSegmentShape:
			{
				// Check for back side collision
				let approximateCentroid = localInput.proxy.points[0];
				for (let i = 1; i < localInput.proxy.count; ++i) {
					approximateCentroid = b2Add(approximateCentroid, localInput.proxy.points[i]);
				}

				approximateCentroid = b2MulSV(1.0 / <f32>localInput.proxy.count, approximateCentroid);

				let segment = shapes[shapeId].chainSegment.segment;
				let edge = b2Sub(segment.point2, segment.point1);
				let r = b2Sub(approximateCentroid, segment.point1);

				if (b2Cross(r, edge) < 0.0) {
					// Shape cast starts behind
					return output;
				}

				output = b2CastShapeSegment(segment, localInput);
			}
			break;
		default:
			return output;
	}

	// Same frame contract as b2RayCastShape, the point stays in the input transform frame
	output.point = b2TransformPoint(transform, output.point);
	output.normal = b2RotateVector(transform.q, output.normal);
	return output;
}

export function b2CollideMover(mover: b2Capsule, world: b2World, shapeId: i32, transform: b2Transform): b2PlaneResult {
	let shapes = world.shapes;
	let localMover = b2Capsule();
	localMover.center1 = b2InvTransformPoint(transform, mover.center1);
	localMover.center2 = b2InvTransformPoint(transform, mover.center2);
	localMover.radius = mover.radius;

	let result = b2PlaneResult();
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			result = b2CollideMoverAndCapsule(localMover, shapes[shapeId].capsule);
			break;
		case b2_circleShape:
			result = b2CollideMoverAndCircle(localMover, shapes[shapeId].circle);
			break;
		case b2_polygonShape:
			result = b2CollideMoverAndPolygon(localMover, shapes[shapeId].polygon);
			break;
		case b2_segmentShape:
			result = b2CollideMoverAndSegment(localMover, shapes[shapeId].segment);
			break;
		case b2_chainSegmentShape:
			result = b2CollideMoverAndSegment(localMover, shapes[shapeId].chainSegment.segment);
			break;
		default:
			return result;
	}

	if (result.hit == false) {
		return result;
	}

	result.plane.normal = b2RotateVector(transform.q, result.plane.normal);
	return result;
}

export function b2CreateShapeProxy(
	world: b2World,
	shapeId: i32,
	type: b2BodyType,
	transform: b2WorldTransform,
	forcePairCreation: bool,
): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[shapeId].proxyKey == B2_NULL_INDEX);

	b2UpdateShapeAABBs(world, shapeId, transform, type);

	// Create proxies in the broad-phase.
	let shapes = world.shapes;
	let id = shapes[shapeId].id;
	shapes[shapeId].proxyKey = b2BroadPhase_CreateProxy(
		world.broadPhase,
		type,
		world.fatAABBs[id],
		shapes[shapeId].filter.categoryBits,
		id,
		forcePairCreation,
	);
	if (B2_ASSERT_ENABLED) B2_ASSERT(B2_PROXY_TYPE(shapes[shapeId].proxyKey) < b2_bodyTypeCount);
}

// C takes the shape and the broad-phase, which is always the world's.
export function b2DestroyShapeProxy(world: b2World, shapeId: i32): void {
	let proxyKey = world.shapes[shapeId].proxyKey;
	if (proxyKey != B2_NULL_INDEX) {
		b2BroadPhase_DestroyProxy(world.broadPhase, proxyKey);
		world.shapes[shapeId].proxyKey = B2_NULL_INDEX;
	}
}

export function b2MakeShapeDistanceProxy(world: b2World, shapeId: i32): b2ShapeProxy {
	let shapes = world.shapes;
	let points: FixedArray<b2Vec2, 8>;
	switch (shapes[shapeId].type) {
		case b2_capsuleShape:
			points[0] = shapes[shapeId].capsule.center1;
			points[1] = shapes[shapeId].capsule.center2;
			return b2MakeProxy(points, 2, shapes[shapeId].capsule.radius);
		case b2_circleShape:
			points[0] = shapes[shapeId].circle.center;
			return b2MakeProxy(points, 1, shapes[shapeId].circle.radius);
		case b2_polygonShape:
			return b2MakeProxy(shapes[shapeId].polygon.vertices, shapes[shapeId].polygon.count, shapes[shapeId].polygon.radius);
		case b2_segmentShape:
			points[0] = shapes[shapeId].segment.point1;
			points[1] = shapes[shapeId].segment.point2;
			return b2MakeProxy(points, 2, 0.0);
		case b2_chainSegmentShape:
			points[0] = shapes[shapeId].chainSegment.segment.point1;
			points[1] = shapes[shapeId].chainSegment.segment.point2;
			return b2MakeProxy(points, 2, 0.0);
		default: {
			B2_ASSERT(false);
			return b2ShapeProxy();
		}
	}
}

export function b2Shape_GetBody(shapeId: b2ShapeId): b2BodyId {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return b2MakeBodyId(world, world.shapes[shape].bodyId);
}

export function b2Shape_GetWorld(shapeId: b2ShapeId): b2WorldId {
	let world = b2GetWorld(shapeId.world0);
	return b2WorldId(<u16>(shapeId.world0 + 1), world.generation);
}

export function b2Shape_SetUserData(shapeId: b2ShapeId, userData: u64): void {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].userData = userData;
}

export function b2Shape_GetUserData(shapeId: b2ShapeId): u64 {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].userData;
}

export function b2Shape_IsSensor(shapeId: b2ShapeId): bool {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].sensorIndex != B2_NULL_INDEX;
}

export function b2Shape_TestPoint(shapeId: b2ShapeId, point: b2Pos): bool {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	let shapes = world.shapes;

	let transform = b2GetBodyTransform(world, shapes[shape].bodyId);
	let localPoint = b2InvTransformWorldPoint(transform, point);

	let result: bool;
	switch (shapes[shape].type) {
		case b2_capsuleShape:
			result = b2PointInCapsule(shapes[shape].capsule, localPoint);
			break;

		case b2_circleShape:
			result = b2PointInCircle(shapes[shape].circle, localPoint);
			break;

		case b2_polygonShape:
			result = b2PointInPolygon(shapes[shape].polygon, localPoint);
			break;

		default:
			result = false;
			break;
	}

	return result;
}

export function b2Shape_CastRay(shapeId: b2ShapeId, origin: b2Pos, translation: b2Vec2): b2WorldCastOutput {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidPos(origin));
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidVec2(translation));

	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);

	// Re-center on the origin so the cast runs in float precision
	let transform = b2ToRelativeTransform(b2GetBodyTransform(world, world.shapes[shape].bodyId), origin);

	// The ray starts at the origin, so its origin in the re-centered frame is zero
	let input = b2RayCastInput(b2Vec2_zero, translation, 1.0);

	// Lift the re-centered float result back to a world position
	let local = b2RayCastShape(input, world, shape, transform);
	let output = b2CastOutput();
	output.normal = local.normal;
	output.point = b2OffsetPos(origin, local.point);
	output.fraction = local.fraction;
	output.iterations = local.iterations;
	output.hit = local.hit;

	return output;
}

export function b2Shape_SetDensity(shapeId: b2ShapeId, density: f32, updateBodyMass: bool): void {
	if ((b2IsValidFloat(density) && density >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( density ) && density >= 0.0f in b2Shape_SetDensity\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	if (density == world.shapes[shape].density) {
		// early return to avoid expensive function
		return;
	}

	world.shapes[shape].density = density;

	if (updateBodyMass == true) {
		b2UpdateBodyMassData(world, world.shapes[shape].bodyId);
	}
}

export function b2Shape_GetDensity(shapeId: b2ShapeId): f32 {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].density;
}

export function b2Shape_SetFriction(shapeId: b2ShapeId, friction: f32): void {
	if ((b2IsValidFloat(friction) && friction >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( friction ) && friction >= 0.0f in b2Shape_SetFriction\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(shapeId.world0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].material.friction = friction;
}

export function b2Shape_GetFriction(shapeId: b2ShapeId): f32 {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].material.friction;
}

export function b2Shape_SetRestitution(shapeId: b2ShapeId, restitution: f32): void {
	if ((b2IsValidFloat(restitution) && restitution >= 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( restitution ) && restitution >= 0.0f in b2Shape_SetRestitution\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(shapeId.world0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].material.restitution = restitution;
}

export function b2Shape_GetRestitution(shapeId: b2ShapeId): f32 {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].material.restitution;
}

export function b2Shape_SetUserMaterial(shapeId: b2ShapeId, material: u64): void {
	let world = b2GetWorld(shapeId.world0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].material.userMaterialId = material;
}

export function b2Shape_GetUserMaterial(shapeId: b2ShapeId): u64 {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].material.userMaterialId;
}

export function b2Shape_GetSurfaceMaterial(shapeId: b2ShapeId): b2SurfaceMaterial {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].material;
}

export function b2Shape_SetSurfaceMaterial(shapeId: b2ShapeId, surfaceMaterial: b2SurfaceMaterial): void {
	if (b2IsValidMaterial(surfaceMaterial) == false) {
		b2Log("invalid input: b2IsValidMaterial( surfaceMaterial ) in b2Shape_SetSurfaceMaterial\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].material = surfaceMaterial;
}

export function b2Shape_GetFilter(shapeId: b2ShapeId): b2Filter {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].filter;
}

function b2ResetProxy(world: b2World, shapeId: i32, destroyProxy: bool): void {
	let shapes = world.shapes;
	let body = shapes[shapeId].bodyId;

	// destroy all contacts associated with this shape
	b2DestroyShapeContacts(world, shapeId, body);

	let transform = b2GetBodyTransformQuick(world, body);
	let proxyKey = shapes[shapeId].proxyKey;
	if (proxyKey != B2_NULL_INDEX) {
		let proxyType = B2_PROXY_TYPE(proxyKey);
		b2UpdateShapeAABBs(world, shapeId, transform, proxyType);

		if (destroyProxy) {
			b2BroadPhase_DestroyProxy(world.broadPhase, proxyKey);

			let forcePairCreation = true;
			shapes[shapeId].proxyKey = b2BroadPhase_CreateProxy(
				world.broadPhase,
				proxyType,
				world.fatAABBs[shapeId],
				shapes[shapeId].filter.categoryBits,
				shapeId,
				forcePairCreation,
			);
		} else {
			b2BroadPhase_MoveProxy(world.broadPhase, proxyKey, world.fatAABBs[shapeId]);
		}
	} else {
		let proxyType = world.bodies[body].type;
		b2UpdateShapeAABBs(world, shapeId, transform, proxyType);
	}
}

export function b2Shape_SetFilter(shapeId: b2ShapeId, filter: b2Filter): void {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	let oldFilter = world.shapes[shape].filter;
	if (filter.maskBits == oldFilter.maskBits && filter.categoryBits == oldFilter.categoryBits && filter.groupIndex == oldFilter.groupIndex) {
		return;
	}

	// If the category bits change, I need to destroy the proxy because it affects the tree sorting.
	let destroyProxy = filter.categoryBits != oldFilter.categoryBits;

	world.shapes[shape].filter = filter;

	b2ResetProxy(world, shape, destroyProxy);

	// note: this does not immediately update sensor overlaps. Instead sensor
	// overlaps are updated the next time step
}

export function b2Shape_EnableSensorEvents(shapeId: b2ShapeId, flag: bool): void {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].enableSensorEvents = flag;
}

export function b2Shape_AreSensorEventsEnabled(shapeId: b2ShapeId): bool {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].enableSensorEvents;
}

export function b2Shape_EnableContactEvents(shapeId: b2ShapeId, flag: bool): void {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].enableContactEvents = flag;
}

export function b2Shape_AreContactEventsEnabled(shapeId: b2ShapeId): bool {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].enableContactEvents;
}

export function b2Shape_EnablePreSolveEvents(shapeId: b2ShapeId, flag: bool): void {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].enablePreSolveEvents = flag;
}

export function b2Shape_ArePreSolveEventsEnabled(shapeId: b2ShapeId): bool {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].enablePreSolveEvents;
}

export function b2Shape_EnableHitEvents(shapeId: b2ShapeId, flag: bool): void {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].enableHitEvents = flag;
}

export function b2Shape_AreHitEventsEnabled(shapeId: b2ShapeId): bool {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].enableHitEvents;
}

export function b2Shape_GetType(shapeId: b2ShapeId): b2ShapeType {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].type;
}

export function b2Shape_GetCircle(shapeId: b2ShapeId): b2Circle {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[shape].type == b2_circleShape);
	return world.shapes[shape].circle;
}

export function b2Shape_GetSegment(shapeId: b2ShapeId): b2Segment {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[shape].type == b2_segmentShape);
	return world.shapes[shape].segment;
}

export function b2Shape_GetChainSegment(shapeId: b2ShapeId): b2ChainSegment {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[shape].type == b2_chainSegmentShape);
	return world.shapes[shape].chainSegment;
}

export function b2Shape_GetCapsule(shapeId: b2ShapeId): b2Capsule {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[shape].type == b2_capsuleShape);
	return world.shapes[shape].capsule;
}

export function b2Shape_GetPolygon(shapeId: b2ShapeId): b2Polygon {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	if (B2_ASSERT_ENABLED) B2_ASSERT(world.shapes[shape].type == b2_polygonShape);
	return world.shapes[shape].polygon;
}

export function b2Shape_SetCircle(shapeId: b2ShapeId, circle: b2Circle): void {
	if (b2IsValidCircle(circle) == false) {
		b2Log("invalid input: b2IsValidCircle( circle ) in b2Shape_SetCircle\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].circle = circle;
	world.shapes[shape].type = b2_circleShape;
	world.shapes[shape].aabbMargin = b2ComputeShapeMargin(world, shape);

	let destroyProxy = true;
	b2ResetProxy(world, shape, destroyProxy);
}

export function b2Shape_SetCapsule(shapeId: b2ShapeId, capsule: b2Capsule): void {
	if (b2IsValidCapsule(capsule) == false) {
		b2Log("invalid input: b2IsValidCapsule( capsule ) in b2Shape_SetCapsule\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let lengthSqr = b2DistanceSquared(capsule.center1, capsule.center2);
	if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].capsule = capsule;
	world.shapes[shape].type = b2_capsuleShape;
	world.shapes[shape].aabbMargin = b2ComputeShapeMargin(world, shape);

	let destroyProxy = true;
	b2ResetProxy(world, shape, destroyProxy);
}

export function b2Shape_SetSegment(shapeId: b2ShapeId, segment: b2Segment): void {
	if (b2IsValidSegment(segment) == false) {
		b2Log("invalid input: b2IsValidSegment( segment ) in b2Shape_SetSegment\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].segment = segment;
	world.shapes[shape].type = b2_segmentShape;
	world.shapes[shape].aabbMargin = b2ComputeShapeMargin(world, shape);

	let destroyProxy = true;
	b2ResetProxy(world, shape, destroyProxy);
}

export function b2Shape_SetPolygon(shapeId: b2ShapeId, polygon: b2Polygon): void {
	if (b2IsValidPolygon(polygon) == false) {
		b2Log("invalid input: b2IsValidPolygon( polygon ) in b2Shape_SetPolygon\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	world.shapes[shape].polygon = polygon;
	world.shapes[shape].type = b2_polygonShape;
	world.shapes[shape].aabbMargin = b2ComputeShapeMargin(world, shape);

	let destroyProxy = true;
	b2ResetProxy(world, shape, destroyProxy);
}

export function b2Shape_SetChainSegment(shapeId: b2ShapeId, chainSegment: b2ChainSegment): void {
	if (b2IsValidChainSegment(chainSegment) == false) {
		b2Log("invalid input: b2IsValidChainSegment( chainSegment ) in b2Shape_SetChainSegment\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b2GetShape(world, shapeId);
	let shapes = world.shapes;

	// Cannot modify a chain segment that has a parent chain shape
	if (shapes[shape].type == b2_chainSegmentShape && shapes[shape].chainSegment.chainId != B2_NULL_INDEX) {
		B2_ASSERT(false);
		return;
	}

	let lengthSqr = b2DistanceSquared(chainSegment.segment.point1, chainSegment.segment.point2);
	if (lengthSqr <= B2_LINEAR_SLOP() * B2_LINEAR_SLOP()) {
		return;
	}

	shapes[shape].chainSegment = chainSegment;
	shapes[shape].chainSegment.chainId = B2_NULL_INDEX;
	shapes[shape].type = b2_chainSegmentShape;
	shapes[shape].aabbMargin = b2ComputeShapeMargin(world, shape);

	let destroyProxy = true;
	b2ResetProxy(world, shape, destroyProxy);
}

export function b2Shape_GetParentChain(shapeId: b2ShapeId): b2ChainId {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	if (world.shapes[shape].type == b2_chainSegmentShape) {
		let chainId = world.shapes[shape].chainSegment.chainId;
		if (chainId != B2_NULL_INDEX) {
			let chain = world.chainShapes[chainId];
			return b2ChainId(chainId + 1, shapeId.world0, chain.generation);
		}
	}

	return b2ChainId();
}

export function b2Shape_GetContactCapacity(shapeId: b2ShapeId): i32 {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b2GetShape(world, shapeId);
	if (world.shapes[shape].sensorIndex != B2_NULL_INDEX) {
		return 0;
	}

	// Conservative and fast
	return world.bodies[world.shapes[shape].bodyId].contactCount;
}

export function b2Shape_GetContactData(shapeId: b2ShapeId, contactData: StaticArray<b2ContactData>, capacity: i32): i32 {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b2GetShape(world, shapeId);
	let shapes = world.shapes;
	if (shapes[shape].sensorIndex != B2_NULL_INDEX) {
		return 0;
	}

	let contactKey = world.bodies[shapes[shape].bodyId].headContactKey;
	let index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];

		// Does contact involve this shape and is it touching?
		if (
			(contact.shapeIdA == shapeId.index1 - 1 || contact.shapeIdB == shapeId.index1 - 1) &&
			(contact.flags & b2_contactTouchingFlag) != 0
		) {
			let shapeIdA = contact.shapeIdA;
			let shapeIdB = contact.shapeIdB;

			let id = b2ContactId();
			id.index1 = contact.contactId + 1;
			id.world0 = shapeId.world0;
			id.generation = contact.generation;
			contactData[index].contactId = id;
			contactData[index].shapeIdA = b2ShapeId(shapes[shapeIdA].id + 1, shapeId.world0, shapes[shapeIdA].generation);
			contactData[index].shapeIdB = b2ShapeId(shapes[shapeIdB].id + 1, shapeId.world0, shapes[shapeIdB].generation);

			let contactSims = b2GetContactSimArray(world, contactId);
			contactData[index].manifold = contactSims[contact.localIndex].manifold;
			index += 1;
		}

		contactKey = contact.edges[edgeIndex].nextKey;
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(index <= capacity);

	return index;
}

export function b2Shape_GetSensorCapacity(shapeId: b2ShapeId): i32 {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b2GetShape(world, shapeId);
	let sensorIndex = world.shapes[shape].sensorIndex;
	if (sensorIndex == B2_NULL_INDEX) {
		return 0;
	}

	return world.sensors[sensorIndex].overlaps2.length;
}

export function b2Shape_GetSensorData(shapeId: b2ShapeId, visitorIds: StaticArray<b2ShapeId>, capacity: i32): i32 {
	let world = b2GetWorldLocked(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b2GetShape(world, shapeId);
	let sensorIndex = world.shapes[shape].sensorIndex;
	if (sensorIndex == B2_NULL_INDEX) {
		return 0;
	}

	let refs = world.sensors[sensorIndex].overlaps2;

	let count = b2MinInt(refs.length, capacity);
	for (let i = 0; i < count; ++i) {
		visitorIds[i] = b2ShapeId(refs[i].shapeId + 1, shapeId.world0, refs[i].generation);
	}

	return count;
}

// b2GetWorld never returns null here, so C's null checks in the next three functions are dropped.
export function b2Shape_GetBounds(shapeId: b2ShapeId): b2AABB {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return world.shapes[shape].aabb;
}

export function b2Shape_ComputeMassData(shapeId: b2ShapeId): b2MassData {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	return b2ComputeShapeMass(world, shape);
}

export function b2Shape_GetClosestPoint(shapeId: b2ShapeId, target: b2Pos): b2Pos {
	let world = b2GetWorld(shapeId.world0);
	let shape = b2GetShape(world, shapeId);
	let transform = b2GetBodyTransformQuick(world, world.shapes[shape].bodyId);

	// The target rides in as the frame of proxy B, so the relative pose is differenced in double
	// and the result stays exact far from the origin
	let zero: FixedArray<b2Vec2, 8>;
	let targetTransform = b2Transform(target, b2Rot_identity);

	let input = b2DistanceInput();
	input.proxyA = b2MakeShapeDistanceProxy(world, shape);
	input.proxyB = b2MakeProxy(zero, 1, 0.0);
	input.transform = b2InvMulWorldTransforms(transform, targetTransform);
	input.useRadii = true;

	let cache = b2SimplexCache();
	let output = b2ComputeShapeDistance(input, cache, null, 0);

	return b2TransformWorldPoint(transform, output.pointA);
}

// Holds the length b2GetLengthAndNormalize writes, which must bind a struct leaf.
struct b2WindSpeed {
	speed: f32 = 0;
}

// https://en.wikipedia.org/wiki/Density_of_air
// https://www.engineeringtoolbox.com/wind-load-d_1775.html
// force = 0.5 * air_density * velocity^2 * area
// https://en.wikipedia.org/wiki/Lift_(force)
export function b2Shape_ApplyWind(shapeId: b2ShapeId, wind: b2Vec2, drag: f32, lift: f32, wake: bool): void {
	if (b2IsValidVec2(wind) == false) {
		b2Log("invalid input: b2IsValidVec2( wind ) in b2Shape_ApplyWind\n");
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidFloat(drag) == false) {
		b2Log("invalid input: b2IsValidFloat( drag ) in b2Shape_ApplyWind\n");
		B2_ASSERT(false);
		return;
	}
	if (b2IsValidFloat(lift) == false) {
		b2Log("invalid input: b2IsValidFloat( lift ) in b2Shape_ApplyWind\n");
		B2_ASSERT(false);
		return;
	}

	let world = b2GetWorld(shapeId.world0);

	let shape = b2GetShape(world, shapeId);
	let shapes = world.shapes;

	let shapeType = shapes[shape].type;
	if (shapeType != b2_circleShape && shapeType != b2_capsuleShape && shapeType != b2_polygonShape) {
		return;
	}

	let body = shapes[shape].bodyId;

	if (world.bodies[body].type != b2_dynamicBody) {
		return;
	}

	if (world.bodies[body].setIndex >= b2_firstSleepingSet && wake == false) {
		return;
	}

	// C keeps the sim pointer across the wake, so a woken body's force lands in the destroyed sleeping set.
	let sim = b2GetBodySim(world, body);
	let wasAwake = world.bodies[body].setIndex == b2_awakeSet;

	if (world.bodies[body].setIndex != b2_awakeSet) {
		// Must wake for state to exist
		b2WakeBody(world, body);
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(world.bodies[body].setIndex == b2_awakeSet);

	let stateIndex = b2GetBodyState(world, body);
	let state = world.solverSets[b2_awakeSet].bodyStates[stateIndex];
	let transform = sim.transform;

	let lengthUnits = b2GetLengthUnitsPerMeter();
	let volumeUnits: f32 = lengthUnits * lengthUnits * lengthUnits;

	// In 2D I'm assuming unit depth
	let airDensity: f32 = 1.225 / volumeUnits;

	let force = b2Vec2_zero;
	let torque: f32 = 0.0;
	let length = b2WindSpeed();

	switch (shapes[shape].type) {
		case b2_circleShape:
			{
				let radius = shapes[shape].circle.radius;
				let centroid = shapes[shape].localCentroid;
				let lever = b2RotateVector(transform.q, b2Sub(centroid, sim.localCenter));
				let shapeVelocity = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, lever));
				let relativeVelocity = b2MulSub(wind, drag, shapeVelocity);
				let direction = b2GetLengthAndNormalize(length.speed, relativeVelocity);
				let speed = length.speed;
				let projectedArea: f32 = 2.0 * radius;
				force = b2MulSV(0.5 * airDensity * projectedArea * speed * speed, direction);
				torque = b2Cross(lever, force);
			}
			break;

		case b2_capsuleShape:
			{
				let centroid = shapes[shape].localCentroid;
				let lever = b2RotateVector(transform.q, b2Sub(centroid, sim.localCenter));
				let shapeVelocity = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, lever));
				let relativeVelocity = b2MulSub(wind, drag, shapeVelocity);
				let direction = b2GetLengthAndNormalize(length.speed, relativeVelocity);
				let speed = length.speed;

				let d = b2Sub(shapes[shape].capsule.center2, shapes[shape].capsule.center1);
				d = b2RotateVector(transform.q, d);

				let radius = shapes[shape].capsule.radius;
				let projectedArea: f32 = 2.0 * radius + b2AbsFloat(b2Cross(d, direction));

				// Normal that opposes the wind
				let normal = b2LeftPerp(b2Normalize(d));
				normal = b2Dot(normal, direction) > 0.0 ? b2Neg(normal) : normal;

				// portion of wind that is perpendicular to surface
				let liftDirection = b2CrossSV(b2Cross(normal, direction), direction);

				let forceMagnitude: f32 = 0.5 * airDensity * projectedArea * speed * speed;
				force = b2MulSV(forceMagnitude, b2MulAdd(direction, lift, liftDirection));

				let edgeLever = b2MulAdd(lever, radius, normal);
				torque = b2Cross(edgeLever, force);
			}
			break;

		case b2_polygonShape:
			{
				let centroid = shapes[shape].localCentroid;
				let lever = b2RotateVector(transform.q, b2Sub(centroid, sim.localCenter));
				let shapeVelocity = b2Add(state.linearVelocity, b2CrossSV(state.angularVelocity, lever));
				let relativeVelocity = b2MulSub(wind, drag, shapeVelocity);
				let direction = b2GetLengthAndNormalize(length.speed, relativeVelocity);
				let speed = length.speed;

				// polygon radius is ignored for simplicity
				let count = shapes[shape].polygon.count;

				let v1 = shapes[shape].polygon.vertices[count - 1];
				for (let i = 0; i < count; ++i) {
					let v2 = shapes[shape].polygon.vertices[i];
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

					let forceMagnitude: f32 = 0.5 * airDensity * projectedArea * speed * speed;
					let f = b2MulSV(forceMagnitude, b2MulAdd(direction, lift, liftDirection));

					let edgeLever = b2RotateVector(transform.q, b2Sub(edgeCenter, sim.localCenter));

					force = b2Add(force, f);
					torque += b2Cross(edgeLever, f);
				}
			}
			break;

		default:
			break;
	}

	if (wasAwake) {
		let sims = world.solverSets[b2_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].force = b2Add(sims[localIndex].force, force);
		sims[localIndex].torque += torque;
	}
}
