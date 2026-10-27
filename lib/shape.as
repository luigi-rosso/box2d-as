// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/shape.c

import { b2AABB, b2Vec2 } from "./math_functions";
import { b2Capsule, b2ChainSegment, b2Circle, b2Polygon, b2Segment } from "./collision";
import { b2Filter, b2QueryFilter, b2ShapeType, b2SurfaceMaterial, b2_capsuleShape, b2_circleShape, b2_polygonShape } from "./types";

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

export function b2GetShapeRadius(shape: b2Shape): f32 {
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
