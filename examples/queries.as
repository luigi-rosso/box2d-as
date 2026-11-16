// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// World queries over three boxes in a row: the closest ray hit, every ray hit, a circle cast, and overlaps of a box
// and of a shape. Each box's shape carries its index as user data.

import {
	b2AABB,
	b2CreateBody,
	b2CreatePolygonShape,
	b2CreateWorld,
	b2DefaultBodyDef,
	b2DefaultQueryFilter,
	b2DefaultShapeDef,
	b2DefaultWorldDef,
	b2DestroyWorld,
	b2MakeBox,
	b2MakeProxy,
	b2Pos,
	b2Pos_zero,
	b2ShapeId,
	b2Shape_GetUserData,
	b2Vec2,
	b2World_CastRay,
	b2World_CastRayClosest,
	b2World_CastShape,
	b2World_OverlapAABB,
	b2World_OverlapShape,
} from "../lib/box2d";

export class QueriesResult {
	closestBox: i32 = -1;
	closestHeight: f32 = 0;
	rayHitCount: i32 = 0;
	castBox: i32 = -1;
	castFraction: f32 = 0;
	aabbOverlaps: i32 = 0;
	shapeOverlaps: i32 = 0;
}

// The user data of boxes is 1 + index, so the ground's 0 reads as -1
function boxIndex(shapeId: b2ShapeId): i32 {
	return <i32>b2Shape_GetUserData(shapeId) - 1;
}

export function queries(): QueriesResult {
	let worldId = b2CreateWorld(b2DefaultWorldDef());

	let groundDef = b2DefaultBodyDef();
	groundDef.position = b2Vec2(0.0, -1.0);
	let groundId = b2CreateBody(worldId, groundDef);
	b2CreatePolygonShape(groundId, b2DefaultShapeDef(), b2MakeBox(10.0, 1.0));

	// 1 m boxes on the ground at x = -4, 0 and 4. Static, so the queries see them without a step.
	for (let i = 0; i < 3; ++i) {
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(<f32>(4 * i - 4), 0.5);
		let bodyId = b2CreateBody(worldId, bodyDef);
		let shapeDef = b2DefaultShapeDef();
		shapeDef.userData = <u64>(i + 1);
		b2CreatePolygonShape(bodyId, shapeDef, b2MakeBox(0.5, 0.5));
	}

	let filter = b2DefaultQueryFilter();
	let result = new QueriesResult();

	// Straight down onto the middle box
	let ray = b2World_CastRayClosest(worldId, b2Vec2(0.0, 10.0), b2Vec2(0.0, -20.0), filter);
	if (ray.hit) {
		result.closestBox = boxIndex(ray.shapeId);
		result.closestHeight = ray.point.y;
	}

	// Along the row through all three boxes. Returning 1 keeps the full ray, so every shape on it reports.
	b2World_CastRay(worldId, b2Vec2(-10.0, 0.5), b2Vec2(20.0, 0.0), filter,
		(shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32): f32 => {
			result.rayHitCount += 1;
			return 1.0;
		});

	// A circle of radius 0.25, one point at the cast's origin, swept along the same line. Returning the fraction
	// clips the cast there, so the last report is the closest hit.
	let center: FixedArray<b2Vec2, 8>;
	let circle = b2MakeProxy(center, 1, 0.25);
	b2World_CastShape(worldId, b2Vec2(-10.0, 0.5), circle, b2Vec2(20.0, 0.0), filter,
		(shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32): f32 => {
			result.castBox = boxIndex(shapeId);
			result.castFraction = fraction;
			return fraction;
		});

	// Bounding boxes overlapping a box around the last box, clear of the ground
	let aabb = b2AABB(b2Vec2(3.0, 0.2), b2Vec2(5.0, 2.0));
	b2World_OverlapAABB(worldId, b2Pos_zero, aabb, filter, (shapeId: b2ShapeId): bool => {
		result.aabbOverlaps += 1;
		return true;
	});

	// Shapes touching a 1 m circle centered on the last box: the box and the ground under it
	let ball = b2MakeProxy(center, 1, 1.0);
	b2World_OverlapShape(worldId, b2Vec2(4.0, 0.5), ball, filter, (shapeId: b2ShapeId): bool => {
		result.shapeOverlaps += 1;
		return true;
	});

	b2DestroyWorld(worldId);
	return result;
}
