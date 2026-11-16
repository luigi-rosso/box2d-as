// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Shapes: one body of each kind, a circle, a capsule, a box, a rounded box and a hull, dropped into a tray made of
// a chain. Each comes to rest on the tray's floor and falls asleep.

import {
	b2BodyId,
	b2Body_GetPosition,
	b2Body_IsAwake,
	b2Capsule,
	b2Circle,
	b2ComputeHull,
	b2CreateBody,
	b2CreateCapsuleShape,
	b2CreateChain,
	b2CreateCircleShape,
	b2CreatePolygonShape,
	b2CreateWorld,
	b2DefaultBodyDef,
	b2DefaultChainDef,
	b2DefaultShapeDef,
	b2DefaultWorldDef,
	b2DestroyWorld,
	b2MakeBox,
	b2MakePolygon,
	b2MakeRoundedBox,
	b2Vec2,
	b2WorldId,
	b2World_Step,
	b2_dynamicBody,
} from "../lib/box2d";

export class ShapesResult {
	circleHeight: f32 = 0;
	capsuleHeight: f32 = 0;
	boxHeight: f32 = 0;
	roundedHeight: f32 = 0;
	hullHeight: f32 = 0;
	awakeCount: i32 = 0;
}

function dynamicBody(worldId: b2WorldId, x: f32): b2BodyId {
	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(x, 3.0);
	return b2CreateBody(worldId, bodyDef);
}

export function shapes(): ShapesResult {
	let worldId = b2CreateWorld(b2DefaultWorldDef());

	// A chain is a run of one sided segments that share their ends, so nothing catches on the seams. It collides
	// on the right of its direction of travel, so a floor runs right to left. The two ghosts carry the run past
	// its ends.
	let groundId = b2CreateBody(worldId, b2DefaultBodyDef());
	let points = new StaticArray<b2Vec2>(4);
	points[0] = b2Vec2(10.0, 4.0);
	points[1] = b2Vec2(10.0, 0.0);
	points[2] = b2Vec2(-10.0, 0.0);
	points[3] = b2Vec2(-10.0, 4.0);
	let chainDef = b2DefaultChainDef();
	chainDef.points = points;
	chainDef.pointCount = points.length;
	chainDef.ghost1 = b2Vec2(10.0, 5.0);
	chainDef.ghost2 = b2Vec2(-10.0, 5.0);
	b2CreateChain(groundId, chainDef);

	let shapeDef = b2DefaultShapeDef();

	let circleId = dynamicBody(worldId, -4.0);
	b2CreateCircleShape(circleId, shapeDef, b2Circle(b2Vec2(0.0, 0.0), 0.5));

	// A capsule is two centers and a radius; this one lies flat
	let capsuleId = dynamicBody(worldId, -2.0);
	b2CreateCapsuleShape(capsuleId, shapeDef, b2Capsule(b2Vec2(-0.5, 0.0), b2Vec2(0.5, 0.0), 0.25));

	let boxId = dynamicBody(worldId, 0.0);
	b2CreatePolygonShape(boxId, shapeDef, b2MakeBox(0.5, 0.5));

	// A rounded box is a smaller box with a radius around it: 0.4 + 0.1 on each side
	let roundedId = dynamicBody(worldId, 2.0);
	b2CreatePolygonShape(roundedId, shapeDef, b2MakeRoundedBox(0.4, 0.4, 0.1));

	// Any convex polygon of up to 8 points comes from its hull
	let corners = new StaticArray<b2Vec2>(4);
	corners[0] = b2Vec2(-0.6, -0.4);
	corners[1] = b2Vec2(0.6, -0.4);
	corners[2] = b2Vec2(0.3, 0.4);
	corners[3] = b2Vec2(-0.3, 0.4);
	let hullId = dynamicBody(worldId, 4.0);
	b2CreatePolygonShape(hullId, shapeDef, b2MakePolygon(b2ComputeHull(corners, corners.length), 0.0));

	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 240; ++i) {
		b2World_Step(worldId, timeStep, 4);
	}

	let result = new ShapesResult();
	result.circleHeight = b2Body_GetPosition(circleId).y;
	result.capsuleHeight = b2Body_GetPosition(capsuleId).y;
	result.boxHeight = b2Body_GetPosition(boxId).y;
	result.roundedHeight = b2Body_GetPosition(roundedId).y;
	result.hullHeight = b2Body_GetPosition(hullId).y;
	let bodies = new Array<b2BodyId>();
	bodies.push(circleId);
	bodies.push(capsuleId);
	bodies.push(boxId);
	bodies.push(roundedId);
	bodies.push(hullId);
	for (let i = 0; i < bodies.length; ++i) {
		if (b2Body_IsAwake(bodies[i])) {
			result.awakeCount += 1;
		}
	}

	b2DestroyWorld(worldId);
	return result;
}
