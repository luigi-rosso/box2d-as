// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Hello world: a world, a ground box and a falling box, stepped until the box rests on the ground.

import {
	b2Body_GetPosition,
	b2Body_IsAwake,
	b2CreateBody,
	b2CreatePolygonShape,
	b2CreateWorld,
	b2DefaultBodyDef,
	b2DefaultShapeDef,
	b2DefaultWorldDef,
	b2DestroyWorld,
	b2MakeBox,
	b2Vec2,
	b2World_Step,
	b2_dynamicBody,
} from "../lib/box2d";

export class HelloWorldResult {
	height: f32 = 0;
	awake: bool = true;
}

export function helloWorld(): HelloWorldResult {
	let worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2(0.0, -10.0);
	let worldId = b2CreateWorld(worldDef);

	// A static body is the default; its box is 20 m wide with the top face at y = 0
	let groundDef = b2DefaultBodyDef();
	groundDef.position = b2Vec2(0.0, -1.0);
	let groundId = b2CreateBody(worldId, groundDef);
	b2CreatePolygonShape(groundId, b2DefaultShapeDef(), b2MakeBox(10.0, 1.0));

	// A dynamic 1 m box, 4 m up
	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(0.0, 4.0);
	let bodyId = b2CreateBody(worldId, bodyDef);
	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;
	b2CreatePolygonShape(bodyId, shapeDef, b2MakeBox(0.5, 0.5));

	// A constant time step and 4 sub-steps, as Box2D recommends
	let timeStep: f32 = <f32>(1.0 / 60.0);
	let subStepCount = 4;
	for (let i = 0; i < 180; ++i) {
		b2World_Step(worldId, timeStep, subStepCount);
	}

	let result = new HelloWorldResult();
	result.height = b2Body_GetPosition(bodyId).y;
	result.awake = b2Body_IsAwake(bodyId);

	// Destroying the world destroys its bodies, shapes and joints
	b2DestroyWorld(worldId);
	return result;
}
