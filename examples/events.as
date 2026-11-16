// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Events: a box falls through a sensor onto the ground. The sensor reports it entering and leaving, and the landing
// reports a contact begin and a hit. Events are read after each step and last until the next one.

import {
	b2CreateBody,
	b2CreatePolygonShape,
	b2CreateWorld,
	b2DefaultBodyDef,
	b2DefaultShapeDef,
	b2DefaultWorldDef,
	b2DestroyWorld,
	b2MakeBox,
	b2Vec2,
	b2World_GetContactEvents,
	b2World_GetSensorEvents,
	b2World_Step,
	b2_dynamicBody,
} from "../lib/box2d";

export class EventsResult {
	sensorBegins: i32 = 0;
	sensorEnds: i32 = 0;
	contactBegins: i32 = 0;
	hits: i32 = 0;
	fastestHit: f32 = 0;
}

export function events(): EventsResult {
	let worldId = b2CreateWorld(b2DefaultWorldDef());

	// Contact and hit events are off by default; either shape of a pair can ask for them
	let groundDef = b2DefaultBodyDef();
	groundDef.position = b2Vec2(0.0, -1.0);
	let groundId = b2CreateBody(worldId, groundDef);
	let groundShapeDef = b2DefaultShapeDef();
	groundShapeDef.enableContactEvents = true;
	groundShapeDef.enableHitEvents = true;
	b2CreatePolygonShape(groundId, groundShapeDef, b2MakeBox(10.0, 1.0));

	// A sensor 2 m tall between y = 2 and y = 4. It detects shapes that enable sensor events, and it needs them on
	// itself to report.
	let sensorDef = b2DefaultBodyDef();
	sensorDef.position = b2Vec2(0.0, 3.0);
	let sensorId = b2CreateBody(worldId, sensorDef);
	let sensorShapeDef = b2DefaultShapeDef();
	sensorShapeDef.isSensor = true;
	sensorShapeDef.enableSensorEvents = true;
	b2CreatePolygonShape(sensorId, sensorShapeDef, b2MakeBox(2.0, 1.0));

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(0.0, 6.0);
	let boxId = b2CreateBody(worldId, bodyDef);
	let shapeDef = b2DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b2CreatePolygonShape(boxId, shapeDef, b2MakeBox(0.5, 0.5));

	let result = new EventsResult();
	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 120; ++i) {
		b2World_Step(worldId, timeStep, 4);

		let sensorEvents = b2World_GetSensorEvents(worldId);
		result.sensorBegins += sensorEvents.beginCount;
		result.sensorEnds += sensorEvents.endCount;

		let contactEvents = b2World_GetContactEvents(worldId);
		result.contactBegins += contactEvents.beginCount;
		for (let j = 0; j < contactEvents.hitCount; ++j) {
			// A hit is a contact that began faster than the world's hitEventThreshold, 1 m/s by default
			let hit = contactEvents.hitEvents[j];
			result.hits += 1;
			result.fastestHit = Mathf.max(result.fastestHit, hit.approachSpeed);
		}
	}

	b2DestroyWorld(worldId);
	return result;
}
