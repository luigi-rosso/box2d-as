// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Threading: the same pyramid stepped on the main lane alone and then on parallel calls over the world's own
// memory. Results do not depend on the thread count, so the two runs end bit for bit the same. notes/threading.md
// says what runs on parallel calls and what it costs.

import {
	b2BodyId,
	b2Body_GetPosition,
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
import { b2World_EnableParallel, b2_parallelAll, b2_parallelStats } from "../lib/parallel_step";

const BASE_COUNT: i32 = 20;
const STEP_COUNT: i32 = 120;

export class PyramidRun {
	// Every box's final position
	positions: StaticArray<f32> = new StaticArray<f32>(0);
	// Steps whose solver stages and narrow phase ran on parallel calls
	solves: i32 = 0;
	collides: i32 = 0;
}

// Steps a pyramid of boxes, on parallel calls when asked
export function stepPyramid(parallel: bool): PyramidRun {
	// Awake for the whole run, so every step has work for the parts
	let worldDef = b2DefaultWorldDef();
	worldDef.enableSleep = false;
	let worldId = b2CreateWorld(worldDef);

	let groundDef = b2DefaultBodyDef();
	groundDef.position = b2Vec2(0.0, -1.0);
	let groundId = b2CreateBody(worldId, groundDef);
	b2CreatePolygonShape(groundId, b2DefaultShapeDef(), b2MakeBox(40.0, 1.0));

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	let box = b2MakeBox(0.5, 0.5);
	let bodies = new Array<b2BodyId>();
	for (let row = 0; row < BASE_COUNT; ++row) {
		for (let i = row; i < BASE_COUNT; ++i) {
			bodyDef.position = b2Vec2(<f32>(2 * i - row) * 0.5 - <f32>BASE_COUNT * 0.5, <f32>row + 0.5);
			let bodyId = b2CreateBody(worldId, bodyDef);
			b2CreatePolygonShape(bodyId, b2DefaultShapeDef(), box);
			bodies.push(bodyId);
		}
	}

	// Parallel calls run as many parts as Parallel.size(), one where the runtime has no threads
	if (parallel) {
		b2World_EnableParallel(worldId, b2_parallelAll);
	}
	let solves = b2_parallelStats.solves;
	let collides = b2_parallelStats.collides;

	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < STEP_COUNT; ++i) {
		b2World_Step(worldId, timeStep, 4);
	}

	let run = new PyramidRun();
	run.solves = b2_parallelStats.solves - solves;
	run.collides = b2_parallelStats.collides - collides;

	let positions = new StaticArray<f32>(2 * bodies.length);
	for (let i = 0; i < bodies.length; ++i) {
		let p = b2Body_GetPosition(bodies[i]);
		positions[2 * i] = p.x;
		positions[2 * i + 1] = p.y;
	}
	run.positions = positions;
	b2DestroyWorld(worldId);
	return run;
}

export class ThreadingResult {
	// Coordinates whose bits differ from the serial run's
	differ: i32 = 0;
	solves: i32 = 0;
	collides: i32 = 0;
}

// Runs the pyramid serially and then on parallel calls
export function threading(): ThreadingResult {
	let serial = stepPyramid(false).positions;
	let threaded = stepPyramid(true);
	let result = new ThreadingResult();
	for (let i = 0; i < serial.length; ++i) {
		if (reinterpret<u32>(serial[i]) != reinterpret<u32>(threaded.positions[i])) {
			result.differ += 1;
		}
	}
	result.solves = threaded.solves;
	result.collides = threaded.collides;
	return result;
}
