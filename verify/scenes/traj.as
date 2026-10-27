// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory driver, the twin of verify/scenes/traj.c. create() builds a scene, step() runs one
// Scene_Step and writes that step's record (step index, body count, counters, bodies, probe count,
// probes) into a word buffer the host reads through recordPointer() and recordLength() before the
// next frame boundary. The host writes the header.

import { Scene, Scene_Create, Scene_GetCount, Scene_GetName, Scene_Step } from "./scenes";
import {
	b2BodyId,
	b2Body_GetAngularVelocity,
	b2Body_GetLinearVelocity,
	b2Body_GetPosition,
	b2Body_GetRotation,
	b2Body_GetType,
	b2Body_IsAwake,
	b2Body_IsValid,
	b2WorldId,
	b2World_GetAwakeBodyCount,
	b2World_GetBodyEvents,
	b2World_GetContactEvents,
	b2World_GetCounters,
	b2World_GetJointEvents,
	b2World_GetSensorEvents,
	b2World_GetWorkerCount,
} from "../../lib/box2d";

export { frameBoundary } from "../as/common";

const TRAJ_BODY_WORDS = 11;
const TRAJ_COUNTER_COUNT = 47;

let g_scene = new Scene();
let g_words = new Uint32Array(1024);
let g_count = 0;
let g_error = 0;

function PushWord(value: u32): void {
	if (g_count == g_words.length) {
		let grown = new Uint32Array(2 * g_words.length);
		for (let i = 0; i < g_count; ++i) {
			grown[i] = g_words[i];
		}
		g_words = grown;
	}
	g_words[g_count] = value;
	g_count += 1;
}

function PushFloat(value: f32): void {
	PushWord(reinterpret<u32>(value));
}

function PushInt(value: i32): void {
	PushWord(<u32>value);
}

function WriteCounters(worldId: b2WorldId): void {
	let c = b2World_GetCounters(worldId);
	let contactEvents = b2World_GetContactEvents(worldId);
	let sensorEvents = b2World_GetSensorEvents(worldId);
	let bodyEvents = b2World_GetBodyEvents(worldId);
	let jointEvents = b2World_GetJointEvents(worldId);
	let start = g_count;

	PushInt(c.bodyCount);
	PushInt(c.shapeCount);
	PushInt(c.contactCount);
	PushInt(c.jointCount);
	PushInt(c.islandCount);
	PushInt(b2World_GetAwakeBodyCount(worldId));
	PushInt(c.awakeContactCount);
	PushInt(c.recycledContactCount);
	PushInt(c.staticTreeHeight);
	PushInt(c.treeHeight);
	PushInt(contactEvents.beginCount);
	PushInt(contactEvents.endCount);
	PushInt(contactEvents.hitCount);
	PushInt(sensorEvents.beginCount);
	PushInt(sensorEvents.endCount);
	PushInt(bodyEvents.moveCount);
	PushInt(jointEvents.count);
	for (let i = 0; i < 24; ++i) {
		PushInt(c.colorCounts[i]);
	}

	// Build and worker dependent
	PushInt(c.taskCount);
	PushInt(c.stackUsed);
	PushWord(<u32>(<u64>c.byteCount & 0xffffffff));
	PushWord(<u32>(<u64>c.byteCount >> 32));
	PushInt(b2World_GetWorkerCount(worldId));
	PushInt(0);

	if (g_count - start != TRAJ_COUNTER_COUNT) {
		g_error = 1;
	}
}

function WriteBody(index: i32, bodyId: b2BodyId): void {
	PushInt(index);

	if (b2Body_IsValid(bodyId) == false) {
		for (let i = 1; i < TRAJ_BODY_WORDS; ++i) {
			PushWord(0);
		}
		return;
	}

	let flags: u32 = 1;
	if (b2Body_IsAwake(bodyId)) {
		flags |= 2;
	}
	flags |= <u32>b2Body_GetType(bodyId) << 4;
	PushWord(flags);
	PushInt(bodyId.index1);
	PushWord(<u32>bodyId.generation);

	let p = b2Body_GetPosition(bodyId);
	let q = b2Body_GetRotation(bodyId);
	let v = b2Body_GetLinearVelocity(bodyId);
	let omega: f32 = b2Body_GetAngularVelocity(bodyId);

	PushFloat(p.x);
	PushFloat(p.y);
	PushFloat(q.c);
	PushFloat(q.s);
	PushFloat(v.x);
	PushFloat(v.y);
	PushFloat(omega);
}

export function sceneCount(): i32 {
	return Scene_GetCount();
}

// Returns false for an unknown scene index
export function create(sceneIndex: i32, workerCount: i32): bool {
	if (sceneIndex < 0 || sceneIndex >= Scene_GetCount()) {
		return false;
	}
	g_scene = new Scene();
	return Scene_Create(g_scene, Scene_GetName(sceneIndex), workerCount);
}

export function counterCount(): i32 {
	return TRAJ_COUNTER_COUNT;
}

export function bodyCount(): i32 {
	return g_scene.bodyCount;
}

export function step(stepIndex: i32): void {
	Scene_Step(g_scene, stepIndex);

	g_count = 0;
	PushInt(stepIndex);
	PushInt(g_scene.bodyCount);
	WriteCounters(g_scene.worldId);
	for (let i = 0; i < g_scene.bodyCount; ++i) {
		WriteBody(i, g_scene.bodies[i]);
	}
	let probes = g_scene.probes;
	PushInt(probes.length);
	for (let i = 0; i < probes.length; ++i) {
		PushWord(probes[i]);
	}
}

export function recordError(): i32 {
	return g_error;
}

export function recordLength(): i32 {
	return g_count;
}

// Valid until the next frame boundary, which may move the buffer
export unsafe function recordPointer(): usize {
	return g_words.dataStart;
}
