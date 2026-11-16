// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// A step's trajectory record, as verify/scenes/traj.c writes it: step index, body count, counters, bodies, probe
// count, probes. traj.as hands it to the host; bench/as hashes it, since a rive-cli run cannot write a file.

import { Scene } from "./scenes";
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

const TRAJ_BODY_WORDS = 11;
export const TRAJ_COUNTER_COUNT = 47;

// The counters from here on depend on the build and the worker count
const TRAJ_STABLE_COUNTER_COUNT = 41;

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

	if (g_count - start != TRAJ_STABLE_COUNTER_COUNT) {
		g_error = 1;
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

// Writes the record of the step scene just took
export function Record_Write(scene: Scene, stepIndex: i32): void {
	g_count = 0;
	PushInt(stepIndex);
	PushInt(scene.bodyCount);
	WriteCounters(scene.worldId);
	for (let i = 0; i < scene.bodyCount; ++i) {
		WriteBody(i, scene.bodies[i]);
	}
	let probes = scene.probes;
	PushInt(probes.length);
	for (let i = 0; i < probes.length; ++i) {
		PushWord(probes[i]);
	}
}

export function Record_Error(): i32 {
	return g_error;
}

export function Record_Length(): i32 {
	return g_count;
}

// Valid until the next frame boundary, which may move the buffer
export unsafe function Record_Pointer(): usize {
	return g_words.dataStart;
}

// FNV-1a over the record's words but the build and worker dependent counters, as verify/threads.mjs hashes a step
// of a native recording
export function Record_Hash(): u32 {
	let h: u32 = 2166136261;
	let skipStart = 2 + TRAJ_STABLE_COUNTER_COUNT;
	let skipEnd = 2 + TRAJ_COUNTER_COUNT;
	for (let i = 0; i < g_count; ++i) {
		if (skipStart <= i && i < skipEnd) {
			continue;
		}
		let bits = g_words[i];
		for (let s: u32 = 0; s < 4; ++s) {
			h = (h ^ ((bits >> (8 * s)) & 0xff)) * 16777619;
		}
	}
	return h;
}
