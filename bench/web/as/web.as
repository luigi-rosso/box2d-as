// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The port on wasm, driven from JS: the same exports as bench/web/c/driver.c.

import { Scene, Scene_Create, Scene_Destroy, Scene_GetCount, Scene_GetName, Scene_Step } from "../../../verify/scenes/scenes";
import {
	b2Body_GetAngularVelocity,
	b2Body_GetLinearVelocity,
	b2Body_GetPosition,
	b2Body_GetRotation,
	b2Body_IsValid,
	b2World_GetProfile,
} from "../../../lib/box2d";

export { frameBoundary } from "../../../verify/as/common";

let g_scene = new Scene();
let g_live = false;

export function sceneCount(): i32 {
	return Scene_GetCount();
}

export function create(sceneIndex: i32, workerCount: i32): i32 {
	if (g_live) {
		Scene_Destroy(g_scene);
		g_live = false;
	}
	if (sceneIndex < 0 || sceneIndex >= Scene_GetCount()) {
		return 0;
	}
	g_scene = new Scene();
	g_live = Scene_Create(g_scene, Scene_GetName(sceneIndex), workerCount);
	return g_live ? 1 : 0;
}

export function step(stepIndex: i32): void {
	Scene_Step(g_scene, stepIndex);
}

function hashWord(h: u32, bits: u32): u32 {
	for (let s = 0; s < 4; ++s) {
		h = (h ^ ((bits >> (8 * <u32>s)) & 0xff)) * 16777619;
	}
	return h;
}

// FNV-1a over each body's pose and velocities in the order of a trajectory record; a destroyed body
// hashes the recording's zero words
export function stateHash(): u32 {
	let h: u32 = 2166136261;
	for (let i = 0; i < g_scene.bodyCount; ++i) {
		let id = g_scene.bodies[i];
		if (!b2Body_IsValid(id)) {
			for (let k = 0; k < 7; ++k) {
				h = hashWord(h, 0);
			}
			continue;
		}
		let p = b2Body_GetPosition(id);
		let q = b2Body_GetRotation(id);
		let v = b2Body_GetLinearVelocity(id);
		let w: f32 = b2Body_GetAngularVelocity(id);
		h = hashWord(h, reinterpret<u32>(p.x));
		h = hashWord(h, reinterpret<u32>(p.y));
		h = hashWord(h, reinterpret<u32>(q.c));
		h = hashWord(h, reinterpret<u32>(q.s));
		h = hashWord(h, reinterpret<u32>(v.x));
		h = hashWord(h, reinterpret<u32>(v.y));
		h = hashWord(h, reinterpret<u32>(w));
	}
	return h;
}

// The b2Profile fields in declaration order
export function profileField(k: i32): f32 {
	let p = b2World_GetProfile(g_scene.worldId);
	let fields: StaticArray<f32> = [
		p.step, p.pairs, p.collide, p.solve, p.solverSetup, p.constraints, p.prepareConstraints,
		p.integrateVelocities, p.warmStart, p.solveImpulses, p.integratePositions, p.relaxImpulses,
		p.restitution, p.storeImpulses, p.splitIslands, p.transforms, p.sensorHits, p.jointEvents,
		p.hitEvents, p.refit, p.bullets, p.sleepIslands, p.sensors,
	];
	return 0 <= k && k < fields.length ? fields[k] : 0;
}

export function bodyCount(): i32 {
	return g_scene.bodyCount;
}
