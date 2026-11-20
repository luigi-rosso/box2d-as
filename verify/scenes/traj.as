// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory driver, the twin of verify/scenes/traj.c. create() builds a scene, step() runs one
// Scene_Step and writes that step's record (record.as) into a word buffer the host reads through
// recordPointer() and recordLength() before the next frame boundary. The host writes the header.

import { Scene, Scene_Create, Scene_GetCount, Scene_GetName, Scene_Step } from "./scenes";
import { Record_Error, Record_Length, Record_Pointer, Record_Write, TRAJ_COUNTER_COUNT } from "./record";

export { frameBoundary } from "../as/common";

let g_scene = new Scene();

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
	Record_Write(g_scene, stepIndex);
}

export function recordError(): i32 {
	return Record_Error();
}

export function recordLength(): i32 {
	return Record_Length();
}

// Valid until the next frame boundary, which may move the buffer
export function recordPointer(): usize {
	return Record_Pointer();
}
