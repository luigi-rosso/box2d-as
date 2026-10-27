// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript mirror of verify/scenes/scenes.c, statement for statement over lib/box2d.as. Every
// int to float conversion is an explicit <f32>, every float local is annotated.

import {
	b2BodyDef,
	b2BodyId,
	b2CreateBody,
	b2CreatePolygonShape,
	b2CreateWorld,
	b2DefaultBodyDef,
	b2DefaultShapeDef,
	b2DefaultWorldDef,
	b2DestroyWorld,
	b2MakeBox,
	b2MakeSquare,
	b2Vec2,
	b2WorldId,
	b2World_Step,
	b2_dynamicBody,
} from "../../lib/box2d";

export const SCENE_SUBSTEP_COUNT: i32 = 4;

export class Scene {
	name: string = "";
	worldId: b2WorldId = b2WorldId();

	// Every body the scene created, in creation order. Destroyed bodies stay in the list.
	bodies: Array<b2BodyId> = new Array<b2BodyId>();
	bodyCount: i32 = 0;
}

const s_sceneNames: StaticArray<string> = ["pyramid"];

export function Scene_GetCount(): i32 {
	return s_sceneNames.length;
}

export function Scene_GetName(index: i32): string {
	return s_sceneNames[index];
}

function AddBody(scene: Scene, def: b2BodyDef): b2BodyId {
	let bodyId = b2CreateBody(scene.worldId, def);
	scene.bodies.push(bodyId);
	scene.bodyCount += 1;
	return bodyId;
}

// A ground box and a 10 box base pyramid of unit boxes, sleeping enabled, settling for 300 steps
function CreatePyramid(scene: Scene): void {
	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(0.0, -1.0);
		let groundId = AddBody(scene, bodyDef);

		let box = b2MakeBox(20.0, 1.0);
		let shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape(groundId, shapeDef, box);
	}

	let baseCount = 10;

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;

	let a: f32 = 0.5;
	let box = b2MakeSquare(a);

	let shift: f32 = 1.0 * a;

	for (let i = 0; i < baseCount; ++i) {
		let y: f32 = (2.0 * <f32>i + 1.0) * shift;

		for (let j = i; j < baseCount; ++j) {
			let x: f32 = (<f32>i + 1.0) * shift + 2.0 * <f32>(j - i) * shift - a * <f32>baseCount;

			bodyDef.position = b2Vec2(x, y);

			let bodyId = AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);
		}
	}
}

// Returns false for an unknown scene name
export function Scene_Create(scene: Scene, name: string, workerCount: i32): bool {
	let index = -1;
	for (let i = 0; i < Scene_GetCount(); ++i) {
		if (name == s_sceneNames[i]) {
			index = i;
		}
	}
	if (index < 0) {
		return false;
	}

	scene.name = s_sceneNames[index];

	let worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	scene.worldId = b2CreateWorld(worldDef);

	if (index == 0) {
		CreatePyramid(scene);
	}

	return true;
}

// Scripted input for this step, then b2World_Step at 60 Hz with 4 sub-steps
export function Scene_Step(scene: Scene, stepIndex: i32): void {
	let timeStep: f32 = <f32>1.0 / <f32>60.0;
	b2World_Step(scene.worldId, timeStep, SCENE_SUBSTEP_COUNT);
}

export function Scene_Destroy(scene: Scene): void {
	b2DestroyWorld(scene.worldId);
	scene.bodies = new Array<b2BodyId>();
	scene.bodyCount = 0;
}
