// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory parity scenes, public Box2D API only. verify/scenes/scenes.as mirrors these line for
// line.

#pragma once

#include "box2d/box2d.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
	SCENE_SUBSTEP_COUNT = 4,
};

typedef struct Scene
{
	const char* name;
	b2WorldId worldId;

	// Every body the scene created, in creation order. Destroyed bodies stay in the list.
	b2BodyId* bodies;
	int bodyCount;
	int bodyCapacity;
} Scene;

int Scene_GetCount( void );
const char* Scene_GetName( int index );

// Returns false for an unknown scene name
bool Scene_Create( Scene* scene, const char* name, int workerCount );

// Scripted input for this step, then b2World_Step at 60 Hz with 4 sub-steps
void Scene_Step( Scene* scene, int stepIndex );

void Scene_Destroy( Scene* scene );
