// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory parity scenes, public Box2D API only. verify/scenes/scenes.as mirrors these line for
// line.

#pragma once

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
	SCENE_SUBSTEP_COUNT = 4,
	SCENE_KEY_CAPACITY = 16,
	SCENE_SPAWN_CAPACITY = 64,
	SCENE_HIT_CAPACITY = 32,
	SCENE_CONTACT_CAPACITY = 8,
	SCENE_MOVER_PLANE_CAPACITY = 8,
	SCENE_BONE_COUNT = 11,
};

// samples/car.h Car, and the bike built the same way
typedef struct SceneVehicle
{
	b2BodyId chassisId;
	b2BodyId rearWheelId;
	b2BodyId frontWheelId;
	b2JointId rearAxleId;
	b2JointId frontAxleId;
} SceneVehicle;

// samples/geometric_mover.h GeometricMover, minus drawing
typedef struct SceneGeometricMover
{
	float jumpSpeed;
	float maxSpeed;
	float minSpeed;
	float stopSpeed;
	float accelerate;
	float airSteer;
	float friction;
	float gravity;
	float pogoRestLength;
	float pogoHertz;
	float pogoDampingRatio;
	float pogoGroundForce;
	float minGroundNormalY;
	bool pushBodies;
	b2Capsule capsule;
	b2QueryFilter filter;
	b2Pos position;
	b2Vec2 velocity;
	float pogoVelocity;
	bool onGround;
	b2CollisionPlane planes[SCENE_MOVER_PLANE_CAPACITY];
	b2Pos planePoints[SCENE_MOVER_PLANE_CAPACITY];
	b2ShapeId planeShapes[SCENE_MOVER_PLANE_CAPACITY];
	int planeCount;
	int totalIterations;
	float pogoFraction;
	bool pogoHit;
} SceneGeometricMover;

// samples/dynamic_mover.h DynamicMoverCastResult
typedef struct SceneCastResult
{
	b2Pos point;
	b2Vec2 normal;
	b2BodyId bodyId;
	float fraction;
	bool hit;
} SceneCastResult;

// samples/dynamic_mover.h DynamicMover, minus drawing
typedef struct SceneDynamicMover
{
	float jumpSpeed;
	float maxSpeed;
	float minSpeed;
	float stopSpeed;
	float accelerate;
	float airSteer;
	float friction;
	float gravityScale;
	float maxGroundForce;
	float maxAirForce;
	float pogoRestLength;
	float pogoHertz;
	float pogoDampingRatio;
	float pogoCompressionScale;
	float pogoTensionScale;
	float minGroundNormalY;
	b2Capsule capsule;
	b2Filter filter;
	b2BodyId groundId;
	b2BodyId moverId;
	b2JointId moverJointId;
	b2JointId pogoJointId;
	SceneCastResult castResult;
	b2Vec2 velocity;
	bool onGround;
	bool walkable;
	bool jumping;
	int jumpTicks;
	float pogoImpulse;
	float pogoVelocity;
	float pogoLength;
	float pogoFraction;
	bool pogoHit;
} SceneDynamicMover;

typedef struct Scene
{
	const char* name;
	int index;
	b2WorldId worldId;

	// Every body the scene created, in creation order. Destroyed bodies stay in the list.
	b2BodyId* bodies;
	int bodyCount;
	int bodyCapacity;

	// Words pushed by Scene_Step for the current step, recorded after the bodies
	uint32_t* probes;
	int probeCount;
	int probeCapacity;

	uint32_t seed;
	float time;
	bool jumpReleased;

	// Handles each scene keeps for its script, slots documented per scene
	b2BodyId groundId;
	b2BodyId keyBodies[SCENE_KEY_CAPACITY];
	b2ShapeId keyShapes[SCENE_KEY_CAPACITY];
	b2JointId keyJoints[SCENE_KEY_CAPACITY];
	int keyJointCount;

	// Bodies created by the script, oldest first
	b2BodyId spawned[SCENE_SPAWN_CAPACITY];
	int spawnCount;
	int spawnHead;

	SceneVehicle car;
	SceneVehicle bike;
	SceneGeometricMover geometricMover;
	SceneDynamicMover dynamicMover;
} Scene;

int Scene_GetCount( void );
const char* Scene_GetName( int index );

// Returns false for an unknown scene name
bool Scene_Create( Scene* scene, const char* name, int workerCount );

// Scripted input for this step, b2World_Step at 60 Hz with 4 sub-steps, then the probes
void Scene_Step( Scene* scene, int stepIndex );

void Scene_Destroy( Scene* scene );
