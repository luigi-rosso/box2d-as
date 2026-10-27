// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory parity scenes. Plain C over the public Box2D API so verify/scenes/scenes.as can mirror
// each builder statement for statement. Add a scene here and in scenes.as, then list it in
// s_sceneNames, in verify/traj.mjs and in verify/traj.sh. C leaves the evaluation order of function
// arguments and initializers open, so every random draw is its own statement.

#include "scenes.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>

enum
{
	SCENE_PYRAMID,
	SCENE_SHAPE_RAIN,
	SCENE_CHAIN_CAR,
	SCENE_JOINTS,
	SCENE_DYNAMIC_MOVER,
	SCENE_CHURN,
	SCENE_BULLETS,
	SCENE_SENSORS,
	SCENE_EVENTS,
	SCENE_QUERIES,
	SCENE_MOVER,
	SCENE_MANY_BODIES,
};

static const char* s_sceneNames[] = {
	"pyramid", "shape_rain", "chain_car", "joints",	 "dynamic_mover", "churn",
	"bullets", "sensors",	 "events",	  "queries", "mover",		  "many_bodies",
};

// samples/sample_character.cpp CollisionBits; AllBits is ~0u, which zero extends
static const uint64_t s_staticBit = 0x0001;
static const uint64_t s_moverBit = 0x0002;
static const uint64_t s_dynamicBit = 0x0004;
static const uint64_t s_allBits = 0xFFFFFFFFull;

int Scene_GetCount( void )
{
	return (int)( sizeof( s_sceneNames ) / sizeof( s_sceneNames[0] ) );
}

const char* Scene_GetName( int index )
{
	return s_sceneNames[index];
}

// Numerical Recipes LCG
static uint32_t Scene_Random( Scene* scene )
{
	scene->seed = scene->seed * 1664525u + 1013904223u;
	return scene->seed;
}

// Uniform in [lo, hi] from the top 24 bits
static float Scene_RandomFloat( Scene* scene, float lo, float hi )
{
	uint32_t bits = Scene_Random( scene ) >> 8;
	float r = (float)bits * ( 1.0f / 16777216.0f );
	float range = hi - lo;
	float scaled = range * r;
	return lo + scaled;
}

static b2BodyId Scene_AddBody( Scene* scene, const b2BodyDef* def )
{
	b2BodyId bodyId = b2CreateBody( scene->worldId, def );
	if ( scene->bodyCount == scene->bodyCapacity )
	{
		scene->bodyCapacity = scene->bodyCapacity < 64 ? 64 : 2 * scene->bodyCapacity;
		scene->bodies = realloc( scene->bodies, scene->bodyCapacity * sizeof( b2BodyId ) );
	}
	scene->bodies[scene->bodyCount] = bodyId;
	scene->bodyCount += 1;
	return bodyId;
}

static void Scene_AddSpawn( Scene* scene, b2BodyId bodyId )
{
	if ( scene->spawnCount < SCENE_SPAWN_CAPACITY )
	{
		scene->spawned[scene->spawnCount] = bodyId;
		scene->spawnCount += 1;
	}
}

static void Probe_Word( Scene* scene, uint32_t value )
{
	if ( scene->probeCount == scene->probeCapacity )
	{
		scene->probeCapacity = scene->probeCapacity < 256 ? 256 : 2 * scene->probeCapacity;
		scene->probes = realloc( scene->probes, scene->probeCapacity * sizeof( uint32_t ) );
	}
	scene->probes[scene->probeCount] = value;
	scene->probeCount += 1;
}

static void Probe_Int( Scene* scene, int value )
{
	Probe_Word( scene, (uint32_t)value );
}

static void Probe_Bool( Scene* scene, bool value )
{
	Probe_Word( scene, value ? 1u : 0u );
}

static void Probe_Float( Scene* scene, float value )
{
	uint32_t bits;
	memcpy( &bits, &value, sizeof( bits ) );
	Probe_Word( scene, bits );
}

static void Probe_Vec2( Scene* scene, b2Vec2 v )
{
	Probe_Float( scene, v.x );
	Probe_Float( scene, v.y );
}

static void Probe_Rot( Scene* scene, b2Rot q )
{
	Probe_Float( scene, q.c );
	Probe_Float( scene, q.s );
}

static void Probe_ShapeId( Scene* scene, b2ShapeId id )
{
	Probe_Int( scene, id.index1 );
	Probe_Word( scene, id.generation );
}

static void Probe_BodyId( Scene* scene, b2BodyId id )
{
	Probe_Int( scene, id.index1 );
	Probe_Word( scene, id.generation );
}

static void Probe_JointId( Scene* scene, b2JointId id )
{
	Probe_Int( scene, id.index1 );
	Probe_Word( scene, id.generation );
}

static void Probe_ContactId( Scene* scene, b2ContactId id )
{
	Probe_Int( scene, id.index1 );
	Probe_Word( scene, id.generation );
}

// Point count, normal, rolling impulse, then per point anchors, separation and impulses.
// normalVelocity is left out: the solver only computes it for hit events.
static void Probe_Manifold( Scene* scene, b2Manifold manifold )
{
	Probe_Int( scene, manifold.pointCount );
	Probe_Vec2( scene, manifold.normal );
	Probe_Float( scene, manifold.rollingImpulse );
	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		b2ManifoldPoint mp = manifold.points[i];
		Probe_Vec2( scene, mp.anchorA );
		Probe_Vec2( scene, mp.anchorB );
		Probe_Float( scene, mp.separation );
		Probe_Float( scene, mp.normalImpulse );
		Probe_Float( scene, mp.tangentImpulse );
		Probe_Float( scene, mp.totalNormalImpulse );
		Probe_Int( scene, mp.id );
		Probe_Bool( scene, mp.persisted );
	}
}

static void Probe_ContactData( Scene* scene, b2ContactData data )
{
	Probe_ContactId( scene, data.contactId );
	Probe_ShapeId( scene, data.shapeIdA );
	Probe_ShapeId( scene, data.shapeIdB );
	Probe_Manifold( scene, data.manifold );
}

// Begin, end and hit events in array order, each block led by its count
static void Probe_ContactEvents( Scene* scene )
{
	b2ContactEvents events = b2World_GetContactEvents( scene->worldId );

	Probe_Int( scene, events.beginCount );
	for ( int i = 0; i < events.beginCount; ++i )
	{
		b2ContactBeginTouchEvent event = events.beginEvents[i];
		Probe_ShapeId( scene, event.shapeIdA );
		Probe_ShapeId( scene, event.shapeIdB );
		Probe_ContactId( scene, event.contactId );
	}

	Probe_Int( scene, events.endCount );
	for ( int i = 0; i < events.endCount; ++i )
	{
		b2ContactEndTouchEvent event = events.endEvents[i];
		Probe_ShapeId( scene, event.shapeIdA );
		Probe_ShapeId( scene, event.shapeIdB );
		Probe_ContactId( scene, event.contactId );
	}

	Probe_Int( scene, events.hitCount );
	for ( int i = 0; i < events.hitCount; ++i )
	{
		b2ContactHitEvent event = events.hitEvents[i];
		Probe_ShapeId( scene, event.shapeIdA );
		Probe_ShapeId( scene, event.shapeIdB );
		Probe_ContactId( scene, event.contactId );
		Probe_Vec2( scene, event.point );
		Probe_Vec2( scene, event.normal );
		Probe_Float( scene, event.approachSpeed );
	}
}

static void Probe_SensorEvents( Scene* scene )
{
	b2SensorEvents events = b2World_GetSensorEvents( scene->worldId );

	Probe_Int( scene, events.beginCount );
	for ( int i = 0; i < events.beginCount; ++i )
	{
		b2SensorBeginTouchEvent event = events.beginEvents[i];
		Probe_ShapeId( scene, event.sensorShapeId );
		Probe_ShapeId( scene, event.visitorShapeId );
	}

	Probe_Int( scene, events.endCount );
	for ( int i = 0; i < events.endCount; ++i )
	{
		b2SensorEndTouchEvent event = events.endEvents[i];
		Probe_ShapeId( scene, event.sensorShapeId );
		Probe_ShapeId( scene, event.visitorShapeId );
	}
}

static void Probe_BodyMoveEvents( Scene* scene )
{
	b2BodyEvents events = b2World_GetBodyEvents( scene->worldId );

	Probe_Int( scene, events.moveCount );
	for ( int i = 0; i < events.moveCount; ++i )
	{
		b2BodyMoveEvent event = events.moveEvents[i];
		Probe_BodyId( scene, event.bodyId );
		Probe_Vec2( scene, event.transform.p );
		Probe_Rot( scene, event.transform.q );
		Probe_Bool( scene, event.fellAsleep );
	}
}

static void Probe_JointEvents( Scene* scene )
{
	b2JointEvents events = b2World_GetJointEvents( scene->worldId );

	Probe_Int( scene, events.count );
	for ( int i = 0; i < events.count; ++i )
	{
		b2JointEvent event = events.jointEvents[i];
		Probe_JointId( scene, event.jointId );
		Probe_Word( scene, (uint32_t)(uintptr_t)event.userData );
	}
}

// Force, torque and separations of any joint, a valid flag first
static void Probe_Joint( Scene* scene, b2JointId jointId )
{
	bool valid = b2Joint_IsValid( jointId );
	Probe_Bool( scene, valid );
	if ( valid == false )
	{
		return;
	}

	Probe_Vec2( scene, b2Joint_GetConstraintForce( jointId ) );
	Probe_Float( scene, b2Joint_GetConstraintTorque( jointId ) );
	Probe_Float( scene, b2Joint_GetLinearSeparation( jointId ) );
	Probe_Float( scene, b2Joint_GetAngularSeparation( jointId ) );
}

static void Scene_WorldStep( Scene* scene )
{
	float timeStep = 1.0f / 60.0f;
	b2World_Step( scene->worldId, timeStep, SCENE_SUBSTEP_COUNT );
}

// A ground box and a 10 box base pyramid of unit boxes, sleeping enabled
static void CreatePyramid( Scene* scene )
{
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 0.0f, -1.0f };
		b2BodyId groundId = Scene_AddBody( scene, &bodyDef );

		b2Polygon box = b2MakeBox( 20.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	int baseCount = 10;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;

	float a = 0.5f;
	b2Polygon box = b2MakeSquare( a );

	float shift = 1.0f * a;

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * (float)i + 1.0f ) * shift;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( (float)i + 1.0f ) * shift + 2.0f * (float)( j - i ) * shift - a * (float)baseCount;

			bodyDef.position = (b2Pos){ x, y };

			b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}
}

// shape_rain: a walled bin of segments with two ramps, a spinning kinematic paddle and a sliding
// kinematic platform, and a rain of every convex shape kind with random materials.
// keyBodies: 0 paddle, 1 platform
static void CreateShapeRain( Scene* scene )
{
	scene->seed = 7919;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = Scene_AddBody( scene, &bodyDef );
	scene->groundId = groundId;

	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2Segment segment = { { -30.0f, 0.0f }, { 30.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &shapeDef, &segment );

	segment = (b2Segment){ { -30.0f, 0.0f }, { -30.0f, 30.0f } };
	b2CreateSegmentShape( groundId, &shapeDef, &segment );

	segment = (b2Segment){ { 30.0f, 0.0f }, { 30.0f, 30.0f } };
	b2CreateSegmentShape( groundId, &shapeDef, &segment );

	shapeDef.material.friction = 0.3f;
	segment = (b2Segment){ { -24.0f, 14.0f }, { -6.0f, 9.0f } };
	b2CreateSegmentShape( groundId, &shapeDef, &segment );

	segment = (b2Segment){ { 6.0f, 12.0f }, { 24.0f, 16.0f } };
	b2CreateSegmentShape( groundId, &shapeDef, &segment );

	{
		b2BodyDef paddleDef = b2DefaultBodyDef();
		paddleDef.type = b2_kinematicBody;
		paddleDef.position = (b2Pos){ 0.0f, 5.0f };
		paddleDef.angularVelocity = 1.5f;
		b2BodyId paddleId = Scene_AddBody( scene, &paddleDef );
		scene->keyBodies[0] = paddleId;

		b2ShapeDef paddleShapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 3.0f, 0.2f );
		b2CreatePolygonShape( paddleId, &paddleShapeDef, &box );
	}

	{
		b2BodyDef platformDef = b2DefaultBodyDef();
		platformDef.type = b2_kinematicBody;
		platformDef.position = (b2Pos){ -18.0f, 3.0f };
		platformDef.linearVelocity = (b2Vec2){ 2.0f, 0.0f };
		b2BodyId platformId = Scene_AddBody( scene, &platformDef );
		scene->keyBodies[1] = platformId;

		b2ShapeDef platformShapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeRoundedBox( 2.0f, 0.25f, 0.1f );
		b2CreatePolygonShape( platformId, &platformShapeDef, &box );
	}
}

// One falling body, its kind cycling through circle, capsule, box, hull, rounded hull and a two
// polygon compound
static void Scene_SpawnRainBody( Scene* scene )
{
	int kind = scene->spawnCount % 6;

	float x = Scene_RandomFloat( scene, -25.0f, 25.0f );
	float angle = Scene_RandomFloat( scene, -B2_PI, B2_PI );
	float friction = Scene_RandomFloat( scene, 0.1f, 0.9f );
	float restitution = Scene_RandomFloat( scene, 0.0f, 0.5f );
	float density = Scene_RandomFloat( scene, 0.5f, 2.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ x, 26.0f };
	bodyDef.rotation = b2MakeRot( angle );
	b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
	Scene_AddSpawn( scene, bodyId );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = friction;
	shapeDef.material.restitution = restitution;
	shapeDef.density = density;

	if ( kind == 0 )
	{
		float radius = Scene_RandomFloat( scene, 0.2f, 0.5f );
		shapeDef.material.rollingResistance = 0.1f;
		b2Circle circle = { { 0.0f, 0.0f }, radius };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}
	else if ( kind == 1 )
	{
		float halfLength = Scene_RandomFloat( scene, 0.2f, 0.6f );
		shapeDef.material.rollingResistance = 0.05f;
		b2Capsule capsule = { { -halfLength, 0.0f }, { halfLength, 0.0f }, 0.25f };
		b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}
	else if ( kind == 2 )
	{
		float hx = Scene_RandomFloat( scene, 0.2f, 0.6f );
		float hy = Scene_RandomFloat( scene, 0.2f, 0.6f );
		b2Polygon box = b2MakeBox( hx, hy );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}
	else if ( kind == 3 || kind == 4 )
	{
		b2Vec2 points[8];
		int count = kind == 3 ? 6 : 5;
		for ( int i = 0; i < count; ++i )
		{
			float px = Scene_RandomFloat( scene, -0.6f, 0.6f );
			float py = Scene_RandomFloat( scene, -0.6f, 0.6f );
			points[i] = (b2Vec2){ px, py };
		}

		float radius = kind == 3 ? 0.0f : 0.1f;
		b2Hull hull = b2ComputeHull( points, count );
		b2Polygon polygon;
		if ( hull.count == 0 )
		{
			polygon = b2MakeRoundedBox( 0.4f, 0.4f, radius );
		}
		else
		{
			polygon = b2MakePolygon( &hull, radius );
		}
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
	}
	else
	{
		b2Polygon box1 = b2MakeOffsetBox( 0.5f, 0.15f, (b2Vec2){ 0.2f, 0.1f }, b2MakeRot( 0.3f ) );
		b2CreatePolygonShape( bodyId, &shapeDef, &box1 );

		b2Polygon box2 = b2MakeOffsetRoundedBox( 0.15f, 0.45f, (b2Vec2){ -0.35f, 0.0f }, b2Rot_identity, 0.05f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box2 );
	}
}

static void StepShapeRain( Scene* scene, int stepIndex )
{
	if ( stepIndex < 300 && stepIndex % 5 == 0 )
	{
		Scene_SpawnRainBody( scene );
		Scene_SpawnRainBody( scene );
	}

	if ( stepIndex % 120 == 60 && stepIndex < 420 )
	{
		b2Vec2 v = b2Body_GetLinearVelocity( scene->keyBodies[1] );
		b2Body_SetLinearVelocity( scene->keyBodies[1], b2Neg( v ) );
	}

	if ( stepIndex == 420 )
	{
		b2Body_SetAngularVelocity( scene->keyBodies[0], 0.0f );
		b2Body_SetLinearVelocity( scene->keyBodies[1], b2Vec2_zero );
	}

	Scene_WorldStep( scene );
}

// chain_car terrain: flat up to x = 0, then two sines with an amplitude that grows with x
static float Scene_TerrainHeight( float x )
{
	if ( x <= 0.0f )
	{
		return 0.0f;
	}

	float amplitude = b2MinFloat( 0.04f * x, 2.0f );
	b2CosSin cs1 = b2ComputeCosSin( 0.08f * x );
	b2CosSin cs2 = b2ComputeCosSin( 0.27f * x );
	float wave = 1.5f * cs1.sine + 0.6f * cs2.sine;
	return amplitude * wave;
}

// samples/car.cpp Car::Spawn
static void Scene_SpawnCar( Scene* scene, SceneVehicle* car, b2Pos position, float scale, float hertz,
							float dampingRatio, float torque )
{
	b2WorldId worldId = scene->worldId;

	b2Vec2 vertices[6] = {
		{ -1.5f, -0.5f }, { 1.5f, -0.5f }, { 1.5f, 0.0f }, { 0.0f, 0.9f }, { -1.15f, 0.9f }, { -1.5f, 0.2f },
	};

	for ( int i = 0; i < 6; ++i )
	{
		vertices[i].x *= 0.85f * scale;
		vertices[i].y *= 0.85f * scale;
	}

	b2Hull hull = b2ComputeHull( vertices, 6 );
	b2Polygon chassis = b2MakePolygon( &hull, 0.15f * scale );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f / scale;
	shapeDef.material.friction = 0.2f;
	shapeDef.filter.groupIndex = -1;

	b2Circle circle = { { 0.0f, 0.0f }, 0.4f * scale };

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2OffsetPos( position, (b2Vec2){ 0.0f, 1.0f * scale } );
	car->chassisId = Scene_AddBody( scene, &bodyDef );
	b2CreatePolygonShape( car->chassisId, &shapeDef, &chassis );

	shapeDef.density = 2.0f / scale;
	shapeDef.material.friction = 1.5f;
	shapeDef.material.rollingResistance = 0.1f;

	bodyDef.position = b2OffsetPos( position, (b2Vec2){ -1.0f * scale, 0.35f * scale } );
	bodyDef.allowFastRotation = true;
	car->rearWheelId = Scene_AddBody( scene, &bodyDef );
	b2CreateCircleShape( car->rearWheelId, &shapeDef, &circle );

	bodyDef.position = b2OffsetPos( position, (b2Vec2){ 1.0f * scale, 0.4f * scale } );
	bodyDef.allowFastRotation = true;
	car->frontWheelId = Scene_AddBody( scene, &bodyDef );
	b2CreateCircleShape( car->frontWheelId, &shapeDef, &circle );

	b2Pos pivot = b2Body_GetPosition( car->rearWheelId );

	b2WheelJointDef jointDef = b2DefaultWheelJointDef();

	jointDef.base.bodyIdA = car->chassisId;
	jointDef.base.bodyIdB = car->rearWheelId;
	jointDef.base.localFrameA.q = b2MakeRot( 0.5f * B2_PI );
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25f * scale;
	jointDef.upperTranslation = 0.25f * scale;
	jointDef.enableLimit = true;
	car->rearAxleId = b2CreateWheelJoint( worldId, &jointDef );

	pivot = b2Body_GetPosition( car->frontWheelId );
	jointDef.base.bodyIdA = car->chassisId;
	jointDef.base.bodyIdB = car->frontWheelId;
	jointDef.base.localFrameA.q = b2MakeRot( 0.5f * B2_PI );
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25f * scale;
	jointDef.upperTranslation = 0.25f * scale;
	jointDef.enableLimit = true;
	car->frontAxleId = b2CreateWheelJoint( worldId, &jointDef );
}

// samples/car.cpp Car::SetSpeed
static void Scene_SetCarSpeed( SceneVehicle* car, float speed )
{
	b2WheelJoint_SetTargetMotorSpeed( car->rearAxleId, speed );
	b2WheelJoint_SetTargetMotorSpeed( car->frontAxleId, speed );
	b2Joint_WakeBodies( car->rearAxleId );
}

// A bike on the Car pattern: one frame with a rider block, two wheels on wheel joints, the rear
// one driven
static void Scene_SpawnBike( Scene* scene, SceneVehicle* bike, b2Pos position )
{
	b2WorldId worldId = scene->worldId;

	b2Vec2 vertices[5] = {
		{ -0.9f, -0.15f }, { 0.9f, -0.15f }, { 0.75f, 0.25f }, { -0.2f, 0.45f }, { -0.8f, 0.2f },
	};

	b2Hull hull = b2ComputeHull( vertices, 5 );
	b2Polygon frame = b2MakePolygon( &hull, 0.05f );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = 0.3f;
	shapeDef.filter.groupIndex = -1;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2OffsetPos( position, (b2Vec2){ 0.0f, 0.75f } );
	bike->chassisId = Scene_AddBody( scene, &bodyDef );
	b2CreatePolygonShape( bike->chassisId, &shapeDef, &frame );

	b2Polygon rider = b2MakeOffsetBox( 0.15f, 0.35f, (b2Vec2){ -0.15f, 0.7f }, b2MakeRot( -0.25f ) );
	shapeDef.density = 0.8f;
	b2CreatePolygonShape( bike->chassisId, &shapeDef, &rider );

	shapeDef.density = 1.5f;
	shapeDef.material.friction = 1.2f;
	shapeDef.material.rollingResistance = 0.05f;

	b2Circle circle = { { 0.0f, 0.0f }, 0.35f };

	bodyDef.position = b2OffsetPos( position, (b2Vec2){ -0.85f, 0.35f } );
	bodyDef.allowFastRotation = true;
	bike->rearWheelId = Scene_AddBody( scene, &bodyDef );
	b2CreateCircleShape( bike->rearWheelId, &shapeDef, &circle );

	bodyDef.position = b2OffsetPos( position, (b2Vec2){ 0.85f, 0.35f } );
	bike->frontWheelId = Scene_AddBody( scene, &bodyDef );
	b2CreateCircleShape( bike->frontWheelId, &shapeDef, &circle );

	b2Pos pivot = b2Body_GetPosition( bike->rearWheelId );

	b2WheelJointDef jointDef = b2DefaultWheelJointDef();
	jointDef.base.bodyIdA = bike->chassisId;
	jointDef.base.bodyIdB = bike->rearWheelId;
	jointDef.base.localFrameA.q = b2MakeRot( 0.5f * B2_PI );
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = 6.0f;
	jointDef.enableMotor = true;
	jointDef.hertz = 5.0f;
	jointDef.dampingRatio = 0.7f;
	jointDef.lowerTranslation = -0.2f;
	jointDef.upperTranslation = 0.2f;
	jointDef.enableLimit = true;
	bike->rearAxleId = b2CreateWheelJoint( worldId, &jointDef );

	pivot = b2Body_GetPosition( bike->frontWheelId );
	jointDef.base.bodyIdB = bike->frontWheelId;
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
	jointDef.enableMotor = false;
	jointDef.hertz = 4.0f;
	bike->frontAxleId = b2CreateWheelJoint( worldId, &jointDef );
}

// chain_car: samples/sample_joints.cpp Driving's car on a smooth open chain with a loop chain rock,
// and a bike behind it on the same chain
static void CreateChainCar( Scene* scene )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = Scene_AddBody( scene, &bodyDef );
	scene->groundId = groundId;

	{
		// Right to left so the solid side faces up
		b2Vec2 points[171];
		for ( int i = 0; i < 171; ++i )
		{
			float x = 300.0f - 2.0f * (float)i;
			float y = Scene_TerrainHeight( x );
			points[i] = (b2Vec2){ x, y };
		}

		b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
		material.friction = 0.8f;

		float ghostX = 302.0f;
		float ghostY = Scene_TerrainHeight( ghostX );

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 171;
		chainDef.ghost1 = (b2Vec2){ ghostX, ghostY };
		chainDef.ghost2 = (b2Vec2){ -42.0f, 0.0f };
		chainDef.materials = &material;
		chainDef.materialCount = 1;
		b2CreateChain( groundId, &chainDef );
	}

	{
		// Counter-clockwise so the solid side faces out
		float x = 40.0f;
		float y = Scene_TerrainHeight( x );
		b2BodyDef rockDef = b2DefaultBodyDef();
		rockDef.position = (b2Pos){ x, y };
		b2BodyId rockId = Scene_AddBody( scene, &rockDef );

		b2Vec2 points[8];
		for ( int i = 0; i < 8; ++i )
		{
			float angle = 0.25f * B2_PI * (float)i;
			b2CosSin cs = b2ComputeCosSin( angle );
			points[i] = (b2Vec2){ 2.5f * cs.cosine, 0.5f * cs.sine };
		}

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 8;
		chainDef.isLoop = true;
		b2CreateChain( rockId, &chainDef );
	}

	Scene_SpawnCar( scene, &scene->car, (b2Pos){ 0.0f, 0.0f }, 1.0f, 5.0f, 0.7f, 5.0f );
	Scene_SpawnBike( scene, &scene->bike, (b2Pos){ -12.0f, 0.0f } );
}

static void Probe_Vehicle( Scene* scene, SceneVehicle* vehicle )
{
	Probe_Float( scene, b2WheelJoint_GetCurrentMotorTorque( vehicle->rearAxleId ) );
	Probe_Joint( scene, vehicle->rearAxleId );
	Probe_Float( scene, b2WheelJoint_GetCurrentMotorTorque( vehicle->frontAxleId ) );
	Probe_Joint( scene, vehicle->frontAxleId );

	b2ContactData contactData[SCENE_CONTACT_CAPACITY];
	int count = b2Body_GetContactData( vehicle->rearWheelId, contactData, SCENE_CONTACT_CAPACITY );
	Probe_Int( scene, count );
	for ( int i = 0; i < count; ++i )
	{
		Probe_ContactData( scene, contactData[i] );
	}
}

static void StepChainCar( Scene* scene, int stepIndex )
{
	SceneVehicle* car = &scene->car;
	SceneVehicle* bike = &scene->bike;

	// Negative motor speed drives to the right
	if ( stepIndex == 30 )
	{
		Scene_SetCarSpeed( car, -35.0f );
	}
	else if ( stepIndex == 330 )
	{
		Scene_SetCarSpeed( car, 0.0f );
	}
	else if ( stepIndex == 420 )
	{
		Scene_SetCarSpeed( car, 20.0f );
	}
	else if ( stepIndex == 540 )
	{
		b2WheelJoint_SetMaxMotorTorque( car->rearAxleId, 8.0f );
		b2WheelJoint_SetMaxMotorTorque( car->frontAxleId, 8.0f );
		b2WheelJoint_SetSpringHertz( car->rearAxleId, 3.0f );
		b2WheelJoint_SetSpringHertz( car->frontAxleId, 3.0f );
		b2WheelJoint_SetSpringDampingRatio( car->rearAxleId, 1.0f );
		b2WheelJoint_SetSpringDampingRatio( car->frontAxleId, 1.0f );
		Scene_SetCarSpeed( car, -40.0f );
	}

	if ( stepIndex == 40 )
	{
		b2WheelJoint_SetTargetMotorSpeed( bike->rearAxleId, -30.0f );
		b2Joint_WakeBodies( bike->rearAxleId );
	}
	else if ( stepIndex == 400 )
	{
		b2WheelJoint_SetMaxMotorTorque( bike->rearAxleId, 10.0f );
		b2WheelJoint_SetTargetMotorSpeed( bike->rearAxleId, 0.0f );
	}
	else if ( stepIndex == 460 )
	{
		b2WheelJoint_SetMaxMotorTorque( bike->rearAxleId, 6.0f );
		b2WheelJoint_SetTargetMotorSpeed( bike->rearAxleId, -40.0f );
	}
	else if ( stepIndex == 600 )
	{
		b2WheelJoint_SetSpringHertz( bike->frontAxleId, 8.0f );
		b2WheelJoint_EnableLimit( bike->rearAxleId, false );
	}

	Scene_WorldStep( scene );

	Probe_Vehicle( scene, car );
	Probe_Vehicle( scene, bike );
}

// shared/human.c CreateHuman, one table row per bone in the order the C creates them. Colors, names
// and user data are dropped.
typedef struct SceneBoneDef
{
	int parentIndex;
	float positionY;
	float linearDamping;
	float frictionScale;
	float capsuleY1;
	float capsuleY2;
	float capsuleRadius;
	float pivotY;
	float lowerAngle;
	float upperAngle;
	bool foot;
	bool rotateFrameA;
} SceneBoneDef;

static const SceneBoneDef s_boneDefs[SCENE_BONE_COUNT] = {
	// hip
	{ -1, 0.95f, 0.0f, 1.0f, -0.02f, 0.02f, 0.095f, 0.0f, 0.0f, 0.0f, false, false },
	// torso
	{ 0, 1.2f, 0.0f, 0.5f, -0.135f, 0.135f, 0.09f, 1.0f, -0.25f, 0.0f, false, false },
	// head
	{ 1, 1.475f, 0.1f, 0.25f, -0.038f, 0.039f, 0.075f, 1.4f, -0.3f, 0.1f, false, false },
	// upper left leg
	{ 0, 0.775f, 0.0f, 1.0f, -0.125f, 0.125f, 0.06f, 0.9f, -0.05f, 0.4f, false, false },
	// lower left leg
	{ 3, 0.475f, 0.0f, 0.5f, -0.155f, 0.125f, 0.045f, 0.625f, -0.5f, -0.02f, true, false },
	// upper right leg
	{ 0, 0.775f, 0.0f, 1.0f, -0.125f, 0.125f, 0.06f, 0.9f, -0.05f, 0.4f, false, false },
	// lower right leg
	{ 5, 0.475f, 0.0f, 0.5f, -0.155f, 0.125f, 0.045f, 0.625f, -0.5f, -0.02f, true, false },
	// upper left arm
	{ 1, 1.225f, 0.0f, 0.5f, -0.125f, 0.125f, 0.035f, 1.35f, -0.1f, 0.8f, false, false },
	// lower left arm
	{ 7, 0.975f, 0.1f, 0.1f, -0.125f, 0.125f, 0.03f, 1.1f, -0.2f, 0.3f, false, true },
	// upper right arm
	{ 1, 1.225f, 0.0f, 0.5f, -0.125f, 0.125f, 0.035f, 1.35f, -0.1f, 0.8f, false, false },
	// lower right arm
	{ 9, 0.975f, 0.1f, 0.1f, -0.125f, 0.125f, 0.03f, 1.1f, -0.2f, 0.3f, false, true },
};

// Returns the torso body
static b2BodyId Scene_CreateHuman( Scene* scene, b2Pos position, float scale, float frictionTorque, float hertz,
								   float dampingRatio, int groupIndex )
{
	b2WorldId worldId = scene->worldId;
	b2BodyId bodyIds[SCENE_BONE_COUNT];

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.sleepThreshold = 0.1f;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.2f;
	shapeDef.filter.groupIndex = -groupIndex;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ( 1 | 2 );

	b2ShapeDef footShapeDef = shapeDef;
	footShapeDef.material.friction = 0.05f;

	// feet don't collide with ragdolls
	footShapeDef.filter.categoryBits = 2;
	footShapeDef.filter.maskBits = 1;

	float s = scale;
	float maxTorque = frictionTorque * s;
	bool enableMotor = true;
	bool enableLimit = true;
	float drawSize = 0.05f;

	b2Vec2 points[4] = {
		{ -0.03f * s, -0.185f * s },
		{ 0.11f * s, -0.185f * s },
		{ 0.11f * s, -0.16f * s },
		{ -0.03f * s, -0.14f * s },
	};

	b2Hull footHull = b2ComputeHull( points, 4 );
	b2Polygon footPolygon = b2MakePolygon( &footHull, 0.015f * s );

	for ( int i = 0; i < SCENE_BONE_COUNT; ++i )
	{
		const SceneBoneDef* def = s_boneDefs + i;

		bodyDef.position = b2OffsetPos( position, (b2Vec2){ 0.0f, def->positionY * s } );
		bodyDef.linearDamping = def->linearDamping;
		bodyIds[i] = Scene_AddBody( scene, &bodyDef );

		b2Capsule capsule = { { 0.0f, def->capsuleY1 * s }, { 0.0f, def->capsuleY2 * s }, def->capsuleRadius * s };
		b2CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );

		if ( def->foot )
		{
			b2CreatePolygonShape( bodyIds[i], &footShapeDef, &footPolygon );
		}

		if ( def->parentIndex < 0 )
		{
			continue;
		}

		b2Pos pivot = b2OffsetPos( position, (b2Vec2){ 0.0f, def->pivotY * s } );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = bodyIds[def->parentIndex];
		jointDef.base.bodyIdB = bodyIds[i];
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
		if ( def->rotateFrameA )
		{
			jointDef.base.localFrameA.q = b2MakeRot( 0.25f * B2_PI );
		}
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = def->lowerAngle * B2_PI;
		jointDef.upperAngle = def->upperAngle * B2_PI;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = def->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.base.drawScale = drawSize;

		b2CreateRevoluteJoint( worldId, &jointDef );
	}

	return bodyIds[1];
}

// joints: one rig per joint type along a floor, from samples/sample_joints.cpp, plus a ragdoll.
// keyJoints: 0 distance, 1 motor, 2 filter, 3 prismatic, 4 revolute, 5 wheel, 6..9 rigid welds,
// 10..13 soft welds. keyBodies: 0 motor body, 1 ragdoll torso, 2 rigid cantilever tip, 3 second
// filtered box.
// Joint user data is the key slot plus one.
static void CreateJoints( Scene* scene )
{
	b2WorldId worldId = scene->worldId;

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = Scene_AddBody( scene, &groundDef );
	scene->groundId = groundId;

	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -40.0f, 0.0f }, { 50.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	// Distance joint with spring, limits and motor
	{
		bodyDef.position = (b2Pos){ -30.0f, 6.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Polygon box = b2MakeSquare( 0.25f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );

		b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ -30.0f, 10.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.25f };
		jointDef.base.forceThreshold = 12.0f;
		jointDef.base.userData = (void*)(uintptr_t)1;
		jointDef.length = 4.0f;
		jointDef.enableSpring = true;
		jointDef.hertz = 2.0f;
		jointDef.dampingRatio = 0.3f;
		jointDef.enableLimit = true;
		jointDef.minLength = 3.0f;
		jointDef.maxLength = 5.0f;
		jointDef.enableMotor = true;
		jointDef.maxMotorForce = 20.0f;
		jointDef.motorSpeed = 1.0f;
		scene->keyJoints[0] = b2CreateDistanceJoint( worldId, &jointDef );
	}

	// Motor joint, velocity driven then spring driven by the script
	{
		bodyDef.position = (b2Pos){ -20.0f, 3.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[0] = bodyId;
		b2Polygon box = b2MakeBox( 0.75f, 0.25f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );

		b2MotorJointDef jointDef = b2DefaultMotorJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ -20.0f, 3.0f };
		jointDef.base.userData = (void*)(uintptr_t)2;
		jointDef.maxVelocityForce = 500.0f;
		jointDef.maxVelocityTorque = 100.0f;
		scene->keyJoints[1] = b2CreateMotorJoint( worldId, &jointDef );
	}

	// Filter joint between two overlapping boxes, destroyed by the script
	{
		bodyDef.position = (b2Pos){ -12.0f, 0.5f };
		b2BodyId bodyIdA = Scene_AddBody( scene, &bodyDef );
		b2Polygon box = b2MakeSquare( 0.5f );
		b2CreatePolygonShape( bodyIdA, &shapeDef, &box );

		bodyDef.position = (b2Pos){ -11.5f, 1.2f };
		b2BodyId bodyIdB = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[3] = bodyIdB;
		b2CreatePolygonShape( bodyIdB, &shapeDef, &box );

		b2FilterJointDef jointDef = b2DefaultFilterJointDef();
		jointDef.base.bodyIdA = bodyIdA;
		jointDef.base.bodyIdB = bodyIdB;
		jointDef.base.userData = (void*)(uintptr_t)3;
		scene->keyJoints[2] = b2CreateFilterJoint( worldId, &jointDef );
	}

	// Prismatic joint on a diagonal axis with limit, motor and spring
	{
		bodyDef.position = (b2Pos){ -5.0f, 4.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Polygon box = b2MakeBox( 0.5f, 0.25f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );

		b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ -5.0f, 4.0f };
		jointDef.base.localFrameA.q = b2MakeRot( 0.25f * B2_PI );
		jointDef.base.userData = (void*)(uintptr_t)4;
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -2.0f;
		jointDef.upperTranslation = 2.0f;
		jointDef.enableMotor = true;
		jointDef.maxMotorForce = 50.0f;
		jointDef.motorSpeed = 1.0f;
		jointDef.enableSpring = true;
		jointDef.hertz = 1.0f;
		jointDef.dampingRatio = 0.5f;
		scene->keyJoints[3] = b2CreatePrismaticJoint( worldId, &jointDef );
	}

	// Revolute joint with limit, motor and spring
	{
		bodyDef.position = (b2Pos){ 4.0f, 5.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Polygon box = b2MakeBox( 1.0f, 0.1f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );

		b2Pos pivot = { 3.0f, 5.0f };
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( groundId, pivot );
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint( bodyId, pivot );
		jointDef.base.torqueThreshold = 8.0f;
		jointDef.base.userData = (void*)(uintptr_t)5;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.5f * B2_PI;
		jointDef.upperAngle = 0.75f * B2_PI;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 50.0f;
		jointDef.motorSpeed = 1.5f;
		jointDef.enableSpring = true;
		jointDef.hertz = 1.0f;
		jointDef.dampingRatio = 0.5f;
		scene->keyJoints[4] = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// Wheel joint with limit, motor and spring
	{
		bodyDef.position = (b2Pos){ 24.0f, 3.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );

		b2WheelJointDef jointDef = b2DefaultWheelJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ 24.0f, 3.0f };
		jointDef.base.localFrameA.q = b2MakeRot( 0.5f * B2_PI );
		jointDef.base.forceThreshold = 30.0f;
		jointDef.base.userData = (void*)(uintptr_t)6;
		jointDef.enableSpring = true;
		jointDef.hertz = 2.0f;
		jointDef.dampingRatio = 0.5f;
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -0.5f;
		jointDef.upperTranslation = 0.5f;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 10.0f;
		jointDef.motorSpeed = 4.0f;
		scene->keyJoints[5] = b2CreateWheelJoint( worldId, &jointDef );
	}

	// Two weld cantilevers from samples/sample_joints.cpp Cantilever, rigid then soft, each with a
	// box dropped on its tip
	for ( int k = 0; k < 2; ++k )
	{
		float x0 = k == 0 ? 10.0f : 17.0f;
		float hx = 0.5f;
		b2Polygon box = b2MakeBox( hx, 0.125f );

		b2WeldJointDef jointDef = b2DefaultWeldJointDef();
		if ( k == 1 )
		{
			jointDef.linearHertz = 5.0f;
			jointDef.angularHertz = 3.0f;
			jointDef.linearDampingRatio = 0.5f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.base.torqueThreshold = 3.0f;
		}

		b2BodyId prevBodyId = groundId;
		for ( int i = 0; i < 4; ++i )
		{
			bodyDef.position = (b2Pos){ x0 + hx + 2.0f * hx * (float)i, 4.0f };
			b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Pos pivot = { x0 + 2.0f * hx * (float)i, 4.0f };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			int slot = 6 + 4 * k + i;
			jointDef.base.userData = (void*)(uintptr_t)( slot + 1 );
			scene->keyJoints[slot] = b2CreateWeldJoint( worldId, &jointDef );

			prevBodyId = bodyId;
		}

		if ( k == 0 )
		{
			scene->keyBodies[2] = prevBodyId;
		}

		bodyDef.position = (b2Pos){ x0 + 3.5f, 7.0f };
		b2BodyId dropId = Scene_AddBody( scene, &bodyDef );
		b2Polygon dropBox = b2MakeSquare( 0.3f );
		b2CreatePolygonShape( dropId, &shapeDef, &dropBox );
	}

	scene->keyJointCount = 14;

	// samples/sample_joints.cpp Ragdoll
	scene->keyBodies[1] = Scene_CreateHuman( scene, (b2Pos){ 34.0f, 5.0f }, 1.0f, 0.03f, 5.0f, 0.5f, 1 );
}

static void StepJoints( Scene* scene, int stepIndex )
{
	b2JointId* joints = scene->keyJoints;
	float timeStep = 1.0f / 60.0f;

	if ( stepIndex == 100 )
	{
		b2DistanceJoint_SetRestLength( joints[0], 2.5f );
	}
	else if ( stepIndex == 160 )
	{
		b2DistanceJoint_SetLimits( joints[0], 2.0f, 3.5f );
		b2DistanceJoint_SetTargetMotorSpeed( joints[0], -2.0f );
	}
	else if ( stepIndex == 250 )
	{
		b2DistanceJoint_EnableSpring( joints[0], false );
	}

	if ( stepIndex < 200 )
	{
		b2CosSin cs1 = b2ComputeCosSin( scene->time );
		float angle2 = 2.0f * scene->time;
		b2CosSin cs2 = b2ComputeCosSin( angle2 );
		b2Vec2 v = { 2.0f * cs1.cosine, 1.5f * cs2.sine };
		b2MotorJoint_SetLinearVelocity( joints[1], v );
		b2MotorJoint_SetAngularVelocity( joints[1], 1.0f );
	}
	else if ( stepIndex == 200 )
	{
		b2MotorJoint_SetLinearVelocity( joints[1], b2Vec2_zero );
		b2MotorJoint_SetAngularVelocity( joints[1], 0.0f );
		b2MotorJoint_SetLinearHertz( joints[1], 2.0f );
		b2MotorJoint_SetLinearDampingRatio( joints[1], 0.7f );
		b2MotorJoint_SetMaxSpringForce( joints[1], 200.0f );
		b2MotorJoint_SetAngularHertz( joints[1], 1.0f );
		b2MotorJoint_SetAngularDampingRatio( joints[1], 0.5f );
		b2MotorJoint_SetMaxSpringTorque( joints[1], 50.0f );
	}

	if ( stepIndex == 200 )
	{
		b2DestroyJoint( joints[2] );
	}
	else if ( stepIndex == 205 )
	{
		// Moving the proxy makes the broad phase pair the boxes again
		b2Body_SetTransform( scene->keyBodies[3], (b2Pos){ -11.6f, 0.6f }, b2Rot_identity );
	}

	if ( stepIndex % 100 == 50 )
	{
		float speed = b2PrismaticJoint_GetTargetMotorSpeed( joints[3] );
		b2PrismaticJoint_SetTargetMotorSpeed( joints[3], -speed );
	}
	if ( stepIndex == 250 )
	{
		b2PrismaticJoint_SetTargetTranslation( joints[3], 1.0f );
	}
	else if ( stepIndex == 300 )
	{
		b2PrismaticJoint_EnableMotor( joints[3], false );
	}
	else if ( stepIndex == 350 )
	{
		b2PrismaticJoint_SetLimits( joints[3], -1.0f, 1.0f );
	}

	if ( stepIndex == 120 )
	{
		b2RevoluteJoint_SetTargetMotorSpeed( joints[4], -2.0f );
	}
	else if ( stepIndex == 200 )
	{
		b2RevoluteJoint_SetLimits( joints[4], -0.25f * B2_PI, 0.25f * B2_PI );
	}
	else if ( stepIndex == 260 )
	{
		b2RevoluteJoint_SetTargetAngle( joints[4], 0.5f );
	}
	else if ( stepIndex == 320 )
	{
		b2RevoluteJoint_EnableMotor( joints[4], false );
		b2RevoluteJoint_SetSpringHertz( joints[4], 3.0f );
	}

	if ( stepIndex == 150 )
	{
		b2WheelJoint_SetTargetMotorSpeed( joints[5], -4.0f );
	}
	else if ( stepIndex == 250 )
	{
		b2WheelJoint_SetSpringHertz( joints[5], 5.0f );
	}
	else if ( stepIndex == 300 )
	{
		b2WheelJoint_EnableLimit( joints[5], false );
	}

	if ( stepIndex == 200 )
	{
		for ( int i = 10; i < 14; ++i )
		{
			b2WeldJoint_SetAngularHertz( joints[i], 1.0f );
			b2WeldJoint_SetLinearDampingRatio( joints[i], 1.0f );
		}
		b2Body_ApplyLinearImpulseToCenter( scene->keyBodies[2], (b2Vec2){ 0.0f, -4.0f }, true );
	}

	if ( stepIndex == 100 )
	{
		b2Body_ApplyAngularImpulse( scene->keyBodies[1], 0.5f, true );
	}
	else if ( stepIndex == 150 )
	{
		b2Body_ApplyLinearImpulseToCenter( scene->keyBodies[1], (b2Vec2){ -3.0f, 4.0f }, true );
	}

	scene->time += timeStep;

	Scene_WorldStep( scene );

	Probe_JointEvents( scene );
	for ( int i = 0; i < scene->keyJointCount; ++i )
	{
		Probe_Joint( scene, joints[i] );
	}

	Probe_Float( scene, b2DistanceJoint_GetCurrentLength( joints[0] ) );
	Probe_Float( scene, b2DistanceJoint_GetSpringForce( joints[0] ) );
	Probe_Float( scene, b2DistanceJoint_GetCurrentMotorForce( joints[0] ) );
	Probe_Float( scene, b2PrismaticJoint_GetCurrentTranslation( joints[3] ) );
	Probe_Float( scene, b2PrismaticJoint_GetCurrentSpeed( joints[3] ) );
	Probe_Float( scene, b2PrismaticJoint_GetCurrentMotorForce( joints[3] ) );
	Probe_Float( scene, b2RevoluteJoint_GetCurrentAngle( joints[4] ) );
	Probe_Float( scene, b2RevoluteJoint_GetCurrentSpeed( joints[4] ) );
	Probe_Float( scene, b2RevoluteJoint_GetCurrentMotorTorque( joints[4] ) );
	Probe_Float( scene, b2WheelJoint_GetCurrentMotorTorque( joints[5] ) );
}

// The svg levels of samples/sample_character.cpp, run through Sample::ParsePath at scale 0.2 and
// stored as float bits, x then y
static const uint32_t s_geometricLevel1Bits[] = {
	0xc1870889, 0xbe5ddd9a, 0x4242f333, 0xbe5ddd9a, 0x4242f333, 0x4114eef0, 0x4240d555, 0x4114eef0,
	0x42385ddd, 0x40f71113, 0x422dc888, 0x40c44447, 0x42232fa7, 0x40a26667, 0x4209cccc, 0x405f3333,
	0x41f1bbba, 0x403d555a, 0x41cba220, 0x3f57779a, 0x41b23bba, 0x3f577767, 0x41b23bba, 0x3fd1554d,
	0x4196b775, 0x3fd15533, 0x4196b775, 0x3f8d999a, 0x41766663, 0x3f8d999a, 0x41766663, 0x3faf7780,
	0x4169b330, 0x3faf7780, 0x4169b330, 0x3fd15567, 0x415cfffd, 0x3fd15567, 0x415cfffd, 0x3ff3334d,
	0x41504cca, 0x3ff3334d, 0x41504cca, 0x400a888d, 0x41439997, 0x400a888d, 0x41439997, 0x401b7780,
	0x4136e663, 0x401b7780, 0x4136e663, 0x402c6673, 0x412a3330, 0x402c6673, 0x412a3330, 0x403d555a,
	0x411d7ffb, 0x403d555a, 0x411d7ffb, 0x404e444d, 0x4110ccc8, 0x404e444d, 0x4110ccc8, 0x405f3340,
	0x41041995, 0x405f3340, 0x41041995, 0x40702227, 0x40eeccc3, 0x40702227, 0x40eeccc3, 0x4080888d,
	0x40d5665d, 0x4080888d, 0x40d5665d, 0x40890007, 0x40bbfff7, 0x40890007, 0x40bbfff7, 0x4091777a,
	0x40a29990, 0x4091777a, 0x40a29990, 0x4099eef3, 0x4089332a, 0x4099eef3, 0x4089332a, 0x40a2666d,
	0x405f9987, 0x40a2666d, 0x405f9987, 0x40aadde0, 0x402cccba, 0x40aadde0, 0x402cccb7, 0x40b35560,
	0x3f15555a, 0x40b35560, 0xc0040000, 0x4091777a, 0xc047bbbd, 0x4091777a, 0xc096aaab, 0x405f3333,
	0xc0da6667, 0x405f3333, 0xc182cccd, 0x4158aaaa, 0xc182cccd, 0x4196aaab, 0xc1870889, 0x4196aaab,
};

static const uint32_t s_dynamicLevel1Bits[] = {
	0xc1870889, 0xbe5ddd9a, 0x4242f333, 0xbe5ddd9a, 0x4242f333, 0x4114eef0, 0x4240d555, 0x4114eef0,
	0x42385ddd, 0x40f71113, 0x422dc888, 0x40c44447, 0x42232fa7, 0x40a26667, 0x4209cccc, 0x405f3333,
	0x41f1bbba, 0x403d555a, 0x41cba220, 0x3e9fff9a, 0x41b23bba, 0x3e9fff33, 0x41b23bba, 0x3fd1554d,
	0x4196b775, 0x3fd15533, 0x4196b775, 0x3f8d999a, 0x41766663, 0x3f8d999a, 0x41766663, 0x3faf7780,
	0x4169b330, 0x3faf7780, 0x4169b330, 0x3fd15567, 0x415cfffd, 0x3fd15567, 0x415cfffd, 0x3ff3334d,
	0x41504cca, 0x3ff3334d, 0x41504cca, 0x400a888d, 0x41439997, 0x400a888d, 0x41439997, 0x401b7780,
	0x4136e663, 0x401b7780, 0x4136e663, 0x402c6673, 0x412a3330, 0x402c6673, 0x412a3330, 0x403d555a,
	0x411d7ffb, 0x403d555a, 0x411d7ffb, 0x404e444d, 0x4110ccc8, 0x404e444d, 0x4110ccc8, 0x405f3340,
	0x41041995, 0x405f3340, 0x41041995, 0x40702227, 0x40eeccc3, 0x40702227, 0x40eeccc3, 0x4080888d,
	0x40d5665d, 0x4080888d, 0x40d5665d, 0x40890007, 0x40bbfff7, 0x40890007, 0x40bbfff7, 0x4091777a,
	0x40a29990, 0x4091777a, 0x40a29990, 0x4099eef3, 0x4089332a, 0x4099eef3, 0x4089332a, 0x40a2666d,
	0x405f9987, 0x40a2666d, 0x405f9987, 0x40aadde0, 0x402cccba, 0x40aadde0, 0x402cccb7, 0x40b35560,
	0x3f15555a, 0x40b35560, 0xc0040000, 0x4091777a, 0xc047bbbd, 0x4091777a, 0xc096aaab, 0x405f3333,
	0xc0da6667, 0x405f3333, 0xc182cccd, 0x4158aaaa, 0xc182cccd, 0x4196aaab, 0xc1870889, 0x4196aaab,
};

static const uint32_t s_level2Bits[] = {
	0x3f077777, 0xbe5ddd9a, 0x429fceef, 0xbe5ddd9a, 0x429fceef, 0x4091777a, 0x42964889, 0x4091777a,
	0x429ec000, 0x3f577767, 0x42033bbb, 0x3f57779a, 0x42032ff5, 0x3ea00000, 0x41fde873, 0x3ea00000,
	0x41fde873, 0x3f577767, 0x41f1517f, 0x3f57779a, 0x41f14cd3, 0x3faf7780, 0x41ed1118, 0x3faf7780,
	0x41e8d55d, 0x3f577767, 0x41dc222a, 0x3f577767, 0x41d7e66d, 0x3ea00000, 0x41d3c202, 0x3f57779a,
	0x41c2bbbc, 0x3f577767, 0x41c2bbbc, 0x3faf7767, 0x41b60889, 0x3faf7767, 0x41b60889, 0x3ff33333,
	0x41a95556, 0x3ff33333, 0x41a95556, 0x401b7773, 0x419ca223, 0x401b7773, 0x419ca223, 0x403d554d,
	0x418feeef, 0x403d554d, 0x418feeef, 0x405f3327, 0x41833bbc, 0x405f3333, 0x41833bbc, 0x4080888d,
	0x416d1112, 0x40808887, 0x416d1112, 0x40917773, 0x4153aaab, 0x4091777a, 0x4153aaab, 0x40a26667,
	0x413a4445, 0x40a26660, 0x413a4445, 0x40b35553, 0x41295556, 0x40b35553, 0x410feeef, 0x40c44447,
	0x40ed1112, 0x40d53333, 0x40cb3333, 0x40f71113, 0x40877778, 0x40f71113, 0x40295556, 0x41040000,
	0x3f877778, 0x4114eef0, 0x3f077778, 0x4114eef0,
};

static b2BodyId Scene_CreateLevelChain( Scene* scene, b2Pos position, const uint32_t* bits, int count )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = position;
	b2BodyId groundId = Scene_AddBody( scene, &bodyDef );

	b2Vec2 points[64];
	for ( int i = 0; i < count; ++i )
	{
		memcpy( &points[i].x, bits + 2 * i, sizeof( float ) );
		memcpy( &points[i].y, bits + 2 * i + 1, sizeof( float ) );
	}

	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.points = points;
	chainDef.pointCount = count;
	chainDef.isLoop = true;
	b2CreateChain( groundId, &chainDef );

	return groundId;
}

// The props both character samples place after the level: plank bridge, balls, elevator, box
// pyramid and motorized door. The geometric sample tags the elevator for the mover planes.
static void Scene_CreateCharacterProps( Scene* scene, b2BodyId groundId1, b2BodyId groundId2, float doorY,
										bool geometric )
{
	{
		b2Polygon box = b2MakeBox( 0.55f, 0.125f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.maxMotorTorque = 10.0f;
		jointDef.enableMotor = true;
		jointDef.hertz = 3.0f;
		jointDef.dampingRatio = 0.8f;
		jointDef.enableSpring = true;

		float xBase = 48.7f;
		float yBase = 9.2f;
		int count = 50;
		b2BodyId prevBodyId = groundId1;
		for ( int i = 0; i < count; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = (b2Pos){ xBase + 0.5f + 1.0f * (float)i, yBase };
			bodyDef.angularDamping = 0.2f;
			b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Pos pivot = { xBase + 1.0f * (float)i, yBase };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			b2CreateRevoluteJoint( scene->worldId, &jointDef );

			prevBodyId = bodyId;
		}

		b2Pos pivot = { xBase + 1.0f * (float)count, yBase };
		jointDef.base.bodyIdA = prevBodyId;
		jointDef.base.bodyIdB = groundId2;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
		b2CreateRevoluteJoint( scene->worldId, &jointDef );
	}

	for ( int i = 0; i < 5; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 7.0f, 7.0f + 0.5f * (float)i };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter = (b2Filter){ s_dynamicBit, s_allBits, 0 };
		shapeDef.material.restitution = 0.7f;
		shapeDef.material.rollingResistance = 0.2f;

		b2Circle circle = { { 0.0f, 0.0f }, 0.25f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = (b2Pos){ 112.0f, 10.0f - 4.0f };
		b2BodyId elevatorId = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[0] = elevatorId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter = (b2Filter){ s_dynamicBit, s_allBits, 0 };
		if ( geometric )
		{
			shapeDef.userData = (void*)(uintptr_t)2;
		}

		b2Polygon box = b2MakeBox( 2.0f, 0.1f );
		b2CreatePolygonShape( elevatorId, &shapeDef, &box );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.x = 140.0f;
		float a = 0.25f;
		b2Polygon square = b2MakeSquare( a );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter = (b2Filter){ s_dynamicBit, s_allBits, 0 };
		for ( int i = 0; i < 10; ++i )
		{
			bodyDef.position.y = ( 2.0f * (float)i + 1.0f ) * a + 1.0f;
			b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &square );
		}
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 160.0f, doorY };
		bodyDef.type = b2_dynamicBody;
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );

		b2Polygon box = b2MakeBox( 0.1f, 4.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		b2CreatePolygonShape( bodyId, &shapeDef, &box );

		b2Pos pivot = b2Add( bodyDef.position, (b2Vec2){ 0.0f, 4.0f } );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId2;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 1.0f;
		jointDef.maxMotorTorque = 500.0f;
		b2CreateRevoluteJoint( scene->worldId, &jointDef );
	}
}

// The elevator script both character samples run before the step, with b2ComputeCosSin for cosf
static void Scene_MoveElevator( Scene* scene, float timeStep )
{
	float angle = 1.0f * scene->time + B2_PI;
	b2CosSin cs = b2ComputeCosSin( angle );
	b2Pos point = { 112.0f, 4.0f * cs.cosine + 10.0f };
	b2WorldTransform target = { point, b2Rot_identity };
	bool wake = true;
	b2Body_SetTargetTransform( scene->keyBodies[0], target, timeStep, wake );

	scene->time += timeStep;
}

// Scripted keys for both movers: throttle and the jump key
static float Scene_MoverThrottle( int stepIndex )
{
	if ( stepIndex < 60 )
	{
		return 0.0f;
	}
	if ( stepIndex < 300 )
	{
		return 1.0f;
	}
	if ( stepIndex < 360 )
	{
		return -1.0f;
	}
	return 1.0f;
}

static bool Scene_MoverJumpKey( int stepIndex )
{
	int phase = stepIndex % 300;
	return stepIndex >= 150 && phase >= 150 && phase < 154;
}

static float Scene_CastCallback( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	SceneCastResult* result = context;
	result->point = point;
	result->normal = normal;
	result->bodyId = b2Shape_GetBody( shapeId );
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

// samples/dynamic_mover.cpp DynamicMover::DynamicMover and Create with the sample's def
static void Scene_CreateDynamicMover( Scene* scene, SceneDynamicMover* m, b2Pos position )
{
	memset( m, 0, sizeof( SceneDynamicMover ) );
	m->capsule = (b2Capsule){ { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };
	m->filter = (b2Filter){ s_moverBit, s_staticBit | s_dynamicBit, 0 };
	m->jumpSpeed = 7.0f;
	m->maxSpeed = 6.0f;
	m->minSpeed = 0.1f;
	m->stopSpeed = 3.0f;
	m->accelerate = 20.0f;
	m->airSteer = 0.5f;
	m->friction = 8.0f;
	m->gravityScale = 1.5f;
	m->maxGroundForce = 70.0f;
	m->maxAirForce = 20.0f;
	m->pogoRestLength = 0.9f;
	m->pogoHertz = 5.0f;
	m->pogoDampingRatio = 0.8f;
	m->pogoCompressionScale = 100.0f;
	m->pogoTensionScale = 100.0f;
	m->minGroundNormalY = 0.7f;
	m->velocity = b2Vec2_zero;
	m->pogoJointId = b2_nullJointId;
	m->pogoFraction = 1.0f;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	m->groundId = Scene_AddBody( scene, &bodyDef );

	// Position is the center of the capsule.
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = position;
	bodyDef.gravityScale = m->gravityScale;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.enableSleep = false;
	bodyDef.name = "mover";

	m->moverId = Scene_AddBody( scene, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;

	// The mover joint and the pogo joint handle all the movement, surface friction would fight them
	shapeDef.material.friction = 0.0f;
	shapeDef.filter = m->filter;
	shapeDef.enablePreSolveEvents = true;

	b2CreateCapsuleShape( m->moverId, &shapeDef, &m->capsule );

	b2MoverJointDef moverDef = b2DefaultMoverJointDef();
	moverDef.linearVelocity = m->velocity;
	moverDef.maxVelocityForce = (b2Vec2){ m->maxGroundForce, 0.0f };
	moverDef.base.bodyIdA = m->groundId;
	moverDef.base.bodyIdB = m->moverId;
	moverDef.base.collideConnected = true;

	m->moverJointId = b2CreateMoverJoint( scene->worldId, &moverDef );
}

// samples/dynamic_mover.cpp DynamicMover::Jump
static bool Scene_DynamicMoverJump( SceneDynamicMover* m )
{
	if ( m->onGround == false || m->walkable == false )
	{
		return false;
	}

	float surfaceVelocity = 0.0f;
	if ( b2Body_IsValid( m->castResult.bodyId ) )
	{
		b2Vec2 v = b2Body_GetWorldPointVelocity( m->castResult.bodyId, m->castResult.point );
		surfaceVelocity = v.y;
	}

	float mass = b2Body_GetMass( m->moverId );
	float vy = b2Body_GetLinearVelocity( m->moverId ).y;

	// Remove the pogo constraint velocity but add the surface velocity.
	float dv = b2MaxFloat( 0.0f, m->jumpSpeed - vy ) + surfaceVelocity;
	b2Body_ApplyLinearImpulseToCenter( m->moverId, (b2Vec2){ 0.0f, mass * dv }, true );

	// This removes the pogo step down on the next update.
	m->onGround = false;
	m->walkable = false;
	m->jumping = true;
	m->jumpTicks = 0;
	return true;
}

// samples/dynamic_mover.cpp DynamicMover::Update
static void Scene_DynamicMoverUpdate( Scene* scene, SceneDynamicMover* m, float timeStep, float throttle )
{
	b2WorldId worldId = scene->worldId;

	// Reach further while grounded so the spring can find the ground over a step
	float stepDownLength = m->pogoRestLength;
	float rayLength = m->onGround ? m->pogoRestLength + stepDownLength : m->pogoRestLength;
	b2Vec2 translation = { 0.0f, -rayLength };

	b2Pos position = b2Body_GetPosition( m->moverId );
	b2Pos origin = b2Add( position, m->capsule.center1 );

	b2QueryFilter queryFilter = { m->filter.categoryBits, m->filter.maskBits };

	memset( &m->castResult, 0, sizeof( SceneCastResult ) );
	b2World_CastRay( worldId, origin, translation, queryFilter, Scene_CastCallback, &m->castResult );

	m->pogoFraction = m->castResult.hit ? m->castResult.fraction : 1.0f;
	m->pogoHit = m->castResult.hit;

	// Should jumping end?
	if ( m->jumping )
	{
		m->jumpTicks += 1;

		// Jump ticks allow time for the jump to leave the ground. Otherwise jumping ends
		// when the character approaches the current ground.
		if ( m->jumpTicks > 2 && m->castResult.hit && b2Dot( m->velocity, m->castResult.normal ) <= 0.0f )
		{
			m->jumping = false;
		}
	}

	if ( m->castResult.hit )
	{
		// The cast can still hit while jumping.
		m->onGround = m->jumping == false;

		// Is the ground too steep to walk?
		m->walkable = m->castResult.normal.y >= m->minGroundNormalY;
	}
	else
	{
		m->onGround = false;
		m->walkable = false;
	}

	m->velocity = b2Body_GetLinearVelocity( m->moverId );

	// Friction
	if ( m->onGround )
	{
		float speed = b2Length( m->velocity );
		if ( speed < m->minSpeed )
		{
			m->velocity.x = 0.0f;
		}
		else
		{
			// Linear damping above stopSpeed and fixed reduction below stopSpeed
			float control = speed < m->stopSpeed ? m->stopSpeed : speed;

			// friction has units of 1/time
			float drop = control * m->friction * timeStep;
			float newSpeed = b2MaxFloat( 0.0f, speed - drop );
			m->velocity.x *= newSpeed / speed;
		}
	}

	// Mover force
	if ( m->onGround )
	{
		float maxForce = m->walkable ? m->maxGroundForce : 0.0f;
		b2MoverJoint_SetMaxVelocityForce( m->moverJointId, (b2Vec2){ maxForce, 0.0f } );
	}
	else
	{
		b2MoverJoint_SetMaxVelocityForce( m->moverJointId, (b2Vec2){ m->maxAirForce, 0.0f } );
	}

	b2Vec2 desiredVelocity = { m->maxSpeed * throttle, 0.0f };
	float desiredSpeed;
	b2Vec2 desiredDirection = b2GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > m->maxSpeed )
	{
		desiredSpeed = m->maxSpeed;
	}

	// Accelerate
	float currentSpeed = b2Dot( m->velocity, desiredDirection );
	float addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > 0.0f )
	{
		float steer = m->walkable ? 1.0f : m->airSteer;
		float accelSpeed = steer * m->accelerate * m->maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m->velocity.x += accelSpeed * desiredDirection.x;
	}

	// The pogo joint is rebuilt every solve because it may land on a different body
	if ( b2Joint_IsValid( m->pogoJointId ) )
	{
		m->pogoLength = b2PogoJoint_GetCurrentLength( m->pogoJointId );
		m->pogoImpulse = b2PogoJoint_GetImpulse( m->pogoJointId );
		m->pogoVelocity = b2PogoJoint_GetCurrentSpeed( m->pogoJointId );

		b2DestroyJoint( m->pogoJointId );
		m->pogoJointId = b2_nullJointId;
	}

	if ( m->castResult.hit == true )
	{
		float moverMass = b2Body_GetMass( m->moverId );
		float moverWeight = m->gravityScale * b2Length( b2World_GetGravity( worldId ) ) * moverMass;

		b2PogoJointDef pogoDef = b2DefaultPogoJointDef();
		pogoDef.base.localFrameA.p = b2Body_GetLocalPoint( m->castResult.bodyId, m->castResult.point );
		pogoDef.base.localFrameB.p = m->capsule.center1;
		pogoDef.normal = m->castResult.normal;
		pogoDef.base.bodyIdA = m->castResult.bodyId;
		pogoDef.base.bodyIdB = m->moverId;
		pogoDef.base.collideConnected = true;
		pogoDef.restLength = m->pogoRestLength;
		pogoDef.hertz = m->pogoHertz;
		pogoDef.dampingRatio = m->pogoDampingRatio;
		pogoDef.maxCompressionForce = m->pogoCompressionScale * moverWeight;

		// Warm start from the joint that was just destroyed
		pogoDef.impulse = m->pogoImpulse;
		pogoDef.velocity = m->pogoVelocity;

		if ( m->jumping )
		{
			// Don't allow the pogo to pull down
			pogoDef.maxTensionForce = 0.0f;
		}
		else
		{
			// The pogo can pull down at a multiple of the gravity force.
			pogoDef.maxTensionForce = m->pogoTensionScale * moverWeight;
		}

		m->pogoJointId = b2CreatePogoJoint( worldId, &pogoDef );
	}
	else
	{
		m->pogoImpulse = 0.0f;
		m->pogoVelocity = 0.0f;
	}

	b2MoverJoint_SetLinearVelocity( m->moverJointId, m->velocity );
}

// samples/sample_character.cpp DynamicMoverSample::PreSolve: the elevator is one way for the mover
static void Scene_PreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context )
{
	Scene* scene = context;
	b2BodyId idA = b2Shape_GetBody( shapeIdA );
	b2BodyId idB = b2Shape_GetBody( shapeIdB );
	b2BodyId elevatorId = scene->keyBodies[0];
	b2BodyId moverId = scene->dynamicMover.moverId;

	if ( B2_ID_EQUALS( idA, elevatorId ) && B2_ID_EQUALS( idB, moverId ) )
	{
		if ( manifold->normal.y < 0.0f )
		{
			manifold->pointCount = 0;
		}
	}
	else if ( B2_ID_EQUALS( idA, moverId ) && B2_ID_EQUALS( idB, elevatorId ) )
	{
		if ( manifold->normal.y > 0.0f )
		{
			manifold->pointCount = 0;
		}
	}
}

static bool Scene_PreContinuous( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, void* context )
{
	(void)point;
	Scene* scene = context;
	b2BodyId idA = b2Shape_GetBody( shapeIdA );
	b2BodyId idB = b2Shape_GetBody( shapeIdB );
	b2BodyId elevatorId = scene->keyBodies[0];
	b2BodyId moverId = scene->dynamicMover.moverId;

	if ( B2_ID_EQUALS( idA, elevatorId ) && B2_ID_EQUALS( idB, moverId ) )
	{
		if ( normal.y < 0.0f )
		{
			return false;
		}

		return true;
	}

	if ( B2_ID_EQUALS( idA, moverId ) && B2_ID_EQUALS( idB, elevatorId ) )
	{
		if ( normal.y > 0.0f )
		{
			return false;
		}

		return true;
	}

	return true;
}

// dynamic_mover: samples/sample_character.cpp DynamicMoverSample. keyBodies: 0 elevator
static void CreateDynamicMover( Scene* scene )
{
	Scene_CreateDynamicMover( scene, &scene->dynamicMover, (b2Pos){ 0.0f, 8.0f } );

	b2BodyId groundId1 = Scene_CreateLevelChain( scene, (b2Pos){ 0.0f, 0.0f }, s_dynamicLevel1Bits,
												 (int)( sizeof( s_dynamicLevel1Bits ) / 8 ) );
	b2BodyId groundId2 =
		Scene_CreateLevelChain( scene, (b2Pos){ 98.0f, 0.0f }, s_level2Bits, (int)( sizeof( s_level2Bits ) / 8 ) );

	Scene_CreateCharacterProps( scene, groundId1, groundId2, 4.0f, false );

	b2World_SetPreSolveCallback( scene->worldId, Scene_PreSolve, Scene_PreContinuous, scene );

	scene->jumpReleased = true;
	scene->time = 0.0f;
}

static void StepDynamicMover( Scene* scene, int stepIndex )
{
	float timeStep = 1.0f / 60.0f;
	SceneDynamicMover* m = &scene->dynamicMover;

	Scene_MoveElevator( scene, timeStep );

	float throttle = Scene_MoverThrottle( stepIndex );
	if ( Scene_MoverJumpKey( stepIndex ) )
	{
		if ( scene->jumpReleased && Scene_DynamicMoverJump( m ) )
		{
			scene->jumpReleased = false;
		}
	}
	else
	{
		scene->jumpReleased = true;
	}

	Scene_DynamicMoverUpdate( scene, m, timeStep, throttle );

	Scene_WorldStep( scene );

	Probe_Bool( scene, m->castResult.hit );
	Probe_Vec2( scene, m->castResult.point );
	Probe_Vec2( scene, m->castResult.normal );
	Probe_BodyId( scene, m->castResult.bodyId );
	Probe_Float( scene, m->castResult.fraction );
	Probe_Vec2( scene, m->velocity );
	Probe_Bool( scene, m->onGround );
	Probe_Bool( scene, m->walkable );
	Probe_Bool( scene, m->jumping );
	Probe_Int( scene, m->jumpTicks );
	Probe_Float( scene, m->pogoImpulse );
	Probe_Float( scene, m->pogoVelocity );
	Probe_Float( scene, m->pogoLength );
	Probe_JointId( scene, m->pogoJointId );
	Probe_Joint( scene, m->moverJointId );
	Probe_Joint( scene, m->pogoJointId );
}

// samples/geometric_mover.cpp GeometricMover::GeometricMover and Create with the sample's def
static void Scene_CreateGeometricMover( SceneGeometricMover* m, b2Pos position )
{
	memset( m, 0, sizeof( SceneGeometricMover ) );
	m->jumpSpeed = 10.0f;
	m->maxSpeed = 6.0f;
	m->minSpeed = 0.1f;
	m->stopSpeed = 3.0f;
	m->accelerate = 20.0f;
	m->airSteer = 0.2f;
	m->friction = 8.0f;
	m->gravity = 30.0f;
	m->pogoRestLength = 0.9f;
	m->pogoHertz = 5.0f;
	m->pogoDampingRatio = 0.8f;
	m->pogoGroundForce = 50.0f;
	m->pushBodies = true;
	m->capsule = (b2Capsule){ { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };
	m->filter = (b2QueryFilter){ s_moverBit, s_staticBit | s_dynamicBit | s_moverBit };
	m->minGroundNormalY = 0.7f;
	m->position = position;
	m->velocity = b2Vec2_zero;
	m->pogoFraction = 1.0f;
}

// samples/geometric_mover.cpp GeometricMover::Jump
static bool Scene_GeometricMoverJump( SceneGeometricMover* m )
{
	if ( m->onGround == false )
	{
		return false;
	}

	m->velocity.y = m->jumpSpeed;
	m->onGround = false;
	return true;
}

// samples/geometric_mover.cpp PlaneResultFcn. The sample points shape user data at a
// MoverShapeUserData; here the user data is a tag: 1 the friendly capsule, 2 the elevator.
static bool Scene_PlaneResultFcn( b2ShapeId shapeId, const b2PlaneResult* planeResult, void* context )
{
	SceneGeometricMover* m = context;
	float maxPush = FLT_MAX;
	bool clipVelocity = true;
	uint64_t tag = (uint64_t)(uintptr_t)b2Shape_GetUserData( shapeId );
	if ( tag == 1 )
	{
		maxPush = 0.025f;
		clipVelocity = false;
	}
	else if ( tag == 2 )
	{
		maxPush = 0.1f;
		clipVelocity = true;
	}

	if ( m->planeCount < SCENE_MOVER_PLANE_CAPACITY )
	{
		int index = m->planeCount;
		m->planes[index].plane = planeResult->plane;
		m->planes[index].pushLimit = maxPush;
		m->planes[index].push = 0.0f;
		m->planes[index].clipVelocity = clipVelocity;

		// The plane point is relative to the origin passed to the collide call
		m->planePoints[index] = b2OffsetPos( m->position, planeResult->point );
		m->planeShapes[index] = shapeId;
		m->planeCount += 1;
	}

	return true;
}

// samples/geometric_mover.cpp GeometricMover::PushBodies
static void Scene_GeometricMoverPushBodies( SceneGeometricMover* m )
{
	for ( int i = 0; i < m->planeCount; ++i )
	{
		b2BodyId bodyId = b2Shape_GetBody( m->planeShapes[i] );
		if ( b2Body_GetType( bodyId ) != b2_dynamicBody )
		{
			continue;
		}

		// Plane normals point at the mover
		b2Pos point = m->planePoints[i];
		b2Vec2 normal = b2Neg( m->planes[i].plane.normal );

		float invMassA = 0.0f;

		float massB = b2Body_GetMass( bodyId );
		float invMassB = massB > 0.0f ? 1.0f / massB : 0.0f;

		float inertiaB = b2Body_GetRotationalInertia( bodyId );
		float invIB = inertiaB > 0.0f ? 1.0f / inertiaB : 0.0f;

		b2Pos centerB = b2Body_GetWorldCenter( bodyId );
		b2Vec2 rB = b2Sub( point, centerB );

		float rnB = b2Cross( rB, normal );
		float kNormal = invMassA + invMassB + invIB * rnB * rnB;
		float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

		b2Vec2 vB = b2Body_GetLinearVelocity( bodyId );
		float omegaB = b2Body_GetAngularVelocity( bodyId );
		b2Vec2 vrB = b2Add( vB, b2CrossSV( omegaB, rB ) );
		float vn = b2Dot( b2Sub( vrB, m->velocity ), normal );

		// Push only, a separating body is left alone
		float impulse = b2MaxFloat( -normalMass * vn, 0.0f );

		b2Vec2 P = b2MulSV( impulse, normal );
		m->velocity = b2MulSub( m->velocity, invMassA, P );

		b2Body_ApplyLinearImpulse( bodyId, P, point, true );
	}
}

// samples/geometric_mover.cpp GeometricMover::Update
static void Scene_GeometricMoverUpdate( Scene* scene, SceneGeometricMover* m, float timeStep, float throttle )
{
	b2WorldId worldId = scene->worldId;

	// Friction
	float speed = b2Length( m->velocity );
	if ( speed < m->minSpeed )
	{
		m->velocity.x = 0.0f;
		m->velocity.y = 0.0f;
	}
	else if ( m->onGround )
	{
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		float control = speed < m->stopSpeed ? m->stopSpeed : speed;

		// friction has units of 1/time
		float drop = control * m->friction * timeStep;
		float newSpeed = b2MaxFloat( 0.0f, speed - drop );
		float ratio = newSpeed / speed;
		m->velocity.x *= ratio;
		m->velocity.y *= ratio;
	}

	b2Vec2 desiredVelocity = { m->maxSpeed * throttle, 0.0f };
	float desiredSpeed;
	b2Vec2 desiredDirection = b2GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > m->maxSpeed )
	{
		desiredSpeed = m->maxSpeed;
	}

	// Accelerate
	float currentSpeed = b2Dot( m->velocity, desiredDirection );
	float addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > 0.0f )
	{
		float steer = m->onGround ? 1.0f : m->airSteer;
		float accelSpeed = steer * m->accelerate * m->maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m->velocity.x += accelSpeed * desiredDirection.x;
		m->velocity.y += accelSpeed * desiredDirection.y;
	}

	// The probe reaches past the rest length so the spring can be stretched
	float rayLength = m->onGround ? 2.0f * m->pogoRestLength : m->pogoRestLength;
	b2Vec2 translation = { 0.0f, -rayLength };
	b2Pos origin = b2Add( m->position, m->capsule.center1 );

	SceneCastResult castResult;
	memset( &castResult, 0, sizeof( SceneCastResult ) );
	b2World_CastRay( worldId, origin, translation, m->filter, Scene_CastCallback, &castResult );

	m->pogoFraction = castResult.hit ? castResult.fraction : 1.0f;
	m->pogoHit = castResult.hit;

	// Avoid snapping to ground if still going up
	bool haveGround = castResult.hit && castResult.normal.y > m->minGroundNormalY;

	if ( m->onGround == false )
	{
		m->onGround = haveGround && m->velocity.y <= 0.01f;
	}
	else
	{
		m->onGround = haveGround;
	}

	if ( m->onGround == false )
	{
		m->velocity.y -= m->gravity * timeStep;
		m->pogoVelocity = 0.0f;
	}
	else
	{
		m->velocity.y = 0.0f;

		float pogoCurrentLength = castResult.fraction * rayLength;

		float offset = pogoCurrentLength - m->pogoRestLength;
		m->pogoVelocity = b2SpringDamper( m->pogoHertz, m->pogoDampingRatio, offset, m->pogoVelocity, timeStep );

		b2Body_ApplyForce( castResult.bodyId, (b2Vec2){ 0.0f, -m->pogoGroundForce }, castResult.point, true );
	}

	b2Vec2 velocityDelta = b2MulSV( timeStep, m->velocity );
	b2Vec2 pogoDelta = b2MulSV( timeStep * m->pogoVelocity, (b2Vec2){ 0.0f, 1.0f } );
	b2Pos target = b2Add( b2Add( m->position, velocityDelta ), pogoDelta );

	m->totalIterations = 0;
	float tolerance = 0.01f;

	// Don't cast against other movers.
	b2QueryFilter castFilter = { m->filter.categoryBits, ( m->filter.maskBits & ~m->filter.categoryBits ) };

	for ( int iteration = 0; iteration < 5; ++iteration )
	{
		m->planeCount = 0;

		b2World_CollideMover( worldId, m->position, &m->capsule, m->filter, Scene_PlaneResultFcn, m );
		b2Vec2 targetDelta = b2Sub( target, m->position );
		b2PlaneSolverResult result = b2SolvePlanes( targetDelta, m->planes, m->planeCount );

		m->totalIterations += result.iterationCount;

		float fraction = b2World_CastMover( worldId, m->position, &m->capsule, result.delta, castFilter );

		b2Vec2 delta = b2MulSV( fraction, result.delta );
		m->position = b2Add( m->position, delta );

		if ( b2LengthSquared( delta ) < tolerance * tolerance )
		{
			break;
		}
	}

	if ( m->pushBodies )
	{
		Scene_GeometricMoverPushBodies( m );
	}

	// Stop pushing into surfaces so speed can't accumulate against a wall
	m->velocity = b2ClipVector( m->velocity, m->planes, m->planeCount );
}

// mover: samples/sample_character.cpp GeometryMoverSample. keyBodies: 0 elevator
static void CreateMover( Scene* scene )
{
	Scene_CreateGeometricMover( &scene->geometricMover, (b2Pos){ 0.0f, 8.0f } );

	b2BodyId groundId1 = Scene_CreateLevelChain( scene, (b2Pos){ 0.0f, 0.0f }, s_geometricLevel1Bits,
												 (int)( sizeof( s_geometricLevel1Bits ) / 8 ) );
	b2BodyId groundId2 =
		Scene_CreateLevelChain( scene, (b2Pos){ 98.0f, 0.0f }, s_level2Bits, (int)( sizeof( s_level2Bits ) / 8 ) );

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 32.0f, 4.5f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter = (b2Filter){ s_moverBit, s_allBits, 0 };
		shapeDef.userData = (void*)(uintptr_t)1;
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2CreateCapsuleShape( bodyId, &shapeDef, &scene->geometricMover.capsule );
	}

	Scene_CreateCharacterProps( scene, groundId1, groundId2, 5.0f, true );

	scene->jumpReleased = true;
	scene->time = 0.0f;
}

static void StepMover( Scene* scene, int stepIndex )
{
	float timeStep = 1.0f / 60.0f;
	SceneGeometricMover* m = &scene->geometricMover;

	Scene_MoveElevator( scene, timeStep );

	Scene_WorldStep( scene );

	float throttle = Scene_MoverThrottle( stepIndex );
	if ( Scene_MoverJumpKey( stepIndex ) )
	{
		if ( scene->jumpReleased && Scene_GeometricMoverJump( m ) )
		{
			scene->jumpReleased = false;
		}
	}
	else
	{
		scene->jumpReleased = true;
	}

	Scene_GeometricMoverUpdate( scene, m, timeStep, throttle );

	Probe_Vec2( scene, m->position );
	Probe_Vec2( scene, m->velocity );
	Probe_Float( scene, m->pogoVelocity );
	Probe_Bool( scene, m->onGround );
	Probe_Bool( scene, m->pogoHit );
	Probe_Float( scene, m->pogoFraction );
	Probe_Int( scene, m->totalIterations );
	Probe_Int( scene, m->planeCount );
	for ( int i = 0; i < m->planeCount; ++i )
	{
		b2CollisionPlane plane = m->planes[i];
		Probe_Vec2( scene, plane.plane.normal );
		Probe_Float( scene, plane.plane.offset );
		Probe_Float( scene, plane.pushLimit );
		Probe_Float( scene, plane.push );
		Probe_Bool( scene, plane.clipVelocity );
		Probe_Vec2( scene, m->planePoints[i] );
		Probe_ShapeId( scene, m->planeShapes[i] );
	}
}

// shared/benchmarks.c CreateSmallPyramid
static void Scene_CreateSmallPyramid( Scene* scene, int baseCount, float extent, float centerX, float baseY )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2Polygon box = b2MakeSquare( extent );

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * (float)i + 1.0f ) * extent + baseY;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( (float)i + 1.0f ) * extent + 2.0f * (float)( j - i ) * extent + centerX - 0.5f;
			bodyDef.position = (b2Pos){ x, y };

			b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}
}

// churn: two pyramids, a three shape body and a jointed pair under a script that creates, destroys,
// sleeps, wakes, disables, retypes, teleports and pushes bodies.
// keyBodies: 0 left pyramid base, 1 right pyramid top, 2 compound, 3 and 4 jointed pair, 5 right
// pyramid base. keyShapes 0..2: compound shapes. keyJoints 0: the pair's revolute.
static void CreateChurn( Scene* scene )
{
	scene->seed = 104729;

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = Scene_AddBody( scene, &groundDef );
	scene->groundId = groundId;

	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon floor = b2MakeOffsetBox( 20.0f, 0.5f, (b2Vec2){ 0.0f, -0.5f }, b2Rot_identity );
		b2CreatePolygonShape( groundId, &shapeDef, &floor );

		b2Segment segment = { { -20.0f, 0.0f }, { -20.0f, 15.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		segment = (b2Segment){ { 20.0f, 0.0f }, { 20.0f, 15.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	int first = scene->bodyCount;
	Scene_CreateSmallPyramid( scene, 4, 0.5f, -12.0f, 0.0f );
	scene->keyBodies[0] = scene->bodies[first];

	first = scene->bodyCount;
	Scene_CreateSmallPyramid( scene, 4, 0.5f, 8.0f, 0.0f );
	scene->keyBodies[5] = scene->bodies[first];
	scene->keyBodies[1] = scene->bodies[scene->bodyCount - 1];

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0f, 2.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[2] = bodyId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		scene->keyShapes[0] = b2CreatePolygonShape( bodyId, &shapeDef, &box );

		shapeDef.density = 3.0f;
		b2Circle circle = { { 0.8f, 0.0f }, 0.4f };
		scene->keyShapes[1] = b2CreateCircleShape( bodyId, &shapeDef, &circle );

		b2Capsule capsule = { { -0.9f, -0.2f }, { -0.9f, 0.2f }, 0.2f };
		scene->keyShapes[2] = b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -3.0f, 6.0f };
		b2BodyId bodyIdA = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[3] = bodyIdA;

		bodyDef.position = (b2Pos){ -2.0f, 6.0f };
		b2BodyId bodyIdB = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[4] = bodyIdB;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 0.5f, 0.2f );
		b2CreatePolygonShape( bodyIdA, &shapeDef, &box );
		b2CreatePolygonShape( bodyIdB, &shapeDef, &box );

		b2Pos pivot = { -2.5f, 6.0f };
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = bodyIdA;
		jointDef.base.bodyIdB = bodyIdB;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( bodyIdA, pivot );
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint( bodyIdB, pivot );
		scene->keyJoints[0] = b2CreateRevoluteJoint( scene->worldId, &jointDef );
	}
}

static void Scene_SpawnChurnBody( Scene* scene )
{
	int kind = scene->spawnCount % 3;
	float x = Scene_RandomFloat( scene, -15.0f, 15.0f );
	float angle = Scene_RandomFloat( scene, -B2_PI, B2_PI );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ x, 14.0f };
	bodyDef.rotation = b2MakeRot( angle );
	b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
	Scene_AddSpawn( scene, bodyId );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	if ( kind == 0 )
	{
		b2Polygon box = b2MakeSquare( 0.4f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}
	else if ( kind == 1 )
	{
		b2Circle circle = { { 0.0f, 0.0f }, 0.4f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}
	else
	{
		b2Capsule capsule = { { -0.3f, 0.0f }, { 0.3f, 0.0f }, 0.25f };
		b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}
}

static void StepChurn( Scene* scene, int stepIndex )
{
	b2BodyId* keys = scene->keyBodies;

	if ( stepIndex % 25 == 0 && stepIndex < 450 )
	{
		Scene_SpawnChurnBody( scene );
	}

	if ( stepIndex >= 60 && stepIndex % 40 == 20 && scene->spawnHead < scene->spawnCount )
	{
		b2DestroyBody( scene->spawned[scene->spawnHead] );
		scene->spawnHead += 1;
	}

	if ( stepIndex == 120 )
	{
		b2DestroyShape( scene->keyShapes[1], true );
	}
	else if ( stepIndex == 130 )
	{
		b2Body_Disable( keys[1] );
	}
	else if ( stepIndex == 150 )
	{
		b2Body_SetAwake( keys[0], false );
	}
	else if ( stepIndex == 160 )
	{
		b2Body_SetType( keys[2], b2_staticBody );
	}
	else if ( stepIndex == 180 )
	{
		b2DestroyJoint( scene->keyJoints[0] );
	}
	else if ( stepIndex == 200 )
	{
		b2Body_Enable( keys[1] );
	}
	else if ( stepIndex == 230 )
	{
		b2Body_SetAwake( keys[0], true );
	}
	else if ( stepIndex == 240 )
	{
		b2Body_SetType( keys[2], b2_kinematicBody );
		b2Body_SetLinearVelocity( keys[2], (b2Vec2){ 1.0f, 0.0f } );
		b2Body_SetAngularVelocity( keys[2], 0.5f );
	}
	else if ( stepIndex == 260 )
	{
		b2Body_SetTransform( keys[5], (b2Pos){ 0.0f, 10.0f }, b2MakeRot( 0.7f ) );
	}
	else if ( stepIndex == 300 )
	{
		b2Body_SetType( keys[2], b2_dynamicBody );
	}
	else if ( stepIndex == 320 )
	{
		b2Pos point = b2Body_GetWorldPoint( keys[0], (b2Vec2){ 0.5f, 0.5f } );
		b2Body_ApplyLinearImpulse( keys[0], (b2Vec2){ 0.0f, 5.0f }, point, false );
	}
	else if ( stepIndex == 330 )
	{
		b2Body_ApplyLinearImpulseToCenter( keys[3], (b2Vec2){ 3.0f, 6.0f }, true );
	}
	else if ( stepIndex == 340 )
	{
		b2Body_ApplyAngularImpulse( keys[4], 2.0f, true );
	}
	else if ( stepIndex == 380 )
	{
		b2Body_WakeTouching( scene->groundId );
	}
	else if ( stepIndex == 390 )
	{
		b2Body_EnableSleeping( keys[2], false );
	}
	else if ( stepIndex == 400 )
	{
		b2Body_SetLinearVelocity( keys[2], (b2Vec2){ -2.0f, 4.0f } );
		b2Body_SetAngularVelocity( keys[2], 3.0f );
	}
	else if ( stepIndex == 420 )
	{
		b2Body_SetGravityScale( keys[3], -0.5f );
	}
	else if ( stepIndex == 450 )
	{
		b2Body_SetGravityScale( keys[3], 1.0f );
	}
	else if ( stepIndex == 470 )
	{
		b2Body_EnableSleeping( keys[2], true );
	}
	else if ( stepIndex == 480 )
	{
		b2World_EnableSleeping( scene->worldId, false );
	}
	else if ( stepIndex == 520 )
	{
		b2World_EnableSleeping( scene->worldId, true );
	}

	if ( 350 <= stepIndex && stepIndex < 370 )
	{
		b2Pos point = b2Body_GetWorldPoint( keys[2], (b2Vec2){ 0.5f, 0.0f } );
		b2Body_ApplyForce( keys[2], (b2Vec2){ 0.0f, 30.0f }, point, true );
		b2Body_ApplyTorque( keys[4], 5.0f, true );
		b2Body_ApplyForceToCenter( keys[3], (b2Vec2){ 20.0f, 0.0f }, true );
	}
	if ( stepIndex == 369 )
	{
		b2Body_ClearForces( keys[3] );
	}

	Scene_WorldStep( scene );

	Probe_Float( scene, b2Body_GetMass( keys[2] ) );
	Probe_Float( scene, b2Body_GetRotationalInertia( keys[2] ) );
	Probe_Vec2( scene, b2Body_GetLocalCenter( keys[2] ) );
	Probe_Int( scene, b2Body_GetShapeCount( keys[2] ) );
	Probe_Int( scene, b2Body_GetJointCount( keys[3] ) );
	Probe_Bool( scene, b2Body_IsEnabled( keys[1] ) );
	Probe_Bool( scene, b2Body_IsSleepingEnabled( keys[2] ) );
	Probe_Bool( scene, b2World_IsSleepingEnabled( scene->worldId ) );
	Probe_Int( scene, scene->spawnCount );
	Probe_Int( scene, scene->spawnHead );
}

// bullets: samples/sample_stacking.cpp VerticalStack with its bullets fired by script, thin wall
// projectiles from samples/sample_continuous.cpp, and a resting box turned into a bullet.
// keyBodies: 0 current bullet, 1 dart
static void CreateBullets( Scene* scene )
{
	scene->seed = 2718;

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 0.0f, 0.0f };
		b2BodyId groundId = Scene_AddBody( scene, &bodyDef );
		scene->groundId = groundId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		b2Segment segment = { { 10.0f, 0.0f }, { 10.0f, 80.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		segment = (b2Segment){ { -100.0f, 0.0f }, { 100.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	{
		b2Polygon box = b2MakeRoundedBox( 0.45f, 0.45f, 0.05f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.3f;

		float offset = 0.01f;
		float x = 8.0f;

		for ( int i = 0; i < 12; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			float shift = ( i % 2 == 0 ? -offset : offset );
			bodyDef.position = (b2Pos){ x + shift, 0.5f + 1.0f * (float)i };
			b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ -60.0f, 5.0f };
		b2BodyId wallId = Scene_AddBody( scene, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 0.05f, 5.0f );
		b2CreatePolygonShape( wallId, &shapeDef, &box );
	}

	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableHitEvents = true;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		bodyDef.position = (b2Pos){ -90.0f, 2.0f };
		bodyDef.linearVelocity = (b2Vec2){ 150.0f, 0.0f };
		b2BodyId circleId = Scene_AddBody( scene, &bodyDef );
		b2Circle circle = { { 0.0f, 0.0f }, 0.25f };
		b2CreateCircleShape( circleId, &shapeDef, &circle );

		bodyDef.position = (b2Pos){ -90.0f, 5.0f };
		bodyDef.linearVelocity = (b2Vec2){ 120.0f, 0.0f };
		bodyDef.angularVelocity = 50.0f;
		b2BodyId spinnerId = Scene_AddBody( scene, &bodyDef );
		b2Polygon thinBox = b2MakeBox( 0.05f, 0.6f );
		b2CreatePolygonShape( spinnerId, &shapeDef, &thinBox );

		bodyDef.position = (b2Pos){ -90.0f, 8.0f };
		bodyDef.linearVelocity = (b2Vec2){ 180.0f, 0.0f };
		bodyDef.angularVelocity = 0.0f;
		b2BodyId capsuleId = Scene_AddBody( scene, &bodyDef );
		b2Capsule capsule = { { -0.3f, 0.0f }, { 0.3f, 0.0f }, 0.1f };
		b2CreateCapsuleShape( capsuleId, &shapeDef, &capsule );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -40.0f, 0.2f };
		b2BodyId dartId = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[1] = dartId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 4.0f;
		shapeDef.enableHitEvents = true;
		b2Polygon box = b2MakeBox( 0.3f, 0.15f );
		b2CreatePolygonShape( dartId, &shapeDef, &box );
	}

	scene->keyBodies[0] = b2_nullBodyId;
}

// samples/sample_stacking.cpp VerticalStack::FireBullets for one bullet, destroying the previous
// one first as Launch does
static void Scene_FireBullet( Scene* scene, bool isBox, float y )
{
	if ( B2_IS_NON_NULL( scene->keyBodies[0] ) )
	{
		b2DestroyBody( scene->keyBodies[0] );
		scene->keyBodies[0] = b2_nullBodyId;
	}

	b2Circle circle = { { 0.0f, 0.0f }, 0.25f };
	b2Polygon box = b2MakeBox( 0.25f, 0.25f );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 4.0f;
	shapeDef.enableHitEvents = true;
	shapeDef.enableContactEvents = true;

	float speed = Scene_RandomFloat( scene, 200.0f, 300.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ -26.7f, y };
	bodyDef.linearVelocity = (b2Vec2){ speed, 0.0f };
	bodyDef.isBullet = true;

	b2BodyId bullet = Scene_AddBody( scene, &bodyDef );

	if ( isBox )
	{
		b2CreatePolygonShape( bullet, &shapeDef, &box );
	}
	else
	{
		b2CreateCircleShape( bullet, &shapeDef, &circle );
	}

	scene->keyBodies[0] = bullet;

	Probe_BodyId( scene, bullet );
	Probe_Float( scene, speed );
}

static void StepBullets( Scene* scene, int stepIndex )
{
	if ( stepIndex == 30 )
	{
		Scene_FireBullet( scene, false, 6.0f );
	}
	else if ( stepIndex == 90 )
	{
		Scene_FireBullet( scene, true, 6.0f );
	}
	else if ( stepIndex == 150 )
	{
		Scene_FireBullet( scene, false, 2.5f );
	}
	else if ( stepIndex == 200 )
	{
		b2Body_SetBullet( scene->keyBodies[1], true );
		b2Body_SetLinearVelocity( scene->keyBodies[1], (b2Vec2){ 250.0f, 0.0f } );
	}

	Scene_WorldStep( scene );

	Probe_Bool( scene, b2Body_IsBullet( scene->keyBodies[1] ) );
	Probe_ContactEvents( scene );
}

// sensors: samples/sample_events.cpp sensor setups: a static sensor zone, a sensor on a sweeping
// kinematic body and a sensor ring on a rolling dynamic ball, with visitors dropped through them.
// keyShapes: 0 static zone, 1 kinematic sweeper, 2 ball ring. keyBodies: 0 sweeper.
static void CreateSensors( Scene* scene )
{
	scene->seed = 31337;

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = Scene_AddBody( scene, &groundDef );
	scene->groundId = groundId;

	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		segment = (b2Segment){ { -20.0f, 0.0f }, { -20.0f, 20.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		segment = (b2Segment){ { 20.0f, 0.0f }, { 20.0f, 20.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ -5.0f, 7.0f };
		b2BodyId zoneId = Scene_AddBody( scene, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		b2Polygon box = b2MakeBox( 3.0f, 1.5f );
		scene->keyShapes[0] = b2CreatePolygonShape( zoneId, &shapeDef, &box );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = (b2Pos){ -15.0f, 1.0f };
		bodyDef.linearVelocity = (b2Vec2){ 3.0f, 0.0f };
		b2BodyId sweeperId = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[0] = sweeperId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		b2Polygon box = b2MakeBox( 1.0f, 1.0f );
		scene->keyShapes[1] = b2CreatePolygonShape( sweeperId, &shapeDef, &box );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 12.0f, 0.5f };
		bodyDef.linearVelocity = (b2Vec2){ -4.0f, 0.0f };
		bodyDef.angularVelocity = 8.0f;
		b2BodyId ballId = Scene_AddBody( scene, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = 0.5f;
		shapeDef.material.rollingResistance = 0.02f;
		shapeDef.enableSensorEvents = true;
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		b2CreateCircleShape( ballId, &shapeDef, &circle );

		b2ShapeDef sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		b2Circle ring = { { 0.0f, 0.0f }, 2.0f };
		scene->keyShapes[2] = b2CreateCircleShape( ballId, &sensorDef, &ring );
	}
}

static void Scene_SpawnVisitor( Scene* scene )
{
	int index = scene->spawnCount;
	float x = Scene_RandomFloat( scene, -9.0f, 3.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ x, 12.0f };
	b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
	Scene_AddSpawn( scene, bodyId );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enableSensorEvents = index != 4;
	if ( index % 2 == 0 )
	{
		b2Circle circle = { { 0.0f, 0.0f }, 0.3f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}
	else
	{
		b2Polygon box = b2MakeSquare( 0.3f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}
}

static void StepSensors( Scene* scene, int stepIndex )
{
	if ( stepIndex < 240 && stepIndex % 12 == 0 )
	{
		Scene_SpawnVisitor( scene );
	}

	if ( stepIndex % 300 == 299 )
	{
		b2Vec2 v = b2Body_GetLinearVelocity( scene->keyBodies[0] );
		b2Body_SetLinearVelocity( scene->keyBodies[0], b2Neg( v ) );
	}

	if ( stepIndex == 400 )
	{
		b2DestroyBody( scene->spawned[0] );
		b2DestroyBody( scene->spawned[1] );
	}
	else if ( stepIndex == 420 )
	{
		b2ShapeId shapeId;
		int count = b2Body_GetShapes( scene->spawned[2], &shapeId, 1 );
		if ( count == 1 )
		{
			b2Shape_EnableSensorEvents( shapeId, false );
		}
	}

	Scene_WorldStep( scene );

	Probe_SensorEvents( scene );
	for ( int i = 0; i < 3; ++i )
	{
		b2ShapeId visitorIds[SCENE_HIT_CAPACITY];
		Probe_Int( scene, b2Shape_GetSensorCapacity( scene->keyShapes[i] ) );
		int count = b2Shape_GetSensorData( scene->keyShapes[i], visitorIds, SCENE_HIT_CAPACITY );
		Probe_Int( scene, count );
		for ( int j = 0; j < count; ++j )
		{
			Probe_ShapeId( scene, visitorIds[j] );
		}
	}
}

// events: samples/sample_events.cpp ContactEvent and BodyMove style drops with contact, hit and
// move events, a spinning kinematic paddle, and a script that changes event flags.
// keyShapes: 0 first dropped shape. keyBodies: 0 paddle. spawned: the dropped bodies.
static void CreateEvents( Scene* scene )
{
	scene->seed = 4242;

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 0.0f, -1.0f };
		b2BodyId groundId = Scene_AddBody( scene, &bodyDef );
		scene->groundId = groundId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 20.0f, 1.0f );
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		b2Segment segment = { { -14.0f, 6.0f }, { -6.0f, 1.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = (b2Pos){ 9.0f, 2.0f };
		bodyDef.angularVelocity = 2.0f;
		b2BodyId paddleId = Scene_AddBody( scene, &bodyDef );
		scene->keyBodies[0] = paddleId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		b2Polygon box = b2MakeBox( 2.0f, 0.2f );
		b2CreatePolygonShape( paddleId, &shapeDef, &box );
	}

	for ( int i = 0; i < 16; ++i )
	{
		int column = i % 4;
		int row = i / 4;
		float jitter = Scene_RandomFloat( scene, 0.0f, 1.0f );
		float restitution = Scene_RandomFloat( scene, 0.2f, 0.7f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -9.0f + 5.0f * (float)column, 3.0f + 2.5f * (float)row + jitter };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		Scene_AddSpawn( scene, bodyId );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = restitution;
		shapeDef.enableContactEvents = true;
		shapeDef.enableHitEvents = i % 2 == 0;

		b2ShapeId shapeId;
		if ( i % 2 == 0 )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.4f };
			shapeId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
		else
		{
			b2Polygon box = b2MakeBox( 0.4f, 0.3f );
			shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		if ( i == 0 )
		{
			scene->keyShapes[0] = shapeId;
		}
	}
}

static void StepEvents( Scene* scene, int stepIndex )
{
	b2BodyId* spawned = scene->spawned;

	if ( stepIndex == 150 )
	{
		for ( int i = 0; i < 4; ++i )
		{
			b2Body_ApplyLinearImpulseToCenter( spawned[i], (b2Vec2){ 0.5f, 6.0f }, true );
		}
	}
	else if ( stepIndex == 200 )
	{
		b2World_SetHitEventThreshold( scene->worldId, 2.0f );
	}
	else if ( stepIndex == 220 )
	{
		b2DestroyBody( spawned[5] );
	}
	else if ( stepIndex == 250 )
	{
		b2Body_SetAngularVelocity( scene->keyBodies[0], 0.0f );
	}
	else if ( stepIndex == 260 )
	{
		b2ShapeId shapeId;
		int count = b2Body_GetShapes( spawned[6], &shapeId, 1 );
		if ( count == 1 )
		{
			b2Shape_EnableContactEvents( shapeId, false );
		}
		b2Body_EnableHitEvents( spawned[8], false );
	}
	else if ( stepIndex == 300 )
	{
		for ( int i = 8; i < 12; ++i )
		{
			b2Body_ApplyLinearImpulseToCenter( spawned[i], (b2Vec2){ -1.0f, 8.0f }, true );
		}
	}

	Scene_WorldStep( scene );

	Probe_ContactEvents( scene );

	// Manifold summary of each begin contact and validity of each end event's shapes
	b2ContactEvents events = b2World_GetContactEvents( scene->worldId );
	for ( int i = 0; i < events.beginCount; ++i )
	{
		b2ContactId contactId = events.beginEvents[i].contactId;
		bool valid = b2Contact_IsValid( contactId );
		Probe_Bool( scene, valid );
		if ( valid )
		{
			b2ContactData data = b2Contact_GetData( contactId );
			Probe_Int( scene, data.manifold.pointCount );
			Probe_Vec2( scene, data.manifold.normal );
		}
	}
	for ( int i = 0; i < events.endCount; ++i )
	{
		Probe_Bool( scene, b2Shape_IsValid( events.endEvents[i].shapeIdA ) );
		Probe_Bool( scene, b2Shape_IsValid( events.endEvents[i].shapeIdB ) );
	}

	Probe_BodyMoveEvents( scene );

	b2ContactData contactData[SCENE_CONTACT_CAPACITY];
	Probe_Int( scene, b2Shape_GetContactCapacity( scene->keyShapes[0] ) );
	int count = b2Shape_GetContactData( scene->keyShapes[0], contactData, SCENE_CONTACT_CAPACITY );
	Probe_Int( scene, count );
	for ( int i = 0; i < count; ++i )
	{
		Probe_ContactData( scene, contactData[i] );
	}
}

// Query results gathered by the callbacks below, in call order
typedef struct SceneHits
{
	b2ShapeId shapeIds[SCENE_HIT_CAPACITY];
	b2Pos points[SCENE_HIT_CAPACITY];
	b2Vec2 normals[SCENE_HIT_CAPACITY];
	float fractions[SCENE_HIT_CAPACITY];
	int count;
	int calls;
} SceneHits;

static bool Scene_OverlapCallback( b2ShapeId shapeId, void* context )
{
	SceneHits* hits = context;
	hits->calls += 1;
	if ( hits->count < SCENE_HIT_CAPACITY )
	{
		hits->shapeIds[hits->count] = shapeId;
		hits->count += 1;
	}
	return true;
}

// Gathers every hit and lets the ray continue
static float Scene_RayAllCallback( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	SceneHits* hits = context;
	hits->calls += 1;
	if ( hits->count < SCENE_HIT_CAPACITY )
	{
		int i = hits->count;
		hits->shapeIds[i] = shapeId;
		hits->points[i] = point;
		hits->normals[i] = normal;
		hits->fractions[i] = fraction;
		hits->count += 1;
	}
	return 1.0f;
}

// Keeps the latest hit in slot 0 and clips the cast to it, so slot 0 ends as the closest
static float Scene_CastClosestCallback( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	SceneHits* hits = context;
	hits->calls += 1;
	hits->shapeIds[0] = shapeId;
	hits->points[0] = point;
	hits->normals[0] = normal;
	hits->fractions[0] = fraction;
	hits->count = 1;
	return fraction;
}

// Shapes tagged 7 that opted into custom filtering pass through each other
static bool Scene_CustomFilter( b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context )
{
	(void)context;
	uint64_t tagA = (uint64_t)(uintptr_t)b2Shape_GetUserData( shapeIdA );
	uint64_t tagB = (uint64_t)(uintptr_t)b2Shape_GetUserData( shapeIdB );
	if ( tagA == 7 && tagB == 7 )
	{
		return false;
	}
	return true;
}

static void Probe_Hits( Scene* scene, const SceneHits* hits, bool withCast )
{
	Probe_Int( scene, hits->calls );
	Probe_Int( scene, hits->count );
	for ( int i = 0; i < hits->count; ++i )
	{
		Probe_ShapeId( scene, hits->shapeIds[i] );
		if ( withCast )
		{
			Probe_Vec2( scene, hits->points[i] );
			Probe_Vec2( scene, hits->normals[i] );
			Probe_Float( scene, hits->fractions[i] );
		}
	}
}

static void Probe_CastOutput( Scene* scene, b2CastOutput output )
{
	Probe_Bool( scene, output.hit );
	if ( output.hit )
	{
		Probe_Vec2( scene, output.point );
		Probe_Vec2( scene, output.normal );
		Probe_Float( scene, output.fraction );
	}
}

static void Probe_AABB( Scene* scene, b2AABB aabb )
{
	Probe_Vec2( scene, aabb.lowerBound );
	Probe_Vec2( scene, aabb.upperBound );
}

// queries: static shapes of every kind on their own bodies, a rain of dynamic bodies, some custom
// filtered, explosions, and every world, shape and body query each step.
// keyShapes: 0 static box, 1 first dynamic shape. keyBodies: 0 first dynamic body.
static void CreateQueries( Scene* scene )
{
	scene->seed = 1234567;
	b2WorldId worldId = scene->worldId;

	b2World_SetCustomFilterCallback( worldId, Scene_CustomFilter, scene );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2BodyDef bodyDef = b2DefaultBodyDef();

	{
		b2BodyId groundId = Scene_AddBody( scene, &bodyDef );
		scene->groundId = groundId;
		b2Segment segment = { { -25.0f, 0.0f }, { 25.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	{
		bodyDef.position = (b2Pos){ -15.0f, 3.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}

	{
		bodyDef.position = (b2Pos){ -8.0f, 2.0f };
		bodyDef.rotation = b2MakeRot( 0.4f );
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f };
		b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	}

	{
		bodyDef.position = (b2Pos){ -2.0f, 4.0f };
		bodyDef.rotation = b2MakeRot( 0.3f );
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Polygon box = b2MakeBox( 1.0f, 1.0f );
		scene->keyShapes[0] = b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}

	{
		bodyDef.position = (b2Pos){ 5.0f, 3.0f };
		bodyDef.rotation = b2Rot_identity;
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Vec2 points[5] = { { -1.0f, -0.5f }, { 1.0f, -0.7f }, { 1.2f, 0.4f }, { 0.0f, 1.0f }, { -0.9f, 0.6f } };
		b2Hull hull = b2ComputeHull( points, 5 );
		b2Polygon polygon = b2MakePolygon( &hull, 0.2f );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
	}

	{
		bodyDef.position = (b2Pos){ 0.0f, 0.0f };
		b2BodyId bodyId = Scene_AddBody( scene, &bodyDef );
		b2Segment segment = { { 10.0f, 1.0f }, { 14.0f, 4.0f } };
		b2CreateSegmentShape( bodyId, &shapeDef, &segment );

		b2Vec2 points[4] = { { 22.0f, 2.0f }, { 20.0f, 1.0f }, { 18.0f, 1.5f }, { 16.0f, 0.5f } };
		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 4;
		chainDef.ghost1 = (b2Vec2){ 24.0f, 2.0f };
		chainDef.ghost2 = (b2Vec2){ 14.0f, 0.5f };
		b2CreateChain( bodyId, &chainDef );
	}

	for ( int i = 0; i < 12; ++i )
	{
		float x = Scene_RandomFloat( scene, -20.0f, 20.0f );

		b2BodyDef dynamicDef = b2DefaultBodyDef();
		dynamicDef.type = b2_dynamicBody;
		dynamicDef.position = (b2Pos){ x, 8.0f + 1.5f * (float)i };
		b2BodyId bodyId = Scene_AddBody( scene, &dynamicDef );

		b2ShapeDef dynamicShapeDef = b2DefaultShapeDef();
		if ( i % 3 == 0 )
		{
			dynamicShapeDef.enableCustomFiltering = true;
			dynamicShapeDef.userData = (void*)(uintptr_t)7;
		}

		b2ShapeId shapeId;
		if ( i % 2 == 0 )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			shapeId = b2CreateCircleShape( bodyId, &dynamicShapeDef, &circle );
		}
		else
		{
			b2Polygon box = b2MakeBox( 0.5f, 0.4f );
			shapeId = b2CreatePolygonShape( bodyId, &dynamicShapeDef, &box );
		}

		if ( i == 0 )
		{
			scene->keyBodies[0] = bodyId;
			scene->keyShapes[1] = shapeId;
		}
	}
}

static void StepQueries( Scene* scene, int stepIndex )
{
	b2WorldId worldId = scene->worldId;

	if ( stepIndex == 120 || stepIndex == 240 )
	{
		b2ExplosionDef def = b2DefaultExplosionDef();
		def.position = (b2Pos){ 0.0f, 1.0f };
		def.radius = 4.0f;
		def.falloff = 2.0f;
		def.impulsePerLength = 4.0f;
		b2World_Explode( worldId, &def );
	}

	Scene_WorldStep( scene );

	float t = (float)( stepIndex % 200 );
	b2QueryFilter filter = b2DefaultQueryFilter();
	SceneHits hits;

	{
		float cx = -20.0f + 0.2f * t;
		b2AABB aabb = { { cx - 2.0f, 0.0f }, { cx + 2.0f, 4.0f } };
		memset( &hits, 0, sizeof( SceneHits ) );
		b2World_OverlapAABB( worldId, b2Pos_zero, aabb, filter, Scene_OverlapCallback, &hits );
		Probe_Hits( scene, &hits, false );
	}

	{
		b2Vec2 points[3] = { { -1.0f, -0.5f }, { 1.0f, -0.5f }, { 0.0f, 1.0f } };
		b2ShapeProxy proxy = b2MakeProxy( points, 3, 0.1f );
		b2Pos origin = { 20.0f - 0.2f * t, 3.0f };
		memset( &hits, 0, sizeof( SceneHits ) );
		b2World_OverlapShape( worldId, origin, &proxy, filter, Scene_OverlapCallback, &hits );
		Probe_Hits( scene, &hits, false );
	}

	{
		b2Pos origin = { -24.0f, 0.5f + 0.03f * t };
		b2Vec2 translation = { 48.0f, 4.0f };
		memset( &hits, 0, sizeof( SceneHits ) );
		b2World_CastRay( worldId, origin, translation, filter, Scene_RayAllCallback, &hits );
		Probe_Hits( scene, &hits, true );
	}

	{
		b2Pos origin = { -20.0f + 0.2f * t, 15.0f };
		b2Vec2 translation = { 0.0f, -20.0f };
		b2RayResult result = b2World_CastRayClosest( worldId, origin, translation, filter );
		Probe_Bool( scene, result.hit );
		if ( result.hit )
		{
			Probe_ShapeId( scene, result.shapeId );
			Probe_Vec2( scene, result.point );
			Probe_Vec2( scene, result.normal );
			Probe_Float( scene, result.fraction );
		}
	}

	{
		b2Vec2 center = { 0.0f, 0.0f };
		b2ShapeProxy proxy = b2MakeProxy( &center, 1, 0.5f );
		b2Pos origin = { -24.0f, 6.0f - 0.025f * t };
		b2Vec2 translation = { 48.0f, -2.0f };
		memset( &hits, 0, sizeof( SceneHits ) );
		b2World_CastShape( worldId, origin, &proxy, translation, filter, Scene_CastClosestCallback, &hits );
		Probe_Hits( scene, &hits, true );
	}

	{
		b2Pos origin = { -2.0f, 8.0f };
		b2Vec2 translation = { 0.02f * t - 2.0f, -8.0f };
		Probe_CastOutput( scene, b2Shape_CastRay( scene->keyShapes[0], origin, translation ) );

		b2Pos target = b2Body_GetPosition( scene->keyBodies[0] );
		b2Pos origin2 = { target.x - 3.0f, target.y + 1.0f };
		b2Vec2 translation2 = { 6.0f, -2.0f };
		Probe_CastOutput( scene, b2Shape_CastRay( scene->keyShapes[1], origin2, translation2 ) );
	}

	{
		b2Pos point = { 0.1f * t - 10.0f, 6.0f };
		Probe_Bool( scene, b2Shape_TestPoint( scene->keyShapes[0], point ) );
		Probe_Vec2( scene, b2Shape_GetClosestPoint( scene->keyShapes[0], point ) );
		Probe_Bool( scene, b2Shape_TestPoint( scene->keyShapes[1], b2Body_GetPosition( scene->keyBodies[0] ) ) );
	}

	{
		b2BodyId bodyId = scene->keyBodies[0];
		Probe_AABB( scene, b2Body_ComputeBounds( bodyId ) );
		Probe_AABB( scene, b2Shape_GetBounds( scene->keyShapes[1] ) );
		Probe_AABB( scene, b2World_GetBounds( worldId ) );
		Probe_Vec2( scene, b2Body_GetWorldPoint( bodyId, (b2Vec2){ 0.5f, 0.0f } ) );
		Probe_Vec2( scene, b2Body_GetLocalPoint( bodyId, (b2Pos){ 0.0f, 0.0f } ) );
		Probe_Vec2( scene, b2Body_GetWorldVector( bodyId, (b2Vec2){ 1.0f, 0.0f } ) );
		Probe_Vec2( scene, b2Body_GetLocalVector( bodyId, (b2Vec2){ 0.0f, 1.0f } ) );
		Probe_Vec2( scene, b2Body_GetLocalPointVelocity( bodyId, (b2Vec2){ 0.5f, 0.0f } ) );
		Probe_Vec2( scene, b2Body_GetWorldPointVelocity( bodyId, (b2Pos){ 0.0f, 0.0f } ) );
		Probe_Vec2( scene, b2Body_GetWorldCenter( bodyId ) );
	}
}

// many_bodies: shared/benchmarks.c CreateManyPyramids at 10 x 10 pyramids
static void CreateManyBodies( Scene* scene )
{
	b2World_EnableSleeping( scene->worldId, false );

	int baseCount = 10;
	float extent = 0.5f;
	int rowCount = 10;
	int columnCount = 10;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = Scene_AddBody( scene, &bodyDef );

	float groundDeltaY = 2.0f * extent * ( (float)baseCount + 1.0f );
	float groundWidth = 2.0f * extent * (float)columnCount * ( (float)baseCount + 1.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	float groundY = 0.0f;

	for ( int i = 0; i < rowCount; ++i )
	{
		b2Segment segment = { { -0.5f * groundWidth, groundY }, { 0.5f * groundWidth, groundY } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
		groundY += groundDeltaY;
	}

	float baseWidth = 2.0f * extent * (float)baseCount;
	float baseY = 0.0f;

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			float centerX = -0.5f * groundWidth + (float)j * ( baseWidth + 2.0f * extent ) + 2.0f * extent;
			Scene_CreateSmallPyramid( scene, baseCount, extent, centerX, baseY );
		}

		baseY += groundDeltaY;
	}
}

bool Scene_Create( Scene* scene, const char* name, int workerCount )
{
	memset( scene, 0, sizeof( Scene ) );

	int index = -1;
	for ( int i = 0; i < Scene_GetCount(); ++i )
	{
		if ( strcmp( name, s_sceneNames[i] ) == 0 )
		{
			index = i;
		}
	}
	if ( index < 0 )
	{
		return false;
	}

	scene->name = s_sceneNames[index];
	scene->index = index;

	// Without task callbacks one worker runs serially and more use C's internal scheduler
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	scene->worldId = b2CreateWorld( &worldDef );

	switch ( index )
	{
		case SCENE_PYRAMID:
			CreatePyramid( scene );
			break;
		case SCENE_SHAPE_RAIN:
			CreateShapeRain( scene );
			break;
		case SCENE_CHAIN_CAR:
			CreateChainCar( scene );
			break;
		case SCENE_JOINTS:
			CreateJoints( scene );
			break;
		case SCENE_DYNAMIC_MOVER:
			CreateDynamicMover( scene );
			break;
		case SCENE_CHURN:
			CreateChurn( scene );
			break;
		case SCENE_BULLETS:
			CreateBullets( scene );
			break;
		case SCENE_SENSORS:
			CreateSensors( scene );
			break;
		case SCENE_EVENTS:
			CreateEvents( scene );
			break;
		case SCENE_QUERIES:
			CreateQueries( scene );
			break;
		case SCENE_MOVER:
			CreateMover( scene );
			break;
		case SCENE_MANY_BODIES:
			CreateManyBodies( scene );
			break;
		default:
			break;
	}

	return true;
}

void Scene_Step( Scene* scene, int stepIndex )
{
	scene->probeCount = 0;

	switch ( scene->index )
	{
		case SCENE_SHAPE_RAIN:
			StepShapeRain( scene, stepIndex );
			break;
		case SCENE_CHAIN_CAR:
			StepChainCar( scene, stepIndex );
			break;
		case SCENE_JOINTS:
			StepJoints( scene, stepIndex );
			break;
		case SCENE_DYNAMIC_MOVER:
			StepDynamicMover( scene, stepIndex );
			break;
		case SCENE_CHURN:
			StepChurn( scene, stepIndex );
			break;
		case SCENE_BULLETS:
			StepBullets( scene, stepIndex );
			break;
		case SCENE_SENSORS:
			StepSensors( scene, stepIndex );
			break;
		case SCENE_EVENTS:
			StepEvents( scene, stepIndex );
			break;
		case SCENE_QUERIES:
			StepQueries( scene, stepIndex );
			break;
		case SCENE_MOVER:
			StepMover( scene, stepIndex );
			break;
		default:
			Scene_WorldStep( scene );
			break;
	}
}

void Scene_Destroy( Scene* scene )
{
	b2DestroyWorld( scene->worldId );
	free( scene->bodies );
	free( scene->probes );
	memset( scene, 0, sizeof( Scene ) );
}
