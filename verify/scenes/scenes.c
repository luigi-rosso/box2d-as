// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory parity scenes. Add a scene here and in scenes.as, then list it in s_sceneNames, in
// verify/traj.mjs and in verify/traj.sh.

#include "scenes.h"

#include <stdlib.h>
#include <string.h>

static const char* s_sceneNames[] = { "pyramid" };

int Scene_GetCount( void )
{
	return (int)( sizeof( s_sceneNames ) / sizeof( s_sceneNames[0] ) );
}

const char* Scene_GetName( int index )
{
	return s_sceneNames[index];
}

static b2BodyId AddBody( Scene* scene, const b2BodyDef* def )
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

// A ground box and a 10 box base pyramid of unit boxes, sleeping enabled, settling for 300 steps
static void CreatePyramid( Scene* scene )
{
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 0.0f, -1.0f };
		b2BodyId groundId = AddBody( scene, &bodyDef );

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
		float y = ( 2.0f * i + 1.0f ) * shift;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - a * baseCount;

			bodyDef.position = (b2Pos){ x, y };

			b2BodyId bodyId = AddBody( scene, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
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

	// Without task callbacks one worker runs serially and more use C's internal scheduler
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	scene->worldId = b2CreateWorld( &worldDef );

	if ( index == 0 )
	{
		CreatePyramid( scene );
	}

	return true;
}

void Scene_Step( Scene* scene, int stepIndex )
{
	(void)stepIndex;
	b2World_Step( scene->worldId, 1.0f / 60.0f, SCENE_SUBSTEP_COUNT );
}

void Scene_Destroy( Scene* scene )
{
	b2DestroyWorld( scene->worldId );
	free( scene->bodies );
	memset( scene, 0, sizeof( Scene ) );
}
