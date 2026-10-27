// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/mover.as

#include "collide.h"

#include <float.h>

int main( void )
{
	vseed( 61 );

	b2CollisionPlane planes[8];
	for ( int i = 0; i < 20000; ++i )
	{
		int count = vrandint( 0, 9 );
		for ( int j = 0; j < count; ++j )
		{
			b2Rot q = randRot();
			planes[j].plane.normal = ( b2Vec2 ){ q.c, q.s };
			planes[j].plane.offset = vrandf( -0.25f, 0.25f );
			int kind = vrandint( 0, 4 );
			planes[j].pushLimit = kind == 0 ? FLT_MAX : vrandf( 0.0f, 0.5f );
			planes[j].push = vrandf( -1.0f, 1.0f );
			planes[j].clipVelocity = ( vrand32() & 1 ) == 0;
		}

		b2Vec2 target = randVec( -1.0f, 1.0f );
		b2PlaneSolverResult result = b2SolvePlanes( target, planes, count );
		vtag( "solve" );
		pv( result.delta );
		vi32( result.iterationCount );
		for ( int j = 0; j < count; ++j )
		{
			vf32( planes[j].push );
		}
		vend();

		b2Vec2 velocity = randVec( -2.0f, 2.0f );
		vtag( "clip" );
		pv( b2ClipVector( velocity, planes, count ) );
		for ( int j = 0; j < count; ++j )
		{
			planes[j].push = ( vrand32() & 1 ) == 0 ? 0.0f : 1.0f;
		}
		pv( b2ClipVector( velocity, planes, count ) );
		vend();
	}

	return 0;
}
