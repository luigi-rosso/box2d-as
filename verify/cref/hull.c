// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/hull.as

#include "collide.h"

// Point clouds: scattered, tiny (welding), nearly collinear, with near duplicates, on a circle.
// The count runs past B2_MAX_POLYGON_VERTICES and below 3.
static int makePoints( b2Vec2* pts )
{
	int kind = vrandint( 0, 5 );
	int count = vrandint( 0, 11 );
	b2Vec2 center = randVec( -4.0f, 4.0f );
	float scale = kind == 1 ? vrandf( 0.0001f, 0.02f ) : vrandf( 0.125f, 3.0f );
	b2Rot dir = randRot();
	b2Vec2 axis = { dir.c, dir.s };
	for ( int i = 0; i < count; ++i )
	{
		if ( kind == 2 )
		{
			float t = vrandf( -1.0f, 1.0f );
			float jitter = vrandf( -0.004f, 0.004f );
			pts[i] = b2Add( center, b2Add( b2MulSV( scale * t, axis ), b2MulSV( jitter, b2LeftPerp( axis ) ) ) );
		}
		else if ( kind == 3 && i > 0 && ( vrand32() & 1 ) == 0 )
		{
			int j = vrandint( 0, i );
			b2Vec2 offset = randVec( -0.004f, 0.004f );
			pts[i] = b2Add( pts[j], offset );
		}
		else if ( kind == 4 )
		{
			b2Rot q = randRot();
			pts[i] = b2Add( center, b2MulSV( scale, ( b2Vec2 ){ q.c, q.s } ) );
		}
		else
		{
			b2Vec2 r = randVec( -1.0f, 1.0f );
			pts[i] = b2Add( center, b2MulSV( scale, r ) );
		}
	}
	return count;
}

static void phull( const b2Hull* hull )
{
	vi32( hull->count );
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		pv( hull->points[i] );
	}
}

int main( void )
{
	vseed( 21 );

	// Fixed cases
	{
		b2Vec2 square[4] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };
		b2Hull hull = b2ComputeHull( square, 4 );
		vtag( "square" );
		phull( &hull );
		vbool( b2ValidateHull( &hull ) );
		vend();

		b2Vec2 line[5] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 2.0f, 0.0f }, { 3.0f, 0.0f }, { 4.0f, 0.0f } };
		hull = b2ComputeHull( line, 5 );
		vtag( "line" );
		phull( &hull );
		vend();

		b2Vec2 same[3] = { { 0.5f, 0.5f }, { 0.5f, 0.5f }, { 0.5f, 0.5f } };
		hull = b2ComputeHull( same, 3 );
		vtag( "same" );
		phull( &hull );
		vend();
	}

	b2Vec2 pts[16];
	for ( int i = 0; i < 30000; ++i )
	{
		int count = makePoints( pts );
		b2Hull hull = b2ComputeHull( pts, count );
		vtag( "h" );
		vi32( count );
		phull( &hull );
		vbool( b2ValidateHull( &hull ) );
		vend();
	}

	// Raw point sets as hulls exercise the rejections of b2ValidateHull
	for ( int i = 0; i < 20000; ++i )
	{
		b2Hull hull = { 0 };
		hull.count = vrandint( 0, 10 );
		int n = hull.count < 8 ? hull.count : 8;
		bool ring = ( vrand32() & 1 ) == 0;
		float phase = vrandf( 0.0f, 1.0f );
		for ( int j = 0; j < n; ++j )
		{
			if ( ring )
			{
				float a = phase + 2.0f * B2_PI * (float)j / (float)n;
				float r = vrandf( 0.5f, 1.0f );
				b2CosSin cs = b2ComputeCosSin( a );
				hull.points[j] = ( b2Vec2 ){ r * cs.cosine, r * cs.sine };
			}
			else
			{
				hull.points[j] = randVec( -1.0f, 1.0f );
			}
		}
		vtag( "v" );
		phull( &hull );
		vbool( b2ValidateHull( &hull ) );
		vend();
	}

	return 0;
}
