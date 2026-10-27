// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/distance.as

#include "collide.h"

static void pvertex( const b2SimplexVertex* v )
{
	pv( v->wA );
	pv( v->wB );
	pv( v->w );
	vf32( v->a );
	vi32( v->indexA );
	vi32( v->indexB );
}

// Vertices past count are uninitialized in C
static void psimplex( const b2Simplex* s )
{
	vi32( s->count );
	if ( s->count > 0 )
	{
		pvertex( &s->v1 );
	}
	if ( s->count > 1 )
	{
		pvertex( &s->v2 );
	}
	if ( s->count > 2 )
	{
		pvertex( &s->v3 );
	}
}

static void psweep( const b2Sweep* s )
{
	pv( s->localCenter );
	pv( s->c1 );
	pv( s->c2 );
	pr( s->q1 );
	pr( s->q2 );
}

static b2ShapeProxy gridProxy( int which )
{
	switch ( which )
	{
		case 0:
		{
			b2Vec2 p = { 0.03125f, -0.0625f };
			return b2MakeProxy( &p, 1, 0.25f );
		}
		case 1:
		{
			b2Vec2 ps[2] = { { -0.5f, 0.0625f }, { 0.4375f, -0.0625f } };
			return b2MakeProxy( ps, 2, 0.125f );
		}
		case 2:
		{
			b2Polygon p = libraryPolygon( 2, 0.0f );
			return b2MakeProxy( p.vertices, p.count, p.radius );
		}
		case 3:
		{
			b2Polygon p = libraryPolygon( 5, 0.0625f );
			return b2MakeProxy( p.vertices, p.count, p.radius );
		}
		case 4:
		{
			b2Polygon p = b2MakeBox( 0.375f, 0.25f );
			return b2MakeProxy( p.vertices, p.count, p.radius );
		}
		default:
		{
			b2Polygon p = libraryPolygon( 3, 0.0f );
			return b2MakeProxy( p.vertices, p.count, 0.1875f );
		}
	}
}

// A cache that is empty, carried from the last query when its indices still fit, or random indices
static void randCache( b2SimplexCache* cache, const b2ShapeProxy* proxyA, const b2ShapeProxy* proxyB )
{
	int kind = vrandint( 0, 3 );
	if ( kind == 2 )
	{
		for ( int i = 0; i < cache->count; ++i )
		{
			if ( cache->indexA[i] >= proxyA->count || cache->indexB[i] >= proxyB->count )
			{
				kind = 0;
			}
		}
	}

	if ( kind == 0 )
	{
		*cache = ( b2SimplexCache ){ 0 };
	}
	else if ( kind == 1 )
	{
		*cache = ( b2SimplexCache ){ 0 };
		cache->count = (uint16_t)vrandint( 1, 4 );
		for ( int i = 0; i < cache->count; ++i )
		{
			cache->indexA[i] = (uint8_t)vrandint( 0, proxyA->count );
			cache->indexB[i] = (uint8_t)vrandint( 0, proxyB->count );
		}
	}
}

static b2Sweep randSweep( float reach )
{
	b2Sweep s;
	s.localCenter = randVec( -0.25f, 0.25f );
	s.c1 = randVec( -reach, reach );
	b2Vec2 d = randVec( -reach, reach );
	s.c2 = b2Add( s.c1, d );
	float angle1 = vrandf( -3.0f, 3.0f );
	float delta = vrandf( -1.5f, 1.5f );
	if ( ( vrand32() & 3 ) == 0 )
	{
		delta = 0.0f;
	}
	s.q1 = b2MakeRot( angle1 );
	s.q2 = b2MakeRot( angle1 + delta );
	return s;
}

static void sweeps( void )
{
	for ( int i = 0; i < 5000; ++i )
	{
		b2Sweep sweep = randSweep( 3.0f );
		float t = vrandf( -0.25f, 1.25f );
		int pick = vrandint( 0, 4 );
		if ( pick == 0 )
		{
			t = 0.0f;
		}
		else if ( pick == 1 )
		{
			t = 1.0f;
		}
		vtag( "sw" );
		pxf( b2GetSweepTransform( &sweep, t ) );
		vend();
	}
}

static void segments( void )
{
	for ( int i = 0; i < 20000; ++i )
	{
		b2Vec2 p1 = randVec( -1.0f, 1.0f );
		b2Vec2 q1 = randVec( -1.0f, 1.0f );
		b2Vec2 p2 = randVec( -1.0f, 1.0f );
		b2Vec2 q2 = randVec( -1.0f, 1.0f );
		int kind = vrandint( 0, 8 );
		if ( kind == 0 )
		{
			q1 = p1;
		}
		else if ( kind == 1 )
		{
			q2 = p2;
		}
		else if ( kind == 2 )
		{
			q1 = p1;
			q2 = p2;
		}
		else if ( kind == 3 )
		{
			// parallel
			b2Vec2 offset = randVec( -0.5f, 0.5f );
			float scale = vrandf( -2.0f, 2.0f );
			p2 = b2Add( p1, offset );
			q2 = b2Add( p2, b2MulSV( scale, b2Sub( q1, p1 ) ) );
		}
		else if ( kind == 4 )
		{
			// touching at an end
			p2 = q1;
		}
		b2SegmentDistanceResult r = b2ComputeSegmentDistance( p1, q1, p2, q2 );
		vtag( "sd" );
		pv( r.closest1 );
		pv( r.closest2 );
		vf32( r.fraction1 );
		vf32( r.fraction2 );
		vf32( r.distanceSquared );
		vend();
	}
}

static void proxies( void )
{
	for ( int i = 0; i < 2000; ++i )
	{
		b2Vec2 pts[10];
		int n = vrandint( 1, 11 );
		for ( int j = 0; j < n; ++j )
		{
			pts[j] = randVec( -2.0f, 2.0f );
		}
		float radius = randRadius( 1.0f );
		b2Vec2 position = randVec( -10.0f, 10.0f );
		b2Rot rotation = randRot();
		b2ShapeProxy a = b2MakeProxy( pts, n, radius );
		b2ShapeProxy b = b2MakeOffsetProxy( pts, n, radius, position, rotation );
		vtag( "px" );
		pproxy( &a );
		pproxy( &b );
		vend();
	}
}

static void distanceQuery( const char* tag, const b2DistanceInput* input, b2SimplexCache* cache, bool record )
{
	b2Simplex simplexes[2];
	vtag( tag );
	pcache( cache );
	b2DistanceOutput output = b2ComputeShapeDistance( input, cache, record ? simplexes : NULL, record ? 2 : 0 );
	pdist( output );
	pcache( cache );
	if ( output.simplexCount > 0 )
	{
		psimplex( simplexes + 0 );
	}
	vend();
}

static void distanceGrid( void )
{
	for ( int a = 0; a < 6; ++a )
	{
		for ( int b = 0; b < 6; ++b )
		{
			b2DistanceInput input = { 0 };
			input.proxyA = gridProxy( a );
			input.proxyB = gridProxy( b );

			b2SimplexCache warm = { 0 };
			for ( int an = 0; an < RING_ANGLES; ++an )
			{
				for ( int dn = 0; dn < RING_DEPTHS; ++dn )
				{
					for ( int rn = 0; rn < RING_ROTS; ++rn )
					{
						input.transform = ringPose( an, dn, rn, 1.0f );
						input.useRadii = ( ( an + dn + rn ) & 1 ) == 0;

						b2SimplexCache cold = { 0 };
						distanceQuery( "dg", &input, &cold, rn == 0 );
						distanceQuery( "dw", &input, &warm, rn == 1 );
					}
				}
			}
		}
	}
}

static void distanceRandom( void )
{
	b2SimplexCache cache = { 0 };
	for ( int i = 0; i < 30000; ++i )
	{
		b2DistanceInput input;
		input.proxyA = randProxy();
		input.proxyB = randProxy();
		input.transform = randXf( 1.5f );
		input.useRadii = ( vrand32() & 1 ) == 0;
		randCache( &cache, &input.proxyA, &input.proxyB );
		distanceQuery( "dr", &input, &cache, ( i & 3 ) == 0 );
	}
}

static void shapeCasts( void )
{
	for ( int i = 0; i < 20000; ++i )
	{
		b2ShapeCastPairInput input;
		input.proxyA = randProxy();
		input.proxyB = randProxy();
		input.transform = randXf( 3.0f );
		b2Vec2 target = randVec( -0.5f, 0.5f );
		float scale = vrandf( 0.25f, 2.5f );
		input.translationB = b2MulSV( scale, b2Sub( target, input.transform.p ) );
		input.maxFraction = vrandf( 0.25f, 1.25f );
		input.canEncroach = ( vrand32() & 1 ) == 0;
		vtag( "cs" );
		pcast( b2CastShape( &input ) );
		vend();
	}
}

static void timeOfImpact( void )
{
	for ( int i = 0; i < 20000; ++i )
	{
		b2TOIInput input;
		input.proxyA = randProxy();
		input.proxyB = randProxy();
		input.sweepA = randSweep( 2.0f );
		input.sweepB = randSweep( 2.0f );
		if ( ( i & 3 ) == 0 )
		{
			// a fast mover against a nearly static one
			input.sweepA.c2 = b2Add( input.sweepA.c1, randVec( -0.125f, 0.125f ) );
			input.sweepB.c2 = b2Sub( b2MulSV( 2.0f, input.sweepA.c1 ), input.sweepB.c1 );
		}
		input.maxFraction = ( i & 1 ) ? 1.0f : vrandf( 0.25f, 1.0f );
		b2TOIOutput output = b2ComputeTimeOfImpact( &input );
		vtag( "toi" );
		psweep( &input.sweepA );
		psweep( &input.sweepB );
		vi32( output.state );
		pv( output.point );
		pv( output.normal );
		vf32( output.fraction );
		vend();
	}
}

int main( void )
{
	vseed( 41 );
	sweeps();
	segments();
	proxies();
	distanceGrid();
	distanceRandom();
	shapeCasts();
	timeOfImpact();
	return 0;
}
