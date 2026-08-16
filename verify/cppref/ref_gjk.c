// Phase B reference generator: GJK distance (cold + warm cache), shape cast,
// TOI, and the geometry.c pieces that need distance.c (point-in-polygon,
// rounded polygon ray cast, b2ShapeCast* wrappers). Lines are self-contained
// (inputs + expected outputs) with floats as raw bit patterns.

#include "refcommon.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include "constants.h"

typedef struct ProxySpec
{
	int count;
	float radius;
	b2Vec2 pts[8];
} ProxySpec;

// Small library of proxy shapes exercised pairwise.
static const ProxySpec specs[] = {
	{ 1, 0.0f, { { 0.0f, 0.0f } } },
	{ 1, 0.35f, { { 0.1f, -0.05f } } },
	{ 2, 0.25f, { { -0.4f, 0.0f }, { 0.4f, 0.0f } } },
	{ 2, 0.0f, { { -0.6f, -0.1f }, { 0.5f, 0.2f } } },
	{ 3, 0.0f, { { -0.5f, -0.3f }, { 0.5f, -0.2f }, { 0.0f, 0.6f } } },
	{ 4, 0.1f, { { -0.4f, -0.3f }, { 0.4f, -0.3f }, { 0.4f, 0.3f }, { -0.4f, 0.3f } } },
	{ 5, 0.0f, { { 0.6f, 0.0f }, { 0.2f, 0.55f }, { -0.45f, 0.35f }, { -0.5f, -0.3f }, { 0.15f, -0.55f } } },
	{ 8,
	  0.05f,
	  { { 0.5f, -0.21f },
		{ 0.5f, 0.21f },
		{ 0.21f, 0.5f },
		{ -0.21f, 0.5f },
		{ -0.5f, 0.21f },
		{ -0.5f, -0.21f },
		{ -0.21f, -0.5f },
		{ 0.21f, -0.5f } } },
};

#define SPEC_COUNT ( (int)( sizeof( specs ) / sizeof( specs[0] ) ) )

static b2ShapeProxy makeSpecProxy( const ProxySpec* s )
{
	return b2MakeProxy( s->pts, s->count, s->radius );
}

static void printProxy( const ProxySpec* s )
{
	pi( s->count );
	ph( s->radius );
	for ( int i = 0; i < s->count; ++i )
	{
		ph( s->pts[i].x );
		ph( s->pts[i].y );
	}
}

static void printTransform( b2Transform xf )
{
	ph( xf.p.x );
	ph( xf.p.y );
	ph( xf.q.c );
	ph( xf.q.s );
}

static void printCache( const b2SimplexCache* cache )
{
	pi( cache->count );
	for ( int i = 0; i < 3; ++i )
	{
		pi( cache->indexA[i] );
	}
	for ( int i = 0; i < 3; ++i )
	{
		pi( cache->indexB[i] );
	}
}

static void emitDistance( const ProxySpec* sa, const ProxySpec* sb, b2Transform xfA, b2Transform xfB, bool useRadii,
						  b2SimplexCache* cache )
{
	printf( "DIST" );
	printProxy( sa );
	printProxy( sb );
	printTransform( xfA );
	printTransform( xfB );
	pi( useRadii ? 1 : 0 );
	printCache( cache );

	b2DistanceInput input;
	input.proxyA = makeSpecProxy( sa );
	input.proxyB = makeSpecProxy( sb );
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = useRadii;

	b2DistanceOutput out = b2ShapeDistance( &input, cache, NULL, 0 );

	ph( out.pointA.x );
	ph( out.pointA.y );
	ph( out.pointB.x );
	ph( out.pointB.y );
	ph( out.normal.x );
	ph( out.normal.y );
	ph( out.distance );
	pi( out.iterations );
	printCache( cache );
	printf( "\n" );
}

static void sweepDistance( void )
{
	const float dxs[] = { -1.8f, -1.1f, -0.55f, -0.2f, 0.0f, 0.15f, 0.6f, 1.05f, 1.7f, 4.0f };
	const float dys[] = { -1.4f, -0.6f, -0.05f, 0.0f, 0.3f, 0.9f, 1.6f };
	const float rots[] = { 0.0f, 0.35f, -1.2f, 2.8f };

	for ( int ia = 0; ia < SPEC_COUNT; ++ia )
	{
		for ( int ib = 0; ib < SPEC_COUNT; ++ib )
		{
			const ProxySpec* sa = specs + ia;
			const ProxySpec* sb = specs + ib;

			b2Transform xfA;
			xfA.p = ( b2Vec2 ){ 0.05f, -0.02f };
			xfA.q = b2MakeRot( 0.15f * (float)( ia - ib ) );

			for ( int r = 0; r < 4; ++r )
			{
				for ( int ix = 0; ix < 10; ++ix )
				{
					for ( int iy = 0; iy < 7; ++iy )
					{
						b2Transform xfB;
						xfB.p = ( b2Vec2 ){ dxs[ix], dys[iy] };
						xfB.q = b2MakeRot( rots[r] );

						bool useRadii = ( ( ix + iy + r ) % 2 ) == 1;

						// cold
						b2SimplexCache cache = { 0 };
						emitDistance( sa, sb, xfA, xfB, useRadii, &cache );

						// warm: nudge B, reuse the cache
						if ( ( ix + iy ) % 2 == 0 )
						{
							b2Transform xfB2;
							xfB2.p = ( b2Vec2 ){ xfB.p.x + 0.013f, xfB.p.y - 0.017f };
							xfB2.q = b2MakeRot( rots[r] + 0.05f );
							emitDistance( sa, sb, xfA, xfB2, useRadii, &cache );
						}
					}
				}
			}
		}
	}
}

static void emitShapeCast( const ProxySpec* sa, const ProxySpec* sb, b2Transform xfA, b2Transform xfB, b2Vec2 translationB,
						   float maxFraction, bool canEncroach )
{
	printf( "SC" );
	printProxy( sa );
	printProxy( sb );
	printTransform( xfA );
	printTransform( xfB );
	ph( translationB.x );
	ph( translationB.y );
	ph( maxFraction );
	pi( canEncroach ? 1 : 0 );

	b2ShapeCastPairInput input;
	input.proxyA = makeSpecProxy( sa );
	input.proxyB = makeSpecProxy( sb );
	input.transformA = xfA;
	input.transformB = xfB;
	input.translationB = translationB;
	input.maxFraction = maxFraction;
	input.canEncroach = canEncroach;

	b2CastOutput out = b2ShapeCast( &input );

	pi( out.hit ? 1 : 0 );
	ph( out.fraction );
	ph( out.normal.x );
	ph( out.normal.y );
	ph( out.point.x );
	ph( out.point.y );
	pi( out.iterations );
	printf( "\n" );
}

static void sweepShapeCast( void )
{
	const float angles[] = { 0.0f, 0.7f, 1.5707964f, 2.6f, 3.6f, 4.7f, 5.6f };
	const float dists[] = { 0.0f, 0.2f, 0.8f, 1.6f, 3.0f };

	for ( int ia = 0; ia < SPEC_COUNT; ++ia )
	{
		for ( int ib = 0; ib < SPEC_COUNT; ++ib )
		{
			const ProxySpec* sa = specs + ia;
			const ProxySpec* sb = specs + ib;

			b2Transform xfA;
			xfA.p = ( b2Vec2 ){ 0.0f, 0.0f };
			xfA.q = b2MakeRot( 0.25f * (float)ia );

			for ( int an = 0; an < 7; ++an )
			{
				for ( int dn = 0; dn < 5; ++dn )
				{
					// Start B at distance d along angle, cast back towards A
					b2Rot dir = b2MakeRot( angles[an] );
					b2Vec2 offset = { dists[dn] * dir.c, dists[dn] * dir.s };

					b2Transform xfB;
					xfB.p = offset;
					xfB.q = b2MakeRot( 0.4f * (float)dn + 0.1f * (float)ib );

					// Cast towards A center (grazing comes from the angle sweep)
					b2Vec2 translation = { -3.5f * dir.c + 0.3f * dir.s, -3.5f * dir.s - 0.3f * dir.c };

					bool canEncroach = ( ( an + dn + ia ) % 3 ) == 0;
					float maxFraction = ( dn % 2 ) == 0 ? 1.0f : 0.35f;

					emitShapeCast( sa, sb, xfA, xfB, translation, maxFraction, canEncroach );
				}
			}
		}
	}
}

static void printSweep( const b2Sweep* s )
{
	ph( s->localCenter.x );
	ph( s->localCenter.y );
	ph( s->c1.x );
	ph( s->c1.y );
	ph( s->c2.x );
	ph( s->c2.y );
	ph( s->q1.c );
	ph( s->q1.s );
	ph( s->q2.c );
	ph( s->q2.s );
}

static void sweepTOI( void )
{
	for ( int ia = 0; ia < SPEC_COUNT; ++ia )
	{
		for ( int ib = 0; ib < SPEC_COUNT; ++ib )
		{
			const ProxySpec* sa = specs + ia;
			const ProxySpec* sb = specs + ib;

			for ( int k = 0; k < 40; ++k )
			{
				b2TOIInput input;
				input.proxyA = makeSpecProxy( sa );
				input.proxyB = makeSpecProxy( sb );

				b2Sweep sweepA;
				sweepA.localCenter = ( b2Vec2 ){ frand( -0.2f, 0.2f ), frand( -0.2f, 0.2f ) };
				sweepA.c1 = ( b2Vec2 ){ frand( -0.5f, 0.5f ), frand( -0.5f, 0.5f ) };
				sweepA.c2 = ( b2Vec2 ){ sweepA.c1.x + frand( -1.0f, 1.0f ), sweepA.c1.y + frand( -1.0f, 1.0f ) };
				sweepA.q1 = b2MakeRot( frand( -3.0f, 3.0f ) );
				sweepA.q2 = b2MakeRot( frand( -3.0f, 3.0f ) );

				b2Sweep sweepB;
				sweepB.localCenter = ( b2Vec2 ){ frand( -0.2f, 0.2f ), frand( -0.2f, 0.2f ) };
				// Bias starting positions towards approach scenarios
				float d0 = frand( 0.0f, 4.0f );
				float ang = frand( 0.0f, 6.28f );
				sweepB.c1 = ( b2Vec2 ){ sweepA.c1.x + d0 * cosf( ang ), sweepA.c1.y + d0 * sinf( ang ) };
				sweepB.c2 = ( b2Vec2 ){ sweepA.c2.x + frand( -0.5f, 0.5f ), sweepA.c2.y + frand( -0.5f, 0.5f ) };
				sweepB.q1 = b2MakeRot( frand( -3.0f, 3.0f ) );
				sweepB.q2 = b2MakeRot( frand( -3.0f, 3.0f ) );

				input.sweepA = sweepA;
				input.sweepB = sweepB;
				input.maxFraction = ( k % 4 ) == 3 ? 0.5f : 1.0f;

				printf( "TOI" );
				printProxy( sa );
				printProxy( sb );
				printSweep( &sweepA );
				printSweep( &sweepB );
				ph( input.maxFraction );

				b2TOIOutput out = b2TimeOfImpact( &input );

				pi( (int)out.state );
				ph( out.fraction );
				ph( out.point.x );
				ph( out.point.y );
				ph( out.normal.x );
				ph( out.normal.y );
				printf( "\n" );
			}
		}
	}
}

// Point sets whose computed hull feeds the polygon-based families.
static const int polyCounts[] = { 3, 4, 5, 6, 8 };

static b2Polygon stagePolygon( int which, float radius, b2Vec2* outPts, int* outCount )
{
	int n = polyCounts[which % 5];
	b2Vec2 pts[8];
	for ( int i = 0; i < n; ++i )
	{
		float a = 6.2831853f * (float)i / (float)n + 0.1f * (float)which;
		float r = 0.5f + 0.15f * (float)( i % 3 );
		pts[i] = ( b2Vec2 ){ r * cosf( a ), r * sinf( a ) };
	}
	b2Hull hull = b2ComputeHull( pts, n );
	for ( int i = 0; i < n; ++i )
	{
		outPts[i] = pts[i];
	}
	*outCount = n;
	return b2MakePolygon( &hull, radius );
}

static void sweepGeometryTail( void )
{
	for ( int w = 0; w < 10; ++w )
	{
		b2Vec2 pts[8];
		int n;
		float radius = ( w % 3 ) == 0 ? 0.0f : 0.1f * (float)( w % 3 );
		b2Polygon poly = stagePolygon( w, radius, pts, &n );

		// Point in polygon over a grid
		for ( int ix = 0; ix < 12; ++ix )
		{
			for ( int iy = 0; iy < 12; ++iy )
			{
				b2Vec2 p = { -1.1f + 0.2f * (float)ix, -1.1f + 0.2f * (float)iy };
				printf( "PIP" );
				pi( n );
				for ( int i = 0; i < n; ++i )
				{
					ph( pts[i].x );
					ph( pts[i].y );
				}
				ph( radius );
				ph( p.x );
				ph( p.y );
				pi( b2PointInPolygon( &poly, p ) ? 1 : 0 );
				printf( "\n" );
			}
		}

		// Rounded polygon ray cast (radius > 0 goes through b2ShapeCast)
		for ( int k = 0; k < 40; ++k )
		{
			b2RayCastInput input;
			input.origin = ( b2Vec2 ){ frand( -2.0f, 2.0f ), frand( -2.0f, 2.0f ) };
			input.translation = ( b2Vec2 ){ frand( -3.0f, 3.0f ), frand( -3.0f, 3.0f ) };
			input.maxFraction = ( k % 3 ) == 0 ? 0.6f : 1.0f;

			printf( "RCRP" );
			pi( n );
			for ( int i = 0; i < n; ++i )
			{
				ph( pts[i].x );
				ph( pts[i].y );
			}
			ph( radius );
			ph( input.origin.x );
			ph( input.origin.y );
			ph( input.translation.x );
			ph( input.translation.y );
			ph( input.maxFraction );

			b2CastOutput out = b2RayCastPolygon( &poly, &input );
			pi( out.hit ? 1 : 0 );
			ph( out.fraction );
			ph( out.normal.x );
			ph( out.normal.y );
			ph( out.point.x );
			ph( out.point.y );
			pi( out.iterations );
			printf( "\n" );
		}
	}

	// Shape cast wrappers against every library proxy
	for ( int ib = 0; ib < SPEC_COUNT; ++ib )
	{
		const ProxySpec* sb = specs + ib;
		for ( int k = 0; k < 30; ++k )
		{
			b2ShapeCastInput input;
			input.proxy = makeSpecProxy( sb );
			float ang = frand( 0.0f, 6.28f );
			float d0 = frand( 0.2f, 3.0f );
			// offset the proxy points
			for ( int i = 0; i < input.proxy.count; ++i )
			{
				input.proxy.points[i].x += d0 * cosf( ang );
				input.proxy.points[i].y += d0 * sinf( ang );
			}
			input.translation = ( b2Vec2 ){ -3.0f * cosf( ang ) + frand( -0.4f, 0.4f ), -3.0f * sinf( ang ) + frand( -0.4f, 0.4f ) };
			input.maxFraction = ( k % 2 ) == 0 ? 1.0f : 0.4f;
			input.canEncroach = ( k % 5 ) == 0;

			int which = k % 4;
			printf( "SCW" );
			pi( which );
			pi( input.proxy.count );
			ph( input.proxy.radius );
			for ( int i = 0; i < input.proxy.count; ++i )
			{
				ph( input.proxy.points[i].x );
				ph( input.proxy.points[i].y );
			}
			ph( input.translation.x );
			ph( input.translation.y );
			ph( input.maxFraction );
			pi( input.canEncroach ? 1 : 0 );

			b2CastOutput out;
			if ( which == 0 )
			{
				b2Circle circle = { { 0.1f, -0.05f }, 0.4f };
				out = b2ShapeCastCircle( &circle, &input );
			}
			else if ( which == 1 )
			{
				b2Capsule capsule = { { -0.4f, 0.1f }, { 0.45f, -0.1f }, 0.3f };
				out = b2ShapeCastCapsule( &capsule, &input );
			}
			else if ( which == 2 )
			{
				b2Segment segment = { { -0.55f, -0.2f }, { 0.5f, 0.25f } };
				out = b2ShapeCastSegment( &segment, &input );
			}
			else
			{
				b2Polygon box = b2MakeRoundedBox( 0.45f, 0.3f, 0.1f );
				out = b2ShapeCastPolygon( &box, &input );
			}

			pi( out.hit ? 1 : 0 );
			ph( out.fraction );
			ph( out.normal.x );
			ph( out.normal.y );
			ph( out.point.x );
			ph( out.point.y );
			pi( out.iterations );
			printf( "\n" );
		}
	}
}

int main( void )
{
	sweepDistance( );
	sweepShapeCast( );
	sweepTOI( );
	sweepGeometryTail( );
	return 0;
}
