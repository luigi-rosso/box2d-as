// Phase B reference generator: every collide pair function in manifold.c
// over grids of relative poses (overlapping, touching, separated, degenerate
// near-parallel). Outputs compare point counts, anchors, separations, the
// world normal, and the packed feature ids bit-exactly.

#include "refcommon.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

static void printTransform( b2Transform xf )
{
	ph( xf.p.x );
	ph( xf.p.y );
	ph( xf.q.c );
	ph( xf.q.s );
}

static void printManifold( const b2Manifold* m )
{
	pi( m->pointCount );
	ph( m->normal.x );
	ph( m->normal.y );
	for ( int i = 0; i < m->pointCount; ++i )
	{
		const b2ManifoldPoint* mp = m->points + i;
		ph( mp->point.x );
		ph( mp->point.y );
		ph( mp->anchorA.x );
		ph( mp->anchorA.y );
		ph( mp->anchorB.x );
		ph( mp->anchorB.y );
		ph( mp->separation );
		pi( (int)mp->id );
	}
	printf( "\n" );
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

// Pose grid shared by all families: angle ring x depth x rotations.
#define ANGLE_COUNT 8
#define DEPTH_COUNT 7
#define ROT_COUNT 4

static const float depths[DEPTH_COUNT] = { -0.35f, -0.12f, -0.02f, 0.0f, 0.015f, 0.1f, 0.6f };
static const float rotsB[ROT_COUNT] = { 0.0f, 0.4f, -1.3f, 2.9f };

// Returns xfB placing shape B roughly at 'reach + depth' from origin along the ring angle.
static b2Transform poseB( int an, int dn, int rn, float reach )
{
	float a = 6.2831853f * (float)an / (float)ANGLE_COUNT + 0.07f;
	float d = reach + depths[dn];
	b2Transform xf;
	xf.p = ( b2Vec2 ){ d * cosf( a ), d * sinf( a ) };
	xf.q = b2MakeRot( rotsB[rn] );
	return xf;
}

static b2Transform poseA( int variant )
{
	b2Transform xf;
	xf.p = ( b2Vec2 ){ 0.03f * (float)variant, -0.02f * (float)variant };
	xf.q = b2MakeRot( 0.3f * (float)variant );
	return xf;
}

static void sweepCircles( void )
{
	b2Circle circleA = { { 0.05f, -0.02f }, 0.4f };
	b2Circle circleB = { { -0.03f, 0.06f }, 0.3f };

	for ( int v = 0; v < 3; ++v )
	{
		b2Transform xfA = poseA( v );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfB = poseB( an, dn, rn, 0.7f );
					printf( "MCC" );
					ph( circleA.center.x );
					ph( circleA.center.y );
					ph( circleA.radius );
					ph( circleB.center.x );
					ph( circleB.center.y );
					ph( circleB.radius );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollideCircles( &circleA, xfA, &circleB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void printCapsule( const b2Capsule* c )
{
	ph( c->center1.x );
	ph( c->center1.y );
	ph( c->center2.x );
	ph( c->center2.y );
	ph( c->radius );
}

static void printCircle( const b2Circle* c )
{
	ph( c->center.x );
	ph( c->center.y );
	ph( c->radius );
}

static void printSegment( const b2Segment* s )
{
	ph( s->point1.x );
	ph( s->point1.y );
	ph( s->point2.x );
	ph( s->point2.y );
}

static void sweepCapsuleAndCircle( void )
{
	b2Capsule capsuleA = { { -0.45f, 0.05f }, { 0.5f, -0.08f }, 0.25f };
	b2Circle circleB = { { 0.02f, -0.03f }, 0.3f };

	for ( int v = 0; v < 3; ++v )
	{
		b2Transform xfA = poseA( v );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfB = poseB( an, dn, rn, 0.8f );
					printf( "MKC" );
					printCapsule( &capsuleA );
					printCircle( &circleB );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollideCapsuleAndCircle( &capsuleA, xfA, &circleB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void sweepCapsules( void )
{
	b2Capsule capsuleA = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.2f };

	for ( int variant = 0; variant < 4; ++variant )
	{
		// Variant 1 is near-parallel, variant 2 is perpendicular, variant 3 is short
		b2Capsule capsuleB;
		if ( variant == 0 )
		{
			capsuleB = ( b2Capsule ){ { -0.4f, 0.02f }, { 0.45f, -0.06f }, 0.15f };
		}
		else if ( variant == 1 )
		{
			capsuleB = ( b2Capsule ){ { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.2f };
		}
		else if ( variant == 2 )
		{
			capsuleB = ( b2Capsule ){ { 0.0f, -0.45f }, { 0.0f, 0.5f }, 0.18f };
		}
		else
		{
			capsuleB = ( b2Capsule ){ { -0.1f, 0.0f }, { 0.12f, 0.03f }, 0.25f };
		}

		b2Transform xfA = poseA( variant );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfB = poseB( an, dn, rn, 0.75f );
					printf( "MKK" );
					printCapsule( &capsuleA );
					printCapsule( &capsuleB );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollideCapsules( &capsuleA, xfA, &capsuleB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void sweepSegmentAndCapsule( void )
{
	b2Segment segmentA = { { -0.7f, -0.05f }, { 0.65f, 0.1f } };
	b2Capsule capsuleB = { { -0.35f, 0.0f }, { 0.4f, 0.05f }, 0.22f };

	for ( int an = 0; an < ANGLE_COUNT; ++an )
	{
		for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
		{
			for ( int rn = 0; rn < ROT_COUNT; ++rn )
			{
				b2Transform xfA = poseA( an % 3 );
				b2Transform xfB = poseB( an, dn, rn, 0.6f );
				printf( "MSK" );
				printSegment( &segmentA );
				printCapsule( &capsuleB );
				printTransform( xfA );
				printTransform( xfB );
				b2Manifold m = b2CollideSegmentAndCapsule( &segmentA, xfA, &capsuleB, xfB );
				printManifold( &m );
			}
		}
	}
}

// Deterministic polygon library (must match the AS driver's construction:
// hull of the emitted points, given radius).
static b2Polygon makeLibraryPolygon( int which, float radius, b2Vec2* outPts, int* outCount )
{
	static const int counts[5] = { 3, 4, 5, 6, 8 };
	int n = counts[which % 5];
	b2Vec2 pts[8];
	for ( int i = 0; i < n; ++i )
	{
		float a = 6.2831853f * (float)i / (float)n + 0.13f * (float)which;
		float r = 0.45f + 0.12f * (float)( i % 3 );
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

static void printPolySpec( const b2Vec2* pts, int n, float radius )
{
	pi( n );
	for ( int i = 0; i < n; ++i )
	{
		ph( pts[i].x );
		ph( pts[i].y );
	}
	ph( radius );
}

static void sweepPolygonAndCircle( void )
{
	b2Circle circleB = { { 0.01f, 0.04f }, 0.28f };

	for ( int w = 0; w < 5; ++w )
	{
		b2Vec2 pts[8];
		int n;
		float radius = ( w % 2 ) == 0 ? 0.0f : 0.08f;
		b2Polygon polyA = makeLibraryPolygon( w, radius, pts, &n );

		b2Transform xfA = poseA( w % 3 );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfB = poseB( an, dn, rn, 0.75f );
					printf( "MPC" );
					printPolySpec( pts, n, radius );
					printCircle( &circleB );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollidePolygonAndCircle( &polyA, xfA, &circleB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void sweepPolygonAndCapsule( void )
{
	b2Capsule capsuleB = { { -0.3f, 0.02f }, { 0.35f, -0.04f }, 0.2f };

	for ( int w = 0; w < 5; ++w )
	{
		b2Vec2 pts[8];
		int n;
		float radius = ( w % 3 ) == 0 ? 0.06f : 0.0f;
		b2Polygon polyA = makeLibraryPolygon( w, radius, pts, &n );

		b2Transform xfA = poseA( w % 3 );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfB = poseB( an, dn, rn, 0.75f );
					printf( "MPK" );
					printPolySpec( pts, n, radius );
					printCapsule( &capsuleB );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollidePolygonAndCapsule( &polyA, xfA, &capsuleB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void sweepPolygons( void )
{
	for ( int wa = 0; wa < 5; ++wa )
	{
		b2Vec2 ptsA[8];
		int nA;
		float radiusA = ( wa % 2 ) == 0 ? 0.0f : 0.07f;
		b2Polygon polyA = makeLibraryPolygon( wa, radiusA, ptsA, &nA );

		for ( int wb = 0; wb < 5; ++wb )
		{
			b2Vec2 ptsB[8];
			int nB;
			float radiusB = ( wb % 3 ) == 0 ? 0.05f : 0.0f;
			b2Polygon polyB = makeLibraryPolygon( wb + 2, radiusB, ptsB, &nB );

			b2Transform xfA = poseA( ( wa + wb ) % 3 );
			for ( int an = 0; an < ANGLE_COUNT; ++an )
			{
				for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
				{
					int rn = ( an + dn + wa + wb ) % ROT_COUNT;
					b2Transform xfB = poseB( an, dn, rn, 0.95f );
					printf( "MPP" );
					printPolySpec( ptsA, nA, radiusA );
					printPolySpec( ptsB, nB, radiusB );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollidePolygons( &polyA, xfA, &polyB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void sweepSegmentAndCircle( void )
{
	b2Segment segmentA = { { -0.6f, 0.05f }, { 0.7f, -0.1f } };
	b2Circle circleB = { { -0.02f, 0.03f }, 0.3f };

	for ( int an = 0; an < ANGLE_COUNT; ++an )
	{
		for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
		{
			for ( int rn = 0; rn < ROT_COUNT; ++rn )
			{
				b2Transform xfA = poseA( dn % 3 );
				b2Transform xfB = poseB( an, dn, rn, 0.55f );
				printf( "MSC" );
				printSegment( &segmentA );
				printCircle( &circleB );
				printTransform( xfA );
				printTransform( xfB );
				b2Manifold m = b2CollideSegmentAndCircle( &segmentA, xfA, &circleB, xfB );
				printManifold( &m );
			}
		}
	}
}

static void sweepSegmentAndPolygon( void )
{
	b2Segment segmentA = { { -0.8f, 0.0f }, { 0.75f, 0.06f } };

	for ( int w = 0; w < 5; ++w )
	{
		b2Vec2 pts[8];
		int n;
		float radius = ( w % 2 ) == 1 ? 0.06f : 0.0f;
		b2Polygon polyB = makeLibraryPolygon( w + 1, radius, pts, &n );

		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfA = poseA( w % 3 );
					b2Transform xfB = poseB( an, dn, rn, 0.7f );
					printf( "MSP" );
					printSegment( &segmentA );
					printPolySpec( pts, n, radius );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollideSegmentAndPolygon( &segmentA, xfA, &polyB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

// Chain segment variants: straight, convex corner, concave corner ghosts.
static b2ChainSegment makeChainSegment( int variant )
{
	b2ChainSegment cs;
	cs.chainId = 0;
	cs.segment = ( b2Segment ){ { -0.6f, 0.0f }, { 0.6f, 0.05f } };
	if ( variant == 0 )
	{
		// nearly straight neighbors
		cs.ghost1 = ( b2Vec2 ){ -1.8f, -0.02f };
		cs.ghost2 = ( b2Vec2 ){ 1.8f, 0.1f };
	}
	else if ( variant == 1 )
	{
		// convex corners
		cs.ghost1 = ( b2Vec2 ){ -1.5f, 0.9f };
		cs.ghost2 = ( b2Vec2 ){ 1.5f, 1.0f };
	}
	else if ( variant == 2 )
	{
		// concave corners
		cs.ghost1 = ( b2Vec2 ){ -1.5f, -0.9f };
		cs.ghost2 = ( b2Vec2 ){ 1.5f, -1.0f };
	}
	else
	{
		// mixed
		cs.ghost1 = ( b2Vec2 ){ -1.4f, 0.8f };
		cs.ghost2 = ( b2Vec2 ){ 1.6f, -0.7f };
	}
	return cs;
}

static void printChainSegment( const b2ChainSegment* cs )
{
	ph( cs->ghost1.x );
	ph( cs->ghost1.y );
	ph( cs->segment.point1.x );
	ph( cs->segment.point1.y );
	ph( cs->segment.point2.x );
	ph( cs->segment.point2.y );
	ph( cs->ghost2.x );
	ph( cs->ghost2.y );
}

static void sweepChainSegmentAndCircle( void )
{
	b2Circle circleB = { { 0.0f, 0.02f }, 0.25f };

	for ( int variant = 0; variant < 4; ++variant )
	{
		b2ChainSegment cs = makeChainSegment( variant );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfA = poseA( variant % 3 );
					b2Transform xfB = poseB( an, dn, rn, 0.5f );
					printf( "MXC" );
					printChainSegment( &cs );
					printCircle( &circleB );
					printTransform( xfA );
					printTransform( xfB );
					b2Manifold m = b2CollideChainSegmentAndCircle( &cs, xfA, &circleB, xfB );
					printManifold( &m );
				}
			}
		}
	}
}

static void sweepChainSegmentAndCapsule( void )
{
	b2Capsule capsuleB = { { -0.3f, 0.0f }, { 0.32f, 0.05f }, 0.18f };

	for ( int variant = 0; variant < 4; ++variant )
	{
		b2ChainSegment cs = makeChainSegment( variant );
		for ( int an = 0; an < ANGLE_COUNT; ++an )
		{
			for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
			{
				for ( int rn = 0; rn < ROT_COUNT; ++rn )
				{
					b2Transform xfA = poseA( variant % 3 );
					b2Transform xfB = poseB( an, dn, rn, 0.55f );

					b2SimplexCache cache = { 0 };
					printf( "MXK" );
					printChainSegment( &cs );
					printCapsule( &capsuleB );
					printTransform( xfA );
					printTransform( xfB );
					printCache( &cache );
					b2Manifold m = b2CollideChainSegmentAndCapsule( &cs, xfA, &capsuleB, xfB, &cache );
					pi( m.pointCount );
					ph( m.normal.x );
					ph( m.normal.y );
					for ( int i = 0; i < m.pointCount; ++i )
					{
						const b2ManifoldPoint* mp = m.points + i;
						ph( mp->point.x );
						ph( mp->point.y );
						ph( mp->anchorA.x );
						ph( mp->anchorA.y );
						ph( mp->anchorB.x );
						ph( mp->anchorB.y );
						ph( mp->separation );
						pi( (int)mp->id );
					}
					printf( " |" );
					printCache( &cache );
					printf( "\n" );
				}
			}
		}
	}
}

static void sweepChainSegmentAndPolygon( void )
{
	for ( int variant = 0; variant < 4; ++variant )
	{
		b2ChainSegment cs = makeChainSegment( variant );

		for ( int w = 0; w < 3; ++w )
		{
			b2Vec2 pts[8];
			int n;
			float radius = ( w % 2 ) == 1 ? 0.05f : 0.0f;
			b2Polygon polyB = makeLibraryPolygon( w + 1, radius, pts, &n );

			for ( int an = 0; an < ANGLE_COUNT; ++an )
			{
				for ( int dn = 0; dn < DEPTH_COUNT; ++dn )
				{
					for ( int rn = 0; rn < ROT_COUNT; ++rn )
					{
						b2Transform xfA = poseA( variant % 3 );
						b2Transform xfB = poseB( an, dn, rn, 0.65f );

						b2SimplexCache cache = { 0 };
						printf( "MXP" );
						printChainSegment( &cs );
						printPolySpec( pts, n, radius );
						printTransform( xfA );
						printTransform( xfB );
						printCache( &cache );
						b2Manifold m = b2CollideChainSegmentAndPolygon( &cs, xfA, &polyB, xfB, &cache );
						pi( m.pointCount );
						ph( m.normal.x );
						ph( m.normal.y );
						for ( int i = 0; i < m.pointCount; ++i )
						{
							const b2ManifoldPoint* mp = m.points + i;
							ph( mp->point.x );
							ph( mp->point.y );
							ph( mp->anchorA.x );
							ph( mp->anchorA.y );
							ph( mp->anchorB.x );
							ph( mp->anchorB.y );
							ph( mp->separation );
							pi( (int)mp->id );
						}
						printf( " |" );
						printCache( &cache );
						printf( "\n" );
					}
				}
			}
		}
	}
}

int main( void )
{
	sweepCircles( );
	sweepCapsuleAndCircle( );
	sweepCapsules( );
	sweepSegmentAndCapsule( );
	sweepPolygonAndCircle( );
	sweepPolygonAndCapsule( );
	sweepPolygons( );
	sweepSegmentAndCircle( );
	sweepSegmentAndPolygon( );
	sweepChainSegmentAndCircle( );
	sweepChainSegmentAndPolygon( );
	sweepChainSegmentAndCapsule( );
	return 0;
}
