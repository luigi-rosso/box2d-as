// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/manifold.as

#include "collide.h"

static b2Transform poseA( int variant )
{
	b2Transform xf;
	xf.p = ( b2Vec2 ){ 0.03125f * (float)variant, -0.015625f * (float)variant };
	xf.q = b2MakeRot( 0.25f * (float)variant );
	return xf;
}

// Pose of B in A for ring cell (an, dn, rn) with A at one of three poses
static b2Transform gridPose( int v, int an, int dn, int rn, float reach )
{
	return b2InvMulTransforms( poseA( v ), ringPose( an, dn, rn, reach ) );
}

// Straight, convex corners, concave corners, mixed
static b2ChainSegment chainSegment( int variant )
{
	b2ChainSegment cs = { 0 };
	cs.segment.point1 = ( b2Vec2 ){ -0.625f, 0.0f };
	cs.segment.point2 = ( b2Vec2 ){ 0.625f, 0.0625f };
	if ( variant == 0 )
	{
		cs.ghost1 = ( b2Vec2 ){ -1.875f, -0.03125f };
		cs.ghost2 = ( b2Vec2 ){ 1.875f, 0.125f };
	}
	else if ( variant == 1 )
	{
		cs.ghost1 = ( b2Vec2 ){ -1.5f, 0.875f };
		cs.ghost2 = ( b2Vec2 ){ 1.5f, 1.0f };
	}
	else if ( variant == 2 )
	{
		cs.ghost1 = ( b2Vec2 ){ -1.5f, -0.875f };
		cs.ghost2 = ( b2Vec2 ){ 1.5f, -1.0f };
	}
	else
	{
		cs.ghost1 = ( b2Vec2 ){ -1.375f, 0.75f };
		cs.ghost2 = ( b2Vec2 ){ 1.625f, -0.6875f };
	}
	return cs;
}

static b2ChainSegment randChainSegment( void )
{
	b2ChainSegment cs = { 0 };
	cs.segment = randSegment();
	b2Vec2 d = b2Sub( cs.segment.point2, cs.segment.point1 );
	b2Vec2 bend1 = randVec( -1.0f, 1.0f );
	b2Vec2 bend2 = randVec( -1.0f, 1.0f );
	cs.ghost1 = b2Add( b2Sub( cs.segment.point1, d ), bend1 );
	cs.ghost2 = b2Add( b2Add( cs.segment.point2, d ), bend2 );
	return cs;
}

static void out( const char* tag, b2LocalManifold m )
{
	vtag( tag );
	plocal( &m );
	vend();
}

static void outCache( const char* tag, b2LocalManifold m, const b2SimplexCache* before, const b2SimplexCache* after )
{
	vtag( tag );
	pcache( before );
	plocal( &m );
	pcache( after );
	vend();
}

static void grids( void )
{
	for ( int v = 0; v < 3; ++v )
	{
		b2Circle circleA = { { 0.0625f, -0.03125f }, 0.375f };
		b2Circle circleB = { { -0.03125f, 0.0625f }, 0.3125f };
		b2Capsule capsuleA = { { -0.4375f, 0.0625f }, { 0.5f, -0.0625f }, 0.25f };
		b2Segment segmentA = { { -0.625f, 0.0625f }, { 0.6875f, -0.125f } };
		b2Polygon polyA = libraryPolygon( v + 1, ( v & 1 ) ? 0.0625f : 0.0f );

		for ( int an = 0; an < RING_ANGLES; ++an )
		{
			for ( int dn = 0; dn < RING_DEPTHS; ++dn )
			{
				for ( int rn = 0; rn < RING_ROTS; ++rn )
				{
					out( "gcc", b2CollideCircles( &circleA, &circleB, gridPose( v, an, dn, rn, 0.6875f ) ) );
					out( "gkc", b2CollideCapsuleAndCircle( &capsuleA, &circleB, gridPose( v, an, dn, rn, 0.5625f ) ) );
					out( "gsc", b2CollideSegmentAndCircle( &segmentA, &circleB, gridPose( v, an, dn, rn, 0.3125f ) ) );
					out( "gpc", b2CollidePolygonAndCircle( &polyA, &circleB, gridPose( v, an, dn, rn, 0.8125f ) ) );
				}
			}
		}
	}

	// Capsule B: general, equal and parallel, perpendicular, short
	b2Capsule capsulesB[4] = {
		{ { -0.375f, 0.03125f }, { 0.4375f, -0.0625f }, 0.15625f },
		{ { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.1875f },
		{ { 0.0f, -0.4375f }, { 0.0f, 0.5f }, 0.1875f },
		{ { -0.125f, 0.0f }, { 0.125f, 0.03125f }, 0.25f },
	};

	for ( int v = 0; v < 4; ++v )
	{
		b2Capsule capsuleA = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.1875f };
		b2Segment segmentA = { { -0.6875f, -0.0625f }, { 0.625f, 0.125f } };
		b2Polygon polyA = libraryPolygon( v + 2, ( v % 3 ) == 0 ? 0.0625f : 0.0f );
		b2Capsule capsuleB = capsulesB[v];

		for ( int an = 0; an < RING_ANGLES; ++an )
		{
			for ( int dn = 0; dn < RING_DEPTHS; ++dn )
			{
				for ( int rn = 0; rn < RING_ROTS; ++rn )
				{
					int va = v % 3;
					out( "gkk", b2CollideCapsules( &capsuleA, &capsuleB, gridPose( va, an, dn, rn, 0.75f ) ) );
					out( "gsk", b2CollideSegmentAndCapsule( &segmentA, &capsuleB, gridPose( va, an, dn, rn, 0.5625f ) ) );
					out( "gpk", b2CollidePolygonAndCapsule( &polyA, &capsuleB, gridPose( va, an, dn, rn, 0.875f ) ) );
				}
			}
		}
	}

	for ( int wa = 0; wa < 6; ++wa )
	{
		b2Polygon polyA = libraryPolygon( wa, ( wa & 1 ) ? 0.0625f : 0.0f );
		b2Segment segmentA = { { -0.8125f, 0.0f }, { 0.75f, 0.0625f } };
		for ( int wb = 0; wb < 6; ++wb )
		{
			b2Polygon polyB = libraryPolygon( wb + 2, ( wb % 3 ) == 0 ? 0.046875f : 0.0f );
			if ( wb == 5 )
			{
				polyB = b2MakeBox( 0.5f, 0.25f );
			}

			for ( int an = 0; an < RING_ANGLES; ++an )
			{
				for ( int dn = 0; dn < RING_DEPTHS; ++dn )
				{
					int rn = ( an + dn + wa + wb ) % RING_ROTS;
					int v = ( wa + wb ) % 3;
					out( "gpp", b2CollidePolygons( &polyA, &polyB, gridPose( v, an, dn, rn, 1.0f ) ) );
					if ( wa == 0 )
					{
						out( "gsp", b2CollideSegmentAndPolygon( &segmentA, &polyB, gridPose( v, an, dn, rn, 0.6875f ) ) );
					}
				}
			}
		}
	}

	// Box on box faces: aligned and nearly aligned edges where clipping and the vertex-vertex fallback meet
	for ( int i = 0; i < 64; ++i )
	{
		b2Polygon boxA = b2MakeBox( 0.5f, 0.5f );
		b2Polygon boxB = b2MakeRoundedBox( 0.5f, 0.25f, ( i & 1 ) ? 0.03125f : 0.0f );
		b2Transform xf;
		xf.p = ( b2Vec2 ){ 0.0625f * (float)( i % 8 ) - 0.25f, 0.75f + 0.0078125f * (float)( i / 8 ) - 0.03125f };
		xf.q = b2MakeRot( 0.001953125f * (float)( ( i % 5 ) - 2 ) );
		out( "gbb", b2CollidePolygons( &boxA, &boxB, xf ) );
	}

	for ( int v = 0; v < 4; ++v )
	{
		b2ChainSegment cs = chainSegment( v );
		b2Circle circleB = { { 0.0f, 0.03125f }, 0.25f };
		b2Capsule capsuleB = { { -0.3125f, 0.0f }, { 0.3125f, 0.0625f }, 0.1875f };

		for ( int w = 0; w < 3; ++w )
		{
			b2Polygon polyB = libraryPolygon( w + 1, ( w & 1 ) ? 0.0625f : 0.0f );
			b2SimplexCache warmK = { 0 };
			b2SimplexCache warmP = { 0 };

			for ( int an = 0; an < RING_ANGLES; ++an )
			{
				for ( int dn = 0; dn < RING_DEPTHS; ++dn )
				{
					for ( int rn = 0; rn < RING_ROTS; ++rn )
					{
						int va = v % 3;
						if ( w == 0 )
						{
							out( "gxc", b2CollideChainSegmentAndCircle( &cs, &circleB, gridPose( va, an, dn, rn, 0.5f ) ) );
						}

						b2SimplexCache cold = { 0 };
						b2SimplexCache before = cold;
						b2Transform xf = gridPose( va, an, dn, rn, 0.5625f );
						b2LocalManifold m = b2CollideChainSegmentAndCapsule( &cs, &capsuleB, xf, &cold );
						outCache( "gxk", m, &before, &cold );

						before = warmK;
						m = b2CollideChainSegmentAndCapsule( &cs, &capsuleB, xf, &warmK );
						outCache( "gxkw", m, &before, &warmK );

						cold = ( b2SimplexCache ){ 0 };
						before = cold;
						xf = gridPose( va, an, dn, rn, 0.6875f );
						m = b2CollideChainSegmentAndPolygon( &cs, &polyB, xf, &cold );
						outCache( "gxp", m, &before, &cold );

						before = warmP;
						m = b2CollideChainSegmentAndPolygon( &cs, &polyB, xf, &warmP );
						outCache( "gxpw", m, &before, &warmP );
					}
				}
			}
		}
	}
}

static void randoms( void )
{
	b2SimplexCache cacheK = { 0 };
	b2SimplexCache cacheP = { 0 };

	for ( int i = 0; i < 6000; ++i )
	{
		b2Circle circleA = randCircle();
		b2Circle circleB = randCircle();
		b2Capsule capsuleA = randCapsule();
		b2Capsule capsuleB = randCapsule();
		b2Segment segmentA = randSegment();
		b2Polygon polyA = randPolygon();
		b2Polygon polyB = randPolygon();
		b2ChainSegment cs = randChainSegment();
		b2Transform xf = randXf( 1.25f );

		out( "rcc", b2CollideCircles( &circleA, &circleB, xf ) );
		out( "rkc", b2CollideCapsuleAndCircle( &capsuleA, &circleB, xf ) );
		out( "rsc", b2CollideSegmentAndCircle( &segmentA, &circleB, xf ) );
		out( "rpc", b2CollidePolygonAndCircle( &polyA, &circleB, xf ) );
		out( "rkk", b2CollideCapsules( &capsuleA, &capsuleB, xf ) );
		out( "rsk", b2CollideSegmentAndCapsule( &segmentA, &capsuleB, xf ) );
		out( "rpk", b2CollidePolygonAndCapsule( &polyA, &capsuleB, xf ) );
		out( "rpp", b2CollidePolygons( &polyA, &polyB, xf ) );
		out( "rsp", b2CollideSegmentAndPolygon( &segmentA, &polyB, xf ) );
		out( "rxc", b2CollideChainSegmentAndCircle( &cs, &circleB, xf ) );

		// The polygon cache carries over only when the next polygon still has the cached vertices
		if ( ( vrand32() & 3 ) == 0 )
		{
			cacheK = ( b2SimplexCache ){ 0 };
		}
		b2SimplexCache before = cacheK;
		b2LocalManifold m = b2CollideChainSegmentAndCapsule( &cs, &capsuleB, xf, &cacheK );
		outCache( "rxk", m, &before, &cacheK );

		cacheP = ( b2SimplexCache ){ 0 };
		before = cacheP;
		m = b2CollideChainSegmentAndPolygon( &cs, &polyB, xf, &cacheP );
		outCache( "rxp", m, &before, &cacheP );

		// Warm repeat at a nearby pose
		b2Transform xf2 = xf;
		b2Vec2 nudge = randVec( -0.03125f, 0.03125f );
		xf2.p = b2Add( xf2.p, nudge );
		before = cacheP;
		m = b2CollideChainSegmentAndPolygon( &cs, &polyB, xf2, &cacheP );
		outCache( "rxpw", m, &before, &cacheP );
	}
}

// Polygons around the segment ends, where vertex-vertex and ghost corner cases live
static void chainCorners( void )
{
	for ( int i = 0; i < 20000; ++i )
	{
		b2ChainSegment cs = ( i & 1 ) ? chainSegment( ( i >> 1 ) & 3 ) : randChainSegment();
		b2Polygon polyB = randPolygon();
		b2Vec2 base = ( vrand32() & 1 ) ? cs.segment.point1 : cs.segment.point2;
		b2Transform xf;
		xf.p = b2Add( base, randVec( -1.0f, 1.0f ) );
		xf.q = randRot();
		b2SimplexCache cache = { 0 };
		b2LocalManifold m = b2CollideChainSegmentAndPolygon( &cs, &polyB, xf, &cache );
		vtag( "cx" );
		plocal( &m );
		pcache( &cache );
		vend();
	}
}

int main( void )
{
	vseed( 51 );
	grids();
	randoms();
	chainCorners();
	return 0;
}
