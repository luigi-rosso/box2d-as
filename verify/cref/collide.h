// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Printers and seeded shape generators shared by the collision drivers (hull, geometry, distance,
// manifold, mover). verify/as/collide.as mirrors every function here.

#pragma once

#include "common.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

static void pv( b2Vec2 v )
{
	vf32( v.x );
	vf32( v.y );
}

static void pr( b2Rot q )
{
	vf32( q.c );
	vf32( q.s );
}

static void pxf( b2Transform t )
{
	pv( t.p );
	pr( t.q );
}

static void paabb( b2AABB a )
{
	pv( a.lowerBound );
	pv( a.upperBound );
}

static void ppoly( const b2Polygon* p )
{
	vi32( p->count );
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		pv( p->vertices[i] );
	}
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		pv( p->normals[i] );
	}
	pv( p->centroid );
	vf32( p->radius );
}

// Points past count are uninitialized in C's b2MakeProxy
static void pproxy( const b2ShapeProxy* p )
{
	vi32( p->count );
	for ( int i = 0; i < p->count; ++i )
	{
		pv( p->points[i] );
	}
	vf32( p->radius );
}

static void pcast( b2CastOutput o )
{
	pv( o.normal );
	pv( o.point );
	vf32( o.fraction );
	vi32( o.iterations );
	vbool( o.hit );
}

static void pcache( const b2SimplexCache* c )
{
	vi32( c->count );
	for ( int i = 0; i < 3; ++i )
	{
		vi32( c->indexA[i] );
	}
	for ( int i = 0; i < 3; ++i )
	{
		vi32( c->indexB[i] );
	}
}

static void pdist( b2DistanceOutput o )
{
	pv( o.pointA );
	pv( o.pointB );
	pv( o.normal );
	vf32( o.distance );
	vi32( o.iterations );
	vi32( o.simplexCount );
}

static void plocal( const b2LocalManifold* m )
{
	vi32( m->pointCount );
	pv( m->normal );
	for ( int i = 0; i < 2; ++i )
	{
		pv( m->points[i].point );
		vf32( m->points[i].separation );
		vi32( m->points[i].id );
	}
}

static b2Vec2 randVec( float lo, float hi )
{
	float x = vrandf( lo, hi );
	float y = vrandf( lo, hi );
	return ( b2Vec2 ){ x, y };
}

static b2Rot randRot( void )
{
	return b2MakeRot( vrandf( -3.25f, 3.25f ) );
}

static b2Transform randXf( float range )
{
	b2Transform t;
	t.p = randVec( -range, range );
	t.q = randRot();
	return t;
}

// A radius that is zero half the time
static float randRadius( float hi )
{
	if ( ( vrand32() & 1 ) == 0 )
	{
		return 0.0f;
	}
	return vrandf( 0.0f, hi );
}

// Ring of points at three radii, hulled. which picks the point count and phase.
static b2Polygon libraryPolygon( int which, float radius )
{
	static const int counts[6] = { 3, 4, 5, 6, 7, 8 };
	int n = counts[which % 6];
	b2Vec2 pts[8];
	for ( int i = 0; i < n; ++i )
	{
		float a = 2.0f * B2_PI * (float)i / (float)n + 0.125f * (float)which;
		float r = 0.4375f + 0.125f * (float)( i % 3 );
		b2CosSin cs = b2ComputeCosSin( a );
		pts[i] = ( b2Vec2 ){ r * cs.cosine, r * cs.sine };
	}
	b2Hull hull = b2ComputeHull( pts, n );
	return b2MakePolygon( &hull, radius );
}

static b2Polygon randPolygon( void )
{
	int n = vrandint( 3, 9 );
	b2Vec2 pts[8];
	for ( int i = 0; i < n; ++i )
	{
		pts[i] = randVec( -0.75f, 0.75f );
	}
	float radius = randRadius( 0.25f );
	b2Hull hull = b2ComputeHull( pts, n );
	if ( hull.count < 3 )
	{
		return b2MakeRoundedBox( 0.5f, 0.25f, radius );
	}
	return b2MakePolygon( &hull, radius );
}

static b2ShapeProxy randProxy( void )
{
	int n = vrandint( 1, 9 );
	b2Vec2 pts[8];
	for ( int i = 0; i < n; ++i )
	{
		pts[i] = randVec( -0.6f, 0.6f );
	}
	float radius = randRadius( 0.3f );
	return b2MakeProxy( pts, n, radius );
}

static b2Circle randCircle( void )
{
	b2Circle c;
	c.center = randVec( -0.25f, 0.25f );
	c.radius = vrandf( 0.0625f, 0.5f );
	return c;
}

static b2Capsule randCapsule( void )
{
	b2Capsule c;
	c.center1 = randVec( -0.6f, 0.6f );
	c.center2 = randVec( -0.6f, 0.6f );
	c.radius = vrandf( 0.03125f, 0.375f );
	return c;
}

static b2Segment randSegment( void )
{
	b2Segment s;
	s.point1 = randVec( -0.8f, 0.8f );
	s.point2 = randVec( -0.8f, 0.8f );
	return s;
}

#define RING_ANGLES 8
#define RING_DEPTHS 7
#define RING_ROTS 4

static const float ringDepths[RING_DEPTHS] = { -0.375f, -0.125f, -0.015625f, 0.0f, 0.015625f, 0.125f, 0.625f };
static const float ringRots[RING_ROTS] = { 0.0f, 0.375f, -1.25f, 2.875f };

// Pose of B at reach plus a depth along a ring angle, with one of four rotations
static b2Transform ringPose( int an, int dn, int rn, float reach )
{
	float a = 2.0f * B2_PI * (float)an / (float)RING_ANGLES + 0.0625f;
	float d = reach + ringDepths[dn];
	b2CosSin cs = b2ComputeCosSin( a );
	b2Transform xf;
	xf.p = ( b2Vec2 ){ d * cs.cosine, d * cs.sine };
	xf.q = b2MakeRot( ringRots[rn] );
	return xf;
}
