// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/geometry.as

#include "collide.h"

#include "shape.h"

static void pmass( b2MassData m )
{
	vf32( m.mass );
	pv( m.center );
	vf32( m.rotationalInertia );
}

static void pplane( b2PlaneResult r )
{
	pv( r.plane.normal );
	vf32( r.plane.offset );
	pv( r.point );
	vbool( r.hit );
}

// A pose that is near the origin, or far from it where float rounding is coarse
static b2Transform randPose( void )
{
	b2Transform xf = randXf( 4.0f );
	if ( ( vrand32() & 3 ) == 0 )
	{
		float far = vrandf( -20000.0f, 20000.0f );
		xf.p.x += far;
	}
	return xf;
}

// Rays from around the shape toward its middle: zero length, scaled, starting inside, short
static b2RayCastInput randRay( void )
{
	b2RayCastInput input;
	int kind = vrandint( 0, 6 );
	b2Vec2 origin = randVec( -2.5f, 2.5f );
	b2Vec2 target = randVec( -0.75f, 0.75f );
	input.origin = origin;
	input.translation = b2Sub( target, origin );
	if ( kind == 0 )
	{
		input.translation = b2Vec2_zero;
	}
	else if ( kind == 1 )
	{
		float scale = vrandf( 0.25f, 3.0f );
		input.translation = b2MulSV( scale, input.translation );
	}
	else if ( kind == 2 )
	{
		input.origin = randVec( -0.25f, 0.25f );
	}
	input.maxFraction = vrandf( 0.0f, 1.5f );
	return input;
}

static b2ShapeCastInput randCastInput( void )
{
	b2Vec2 pts[8];
	int n = vrandint( 1, 9 );
	for ( int i = 0; i < n; ++i )
	{
		pts[i] = randVec( -0.375f, 0.375f );
	}
	float radius = randRadius( 0.25f );
	b2Vec2 position = randVec( -3.0f, 3.0f );
	b2Rot rotation = randRot();

	b2ShapeCastInput input;
	input.proxy = b2MakeOffsetProxy( pts, n, radius, position, rotation );
	b2Vec2 target = randVec( -0.5f, 0.5f );
	float scale = vrandf( 0.5f, 2.0f );
	input.translation = b2MulSV( scale, b2Sub( target, position ) );
	input.maxFraction = vrandf( 0.25f, 1.5f );
	input.canEncroach = ( vrand32() & 1 ) == 1;
	return input;
}

static b2Capsule randMover( void )
{
	b2Capsule mover;
	mover.center1 = randVec( -1.25f, 1.25f );
	b2Vec2 d = randVec( -0.5f, 0.5f );
	mover.center2 = b2Add( mover.center1, d );
	mover.radius = vrandf( 0.0625f, 0.375f );
	return mover;
}

static void makers( void )
{
	for ( int w = 0; w < 12; ++w )
	{
		b2Polygon p = libraryPolygon( w, ( w & 1 ) ? 0.0625f : 0.0f );
		vtag( "lib" );
		ppoly( &p );
		vend();
	}

	for ( int i = 0; i < 4000; ++i )
	{
		float hw = vrandf( 0.01f, 3.0f );
		float hh = vrandf( 0.01f, 3.0f );
		float radius = vrandf( 0.0f, 0.5f );
		b2Vec2 center = randVec( -5.0f, 5.0f );
		b2Rot rotation = randRot();

		b2Polygon p = b2MakeBox( hw, hh );
		vtag( "box" );
		ppoly( &p );
		vend();

		p = b2MakeSquare( hw );
		vtag( "square" );
		ppoly( &p );
		vend();

		p = b2MakeRoundedBox( hw, hh, radius );
		vtag( "rbox" );
		ppoly( &p );
		vend();

		p = b2MakeOffsetBox( hw, hh, center, rotation );
		vtag( "obox" );
		ppoly( &p );
		vend();

		p = b2MakeOffsetRoundedBox( hw, hh, center, rotation, radius );
		vtag( "orbox" );
		ppoly( &p );
		vend();

		b2Polygon q = randPolygon();
		vtag( "rpoly" );
		ppoly( &q );
		vend();

		b2Transform xf = randXf( 5.0f );
		p = b2TransformPolygon( xf, &q );
		vtag( "xpoly" );
		ppoly( &p );
		vend();

		b2Vec2 pts[8];
		int n = vrandint( 3, 9 );
		for ( int j = 0; j < n; ++j )
		{
			pts[j] = randVec( -1.0f, 1.0f );
		}
		b2Hull hull = b2ComputeHull( pts, n );
		if ( hull.count >= 3 )
		{
			p = b2MakeOffsetPolygon( &hull, center, rotation );
			vtag( "opoly" );
			ppoly( &p );
			vend();

			p = b2MakeOffsetRoundedPolygon( &hull, center, rotation, radius );
			vtag( "orpoly" );
			ppoly( &p );
			vend();
		}
	}

	// A bad hull falls back to a square when asserts are off
	b2Hull bad = { 0 };
	b2Polygon p = b2MakePolygon( &bad, 0.0f );
	vtag( "bad" );
	ppoly( &p );
	vend();
	p = b2MakeOffsetRoundedPolygon( &bad, b2Vec2_zero, b2Rot_identity, 0.25f );
	ppoly( &p );
	vend();
}

static void masses( void )
{
	for ( int i = 0; i < 5000; ++i )
	{
		float density = vrandf( 0.0f, 10.0f );

		b2Circle circle = randCircle();
		vtag( "mc" );
		pmass( b2ComputeCircleMass( &circle, density ) );
		vend();

		b2Capsule capsule = randCapsule();
		vtag( "mk" );
		pmass( b2ComputeCapsuleMass( &capsule, density ) );
		vend();

		b2Polygon polygon = randPolygon();
		vtag( "mp" );
		pmass( b2ComputePolygonMass( &polygon, density ) );
		vend();

		// One and two point polygons go through the circle and capsule paths
		b2Polygon small = { 0 };
		small.count = 1 + ( i & 1 );
		small.vertices[0] = randVec( -1.0f, 1.0f );
		small.vertices[1] = randVec( -1.0f, 1.0f );
		small.radius = vrandf( 0.0f, 0.5f );
		vtag( "ms" );
		pmass( b2ComputePolygonMass( &small, density ) );
		vend();
	}
}

static void bounds( void )
{
	for ( int i = 0; i < 5000; ++i )
	{
		b2Transform xf = randPose();
		float extra = vrandf( 0.0f, 0.25f );

		b2Shape shape = { 0 };
		shape.type = b2_circleShape;
		shape.circle = randCircle();
		vtag( "bc" );
		paabb( b2ComputeCircleBounds( &shape.circle, xf ) );
		paabb( b2ComputeFatShapeAABB( &shape, xf, extra ) );
		vend();

		shape.type = b2_capsuleShape;
		shape.capsule = randCapsule();
		vtag( "bk" );
		paabb( b2ComputeCapsuleBounds( &shape.capsule, xf ) );
		paabb( b2ComputeFatShapeAABB( &shape, xf, extra ) );
		vend();

		shape.type = b2_polygonShape;
		shape.polygon = randPolygon();
		vtag( "bp" );
		paabb( b2ComputePolygonBounds( &shape.polygon, xf ) );
		paabb( b2ComputeFatShapeAABB( &shape, xf, extra ) );
		vend();

		shape.type = b2_segmentShape;
		shape.segment = randSegment();
		vtag( "bs" );
		paabb( b2ComputeSegmentBounds( &shape.segment, xf ) );
		paabb( b2ComputeFatShapeAABB( &shape, xf, extra ) );
		vend();
	}
}

static void points( void )
{
	for ( int i = 0; i < 5000; ++i )
	{
		b2Circle circle = randCircle();
		b2Capsule capsule = randCapsule();
		if ( ( i & 7 ) == 0 )
		{
			capsule.center2 = capsule.center1;
		}
		b2Polygon polygon = randPolygon();

		for ( int j = 0; j < 4; ++j )
		{
			b2Vec2 point = randVec( -1.25f, 1.25f );
			vtag( "pt" );
			pv( point );
			vbool( b2PointInCircle( &circle, point ) );
			vbool( b2PointInCapsule( &capsule, point ) );
			vbool( b2PointInPolygon( &polygon, point ) );
			vend();
		}
	}
}

static void rays( void )
{
	for ( int i = 0; i < 5000; ++i )
	{
		b2RayCastInput input;
		input.origin = ( b2Vec2 ){ vspecialf(), 0.0f };
		input.origin.y = vspecialf();
		input.translation = randVec( -1.0f, 1.0f );
		input.maxFraction = vspecialf();
		vtag( "valid" );
		vbool( b2IsValidRay( &input ) );
		vend();
	}

	for ( int i = 0; i < 6000; ++i )
	{
		b2Circle circle = randCircle();
		b2RayCastInput input = randRay();
		vtag( "rc" );
		pcast( b2CastRayCircle( &circle, &input ) );
		vend();

		b2Capsule capsule = randCapsule();
		if ( ( i & 15 ) == 0 )
		{
			capsule.center2 = capsule.center1;
		}
		input = randRay();
		if ( ( i & 7 ) == 1 )
		{
			// Parallel to the capsule axis
			float scale = vrandf( -2.0f, 2.0f );
			input.translation = b2MulSV( scale, b2Sub( capsule.center2, capsule.center1 ) );
		}
		vtag( "rk" );
		pcast( b2CastRayCapsule( &capsule, &input ) );
		vend();

		b2Segment segment = randSegment();
		input = randRay();
		vtag( "rs" );
		pcast( b2CastRaySegment( &segment, &input, false ) );
		pcast( b2CastRaySegment( &segment, &input, true ) );
		vend();

		b2Polygon polygon = randPolygon();
		input = randRay();
		vtag( "rp" );
		pcast( b2CastRayPolygon( &polygon, &input ) );
		vend();
	}
}

static void casts( void )
{
	for ( int i = 0; i < 4000; ++i )
	{
		b2Circle circle = randCircle();
		b2ShapeCastInput input = randCastInput();
		vtag( "sc" );
		pproxy( &input.proxy );
		pcast( b2CastShapeCircle( &circle, &input ) );
		vend();

		b2Capsule capsule = randCapsule();
		input = randCastInput();
		vtag( "sk" );
		pcast( b2CastShapeCapsule( &capsule, &input ) );
		vend();

		b2Segment segment = randSegment();
		input = randCastInput();
		vtag( "ss" );
		pcast( b2CastShapeSegment( &segment, &input ) );
		vend();

		b2Polygon polygon = randPolygon();
		input = randCastInput();
		vtag( "sp" );
		pcast( b2CastShapePolygon( &polygon, &input ) );
		vend();
	}
}

static void movers( void )
{
	for ( int i = 0; i < 5000; ++i )
	{
		b2Capsule mover = randMover();

		b2Circle circle = randCircle();
		vtag( "mvc" );
		pplane( b2CollideMoverAndCircle( &mover, &circle ) );
		vend();

		b2Capsule capsule = randCapsule();
		vtag( "mvk" );
		pplane( b2CollideMoverAndCapsule( &mover, &capsule ) );
		vend();

		b2Polygon polygon = randPolygon();
		vtag( "mvp" );
		pplane( b2CollideMoverAndPolygon( &mover, &polygon ) );
		vend();

		b2Segment segment = randSegment();
		vtag( "mvs" );
		pplane( b2CollideMoverAndSegment( &mover, &segment ) );
		vend();
	}
}

int main( void )
{
	vseed( 31 );
	makers();
	masses();
	bounds();
	points();
	rays();
	casts();
	movers();
	return 0;
}
