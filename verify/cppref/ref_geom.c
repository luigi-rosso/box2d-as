// Phase A geometry/hull/aabb reference sweeps against the canonical box2d
// sources. Lines are self-contained: raw inputs plus expected outputs.

#include "aabb.h"
#include "constants.h"
#include "refcommon.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

static b2Rot rrot( void )
{
	return b2MakeRot( frand( -B2_PI, B2_PI ) );
}

static void phv( b2Vec2 v )
{
	ph( v.x );
	ph( v.y );
}

static void phAabb( b2AABB a )
{
	phv( a.lowerBound );
	phv( a.upperBound );
}

static void phCast( b2CastOutput o )
{
	pi( o.hit ? 1 : 0 );
	ph( o.fraction );
	phv( o.normal );
	phv( o.point );
}

static void phPolyDump( const b2Polygon* p )
{
	pi( p->count );
	ph( p->radius );
	phv( p->centroid );
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		phv( p->vertices[i] );
	}
	for ( int i = 0; i < B2_MAX_POLYGON_VERTICES; ++i )
	{
		phv( p->normals[i] );
	}
}

static void phPts( const b2Vec2* pts, int n )
{
	pi( n );
	for ( int i = 0; i < n; ++i )
	{
		phv( pts[i] );
	}
}

// Point-set generator covering welding, collinearity, duplicates, and scale.
static int genPoints( int category, b2Vec2* pts )
{
	int n;
	switch ( category % 8 )
	{
		case 0:
			n = 3 + irand( 6 );
			for ( int i = 0; i < n; ++i )
			{
				pts[i].x = frand( -1.0f, 1.0f );
				pts[i].y = frand( -1.0f, 1.0f );
			}
			break;
		case 1:
			n = 3 + irand( 6 );
			for ( int i = 0; i < n; ++i )
			{
				pts[i].x = frand( -100.0f, 100.0f );
				pts[i].y = frand( -100.0f, 100.0f );
			}
			break;
		case 2:
			// tiny scale: aggressive welding kicks in
			n = 3 + irand( 6 );
			for ( int i = 0; i < n; ++i )
			{
				pts[i].x = frand( -0.02f, 0.02f );
				pts[i].y = frand( -0.02f, 0.02f );
			}
			break;
		case 3:
		{
			// square + interior points
			float h = frand( 0.5f, 3.0f );
			pts[0] = (b2Vec2){ -h, -h };
			pts[1] = (b2Vec2){ h, -h };
			pts[2] = (b2Vec2){ h, h };
			pts[3] = (b2Vec2){ -h, h };
			n = 4;
			int extra = irand( 4 );
			for ( int i = 0; i < extra && n < 8; ++i )
			{
				pts[n].x = frand( -0.9f, 0.9f ) * h;
				pts[n].y = frand( -0.9f, 0.9f ) * h;
				n++;
			}
			break;
		}
		case 4:
			// collinear
			n = 3 + irand( 5 );
			for ( int i = 0; i < n; ++i )
			{
				float t = frand( -2.0f, 2.0f );
				pts[i].x = t;
				pts[i].y = 0.5f * t + 1.0f;
			}
			break;
		case 5:
		{
			// regular n-gon
			n = 3 + irand( 5 );
			float radius = frand( 0.5f, 5.0f );
			for ( int i = 0; i < n; ++i )
			{
				float t = 2.0f * B2_PI * (float)i / (float)n;
				b2CosSin cs = b2ComputeCosSin( t );
				pts[i].x = radius * cs.cosine;
				pts[i].y = radius * cs.sine;
			}
			break;
		}
		case 6:
			// duplicate heavy
			n = 4 + irand( 4 );
			for ( int i = 0; i < n; ++i )
			{
				if ( i > 0 && irand( 2 ) == 0 )
				{
					pts[i] = pts[irand( i )];
				}
				else
				{
					pts[i].x = frand( -2.0f, 2.0f );
					pts[i].y = frand( -2.0f, 2.0f );
				}
			}
			break;
		default:
		{
			// two tight clusters
			b2Vec2 c1 = { frand( -1.0f, 1.0f ), frand( -1.0f, 1.0f ) };
			b2Vec2 c2 = { frand( -1.0f, 1.0f ), frand( -1.0f, 1.0f ) };
			n = 4 + irand( 4 );
			for ( int i = 0; i < n; ++i )
			{
				b2Vec2 c = ( i & 1 ) ? c1 : c2;
				pts[i].x = c.x + frand( -0.005f, 0.005f );
				pts[i].y = c.y + frand( -0.005f, 0.005f );
			}
			break;
		}
	}
	return n;
}

static void emitHulls( void )
{
	for ( int i = 0; i < 400; ++i )
	{
		b2Vec2 pts[8];
		int n = genPoints( i, pts );
		b2Hull hull = b2ComputeHull( pts, n );

		printf( "HULL" );
		phPts( pts, n );
		pi( hull.count );
		pi( hull.count > 0 ? ( b2ValidateHull( &hull ) ? 1 : 0 ) : 0 );
		for ( int k = 0; k < hull.count; ++k )
		{
			phv( hull.points[k] );
		}
		printf( "\n" );

		if ( hull.count >= 3 )
		{
			// polygon makers from the same raw points
			float radius = ( i & 1 ) ? frand( 0.01f, 0.5f ) : 0.0f;
			b2Polygon poly = b2MakePolygon( &hull, radius );
			printf( "MAKEP" );
			phPts( pts, n );
			ph( radius );
			phPolyDump( &poly );
			printf( "\n" );

			b2Vec2 c = { frand( -10.0f, 10.0f ), frand( -10.0f, 10.0f ) };
			b2Rot q = rrot( );
			b2Polygon opoly = b2MakeOffsetRoundedPolygon( &hull, c, q, radius );
			printf( "ORPOLY" );
			phPts( pts, n );
			phv( c );
			ph( q.c );
			ph( q.s );
			ph( radius );
			phPolyDump( &opoly );
			printf( "\n" );

			b2Transform xf = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) }, rrot( ) };
			b2Polygon tpoly = b2TransformPolygon( xf, &poly );
			printf( "TPOLY" );
			phPts( pts, n );
			ph( radius );
			phv( xf.p );
			ph( xf.q.c );
			ph( xf.q.s );
			phPolyDump( &tpoly );
			printf( "\n" );

			float density = frand( 0.1f, 5.0f );
			b2MassData mass = b2ComputePolygonMass( &poly, density );
			printf( "PMASS" );
			phPts( pts, n );
			ph( radius );
			ph( density );
			ph( mass.mass );
			phv( mass.center );
			ph( mass.rotationalInertia );
			printf( "\n" );

			b2AABB aabb = b2ComputePolygonAABB( &poly, xf );
			printf( "PAABB" );
			phPts( pts, n );
			ph( radius );
			phv( xf.p );
			ph( xf.q.c );
			ph( xf.q.s );
			phAabb( aabb );
			printf( "\n" );

			if ( radius == 0.0f )
			{
				for ( int r = 0; r < 3; ++r )
				{
					b2RayCastInput input;
					if ( r == 0 )
					{
						// aim from outside toward the centroid
						b2Vec2 dir = b2Normalize( (b2Vec2){ frand( -1.0f, 1.0f ), frand( -1.0f, 1.0f ) } );
						input.origin = b2MulAdd( poly.centroid, 150.0f, dir );
						input.translation = b2MulSV( -160.0f, dir );
					}
					else if ( r == 1 )
					{
						input.origin = poly.centroid;
						input.translation = (b2Vec2){ frand( -2.0f, 2.0f ), frand( -2.0f, 2.0f ) };
					}
					else
					{
						input.origin = (b2Vec2){ frand( -200.0f, 200.0f ), frand( -200.0f, 200.0f ) };
						input.translation = (b2Vec2){ frand( -100.0f, 100.0f ), frand( -100.0f, 100.0f ) };
					}
					input.maxFraction = 1.0f;
					b2CastOutput out = b2RayCastPolygon( &poly, &input );
					printf( "RCP" );
					phPts( pts, n );
					phv( input.origin );
					phv( input.translation );
					ph( input.maxFraction );
					phCast( out );
					printf( "\n" );
				}
			}
		}
	}

	// raw hull validation sweeps, including intentionally bad hulls
	for ( int i = 0; i < 200; ++i )
	{
		b2Hull hull;
		int mode = i % 4;
		if ( mode == 0 )
		{
			// genuine hull
			b2Vec2 pts[8];
			int n = genPoints( i, pts );
			hull = b2ComputeHull( pts, n );
			if ( hull.count < 3 )
			{
				continue;
			}
		}
		else if ( mode == 1 )
		{
			// clockwise square: winding violation
			float h = frand( 0.5f, 2.0f );
			hull.count = 4;
			hull.points[0] = (b2Vec2){ -h, -h };
			hull.points[1] = (b2Vec2){ -h, h };
			hull.points[2] = (b2Vec2){ h, h };
			hull.points[3] = (b2Vec2){ h, -h };
		}
		else if ( mode == 2 )
		{
			// collinear midpoint
			hull.count = 4;
			hull.points[0] = (b2Vec2){ 0.0f, 0.0f };
			hull.points[1] = (b2Vec2){ 1.0f, 0.0f };
			hull.points[2] = (b2Vec2){ 2.0f, 0.0f };
			hull.points[3] = (b2Vec2){ 1.0f, 1.5f };
		}
		else
		{
			// random garbage
			hull.count = 3 + irand( 6 );
			for ( int k = 0; k < hull.count; ++k )
			{
				hull.points[k].x = frand( -3.0f, 3.0f );
				hull.points[k].y = frand( -3.0f, 3.0f );
			}
		}
		printf( "VH" );
		phPts( hull.points, hull.count );
		pi( b2ValidateHull( &hull ) ? 1 : 0 );
		printf( "\n" );
	}
}

static void emitBoxes( void )
{
	for ( int i = 0; i < 150; ++i )
	{
		float hw = frand( 0.05f, 10.0f );
		float hh = frand( 0.05f, 10.0f );
		float r = frand( 0.0f, 1.0f );
		b2Vec2 c = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		b2Rot q = rrot( );

		b2Polygon box = b2MakeBox( hw, hh );
		printf( "BOX" );
		ph( hw );
		ph( hh );
		phPolyDump( &box );
		printf( "\n" );

		b2Polygon sq = b2MakeSquare( hw );
		printf( "SQ" );
		ph( hw );
		phPolyDump( &sq );
		printf( "\n" );

		b2Polygon rbox = b2MakeRoundedBox( hw, hh, r );
		printf( "RBOX" );
		ph( hw );
		ph( hh );
		ph( r );
		phPolyDump( &rbox );
		printf( "\n" );

		b2Polygon obox = b2MakeOffsetBox( hw, hh, c, q );
		printf( "OBOX" );
		ph( hw );
		ph( hh );
		phv( c );
		ph( q.c );
		ph( q.s );
		phPolyDump( &obox );
		printf( "\n" );

		b2Polygon orbox = b2MakeOffsetRoundedBox( hw, hh, c, q, r );
		printf( "ORBOX" );
		ph( hw );
		ph( hh );
		phv( c );
		ph( q.c );
		ph( q.s );
		ph( r );
		phPolyDump( &orbox );
		printf( "\n" );
	}
}

static void emitMass( void )
{
	for ( int i = 0; i < 300; ++i )
	{
		b2Circle circle = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) }, frand( 0.01f, 4.0f ) };
		float density = frand( 0.1f, 5.0f );
		b2MassData m = b2ComputeCircleMass( &circle, density );
		printf( "CMASS" );
		phv( circle.center );
		ph( circle.radius );
		ph( density );
		ph( m.mass );
		phv( m.center );
		ph( m.rotationalInertia );
		printf( "\n" );

		b2Capsule capsule = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) },
							  { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) },
							  frand( 0.01f, 2.0f ) };
		if ( i < 10 )
		{
			capsule.center2 = capsule.center1;
		}
		m = b2ComputeCapsuleMass( &capsule, density );
		printf( "KMASS" );
		phv( capsule.center1 );
		phv( capsule.center2 );
		ph( capsule.radius );
		ph( density );
		ph( m.mass );
		phv( m.center );
		ph( m.rotationalInertia );
		printf( "\n" );
	}
}

static void emitShapeAabbs( void )
{
	for ( int i = 0; i < 300; ++i )
	{
		b2Transform xf = { { frand( -30.0f, 30.0f ), frand( -30.0f, 30.0f ) }, rrot( ) };

		b2Circle circle = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) }, frand( 0.01f, 4.0f ) };
		b2AABB a = b2ComputeCircleAABB( &circle, xf );
		printf( "CAABB" );
		phv( circle.center );
		ph( circle.radius );
		phv( xf.p );
		ph( xf.q.c );
		ph( xf.q.s );
		phAabb( a );
		printf( "\n" );

		b2Capsule capsule = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) },
							  { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) },
							  frand( 0.01f, 2.0f ) };
		a = b2ComputeCapsuleAABB( &capsule, xf );
		printf( "KAABB" );
		phv( capsule.center1 );
		phv( capsule.center2 );
		ph( capsule.radius );
		phv( xf.p );
		ph( xf.q.c );
		ph( xf.q.s );
		phAabb( a );
		printf( "\n" );

		b2Segment seg = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) }, { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) } };
		a = b2ComputeSegmentAABB( &seg, xf );
		printf( "SAABB" );
		phv( seg.point1 );
		phv( seg.point2 );
		phv( xf.p );
		ph( xf.q.c );
		ph( xf.q.s );
		phAabb( a );
		printf( "\n" );
	}
}

static void emitPointTests( void )
{
	for ( int i = 0; i < 400; ++i )
	{
		b2Circle circle = { { frand( -3.0f, 3.0f ), frand( -3.0f, 3.0f ) }, frand( 0.1f, 3.0f ) };
		b2Vec2 p;
		if ( i < 50 )
		{
			// exact boundary: center + radius along x
			p.x = circle.center.x + circle.radius;
			p.y = circle.center.y;
		}
		else
		{
			p.x = frand( -6.0f, 6.0f );
			p.y = frand( -6.0f, 6.0f );
		}
		printf( "PIC" );
		phv( circle.center );
		ph( circle.radius );
		phv( p );
		pi( b2PointInCircle( &circle, p ) ? 1 : 0 );
		printf( "\n" );

		b2Capsule capsule = {
			{ frand( -3.0f, 3.0f ), frand( -3.0f, 3.0f ) }, { frand( -3.0f, 3.0f ), frand( -3.0f, 3.0f ) }, frand( 0.1f, 2.0f ) };
		if ( i % 7 == 0 )
		{
			capsule.center2 = capsule.center1;
		}
		p.x = frand( -6.0f, 6.0f );
		p.y = frand( -6.0f, 6.0f );
		printf( "PIK" );
		phv( capsule.center1 );
		phv( capsule.center2 );
		ph( capsule.radius );
		phv( p );
		pi( b2PointInCapsule( &capsule, p ) ? 1 : 0 );
		printf( "\n" );
	}
}

static void emitRayCasts( void )
{
	for ( int i = 0; i < 500; ++i )
	{
		b2RayCastInput input;
		input.origin = (b2Vec2){ frand( -10.0f, 10.0f ), frand( -10.0f, 10.0f ) };
		input.translation = (b2Vec2){ frand( -15.0f, 15.0f ), frand( -15.0f, 15.0f ) };
		input.maxFraction = ( i % 5 == 0 ) ? frand( 0.1f, 0.9f ) : 1.0f;
		if ( i % 11 == 0 )
		{
			input.translation = (b2Vec2){ 0.0f, 0.0f };
		}

		b2Circle circle = { { frand( -4.0f, 4.0f ), frand( -4.0f, 4.0f ) }, frand( 0.1f, 3.0f ) };
		b2CastOutput out = b2RayCastCircle( &circle, &input );
		printf( "RCC" );
		phv( input.origin );
		phv( input.translation );
		ph( input.maxFraction );
		phv( circle.center );
		ph( circle.radius );
		phCast( out );
		printf( "\n" );

		b2Capsule capsule = {
			{ frand( -4.0f, 4.0f ), frand( -4.0f, 4.0f ) }, { frand( -4.0f, 4.0f ), frand( -4.0f, 4.0f ) }, frand( 0.1f, 2.0f ) };
		if ( i % 13 == 0 )
		{
			capsule.center2 = capsule.center1;
		}
		out = b2RayCastCapsule( &capsule, &input );
		printf( "RCK" );
		phv( input.origin );
		phv( input.translation );
		ph( input.maxFraction );
		phv( capsule.center1 );
		phv( capsule.center2 );
		ph( capsule.radius );
		phCast( out );
		printf( "\n" );

		b2Segment seg = { { frand( -4.0f, 4.0f ), frand( -4.0f, 4.0f ) }, { frand( -4.0f, 4.0f ), frand( -4.0f, 4.0f ) } };
		if ( i % 17 == 0 )
		{
			seg.point2 = seg.point1;
		}
		int oneSided = irand( 2 );
		out = b2RayCastSegment( &seg, &input, oneSided != 0 );
		printf( "RCS" );
		phv( input.origin );
		phv( input.translation );
		ph( input.maxFraction );
		phv( seg.point1 );
		phv( seg.point2 );
		pi( oneSided );
		phCast( out );
		printf( "\n" );
	}
}

static void emitAabbOps( void )
{
	for ( int i = 0; i < 300; ++i )
	{
		b2Vec2 v1 = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		b2Vec2 v2 = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		b2AABB a = { b2Min( v1, v2 ), b2Max( v1, v2 ) };
		b2Vec2 v3 = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		b2Vec2 v4 = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		b2AABB b = { b2Min( v3, v4 ), b2Max( v3, v4 ) };

		b2AABB acopy = a;
		int changed = b2EnlargeAABB( &acopy, b ) ? 1 : 0;
		printf( "ENL" );
		phAabb( a );
		phAabb( b );
		pi( changed );
		phAabb( acopy );
		printf( "\n" );

		printf( "ABV" );
		phAabb( a );
		pi( b2IsValidAABB( a ) ? 1 : 0 );
		printf( "\n" );

		printf( "ABP" );
		phAabb( a );
		ph( b2Perimeter( a ) );
		printf( "\n" );
	}

	// invalid aabbs for ABV
	for ( int i = 0; i < 40; ++i )
	{
		b2AABB a = { { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) }, { frand( -5.0f, 5.0f ), frand( -5.0f, 5.0f ) } };
		if ( i % 4 == 0 )
		{
			a.lowerBound.x = NAN;
		}
		printf( "ABV" );
		phAabb( a );
		pi( b2IsValidAABB( a ) ? 1 : 0 );
		printf( "\n" );
	}

	for ( int i = 0; i < 500; ++i )
	{
		b2Vec2 v1 = { frand( -10.0f, 10.0f ), frand( -10.0f, 10.0f ) };
		b2Vec2 v2 = { frand( -10.0f, 10.0f ), frand( -10.0f, 10.0f ) };
		b2AABB a = { b2Min( v1, v2 ), b2Max( v1, v2 ) };

		b2Vec2 p1 = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		b2Vec2 p2 = { frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
		if ( i % 5 == 0 )
		{
			// axis-parallel ray
			p2.x = p1.x;
		}
		if ( i % 7 == 0 )
		{
			// start inside
			p1 = b2AABB_Center( a );
		}
		b2CastOutput out = b2AABB_RayCast( a, p1, p2 );
		printf( "ARC" );
		phAabb( a );
		phv( p1 );
		phv( p2 );
		phCast( out );
		printf( "\n" );
	}
}

int main( void )
{
	emitHulls( );
	emitBoxes( );
	emitMass( );
	emitShapeAabbs( );
	emitPointTests( );
	emitRayCasts( );
	emitAabbOps( );
	return 0;
}
