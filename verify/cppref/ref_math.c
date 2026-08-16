// Phase A math reference sweeps against the canonical box2d sources.
// Families and op numbers must stay in sync with _verifyA.as.

#include "constants.h"
#include "core.h"
#include "refcommon.h"

#include "box2d/id.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

#define POOL_COUNT 64
static float pool[POOL_COUNT];

static void initPool( void )
{
	int n = 0;
	pool[n++] = 0.0f;
	pool[n++] = -0.0f;
	pool[n++] = 1.0f;
	pool[n++] = -1.0f;
	pool[n++] = 0.5f;
	pool[n++] = -0.5f;
	pool[n++] = FLT_EPSILON;
	pool[n++] = -FLT_EPSILON;
	pool[n++] = FLT_MIN;
	pool[n++] = -FLT_MIN;
	pool[n++] = 1.401298464e-45f; // smallest denormal
	pool[n++] = -1.401298464e-45f;
	pool[n++] = 1e-20f;
	pool[n++] = -1e-20f;
	pool[n++] = B2_PI;
	pool[n++] = -B2_PI;
	pool[n++] = 0.5f * B2_PI;
	pool[n++] = -0.5f * B2_PI;
	pool[n++] = 2.0f * B2_PI;
	pool[n++] = 100.0f;
	pool[n++] = -100.0f;
	pool[n++] = 1e6f;
	pool[n++] = -1e6f;
	pool[n++] = 1e30f;
	pool[n++] = -1e30f;
	pool[n++] = 3.0f;
	pool[n++] = -7.25f;
	while ( n < POOL_COUNT )
	{
		float mag = frand( -6.0f, 6.0f );
		pool[n++] = frand( -1.0f, 1.0f ) * powf( 10.0f, mag );
	}
}

static float pickPool( void )
{
	return pool[irand( POOL_COUNT )];
}

static void emitF1( void )
{
	// op 0 unwind over grid + pool
	for ( int i = 0; i <= 4000; ++i )
	{
		float a = -40.0f + 80.0f * (float)i / 4000.0f;
		printf( "F1 0" );
		ph( a );
		ph( b2UnwindAngle( a ) );
		printf( "\n" );
	}
	for ( int i = 0; i < POOL_COUNT; ++i )
	{
		printf( "F1 0" );
		ph( pool[i] );
		ph( b2UnwindAngle( pool[i] ) );
		printf( "\n" );
		printf( "F1 1" );
		ph( pool[i] );
		ph( b2AbsFloat( pool[i] ) );
		printf( "\n" );
	}

	// op 2 isValidFloat including nan/inf
	float specials[4];
	specials[0] = NAN;
	specials[1] = INFINITY;
	specials[2] = -INFINITY;
	specials[3] = FLT_MAX;
	for ( int i = 0; i < POOL_COUNT; ++i )
	{
		printf( "F1 2" );
		ph( pool[i] );
		pi( b2IsValidFloat( pool[i] ) ? 1 : 0 );
		printf( "\n" );
	}
	for ( int i = 0; i < 4; ++i )
	{
		printf( "F1 2" );
		ph( specials[i] );
		pi( b2IsValidFloat( specials[i] ) ? 1 : 0 );
		printf( "\n" );
	}
}

static void emitF2F3( void )
{
	for ( int i = 0; i < 400; ++i )
	{
		float a = pickPool( );
		float b = pickPool( );
		printf( "F2 0" );
		ph( a );
		ph( b );
		ph( b2MinFloat( a, b ) );
		printf( "\n" );
		printf( "F2 1" );
		ph( a );
		ph( b );
		ph( b2MaxFloat( a, b ) );
		printf( "\n" );

		float lo = b2MinFloat( a, b );
		float hi = b2MaxFloat( a, b );
		float v = pickPool( );
		printf( "F3" );
		ph( v );
		ph( lo );
		ph( hi );
		ph( b2ClampFloat( v, lo, hi ) );
		printf( "\n" );
	}
}

static void emitAtan2( void )
{
	// axis cases and zero combos
	float axis[8] = { 0.0f, -0.0f, 1.0f, -1.0f, 0.5f, -0.5f, 1e-20f, -1e-20f };
	for ( int i = 0; i < 8; ++i )
	{
		for ( int j = 0; j < 8; ++j )
		{
			printf( "AT" );
			ph( axis[i] );
			ph( axis[j] );
			ph( b2Atan2( axis[i], axis[j] ) );
			printf( "\n" );
		}
	}

	// dense circle sweep at several magnitudes
	for ( int m = 0; m < 4; ++m )
	{
		float mag = powf( 10.0f, (float)( 2 * m ) - 3.0f );
		for ( int i = 0; i < 720; ++i )
		{
			float t = 2.0f * B2_PI * (float)i / 720.0f;
			float y = mag * sinf( t );
			float x = mag * cosf( t );
			printf( "AT" );
			ph( y );
			ph( x );
			ph( b2Atan2( y, x ) );
			printf( "\n" );
		}
	}

	for ( int i = 0; i < 2000; ++i )
	{
		float y = pickPool( );
		float x = pickPool( );
		printf( "AT" );
		ph( y );
		ph( x );
		ph( b2Atan2( y, x ) );
		printf( "\n" );
	}
}

static void emitCS( float a )
{
	b2CosSin cs = b2ComputeCosSin( a );
	printf( "CS" );
	ph( a );
	ph( cs.cosine );
	ph( cs.sine );
	printf( "\n" );
}

static void emitCosSin( void )
{
	for ( int i = 0; i <= 8000; ++i )
	{
		emitCS( -8.0f * B2_PI + 16.0f * B2_PI * (float)i / 8000.0f );
	}

	// branch boundaries in the Bhaskara split and near-zero rotations
	float b[14] = { 0.0f,
					-0.0f,
					0.5f * B2_PI,
					-0.5f * B2_PI,
					B2_PI,
					-B2_PI,
					nextafterf( 0.5f * B2_PI, 4.0f ),
					nextafterf( 0.5f * B2_PI, 0.0f ),
					nextafterf( -0.5f * B2_PI, -4.0f ),
					nextafterf( -0.5f * B2_PI, 0.0f ),
					1e-8f,
					-1e-8f,
					1.401298464e-45f,
					-1.401298464e-45f };
	for ( int i = 0; i < 14; ++i )
	{
		emitCS( b[i] );
	}

	for ( int i = 0; i < 1000; ++i )
	{
		emitCS( frand( -1000.0f, 1000.0f ) );
	}
}

static void emitSpring( void )
{
	for ( int i = 0; i < 500; ++i )
	{
		float h = frand( 0.0f, 120.0f );
		float d = frand( 0.0f, 4.0f );
		float p = frand( -10.0f, 10.0f );
		float v = frand( -50.0f, 50.0f );
		float t = frand( 1.0f / 240.0f, 1.0f / 30.0f );
		printf( "SPR" );
		ph( h );
		ph( d );
		ph( p );
		ph( v );
		ph( t );
		ph( b2SpringDamper( h, d, p, v, t ) );
		printf( "\n" );
	}
}

static b2Vec2 rv( void )
{
	b2Vec2 v = { pickPool( ), pickPool( ) };
	return v;
}

static b2Vec2 rvSmall( void )
{
	b2Vec2 v = { frand( -100.0f, 100.0f ), frand( -100.0f, 100.0f ) };
	return v;
}

static void emitVecOps( void )
{
	for ( int op = 0; op <= 16; ++op )
	{
		for ( int i = 0; i < 300; ++i )
		{
			b2Vec2 a = ( i & 1 ) ? rv( ) : rvSmall( );
			b2Vec2 b = ( i & 2 ) ? rv( ) : rvSmall( );
			float s = pickPool( );
			if ( i < 8 )
			{
				// force degenerate vectors through normalize paths
				a.x = ( i & 1 ) ? 0.0f : 1e-23f;
				a.y = ( i & 2 ) ? -0.0f : -1e-23f;
			}
			b2Vec2 o;
			switch ( op )
			{
				case 0:
					o = b2Add( a, b );
					break;
				case 1:
					o = b2Sub( a, b );
					break;
				case 2:
					o = b2Neg( a );
					break;
				case 3:
					o = b2Lerp( a, b, s );
					break;
				case 4:
					o = b2Mul( a, b );
					break;
				case 5:
					o = b2MulSV( s, a );
					break;
				case 6:
					o = b2MulAdd( a, s, b );
					break;
				case 7:
					o = b2MulSub( a, s, b );
					break;
				case 8:
					o = b2Abs( a );
					break;
				case 9:
					o = b2Min( a, b );
					break;
				case 10:
					o = b2Max( a, b );
					break;
				case 11:
					o = b2Normalize( a );
					break;
				case 12:
					o = b2CrossVS( a, s );
					break;
				case 13:
					o = b2CrossSV( s, a );
					break;
				case 14:
					o = b2LeftPerp( a );
					break;
				case 15:
					o = b2RightPerp( a );
					break;
				default:
				{
					float len;
					o = b2GetLengthAndNormalize( &len, a );
					break;
				}
			}
			printf( "V %d", op );
			ph( a.x );
			ph( a.y );
			ph( b.x );
			ph( b.y );
			ph( s );
			ph( o.x );
			ph( o.y );
			printf( "\n" );
		}
	}

	for ( int i = 0; i < 200; ++i )
	{
		b2Vec2 v = rvSmall( );
		b2Vec2 a = rvSmall( );
		b2Vec2 b = rvSmall( );
		b2Vec2 lo = b2Min( a, b );
		b2Vec2 hi = b2Max( a, b );
		b2Vec2 o = b2Clamp( v, lo, hi );
		printf( "VC" );
		ph( v.x );
		ph( v.y );
		ph( lo.x );
		ph( lo.y );
		ph( hi.x );
		ph( hi.y );
		ph( o.x );
		ph( o.y );
		printf( "\n" );
	}

	for ( int op = 0; op <= 8; ++op )
	{
		for ( int i = 0; i < 300; ++i )
		{
			b2Vec2 a = ( i & 1 ) ? rv( ) : rvSmall( );
			b2Vec2 b = rvSmall( );
			if ( op == 6 && i < 100 )
			{
				// walk the isNormalized tolerance boundary
				float qq = 1.0f + (float)( i - 50 ) * FLT_EPSILON * 4.0f;
				a.x = sqrtf( qq );
				a.y = 0.0f;
			}
			float out;
			int outIsBool = 0;
			int flag = 0;
			switch ( op )
			{
				case 0:
					out = b2Dot( a, b );
					break;
				case 1:
					out = b2Cross( a, b );
					break;
				case 2:
					out = b2Length( a );
					break;
				case 3:
					out = b2Distance( a, b );
					break;
				case 4:
					out = b2LengthSquared( a );
					break;
				case 5:
					out = b2DistanceSquared( a, b );
					break;
				case 6:
					outIsBool = 1;
					flag = b2IsNormalized( a ) ? 1 : 0;
					out = 0;
					break;
				case 7:
				{
					float len;
					b2GetLengthAndNormalize( &len, a );
					out = len;
					break;
				}
				default:
					outIsBool = 1;
					flag = b2IsValidVec2( a ) ? 1 : 0;
					out = 0;
					break;
			}
			printf( "VF %d", op );
			ph( a.x );
			ph( a.y );
			ph( b.x );
			ph( b.y );
			if ( outIsBool )
			{
				pi( flag );
			}
			else
			{
				ph( out );
			}
			printf( "\n" );
		}
	}
}

static b2Rot rrot( void )
{
	return b2MakeRot( frand( -B2_PI, B2_PI ) );
}

static void emitRotOps( void )
{
	// op 0 makeRot dense grid rides on the cos/sin coverage but checks the pair
	for ( int i = 0; i <= 2000; ++i )
	{
		float a = -8.0f + 16.0f * (float)i / 2000.0f;
		b2Rot o = b2MakeRot( a );
		printf( "R 0" );
		ph( a );
		ph( 0.0f );
		ph( 0.0f );
		ph( 0.0f );
		ph( o.c );
		ph( o.s );
		printf( "\n" );
	}

	for ( int op = 1; op <= 7; ++op )
	{
		for ( int i = 0; i < 300; ++i )
		{
			b2Rot q = rrot( );
			b2Rot r = rrot( );
			float a = q.c;
			float b = q.s;
			float c = r.c;
			float d = r.s;
			if ( op == 1 )
			{
				// feed unnormalized rotations
				a = frand( -2.0f, 2.0f );
				b = frand( -2.0f, 2.0f );
				if ( i == 0 )
				{
					a = 0.0f;
					b = 0.0f;
				}
				q.c = a;
				q.s = b;
			}
			if ( op == 2 )
			{
				// deltaAngle in third slot, including tiny steps
				c = ( i < 50 ) ? frand( -1e-5f, 1e-5f ) : frand( -0.5f, 0.5f );
			}
			b2Rot o;
			switch ( op )
			{
				case 1:
					o = b2NormalizeRot( q );
					break;
				case 2:
					o = b2IntegrateRotation( q, c );
					break;
				case 3:
					o = b2MulRot( q, r );
					break;
				case 4:
					o = b2InvMulRot( q, r );
					break;
				case 5:
					o = b2InvertRot( q );
					break;
				case 6:
				{
					b2Vec2 u = { q.c, q.s };
					o = b2MakeRotFromUnitVector( u );
					break;
				}
				default:
				{
					b2Vec2 u1 = { q.c, q.s };
					b2Vec2 u2 = { r.c, r.s };
					o = b2ComputeRotationBetweenUnitVectors( u1, u2 );
					break;
				}
			}
			printf( "R %d", op );
			ph( a );
			ph( b );
			ph( c );
			ph( d );
			ph( o.c );
			ph( o.s );
			printf( "\n" );
		}
	}

	for ( int i = 0; i < 300; ++i )
	{
		b2Rot q = rrot( );
		b2Rot r = rrot( );
		float t = frand( 0.0f, 1.0f );
		b2Rot o = b2NLerp( q, r, t );
		printf( "NL" );
		ph( q.c );
		ph( q.s );
		ph( r.c );
		ph( r.s );
		ph( t );
		ph( o.c );
		ph( o.s );
		printf( "\n" );
	}

	for ( int op = 0; op <= 4; ++op )
	{
		for ( int i = 0; i < 300; ++i )
		{
			b2Rot q = rrot( );
			b2Rot r = rrot( );
			float e = frand( 30.0f, 240.0f );
			if ( op >= 3 && i < 100 )
			{
				// walk the normalization tolerance boundary
				float qq = 1.0f + (float)( i - 50 ) * 0.0006f * 0.05f;
				q.c = sqrtf( qq );
				q.s = 0.0f;
			}
			printf( "RF %d", op );
			ph( q.c );
			ph( q.s );
			ph( r.c );
			ph( r.s );
			ph( e );
			switch ( op )
			{
				case 0:
					ph( b2Rot_GetAngle( q ) );
					break;
				case 1:
					ph( b2RelativeAngle( q, r ) );
					break;
				case 2:
					ph( b2ComputeAngularVelocity( q, r, e ) );
					break;
				case 3:
					pi( b2IsNormalizedRot( q ) ? 1 : 0 );
					break;
				default:
					pi( b2IsValidRotation( q ) ? 1 : 0 );
					break;
			}
			printf( "\n" );
		}
	}

	for ( int op = 0; op <= 3; ++op )
	{
		for ( int i = 0; i < 300; ++i )
		{
			b2Rot q = rrot( );
			b2Vec2 v = rvSmall( );
			b2Vec2 o;
			switch ( op )
			{
				case 0:
					o = b2RotateVector( q, v );
					break;
				case 1:
					o = b2InvRotateVector( q, v );
					break;
				case 2:
					o = b2Rot_GetXAxis( q );
					break;
				default:
					o = b2Rot_GetYAxis( q );
					break;
			}
			printf( "RV %d", op );
			ph( q.c );
			ph( q.s );
			ph( v.x );
			ph( v.y );
			ph( o.x );
			ph( o.y );
			printf( "\n" );
		}
	}
}

static b2Transform rxf( void )
{
	b2Transform t;
	t.p.x = frand( -50.0f, 50.0f );
	t.p.y = frand( -50.0f, 50.0f );
	t.q = rrot( );
	return t;
}

static void emitTransformOps( void )
{
	for ( int op = 0; op <= 1; ++op )
	{
		for ( int i = 0; i < 400; ++i )
		{
			b2Transform t = rxf( );
			b2Vec2 v = rvSmall( );
			b2Vec2 o = op == 0 ? b2TransformPoint( t, v ) : b2InvTransformPoint( t, v );
			printf( "TP %d", op );
			ph( t.p.x );
			ph( t.p.y );
			ph( t.q.c );
			ph( t.q.s );
			ph( v.x );
			ph( v.y );
			ph( o.x );
			ph( o.y );
			printf( "\n" );
		}
	}

	for ( int op = 0; op <= 1; ++op )
	{
		for ( int i = 0; i < 400; ++i )
		{
			b2Transform A = rxf( );
			b2Transform B = rxf( );
			b2Transform C = op == 0 ? b2MulTransforms( A, B ) : b2InvMulTransforms( A, B );
			printf( "TT %d", op );
			ph( A.p.x );
			ph( A.p.y );
			ph( A.q.c );
			ph( A.q.s );
			ph( B.p.x );
			ph( B.p.y );
			ph( B.q.c );
			ph( B.q.s );
			ph( C.p.x );
			ph( C.p.y );
			ph( C.q.c );
			ph( C.q.s );
			printf( "\n" );
		}
	}

	for ( int i = 0; i < 100; ++i )
	{
		b2Transform t = rxf( );
		if ( i % 3 == 1 )
		{
			t.q.c *= 2.0f;
		}
		if ( i % 3 == 2 )
		{
			t.p.x = NAN;
		}
		printf( "TV" );
		ph( t.p.x );
		ph( t.p.y );
		ph( t.q.c );
		ph( t.q.s );
		pi( b2IsValidTransform( t ) ? 1 : 0 );
		printf( "\n" );
	}
}

static void emitMatOps( void )
{
	for ( int op = 0; op <= 1; ++op )
	{
		for ( int i = 0; i < 300; ++i )
		{
			b2Mat22 A;
			A.cx = rvSmall( );
			A.cy = rvSmall( );
			if ( i < 20 )
			{
				// exactly singular: cy = 2 * cx
				A.cy.x = 2.0f * A.cx.x;
				A.cy.y = 2.0f * A.cx.y;
			}
			b2Vec2 v = rvSmall( );
			b2Vec2 o = op == 0 ? b2MulMV( A, v ) : b2Solve22( A, v );
			printf( "M %d", op );
			ph( A.cx.x );
			ph( A.cx.y );
			ph( A.cy.x );
			ph( A.cy.y );
			ph( v.x );
			ph( v.y );
			ph( o.x );
			ph( o.y );
			printf( "\n" );
		}
	}

	for ( int i = 0; i < 300; ++i )
	{
		b2Mat22 A;
		A.cx = rvSmall( );
		A.cy = rvSmall( );
		if ( i < 20 )
		{
			A.cy.x = 2.0f * A.cx.x;
			A.cy.y = 2.0f * A.cx.y;
		}
		b2Mat22 O = b2GetInverse22( A );
		printf( "MI" );
		ph( A.cx.x );
		ph( A.cx.y );
		ph( A.cy.x );
		ph( A.cy.y );
		ph( O.cx.x );
		ph( O.cx.y );
		ph( O.cy.x );
		ph( O.cy.y );
		printf( "\n" );
	}
}

static b2AABB raabb( void )
{
	b2Vec2 a = rvSmall( );
	b2Vec2 b = rvSmall( );
	b2AABB r;
	r.lowerBound = b2Min( a, b );
	r.upperBound = b2Max( a, b );
	return r;
}

static void emitAabbMathOps( void )
{
	for ( int i = 0; i < 400; ++i )
	{
		b2AABB a = raabb( );
		b2AABB b = raabb( );
		if ( i < 50 )
		{
			// touching edges: containment and overlap boundaries exact
			b.lowerBound.x = a.upperBound.x;
		}
		if ( i >= 50 && i < 100 )
		{
			b = a;
		}
		printf( "AB 0" );
		ph( a.lowerBound.x );
		ph( a.lowerBound.y );
		ph( a.upperBound.x );
		ph( a.upperBound.y );
		ph( b.lowerBound.x );
		ph( b.lowerBound.y );
		ph( b.upperBound.x );
		ph( b.upperBound.y );
		pi( b2AABB_Contains( a, b ) ? 1 : 0 );
		printf( "\n" );
		printf( "AB 1" );
		ph( a.lowerBound.x );
		ph( a.lowerBound.y );
		ph( a.upperBound.x );
		ph( a.upperBound.y );
		ph( b.lowerBound.x );
		ph( b.lowerBound.y );
		ph( b.upperBound.x );
		ph( b.upperBound.y );
		pi( b2AABB_Overlaps( a, b ) ? 1 : 0 );
		printf( "\n" );
	}

	for ( int i = 0; i < 200; ++i )
	{
		b2AABB a = raabb( );
		b2AABB b = raabb( );
		b2AABB u = b2AABB_Union( a, b );
		printf( "ABU" );
		ph( a.lowerBound.x );
		ph( a.lowerBound.y );
		ph( a.upperBound.x );
		ph( a.upperBound.y );
		ph( b.lowerBound.x );
		ph( b.lowerBound.y );
		ph( b.upperBound.x );
		ph( b.upperBound.y );
		ph( u.lowerBound.x );
		ph( u.lowerBound.y );
		ph( u.upperBound.x );
		ph( u.upperBound.y );
		printf( "\n" );

		b2Vec2 c = b2AABB_Center( a );
		printf( "ABC" );
		ph( a.lowerBound.x );
		ph( a.lowerBound.y );
		ph( a.upperBound.x );
		ph( a.upperBound.y );
		ph( c.x );
		ph( c.y );
		printf( "\n" );

		b2Vec2 e = b2AABB_Extents( a );
		printf( "ABX" );
		ph( a.lowerBound.x );
		ph( a.lowerBound.y );
		ph( a.upperBound.x );
		ph( a.upperBound.y );
		ph( e.x );
		ph( e.y );
		printf( "\n" );
	}

	// b2MakeAABB point clouds
	for ( int i = 0; i < 100; ++i )
	{
		int count = 1 + irand( 12 );
		b2Vec2 pts[12];
		printf( "MKAABB %d", count );
		for ( int k = 0; k < count; ++k )
		{
			pts[k] = rvSmall( );
		}
		float radius = ( i & 1 ) ? frand( 0.0f, 5.0f ) : 0.0f;
		ph( radius );
		for ( int k = 0; k < count; ++k )
		{
			ph( pts[k].x );
			ph( pts[k].y );
		}
		b2AABB a = b2MakeAABB( pts, count, radius );
		ph( a.lowerBound.x );
		ph( a.lowerBound.y );
		ph( a.upperBound.x );
		ph( a.upperBound.y );
		printf( "\n" );
	}
}

static void emitPlaneOps( void )
{
	for ( int i = 0; i < 200; ++i )
	{
		b2Plane p;
		p.normal = b2Rot_GetXAxis( rrot( ) );
		p.offset = frand( -20.0f, 20.0f );
		b2Vec2 pt = rvSmall( );
		printf( "PL" );
		ph( p.normal.x );
		ph( p.normal.y );
		ph( p.offset );
		ph( pt.x );
		ph( pt.y );
		ph( b2PlaneSeparation( p, pt ) );
		printf( "\n" );

		if ( i % 3 == 1 )
		{
			p.normal.x *= 3.0f;
		}
		if ( i % 3 == 2 )
		{
			p.offset = NAN;
		}
		printf( "PLV" );
		ph( p.normal.x );
		ph( p.normal.y );
		ph( p.offset );
		pi( b2IsValidPlane( p ) ? 1 : 0 );
		printf( "\n" );
	}
}

static void emitConstants( void )
{
	printf( "K 0" );
	ph( B2_LINEAR_SLOP );
	printf( "\n" );
	printf( "K 1" );
	ph( B2_HUGE );
	printf( "\n" );
	printf( "K 2" );
	ph( B2_AABB_MARGIN );
	printf( "\n" );
	printf( "K 3" );
	ph( B2_SPECULATIVE_DISTANCE );
	printf( "\n" );
	printf( "K 4" );
	ph( B2_MAX_ROTATION );
	printf( "\n" );
	printf( "K 5" );
	ph( B2_TIME_TO_SLEEP );
	printf( "\n" );

	// scaled units: emit slop/huge/margin/speculative under 2.5 then restore
	b2SetLengthUnitsPerMeter( 2.5f );
	printf( "LUPM" );
	ph( 2.5f );
	ph( B2_LINEAR_SLOP );
	ph( B2_HUGE );
	ph( B2_AABB_MARGIN );
	ph( B2_SPECULATIVE_DISTANCE );
	printf( "\n" );
	b2SetLengthUnitsPerMeter( 1.0f );
}

static void emitIds( void )
{
	for ( int i = 0; i < 100; ++i )
	{
		uint16_t idx = (uint16_t)rnd( );
		uint16_t gen = (uint16_t)rnd( );
		b2WorldId w = { idx, gen };
		uint32_t packed = b2StoreWorldId( w );
		b2WorldId back = b2LoadWorldId( packed );
		printf( "IDW %u %u %08x %u %u\n", idx, gen, packed, back.index1, back.generation );

		int32_t bidx = (int32_t)rnd( );
		uint16_t w0 = (uint16_t)rnd( );
		uint16_t bgen = (uint16_t)rnd( );
		b2BodyId bid = { bidx, w0, bgen };
		uint64_t bp = b2StoreBodyId( bid );
		b2BodyId bback = b2LoadBodyId( bp );
		printf( "IDB %d %u %u", bidx, w0, bgen );
		pu64( bp );
		printf( " %d %u %u\n", bback.index1, bback.world0, bback.generation );

		int32_t cidx = (int32_t)rnd( );
		uint16_t cw = (uint16_t)rnd( );
		uint32_t cgen = rnd( );
		b2ContactId cid = { cidx, cw, 0, cgen };
		uint32_t values[3];
		b2StoreContactId( cid, values );
		b2ContactId cback = b2LoadContactId( values );
		printf( "IDC %d %u %u %08x %08x %08x %d %u %u\n", cidx, cw, cgen, values[0], values[1], values[2], cback.index1,
				cback.world0, cback.generation );
	}
}

int main( void )
{
	initPool( );
	emitF1( );
	emitF2F3( );
	emitAtan2( );
	emitCosSin( );
	emitSpring( );
	emitVecOps( );
	emitRotOps( );
	emitTransformOps( );
	emitMatOps( );
	emitAabbMathOps( );
	emitPlaneOps( );
	emitConstants( );
	emitIds( );
	return 0;
}
