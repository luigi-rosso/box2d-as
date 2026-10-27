// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/simd.as: the scalar tree helpers of simd.h and the B2_SIMD_NONE lane
// math of contact_solver_wide.inl, included here at width 4 as contact_solver_w4.c does

#define B2_SIMD_WIDTH 4
#include "contact_solver_wide.inl"

#include "simd.h"

#include "common.h"

static void pw( b2FloatW a )
{
	vf32( a.x );
	vf32( a.y );
	vf32( a.z );
	vf32( a.w );
}

static void pab( b2AABB a )
{
	vf32( a.lowerBound.x );
	vf32( a.lowerBound.y );
	vf32( a.upperBound.x );
	vf32( a.upperBound.y );
}

// mode 0 special, 1 tiny, 2 ordinary, 3 small integers so lanes tie and hit zero
static float rlane( int mode )
{
	if ( mode == 0 )
	{
		return vspecialf();
	}
	if ( mode == 1 )
	{
		return vrandf( -1.0e-3f, 1.0e-3f );
	}
	if ( mode == 2 )
	{
		return vrandf( -10.0f, 10.0f );
	}
	return (float)vrandint( -2, 3 );
}

static b2FloatW rw( void )
{
	b2FloatW a;
	int mode = (int)( vrand32() % 4 );
	a.x = rlane( mode );
	a.y = rlane( mode );
	a.z = rlane( mode );
	a.w = rlane( mode );
	return a;
}

// The scalar path's masks are 1.0 or 0.0
static b2FloatW rmask( void )
{
	b2FloatW m;
	m.x = (float)( vrand32() & 1 );
	m.y = (float)( vrand32() & 1 );
	m.z = (float)( vrand32() & 1 );
	m.w = (float)( vrand32() & 1 );
	return m;
}

static b2AABB raabb( void )
{
	b2AABB a;
	a.lowerBound.x = vrandf( -10.0f, 10.0f );
	a.lowerBound.y = vrandf( -10.0f, 10.0f );
	float ex = vrandf( -1.0f, 6.0f );
	float ey = vrandf( -1.0f, 6.0f );
	a.upperBound.x = a.lowerBound.x + ex;
	a.upperBound.y = a.lowerBound.y + ey;
	return a;
}

int main( void )
{
	vseed( 7 );

	for ( int i = 0; i < 20000; ++i )
	{
		b2FloatW a = rw();
		b2FloatW b = rw();
		b2FloatW c = rw();
		b2FloatW mask = rmask();
		b2FloatW mask2 = rmask();

		vtag( "in" );
		pw( a );
		pw( b );
		pw( c );
		pw( mask );
		vend();

		vtag( "arith" );
		pw( b2NegW( a ) );
		pw( b2AddW( a, b ) );
		pw( b2SubW( a, b ) );
		pw( b2MulW( a, b ) );
		pw( b2DivW( a, b ) );
		pw( b2MulAddW( a, b, c ) );
		pw( b2MulSubW( a, b, c ) );
		vend();

		vtag( "cmp" );
		pw( b2MinW( a, b ) );
		pw( b2MaxW( a, b ) );
		pw( b2SymClampW( a, b ) );
		pw( b2GreaterThanW( a, b ) );
		pw( b2LessThanW( a, b ) );
		pw( b2AndW( mask, mask2 ) );
		pw( b2AndW( b2GreaterThanW( a, b ), b2LessThanW( c, b ) ) );
		vbool( b2AnyTrueW( mask ) );
		vbool( b2AllZeroW( a ) );
		vbool( b2AllZeroW( mask ) );
		pw( b2BlendW( a, b, mask ) );
		pw( b2BlendW( a, b, b2GreaterThanW( a, c ) ) );
		vend();

		vtag( "lanes" );
		pw( b2UnpackLoW( a, b ) );
		pw( b2UnpackHiW( a, b ) );
		pw( b2SplatW( a.y ) );
		pw( b2SetW( c.w, b.z, a.y, c.x ) );
		pw( b2ZeroW() );
		float data[8];
		b2StoreW( data, a );
		b2StoreW( data + 4, b );
		pw( b2LoadU( data + 2 ) );
		int indexA[4];
		int indexB[4];
		for ( int k = 0; k < 4; ++k )
		{
			indexA[k] = vrandint( 0, 3 );
			indexB[k] = vrandint( 0, 3 );
		}
		pw( b2SoftMaskW( indexA, indexB ) );
		vend();

		b2Vec2W va = { a, b };
		b2Vec2W vb = { c, a };
		b2RotW q = { b, c };
		vtag( "vec" );
		b2Vec2W d = b2SubVW( va, vb );
		pw( d.X );
		pw( d.Y );
		b2Vec2W p = b2RightPerpW( va );
		pw( p.X );
		pw( p.Y );
		pw( b2DotW( va, vb ) );
		pw( b2CrossW( va, vb ) );
		b2Vec2W r = b2RotateVectorW( q, va );
		pw( r.X );
		pw( r.Y );
		vend();
	}

	for ( int i = 0; i < 5000; ++i )
	{
		b2TreeNode nodes[4] = { 0 };
		for ( int k = 0; k < 4; ++k )
		{
			nodes[k].aabb = raabb();
		}
		b2AABB a = raabb();
		b2AABB b = raabb();
		bool condition = ( vrand32() & 1 ) != 0;

		b2AABBV av = b2LoadAABBV( &a );
		vtag( "tree" );
		vbool( b2OverlapNode( av, nodes + 1 ) );
		vbool( b2OverlapV( &a, &b ) );
		pab( b2UnionV( a, b ) );
		pab( b2UnionPairV( nodes + 2 ) );
		b2AABB stored = b;
		b2StoreAABBV( &stored, av, condition );
		pab( stored );
		vi32( b2OverlapPairMask( nodes, nodes + 2 ) );
		vi32( b2OverlapNodePairMask( av, nodes + 2 ) );
		vend();
	}

	return 0;
}
