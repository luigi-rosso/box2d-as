// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/container.as

#include "container.h"

#include "common.h"

typedef struct Pair
{
	float a;
	int b;
} Pair;

b2DeclareArrayNative( Pair );

static void emitArrays( b2Array( int ) * ints, b2Array( Pair ) * pairs )
{
	vtag( "ints" );
	vi32( ints->count );
	for ( int i = 0; i < ints->count; ++i )
	{
		vi32( ints->data[i] );
	}
	vend();

	vtag( "pairs" );
	vi32( pairs->count );
	for ( int i = 0; i < pairs->count; ++i )
	{
		vf32( pairs->data[i].a );
		vi32( pairs->data[i].b );
	}
	vend();
}

int main( void )
{
	vseed( 6 );
	b2Array( int ) ints = { 0 };
	b2Array( Pair ) pairs = { 0 };
	int src[64];

	for ( int phase = 0; phase < 200; ++phase )
	{
		int ops = vrandint( 1, 20 );
		for ( int k = 0; k < ops; ++k )
		{
			uint32_t op = vrand32() % 8;
			if ( op == 0 )
			{
				int* slot = b2Array_Emplace( ints );
				int index = (int)( slot - ints.data );
				*slot = vrandint( -1000, 1000 );
				Pair* pair = b2Array_Emplace( pairs );
				int p = (int)( pair - pairs.data );
				float a = vrandf( -1.0f, 1.0f );
				*pair = (Pair){ a, index };
				vtag( "add" );
				vi32( index );
				vi32( p );
				vend();
			}
			else if ( op == 1 && ints.count > 0 )
			{
				int moved = b2Array_RemoveSwap( ints, vrandint( 0, ints.count ) );
				vtag( "rs" );
				vi32( moved );
				vend();
			}
			else if ( op == 2 && pairs.count > 0 )
			{
				int moved = b2Array_RemoveSwap( pairs, vrandint( 0, pairs.count ) );
				vtag( "rsp" );
				vi32( moved );
				vend();
			}
			else if ( op == 3 )
			{
				int n = vrandint( 0, 8 );
				for ( int i = 0; i < n; ++i )
				{
					src[i] = vrandint( 0, 50 );
				}
				for ( int i = 0; i < n; ++i )
				{
					b2Array_Push( ints, src[i] );
				}
			}
			else if ( op == 4 )
			{
				int oldCount = pairs.count;
				int n = vrandint( 0, oldCount + 4 );
				b2Array_Resize( pairs, n );
				for ( int i = oldCount; i < n; ++i )
				{
					pairs.data[i] = (Pair){ (float)i, -i };
				}
			}
			else if ( op == 5 && ints.count > 0 )
			{
				int last = b2Array_Pop( ints );
				vtag( "pop" );
				vi32( last );
				vend();
			}
			else if ( op == 6 )
			{
				int n = vrandint( 0, 6 );
				b2Array_ResizeAndSetZero( ints, n );
			}
			else if ( pairs.count > 1 )
			{
				int i = vrandint( 0, pairs.count );
				int j = vrandint( 0, pairs.count );
				B2_SWAP( pairs.data[i], pairs.data[j] );
			}
		}
		emitArrays( &ints, &pairs );
	}

	b2Array_Destroy( ints );
	b2Array_Destroy( pairs );
	return 0;
}
