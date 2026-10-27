// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/ctz.as

#include "platform.h"

#include "common.h"

// Mixes powers of two and their neighbours into the random draws.
static uint32_t pickU32( void )
{
	uint32_t kind = vrand32() % 4;
	uint32_t shift = vrand32() % 32;
	if ( kind == 0 )
	{
		return 1u << shift;
	}
	if ( kind == 1 )
	{
		return ( 1u << shift ) + 1u;
	}
	if ( kind == 2 )
	{
		return ( 1u << shift ) - 1u;
	}
	return vrand32() >> ( vrand32() % 32 );
}

int main( void )
{
	vseed( 1 );
	for ( int i = 0; i < 20000; ++i )
	{
		uint32_t a = pickU32();
		uint64_t b = ( (uint64_t)pickU32() << ( vrand32() % 33 ) ) | pickU32();
		vtag( "u" );
		vu32( a );
		vu64( b );
		if ( a != 0 )
		{
			vu32( b2CTZ32( a ) );
			vu32( b2CLZ32( a ) );
		}
		if ( b != 0 )
		{
			vu32( b2CTZ64( b ) );
		}
		vi32( b2PopCount64( b ) );
		vend();

		int x = (int)( a & 0x3fffffff );
		vtag( "p" );
		vi32( x );
		vbool( b2IsPowerOf2( x ) );
		vi32( b2BoundingPowerOf2( x ) );
		vi32( b2RoundUpPowerOf2( x ) );
		vend();
	}

	// Negative and boundary inputs
	int edges[] = { -2147483647 - 1, -5, -1, 0, 1, 2, 3, 0x40000000, 0x7fffffff };
	for ( int i = 0; i < (int)( sizeof( edges ) / sizeof( edges[0] ) ); ++i )
	{
		int x = edges[i];
		vtag( "e" );
		vi32( x );
		vbool( b2IsPowerOf2( x ) );
		vi32( b2BoundingPowerOf2( x ) );
		if ( x <= 0x40000000 )
		{
			vi32( b2RoundUpPowerOf2( x ) );
		}
		vend();
	}
	return 0;
}
