// Shared helpers for the Phase A reference generators. Floats travel as raw
// bit patterns so the node comparison is bit-exact.

#pragma once

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void ph( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	printf( " %08x", u );
}

static void pi( int v )
{
	printf( " %d", v );
}

static void pu64( uint64_t v )
{
	printf( " %016" PRIx64, v );
}

// xorshift32; fixed seed so vector files are reproducible.
static uint32_t g_rng = 0x12345678u;

static uint32_t rnd( void )
{
	g_rng ^= g_rng << 13;
	g_rng ^= g_rng >> 17;
	g_rng ^= g_rng << 5;
	return g_rng;
}

static float frand( float lo, float hi )
{
	return lo + ( hi - lo ) * (float)( rnd( ) * ( 1.0 / 4294967296.0 ) );
}

static int irand( int n )
{
	return (int)( rnd( ) % (uint32_t)n );
}
