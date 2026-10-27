// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The node runner's twin on native Box2D: native <scene index> <steps> <workers> <prewarm>
// With prewarm 1 the scene first runs once untimed, as run.mjs does for V8's tier up.
// Prints one JSON line with each step's ms, b2Profile fields and state hash, as run.mjs does.

#include "scenes.h"

#include <stdio.h>
#include <stdlib.h>

int create( int sceneIndex, int workerCount );
void step( int stepIndex );
uint32_t stateHash( void );
float profileField( int k );

int main( int argc, char** argv )
{
	if ( argc != 5 )
	{
		fprintf( stderr, "usage: native <scene index> <steps> <workers> <prewarm>\n" );
		return 1;
	}
	int sceneIndex = atoi( argv[1] );
	int stepCount = atoi( argv[2] );
	int workerCount = atoi( argv[3] );
	if ( atoi( argv[4] ) != 0 )
	{
		if ( create( sceneIndex, workerCount ) == 0 )
		{
			return 1;
		}
		for ( int i = 0; i < stepCount; ++i )
		{
			step( i );
			stateHash();
		}
	}
	if ( create( sceneIndex, workerCount ) == 0 )
	{
		return 1;
	}

	double* ms = malloc( stepCount * sizeof( double ) );
	float* stages = malloc( stepCount * 23 * sizeof( float ) );
	uint32_t* hashes = malloc( stepCount * sizeof( uint32_t ) );
	for ( int i = 0; i < stepCount; ++i )
	{
		uint64_t ticks = b2GetTicks();
		step( i );
		ms[i] = b2GetMilliseconds( ticks );
		for ( int k = 0; k < 23; ++k )
		{
			stages[23 * i + k] = profileField( k );
		}
		hashes[i] = stateHash();
	}

	printf( "{\"ms\":[" );
	for ( int i = 0; i < stepCount; ++i )
	{
		printf( "%s%.6f", i ? "," : "", ms[i] );
	}
	printf( "],\"stages\":[" );
	for ( int i = 0; i < stepCount * 23; ++i )
	{
		printf( "%s%.6f", i ? "," : "", stages[i] );
	}
	printf( "],\"hashes\":[" );
	for ( int i = 0; i < stepCount; ++i )
	{
		printf( "%s%u", i ? "," : "", hashes[i] );
	}
	printf( "]}\n" );
	return 0;
}
