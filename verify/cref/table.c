// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/table.as

#include "table.h"

#include "common.h"

static uint64_t keys[1 << 16];
static int keyCount;

// One line per occupied slot, so slot placement and backward shift deletion are compared exactly.
static void emitSet( const char* tag, const b2HashSet* set )
{
	vtag( tag );
	vu32( set->capacity );
	vu32( set->count );
	vi32( b2GetHashSetBytes( (b2HashSet*)set ) );
	vi32( b2GetSetCount( (b2HashSet*)set ) );
	vi32( b2GetSetCapacity( (b2HashSet*)set ) );
	vend();
	for ( uint32_t i = 0; i < set->capacity; ++i )
	{
		if ( set->items[i].key != 0 )
		{
			vtag( "slot" );
			vu32( i );
			vu64( set->items[i].key );
			vend();
		}
	}
}

static uint64_t makeKey( void )
{
	uint32_t kind = vrand32() % 4;
	if ( kind == 3 )
	{
		uint64_t key = vrand64();
		return key == 0 ? 1 : key;
	}

	// Pair keys from small increasing shape indices, as the broad phase makes them
	int s1 = vrandint( 0, 64 + keyCount / 4 );
	int s2 = vrandint( 0, 64 + keyCount / 4 );
	if ( s1 == s2 )
	{
		s2 += 1;
	}
	uint64_t key = B2_SHAPE_PAIR_KEY( s1, s2 );
	return key;
}

int main( void )
{
	vseed( 4 );

	int capacities[] = { -3, 0, 1, 16, 17, 31, 32, 33, 100, 1000 };
	for ( int i = 0; i < (int)( sizeof( capacities ) / sizeof( capacities[0] ) ); ++i )
	{
		b2HashSet set = b2CreateSet( capacities[i] );
		vtag( "cap" );
		vi32( capacities[i] );
		vu32( set.capacity );
		vend();
		b2DestroySet( &set );
	}

	int pairs[][2] = { { 0, 1 }, { 1, 0 }, { 5, 5 }, { 4194303, 0 }, { -1, 7 }, { 7, -1 }, { -5, -9 }, { 2147483647, 100 } };
	for ( int i = 0; i < 8; ++i )
	{
		uint64_t key = B2_SHAPE_PAIR_KEY( pairs[i][0], pairs[i][1] );
		vtag( "pair" );
		vu64( key );
		vu64( b2KeyHash( key ) );
		vend();
	}

	for ( int round = 0; round < 6; ++round )
	{
		b2HashSet set = b2CreateSet( vrandint( 0, 64 ) );
		keyCount = 0;
		emitSet( "create", &set );

		for ( int phase = 0; phase < 12; ++phase )
		{
			int adds = vrandint( 0, 400 );
			for ( int k = 0; k < adds; ++k )
			{
				uint64_t key = makeKey();
				if ( keyCount > 0 && vrand32() % 8 == 0 )
				{
					key = keys[vrandint( 0, keyCount )];
				}
				bool found = b2AddKey( &set, key );
				if ( keyCount < ( 1 << 16 ) )
				{
					keys[keyCount++] = key;
				}
				vtag( "add" );
				vu64( key );
				vu64( b2KeyHash( key ) );
				vbool( found );
				vend();
			}
			emitSet( "added", &set );

			int removes = keyCount > 0 ? vrandint( 0, adds / 2 + 2 ) : 0;
			for ( int k = 0; k < removes; ++k )
			{
				uint64_t key = vrand32() % 10 == 0 ? makeKey() : keys[vrandint( 0, keyCount )];
				bool found = b2RemoveKey( &set, key );
				vtag( "remove" );
				vu64( key );
				vbool( found );
				vend();
			}
			emitSet( "removed", &set );

			int queries = vrandint( 0, 100 );
			for ( int k = 0; k < queries; ++k )
			{
				uint64_t key = vrand32() % 3 == 0 || keyCount == 0 ? makeKey() : keys[vrandint( 0, keyCount )];
				vtag( "has" );
				vu64( key );
				vbool( b2ContainsKey( &set, key ) );
				vbool( b2ContainsHashedKey( &set, key, b2KeyHash( key ) ) );
				vend();
			}
		}

		if ( round % 2 == 1 )
		{
			b2ClearSet( &set );
			emitSet( "clear", &set );
			for ( int k = 0; k < 50; ++k )
			{
				uint64_t key = makeKey();
				vtag( "add" );
				vu64( key );
				vbool( b2AddKey( &set, key ) );
				vend();
			}
			emitSet( "refill", &set );
		}

		b2DestroySet( &set );
	}
	return 0;
}
