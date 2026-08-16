// Phase A container reference traces: id pool, int array growth, bitset,
// arena allocator. Each line is an op plus the expected observable state so
// the node harness can replay the identical sequence on the AS module.

#include "arena_allocator.h"
#include "array.h"
#include "bitset.h"
#include "id_pool.h"
#include "refcommon.h"

// ---- id pool ----

static void emitIdPool( void )
{
	b2IdPool pool = b2CreateIdPool( );
	printf( "IP RESET\n" );

	int live[4096];
	int liveCount = 0;

	for ( int i = 0; i < 3000; ++i )
	{
		int doAlloc = liveCount == 0 || irand( 100 ) < 55;
		if ( doAlloc && liveCount < 4096 )
		{
			int id = b2AllocId( &pool );
			live[liveCount++] = id;
			printf( "IP A %d %d %d %d %d\n", id, b2GetIdCount( &pool ), b2GetIdCapacity( &pool ), pool.freeArray.count,
					pool.freeArray.capacity );
		}
		else
		{
			int k = irand( liveCount );
			int id = live[k];
			live[k] = live[liveCount - 1];
			liveCount--;
			b2FreeId( &pool, id );
			printf( "IP F %d %d %d %d %d\n", id, b2GetIdCount( &pool ), b2GetIdCapacity( &pool ), pool.freeArray.count,
					pool.freeArray.capacity );
		}
	}

	// drain everything to churn generations of free list growth
	while ( liveCount > 0 )
	{
		int id = live[--liveCount];
		b2FreeId( &pool, id );
		printf( "IP F %d %d %d %d %d\n", id, b2GetIdCount( &pool ), b2GetIdCapacity( &pool ), pool.freeArray.count,
				pool.freeArray.capacity );
	}

	b2DestroyIdPool( &pool );
}

// ---- int array ----

static void emitIntArray( void )
{
	for ( int round = 0; round < 4; ++round )
	{
		int caps[4] = { 0, 1, 7, 100 };
		b2IntArray a = b2IntArray_Create( caps[round] );
		printf( "IA RESET %d %d %d\n", caps[round], a.count, a.capacity );

		for ( int i = 0; i < 1200; ++i )
		{
			int roll = irand( 100 );
			if ( roll < 50 )
			{
				int v = (int)rnd( );
				b2IntArray_Push( &a, v );
				printf( "IA PUSH %d %d %d\n", v, a.count, a.capacity );
			}
			else if ( roll < 65 )
			{
				if ( a.count > 0 )
				{
					int v = b2IntArray_Pop( &a );
					printf( "IA POP %d %d\n", v, a.count );
				}
			}
			else if ( roll < 80 )
			{
				if ( a.count > 0 )
				{
					int idx = irand( a.count );
					int moved = b2IntArray_RemoveSwap( &a, idx );
					printf( "IA RS %d %d %d\n", idx, moved, a.count );
				}
			}
			else if ( roll < 85 )
			{
				int n = irand( 300 );
				b2IntArray_Reserve( &a, n );
				printf( "IA RES %d %d\n", n, a.capacity );
			}
			else if ( roll < 95 )
			{
				if ( a.count > 0 )
				{
					int idx = irand( a.count );
					printf( "IA GET %d %d\n", idx, *b2IntArray_Get( &a, idx ) );
				}
			}
			else
			{
				int n = irand( 200 );
				b2IntArray_Resize( &a, n );
				printf( "IA RZ %d %d %d\n", n, a.count, a.capacity );
			}
		}

		b2IntArray_Destroy( &a );
	}
}

// ---- bitset ----

static uint64_t bsHash( const b2BitSet* s )
{
	uint64_t h = 0xcbf29ce484222325ull;
	for ( uint32_t i = 0; i < s->blockCount; ++i )
	{
		uint64_t w = s->bits[i];
		for ( int k = 0; k < 8; ++k )
		{
			h ^= ( w >> ( k * 8 ) ) & 0xff;
			h *= 0x100000001b3ull;
		}
	}
	return h;
}

static void bsState( const char* tag, int which, const b2BitSet* s )
{
	printf( "BS %s %d %u %u", tag, which, s->blockCount, s->blockCapacity );
	pu64( bsHash( s ) );
	printf( "\n" );
}

static void emitBitSet( void )
{
	b2BitSet sets[2];
	sets[0] = b2CreateBitSet( 64 );
	sets[1] = b2CreateBitSet( 64 );
	printf( "BS RESET 0 64\n" );
	printf( "BS RESET 1 64\n" );

	for ( int i = 0; i < 1500; ++i )
	{
		int which = irand( 2 );
		b2BitSet* s = &sets[which];
		int roll = irand( 100 );
		if ( roll < 45 )
		{
			uint32_t bit = (uint32_t)irand( 3000 );
			b2SetBitGrow( s, bit );
			printf( "BS SG %d %u\n", which, bit );
			bsState( "ST", which, s );
		}
		else if ( roll < 60 )
		{
			uint32_t bit = (uint32_t)irand( 3000 );
			b2ClearBit( s, bit );
			printf( "BS CB %d %u\n", which, bit );
			bsState( "ST", which, s );
		}
		else if ( roll < 75 )
		{
			uint32_t bit = (uint32_t)irand( 3000 );
			printf( "BS GB %d %u %d\n", which, bit, b2GetBit( s, bit ) ? 1 : 0 );
		}
		else if ( roll < 85 )
		{
			printf( "BS CNT %d %d\n", which, b2CountSetBits( s ) );
		}
		else if ( roll < 95 )
		{
			// setBit within blockCount only
			if ( s->blockCount > 0 )
			{
				uint32_t bit = (uint32_t)irand( (int)( s->blockCount * 64 ) );
				b2SetBit( s, bit );
				printf( "BS SB %d %u\n", which, bit );
				bsState( "ST", which, s );
			}
		}
		else
		{
			uint32_t n = (uint32_t)( 1 + irand( 2500 ) );
			b2SetBitCountAndClear( s, n );
			printf( "BS SCC %d %u\n", which, n );
			bsState( "ST", which, s );
		}
	}

	// union rounds require equal blockCounts
	for ( int round = 0; round < 20; ++round )
	{
		uint32_t bitCount = (uint32_t)( 64 + irand( 2000 ) );
		b2SetBitCountAndClear( &sets[0], bitCount );
		b2SetBitCountAndClear( &sets[1], bitCount );
		printf( "BS SCC 0 %u\n", bitCount );
		printf( "BS SCC 1 %u\n", bitCount );
		for ( int i = 0; i < 60; ++i )
		{
			uint32_t bit = (uint32_t)irand( (int)bitCount );
			int which = irand( 2 );
			b2SetBit( &sets[which], bit );
			printf( "BS SB %d %u\n", which, bit );
		}
		b2InPlaceUnion( &sets[0], &sets[1] );
		printf( "BS U\n" );
		bsState( "ST", 0, &sets[0] );
		bsState( "ST", 1, &sets[1] );
		printf( "BS CNT 0 %d\n", b2CountSetBits( &sets[0] ) );
	}

	b2DestroyBitSet( &sets[0] );
	b2DestroyBitSet( &sets[1] );
}

// ---- arena ----

static void arState( const b2ArenaAllocator* a )
{
	printf( "AR ST %d %d %d %d %d\n", a->index, a->allocation, a->maxAllocation, a->capacity, a->entries.count );
}

static void emitArena( void )
{
	int caps[3] = { 1024, 0, 300 };
	for ( int round = 0; round < 3; ++round )
	{
		b2ArenaAllocator a = b2CreateArenaAllocator( caps[round] );
		printf( "AR RESET %d\n", caps[round] );
		arState( &a );

		void* ptrs[16];
		int offsets[16];
		int depth = 0;

		for ( int i = 0; i < 400; ++i )
		{
			int doAlloc = depth == 0 || ( depth < 16 && irand( 100 ) < 55 );
			if ( doAlloc )
			{
				int size = 1 + irand( 600 );
				void* p = b2AllocateArenaItem( &a, size, "verify" );
				b2ArenaEntry* entry = a.entries.data + ( a.entries.count - 1 );
				int off = entry->usedMalloc ? -1 : (int)( (char*)p - a.data );
				ptrs[depth] = p;
				offsets[depth] = off;
				depth++;
				printf( "AR AL %d %d\n", size, off );
				arState( &a );
			}
			else
			{
				depth--;
				b2FreeArenaItem( &a, ptrs[depth] );
				printf( "AR FR\n" );
				arState( &a );
			}
		}

		while ( depth > 0 )
		{
			depth--;
			b2FreeArenaItem( &a, ptrs[depth] );
			printf( "AR FR\n" );
			arState( &a );
		}

		b2GrowArena( &a );
		printf( "AR GROW\n" );
		arState( &a );

		// after growing, one nested run that fits
		for ( int i = 0; i < 5; ++i )
		{
			void* p = b2AllocateArenaItem( &a, 64 + 32 * i, "verify" );
			b2ArenaEntry* entry = a.entries.data + ( a.entries.count - 1 );
			int off = entry->usedMalloc ? -1 : (int)( (char*)p - a.data );
			ptrs[depth] = p;
			offsets[depth] = off;
			depth++;
			printf( "AR AL %d %d\n", 64 + 32 * i, off );
			arState( &a );
		}
		while ( depth > 0 )
		{
			depth--;
			b2FreeArenaItem( &a, ptrs[depth] );
			printf( "AR FR\n" );
			arState( &a );
		}

		(void)offsets;
		b2DestroyArenaAllocator( &a );
	}
}

int main( void )
{
	emitIdPool( );
	emitIntArray( );
	emitBitSet( );
	emitArena( );
	return 0;
}
