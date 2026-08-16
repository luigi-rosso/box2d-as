// Phase B reference generator: dynamic tree operation traces (insert, remove,
// move, enlarge, rebuild) with a full-structure hash per op, query/ray/shape
// cast visit orders, hash set traces with slot-exact state hashes, and the
// broad phase move-buffer / pair-sweep traces.
//
// dynamic_tree.c is #included (not linked) so the node internals are visible
// for the structure hash. The pair sweep replicates the portable body of
// b2PairQueryCallback from broad_phase.c (the world-dependent filtering is
// Phase C); pair keys are added to pairSet afterwards like b2CreateContact.

#include "refcommon.h"

#include "dynamic_tree.c"

#include "aabb.h"
#include "array.h"
#include "table.h"

// ---- structure hash (mirrored in _verifyB.as) ----

static uint64_t g_h;

static void mix( uint32_t v )
{
	g_h ^= (uint64_t)v;
	g_h *= 0x100000001b3ULL;
}

static void mixF( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	mix( u );
}

static uint64_t treeHash( const b2DynamicTree* tree )
{
	g_h = 0xcbf29ce484222325ULL;
	mix( (uint32_t)tree->root );
	mix( (uint32_t)tree->nodeCount );
	mix( (uint32_t)tree->nodeCapacity );
	mix( (uint32_t)tree->freeList );
	mix( (uint32_t)tree->proxyCount );
	for ( int i = 0; i < tree->nodeCapacity; ++i )
	{
		const b2TreeNode* n = tree->nodes + i;
		mix( n->flags );
		if ( n->flags & b2_allocatedNode )
		{
			mix( (uint32_t)n->parent );
			mix( n->height );
			mix( (uint32_t)n->categoryBits );
			mix( (uint32_t)( n->categoryBits >> 32 ) );
			mixF( n->aabb.lowerBound.x );
			mixF( n->aabb.lowerBound.y );
			mixF( n->aabb.upperBound.x );
			mixF( n->aabb.upperBound.y );
			if ( n->flags & b2_leafNode )
			{
				mix( (uint32_t)n->userData );
				mix( (uint32_t)( n->userData >> 32 ) );
			}
			else
			{
				mix( (uint32_t)n->children.child1 );
				mix( (uint32_t)n->children.child2 );
			}
		}
		else
		{
			mix( (uint32_t)n->next );
		}
	}
	return g_h;
}

static void printTreeState( const b2DynamicTree* tree )
{
	pi( tree->root );
	pi( tree->nodeCount );
	pi( tree->nodeCapacity );
	pi( tree->freeList );
	pi( tree->proxyCount );
	pu64( treeHash( tree ) );
}

// ---- tree query/cast callbacks recording visit order ----

static int g_visited[512];
static int g_visitCount;

static bool recordQueryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	(void)context;
	if ( g_visitCount < 512 )
	{
		g_visited[g_visitCount++] = proxyId;
	}
	return true;
}

static float recordRayCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	(void)context;
	if ( g_visitCount < 512 )
	{
		g_visited[g_visitCount++] = proxyId;
	}
	return input->maxFraction;
}

// Clip-mode callbacks need the tree; use a global.
static const b2DynamicTree* g_rayTree;

static float clipRayCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	(void)context;
	if ( g_visitCount < 512 )
	{
		g_visited[g_visitCount++] = proxyId;
	}

	b2AABB aabb = b2DynamicTree_GetAABB( g_rayTree, proxyId );
	b2Vec2 p2 = b2Add( input->origin, input->translation );
	b2CastOutput out = b2AABB_RayCast( aabb, input->origin, p2 );
	if ( out.hit && out.fraction <= input->maxFraction )
	{
		return out.fraction;
	}
	return input->maxFraction;
}

static float recordShapeCastCallback( const b2ShapeCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	(void)context;
	if ( g_visitCount < 512 )
	{
		g_visited[g_visitCount++] = proxyId;
	}
	return input->maxFraction;
}

static float clipShapeCastCallback( const b2ShapeCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	(void)context;
	if ( g_visitCount < 512 )
	{
		g_visited[g_visitCount++] = proxyId;
	}

	b2AABB aabb = b2DynamicTree_GetAABB( g_rayTree, proxyId );
	b2Vec2 p2 = b2Add( input->proxy.points[0], input->translation );
	b2CastOutput out = b2AABB_RayCast( aabb, input->proxy.points[0], p2 );
	if ( out.hit && out.fraction <= input->maxFraction )
	{
		return out.fraction;
	}
	return input->maxFraction;
}

static void printVisits( b2TreeStats stats )
{
	pi( stats.nodeVisits );
	pi( stats.leafVisits );
	pi( g_visitCount );
	for ( int i = 0; i < g_visitCount; ++i )
	{
		pi( g_visited[i] );
	}
	printf( "\n" );
}

// ---- tree trace ----

static b2AABB randAABB( float span )
{
	float cx = frand( -span, span );
	float cy = frand( -span, span );
	float hx = frand( 0.05f, 1.2f );
	float hy = frand( 0.05f, 1.2f );
	b2AABB aabb = { { cx - hx, cy - hy }, { cx + hx, cy + hy } };
	return aabb;
}

static void printAABBIn( b2AABB a )
{
	ph( a.lowerBound.x );
	ph( a.lowerBound.y );
	ph( a.upperBound.x );
	ph( a.upperBound.y );
}

static void traceTree( int seedRun )
{
	b2DynamicTree tree = b2DynamicTree_Create( );
	printf( "TRR\n" );

	int proxyIds[400];
	b2AABB proxyBoxes[400];
	int liveCount = 0;

	int opCount = 1400;
	for ( int op = 0; op < opCount; ++op )
	{
		int action = irand( 100 );

		if ( liveCount < 12 || ( action < 38 && liveCount < 380 ) )
		{
			// create
			b2AABB aabb = randAABB( 10.0f + 2.0f * (float)seedRun );
			uint64_t cat = 1ULL << irand( 8 );
			uint64_t userData = (uint64_t)irand( 100000 );
			printf( "TRC" );
			printAABBIn( aabb );
			pu64( cat );
			pu64( userData );
			int proxyId = b2DynamicTree_CreateProxy( &tree, aabb, cat, userData );
			proxyIds[liveCount] = proxyId;
			proxyBoxes[liveCount] = aabb;
			liveCount += 1;
			pi( proxyId );
			printTreeState( &tree );
			printf( "\n" );
		}
		else if ( action < 48 && liveCount > 0 )
		{
			// destroy
			int pick = irand( liveCount );
			int proxyId = proxyIds[pick];
			printf( "TRD" );
			pi( proxyId );
			b2DynamicTree_DestroyProxy( &tree, proxyId );
			proxyIds[pick] = proxyIds[liveCount - 1];
			proxyBoxes[pick] = proxyBoxes[liveCount - 1];
			liveCount -= 1;
			printTreeState( &tree );
			printf( "\n" );
		}
		else if ( action < 70 && liveCount > 0 )
		{
			// move
			int pick = irand( liveCount );
			int proxyId = proxyIds[pick];
			b2AABB aabb = randAABB( 10.0f );
			printf( "TRM" );
			pi( proxyId );
			printAABBIn( aabb );
			b2DynamicTree_MoveProxy( &tree, proxyId, aabb );
			proxyBoxes[pick] = aabb;
			printTreeState( &tree );
			printf( "\n" );
		}
		else if ( action < 80 && liveCount > 0 )
		{
			// enlarge: grow the current box so containment fails as required
			int pick = irand( liveCount );
			int proxyId = proxyIds[pick];
			b2AABB aabb = proxyBoxes[pick];
			aabb.lowerBound.x -= frand( 0.01f, 1.5f );
			aabb.lowerBound.y -= frand( 0.01f, 1.5f );
			aabb.upperBound.x += frand( 0.01f, 1.5f );
			aabb.upperBound.y += frand( 0.01f, 1.5f );
			printf( "TRE" );
			pi( proxyId );
			printAABBIn( aabb );
			b2DynamicTree_EnlargeProxy( &tree, proxyId, aabb );
			proxyBoxes[pick] = aabb;
			printTreeState( &tree );
			printf( "\n" );
		}
		else if ( action < 84 )
		{
			// rebuild (partial mostly, full sometimes)
			int full = ( op % 5 ) == 0 ? 1 : 0;
			printf( "TRB" );
			pi( full );
			int leafCount = b2DynamicTree_Rebuild( &tree, full != 0 );
			pi( leafCount );
			printTreeState( &tree );
			printf( "\n" );
		}
		else if ( action < 92 )
		{
			// query
			b2AABB aabb = randAABB( 9.0f );
			uint64_t mask = ( op % 3 ) == 0 ? ~0ULL : (uint64_t)( 1 + irand( 255 ) );
			printf( "TRQ" );
			printAABBIn( aabb );
			pu64( mask );
			g_visitCount = 0;
			b2TreeStats stats = b2DynamicTree_Query( &tree, aabb, mask, recordQueryCallback, NULL );
			printVisits( stats );
		}
		else if ( action < 97 )
		{
			// ray cast
			b2RayCastInput input;
			input.origin = ( b2Vec2 ){ frand( -12.0f, 12.0f ), frand( -12.0f, 12.0f ) };
			input.translation = ( b2Vec2 ){ frand( -20.0f, 20.0f ), frand( -20.0f, 20.0f ) };
			input.maxFraction = ( op % 4 ) == 0 ? 0.5f : 1.0f;
			int mode = op % 2;
			printf( "TRRC" );
			ph( input.origin.x );
			ph( input.origin.y );
			ph( input.translation.x );
			ph( input.translation.y );
			ph( input.maxFraction );
			pi( mode );
			g_visitCount = 0;
			g_rayTree = &tree;
			b2TreeStats stats;
			if ( mode == 0 )
			{
				stats = b2DynamicTree_RayCast( &tree, &input, ~0ULL, recordRayCallback, NULL );
			}
			else
			{
				stats = b2DynamicTree_RayCast( &tree, &input, ~0ULL, clipRayCallback, NULL );
			}
			printVisits( stats );
		}
		else
		{
			// shape cast
			b2ShapeCastInput input;
			memset( &input, 0, sizeof( input ) );
			input.proxy.count = 3;
			float cx = frand( -12.0f, 12.0f );
			float cy = frand( -12.0f, 12.0f );
			input.proxy.points[0] = ( b2Vec2 ){ cx - 0.4f, cy - 0.3f };
			input.proxy.points[1] = ( b2Vec2 ){ cx + 0.5f, cy - 0.25f };
			input.proxy.points[2] = ( b2Vec2 ){ cx, cy + 0.45f };
			input.proxy.radius = ( op % 3 ) == 0 ? 0.1f : 0.0f;
			input.translation = ( b2Vec2 ){ frand( -18.0f, 18.0f ), frand( -18.0f, 18.0f ) };
			input.maxFraction = 1.0f;
			input.canEncroach = false;
			int mode = op % 2;
			printf( "TRSC" );
			pi( input.proxy.count );
			ph( input.proxy.radius );
			for ( int i = 0; i < input.proxy.count; ++i )
			{
				ph( input.proxy.points[i].x );
				ph( input.proxy.points[i].y );
			}
			ph( input.translation.x );
			ph( input.translation.y );
			ph( input.maxFraction );
			pi( mode );
			g_visitCount = 0;
			g_rayTree = &tree;
			b2TreeStats stats;
			if ( mode == 0 )
			{
				stats = b2DynamicTree_ShapeCast( &tree, &input, ~0ULL, recordShapeCastCallback, NULL );
			}
			else
			{
				stats = b2DynamicTree_ShapeCast( &tree, &input, ~0ULL, clipShapeCastCallback, NULL );
			}
			printVisits( stats );
		}
	}

	b2DynamicTree_Destroy( &tree );
}

// ---- hash set trace ----

static uint64_t setHash( const b2HashSet* set )
{
	g_h = 0xcbf29ce484222325ULL;
	mix( set->capacity );
	mix( set->count );
	for ( uint32_t i = 0; i < set->capacity; ++i )
	{
		mix( (uint32_t)set->items[i].key );
		mix( (uint32_t)( set->items[i].key >> 32 ) );
	}
	return g_h;
}

static void traceHashSet( void )
{
	b2HashSet set = b2CreateSet( 16 );
	printf( "HSR" );
	pi( 16 );
	pi( (int)set.capacity );
	printf( "\n" );

	uint64_t keys[600];
	int keyCount = 0;

	for ( int op = 0; op < 4000; ++op )
	{
		int action = irand( 100 );
		if ( action < 55 || keyCount == 0 )
		{
			// add (sometimes a duplicate)
			uint64_t key;
			if ( keyCount > 0 && action < 12 )
			{
				key = keys[irand( keyCount )];
			}
			else
			{
				uint32_t a = rnd( ) % 5000u;
				uint32_t b = rnd( ) % 5000u;
				key = B2_SHAPE_PAIR_KEY( a, b );
				if ( key == 0 )
				{
					key = 1;
				}
			}
			printf( "HSA" );
			pu64( key );
			bool found = b2AddKey( &set, key );
			if ( !found && keyCount < 600 )
			{
				keys[keyCount++] = key;
			}
			pi( found ? 1 : 0 );
			pi( (int)set.count );
			pi( (int)set.capacity );
			pu64( setHash( &set ) );
			printf( "\n" );
		}
		else if ( action < 85 )
		{
			// remove (mostly existing keys)
			uint64_t key;
			if ( action < 75 && keyCount > 0 )
			{
				int pick = irand( keyCount );
				key = keys[pick];
				keys[pick] = keys[keyCount - 1];
				keyCount -= 1;
			}
			else
			{
				key = B2_SHAPE_PAIR_KEY( rnd( ) % 5000u, rnd( ) % 5000u );
				if ( key == 0 )
				{
					key = 1;
				}
			}
			printf( "HSD" );
			pu64( key );
			bool found = b2RemoveKey( &set, key );
			pi( found ? 1 : 0 );
			pi( (int)set.count );
			pi( (int)set.capacity );
			pu64( setHash( &set ) );
			printf( "\n" );
		}
		else if ( action < 99 )
		{
			uint64_t key = B2_SHAPE_PAIR_KEY( rnd( ) % 5000u, rnd( ) % 5000u );
			if ( key == 0 )
			{
				key = 1;
			}
			printf( "HSC" );
			pu64( key );
			pi( b2ContainsKey( &set, key ) ? 1 : 0 );
			printf( "\n" );
		}
		else
		{
			printf( "HSCL" );
			b2ClearSet( &set );
			pi( (int)set.count );
			pi( (int)set.capacity );
			pu64( setHash( &set ) );
			printf( "\n" );
		}
	}

	b2DestroySet( &set );
}

// ---- broad phase trace ----
// Minimal b2BroadPhase mirror: trees + move buffers + pair set. The proxy
// functions are the verbatim bodies from broad_phase.c.

#define PROXY_TYPE( KEY ) ( ( KEY ) & 3 )
#define PROXY_ID( KEY ) ( ( KEY ) >> 2 )
#define PROXY_KEY( ID, TYPE ) ( ( ( ID ) << 2 ) | ( TYPE ) )

typedef struct RefBroadPhase
{
	b2DynamicTree trees[3];
	b2HashSet moveSet;
	b2IntArray moveArray;
	b2HashSet pairSet;
} RefBroadPhase;

static void bpCreate( RefBroadPhase* bp )
{
	bp->moveSet = b2CreateSet( 16 );
	bp->moveArray = b2IntArray_Create( 16 );
	bp->pairSet = b2CreateSet( 32 );
	for ( int i = 0; i < 3; ++i )
	{
		bp->trees[i] = b2DynamicTree_Create( );
	}
}

static void bpDestroy( RefBroadPhase* bp )
{
	for ( int i = 0; i < 3; ++i )
	{
		b2DynamicTree_Destroy( bp->trees + i );
	}
	b2DestroySet( &bp->moveSet );
	b2IntArray_Destroy( &bp->moveArray );
	b2DestroySet( &bp->pairSet );
}

static void bpBufferMove( RefBroadPhase* bp, int queryProxy )
{
	bool alreadyAdded = b2AddKey( &bp->moveSet, queryProxy + 1 );
	if ( alreadyAdded == false )
	{
		b2IntArray_Push( &bp->moveArray, queryProxy );
	}
}

static void bpUnBufferMove( RefBroadPhase* bp, int proxyKey )
{
	bool found = b2RemoveKey( &bp->moveSet, proxyKey + 1 );

	if ( found )
	{
		int count = bp->moveArray.count;
		for ( int i = 0; i < count; ++i )
		{
			if ( bp->moveArray.data[i] == proxyKey )
			{
				b2IntArray_RemoveSwap( &bp->moveArray, i );
				break;
			}
		}
	}
}

static int bpCreateProxy( RefBroadPhase* bp, int proxyType, b2AABB aabb, uint64_t categoryBits, int shapeIndex,
						  bool forcePairCreation )
{
	int proxyId = b2DynamicTree_CreateProxy( bp->trees + proxyType, aabb, categoryBits, (uint64_t)shapeIndex );
	int proxyKey = PROXY_KEY( proxyId, proxyType );
	if ( proxyType != 0 || forcePairCreation )
	{
		bpBufferMove( bp, proxyKey );
	}
	return proxyKey;
}

static void bpDestroyProxy( RefBroadPhase* bp, int proxyKey )
{
	bpUnBufferMove( bp, proxyKey );
	b2DynamicTree_DestroyProxy( bp->trees + PROXY_TYPE( proxyKey ), PROXY_ID( proxyKey ) );
}

static void bpMoveProxy( RefBroadPhase* bp, int proxyKey, b2AABB aabb )
{
	b2DynamicTree_MoveProxy( bp->trees + PROXY_TYPE( proxyKey ), PROXY_ID( proxyKey ), aabb );
	bpBufferMove( bp, proxyKey );
}

static void bpEnlargeProxy( RefBroadPhase* bp, int proxyKey, b2AABB aabb )
{
	b2DynamicTree_EnlargeProxy( bp->trees + PROXY_TYPE( proxyKey ), PROXY_ID( proxyKey ), aabb );
	bpBufferMove( bp, proxyKey );
}

// Pair sweep: portable body of b2PairQueryCallback + the deterministic
// creation loop of b2UpdateBroadPhasePairs.

typedef struct RefPair
{
	int a, b;
} RefPair;

typedef struct RefPairContext
{
	RefBroadPhase* bp;
	int queryTreeType;
	int queryProxyKey;
	int queryShapeIndex;
	RefPair pairs[4096];
	int pairCount;
} RefPairContext;

static bool refPairCallback( int proxyId, uint64_t userData, void* context )
{
	int shapeId = (int)userData;
	RefPairContext* ctx = context;
	RefBroadPhase* bp = ctx->bp;

	int proxyKey = PROXY_KEY( proxyId, ctx->queryTreeType );
	int queryProxyKey = ctx->queryProxyKey;

	if ( proxyKey == queryProxyKey )
	{
		return true;
	}

	int treeType = ctx->queryTreeType;
	int queryProxyType = PROXY_TYPE( queryProxyKey );

	if ( queryProxyType == 2 )
	{
		if ( treeType == 2 && proxyKey < queryProxyKey )
		{
			bool moved = b2ContainsKey( &bp->moveSet, proxyKey + 1 );
			if ( moved )
			{
				return true;
			}
		}
	}
	else
	{
		bool moved = b2ContainsKey( &bp->moveSet, proxyKey + 1 );
		if ( moved )
		{
			return true;
		}
	}

	uint64_t pairKey = B2_SHAPE_PAIR_KEY( shapeId, ctx->queryShapeIndex );
	if ( b2ContainsKey( &bp->pairSet, pairKey ) )
	{
		return true;
	}

	int shapeIdA, shapeIdB;
	if ( proxyKey < queryProxyKey )
	{
		shapeIdA = shapeId;
		shapeIdB = ctx->queryShapeIndex;
	}
	else
	{
		shapeIdA = ctx->queryShapeIndex;
		shapeIdB = shapeId;
	}

	if ( ctx->pairCount < 4096 )
	{
		ctx->pairs[ctx->pairCount].a = shapeIdA;
		ctx->pairs[ctx->pairCount].b = shapeIdB;
		ctx->pairCount += 1;
	}

	return true;
}

static void bpUpdatePairs( RefBroadPhase* bp )
{
	int moveCount = bp->moveArray.count;

	printf( "BPU" );

	if ( moveCount == 0 )
	{
		pi( 0 );
		pi( (int)bp->pairSet.count );
		pi( (int)bp->pairSet.capacity );
		printf( "\n" );
		return;
	}

	RefPairContext ctx;
	ctx.bp = bp;

	// Creation-ordered pair list across all moves
	static RefPair created[8192];
	int createdCount = 0;

	for ( int i = 0; i < moveCount; ++i )
	{
		int proxyKey = bp->moveArray.data[i];
		if ( proxyKey == B2_NULL_INDEX )
		{
			continue;
		}

		int proxyType = PROXY_TYPE( proxyKey );
		int proxyId = PROXY_ID( proxyKey );
		ctx.queryProxyKey = proxyKey;
		ctx.pairCount = 0;

		const b2DynamicTree* baseTree = bp->trees + proxyType;
		b2AABB fatAABB = b2DynamicTree_GetAABB( baseTree, proxyId );
		ctx.queryShapeIndex = (int)b2DynamicTree_GetUserData( baseTree, proxyId );

		if ( proxyType == 2 )
		{
			ctx.queryTreeType = 1;
			b2DynamicTree_Query( bp->trees + 1, fatAABB, ~0ULL, refPairCallback, &ctx );
			ctx.queryTreeType = 0;
			b2DynamicTree_Query( bp->trees + 0, fatAABB, ~0ULL, refPairCallback, &ctx );
		}

		ctx.queryTreeType = 2;
		b2DynamicTree_Query( bp->trees + 2, fatAABB, ~0ULL, refPairCallback, &ctx );

		// C prepends to pairList then walks it: creation order is reversed.
		for ( int j = ctx.pairCount - 1; j >= 0; --j )
		{
			if ( createdCount < 8192 )
			{
				created[createdCount++] = ctx.pairs[j];
			}
		}
	}

	b2IntArray_Clear( &bp->moveArray );
	b2ClearSet( &bp->moveSet );

	// Create contacts: add pair keys like b2CreateContact
	for ( int i = 0; i < createdCount; ++i )
	{
		uint64_t pairKey = B2_SHAPE_PAIR_KEY( created[i].a, created[i].b );
		b2AddKey( &bp->pairSet, pairKey );
	}

	pi( createdCount );
	for ( int i = 0; i < createdCount; ++i )
	{
		pi( created[i].a );
		pi( created[i].b );
	}
	pi( (int)bp->pairSet.count );
	pi( (int)bp->pairSet.capacity );
	printf( "\n" );
}

static void printBPState( RefBroadPhase* bp )
{
	pi( bp->moveArray.count );
	for ( int i = 0; i < bp->moveArray.count; ++i )
	{
		pi( bp->moveArray.data[i] );
	}
	pi( (int)bp->moveSet.count );
	pi( (int)bp->moveSet.capacity );
	pu64( setHash( &bp->moveSet ) );
}

static void traceBroadPhase( void )
{
	RefBroadPhase bp;
	bpCreate( &bp );
	printf( "BPR\n" );

	int proxyKeys[300];
	b2AABB proxyBoxes[300];
	int liveCount = 0;
	int nextShapeIndex = 0;

	for ( int op = 0; op < 1200; ++op )
	{
		int action = irand( 100 );

		if ( liveCount < 15 || ( action < 35 && liveCount < 290 ) )
		{
			int proxyType = irand( 3 );
			b2AABB aabb = randAABB( 8.0f );
			uint64_t cat = 1ULL << irand( 4 );
			int shapeIndex = nextShapeIndex++;
			bool force = irand( 10 ) == 0;
			printf( "BPC" );
			pi( proxyType );
			printAABBIn( aabb );
			pu64( cat );
			pi( shapeIndex );
			pi( force ? 1 : 0 );
			int proxyKey = bpCreateProxy( &bp, proxyType, aabb, cat, shapeIndex, force );
			proxyKeys[liveCount] = proxyKey;
			proxyBoxes[liveCount] = aabb;
			liveCount += 1;
			pi( proxyKey );
			printBPState( &bp );
			printf( "\n" );
		}
		else if ( action < 45 && liveCount > 0 )
		{
			int pick = irand( liveCount );
			int proxyKey = proxyKeys[pick];
			printf( "BPD" );
			pi( proxyKey );
			bpDestroyProxy( &bp, proxyKey );
			proxyKeys[pick] = proxyKeys[liveCount - 1];
			proxyBoxes[pick] = proxyBoxes[liveCount - 1];
			liveCount -= 1;
			printBPState( &bp );
			printf( "\n" );
		}
		else if ( action < 70 && liveCount > 0 )
		{
			int pick = irand( liveCount );
			int proxyKey = proxyKeys[pick];
			b2AABB aabb = randAABB( 8.0f );
			printf( "BPM" );
			pi( proxyKey );
			printAABBIn( aabb );
			bpMoveProxy( &bp, proxyKey, aabb );
			proxyBoxes[pick] = aabb;
			printBPState( &bp );
			printf( "\n" );
		}
		else if ( action < 78 && liveCount > 0 )
		{
			// enlarge a non-static proxy
			int pick = -1;
			for ( int tryIdx = 0; tryIdx < liveCount; ++tryIdx )
			{
				int candidate = irand( liveCount );
				if ( PROXY_TYPE( proxyKeys[candidate] ) != 0 )
				{
					pick = candidate;
					break;
				}
			}
			if ( pick >= 0 )
			{
				int proxyKey = proxyKeys[pick];
				b2AABB aabb = proxyBoxes[pick];
				aabb.lowerBound.x -= frand( 0.01f, 1.0f );
				aabb.lowerBound.y -= frand( 0.01f, 1.0f );
				aabb.upperBound.x += frand( 0.01f, 1.0f );
				aabb.upperBound.y += frand( 0.01f, 1.0f );
				printf( "BPE" );
				pi( proxyKey );
				printAABBIn( aabb );
				bpEnlargeProxy( &bp, proxyKey, aabb );
				proxyBoxes[pick] = aabb;
				printBPState( &bp );
				printf( "\n" );
			}
		}
		else if ( action < 90 )
		{
			bpUpdatePairs( &bp );
		}
		else if ( action < 96 && liveCount > 1 )
		{
			int a = irand( liveCount );
			int b = irand( liveCount );
			printf( "BPT" );
			pi( proxyKeys[a] );
			pi( proxyKeys[b] );
			b2AABB aabbA = b2DynamicTree_GetAABB( bp.trees + PROXY_TYPE( proxyKeys[a] ), PROXY_ID( proxyKeys[a] ) );
			b2AABB aabbB = b2DynamicTree_GetAABB( bp.trees + PROXY_TYPE( proxyKeys[b] ), PROXY_ID( proxyKeys[b] ) );
			pi( b2AABB_Overlaps( aabbA, aabbB ) ? 1 : 0 );
			printf( "\n" );
		}
		else
		{
			// rebuild dynamic + kinematic trees
			printf( "BPRB" );
			b2DynamicTree_Rebuild( bp.trees + 2, false );
			b2DynamicTree_Rebuild( bp.trees + 1, false );
			pu64( treeHash( bp.trees + 0 ) );
			pu64( treeHash( bp.trees + 1 ) );
			pu64( treeHash( bp.trees + 2 ) );
			printf( "\n" );
		}
	}

	bpDestroy( &bp );
}

int main( void )
{
	traceTree( 0 );
	traceTree( 1 );
	traceTree( 2 );
	traceHashSet( );
	traceBroadPhase( );
	return 0;
}
