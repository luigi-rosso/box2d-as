// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/aabb.as, lib/dynamic_tree.as and lib/broad_phase_core.as.
// broad_phase.c is included so its static pair search runs here; the world it reads is a fake whose
// shapes come in pairs on one body, so the only filter that culls is the same body test.

#include "aabb.h"
#include "dynamic_tree.h"

#include "broad_phase.c"

#include "common.h"

#include <stdlib.h>

// ---- shared helpers ----

static uint64_t g_hash;

static void mix( uint32_t v )
{
	g_hash ^= (uint64_t)v;
	g_hash *= 0x100000001b3ull;
}

static void mixF( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	mix( u );
}

static void mix64( uint64_t v )
{
	mix( (uint32_t)v );
	mix( (uint32_t)( v >> 32 ) );
}

static uint64_t treeHash( const b2DynamicTree* tree )
{
	g_hash = 0xcbf29ce484222325ull;
	mix( (uint32_t)tree->nodeEnd );
	mix( (uint32_t)tree->nodeCapacity );
	mix( (uint32_t)tree->pairFreeList );
	mix( (uint32_t)tree->proxyCount );
	mix( (uint32_t)tree->proxyCapacity );
	mix( (uint32_t)tree->proxyFreeList );
	mix( tree->dfsOrdered ? 1u : 0u );
	mix( (uint32_t)tree->rebuildCapacity );
	mix( tree->swapNodes != NULL ? 1u : 0u );
	for ( int i = 0; i < tree->nodeEnd; ++i )
	{
		const b2TreeNode* n = tree->nodes + i;
		mixF( n->aabb.lowerBound.x );
		mixF( n->aabb.lowerBound.y );
		mixF( n->aabb.upperBound.x );
		mixF( n->aabb.upperBound.y );
		mix( n->flagIndex );
		mix( (uint32_t)n->height );
		mix( (uint32_t)tree->parents[i] );
	}
	for ( int i = 0; i < tree->proxyCapacity; ++i )
	{
		const b2TreeProxy* p = tree->proxies + i;
		mix64( p->userData );
		mix64( p->categoryBits );
		mix( (uint32_t)p->node );
		mix( (uint32_t)p->next );
	}
	return g_hash;
}

static void vaabb( b2AABB a )
{
	vf32( a.lowerBound.x );
	vf32( a.lowerBound.y );
	vf32( a.upperBound.x );
	vf32( a.upperBound.y );
}

static void dumpTree( const char* tag, const b2DynamicTree* tree )
{
	vtag( tag );
	vi32( tree->nodeEnd );
	vi32( tree->nodeCapacity );
	vi32( tree->pairFreeList );
	vi32( tree->proxyCount );
	vi32( tree->proxyCapacity );
	vi32( tree->proxyFreeList );
	vbool( tree->dfsOrdered );
	vi32( tree->rebuildCapacity );
	vbool( tree->swapNodes != NULL );
	vi32( b2DynamicTree_GetHeight( tree ) );
	vf32( b2DynamicTree_GetAreaRatio( tree ) );
	vaabb( b2DynamicTree_GetRootBounds( tree ) );
	vi32( b2DynamicTree_GetProxyCount( tree ) );
	vi32( (int)b2DynamicTree_GetByteCount( tree ) );
	vbool( b2HasTreeMoved( tree ) );
	vbool( b2NeedsRebuild( tree ) );
	vend();
	for ( int i = 0; i < tree->nodeEnd; ++i )
	{
		const b2TreeNode* n = tree->nodes + i;
		vtag( "n" );
		vi32( i );
		vaabb( n->aabb );
		vu32( n->flagIndex );
		vi32( n->height );
		vi32( tree->parents[i] );
		vend();
	}
	for ( int i = 0; i < tree->proxyCapacity; ++i )
	{
		const b2TreeProxy* p = tree->proxies + i;
		vtag( "p" );
		vi32( i );
		vu64( p->userData );
		vu64( p->categoryBits );
		vi32( p->node );
		vi32( p->next );
		vend();
	}
}

// ---- aabb ----

static b2Vec2 randVec( float span )
{
	b2Vec2 v;
	v.x = vrandf( -span, span );
	v.y = vrandf( -span, span );
	return v;
}

static void aabbChecks( void )
{
	vseed( 21 );
	for ( int i = 0; i < 4000; ++i )
	{
		b2AABB a;
		a.lowerBound.x = vspecialf();
		a.lowerBound.y = vspecialf();
		a.upperBound.x = vspecialf();
		a.upperBound.y = vspecialf();
		b2AABB b;
		b.lowerBound.x = vspecialf();
		b.lowerBound.y = vspecialf();
		b.upperBound.x = vspecialf();
		b.upperBound.y = vspecialf();
		vtag( "aabb" );
		vf32( b2Perimeter( a ) );
		vbool( b2IsValidAABB( a ) );
		b2AABB c = a;
		vbool( b2EnlargeAABB( &c, b ) );
		vaabb( c );
		b2Pos origin;
		origin.x = vspecialf();
		origin.y = vspecialf();
		vaabb( b2OffsetAABB( a, origin ) );
		vend();
	}

	for ( int i = 0; i < 6000; ++i )
	{
		b2AABB a;
		a.lowerBound = randVec( 10.0f );
		a.upperBound.x = a.lowerBound.x + vrandf( 0.0f, 5.0f );
		a.upperBound.y = a.lowerBound.y + vrandf( 0.0f, 5.0f );
		b2Vec2 p1 = randVec( 15.0f );
		b2Vec2 p2;
		int kind = vrandint( 0, 4 );
		if ( kind == 0 )
		{
			p2 = randVec( 15.0f );
		}
		else if ( kind == 1 )
		{
			p2.x = p1.x;
			p2.y = vrandf( -15.0f, 15.0f );
		}
		else if ( kind == 2 )
		{
			p2.x = vrandf( -15.0f, 15.0f );
			p2.y = p1.y;
		}
		else
		{
			p2 = p1;
		}
		b2CastOutput out = b2AABB_RayCast( a, p1, p2 );
		vtag( "ray" );
		vf32( out.normal.x );
		vf32( out.normal.y );
		vf32( out.point.x );
		vf32( out.point.y );
		vf32( out.fraction );
		vi32( out.iterations );
		vbool( out.hit );
		vend();
	}
}

// ---- tree ----

static b2DynamicTree* g_tree;
static int g_boxKind;
static int g_mode;
static int g_hitBudget;
static int g_depth;
static int g_nestBudget;
static int g_nestedHits;

static b2AABB randBox( void )
{
	b2AABB box;
	if ( g_boxKind == 0 )
	{
		float cx = vrandf( -30.0f, 30.0f );
		float cy = vrandf( -30.0f, 30.0f );
		float hx = vrandf( 0.05f, 2.0f );
		float hy = vrandf( 0.05f, 2.0f );
		box.lowerBound.x = cx - hx;
		box.lowerBound.y = cy - hy;
		box.upperBound.x = cx + hx;
		box.upperBound.y = cy + hy;
	}
	else if ( g_boxKind == 1 )
	{
		float cx = (float)vrandint( -6, 7 );
		float cy = (float)vrandint( -6, 7 );
		float h = 0.5f * (float)vrandint( 1, 4 );
		box.lowerBound.x = cx - h;
		box.lowerBound.y = cy - h;
		box.upperBound.x = cx + h;
		box.upperBound.y = cy + h;
	}
	else if ( g_boxKind == 2 )
	{
		float cx = vrandf( -10.0f, 10.0f );
		float cy = vrandf( -10.0f, 10.0f );
		int flat = vrandint( 0, 3 );
		float hx = flat == 1 ? 0.0f : vrandf( 0.0f, 1.0f );
		float hy = flat == 2 ? 0.0f : vrandf( 0.0f, 1.0f );
		if ( flat == 0 )
		{
			hx = 0.0f;
			hy = 0.0f;
		}
		box.lowerBound.x = cx - hx;
		box.lowerBound.y = cy - hy;
		box.upperBound.x = cx + hx;
		box.upperBound.y = cy + hy;
	}
	else if ( g_boxKind == 3 )
	{
		float cx = 100000.0f + vrandf( -60.0f, 60.0f );
		float cy = -50000.0f + vrandf( -60.0f, 60.0f );
		float hx = vrandf( 0.01f, 3.0f );
		float hy = vrandf( 0.01f, 3.0f );
		box.lowerBound.x = cx - hx;
		box.lowerBound.y = cy - hy;
		box.upperBound.x = cx + hx;
		box.upperBound.y = cy + hy;
	}
	else if ( g_boxKind == 4 )
	{
		box.lowerBound.x = -1.0f;
		box.lowerBound.y = -1.0f;
		box.upperBound.x = 1.0f;
		box.upperBound.y = 1.0f;
	}
	else
	{
		// a line of boxes
		float cx = vrandf( -200.0f, 200.0f );
		float h = vrandf( 0.1f, 1.0f );
		box.lowerBound.x = cx - h;
		box.lowerBound.y = -h;
		box.upperBound.x = cx + h;
		box.upperBound.y = h;
	}
	return box;
}

static b2AABB growBox( b2AABB a )
{
	b2AABB b = a;
	b.lowerBound.x -= vrandf( 0.0f, 0.5f );
	b.lowerBound.y -= vrandf( 0.0f, 0.5f );
	b.upperBound.x += vrandf( 0.0f, 0.5f );
	b.upperBound.y += vrandf( 0.0f, 0.5f );
	return b;
}

static uint64_t randBits( void )
{
	int kind = vrandint( 0, 4 );
	if ( kind == 0 )
	{
		return UINT64_MAX;
	}
	if ( kind == 1 )
	{
		return 1ull << vrandint( 0, 8 );
	}
	if ( kind == 2 )
	{
		return vrand64();
	}
	return (uint64_t)vrandint( 0, 16 );
}

static b2TreeStats nestedOverlap( int proxyId );

static bool overlapCallback( int proxyId, uint64_t userData, void* context )
{
	(void)context;
	vtag( "qh" );
	vi32( g_depth );
	vi32( proxyId );
	vu64( userData );
	vend();

	if ( g_mode == 2 )
	{
		g_hitBudget -= 1;
		return g_hitBudget > 0;
	}

	if ( g_mode == 4 && g_depth < 3 && g_nestBudget > 0 && ( proxyId % 3 ) == 0 )
	{
		b2TreeStats s = nestedOverlap( proxyId );
		vtag( "qn" );
		vi32( s.nodeVisits );
		vi32( s.leafVisits );
		vend();
	}

	return true;
}

static b2TreeStats nestedOverlap( int proxyId )
{
	g_nestBudget -= 1;
	g_depth += 1;
	b2AABB box = b2DynamicTree_GetProxyBounds( g_tree, proxyId );
	b2TreeStats s = b2DynamicTree_OverlapAABB( g_tree, box, UINT64_MAX, overlapCallback, NULL );
	g_depth -= 1;
	return s;
}

static float rayCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)context;
	float value = input->maxFraction;
	if ( g_mode == 2 )
	{
		g_hitBudget -= 1;
		value = g_hitBudget > 0 ? input->maxFraction : 0.0f;
	}
	else if ( g_mode == 3 && ( proxyId & 1 ) == 1 )
	{
		value = -1.0f;
	}
	else if ( g_mode != 0 )
	{
		if ( g_mode == 4 && g_depth < 2 && g_nestBudget > 0 )
		{
			b2TreeStats s = nestedOverlap( proxyId );
			vtag( "rn" );
			vi32( s.nodeVisits );
			vi32( s.leafVisits );
			vend();
		}

		b2AABB box = b2DynamicTree_GetProxyBounds( g_tree, proxyId );
		b2Vec2 p2 = b2Add( input->origin, input->translation );
		b2CastOutput out = b2AABB_RayCast( box, input->origin, p2 );
		if ( out.hit && out.fraction < input->maxFraction )
		{
			value = out.fraction;
		}
	}

	vtag( "rh" );
	vi32( proxyId );
	vu64( userData );
	vf32( input->maxFraction );
	vf32( value );
	vend();
	return value;
}

static float boxCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)context;
	float value = input->maxFraction;
	if ( g_mode == 2 )
	{
		g_hitBudget -= 1;
		value = g_hitBudget > 0 ? input->maxFraction : 0.0f;
	}
	else if ( g_mode == 3 && ( proxyId & 1 ) == 1 )
	{
		value = -1.0f;
	}
	else if ( g_mode != 0 )
	{
		if ( g_mode == 4 && g_depth < 2 && g_nestBudget > 0 )
		{
			b2TreeStats s = nestedOverlap( proxyId );
			vtag( "bn" );
			vi32( s.nodeVisits );
			vi32( s.leafVisits );
			vend();
		}

		// Minkowski sum of the proxy box and the cast box, swept by the box center
		b2AABB box = b2DynamicTree_GetProxyBounds( g_tree, proxyId );
		b2Vec2 e = b2AABB_Extents( input->box );
		box.lowerBound = b2Sub( box.lowerBound, e );
		box.upperBound = b2Add( box.upperBound, e );
		b2Vec2 p1 = b2AABB_Center( input->box );
		b2Vec2 p2 = b2Add( p1, input->translation );
		b2CastOutput out = b2AABB_RayCast( box, p1, p2 );
		if ( out.hit && out.fraction < input->maxFraction )
		{
			value = out.fraction;
		}
	}

	vtag( "bh" );
	vi32( proxyId );
	vu64( userData );
	vf32( input->maxFraction );
	vf32( value );
	vend();
	return value;
}

static void vstats( const char* tag, b2TreeStats s )
{
	vtag( tag );
	vi32( s.nodeVisits );
	vi32( s.leafVisits );
	vend();
}

#define MAX_LIVE 700

static int g_live[MAX_LIVE];
static int g_liveCount;
static int g_moved[4096];

static void vhash( void )
{
	vu64( treeHash( g_tree ) );
	vend();
}

static void treeRound( int round )
{
	int capacities[] = { 0, 1, 16, 40, 5, 100 };
	int capacity = capacities[round % 6];
	g_boxKind = round % 6;
	g_tree = b2CreateDynamicTree( capacity );
	g_liveCount = 0;
	vtag( "round" );
	vi32( round );
	vi32( capacity );
	vhash();

	int maxLive = round < 6 ? 120 : MAX_LIVE;
	int opCount = round < 6 ? 400 : 900;
	for ( int op = 0; op < opCount; ++op )
	{
		int r = vrandint( 0, 100 );
		if ( ( g_liveCount < 6 || r < 30 ) && g_liveCount < maxLive )
		{
			b2AABB box = randBox();
			uint64_t bits = randBits();
			uint64_t userData = vrand64();
			int which = vrandint( 0, 3 );
			int proxyId;
			if ( which == 0 )
			{
				proxyId = b2CreateTreeProxy( g_tree, box, bits, userData );
			}
			else
			{
				proxyId = b2CreateTreeProxyInternal( g_tree, box, bits, userData, which == 2 );
			}
			g_live[g_liveCount++] = proxyId;
			vtag( "create" );
			vi32( which );
			vi32( proxyId );
		}
		else if ( r < 40 )
		{
			int pick = vrandint( 0, g_liveCount );
			int proxyId = g_live[pick];
			g_live[pick] = g_live[g_liveCount - 1];
			g_liveCount -= 1;
			b2DestroyTreeProxy( g_tree, proxyId );
			vtag( "destroy" );
			vi32( proxyId );
		}
		else if ( r < 52 )
		{
			int proxyId = g_live[vrandint( 0, g_liveCount )];
			b2AABB box = randBox();
			int which = vrandint( 0, 3 );
			if ( which == 0 )
			{
				b2DynamicTree_MoveProxy( g_tree, proxyId, box );
			}
			else
			{
				b2DynamicTree_MoveProxyInternal( g_tree, proxyId, box, which == 2 );
			}
			vtag( "move" );
			vi32( which );
			vi32( proxyId );
		}
		else if ( r < 58 )
		{
			int proxyId = g_live[vrandint( 0, g_liveCount )];
			b2AABB box = growBox( b2DynamicTree_GetProxyBounds( g_tree, proxyId ) );
			b2DynamicTree_EnlargeProxy( g_tree, proxyId, box );
			vtag( "enlarge" );
			vi32( proxyId );
		}
		else if ( r < 64 )
		{
			int proxyId = g_live[vrandint( 0, g_liveCount )];
			b2AABB box = growBox( b2DynamicTree_GetProxyBounds( g_tree, proxyId ) );
			b2DynamicTree_MarkProxyMoved( g_tree, proxyId, box );
			vtag( "mark" );
			vi32( proxyId );
		}
		else if ( r < 67 )
		{
			int proxyId = g_live[vrandint( 0, g_liveCount )];
			b2DynamicTree_MarkProxyMovedSerial( g_tree, proxyId );
			vtag( "marks" );
			vi32( proxyId );
		}
		else if ( r < 69 )
		{
			int proxyId = g_live[vrandint( 0, g_liveCount )];
			uint64_t bits = randBits();
			b2DynamicTree_SetCategoryBits( g_tree, proxyId, bits );
			vtag( "cat" );
			vi32( proxyId );
			vu64( b2DynamicTree_GetCategoryBits( g_tree, proxyId ) );
			vu64( b2DynamicTree_GetUserData( g_tree, proxyId ) );
		}
		else if ( r < 72 )
		{
			b2DynamicTree_Refit( g_tree );
			vtag( "refit" );
		}
		else if ( r < 78 )
		{
			bool full = r >= 76;
			int count = b2DynamicTree_Rebuild( g_tree, full );
			vtag( "rebuild" );
			vbool( full );
			vi32( count );
		}
		else if ( r < 80 )
		{
			// The solver always refits first; a clear on stale ancestors would leave them unmarked.
			b2DynamicTree_Refit( g_tree );
			b2DynamicTree_ClearMoved( g_tree );
			vtag( "clear" );
		}
		else if ( r < 82 )
		{
			int count = b2DynamicTree_GatherMovedProxies( g_tree, g_moved );
			vtag( "gather" );
			vi32( count );
			for ( int i = 0; i < count; ++i )
			{
				vi32( g_moved[i] );
			}
		}
		else if ( r < 90 )
		{
			b2AABB box = randBox();
			uint64_t mask = randBits();
			g_mode = vrandint( 0, 5 );
			g_hitBudget = vrandint( 1, 6 );
			g_nestBudget = 6;
			int visitor = vrandint( 0, 2 );
			(void)visitor;
			bool all = r >= 88;
			vtag( "query" );
			vbool( all );
			vi32( g_mode );
			vend();
			b2TreeStats s;
			if ( all )
			{
				s = b2DynamicTree_OverlapAABBAll( g_tree, box, overlapCallback, NULL );
			}
			else
			{
				s = b2DynamicTree_OverlapAABB( g_tree, box, mask, overlapCallback, NULL );
			}
			vtag( "qs" );
			vi32( s.nodeVisits );
			vi32( s.leafVisits );
		}
		else if ( r < 95 )
		{
			b2RayCastInput input;
			b2AABB box = randBox();
			input.origin = b2AABB_Center( box );
			int kind = vrandint( 0, 5 );
			if ( kind == 0 )
			{
				input.translation.x = vrandf( -60.0f, 60.0f );
				input.translation.y = 0.0f;
			}
			else if ( kind == 1 )
			{
				input.translation.x = 0.0f;
				input.translation.y = vrandf( -60.0f, 60.0f );
			}
			else if ( kind == 2 )
			{
				input.translation = b2Vec2_zero;
			}
			else
			{
				input.translation.x = vrandf( -60.0f, 60.0f );
				input.translation.y = vrandf( -60.0f, 60.0f );
			}
			input.maxFraction = vrandint( 0, 2 ) == 0 ? 1.0f : vrandf( 0.2f, 1.0f );
			uint64_t mask = randBits();
			g_mode = vrandint( 0, 5 );
			g_hitBudget = vrandint( 1, 6 );
			g_nestBudget = 6;
			int visitor = vrandint( 0, 2 );
			(void)visitor;
			vtag( "ray" );
			vi32( g_mode );
			vend();
			b2TreeStats s = b2DynamicTree_CastRay( g_tree, &input, mask, rayCallback, NULL );
			vtag( "rs" );
			vi32( s.nodeVisits );
			vi32( s.leafVisits );
		}
		else
		{
			b2BoxCastInput input;
			b2AABB box = randBox();
			float shrink = vrandf( 0.0f, 0.9f );
			b2Vec2 c = b2AABB_Center( box );
			b2Vec2 e = b2MulSV( 1.0f - shrink, b2AABB_Extents( box ) );
			input.box.lowerBound = b2Sub( c, e );
			input.box.upperBound = b2Add( c, e );
			input.translation.x = vrandf( -40.0f, 40.0f );
			input.translation.y = vrandf( -40.0f, 40.0f );
			input.maxFraction = vrandint( 0, 2 ) == 0 ? 1.0f : vrandf( 0.2f, 1.0f );
			uint64_t mask = randBits();
			g_mode = vrandint( 0, 5 );
			g_hitBudget = vrandint( 1, 6 );
			g_nestBudget = 6;
			int visitor = vrandint( 0, 2 );
			(void)visitor;
			vtag( "box" );
			vi32( g_mode );
			vend();
			b2TreeStats s = b2DynamicTree_CastBox( g_tree, &input, mask, boxCallback, NULL );
			vtag( "bs" );
			vi32( s.nodeVisits );
			vi32( s.leafVisits );
		}
		vhash();
	}

	dumpTree( "tree", g_tree );

	// Solver style finish, then drain
	b2DynamicTree_Refit( g_tree );
	vtag( "final" );
	vi32( b2DynamicTree_Rebuild( g_tree, false ) );
	vi32( b2DynamicTree_Rebuild( g_tree, true ) );
	vhash();
	while ( g_liveCount > 0 )
	{
		int proxyId = g_live[--g_liveCount];
		b2DestroyTreeProxy( g_tree, proxyId );
	}
	vtag( "drained" );
	vi32( b2DynamicTree_Rebuild( g_tree, true ) );
	vhash();
	dumpTree( "empty", g_tree );

	b2DestroyDynamicTree( g_tree );
	g_tree = NULL;
}

// ---- broad phase ----

static b2World g_world;
static int g_shapeCount;

// Shape s lives on body s / 2, so pairs of shapes share a body
static void addShape( void )
{
	int s = g_shapeCount++;
	b2Shape shape = { 0 };
	shape.id = s;
	shape.bodyId = s >> 1;
	shape.sensorIndex = B2_NULL_INDEX;
	shape.filter = b2DefaultFilter();
	shape.type = b2_circleShape;
	b2Array_Push( g_world.shapes, shape );

	if ( ( s & 1 ) == 0 )
	{
		b2Body body = { 0 };
		body.id = s >> 1;
		body.type = b2_dynamicBody;
		body.headJointKey = B2_NULL_INDEX;
		b2Array_Push( g_world.bodies, body );
	}
}

#define BP_MAX 600

typedef struct LiveProxy
{
	int key;
	int shape;
} LiveProxy;

static LiveProxy g_proxies[BP_MAX];
static int g_proxyCount;
static uint64_t g_pairs[1 << 15];
static int g_pairCount;
static int g_siblings[1 << 13];
static b2NodePair g_seeds[2 * B2_CROSS_SEED_COUNT];
static uint64_t g_tempKeys[1 << 15];

static void removePairsOf( int shape )
{
	int i = 0;
	while ( i < g_pairCount )
	{
		uint64_t key = g_pairs[i];
		int a = (int)( key >> 32 );
		int b = (int)( key & 0xFFFFFFFF );
		if ( a == shape || b == shape )
		{
			b2RemoveKey( &g_world.broadPhase.pairSet, key );
			g_pairs[i] = g_pairs[g_pairCount - 1];
			g_pairCount -= 1;
		}
		else
		{
			i += 1;
		}
	}
}

// b2ParallelFor with one worker and every task inline: the blocks run in order
static void runCross( int itemCount, b2CrossContext* context )
{
	if ( itemCount <= 0 )
	{
		return;
	}
	int minRange = 1;
	int maxBlockCount = 8;
	int blockSize = itemCount <= minRange * maxBlockCount ? minRange : ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
	for ( int start = 0; start < itemCount; start += blockSize )
	{
		int end = start + blockSize < itemCount ? start + blockSize : itemCount;
		b2CrossPairsTask( start, end, 0, context );
	}
}

static void runSelf( int itemCount )
{
	if ( itemCount <= 0 )
	{
		return;
	}
	int minRange = 64;
	int maxBlockCount = 8;
	int blockSize = itemCount <= minRange * maxBlockCount ? minRange : ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
	for ( int start = 0; start < itemCount; start += blockSize )
	{
		int end = start + blockSize < itemCount ? start + blockSize : itemCount;
		b2SelfPairsTask( start, end, 0, &g_world );
	}
}

static void vtrees( void )
{
	vtag( "trees" );
	for ( int t = 0; t < b2_bodyTypeCount; ++t )
	{
		vu64( treeHash( g_world.broadPhase.trees + t ) );
	}
	vu32( g_world.broadPhase.pairSet.count );
	vend();
}

// b2UpdateBroadPhasePairs up to the contacts, which here only enter the pair set
static void updatePairs( void )
{
	b2BroadPhase* bp = &g_world.broadPhase;

	bool needUpdate = b2HasTreeMoved( bp->trees + b2_staticBody );
	needUpdate = needUpdate || b2NeedsRebuild( bp->trees + b2_kinematicBody );
	needUpdate = needUpdate || b2NeedsRebuild( bp->trees + b2_dynamicBody );

	vtag( "update" );
	vbool( needUpdate );
	vend();
	if ( needUpdate == false )
	{
		return;
	}

	b2Array_Clear( g_world.taskContexts.data[0].pairKeys );

	const b2DynamicTree* dynamicTree = bp->trees + b2_dynamicBody;
	bp->movedSiblings = g_siblings;
	int dynamicMoveCount = b2GatherMovedSiblings( dynamicTree, bp->movedSiblings );

	int staticSeedCount = b2GatherCrossSeeds( dynamicTree, bp->trees + b2_staticBody, g_seeds );
	int kinematicSeedCount = b2GatherCrossSeeds( dynamicTree, bp->trees + b2_kinematicBody, g_seeds + staticSeedCount );
	int crossMoveCount = staticSeedCount + kinematicSeedCount;

	vtag( "siblings" );
	vi32( dynamicMoveCount );
	for ( int i = 0; i < dynamicMoveCount; ++i )
	{
		vi32( g_siblings[i] );
	}
	vend();
	vtag( "seeds" );
	vi32( staticSeedCount );
	vi32( kinematicSeedCount );
	for ( int i = 0; i < crossMoveCount; ++i )
	{
		vu32( g_seeds[i].a.flagIndex );
		vu32( g_seeds[i].b.flagIndex );
	}
	vend();

	b2CrossContext crossContext = {
		.world = &g_world,
		.seeds = g_seeds,
		.staticSeedCount = staticSeedCount,
	};
	runCross( crossMoveCount, &crossContext );
	runSelf( dynamicMoveCount );

	b2DynamicTree_ClearMoved( bp->trees + b2_staticBody );

	int dynamicLeaves = b2DynamicTree_Rebuild( bp->trees + b2_dynamicBody, false );
	int kinematicLeaves = b2DynamicTree_Rebuild( bp->trees + b2_kinematicBody, false );

	b2Array( uint64_t )* pairKeys = &g_world.taskContexts.data[0].pairKeys;
	int pairCount = pairKeys->count;
	vtag( "pairs" );
	vi32( dynamicLeaves );
	vi32( kinematicLeaves );
	vi32( pairCount );
	for ( int i = 0; i < pairCount; ++i )
	{
		vu64( pairKeys->data[i] );
	}
	vend();

	if ( pairCount > 1 )
	{
		b2RadixSortKeys( pairKeys->data, g_tempKeys, pairCount );
	}

	vtag( "sorted" );
	for ( int i = 0; i < pairCount; ++i )
	{
		uint64_t key = pairKeys->data[i];
		vu64( key );
		b2AddKey( &bp->pairSet, key );
		g_pairs[g_pairCount++] = key;
	}
	vend();

	bp->movedSiblings = NULL;
	vtrees();
}

static b2AABB bpBox( int type )
{
	b2AABB box;
	float span = type == b2_staticBody ? 40.0f : 25.0f;
	float cx = vrandf( -span, span );
	float cy = vrandf( -span, span );
	float hx = vrandf( 0.1f, type == b2_staticBody ? 6.0f : 2.0f );
	float hy = vrandf( 0.1f, 2.0f );
	box.lowerBound.x = cx - hx;
	box.lowerBound.y = cy - hy;
	box.upperBound.x = cx + hx;
	box.upperBound.y = cy + hy;
	return box;
}

static b2AABB shiftBox( b2AABB a )
{
	float dx = vrandf( -1.0f, 1.0f );
	float dy = vrandf( -1.0f, 1.0f );
	b2AABB b;
	b.lowerBound.x = a.lowerBound.x + dx;
	b.lowerBound.y = a.lowerBound.y + dy;
	b.upperBound.x = a.upperBound.x + dx;
	b.upperBound.y = a.upperBound.y + dy;
	return b;
}

static void broadRound( int round )
{
	b2Capacity capacity = { 0 };
	capacity.staticShapeCount = vrandint( 0, 40 );
	capacity.dynamicShapeCount = vrandint( 0, 80 );
	capacity.contactCount = vrandint( 0, 100 );

	memset( &g_world, 0, sizeof( g_world ) );
	b2CreateBroadPhase( &g_world.broadPhase, &capacity );
	b2Array_Push( g_world.taskContexts, ( b2TaskContext ){ 0 } );
	g_world.workerCount = 1;
	g_shapeCount = 0;
	g_proxyCount = 0;
	g_pairCount = 0;

	vtag( "bround" );
	vi32( round );
	vend();
	vtrees();

	int maxProxies = round < 2 ? 60 : BP_MAX;
	int steps = round < 2 ? 30 : 60;
	for ( int step = 0; step < steps; ++step )
	{
		int ops = vrandint( 1, round < 2 ? 12 : 40 );
		for ( int op = 0; op < ops; ++op )
		{
			int r = vrandint( 0, 100 );
			if ( ( g_proxyCount < 4 || r < 35 ) && g_proxyCount < maxProxies )
			{
				int t = vrandint( 0, 10 );
				int type = t < 3 ? b2_staticBody : ( t < 4 ? b2_kinematicBody : b2_dynamicBody );
				b2AABB box = bpBox( type );
				uint64_t bits = randBits();
				bool force = vrandint( 0, 4 ) == 0;
				addShape();
				int shape = g_shapeCount - 1;
				int key = b2BroadPhase_CreateProxy( &g_world.broadPhase, type, box, bits, shape, force );
				g_proxies[g_proxyCount].key = key;
				g_proxies[g_proxyCount].shape = shape;
				g_proxyCount += 1;
				vtag( "bcreate" );
				vi32( type );
				vbool( force );
				vi32( key );
				vend();
			}
			else if ( r < 45 )
			{
				int pick = vrandint( 0, g_proxyCount );
				LiveProxy p = g_proxies[pick];
				g_proxies[pick] = g_proxies[g_proxyCount - 1];
				g_proxyCount -= 1;
				b2BroadPhase_DestroyProxy( &g_world.broadPhase, p.key );
				removePairsOf( p.shape );
				vtag( "bdestroy" );
				vi32( p.key );
				vend();
			}
			else if ( r < 60 )
			{
				LiveProxy p = g_proxies[vrandint( 0, g_proxyCount )];
				b2AABB box = bpBox( B2_PROXY_TYPE( p.key ) );
				b2BroadPhase_MoveProxy( &g_world.broadPhase, p.key, box );
				vtag( "bmove" );
				vi32( p.key );
				vend();
			}
			else if ( r < 85 )
			{
				LiveProxy p = g_proxies[vrandint( 0, g_proxyCount )];
				int type = B2_PROXY_TYPE( p.key );
				if ( type == b2_staticBody )
				{
					continue;
				}
				b2DynamicTree* tree = g_world.broadPhase.trees + type;
				b2AABB box = shiftBox( b2DynamicTree_GetProxyBounds( tree, B2_PROXY_ID( p.key ) ) );
				int which = vrandint( 0, 3 );
				if ( which == 0 || type == b2_kinematicBody )
				{
					b2BroadPhase_MarkProxyMoved( &g_world.broadPhase, p.key, box );
				}
				else if ( which == 1 )
				{
					b2AABB grown = b2AABB_Union( box, b2DynamicTree_GetProxyBounds( tree, B2_PROXY_ID( p.key ) ) );
					b2DynamicTree_EnlargeProxy( tree, B2_PROXY_ID( p.key ), grown );
				}
				else
				{
					b2BroadPhase_MarkProxyMovedSerial( &g_world.broadPhase, p.key );
				}
				vtag( "bmark" );
				vi32( which );
				vi32( p.key );
				vend();
			}
			else if ( g_pairCount > 0 )
			{
				// a contact ends
				int pick = vrandint( 0, g_pairCount );
				uint64_t key = g_pairs[pick];
				b2RemoveKey( &g_world.broadPhase.pairSet, key );
				g_pairs[pick] = g_pairs[g_pairCount - 1];
				g_pairCount -= 1;
				vtag( "bunpair" );
				vu64( key );
				vend();
			}
		}

		b2DynamicTree_Refit( g_world.broadPhase.trees + b2_kinematicBody );
		b2DynamicTree_Refit( g_world.broadPhase.trees + b2_dynamicBody );
		b2ValidateBroadphase( &g_world.broadPhase );
		updatePairs();
	}

	for ( int t = 0; t < b2_bodyTypeCount; ++t )
	{
		dumpTree( "btree", g_world.broadPhase.trees + t );
	}

	b2DestroyBroadPhase( &g_world.broadPhase );
	b2Array_Destroy( g_world.shapes );
	b2Array_Destroy( g_world.bodies );
	b2Array_Destroy( g_world.taskContexts.data[0].pairKeys );
	b2Array_Destroy( g_world.taskContexts );
}

// ---- radix sort ----

static uint64_t g_keys[5000];

static void sortChecks( void )
{
	vseed( 33 );
	int counts[] = { 2, 3, 17, 256, 1000, 4999 };
	for ( int c = 0; c < 6; ++c )
	{
		for ( int kind = 0; kind < 4; ++kind )
		{
			int count = counts[c];
			for ( int i = 0; i < count; ++i )
			{
				uint64_t key;
				if ( kind == 0 )
				{
					key = vrand64();
				}
				else if ( kind == 1 )
				{
					int a = vrandint( 0, 300 );
					int b = vrandint( 0, 300 );
					key = B2_SHAPE_PAIR_KEY( a, b );
				}
				else if ( kind == 2 )
				{
					key = (uint64_t)vrandint( 0, 4 ) << 40;
				}
				else
				{
					key = 77;
				}
				g_keys[i] = key;
			}
			b2RadixSortKeys( g_keys, g_tempKeys, count );
			vtag( "sort" );
			vi32( count );
			vi32( kind );
			for ( int i = 0; i < count; ++i )
			{
				vu64( g_keys[i] );
			}
			vend();
		}
	}
}

int main( void )
{
	aabbChecks();

	vseed( 7 );
	for ( int round = 0; round < 18; ++round )
	{
		treeRound( round );
	}

	vseed( 11 );
	for ( int round = 0; round < 8; ++round )
	{
		broadRound( round );
	}

	sortChecks();
	return 0;
}
