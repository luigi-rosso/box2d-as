// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/contact_solver.as: the overflow kernels of contact_solver.c and the
// B2_SIMD_NONE wide kernels of contact_solver_wide.inl, included here at width 4 as
// contact_solver_w4.c does, driven over synthetic colors, contacts and body states.

#define B2_SIMD_WIDTH 4
#include "contact_solver_wide.inl"

#include "bitset.h"

#include "common.h"

#include <stdlib.h>

#define MAX_STATES 24
#define MAX_CONTACTS 16
#define COLOR_COUNT 2
#define MAX_WIDE ( COLOR_COUNT * MAX_CONTACTS / 4 )

static const int colorIds[COLOR_COUNT] = { 3, 7 };

static b2World world;
static b2StepContext ctx;
static b2BodyState states[MAX_STATES];
static b2SolverSet sets[3];
static b2TaskContext taskContext;
static b2ContactSim colorSims[COLOR_COUNT][MAX_CONTACTS];
static b2ContactSim overflowSims[MAX_CONTACTS];
static b2ContactConstraintWide wide[MAX_WIDE];
static b2ContactConstraint overflowConstraints[MAX_CONTACTS];
static b2ContactPrepareSpan spans[COLOR_COUNT + 1];
static int stateCount;
static int colorCounts[COLOR_COUNT];
static int wideStarts[COLOR_COUNT];
static int wideCounts[COLOR_COUNT];
static int wideTotal;
static int overflowCount;

static void words( const void* p, int byteCount )
{
	const float* f = p;
	for ( int i = 0; i < byteCount / 4; ++i )
	{
		vf32( f[i] );
	}
}

static void genState( b2BodyState* s )
{
	int kind = vrandint( 0, 8 );
	s->flags = kind == 0 ? 0 : b2_dynamicFlag;
	s->linearVelocity.x = vrandf( -5.0f, 5.0f );
	s->linearVelocity.y = vrandf( -5.0f, 5.0f );
	s->angularVelocity = vrandf( -5.0f, 5.0f );
	s->deltaPosition.x = vrandf( -0.1f, 0.1f );
	s->deltaPosition.y = vrandf( -0.1f, 0.1f );
	float angle = vrandf( -0.2f, 0.2f );
	s->deltaRotation = b2MakeRot( angle );
}

static bool isDynamic( int encoded )
{
	return encoded >= 0 && ( states[encoded].flags & b2_dynamicFlag ) != 0;
}

// An encoded body sim: static (negative), the first kinematic state, or the next dynamic state from cursor
static int pickBody( int* cursor )
{
	int kind = vrandint( 0, 4 );
	if ( kind == 0 )
	{
		int staticIndex = vrandint( 0, 3 );
		return -( staticIndex + 2 );
	}

	if ( kind == 1 )
	{
		for ( int i = 0; i < stateCount; ++i )
		{
			if ( ( states[i].flags & b2_dynamicFlag ) == 0 )
			{
				return i;
			}
		}
		return -2;
	}

	while ( *cursor < stateCount && ( states[*cursor].flags & b2_dynamicFlag ) == 0 )
	{
		*cursor += 1;
	}

	if ( *cursor < stateCount )
	{
		int i = *cursor;
		*cursor += 1;
		return i;
	}

	return -2;
}

static void genContact( b2ContactSim* c, int encodedA, int encodedB, int contactId )
{
	memset( c, 0, sizeof( *c ) );
	c->contactId = contactId;
	c->encodedBodySimA = encodedA;
	c->encodedBodySimB = encodedB;

	int hit = vrandint( 0, 2 );
	c->simFlags = hit == 0 ? b2_simEnableHitEvent : 0;

	c->invMassA = isDynamic( encodedA ) ? vrandf( 0.1f, 2.0f ) : 0.0f;
	c->invIA = isDynamic( encodedA ) ? vrandf( 0.1f, 2.0f ) : 0.0f;
	c->invMassB = isDynamic( encodedB ) ? vrandf( 0.1f, 2.0f ) : 0.0f;
	c->invIB = isDynamic( encodedB ) ? vrandf( 0.1f, 2.0f ) : 0.0f;

	float angle = vrandf( -3.14f, 3.14f );
	b2Rot r = b2MakeRot( angle );
	c->manifold.normal.x = r.c;
	c->manifold.normal.y = r.s;

	c->friction = vrandf( 0.0f, 1.0f );
	int pick = vrandint( 0, 3 );
	c->restitution = pick == 0 ? vrandf( 0.0f, 1.0f ) : 0.0f;
	pick = vrandint( 0, 4 );
	c->rollingResistance = pick == 0 ? vrandf( 0.0f, 0.5f ) : 0.0f;
	pick = vrandint( 0, 4 );
	c->tangentSpeed = pick == 0 ? vrandf( -1.0f, 1.0f ) : 0.0f;

	c->manifold.pointCount = vrandint( 1, 3 );
	c->manifold.rollingImpulse = vrandf( -0.1f, 0.1f );

	// Both points, so a one point lane's stale second point is read as C reads it
	for ( int p = 0; p < 2; ++p )
	{
		b2ManifoldPoint* mp = c->manifold.points + p;
		mp->anchorA.x = vrandf( -1.0f, 1.0f );
		mp->anchorA.y = vrandf( -1.0f, 1.0f );
		mp->anchorB.x = vrandf( -1.0f, 1.0f );
		mp->anchorB.y = vrandf( -1.0f, 1.0f );
		mp->separation = vrandf( -0.05f, 0.05f );
		mp->normalImpulse = vrandf( 0.0f, 2.0f );
		mp->tangentImpulse = vrandf( -1.0f, 1.0f );
		mp->totalNormalImpulse = vrandf( 0.0f, 1.0f );
		mp->normalVelocity = vrandf( -5.0f, 5.0f );
	}
}

static void dumpSim( const b2ContactSim* c )
{
	vf32( c->manifold.rollingImpulse );
	for ( int p = 0; p < 2; ++p )
	{
		const b2ManifoldPoint* mp = c->manifold.points + p;
		vf32( mp->normalImpulse );
		vf32( mp->tangentImpulse );
		vf32( mp->totalNormalImpulse );
		vf32( mp->normalVelocity );
	}
}

static void dump( const char* phase )
{
	vtag( phase );
	vi32( b2AtomicLoadInt( &ctx.anyRestitution ) );
	vbool( taskContext.hasHitEvents );
	vu64( taskContext.hitEventBitSet.bits[0] );
	vend();

	for ( int i = 0; i < stateCount; ++i )
	{
		vtag( "state" );
		words( states + i, sizeof( b2BodyState ) );
		vend();
	}

	for ( int i = 0; i < wideTotal; ++i )
	{
		vtag( "wide" );
		words( wide + i, sizeof( b2ContactConstraintWide ) );
		vend();
	}

	for ( int i = 0; i < overflowCount; ++i )
	{
		vtag( "ovf" );
		words( overflowConstraints + i, sizeof( b2ContactConstraint ) );
		vend();
	}

	for ( int k = 0; k < COLOR_COUNT; ++k )
	{
		for ( int i = 0; i < colorCounts[k]; ++i )
		{
			vtag( "sim" );
			dumpSim( colorSims[k] + i );
			vend();
		}
	}

	for ( int i = 0; i < overflowCount; ++i )
	{
		vtag( "osim" );
		dumpSim( overflowSims + i );
		vend();
	}
}

static b2SolverBlock makeBlock( int start, int count, int colorIndex )
{
	b2SolverBlock block = { 0 };
	block.startIndex = start;
	block.count = (uint16_t)count;
	block.blockType = b2_graphContactBlock;
	block.colorIndex = (uint8_t)colorIndex;
	return block;
}

static void scenario( void )
{
	stateCount = vrandint( 4, MAX_STATES + 1 );
	for ( int i = 0; i < stateCount; ++i )
	{
		genState( states + i );
	}

	memset( wide, 0, sizeof( wide ) );
	memset( overflowConstraints, 0, sizeof( overflowConstraints ) );

	int contactId = 0;
	wideTotal = 0;
	for ( int k = 0; k < COLOR_COUNT; ++k )
	{
		int cursor = 0;
		int count = vrandint( 1, MAX_CONTACTS + 1 );
		for ( int i = 0; i < count; ++i )
		{
			int a = pickBody( &cursor );
			int b = pickBody( &cursor );
			genContact( colorSims[k] + i, a, b, contactId );
			contactId += 1;
		}

		colorCounts[k] = count;
		wideStarts[k] = wideTotal;
		wideCounts[k] = ( ( count - 1 ) >> 2 ) + 1;
		wideTotal += wideCounts[k];
	}

	overflowCount = vrandint( 0, 7 );
	for ( int i = 0; i < overflowCount; ++i )
	{
		int ca = vrandint( 0, stateCount );
		int cb = vrandint( 0, stateCount );
		int a = pickBody( &ca );
		int b = pickBody( &cb );
		genContact( overflowSims + i, a, b, contactId );
		contactId += 1;
	}

	world.hitEventThreshold = vrandf( 0.5f, 2.0f );
	world.restitutionThreshold = vrandf( 0.5f, 2.0f );
	int propagate = vrandint( 0, 2 );
	world.enableRestitutionPropagation = propagate == 1;
	int warm = vrandint( 0, 4 );
	world.enableWarmStarting = warm != 0;
	world.contactSpeed = 3.0f;

	float dt = 1.0f / 60.0f;
	ctx.dt = dt;
	ctx.inv_dt = 1.0f / dt;
	ctx.subStepCount = 4;
	ctx.h = dt / 4.0f;
	ctx.inv_h = 4.0f * ctx.inv_dt;
	ctx.contactSoftness = b2MakeSoft( 30.0f, 10.0f, ctx.h );
	ctx.staticSoftness = b2MakeSoft( 60.0f, 10.0f, ctx.h );
	b2AtomicStoreInt( &ctx.anyRestitution, 0 );

	sets[b2_awakeSet].bodyStates.data = states;
	sets[b2_awakeSet].bodyStates.count = stateCount;
	sets[b2_awakeSet].bodyStates.capacity = MAX_STATES;

	b2SetBitCountAndClear( &taskContext.hitEventBitSet, 64 );
	taskContext.hasHitEvents = false;

	for ( int k = 0; k < COLOR_COUNT; ++k )
	{
		b2GraphColor* color = world.constraintGraph.colors + colorIds[k];
		color->contactSims.data = colorSims[k];
		color->contactSims.count = colorCounts[k];
		color->contactSims.capacity = MAX_CONTACTS;
		color->wideConstraints = wide + wideStarts[k];
		color->wideConstraintCount = wideCounts[k];

		spans[k].start = wideStarts[k];
		spans[k].count = colorCounts[k];
		spans[k].contacts = colorSims[k];
	}

	spans[COLOR_COUNT].start = wideTotal;
	spans[COLOR_COUNT].count = 0;
	spans[COLOR_COUNT].contacts = NULL;

	b2GraphColor* overflow = world.constraintGraph.colors + B2_OVERFLOW_INDEX;
	overflow->contactSims.data = overflowSims;
	overflow->contactSims.count = overflowCount;
	overflow->contactSims.capacity = MAX_CONTACTS;
	overflow->overflowConstraints = overflowConstraints;

	// Prepare and store split the flat wide range into two blocks
	int split = vrandint( 0, wideTotal + 1 );
	vtag( "scenario" );
	vi32( stateCount );
	vi32( wideTotal );
	vi32( overflowCount );
	vi32( split );
	vend();

	if ( split > 0 )
	{
		b2PrepareContacts_WideW4( makeBlock( 0, split, 0 ), &ctx );
	}
	if ( wideTotal - split > 0 )
	{
		b2PrepareContacts_WideW4( makeBlock( split, wideTotal - split, 0 ), &ctx );
	}
	b2PrepareContacts_Overflow( &ctx );
	dump( "prepare" );

	float h = ctx.h;
	for ( int sub = 0; sub < 4; ++sub )
	{
		b2WarmStartContacts_Overflow( &ctx );
		for ( int k = 0; k < COLOR_COUNT; ++k )
		{
			b2WarmStartContacts_WideW4( makeBlock( 0, wideCounts[k], colorIds[k] ), &ctx );
		}

		b2PushContacts_Overflow( &ctx );
		for ( int k = 0; k < COLOR_COUNT; ++k )
		{
			b2PushContacts_WideW4( makeBlock( 0, wideCounts[k], colorIds[k] ), &ctx );
		}

		for ( int i = 0; i < stateCount; ++i )
		{
			b2BodyState* s = states + i;
			if ( s->flags & b2_dynamicFlag )
			{
				s->deltaPosition = b2MulAdd( s->deltaPosition, h, s->linearVelocity );
				s->deltaRotation = b2IntegrateRotation( s->deltaRotation, h * s->angularVelocity );
			}
		}

		b2SolveContacts_Overflow( &ctx );
		for ( int k = 0; k < COLOR_COUNT; ++k )
		{
			b2SolveContacts_WideW4( makeBlock( 0, wideCounts[k], colorIds[k] ), &ctx );
		}
	}
	dump( "solve" );

	for ( int j = 0; j < 2; ++j )
	{
		b2ApplyRestitution_Overflow( &ctx );
		for ( int k = 0; k < COLOR_COUNT; ++k )
		{
			b2ApplyRestitution_WideW4( makeBlock( 0, wideCounts[k], colorIds[k] ), &ctx );
		}
	}
	dump( "restitution" );

	b2StoreImpulses_Overflow( &ctx );
	if ( split > 0 )
	{
		b2StoreImpulses_WideW4( makeBlock( 0, split, 0 ), &ctx, 0 );
	}
	if ( wideTotal - split > 0 )
	{
		b2StoreImpulses_WideW4( makeBlock( split, wideTotal - split, 0 ), &ctx, 0 );
	}
	dump( "store" );
}

int main( void )
{
	vtag( "sizes" );
	vi32( (int)sizeof( b2ContactConstraintWide ) );
	vi32( (int)sizeof( b2ContactConstraint ) );
	vi32( (int)sizeof( b2BodyState ) );
	vend();

	world.solverSets.data = sets;
	world.solverSets.count = 3;
	world.solverSets.capacity = 3;
	world.taskContexts.data = &taskContext;
	world.taskContexts.count = 1;
	world.taskContexts.capacity = 1;
	taskContext.hitEventBitSet = b2CreateBitSet( 64 );

	ctx.world = &world;
	ctx.graph = &world.constraintGraph;
	ctx.states = states;
	ctx.contactPrepareSpans = spans;
	ctx.wideContactConstraints = wide;

	vseed( 0x5eed2d );
	for ( int i = 0; i < 400; ++i )
	{
		scenario();
	}

	return 0;
}
