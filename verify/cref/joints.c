// Joint prepare, warm start and solve run by hand on a world built through the public API, plus
// every joint getter and setter. The solver stage is not used, so this checks the joint files alone.
#include "bitset.h"
#include "body.h"
#include "constraint_graph.h"
#include "joint.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"
#include "box2d/box2d.h"

#include "common.h"

static void silentLog( const char* m )
{
	(void)m;
}

static b2WorldId worldId;
static b2BodyId bodies[9];
static b2JointId joints[16];
static int jointCount;

static void vvec( b2Vec2 v )
{
	vf32( v.x );
	vf32( v.y );
}

static void emitStates( const char* tag )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* set = world->solverSets.data + b2_awakeSet;
	for ( int i = 0; i < set->bodyStates.count; ++i )
	{
		b2BodyState* s = set->bodyStates.data + i;
		vtag( tag );
		vi32( i );
		vvec( s->linearVelocity );
		vf32( s->angularVelocity );
		vend();
	}
}

static void emitSims( const char* tag )
{
	b2World* world = b2GetWorldFromId( worldId );
	for ( int c = 0; c < B2_GRAPH_COLOR_COUNT; ++c )
	{
		b2GraphColor* color = world->constraintGraph.colors + c;
		for ( int i = 0; i < color->jointSims.count; ++i )
		{
			b2JointSim* sim = color->jointSims.data + i;
			float force, torque;
			b2GetJointReaction( sim, 60.0f, &force, &torque );
			vtag( tag );
			vi32( c );
			vi32( sim->jointId );
			vf32( force );
			vf32( torque );
			vf32( sim->invMassA );
			vf32( sim->invIB );
			vf32( sim->constraintSoftness.biasRate );
			vf32( sim->constraintSoftness.massScale );
			vend();
		}
	}
}

static void addJoint( b2JointId id )
{
	joints[jointCount++] = id;
}

static void build( void )
{
	b2WorldDef wd = b2DefaultWorldDef();
	worldId = b2CreateWorld( &wd );

	b2BodyDef bd = b2DefaultBodyDef();
	bodies[0] = b2CreateBody( worldId, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Polygon ground = b2MakeBox( 10.0f, 0.5f );
	b2CreatePolygonShape( bodies[0], &sd, &ground );

	for ( int i = 1; i < 9; ++i )
	{
		bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = (b2Vec2){ -4.0f + 1.1f * (float)i, 2.0f + 0.3f * (float)i };
		bd.rotation = b2MakeRot( 0.2f * (float)i );
		bodies[i] = b2CreateBody( worldId, &bd );
		b2Polygon box = b2MakeOffsetBox( 0.4f, 0.2f + 0.05f * (float)i, (b2Vec2){ 0.1f, -0.05f }, b2MakeRot( 0.1f ) );
		b2CreatePolygonShape( bodies[i], &sd, &box );
	}

	{
		b2DistanceJointDef d = b2DefaultDistanceJointDef();
		d.base.bodyIdA = bodies[0];
		d.base.bodyIdB = bodies[1];
		d.base.localFrameA.p = (b2Vec2){ -3.0f, 4.0f };
		d.base.localFrameB.p = (b2Vec2){ 0.2f, 0.1f };
		d.length = 1.5f;
		addJoint( b2CreateDistanceJoint( worldId, &d ) );

		d = b2DefaultDistanceJointDef();
		d.base.bodyIdA = bodies[1];
		d.base.bodyIdB = bodies[2];
		d.base.localFrameB.p = (b2Vec2){ -0.3f, 0.1f };
		d.length = 1.2f;
		d.enableSpring = true;
		d.hertz = 3.0f;
		d.dampingRatio = 0.4f;
		d.lowerSpringForce = -50.0f;
		d.upperSpringForce = 40.0f;
		d.enableLimit = true;
		d.minLength = 0.9f;
		d.maxLength = 1.1f;
		d.enableMotor = true;
		d.maxMotorForce = 5.0f;
		d.motorSpeed = 0.5f;
		d.base.forceThreshold = 0.0f;
		addJoint( b2CreateDistanceJoint( worldId, &d ) );
	}

	{
		b2MotorJointDef m = b2DefaultMotorJointDef();
		m.base.bodyIdA = bodies[0];
		m.base.bodyIdB = bodies[3];
		m.base.localFrameA.p = (b2Vec2){ -0.5f, 3.0f };
		m.base.localFrameA.q = b2MakeRot( 0.3f );
		m.linearVelocity = (b2Vec2){ 0.5f, -0.2f };
		m.maxVelocityForce = 20.0f;
		m.angularVelocity = 0.7f;
		m.maxVelocityTorque = 4.0f;
		m.linearHertz = 2.0f;
		m.linearDampingRatio = 0.6f;
		m.maxSpringForce = 30.0f;
		m.angularHertz = 1.5f;
		m.angularDampingRatio = 0.5f;
		m.maxSpringTorque = 6.0f;
		addJoint( b2CreateMotorJoint( worldId, &m ) );
	}

	{
		b2MoverJointDef m = b2DefaultMoverJointDef();
		m.base.bodyIdA = bodies[0];
		m.base.bodyIdB = bodies[4];
		m.linearVelocity = (b2Vec2){ 1.0f, 0.3f };
		m.maxVelocityForce = (b2Vec2){ 10.0f, 2.0f };
		addJoint( b2CreateMoverJoint( worldId, &m ) );
	}

	{
		b2PogoJointDef p = b2DefaultPogoJointDef();
		p.base.bodyIdA = bodies[0];
		p.base.bodyIdB = bodies[5];
		p.base.localFrameA.p = (b2Vec2){ 1.5f, 0.5f };
		p.normal = b2Normalize( (b2Vec2){ 0.2f, 1.0f } );
		p.restLength = 3.0f;
		p.hertz = 4.0f;
		p.dampingRatio = 0.8f;
		p.maxTensionForce = 15.0f;
		p.maxCompressionForce = 60.0f;
		p.impulse = 0.05f;
		p.velocity = -0.1f;
		addJoint( b2CreatePogoJoint( worldId, &p ) );
	}

	{
		b2PrismaticJointDef p = b2DefaultPrismaticJointDef();
		p.base.bodyIdA = bodies[2];
		p.base.bodyIdB = bodies[3];
		p.base.localFrameA.q = b2MakeRot( 0.6f );
		p.base.localFrameA.p = (b2Vec2){ 0.3f, 0.2f };
		p.base.localFrameB.p = (b2Vec2){ -0.2f, 0.0f };
		p.enableSpring = true;
		p.hertz = 2.5f;
		p.dampingRatio = 0.3f;
		p.targetTranslation = 0.4f;
		p.enableLimit = true;
		p.lowerTranslation = -0.2f;
		p.upperTranslation = 0.3f;
		p.enableMotor = true;
		p.maxMotorForce = 8.0f;
		p.motorSpeed = -0.6f;
		p.base.torqueThreshold = 0.5f;
		addJoint( b2CreatePrismaticJoint( worldId, &p ) );
	}

	{
		b2RevoluteJointDef r = b2DefaultRevoluteJointDef();
		r.base.bodyIdA = bodies[3];
		r.base.bodyIdB = bodies[4];
		r.base.localFrameA.p = (b2Vec2){ 0.5f, 0.1f };
		r.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };
		r.base.localFrameB.q = b2MakeRot( -0.4f );
		r.targetAngle = 0.3f;
		r.enableSpring = true;
		r.hertz = 1.0f;
		r.dampingRatio = 0.2f;
		r.enableLimit = true;
		r.lowerAngle = -0.25f;
		r.upperAngle = 0.1f;
		r.enableMotor = true;
		r.maxMotorTorque = 3.0f;
		r.motorSpeed = 1.2f;
		addJoint( b2CreateRevoluteJoint( worldId, &r ) );
	}

	{
		b2WeldJointDef w = b2DefaultWeldJointDef();
		w.base.bodyIdA = bodies[5];
		w.base.bodyIdB = bodies[6];
		w.base.localFrameA.p = (b2Vec2){ 0.5f, 0.0f };
		w.base.localFrameB.p = (b2Vec2){ -0.5f, 0.1f };
		addJoint( b2CreateWeldJoint( worldId, &w ) );

		w = b2DefaultWeldJointDef();
		w.base.bodyIdA = bodies[6];
		w.base.bodyIdB = bodies[7];
		w.base.localFrameA.p = (b2Vec2){ 0.4f, 0.0f };
		w.base.localFrameB.p = (b2Vec2){ -0.4f, 0.0f };
		w.linearHertz = 5.0f;
		w.linearDampingRatio = 0.7f;
		w.angularHertz = 3.0f;
		w.angularDampingRatio = 0.4f;
		addJoint( b2CreateWeldJoint( worldId, &w ) );
	}

	{
		b2WheelJointDef w = b2DefaultWheelJointDef();
		w.base.bodyIdA = bodies[7];
		w.base.bodyIdB = bodies[8];
		w.base.localFrameA.q = b2MakeRot( 1.3f );
		w.base.localFrameA.p = (b2Vec2){ 0.0f, -0.3f };
		w.enableLimit = true;
		w.lowerTranslation = -0.1f;
		w.upperTranslation = 0.15f;
		w.enableMotor = true;
		w.maxMotorTorque = 2.0f;
		w.motorSpeed = -3.0f;
		addJoint( b2CreateWheelJoint( worldId, &w ) );
	}

	{
		b2FilterJointDef f = b2DefaultFilterJointDef();
		f.base.bodyIdA = bodies[1];
		f.base.bodyIdB = bodies[8];
		addJoint( b2CreateFilterJoint( worldId, &f ) );
	}

	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* set = world->solverSets.data + b2_awakeSet;
	vseed( 7 );
	for ( int i = 0; i < set->bodyStates.count; ++i )
	{
		b2BodyState* s = set->bodyStates.data + i;
		float vx = vrandf( -2.0f, 2.0f );
		float vy = vrandf( -2.0f, 2.0f );
		float w = vrandf( -3.0f, 3.0f );
		s->linearVelocity = (b2Vec2){ vx, vy };
		s->angularVelocity = w;
	}
}

static void stepJoints( bool warmStarting )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* set = world->solverSets.data + b2_awakeSet;

	b2StepContext ctx = { 0 };
	ctx.world = world;
	ctx.graph = &world->constraintGraph;
	ctx.dt = 1.0f / 60.0f;
	ctx.inv_dt = 60.0f;
	ctx.subStepCount = 4;
	ctx.h = ctx.dt / 4.0f;
	ctx.inv_h = 4.0f * 60.0f;
	ctx.states = set->bodyStates.data;
	ctx.enableWarmStarting = warmStarting;
	world->inv_h = ctx.inv_h;

	b2JointPrepareSpan spans[B2_GRAPH_COLOR_COUNT + 1];
	int spanCount = 0;
	int total = 0;
	for ( int c = 0; c < B2_OVERFLOW_INDEX; ++c )
	{
		b2GraphColor* color = world->constraintGraph.colors + c;
		if ( color->jointSims.count == 0 )
		{
			continue;
		}
		spans[spanCount].start = total;
		spans[spanCount].count = color->jointSims.count;
		spans[spanCount].joints = color->jointSims.data;
		total += color->jointSims.count;
		spanCount += 1;
	}
	spans[spanCount].start = total;
	spans[spanCount].count = 0;
	spans[spanCount].joints = NULL;
	ctx.jointPrepareSpans = spans;
	ctx.jointCount = total;

	b2BitSet* bits = &world->taskContexts.data[0].jointStateBitSet;
	b2SetBitCountAndClear( bits, world->joints.count );

	// two blocks so the color search runs from an inner start
	int half = total / 2;
	b2PrepareJointsTask( (b2SolverBlock){ .startIndex = 0, .count = (uint16_t)half }, &ctx );
	b2PrepareJointsTask( (b2SolverBlock){ .startIndex = half, .count = (uint16_t)( total - half ) }, &ctx );
	b2PrepareJoints_Overflow( &ctx );
	emitSims( "p" );

	for ( int sub = 0; sub < 4; ++sub )
	{
		for ( int c = 0; c < B2_OVERFLOW_INDEX; ++c )
		{
			int n = world->constraintGraph.colors[c].jointSims.count;
			if ( n > 0 )
			{
				b2WarmStartJointsTask( (b2SolverBlock){ .startIndex = 0, .count = (uint16_t)n, .colorIndex = (uint8_t)c }, &ctx );
			}
		}
		b2WarmStartJoints_Overflow( &ctx );
		emitStates( "w" );

		for ( int c = 0; c < B2_OVERFLOW_INDEX; ++c )
		{
			int n = world->constraintGraph.colors[c].jointSims.count;
			if ( n > 0 )
			{
				b2SolveJointsTask( (b2SolverBlock){ .startIndex = 0, .count = (uint16_t)n, .colorIndex = (uint8_t)c }, &ctx, true, 0 );
			}
		}
		b2SolveJoints_Overflow( &ctx, true );
		emitStates( "s" );

		for ( int i = 0; i < set->bodyStates.count; ++i )
		{
			b2BodyState* s = set->bodyStates.data + i;
			s->deltaPosition = b2MulAdd( s->deltaPosition, ctx.h, s->linearVelocity );
			s->deltaRotation = b2IntegrateRotation( s->deltaRotation, ctx.h * s->angularVelocity );
		}

		for ( int c = 0; c < B2_OVERFLOW_INDEX; ++c )
		{
			int n = world->constraintGraph.colors[c].jointSims.count;
			if ( n > 0 )
			{
				b2SolveJointsTask( (b2SolverBlock){ .startIndex = 0, .count = (uint16_t)n, .colorIndex = (uint8_t)c }, &ctx, false, 0 );
			}
		}
		b2SolveJoints_Overflow( &ctx, false );
		emitStates( "r" );
		emitSims( "j" );
	}

	vtag( "bits" );
	for ( int i = 0; i < jointCount; ++i )
	{
		vbool( b2GetBit( bits, joints[i].index1 - 1 ) );
	}
	vend();
}

static void emitApi( void )
{
	for ( int i = 0; i < jointCount; ++i )
	{
		b2JointId id = joints[i];
		vtag( "a" );
		vi32( b2Joint_GetType( id ) );
		vi32( b2Joint_GetBodyA( id ).index1 );
		vi32( b2Joint_GetBodyB( id ).index1 );
		vbool( b2Joint_GetCollideConnected( id ) );
		vvec( b2Joint_GetConstraintForce( id ) );
		b2JointType type = b2Joint_GetType( id );
		// C asserts on these for the mover and pogo joints
		if ( type != b2_moverJoint && type != b2_pogoJoint )
		{
			vf32( b2Joint_GetLinearSeparation( id ) );
			vf32( b2Joint_GetAngularSeparation( id ) );
			vf32( b2Joint_GetConstraintTorque( id ) );
		}
		float hz, zeta;
		b2Joint_GetConstraintTuning( id, &hz, &zeta );
		vf32( hz );
		vf32( zeta );
		vf32( b2Joint_GetForceThreshold( id ) );
		vf32( b2Joint_GetTorqueThreshold( id ) );
		vvec( b2Joint_GetLocalFrameA( id ).p );
		vvec( b2Joint_GetLocalFrameB( id ).p );
		vf32( b2Joint_GetLocalFrameB( id ).q.s );
		switch ( type )
		{
			case b2_distanceJoint:
			{
				float lo, hi;
				b2DistanceJoint_GetSpringForceRange( id, &lo, &hi );
				vf32( b2DistanceJoint_GetRestLength( id ) );
				vf32( b2DistanceJoint_GetLowerLimit( id ) );
				vf32( b2DistanceJoint_GetUpperLimit( id ) );
				vf32( b2DistanceJoint_GetCurrentLength( id ) );
				vbool( b2DistanceJoint_IsSpringEnabled( id ) );
				vbool( b2DistanceJoint_IsLimitEnabled( id ) );
				vbool( b2DistanceJoint_IsMotorEnabled( id ) );
				vf32( lo );
				vf32( hi );
				vf32( b2DistanceJoint_GetSpringHertz( id ) );
				vf32( b2DistanceJoint_GetSpringDampingRatio( id ) );
				vf32( b2DistanceJoint_GetSpringForce( id ) );
				vf32( b2DistanceJoint_GetTargetMotorSpeed( id ) );
				vf32( b2DistanceJoint_GetCurrentMotorForce( id ) );
				vf32( b2DistanceJoint_GetMaxMotorForce( id ) );
			}
			break;
			case b2_motorJoint:
				vvec( b2MotorJoint_GetLinearVelocity( id ) );
				vf32( b2MotorJoint_GetAngularVelocity( id ) );
				vf32( b2MotorJoint_GetMaxVelocityForce( id ) );
				vf32( b2MotorJoint_GetMaxVelocityTorque( id ) );
				vf32( b2MotorJoint_GetLinearHertz( id ) );
				vf32( b2MotorJoint_GetLinearDampingRatio( id ) );
				vf32( b2MotorJoint_GetAngularHertz( id ) );
				vf32( b2MotorJoint_GetAngularDampingRatio( id ) );
				vf32( b2MotorJoint_GetMaxSpringForce( id ) );
				vf32( b2MotorJoint_GetMaxSpringTorque( id ) );
				break;
			case b2_moverJoint:
				vvec( b2MoverJoint_GetLinearVelocity( id ) );
				vvec( b2MoverJoint_GetMaxVelocityForce( id ) );
				break;
			case b2_pogoJoint:
				vf32( b2PogoJoint_GetRestLength( id ) );
				vf32( b2PogoJoint_GetSpringHertz( id ) );
				vf32( b2PogoJoint_GetSpringDampingRatio( id ) );
				vf32( b2PogoJoint_GetCurrentLength( id ) );
				vf32( b2PogoJoint_GetImpulse( id ) );
				vf32( b2PogoJoint_GetCurrentSpeed( id ) );
				break;
			case b2_prismaticJoint:
				vbool( b2PrismaticJoint_IsSpringEnabled( id ) );
				vf32( b2PrismaticJoint_GetSpringHertz( id ) );
				vf32( b2PrismaticJoint_GetSpringDampingRatio( id ) );
				vf32( b2PrismaticJoint_GetTargetTranslation( id ) );
				vbool( b2PrismaticJoint_IsLimitEnabled( id ) );
				vf32( b2PrismaticJoint_GetLowerLimit( id ) );
				vf32( b2PrismaticJoint_GetUpperLimit( id ) );
				vbool( b2PrismaticJoint_IsMotorEnabled( id ) );
				vf32( b2PrismaticJoint_GetTargetMotorSpeed( id ) );
				vf32( b2PrismaticJoint_GetMaxMotorForce( id ) );
				vf32( b2PrismaticJoint_GetCurrentMotorForce( id ) );
				vf32( b2PrismaticJoint_GetCurrentTranslation( id ) );
				vf32( b2PrismaticJoint_GetCurrentSpeed( id ) );
				break;
			case b2_revoluteJoint:
				vbool( b2RevoluteJoint_IsSpringEnabled( id ) );
				vf32( b2RevoluteJoint_GetSpringHertz( id ) );
				vf32( b2RevoluteJoint_GetSpringDampingRatio( id ) );
				vf32( b2RevoluteJoint_GetTargetAngle( id ) );
				vbool( b2RevoluteJoint_IsLimitEnabled( id ) );
				vf32( b2RevoluteJoint_GetLowerLimit( id ) );
				vf32( b2RevoluteJoint_GetUpperLimit( id ) );
				vbool( b2RevoluteJoint_IsMotorEnabled( id ) );
				vf32( b2RevoluteJoint_GetTargetMotorSpeed( id ) );
				vf32( b2RevoluteJoint_GetMaxMotorTorque( id ) );
				vf32( b2RevoluteJoint_GetCurrentMotorTorque( id ) );
				vf32( b2RevoluteJoint_GetCurrentAngle( id ) );
				vf32( b2RevoluteJoint_GetCurrentSpeed( id ) );
				break;
			case b2_weldJoint:
				vf32( b2WeldJoint_GetLinearHertz( id ) );
				vf32( b2WeldJoint_GetLinearDampingRatio( id ) );
				vf32( b2WeldJoint_GetAngularHertz( id ) );
				vf32( b2WeldJoint_GetAngularDampingRatio( id ) );
				break;
			case b2_wheelJoint:
				vbool( b2WheelJoint_IsSpringEnabled( id ) );
				vf32( b2WheelJoint_GetSpringHertz( id ) );
				vf32( b2WheelJoint_GetSpringDampingRatio( id ) );
				vbool( b2WheelJoint_IsLimitEnabled( id ) );
				vf32( b2WheelJoint_GetLowerLimit( id ) );
				vf32( b2WheelJoint_GetUpperLimit( id ) );
				vbool( b2WheelJoint_IsMotorEnabled( id ) );
				vf32( b2WheelJoint_GetTargetMotorSpeed( id ) );
				vf32( b2WheelJoint_GetMaxMotorTorque( id ) );
				vf32( b2WheelJoint_GetCurrentMotorTorque( id ) );
				break;
			default:
				break;
		}
		vend();
	}
}

static void setters( void )
{
	b2DistanceJoint_SetRestLength( joints[0], 1.7f );
	b2DistanceJoint_SetLimits( joints[1], 1.3f, 0.8f );
	b2DistanceJoint_SetSpringForceRange( joints[1], -30.0f, 20.0f );
	b2DistanceJoint_SetSpringHertz( joints[1], 4.0f );
	b2DistanceJoint_SetSpringDampingRatio( joints[1], 0.5f );
	b2DistanceJoint_EnableMotor( joints[1], false );
	b2DistanceJoint_SetTargetMotorSpeed( joints[1], 0.2f );
	b2DistanceJoint_SetMaxMotorForce( joints[1], 3.0f );
	b2DistanceJoint_EnableLimit( joints[0], true );
	b2DistanceJoint_EnableSpring( joints[0], true );
	b2MotorJoint_SetLinearVelocity( joints[2], (b2Vec2){ -0.3f, 0.1f } );
	b2MotorJoint_SetAngularVelocity( joints[2], -0.4f );
	b2MotorJoint_SetMaxVelocityForce( joints[2], 25.0f );
	b2MotorJoint_SetMaxVelocityTorque( joints[2], 5.0f );
	b2MotorJoint_SetLinearHertz( joints[2], 3.0f );
	b2MotorJoint_SetLinearDampingRatio( joints[2], 0.9f );
	b2MotorJoint_SetAngularHertz( joints[2], 2.0f );
	b2MotorJoint_SetAngularDampingRatio( joints[2], 0.3f );
	b2MotorJoint_SetMaxSpringForce( joints[2], -1.0f );
	b2MotorJoint_SetMaxSpringTorque( joints[2], 7.0f );
	b2MoverJoint_SetLinearVelocity( joints[3], (b2Vec2){ -1.0f, 0.0f } );
	b2MoverJoint_SetMaxVelocityForce( joints[3], (b2Vec2){ 4.0f, 4.0f } );
	b2PogoJoint_SetRestLength( joints[4], 2.5f );
	b2PogoJoint_SetSpringHertz( joints[4], 6.0f );
	b2PogoJoint_SetSpringDampingRatio( joints[4], 0.9f );
	b2PrismaticJoint_EnableSpring( joints[5], false );
	b2PrismaticJoint_SetSpringHertz( joints[5], 1.0f );
	b2PrismaticJoint_SetSpringDampingRatio( joints[5], 0.1f );
	b2PrismaticJoint_SetTargetTranslation( joints[5], -0.1f );
	b2PrismaticJoint_EnableLimit( joints[5], false );
	b2PrismaticJoint_SetLimits( joints[5], -0.5f, 0.5f );
	b2PrismaticJoint_EnableMotor( joints[5], false );
	b2PrismaticJoint_SetTargetMotorSpeed( joints[5], 0.9f );
	b2PrismaticJoint_SetMaxMotorForce( joints[5], 2.0f );
	b2RevoluteJoint_EnableSpring( joints[6], false );
	b2RevoluteJoint_SetSpringHertz( joints[6], 2.0f );
	b2RevoluteJoint_SetSpringDampingRatio( joints[6], 0.6f );
	b2RevoluteJoint_SetTargetAngle( joints[6], -0.2f );
	b2RevoluteJoint_EnableLimit( joints[6], false );
	b2RevoluteJoint_SetLimits( joints[6], -4.0f, 3.5f );
	b2RevoluteJoint_EnableMotor( joints[6], false );
	b2RevoluteJoint_SetTargetMotorSpeed( joints[6], -0.8f );
	b2RevoluteJoint_SetMaxMotorTorque( joints[6], 1.0f );
	b2WeldJoint_SetLinearHertz( joints[7], 2.0f );
	b2WeldJoint_SetLinearDampingRatio( joints[7], 0.2f );
	b2WeldJoint_SetAngularHertz( joints[7], 1.0f );
	b2WeldJoint_SetAngularDampingRatio( joints[7], 0.3f );
	b2WheelJoint_EnableSpring( joints[9], false );
	b2WheelJoint_SetSpringHertz( joints[9], 3.0f );
	b2WheelJoint_SetSpringDampingRatio( joints[9], 0.2f );
	b2WheelJoint_EnableLimit( joints[9], false );
	b2WheelJoint_SetLimits( joints[9], -0.2f, 0.2f );
	b2WheelJoint_EnableMotor( joints[9], false );
	b2WheelJoint_SetTargetMotorSpeed( joints[9], 2.0f );
	b2WheelJoint_SetMaxMotorTorque( joints[9], 4.0f );
	b2Joint_SetConstraintTuning( joints[6], 30.0f, 1.0f );
	b2Joint_SetForceThreshold( joints[5], 3.0f );
	b2Joint_SetTorqueThreshold( joints[5], 2.0f );
	b2Transform frame = { (b2Vec2){ 0.1f, 0.2f }, b2MakeRot( 0.5f ) };
	b2Joint_SetLocalFrameA( joints[8], frame );
	b2Joint_SetLocalFrameB( joints[8], frame );
	b2Joint_SetCollideConnected( joints[10], true );
	b2Joint_SetUserData( joints[10], (void*)(uintptr_t)42 );
	vtag( "ud" );
	vi32( (int)(uintptr_t)b2Joint_GetUserData( joints[10] ) );
	vend();
}

int main( void )
{
	b2SetLogCallback( silentLog );
	build();
	emitApi();
	stepJoints( true );
	emitApi();
	setters();
	emitApi();
	stepJoints( true );
	emitApi();
	stepJoints( false );
	emitApi();

	b2Joint_SetCollideConnected( joints[10], false );
	b2DestroyJoint( joints[1] );
	b2DestroyJoint( joints[10] );
	jointCount = 0;
	for ( int i = 0; i < 11; ++i )
	{
		if ( i != 1 && i != 10 )
		{
			joints[jointCount++] = joints[i];
		}
	}
	b2Body_SetType( bodies[8], b2_staticBody );
	stepJoints( true );
	emitApi();

	b2World* world = b2GetWorldFromId( worldId );
	vtag( "end" );
	vi32( world->joints.count );
	vi32( b2GetIdCount( &world->jointIdPool ) );
	vend();
	return 0;
}
