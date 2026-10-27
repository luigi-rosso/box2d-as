#include "body.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"
#include "box2d/box2d.h"

#include "common.h"

static void silentLog( const char* m )
{
	(void)m;
}

static b2WorldId worldId;
static b2BodyId ids[16];
static int idCount;

static void vvec( b2Vec2 v )
{
	vf32( v.x );
	vf32( v.y );
}

static void vaabb( b2AABB a )
{
	vvec( a.lowerBound );
	vvec( a.upperBound );
}

static void emitShape( b2ShapeId id )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Shape* s = world->shapes.data + ( id.index1 - 1 );
	vtag( "s" );
	vi32( id.index1 );
	vi32( s->type );
	vaabb( s->aabb );
	vaabb( world->fatAABBs.data[id.index1 - 1] );
	vf32( s->aabbMargin );
	vi32( s->proxyKey );
	vvec( s->localCentroid );
	vi32( s->sensorIndex );
	vi32( s->prevShapeId );
	vi32( s->nextShapeId );
	vf32( s->density );
	vf32( s->material.friction );
	vu32( (uint32_t)s->filter.categoryBits );
	vend();
}

static void emitBody( b2BodyId id )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Body* body = world->bodies.data + ( id.index1 - 1 );
	b2BodySim* sim = b2GetBodySim( world, body );
	vtag( "b" );
	vi32( id.index1 );
	vi32( body->setIndex );
	vi32( body->localIndex );
	vi32( body->islandId >= 0 ? 1 : 0 );
	vu32( body->flags );
	vu32( sim->flags );
	vf32( body->mass );
	vf32( body->inertia );
	vf32( sim->invMass );
	vf32( sim->invInertia );
	vvec( sim->localCenter );
	vvec( sim->center );
	vvec( sim->transform.p );
	vf32( sim->transform.q.c );
	vf32( sim->transform.q.s );
	vf32( sim->minExtent );
	vf32( sim->maxExtent );
	vvec( sim->force );
	vf32( sim->torque );
	vvec( b2Body_GetLinearVelocity( id ) );
	vf32( b2Body_GetAngularVelocity( id ) );
	vi32( body->shapeCount );
	vaabb( b2Body_ComputeBounds( id ) );
	vend();

	b2ShapeId shapes[32];
	int n = b2Body_GetShapes( id, shapes, 32 );
	for ( int i = 0; i < n; ++i )
	{
		emitShape( shapes[i] );
	}
}

static void emitAll( const char* tag )
{
	vtag( tag );
	vend();
	for ( int i = 0; i < idCount; ++i )
	{
		if ( b2Body_IsValid( ids[i] ) )
		{
			emitBody( ids[i] );
		}
	}
}

static void queries( void )
{
	for ( int i = 0; i < idCount; ++i )
	{
		if ( b2Body_IsValid( ids[i] ) == false )
		{
			continue;
		}

		b2ShapeId shapes[32];
		int n = b2Body_GetShapes( ids[i], shapes, 32 );
		for ( int j = 0; j < n; ++j )
		{
			b2ShapeId s = shapes[j];
			vtag( "q" );
			vbool( b2Shape_TestPoint( s, (b2Vec2){ 1.0f, 2.0f } ) );
			vbool( b2Shape_TestPoint( s, (b2Vec2){ 0.0f, 0.0f } ) );
			b2CastOutput o = b2Shape_CastRay( s, (b2Vec2){ -3.0f, 2.1f }, (b2Vec2){ 8.0f, -0.5f } );
			vbool( o.hit );
			vvec( o.point );
			vvec( o.normal );
			vf32( o.fraction );
			o = b2Shape_CastRay( s, (b2Vec2){ 1.0f, 6.0f }, (b2Vec2){ 0.2f, -8.0f } );
			vbool( o.hit );
			vvec( o.point );
			vvec( o.normal );
			vf32( o.fraction );
			vvec( b2Shape_GetClosestPoint( s, (b2Vec2){ 4.0f, 4.0f } ) );
			b2MassData md = b2Shape_ComputeMassData( s );
			vf32( md.mass );
			vvec( md.center );
			vf32( md.rotationalInertia );
			vaabb( b2Shape_GetBounds( s ) );
			vend();
		}
	}
}

int main( void )
{
	b2SetLogCallback( silentLog );

	b2WorldDef wd = b2DefaultWorldDef();
	worldId = b2CreateWorld( &wd );

	// ground with box, segment
	b2BodyDef bd = b2DefaultBodyDef();
	bd.name = "ground";
	b2BodyId ground = b2CreateBody( worldId, &bd );
	ids[idCount++] = ground;
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 5.0f, 0.5f );
	b2CreatePolygonShape( ground, &sd, &box );
	b2Segment seg = { { -6.0f, 1.0f }, { 6.0f, 1.5f } };
	b2CreateSegmentShape( ground, &sd, &seg );

	// chain loop with per segment materials
	bd = b2DefaultBodyDef();
	bd.position = (b2Vec2){ 0.0f, -4.0f };
	b2BodyId chainBody = b2CreateBody( worldId, &bd );
	ids[idCount++] = chainBody;
	b2Vec2 loopPoints[5] = { { -3.0f, 0.0f }, { 3.0f, 0.0f }, { 3.0f, 2.0f }, { 0.0f, 3.0f }, { -3.0f, 2.0f } };
	b2SurfaceMaterial mats[5];
	for ( int i = 0; i < 5; ++i )
	{
		mats[i] = b2DefaultSurfaceMaterial();
		mats[i].friction = 0.1f * (float)( i + 1 );
	}
	b2ChainDef cd = b2DefaultChainDef();
	cd.points = loopPoints;
	cd.pointCount = 5;
	cd.materials = mats;
	cd.materialCount = 5;
	cd.isLoop = true;
	b2ChainId loop = b2CreateChain( chainBody, &cd );

	b2Vec2 openPoints[4] = { { -8.0f, -6.0f }, { -4.0f, -6.5f }, { 0.0f, -6.0f }, { 4.0f, -7.0f } };
	cd = b2DefaultChainDef();
	cd.points = openPoints;
	cd.pointCount = 4;
	cd.ghost1 = (b2Vec2){ -10.0f, -5.0f };
	cd.ghost2 = (b2Vec2){ 6.0f, -6.0f };
	b2ChainId open = b2CreateChain( chainBody, &cd );

	// dynamic body with several shapes and a sensor
	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = (b2Vec2){ 1.0f, 2.0f };
	bd.rotation = b2MakeRot( 0.3f );
	bd.angularVelocity = 0.7f;
	bd.linearVelocity = (b2Vec2){ 0.5f, -0.25f };
	bd.name = "averyveryverylongname";
	b2BodyId a = b2CreateBody( worldId, &bd );
	ids[idCount++] = a;
	b2Circle circle = { { 0.1f, 0.2f }, 0.5f };
	b2ShapeId circleId = b2CreateCircleShape( a, &sd, &circle );
	b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.1f }, 0.25f };
	sd.density = 2.5f;
	b2ShapeId capsuleId = b2CreateCapsuleShape( a, &sd, &capsule );
	b2Polygon rbox = b2MakeOffsetRoundedBox( 0.3f, 0.4f, (b2Vec2){ 0.2f, -0.1f }, b2MakeRot( 0.5f ), 0.05f );
	sd.density = 0.7f;
	b2ShapeId polyId = b2CreatePolygonShape( a, &sd, &rbox );
	sd = b2DefaultShapeDef();
	sd.isSensor = true;
	sd.enableSensorEvents = true;
	b2Circle sensorCircle = { { 0.0f, 0.0f }, 1.5f };
	b2ShapeId sensorId = b2CreateCircleShape( a, &sd, &sensorCircle );
	sd = b2DefaultShapeDef();

	// kinematic body
	bd = b2DefaultBodyDef();
	bd.type = b2_kinematicBody;
	bd.position = (b2Vec2){ -2.0f, 3.0f };
	bd.angularVelocity = -0.4f;
	b2BodyId k = b2CreateBody( worldId, &bd );
	ids[idCount++] = k;
	b2Polygon kbox = b2MakeBox( 1.0f, 0.75f );
	b2CreatePolygonShape( k, &sd, &kbox );

	// sleeping dynamic body
	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = (b2Vec2){ 3.0f, 5.0f };
	bd.isAwake = false;
	b2BodyId c = b2CreateBody( worldId, &bd );
	ids[idCount++] = c;
	b2Circle cc = { { 0.0f, 0.0f }, 0.3f };
	b2CreateCircleShape( c, &sd, &cc );

	// disabled body, deferred mass
	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = (b2Vec2){ -4.0f, 5.0f };
	bd.isEnabled = false;
	bd.motionLocks.angularZ = true;
	b2BodyId d = b2CreateBody( worldId, &bd );
	ids[idCount++] = d;
	sd.updateBodyMass = false;
	b2Polygon dbox = b2MakeSquare( 0.4f );
	b2CreatePolygonShape( d, &sd, &dbox );
	sd = b2DefaultShapeDef();

	emitAll( "create" );
	queries();

	vtag( "name" );
	vtag( b2Body_GetName( a ) );
	vtag( b2Body_GetName( ground ) );
	vi32( b2Chain_GetSegmentCount( loop ) );
	vf32( b2Chain_GetSurfaceMaterial( loop, 3 ).friction );
	vend();

	b2Body_SetTransform( a, (b2Vec2){ 1.5f, 2.5f }, b2MakeRot( -0.2f ) );
	b2Body_ApplyLinearImpulse( a, (b2Vec2){ 0.3f, 0.6f }, (b2Vec2){ 1.7f, 2.2f }, true );
	b2Body_ApplyAngularImpulse( a, 0.25f, true );
	b2Body_ApplyForce( a, (b2Vec2){ 3.0f, -1.0f }, (b2Vec2){ 2.0f, 2.0f }, true );
	b2Body_ApplyTorque( a, 1.5f, true );
	b2Shape_ApplyWind( capsuleId, (b2Vec2){ 4.0f, 1.0f }, 1.0f, 0.5f, true );
	b2Shape_ApplyWind( polyId, (b2Vec2){ -2.0f, 3.0f }, 1.0f, 0.25f, true );
	b2Shape_ApplyWind( circleId, (b2Vec2){ 1.0f, 0.0f }, 2.0f, 0.0f, true );
	emitAll( "moved" );

	vtag( "pv" );
	vvec( b2Body_GetLocalPointVelocity( a, (b2Vec2){ 0.5f, 0.5f } ) );
	vvec( b2Body_GetWorldPointVelocity( a, (b2Vec2){ 2.0f, 3.0f } ) );
	vvec( b2Body_GetWorldPoint( a, (b2Vec2){ 0.5f, 0.5f } ) );
	vvec( b2Body_GetLocalPoint( a, (b2Vec2){ 0.5f, 0.5f } ) );
	vf32( b2Body_GetMaxExtentFromOrigin( a ) );
	vend();

	b2MassData md = { 3.0f, { 0.1f, -0.2f }, 0.9f };
	b2Body_SetMassData( a, md );
	emitAll( "massdata" );
	b2Body_UpdateMassFromShapes( a );
	b2MotionLocks locks = { false, true, true };
	b2Body_SetMotionLocks( a, locks );
	emitAll( "locks" );
	locks = (b2MotionLocks){ false, false, false };
	b2Body_SetMotionLocks( a, locks );

	b2Body_SetType( a, b2_staticBody );
	emitAll( "static" );
	b2Body_SetType( a, b2_kinematicBody );
	emitAll( "kinematic" );
	b2Body_SetType( a, b2_dynamicBody );
	emitAll( "dynamic" );

	b2Body_Disable( a );
	emitAll( "disabled" );
	b2Body_Enable( a );
	b2Body_Enable( d );
	b2Body_UpdateMassFromShapes( d );
	emitAll( "enabled" );

	b2Body_SetTargetTransform( c, (b2Transform){ { 3.5f, 4.5f }, b2MakeRot( 0.1f ) }, 1.0f / 60.0f, true );
	b2Body_SetLinearVelocity( k, (b2Vec2){ 1.0f, 0.0f } );
	b2Body_SetBullet( a, true );
	b2Body_EnableSleeping( c, false );
	emitAll( "target" );

	b2Shape_SetDensity( polyId, 3.0f, true );
	b2Polygon np = b2MakeBox( 0.2f, 0.6f );
	b2Shape_SetPolygon( circleId, &np );
	b2Capsule nc = { { 0.0f, -0.3f }, { 0.0f, 0.3f }, 0.1f };
	b2Shape_SetCapsule( polyId, &nc );
	b2Segment ns = { { -1.0f, 0.0f }, { 1.0f, 0.2f } };
	b2Shape_SetSegment( capsuleId, &ns );
	b2Filter f = b2DefaultFilter();
	f.categoryBits = 4;
	b2Shape_SetFilter( polyId, f );
	b2Shape_SetFriction( polyId, 0.9f );
	b2Body_UpdateMassFromShapes( a );
	b2SurfaceMaterial m = b2DefaultSurfaceMaterial();
	m.friction = 0.33f;
	b2Chain_SetSurfaceMaterial( loop, &m, 1 );
	emitAll( "setters" );
	queries();

	b2DestroyShape( sensorId, true );
	b2DestroyChain( open );
	emitAll( "destroyed" );
	b2DestroyBody( a );
	b2DestroyBody( k );

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = (b2Vec2){ 0.0f, 8.0f };
	b2BodyId e = b2CreateBody( worldId, &bd );
	ids[idCount++] = e;
	b2Polygon ebox = b2MakeRoundedBox( 0.5f, 0.5f, 0.1f );
	b2CreatePolygonShape( e, &sd, &ebox );
	vtag( "reuse" );
	vi32( e.index1 );
	vu32( e.generation );
	vend();
	emitAll( "final" );
	queries();

	b2Body_SetAwake( e, false );
	emitAll( "sleep" );
	b2Body_SetAwake( e, true );
	emitAll( "wake" );

	b2DestroyWorld( worldId );
	return 0;
}
