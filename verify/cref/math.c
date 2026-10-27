// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Reference vectors for lib/math_functions.as, lib/core.as, lib/constants.as, lib/id.as,
// lib/types.as and lib/collision.as

#include "common.h"

#include "core.h"
#include "simd.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

#include <float.h>

static void silentLog( const char* message )
{
	(void)message;
}

// The invalid unit inputs trip B2_CHECK_INPUT's assert in assert builds
static int skipAssert( const char* condition, const char* fileName, int lineNumber )
{
	(void)condition;
	(void)fileName;
	(void)lineNumber;
	return 0;
}

static void pv( b2Vec2 v )
{
	vf32( v.x );
	vf32( v.y );
}

static void pr( b2Rot q )
{
	vf32( q.c );
	vf32( q.s );
}

static void pxf( b2Transform t )
{
	pv( t.p );
	pr( t.q );
}

static void pab( b2AABB a )
{
	pv( a.lowerBound );
	pv( a.upperBound );
}

static void pm( b2Mat22 m )
{
	pv( m.cx );
	pv( m.cy );
}

static void ppl( b2Plane p )
{
	pv( p.normal );
	vf32( p.offset );
}

// mode 0 special, 1 tiny, 2 large, 3 ordinary
static float rcomp( int mode )
{
	if ( mode == 0 )
	{
		return vspecialf();
	}
	if ( mode == 1 )
	{
		return vrandf( -1.0e-3f, 1.0e-3f );
	}
	if ( mode == 2 )
	{
		return vrandf( -1.0e4f, 1.0e4f );
	}
	return vrandf( -10.0f, 10.0f );
}

static float rscalar( void )
{
	int mode = (int)( vrand32() % 4 );
	return rcomp( mode );
}

static b2Vec2 rvec( void )
{
	int mode = (int)( vrand32() % 4 );
	b2Vec2 v;
	v.x = rcomp( mode );
	v.y = rcomp( mode );
	return v;
}

static b2Vec2 rvecn( void )
{
	b2Vec2 v;
	v.x = vrandf( -10.0f, 10.0f );
	v.y = vrandf( -10.0f, 10.0f );
	return v;
}

static b2Vec2 runit( void )
{
	b2Vec2 v = rvecn();
	v = b2Normalize( v );
	if ( b2LengthSquared( v ) == 0.0f )
	{
		v = (b2Vec2){ 1.0f, 0.0f };
	}
	return v;
}

// Normalized rotations, raw component pairs and special values
static b2Rot rrot( void )
{
	uint32_t mode = vrand32() % 4;
	if ( mode == 0 )
	{
		b2Rot q;
		q.c = vspecialf();
		q.s = vspecialf();
		return q;
	}
	if ( mode == 1 )
	{
		b2Rot q;
		q.c = vrandf( -2.0f, 2.0f );
		q.s = vrandf( -2.0f, 2.0f );
		return q;
	}
	float angle = vrandf( -4.0f, 4.0f );
	return b2MakeRot( angle );
}

static b2Rot rrotn( void )
{
	float angle = vrandf( -4.0f, 4.0f );
	return b2MakeRot( angle );
}

static b2Transform rxf( void )
{
	b2Transform t;
	t.p = rvec();
	t.q = rrot();
	return t;
}

static b2AABB raabb( void )
{
	uint32_t mode = vrand32() % 6;
	b2AABB a;
	if ( mode == 0 )
	{
		a.lowerBound = rvec();
		a.upperBound = rvec();
		return a;
	}
	b2Vec2 c = rvecn();
	b2Vec2 e = b2Abs( rvecn() );
	if ( mode == 1 )
	{
		e = b2Neg( e );
	}
	a.lowerBound = b2Sub( c, e );
	a.upperBound = b2Add( c, e );
	return a;
}

static b2Mat22 rmat( void )
{
	uint32_t mode = vrand32() % 5;
	b2Mat22 m;
	if ( mode == 0 )
	{
		m.cx = rvec();
		m.cy = rvec();
		return m;
	}
	if ( mode == 1 )
	{
		m.cx = rvecn();
		float s = vrandf( -3.0f, 3.0f );
		m.cy = b2MulSV( s, m.cx );
		return m;
	}
	m.cx = rvecn();
	m.cy = rvecn();
	return m;
}

static b2Plane rplane( void )
{
	b2Plane p;
	uint32_t mode = vrand32() % 4;
	if ( mode == 0 )
	{
		p.normal = rvec();
		p.offset = vspecialf();
		return p;
	}
	p.normal = runit();
	p.offset = vrandf( -10.0f, 10.0f );
	return p;
}

static void checkConstants( void )
{
	vtag( "const" );
	vf32( B2_PI );
	vf32( FLT_MIN );
	vf32( FLT_MAX );
	vf32( FLT_EPSILON );
	vf32( INFINITY );
	vf32( B2_MAX_ROTATION );
	vf32( B2_CONTACT_RECYCLE_COS_ANGLE );
	vf32( B2_AABB_MARGIN_FRACTION );
	vf32( B2_TIME_TO_SLEEP );
	vend();

	vtag( "iconst" );
	vi32( B2_MAX_WORKERS );
	vi32( B2_MAX_TASKS );
	vi32( B2_GRAPH_COLOR_COUNT );
	vi32( B2_MAX_WORLDS );
	vi32( B2_NAME_LENGTH );
	vi32( B2_MAX_RESTITUTION_ITERATIONS );
	vi32( B2_MAX_POLYGON_VERTICES );
	vi32( B2_NULL_INDEX );
	vu32( B2_HASH_INIT );
	vi32( B2_SECRET_COOKIE );
	vi32( B2_ALIGNMENT );
	vu64( B2_DEFAULT_CATEGORY_BITS );
	vu64( B2_DEFAULT_MASK_BITS );
	vi32( B2_VERSION_MAJOR );
	vi32( B2_VERSION_MINOR );
	vi32( B2_VERSION_REVISION );
	vend();

	b2Version version = b2GetVersion();
	vtag( "version" );
	vi32( version.major );
	vi32( version.minor );
	vi32( version.revision );
	vbool( b2IsDoublePrecision() );
	vbool( b2IsAVX2Available() );
	vi32( b2GetSIMDWidth() );
	b2SetSIMDWidth( 8 );
	vi32( b2GetSIMDWidth() );
	b2SetSIMDWidth( 0 );
	vi32( b2GetSIMDWidth() );
	vend();

	vtag( "statics" );
	pv( b2Vec2_zero );
	pr( b2Rot_identity );
	pxf( b2Transform_identity );
	pm( b2Mat22_zero );
	pv( b2Pos_zero );
	pxf( b2WorldTransform_identity );
	vu32( b2_emptySimplexCache.count );
	vu32( b2_emptySimplexCache.indexA[0] );
	vu32( b2_emptySimplexCache.indexB[2] );
	vend();

	vtag( "null" );
	vu32( b2StoreWorldId( b2_nullWorldId ) );
	vu64( b2StoreBodyId( b2_nullBodyId ) );
	vu64( b2StoreShapeId( b2_nullShapeId ) );
	vu64( b2StoreChainId( b2_nullChainId ) );
	vu64( b2StoreJointId( b2_nullJointId ) );
	vi32( b2_nullContactId.index1 );
	vu32( b2_nullContactId.generation );
	vend();
}

static void checkUnitConstants( float units )
{
	b2SetLengthUnitsPerMeter( units );
	vtag( "units" );
	vf32( b2GetLengthUnitsPerMeter() );
	vf32( b2_lengthUnitsPerMeter );
	vf32( B2_HUGE );
	vf32( B2_LINEAR_SLOP );
	vf32( B2_SPECULATIVE_DISTANCE );
	vf32( B2_CONTACT_RECYCLE_DISTANCE );
	vf32( B2_MAX_AABB_MARGIN );
	vend();
}

static void checkInvalidUnits( void )
{
	float bad[] = { 0.0f, -1.0f, -0.0f, INFINITY, -INFINITY, NAN };
	for ( int i = 0; i < 6; ++i )
	{
		b2SetLengthUnitsPerMeter( bad[i] );
		vtag( "badunits" );
		vf32( b2GetLengthUnitsPerMeter() );
		vend();
	}
}

static void pfilter( b2Filter f )
{
	vu64( f.categoryBits );
	vu64( f.maskBits );
	vi32( f.groupIndex );
}

static void psurface( b2SurfaceMaterial m )
{
	vf32( m.friction );
	vf32( m.restitution );
	vf32( m.rollingResistance );
	vf32( m.tangentSpeed );
	vu64( m.userMaterialId );
	vu32( m.customColor );
}

static void pjointdef( b2JointDef d )
{
	vu64( (uint64_t)(uintptr_t)d.userData );
	vu64( b2StoreBodyId( d.bodyIdA ) );
	vu64( b2StoreBodyId( d.bodyIdB ) );
	pxf( d.localFrameA );
	pxf( d.localFrameB );
	vf32( d.forceThreshold );
	vf32( d.torqueThreshold );
	vf32( d.constraintHertz );
	vf32( d.constraintDampingRatio );
	vf32( d.drawScale );
	vbool( d.collideConnected );
}

static void checkDefaults( float units )
{
	b2SetLengthUnitsPerMeter( units );

	b2WorldDef w = b2DefaultWorldDef();
	vtag( "worlddef" );
	pv( w.gravity );
	vf32( w.restitutionThreshold );
	vi32( w.restitutionIterations );
	vbool( w.enableRestitutionPropagation );
	vf32( w.hitEventThreshold );
	vf32( w.contactHertz );
	vf32( w.contactDampingRatio );
	vf32( w.contactSpeed );
	vf32( w.maximumLinearSpeed );
	vbool( w.frictionCallback == NULL );
	vbool( w.restitutionCallback == NULL );
	vbool( w.enableSleep );
	vbool( w.enableContinuous );
	vi32( w.workerCount );
	vbool( w.enqueueTask == NULL );
	vbool( w.finishTask == NULL );
	vu64( (uint64_t)(uintptr_t)w.userTaskContext );
	vu64( (uint64_t)(uintptr_t)w.userData );
	vi32( w.capacity.staticShapeCount );
	vi32( w.capacity.dynamicShapeCount );
	vi32( w.capacity.staticBodyCount );
	vi32( w.capacity.dynamicBodyCount );
	vi32( w.capacity.contactCount );
	vi32( w.capacity.arenaByteCount );
	vi32( w.internalValue );
	vend();

	b2BodyDef b = b2DefaultBodyDef();
	vtag( "bodydef" );
	vi32( b.type );
	pv( b.position );
	pr( b.rotation );
	pv( b.linearVelocity );
	vf32( b.angularVelocity );
	vf32( b.linearDamping );
	vf32( b.angularDamping );
	vf32( b.gravityScale );
	vf32( b.sleepThreshold );
	vf32( b.safetyFactor );
	vbool( b.name == NULL );
	vu64( (uint64_t)(uintptr_t)b.userData );
	vbool( b.motionLocks.linearX );
	vbool( b.motionLocks.linearY );
	vbool( b.motionLocks.angularZ );
	vbool( b.enableSleep );
	vbool( b.isAwake );
	vbool( b.isBullet );
	vbool( b.isEnabled );
	vbool( b.allowFastRotation );
	vbool( b.enableContactRecycling );
	vi32( b.internalValue );
	vend();

	vtag( "filters" );
	pfilter( b2DefaultFilter() );
	b2QueryFilter qf = b2DefaultQueryFilter();
	vu64( qf.categoryBits );
	vu64( qf.maskBits );
	psurface( b2DefaultSurfaceMaterial() );
	vend();

	b2ShapeDef s = b2DefaultShapeDef();
	vtag( "shapedef" );
	vu64( (uint64_t)(uintptr_t)s.userData );
	psurface( s.material );
	vf32( s.density );
	pfilter( s.filter );
	vbool( s.enableCustomFiltering );
	vbool( s.isSensor );
	vbool( s.enableSensorEvents );
	vbool( s.enableContactEvents );
	vbool( s.enableHitEvents );
	vbool( s.enablePreSolveEvents );
	vbool( s.invokeContactCreation );
	vbool( s.updateBodyMass );
	vi32( s.internalValue );
	vend();

	b2ChainDef c = b2DefaultChainDef();
	vtag( "chaindef" );
	vu64( (uint64_t)(uintptr_t)c.userData );
	vi32( c.pointCount );
	pv( c.ghost1 );
	pv( c.ghost2 );
	vi32( c.materialCount );
	psurface( c.materials[0] );
	pfilter( c.filter );
	vbool( c.isLoop );
	vbool( c.enableSensorEvents );
	vi32( c.internalValue );
	vend();

	b2DistanceJointDef dj = b2DefaultDistanceJointDef();
	vtag( "distancedef" );
	pjointdef( dj.base );
	vf32( dj.length );
	vbool( dj.enableSpring );
	vf32( dj.lowerSpringForce );
	vf32( dj.upperSpringForce );
	vf32( dj.hertz );
	vf32( dj.dampingRatio );
	vbool( dj.enableLimit );
	vf32( dj.minLength );
	vf32( dj.maxLength );
	vbool( dj.enableMotor );
	vf32( dj.maxMotorForce );
	vf32( dj.motorSpeed );
	vi32( dj.internalValue );
	vend();

	b2FilterJointDef fj = b2DefaultFilterJointDef();
	vtag( "filterdef" );
	pjointdef( fj.base );
	vi32( fj.internalValue );
	vend();

	b2MotorJointDef mj = b2DefaultMotorJointDef();
	vtag( "motordef" );
	pjointdef( mj.base );
	pv( mj.linearVelocity );
	vf32( mj.maxVelocityForce );
	vf32( mj.angularVelocity );
	vf32( mj.maxVelocityTorque );
	vf32( mj.linearHertz );
	vf32( mj.linearDampingRatio );
	vf32( mj.maxSpringForce );
	vf32( mj.angularHertz );
	vf32( mj.angularDampingRatio );
	vf32( mj.maxSpringTorque );
	vi32( mj.internalValue );
	vend();

	b2MoverJointDef vj = b2DefaultMoverJointDef();
	vtag( "moverdef" );
	pjointdef( vj.base );
	pv( vj.linearVelocity );
	pv( vj.maxVelocityForce );
	vi32( vj.internalValue );
	vend();

	b2PogoJointDef pj = b2DefaultPogoJointDef();
	vtag( "pogodef" );
	pjointdef( pj.base );
	pv( pj.normal );
	vf32( pj.hertz );
	vf32( pj.dampingRatio );
	vf32( pj.restLength );
	vf32( pj.maxTensionForce );
	vf32( pj.maxCompressionForce );
	vf32( pj.impulse );
	vf32( pj.velocity );
	vi32( pj.internalValue );
	vend();

	b2PrismaticJointDef xj = b2DefaultPrismaticJointDef();
	vtag( "prismaticdef" );
	pjointdef( xj.base );
	vbool( xj.enableSpring );
	vf32( xj.hertz );
	vf32( xj.dampingRatio );
	vf32( xj.targetTranslation );
	vbool( xj.enableLimit );
	vf32( xj.lowerTranslation );
	vf32( xj.upperTranslation );
	vbool( xj.enableMotor );
	vf32( xj.maxMotorForce );
	vf32( xj.motorSpeed );
	vi32( xj.internalValue );
	vend();

	b2RevoluteJointDef rj = b2DefaultRevoluteJointDef();
	vtag( "revolutedef" );
	pjointdef( rj.base );
	vf32( rj.targetAngle );
	vbool( rj.enableSpring );
	vf32( rj.hertz );
	vf32( rj.dampingRatio );
	vbool( rj.enableLimit );
	vf32( rj.lowerAngle );
	vf32( rj.upperAngle );
	vbool( rj.enableMotor );
	vf32( rj.maxMotorTorque );
	vf32( rj.motorSpeed );
	vi32( rj.internalValue );
	vend();

	b2WeldJointDef wj = b2DefaultWeldJointDef();
	vtag( "welddef" );
	pjointdef( wj.base );
	vf32( wj.linearHertz );
	vf32( wj.angularHertz );
	vf32( wj.linearDampingRatio );
	vf32( wj.angularDampingRatio );
	vi32( wj.internalValue );
	vend();

	b2WheelJointDef hj = b2DefaultWheelJointDef();
	vtag( "wheeldef" );
	pjointdef( hj.base );
	vbool( hj.enableSpring );
	vf32( hj.hertz );
	vf32( hj.dampingRatio );
	vbool( hj.enableLimit );
	vf32( hj.lowerTranslation );
	vf32( hj.upperTranslation );
	vbool( hj.enableMotor );
	vf32( hj.maxMotorTorque );
	vf32( hj.motorSpeed );
	vi32( hj.internalValue );
	vend();

	b2ExplosionDef ex = b2DefaultExplosionDef();
	vtag( "explosiondef" );
	vu64( ex.maskBits );
	pv( ex.position );
	vf32( ex.radius );
	vf32( ex.falloff );
	vf32( ex.impulsePerLength );
	vend();
}

static void checkScalars( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		int ia = (int)vrand32();
		int ib = vrandint( -1000, 1000 );
		int ic = vrandint( -1000, 1000 );
		int num = vrandint( 0, 100000 );
		int den = vrandint( 1, 1000 );
		vtag( "int" );
		vi32( b2MinInt( ia, ib ) );
		vi32( b2MaxInt( ia, ib ) );
		vi32( b2AbsInt( ia ) );
		vi32( b2AbsInt( ib ) );
		vi32( b2ClampInt( ia, ib, ic ) );
		vi32( b2ClampInt( ib, -100, 100 ) );
		vi32( b2CeilingInt( num, den ) );
		vend();

		float a = rscalar();
		float b = rscalar();
		float c = rscalar();
		vtag( "f" );
		vf32( a );
		vf32( b );
		vf32( c );
		vf32( b2MinFloat( a, b ) );
		vf32( b2MaxFloat( a, b ) );
		vf32( b2AbsFloat( a ) );
		vf32( b2ClampFloat( a, b, c ) );
		vbool( b2IsValidFloat( a ) );
		vend();

		vtag( "trig" );
		vf32( b2Atan2( a, b ) );
		b2CosSin cs = b2ComputeCosSin( a );
		vf32( cs.cosine );
		vf32( cs.sine );
		vf32( b2UnwindAngle( a ) );
		b2Rot q = b2MakeRot( b );
		pr( q );
		vend();

		float angle = vrandf( -20.0f, 20.0f );
		float y = vrandf( -10.0f, 10.0f );
		float x = vrandf( -10.0f, 10.0f );
		vtag( "trign" );
		vf32( b2Atan2( y, x ) );
		vf32( b2Atan2( y, 0.0f ) );
		vf32( b2Atan2( 0.0f, x ) );
		b2CosSin csn = b2ComputeCosSin( angle );
		vf32( csn.cosine );
		vf32( csn.sine );
		vf32( b2UnwindAngle( angle ) );
		vend();

		float hertz = vrandf( 0.0f, 60.0f );
		float zeta = vrandf( 0.0f, 10.0f );
		float position = rscalar();
		float velocity = rscalar();
		float h = vrandf( 0.0f, 0.1f );
		vtag( "spring" );
		vf32( b2SpringDamper( hertz, zeta, position, velocity, h ) );
		vend();

		vtag( "cast" );
		vi32( (int)a );
		vu32( (uint32_t)a );
		vi32( (int)( a * 1.0e8f ) );
		vend();
	}

	float edges[] = { 0.0f, -0.0f, 1.0f, -1.0f, B2_PI, -B2_PI, 0.5f * B2_PI, -0.5f * B2_PI, 2.0f * B2_PI, 1.0e6f, -1.0e6f, 1.0e7f, INFINITY, -INFINITY, NAN, FLT_MIN, FLT_MAX };
	for ( int i = 0; i < (int)( sizeof( edges ) / sizeof( edges[0] ) ); ++i )
	{
		float e = edges[i];
		vtag( "edge" );
		vf32( e );
		vf32( b2UnwindAngle( e ) );
		b2CosSin cs = b2ComputeCosSin( e );
		vf32( cs.cosine );
		vf32( cs.sine );
		vf32( b2Atan2( e, 1.0f ) );
		vf32( b2Atan2( 1.0f, e ) );
		vf32( b2Atan2( e, e ) );
		vf32( b2Atan2( -e, e ) );
		vbool( b2IsValidFloat( e ) );
		vend();
	}
}

static void checkVectors( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		b2Vec2 a = rvec();
		b2Vec2 b = rvec();
		b2Vec2 c = rvec();
		float s = rscalar();
		float t = vrandf( -0.5f, 1.5f );

		vtag( "v" );
		pv( a );
		pv( b );
		vf32( s );
		vf32( b2Dot( a, b ) );
		vf32( b2Cross( a, b ) );
		pv( b2CrossVS( a, s ) );
		pv( b2CrossSV( s, a ) );
		pv( b2LeftPerp( a ) );
		pv( b2RightPerp( a ) );
		pv( b2Add( a, b ) );
		pv( b2Sub( a, b ) );
		pv( b2Neg( a ) );
		pv( b2Lerp( a, b, t ) );
		pv( b2Mul( a, b ) );
		pv( b2MulSV( s, a ) );
		pv( b2MulAdd( a, s, b ) );
		pv( b2MulSub( a, s, b ) );
		vend();

		vtag( "v2" );
		pv( b2Abs( a ) );
		pv( b2Min( a, b ) );
		pv( b2Max( a, b ) );
		pv( b2Clamp( a, b, c ) );
		vf32( b2Length( a ) );
		vf32( b2Distance( a, b ) );
		pv( b2Normalize( a ) );
		vbool( b2IsNormalized( a ) );
		float length = -7.0f;
		b2Vec2 n = b2GetLengthAndNormalize( &length, a );
		pv( n );
		vf32( length );
		vf32( b2LengthSquared( a ) );
		vf32( b2DistanceSquared( a, b ) );
		vbool( b2IsValidVec2( a ) );
		vbool( b2IsNormalized( n ) );
		vend();

		vtag( "pos" );
		pv( b2ToPos( a ) );
		pv( b2ToVec2( a ) );
		pv( b2SubPos( a, b ) );
		pv( b2OffsetPos( a, b ) );
		pv( b2LerpPos( a, b, t ) );
		vbool( b2IsValidPos( a ) );
		vf32( b2RoundDownFloat( (double)s * 1.000001 ) );
		vf32( b2RoundUpFloat( (double)s * 1.000001 ) );
		vend();
	}
}

static void checkRots( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		b2Rot q = rrot();
		b2Rot r = rrot();
		b2Vec2 v = rvec();
		float t = vrandf( -0.5f, 1.5f );
		float d = rscalar();
		float invH = vrandf( 0.0f, 240.0f );

		vtag( "q" );
		pr( q );
		pr( r );
		pr( b2NormalizeRot( q ) );
		pr( b2IntegrateRotation( q, d ) );
		vbool( b2IsNormalizedRot( q ) );
		pr( b2InvertRot( q ) );
		pr( b2NLerp( q, r, t ) );
		vf32( b2ComputeAngularVelocity( q, r, invH ) );
		vf32( b2Rot_GetAngle( q ) );
		pv( b2Rot_GetXAxis( q ) );
		pv( b2Rot_GetYAxis( q ) );
		pr( b2MulRot( q, r ) );
		pr( b2InvMulRot( q, r ) );
		vf32( b2RelativeAngle( q, r ) );
		pv( b2RotateVector( q, v ) );
		pv( b2InvRotateVector( q, v ) );
		vbool( b2IsValidRot( q ) );
		vend();

		b2Vec2 u1 = runit();
		b2Vec2 u2 = runit();
		vtag( "qu" );
		pr( b2MakeRotFromUnitVector( u1 ) );
		pr( b2ComputeRotationBetweenUnitVectors( u1, u2 ) );
		b2Rot qn = rrotn();
		float dn = vrandf( -0.5f, 0.5f );
		pr( b2IntegrateRotation( qn, dn ) );
		vf32( b2RelativeAngle( qn, b2IntegrateRotation( qn, dn ) ) );
		vend();
	}
}

static void checkTransforms( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		b2Transform A = rxf();
		b2Transform B = rxf();
		b2Vec2 p = rvec();
		b2Vec2 base = rvec();

		vtag( "xf" );
		pv( b2TransformPoint( A, p ) );
		pv( b2InvTransformPoint( A, p ) );
		pxf( b2MulTransforms( A, B ) );
		pxf( b2InvMulTransforms( A, B ) );
		vbool( b2IsValidTransform( A ) );
		pv( b2TransformWorldPoint( A, p ) );
		pv( b2InvTransformWorldPoint( A, p ) );
		pxf( b2InvMulWorldTransforms( A, B ) );
		pxf( b2MulWorldTransforms( A, B ) );
		pxf( b2ToRelativeTransform( A, base ) );
		pxf( b2MakeWorldTransform( A ) );
		vbool( b2IsValidWorldTransform( A ) );
		vend();

		b2Mat22 m = rmat();
		b2Vec2 v = rvec();
		vtag( "m" );
		pv( b2MulMV( m, v ) );
		pm( b2GetInverse22( m ) );
		pv( b2Solve22( m, v ) );
		vend();
	}
}

static void checkAABBs( void )
{
	for ( int i = 0; i < 6000; ++i )
	{
		b2AABB a = raabb();
		b2AABB b = raabb();

		vtag( "ab" );
		pab( a );
		pab( b );
		vbool( b2AABB_Contains( a, b ) );
		pv( b2AABB_Center( a ) );
		pv( b2AABB_Extents( a ) );
		pab( b2AABB_Union( a, b ) );
		vbool( b2AABB_Overlaps( a, b ) );
		vend();

		b2ShapeProxy proxy = { 0 };
		proxy.count = vrandint( 1, B2_MAX_POLYGON_VERTICES + 1 );
		for ( int k = 0; k < proxy.count; ++k )
		{
			proxy.points[k] = rvec();
		}
		proxy.radius = vrandf( 0.0f, 2.0f );
		vtag( "mk" );
		pab( b2MakeAABB( proxy.points, proxy.count, proxy.radius ) );
		vend();

		b2Plane plane = rplane();
		b2Vec2 point = rvec();
		vtag( "pl" );
		ppl( plane );
		vbool( b2IsValidPlane( plane ) );
		vf32( b2PlaneSeparation( plane, point ) );
		vend();
	}
}

static void checkHash( void )
{
	uint8_t buffer[512];
	for ( int i = 0; i < 400; ++i )
	{
		int len;
		if ( i < 260 )
		{
			len = i;
		}
		else
		{
			len = vrandint( 0, 512 );
		}

		for ( int j = 0; j < len; ++j )
		{
			buffer[j] = (uint8_t)( vrand32() & 0xff );
		}

		uint32_t seed = i % 2 == 0 ? B2_HASH_INIT : vrand32();
		vtag( "h" );
		vi32( len );
		vu32( b2Hash( seed, buffer, len ) );
		vend();
	}
}

static void checkIds( void )
{
	for ( int i = 0; i < 2000; ++i )
	{
		uint32_t x32 = vrand32();
		uint64_t x64 = vrand64();
		uint64_t y64 = vrand64();
		uint64_t z64 = vrand64();
		uint64_t c64 = vrand64();
		uint32_t values[3];
		values[0] = vrand32();
		values[1] = vrand32();
		values[2] = vrand32();

		vtag( "id" );
		b2WorldId w = b2LoadWorldId( x32 );
		vu32( w.index1 );
		vu32( w.generation );
		vu32( b2StoreWorldId( w ) );
		b2BodyId bo = b2LoadBodyId( x64 );
		vi32( bo.index1 );
		vu32( bo.world0 );
		vu32( bo.generation );
		vu64( b2StoreBodyId( bo ) );
		b2ShapeId sh = b2LoadShapeId( y64 );
		vi32( sh.index1 );
		vu32( sh.world0 );
		vu32( sh.generation );
		vu64( b2StoreShapeId( sh ) );
		b2ChainId ch = b2LoadChainId( c64 );
		vi32( ch.index1 );
		vu32( ch.world0 );
		vu32( ch.generation );
		vu64( b2StoreChainId( ch ) );
		b2JointId jo = b2LoadJointId( z64 );
		vi32( jo.index1 );
		vu32( jo.world0 );
		vu32( jo.generation );
		vu64( b2StoreJointId( jo ) );
		b2ContactId co = b2LoadContactId( values );
		vi32( co.index1 );
		vu32( co.world0 );
		vi32( co.padding );
		vu32( co.generation );
		uint32_t out[3];
		b2StoreContactId( co, out );
		vu32( out[0] );
		vu32( out[1] );
		vu32( out[2] );
		b2BodyId same = b2LoadBodyId( x64 );
		b2BodyId other = b2LoadBodyId( x64 ^ 1 );
		vbool( B2_ID_EQUALS( bo, same ) );
		vbool( B2_ID_EQUALS( bo, other ) );
		vbool( B2_IS_NULL( bo ) );
		vbool( B2_IS_NON_NULL( bo ) );
		vbool( B2_IS_NULL( b2_nullShapeId ) );
		vend();
	}
}

int main( void )
{
	b2SetLogCallback( silentLog );
	b2SetAssertCallback( skipAssert );
	vseed( 0x5eed );
	checkConstants();
	checkUnitConstants( 1.0f );
	checkDefaults( 1.0f );
	checkUnitConstants( 0.37f );
	checkDefaults( 0.37f );
	checkUnitConstants( 100.0f );
	checkInvalidUnits();
	b2SetLengthUnitsPerMeter( 1.0f );
	checkScalars();
	checkVectors();
	checkRots();
	checkTransforms();
	checkAABBs();
	checkHash();
	checkIds();
	return 0;
}
