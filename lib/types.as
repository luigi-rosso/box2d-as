// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box2d/types.h and src/types.c, plus the joint and explosion defaults
// of src/joint.c

import { B2_SECRET_COOKIE, FLT_MAX, INFINITY, UINT64_MAX, b2GetLengthUnitsPerMeter } from "./core";
import { B2_HUGE } from "./constants";
import { b2AABB, b2Pos, b2Rot, b2Rot_identity, b2Transform, b2Vec2, b2WorldTransform } from "./math_functions";
import { b2Manifold, b2PlaneResult } from "./collision";
import { b2BodyId, b2ContactId, b2JointId, b2ShapeId } from "./id";

/// Default filter category bits.
export const B2_DEFAULT_CATEGORY_BITS: u64 = 1;

/// Default filter mask bits.
export const B2_DEFAULT_MASK_BITS: u64 = UINT64_MAX;

// Callbacks drop C's void* context; the closure carries it.

/// Task interface This is the prototype for a Box2D task. Your task system is expected to run this
/// callback on a worker thread, exactly once per enqueue, passing back the same taskContext pointer
/// supplied to b2EnqueueTaskCallbackFcn.
export type b2TaskCallbackFcn = (taskContext: u64) => void;

/// These functions can be provided to Box2D to invoke a task system. Returns a pointer to the
/// user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
/// serially within the callback and there is no need to call b2FinishTaskCallbackFcn.
export type b2EnqueueTaskCallbackFcn = (task: b2TaskCallbackFcn, taskContext: u64, userContext: u64) => u64;

/// Finishes a user task object that wraps a Box2D task.
export type b2FinishTaskCallbackFcn = (userTask: u64, userContext: u64) => void;

/// Optional friction mixing callback. This intentionally provides no context objects because this
/// is called from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
export type b2FrictionCallbackFcn = (frictionA: f32, userMaterialIdA: u64, frictionB: f32, userMaterialIdB: u64) => f32;

/// Optional restitution mixing callback. This intentionally provides no context objects because
/// this is called from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
export type b2RestitutionCallbackFcn = (
	restitutionA: f32,
	userMaterialIdA: u64,
	restitutionB: f32,
	userMaterialIdB: u64,
) => f32;

/// Result from b2World_CastRayClosest If there is initial overlap the fraction and normal will be
/// zero while the point is an arbitrary point in the overlap region.
export struct b2RayResult {
	/// The shape hit.
	shapeId: b2ShapeId = b2ShapeId();

	/// The point of contact, on the surface of the hit shape.
	point: b2Pos = b2Vec2();

	/// The unit normal vector at the point of contact.
	normal: b2Vec2 = b2Vec2();

	/// The fraction of the input ray.
	fraction: f32 = 0;

	/// The number of BVH nodes visited. Diagnostic.
	nodeVisits: i32 = 0;

	/// The number of BVH leaves visited. Diagnostic.
	leafVisits: i32 = 0;

	/// Did the ray hit? If false, all other data is invalid.
	hit: bool = false;
}

/// Optional world capacities that can be used to avoid run-time allocations.
/// @see b2World_GetMaxCapacity
export struct b2Capacity {
	/// Number of expected static shapes.
	staticShapeCount: i32 = 0;

	/// Number of expected dynamic and kinematic shapes.
	dynamicShapeCount: i32 = 0;

	/// Number of expected static bodies.
	staticBodyCount: i32 = 0;

	/// Number of expected dynamic and kinematic bodies.
	dynamicBodyCount: i32 = 0;

	/// Number of expected contacts.
	contactCount: i32 = 0;

	/// Reserve arena allocator space, used for temporary allocations.
	arenaByteCount: i32 = 0;
}

/// World definition used to create a simulation world. Must be initialized using
/// b2DefaultWorldDef().
export class b2WorldDef {
	/// Gravity vector. Box2D has no up-vector defined.
	gravity: b2Vec2 = b2Vec2();

	/// Restitution speed threshold, usually in m/s. Collisions above this speed have restitution
	/// applied (will bounce).
	restitutionThreshold: f32 = 0;

	/// Number of iterations of the restitution solver. More iterations can lead to less box
	/// spinning.
	/// @see B2_MAX_RESTITUTION_ITERATIONS
	restitutionIterations: i32 = 0;

	/// Enable full contact propagation in the restitution solver. Expensive. (Experimental)
	enableRestitutionPropagation: bool = false;

	/// Threshold speed for hit events. Usually meters per second.
	hitEventThreshold: f32 = 0;

	/// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap
	/// recovery, but can introduce jitter.
	contactHertz: f32 = 0;

	/// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this
	/// with the trade-off that overlap resolution becomes more energetic.
	contactDampingRatio: f32 = 0;

	/// This parameter controls how fast overlap is resolved and usually has units of meters per
	/// second. This only puts a cap on the resolution speed. The resolution speed is increased by
	/// increasing the hertz and/or decreasing the damping ratio.
	contactSpeed: f32 = 0;

	/// Maximum linear speed. Usually meters per second.
	maximumLinearSpeed: f32 = 0;

	/// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	frictionCallback: b2FrictionCallbackFcn? = null;

	/// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	restitutionCallback: b2RestitutionCallbackFcn? = null;

	/// Can bodies go to sleep to improve performance
	enableSleep: bool = false;

	/// Enable continuous collision
	enableContinuous: bool = false;

	/// Number of workers for multithreading. The port runs single threaded.
	workerCount: i32 = 0;

	/// Function to spawn tasks
	enqueueTask: b2EnqueueTaskCallbackFcn? = null;

	/// Function to finish a task
	finishTask: b2FinishTaskCallbackFcn? = null;

	/// User context that is provided to enqueueTask and finishTask
	userTaskContext: u64 = 0;

	/// User data
	userData: u64 = 0;

	/// Optional initial capacities
	capacity: b2Capacity = b2Capacity();

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// The body simulation type. Each body is one of these three types. The type determines how the
/// body behaves in the simulation.
export type b2BodyType = i32;

/// zero mass, zero velocity, may be manually moved
export const b2_staticBody: b2BodyType = 0;

/// zero mass, velocity set by user, moved by solver
export const b2_kinematicBody: b2BodyType = 1;

/// positive mass, velocity determined by forces, moved by solver
export const b2_dynamicBody: b2BodyType = 2;

/// number of body types
export const b2_bodyTypeCount: b2BodyType = 3;

/// Motion locks to restrict the body movement
export struct b2MotionLocks {
	/// Prevent translation along the x-axis
	linearX: bool = false;

	/// Prevent translation along the y-axis
	linearY: bool = false;

	/// Prevent rotation around the z-axis
	angularZ: bool = false;
}

/// A body definition holds all the data needed to construct a rigid body. You can safely re-use
/// body definitions. Shapes are added to a body after construction. Body definitions are temporary
/// objects used to bundle creation parameters. Must be initialized using b2DefaultBodyDef().
export class b2BodyDef {
	/// The body type: static, kinematic, or dynamic.
	type: b2BodyType = 0;

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body
	/// creation, especially if the body is moved after shapes have been added.
	position: b2Pos = b2Vec2();

	/// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
	rotation: b2Rot = b2Rot();

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	linearVelocity: b2Vec2 = b2Vec2();

	/// The initial angular velocity of the body. Radians per second.
	angularVelocity: f32 = 0;

	/// Linear damping is used to reduce the linear velocity.
	linearDamping: f32 = 0;

	/// Angular damping is used to reduce the angular velocity.
	angularDamping: f32 = 0;

	/// Scale the gravity applied to this body. Non-dimensional.
	gravityScale: f32 = 0;

	/// Sleep speed threshold, default is 0.05 meters per second
	sleepThreshold: f32 = 0;

	/// Continuous collision safety factor. Non-dimensional. Recommended range [0.01, 0.5]. Default
	/// is 0.5 for high performance with low tunneling risk.
	safetyFactor: f32 = 0;

	/// Optional body name for debugging. Up to B2_NAME_LENGTH characters. Empty for none.
	name: string = "";

	/// Use this to store application specific body data.
	userData: u64 = 0;

	/// Motion locks to restrict linear and angular movement. Caution: may lead to softer
	/// constraints along the locked direction
	motionLocks: b2MotionLocks = b2MotionLocks();

	/// Set this flag to false if this body should never fall asleep.
	enableSleep: bool = false;

	/// Is this body initially awake or sleeping?
	isAwake: bool = false;

	/// Treat this dynamic body as a high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	isBullet: bool = false;

	/// Used to disable a body. A disabled body does not move or collide.
	isEnabled: bool = false;

	/// This allows this body to bypass rotational speed limits. Should only be used for circular
	/// objects, like wheels.
	allowFastRotation: bool = false;

	/// Enable contact recycling. True by default. Leaving this enabled improves performance but may
	/// lead to ghost collisions that should be avoided on characters.
	enableContactRecycling: bool = false;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// This is used to filter collision on shapes. It affects shape-vs-shape collision and
/// shape-versus-query collision (such as b2World_CastRay).
export struct b2Filter {
	/// The collision category bits. Normally you would just set one bit. The category bits should
	/// represent your application object types.
	categoryBits: u64 = 0;

	/// The collision mask bits. This states the categories that this shape would accept for
	/// collision.
	maskBits: u64 = 0;

	/// Collision groups allow a certain group of objects to never collide (negative) or always
	/// collide (positive). A group index of zero has no effect. Non-zero group filtering always
	/// wins against the mask bits.
	groupIndex: i32 = 0;

	constructor(categoryBits: u64, maskBits: u64, groupIndex: i32) {
		this.categoryBits = categoryBits;
		this.maskBits = maskBits;
		this.groupIndex = groupIndex;
	}
}

/// The query filter is used to filter collisions between queries and shapes. For example, you may
/// want a ray-cast representing a projectile to hit players and the static environment but not
/// debris.
export struct b2QueryFilter {
	/// The collision category bits of this query. Normally you would just set one bit.
	categoryBits: u64 = 0;

	/// The collision mask bits. This states the shape categories that this query would accept for
	/// collision.
	maskBits: u64 = 0;

	constructor(categoryBits: u64, maskBits: u64) {
		this.categoryBits = categoryBits;
		this.maskBits = maskBits;
	}
}

/// Shape type
export type b2ShapeType = i32;

/// A circle with an offset
export const b2_circleShape: b2ShapeType = 0;

/// A capsule is an extruded circle
export const b2_capsuleShape: b2ShapeType = 1;

/// A line segment
export const b2_segmentShape: b2ShapeType = 2;

/// A convex polygon
export const b2_polygonShape: b2ShapeType = 3;

/// A line segment owned by a chain shape
export const b2_chainSegmentShape: b2ShapeType = 4;

/// The number of shape types
export const b2_shapeTypeCount: b2ShapeType = 5;

/// The surface material groups together parameters related to the shape surface, which is the 2D
/// boundary. Surface materials allow chain shapes to have per segment properties.
export struct b2SurfaceMaterial {
	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	friction: f32 = 0;

	/// The coefficient of restitution (bounce) usually in the range [0,1].
	/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	restitution: f32 = 0;

	/// The rolling resistance usually in the range [0,1].
	rollingResistance: f32 = 0;

	/// The tangent speed for conveyor belts
	tangentSpeed: f32 = 0;

	/// User material identifier. This is passed with query results and to friction and restitution
	/// combining functions. It is not used internally.
	userMaterialId: u64 = 0;

	/// Custom debug draw color.
	customColor: u32 = 0;
}

/// Used to create a shape. This is a temporary object used to bundle shape creation parameters. You
/// may use the same shape definition to create multiple shapes. Must be initialized using
/// b2DefaultShapeDef().
export struct b2ShapeDef {
	/// Use this to store application specific shape data.
	userData: u64 = 0;

	/// The surface material for this shape.
	material: b2SurfaceMaterial = b2SurfaceMaterial();

	/// The density, usually in kg/m^2.
	density: f32 = 0;

	/// Collision filtering data.
	filter: b2Filter = b2Filter();

	/// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See
	/// b2World_SetCustomFilterCallback.
	enableCustomFiltering: bool = false;

	/// A sensor shape generates overlap events but never generates a collision response.
	isSensor: bool = false;

	/// Enable sensor events for this shape. This applies to sensors and non-sensors. Both shapes
	/// involved must have this flag set to true. False by default, even for sensors.
	enableSensorEvents: bool = false;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Only one
	/// shape involved needs this flag set to true. Ignored for sensors. False by default.
	enableContactEvents: bool = false;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Only one
	/// shape involved needs this flag set to true. Ignored for sensors. False by default.
	enableHitEvents: bool = false;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are
	/// expensive and must be carefully handled due to multithreading. Ignored for sensors.
	enablePreSolveEvents: bool = false;

	/// When shapes are created they will scan the environment for collision the next time step.
	/// This flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	invokeContactCreation: bool = false;

	/// Should the body update the mass properties when this shape is created. Default is true.
	updateBodyMass: bool = false;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

const b2_emptyPoints = new StaticArray<b2Vec2>(0);

/// Used to create a chain of line segments. This is designed to eliminate ghost collisions with
/// some limitations. Must be initialized using b2DefaultChainDef().
/// @warning Do not use chain shapes unless you understand the limitations. This is an advanced
/// feature.
export class b2ChainDef {
	/// Use this to store application specific shape data.
	userData: u64 = 0;

	/// Array of chain points. Each consecutive pair forms a segment. For a loop the last and first
	/// point form a segment. Cloned. Only the first pointCount are used.
	points: StaticArray<b2Vec2> = b2_emptyPoints;

	/// The point count. At least 2 for an open chain and at least 3 for a loop.
	/// segmentCount = isLoop ? pointCount : pointCount - 1
	pointCount: i32 = 0;

	/// Leading ghost point for an open chain. Distance from first point must be greater than
	/// B2_LINEAR_SLOP. Ignored for loops.
	ghost1: b2Vec2 = b2Vec2();

	/// Ending ghost point for an open chain. Distance from last point must be greater than
	/// B2_LINEAR_SLOP. Ignored for loops.
	ghost2: b2Vec2 = b2Vec2();

	/// One material for the whole chain or one for each segment. Cloned.
	materials: StaticArray<b2SurfaceMaterial> = b2_defaultChainMaterials;

	/// The material count. Must be 1 or the segment count.
	materialCount: i32 = 0;

	/// Contact filtering data.
	filter: b2Filter = b2Filter();

	/// Indicates a closed chain formed by connecting the first and last point.
	isLoop: bool = false;

	/// Enable sensors to detect this chain. False by default.
	enableSensorEvents: bool = false;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Profiling data. Times are in milliseconds.
export struct b2Profile {
	step: f32 = 0;
	pairs: f32 = 0;
	collide: f32 = 0;
	solve: f32 = 0;
	solverSetup: f32 = 0;
	constraints: f32 = 0;
	prepareConstraints: f32 = 0;
	integrateVelocities: f32 = 0;
	warmStart: f32 = 0;
	solveImpulses: f32 = 0;
	integratePositions: f32 = 0;
	relaxImpulses: f32 = 0;
	restitution: f32 = 0;
	storeImpulses: f32 = 0;
	splitIslands: f32 = 0;
	transforms: f32 = 0;
	sensorHits: f32 = 0;
	jointEvents: f32 = 0;
	hitEvents: f32 = 0;
	refit: f32 = 0;
	bullets: f32 = 0;
	sleepIslands: f32 = 0;
	sensors: f32 = 0;
}

/// Counters that give details of the simulation size.
export struct b2Counters {
	byteCount: i64 = 0;
	bodyCount: i32 = 0;
	shapeCount: i32 = 0;
	contactCount: i32 = 0;
	jointCount: i32 = 0;
	islandCount: i32 = 0;
	stackUsed: i32 = 0;
	staticTreeHeight: i32 = 0;
	treeHeight: i32 = 0;
	taskCount: i32 = 0;
	colorCounts: FixedArray<i32, 24>;

	/// Number of contacts touched by the collide pass (graph contacts + awake-set non-touching).
	awakeContactCount: i32 = 0;

	/// Number of contacts recycled in the most recent step.
	recycledContactCount: i32 = 0;
}

/// Joint type enumeration
///
/// This is useful because all joint types use b2JointId and sometimes you want to get the type of a
/// joint.
export type b2JointType = i32;
export const b2_distanceJoint: b2JointType = 0; ///< constrain the distance between two points
export const b2_filterJoint: b2JointType = 1; ///< filter collision between two bodies
export const b2_motorJoint: b2JointType = 2; ///< fully control the movement of a body
export const b2_moverJoint: b2JointType = 3; ///< control the movement of the dynamic character mover
export const b2_pogoJoint: b2JointType = 4; ///< ground interaction for the dynamic character mover
export const b2_prismaticJoint: b2JointType = 5; ///< slide along an axis
export const b2_revoluteJoint: b2JointType = 6; ///< rotate around a pivot
export const b2_weldJoint: b2JointType = 7; ///< bring two bodies together with some flex
export const b2_wheelJoint: b2JointType = 8; ///< wheel spins with shock absorber

/// Base joint definition used by all joint types. The local frames are measured from the body's
/// origin rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken.
export struct b2JointDef {
	/// User data pointer
	userData: u64 = 0;

	/// The first attached body
	bodyIdA: b2BodyId = b2BodyId();

	/// The second attached body
	bodyIdB: b2BodyId = b2BodyId();

	/// The first local joint frame
	localFrameA: b2Transform = b2Transform();

	/// The second local joint frame
	localFrameB: b2Transform = b2Transform();

	/// Force threshold for joint events
	forceThreshold: f32 = 0;

	/// Torque threshold for joint events
	torqueThreshold: f32 = 0;

	/// Constraint hertz (advanced feature)
	constraintHertz: f32 = 0;

	/// Constraint damping ratio (advanced feature)
	constraintDampingRatio: f32 = 0;

	/// Debug draw scale
	drawScale: f32 = 0;

	/// Set this flag to true if the attached bodies should collide
	collideConnected: bool = false;
}

/// Distance joint definition Connects a point on body A with a point on body B by a segment. Useful
/// for ropes and springs.
export struct b2DistanceJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// The rest length of this joint. Clamped to a stable minimum value.
	length: f32 = 0;

	/// Enable the distance constraint to behave like a spring. If false then the distance joint
	/// will be rigid, overriding the limit and motor.
	enableSpring: bool = false;

	/// The lower spring force controls how much tension it can sustain.
	lowerSpringForce: f32 = 0;

	/// The upper spring force controls how much compression it can sustain.
	upperSpringForce: f32 = 0;

	/// The spring linear stiffness Hertz, cycles per second.
	hertz: f32 = 0;

	/// The spring linear damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// Enable/disable the joint limit
	enableLimit: bool = false;

	/// Minimum length for limit. Clamped to a stable minimum value.
	minLength: f32 = 0;

	/// Maximum length for limit. Must be greater than or equal to the minimum length.
	maxLength: f32 = 0;

	/// Enable/disable the joint motor
	enableMotor: bool = false;

	/// The maximum motor force, usually in newtons
	maxMotorForce: f32 = 0;

	/// The desired motor speed, usually in meters per second
	motorSpeed: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// A filter joint is used to disable collision between two specific bodies.
export struct b2FilterJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// A motor joint is used to control the relative velocity and/or transform between two bodies. With
/// a velocity of zero this acts like top-down friction.
export struct b2MotorJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// The desired linear velocity
	linearVelocity: b2Vec2 = b2Vec2();

	/// The maximum motor force in newtons
	maxVelocityForce: f32 = 0;

	/// The desired angular velocity
	angularVelocity: f32 = 0;

	/// The maximum motor torque in newton-meters
	maxVelocityTorque: f32 = 0;

	/// Linear spring hertz for position control
	linearHertz: f32 = 0;

	/// Linear spring damping ratio
	linearDampingRatio: f32 = 0;

	/// Maximum spring force in newtons
	maxSpringForce: f32 = 0;

	/// Angular spring hertz for position control
	angularHertz: f32 = 0;

	/// Angular spring damping ratio
	angularDampingRatio: f32 = 0;

	/// Maximum spring torque in newton-meters
	maxSpringTorque: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// A mover joint is used to move a dynamic character mover through velocity commands. The x and y
/// directions are handled separately. Does not affect rotation.
export struct b2MoverJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// The desired linear velocity
	linearVelocity: b2Vec2 = b2Vec2();

	/// The maximum motor force in newtons.
	maxVelocityForce: b2Vec2 = b2Vec2();

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Pogo joint definition
export struct b2PogoJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// Normal vector at the contact
	normal: b2Vec2 = b2Vec2();

	/// The spring stiffness Hertz, cycles per second
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// Spring length.
	restLength: f32 = 0;

	/// Maximum tension force. Controls how sticky the mover is to ground.
	maxTensionForce: f32 = 0;

	/// Maximum compression force. How hard the pogo pushes the mover up from the ground.
	maxCompressionForce: f32 = 0;

	/// The initial pogo internal impulse. The pogo is typically recreated every frame. This gives a
	/// way to move the internal state forward for smoother spring behavior.
	impulse: f32 = 0;

	/// The initial pogo internal velocity. The pogo is typically recreated every frame. This gives
	/// a way to move the internal state forward for smoother spring behavior.
	velocity: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Prismatic joint definition Body B may slide along the x-axis in local frame A. Body B cannot
/// rotate relative to body A. The joint translation is zero when the local frame origins coincide
/// in world space.
export struct b2PrismaticJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// Enable a linear spring along the prismatic joint axis
	enableSpring: bool = false;

	/// The spring stiffness Hertz, cycles per second
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// The target translation for the joint in meters. The spring-damper will drive to this
	/// translation.
	targetTranslation: f32 = 0;

	/// Enable/disable the joint limit
	enableLimit: bool = false;

	/// The lower translation limit
	lowerTranslation: f32 = 0;

	/// The upper translation limit
	upperTranslation: f32 = 0;

	/// Enable/disable the joint motor
	enableMotor: bool = false;

	/// The maximum motor force, typically in newtons
	maxMotorForce: f32 = 0;

	/// The desired motor speed, typically in meters per second
	motorSpeed: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Revolute joint definition A point on body B is fixed to a point on body A. Allows relative
/// rotation.
export struct b2RevoluteJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// The target angle for the joint in radians. The spring-damper will drive to this angle.
	targetAngle: f32 = 0;

	/// Enable a rotational spring on the revolute hinge axis
	enableSpring: bool = false;

	/// The spring stiffness Hertz, cycles per second
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// A flag to enable joint limits
	enableLimit: bool = false;

	/// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	lowerAngle: f32 = 0;

	/// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	upperAngle: f32 = 0;

	/// A flag to enable the joint motor
	enableMotor: bool = false;

	/// The maximum motor torque, typically in newton-meters
	maxMotorTorque: f32 = 0;

	/// The desired motor speed in radians per second
	motorSpeed: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Weld joint definition Connects two bodies together rigidly. This constraint provides springs to
/// mimic soft-body simulation.
/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
export struct b2WeldJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	linearHertz: f32 = 0;

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	angularHertz: f32 = 0;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linearDampingRatio: f32 = 0;

	/// Angular damping ratio, non-dimensional. Use 1 for critical damping.
	angularDampingRatio: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Wheel joint definition Body B is a wheel that may rotate freely and slide along the local x-axis
/// in frame A. The joint translation is zero when the local frame origins coincide in world space.
export struct b2WheelJointDef {
	/// Base joint definition
	base: b2JointDef = b2JointDef();

	/// Enable a linear spring along the local axis
	enableSpring: bool = false;

	/// Spring stiffness in Hertz
	hertz: f32 = 0;

	/// Spring damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// Enable/disable the joint linear limit
	enableLimit: bool = false;

	/// The lower translation limit
	lowerTranslation: f32 = 0;

	/// The upper translation limit
	upperTranslation: f32 = 0;

	/// Enable/disable the joint rotational motor
	enableMotor: bool = false;

	/// The maximum motor torque, typically in newton-meters
	maxMotorTorque: f32 = 0;

	/// The desired motor speed in radians per second
	motorSpeed: f32 = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// The explosion definition is used to configure options for explosions. Explosions consider shape
/// geometry when computing the impulse.
export struct b2ExplosionDef {
	/// Mask bits to filter shapes
	maskBits: u64 = 0;

	/// The center of the explosion in world space
	position: b2Pos = b2Vec2();

	/// The radius of the explosion
	radius: f32 = 0;

	/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	falloff: f32 = 0;

	/// Impulse per unit length. This applies an impulse according to the shape perimeter that is
	/// facing the explosion. Explosions only apply to circles, capsules, and polygons. This may be
	/// negative for implosions.
	impulsePerLength: f32 = 0;
}

/**
 * @defgroup events Events
 * World event types.
 */

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
export struct b2SensorBeginTouchEvent {
	/// The id of the sensor shape
	sensorShapeId: b2ShapeId = b2ShapeId();

	/// The id of the shape that began touching the sensor shape
	visitorShapeId: b2ShapeId = b2ShapeId();
}

/// An end touch event is generated when a shape stops overlapping a sensor shape. You should always
/// confirm the shape id is valid using b2Shape_IsValid.
export struct b2SensorEndTouchEvent {
	/// The id of the sensor shape
	/// @warning this shape may have been destroyed
	/// @see b2Shape_IsValid
	sensorShapeId: b2ShapeId = b2ShapeId();

	/// The id of the shape that stopped touching the sensor shape
	/// @warning this shape may have been destroyed
	/// @see b2Shape_IsValid
	visitorShapeId: b2ShapeId = b2ShapeId();
}

const b2_emptySensorBeginEvents = new Array<b2SensorBeginTouchEvent>();
const b2_emptySensorEndEvents = new Array<b2SensorEndTouchEvent>();

/// Sensor events are buffered in the world and are available as begin/end overlap event arrays
/// after the time step is complete. Note: these may become invalid if bodies and/or shapes are
/// destroyed. The arrays are the world's event arrays; only the first count entries are events.
export class b2SensorEvents {
	/// Array of sensor begin touch events
	beginEvents: Array<b2SensorBeginTouchEvent> = b2_emptySensorBeginEvents;

	/// Array of sensor end touch events
	endEvents: Array<b2SensorEndTouchEvent> = b2_emptySensorEndEvents;

	/// The number of begin touch events
	beginCount: i32 = 0;

	/// The number of end touch events
	endCount: i32 = 0;
}

/// A begin touch event is generated when two shapes begin touching.
export struct b2ContactBeginTouchEvent {
	/// Id of the first shape
	shapeIdA: b2ShapeId = b2ShapeId();

	/// Id of the second shape
	shapeIdB: b2ShapeId = b2ShapeId();

	/// The transient contact id. This contact may be destroyed automatically when the world is
	/// modified or simulated. Use b2Contact_IsValid before using this id.
	contactId: b2ContactId = b2ContactId();
}

/// An end touch event is generated when two shapes stop touching.
export struct b2ContactEndTouchEvent {
	/// Id of the first shape
	/// @warning this shape may have been destroyed
	/// @see b2Shape_IsValid
	shapeIdA: b2ShapeId = b2ShapeId();

	/// Id of the second shape
	/// @warning this shape may have been destroyed
	/// @see b2Shape_IsValid
	shapeIdB: b2ShapeId = b2ShapeId();

	/// Id of the contact.
	/// @warning this contact may have been destroyed
	/// @see b2Contact_IsValid
	contactId: b2ContactId = b2ContactId();
}

/// A hit event is generated when two shapes collide with a speed faster than the hit event
/// threshold. This may be reported for speculative contacts that have a confirmed impulse.
export struct b2ContactHitEvent {
	/// Id of the first shape
	shapeIdA: b2ShapeId = b2ShapeId();

	/// Id of the second shape
	shapeIdB: b2ShapeId = b2ShapeId();

	/// Id of the contact.
	/// @warning this contact may have been destroyed
	/// @see b2Contact_IsValid
	contactId: b2ContactId = b2ContactId();

	/// Point where the shapes hit. This is a mid-point between the two surfaces. It could be a
	/// speculative point where the shapes were not touching at the beginning of the time step.
	point: b2Pos = b2Vec2();

	/// Normal vector pointing from shape A to shape B
	normal: b2Vec2 = b2Vec2();

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	approachSpeed: f32 = 0;
}

const b2_emptyContactBeginEvents = new Array<b2ContactBeginTouchEvent>();
const b2_emptyContactEndEvents = new Array<b2ContactEndTouchEvent>();
const b2_emptyContactHitEvents = new Array<b2ContactHitEvent>();

/// Contact events are buffered in the Box2D world and are available as event arrays after the time
/// step is complete. Note: these may become invalid if bodies and/or shapes are destroyed
export class b2ContactEvents {
	/// Array of begin touch events
	beginEvents: Array<b2ContactBeginTouchEvent> = b2_emptyContactBeginEvents;

	/// Array of end touch events
	endEvents: Array<b2ContactEndTouchEvent> = b2_emptyContactEndEvents;

	/// Array of hit events
	hitEvents: Array<b2ContactHitEvent> = b2_emptyContactHitEvents;

	/// Number of begin touch events
	beginCount: i32 = 0;

	/// Number of end touch events
	endCount: i32 = 0;

	/// Number of hit events
	hitCount: i32 = 0;
}

/// Body move events triggered when a body moves. Triggered when a body moves due to simulation. Not
/// reported for bodies moved by the user. This also has a flag to indicate that the body went to
/// sleep so the application can also sleep that actor/entity/object associated with the body.
/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
export struct b2BodyMoveEvent {
	/// The body user data.
	userData: u64 = 0;

	/// The world transform of the body.
	transform: b2WorldTransform = b2Transform();

	/// The id of the body that moved.
	bodyId: b2BodyId = b2BodyId();

	/// Did the body fall asleep this time step?
	fellAsleep: bool = false;
}

const b2_emptyBodyMoveEvents = new Array<b2BodyMoveEvent>();

/// Body events are buffered in the Box2D world and are available as event arrays after the time
/// step is complete. Note: this data becomes invalid if bodies are destroyed
export class b2BodyEvents {
	/// Array of move events
	moveEvents: Array<b2BodyMoveEvent> = b2_emptyBodyMoveEvents;

	/// Number of move events
	moveCount: i32 = 0;
}

/// Joint events report joints that are awake and have a force and/or torque exceeding the
/// threshold. The observed forces and torques are not returned for efficiency reasons.
export struct b2JointEvent {
	/// The joint id
	jointId: b2JointId = b2JointId();

	/// The user data from the joint for convenience
	userData: u64 = 0;
}

const b2_emptyJointEvents = new Array<b2JointEvent>();

/// Joint events are buffered in the world and are available as event arrays after the time step is
/// complete. Note: this data becomes invalid if joints are destroyed
export class b2JointEvents {
	/// Array of events
	jointEvents: Array<b2JointEvent> = b2_emptyJointEvents;

	/// Number of events
	count: i32 = 0;
}

/// The contact data for two shapes. By convention the manifold normal points from shape A to shape
/// B.
/// @see b2Shape_GetContactData() and b2Body_GetContactData()
export struct b2ContactData {
	/// The contact id. You may hold onto this to track a contact across time steps. This id may
	/// become orphaned. Use b2Contact_IsValid before using it for other functions.
	contactId: b2ContactId = b2ContactId();

	/// The first shape id.
	shapeIdA: b2ShapeId = b2ShapeId();

	/// The second shape id.
	shapeIdB: b2ShapeId = b2ShapeId();

	/// The manifold copied from the contact.
	manifold: b2Manifold = b2Manifold();
}

/// Prototype for a contact filter callback. This is called when a contact pair is considered for
/// collision. This allows you to perform custom logic to prevent collision between shapes. This is
/// only called if one of the two shapes has custom filtering enabled. Return false if you want to
/// disable the collision
/// @warning Do not attempt to modify the world inside this callback
export type b2CustomFilterCallbackFcn = (shapeIdA: b2ShapeId, shapeIdB: b2ShapeId) => bool;

// C passes a b2Manifold* the callback may modify; closures cannot take ref parameters, so the
// caller copies the manifold into this holder and back.
export class b2PreSolveManifold {
	manifold: b2Manifold = b2Manifold();
}

/// Prototype for a pre-solve callback. This is called after a contact manifold is updated. This
/// allows you to inspect a manifold before it goes to the solver. If you are careful, you can
/// modify the manifold (e.g. modify the normal). You can set the manifold point count to zero to
/// disable collision.
/// @warning Do not attempt to modify the world inside this callback
export type b2PreSolveCallbackFcn = (shapeIdA: b2ShapeId, shapeIdB: b2ShapeId, manifold: b2PreSolveManifold) => void;

/// Prototype for a pre-continuous callback. This is called when a time of impact event is computed
/// in continuous collision. This function can be used to skip the event. Return false if you want
/// to disable the time of impact event.
/// @warning Do not attempt to modify the world inside this callback
export type b2PreContinuousCallbackFcn = (shapeIdA: b2ShapeId, shapeIdB: b2ShapeId, point: b2Pos, normal: b2Vec2) => bool;

/// Prototype callback for overlap queries. Called for each shape found in the query.
/// @see b2World_OverlapAABB
/// @return false to terminate the query.
export type b2OverlapResultCallbackFcn = (shapeId: b2ShapeId) => bool;

/// Prototype callback for ray and shape casts. Called for each shape found in the query. You
/// control how the ray cast proceeds by returning a float:
/// - return -1: ignore this shape and continue
/// - return 0: terminate the ray cast
/// - return fraction: clip the ray to this point
/// - return 1: don't clip the ray and continue A cast with initial overlap will return a zero
///   fraction and a zero normal.
/// @see b2World_CastRay
export type b2CastResultCallbackFcn = (shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32) => f32;

/// Used to collect collision planes for character movers. Return true to continue gathering planes.
export type b2PlaneResultCallbackFcn = (shapeId: b2ShapeId, plane: b2PlaneResult) => bool;

/// Static metadata describing a recording, resolved once when the player opens the file.
export struct b2ReplayInfo {
	frameCount: i32 = 0; ///< total recorded steps
	workerCount: i32 = 0; ///< worker count used for the replay world
	timeStep: f32 = 0; ///< dt of the recorded steps
	subStepCount: i32 = 0; ///< recorded sub-steps
	lengthScale: f32 = 0; ///< length units per meter in effect when recorded
	bounds: b2AABB = b2AABB(); ///< accumulated world bounds over the recording, zero-extent if unavailable
}

/// The kind of a recorded spatial query, matching the public query and cast functions.
export type b2ReplayQueryType = i32;
export const b2_replayQueryOverlapAABB: b2ReplayQueryType = 0;
export const b2_replayQueryOverlapShape: b2ReplayQueryType = 1;
export const b2_replayQueryCastRay: b2ReplayQueryType = 2;
export const b2_replayQueryCastShape: b2ReplayQueryType = 3;
export const b2_replayQueryCollideMover: b2ReplayQueryType = 4;
export const b2_replayQueryCastRayClosest: b2ReplayQueryType = 5;
export const b2_replayQueryCastMover: b2ReplayQueryType = 6;

/// A spatial query recorded during a replayed frame, exposed for inspection.
export struct b2ReplayQueryInfo {
	type: b2ReplayQueryType = 0; ///< the query type
	filter: b2QueryFilter = b2QueryFilter(); ///< the query filter
	aabb: b2AABB = b2AABB(); ///< overlap AABB, relative to origin
	origin: b2Pos = b2Vec2(); ///< query origin
	translation: b2Vec2 = b2Vec2(); ///< ray and cast translation
	hitCount: i32 = 0; ///< number of recorded results
}

/// One result of a recorded spatial query.
export struct b2ReplayQueryHit {
	shapeId: b2ShapeId = b2ShapeId(); ///< the shape hit
	point: b2Pos = b2Vec2(); ///< the hit point on the shape surface
	normal: b2Vec2 = b2Vec2(); ///< the unit normal vector at the hit point
	fraction: f32 = 0; ///< the query translation fraction
}

// src/types.c

/// Use this to initialize your world definition
export function b2DefaultWorldDef(): b2WorldDef {
	let lengthUnits = b2GetLengthUnitsPerMeter();
	let def = new b2WorldDef();
	def.gravity.x = 0.0;
	def.gravity.y = -10.0;
	def.hitEventThreshold = 1.0 * lengthUnits;
	def.restitutionThreshold = 1.0 * lengthUnits;
	def.restitutionIterations = 2;
	def.enableRestitutionPropagation = false;
	def.contactSpeed = 3.0 * lengthUnits;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 10.0;

	// 400 meters per second, faster than the speed of sound
	def.maximumLinearSpeed = 400.0 * lengthUnits;
	def.enableSleep = true;
	def.enableContinuous = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your body definition
export function b2DefaultBodyDef(): b2BodyDef {
	let def = new b2BodyDef();
	def.type = b2_staticBody;
	def.rotation = b2Rot_identity;
	def.safetyFactor = 0.5;
	def.sleepThreshold = 0.05 * b2GetLengthUnitsPerMeter();
	def.gravityScale = 1.0;
	def.enableSleep = true;
	def.enableContactRecycling = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your filter
export function b2DefaultFilter(): b2Filter {
	let filter = b2Filter(B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS, 0);
	return filter;
}

/// Use this to initialize your query filter
export function b2DefaultQueryFilter(): b2QueryFilter {
	let filter = b2QueryFilter(B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS);
	return filter;
}

/// Use this to initialize your shape definition
export function b2DefaultShapeDef(): b2ShapeDef {
	let def = b2ShapeDef();
	def.material.friction = 0.6;
	def.density = 1.0;
	def.filter = b2DefaultFilter();
	def.updateBodyMass = true;
	def.invokeContactCreation = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your surface material
export function b2DefaultSurfaceMaterial(): b2SurfaceMaterial {
	let material = b2SurfaceMaterial();
	material.friction = 0.6;

	return material;
}

function b2MakeDefaultChainMaterials(): StaticArray<b2SurfaceMaterial> {
	let materials = new StaticArray<b2SurfaceMaterial>(1);
	materials[0].friction = 0.6;
	return materials;
}

// C's function static, shared by every default chain definition
const b2_defaultChainMaterials: StaticArray<b2SurfaceMaterial> = b2MakeDefaultChainMaterials();

/// Use this to initialize your chain definition
export function b2DefaultChainDef(): b2ChainDef {
	let def = new b2ChainDef();
	def.materials = b2_defaultChainMaterials;
	def.materialCount = 1;
	// using infinity as a sentinel
	def.ghost1.x = INFINITY;
	def.ghost1.y = INFINITY;
	def.ghost2.x = INFINITY;
	def.ghost2.y = INFINITY;
	def.filter = b2DefaultFilter();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

// src/joint.c defaults. They live here with the other defaults; lib/joint.as re-exports them.

function b2DefaultJointDef(): b2JointDef {
	let def = b2JointDef();
	def.localFrameA.q = b2Rot_identity;
	def.localFrameB.q = b2Rot_identity;
	def.forceThreshold = FLT_MAX;
	def.torqueThreshold = FLT_MAX;
	def.constraintHertz = 60.0;
	def.constraintDampingRatio = 2.0;
	def.drawScale = b2GetLengthUnitsPerMeter();
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultDistanceJointDef(): b2DistanceJointDef {
	let def = b2DistanceJointDef();
	def.base = b2DefaultJointDef();
	def.lowerSpringForce = -FLT_MAX;
	def.upperSpringForce = FLT_MAX;
	def.length = 1.0;
	def.maxLength = B2_HUGE();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultFilterJointDef(): b2FilterJointDef {
	let def = b2FilterJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultMotorJointDef(): b2MotorJointDef {
	let def = b2MotorJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultMoverJointDef(): b2MoverJointDef {
	let def = b2MoverJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultPogoJointDef(): b2PogoJointDef {
	let def = b2PogoJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultPrismaticJointDef(): b2PrismaticJointDef {
	let def = b2PrismaticJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition.
export function b2DefaultRevoluteJointDef(): b2RevoluteJointDef {
	let def = b2RevoluteJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultWeldJointDef(): b2WeldJointDef {
	let def = b2WeldJointDef();
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b2DefaultWheelJointDef(): b2WheelJointDef {
	let def = b2WheelJointDef();
	def.base = b2DefaultJointDef();
	def.enableSpring = true;
	def.hertz = 1.0;
	def.dampingRatio = 0.7;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your explosion definition
export function b2DefaultExplosionDef(): b2ExplosionDef {
	let def = b2ExplosionDef();
	def.maskBits = B2_DEFAULT_MASK_BITS;
	return def;
}
