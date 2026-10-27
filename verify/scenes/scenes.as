// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript mirror of verify/scenes/scenes.c, statement for statement over lib/box2d.as. Every
// int to float conversion is an explicit <f32>, every float local is annotated. Query callbacks
// carry no context, so the state they fill sits in module globals.

import { FLT_MAX } from "../../lib/core";
import {
	B2_PI,
	b2AABB,
	b2Add,
	b2BodyDef,
	b2BodyId,
	b2Body_ApplyAngularImpulse,
	b2Body_ApplyForce,
	b2Body_ApplyForceToCenter,
	b2Body_ApplyLinearImpulse,
	b2Body_ApplyLinearImpulseToCenter,
	b2Body_ApplyTorque,
	b2Body_ClearForces,
	b2Body_ComputeBounds,
	b2Body_Disable,
	b2Body_Enable,
	b2Body_EnableHitEvents,
	b2Body_EnableSleeping,
	b2Body_GetAngularVelocity,
	b2Body_GetContactData,
	b2Body_GetJointCount,
	b2Body_GetLinearVelocity,
	b2Body_GetLocalCenter,
	b2Body_GetLocalPoint,
	b2Body_GetLocalPointVelocity,
	b2Body_GetLocalVector,
	b2Body_GetMass,
	b2Body_GetPosition,
	b2Body_GetRotationalInertia,
	b2Body_GetShapeCount,
	b2Body_GetShapes,
	b2Body_GetType,
	b2Body_GetWorldCenter,
	b2Body_GetWorldPoint,
	b2Body_GetWorldPointVelocity,
	b2Body_GetWorldVector,
	b2Body_IsBullet,
	b2Body_IsEnabled,
	b2Body_IsSleepingEnabled,
	b2Body_IsValid,
	b2Body_SetAngularVelocity,
	b2Body_SetAwake,
	b2Body_SetBullet,
	b2Body_SetGravityScale,
	b2Body_SetLinearVelocity,
	b2Body_SetTargetTransform,
	b2Body_SetTransform,
	b2Body_SetType,
	b2Body_WakeTouching,
	b2Capacity,
	b2Capsule,
	b2CastOutput,
	b2Circle,
	b2ClipVector,
	b2CollisionPlane,
	b2ComputeCosSin,
	b2ComputeHull,
	b2ContactData,
	b2ContactId,
	b2Contact_GetData,
	b2Contact_IsValid,
	b2CreateBody,
	b2CreateCapsuleShape,
	b2CreateChain,
	b2CreateCircleShape,
	b2CreateDistanceJoint,
	b2CreateFilterJoint,
	b2CreateMotorJoint,
	b2CreateMoverJoint,
	b2CreatePogoJoint,
	b2CreatePolygonShape,
	b2CreatePrismaticJoint,
	b2CreateRevoluteJoint,
	b2CreateSegmentShape,
	b2CreateWeldJoint,
	b2CreateWheelJoint,
	b2CreateWorld,
	b2Cross,
	b2CrossSV,
	b2DefaultBodyDef,
	b2DefaultChainDef,
	b2DefaultDistanceJointDef,
	b2DefaultExplosionDef,
	b2DefaultFilterJointDef,
	b2DefaultMotorJointDef,
	b2DefaultMoverJointDef,
	b2DefaultPogoJointDef,
	b2DefaultPrismaticJointDef,
	b2DefaultQueryFilter,
	b2DefaultRevoluteJointDef,
	b2DefaultShapeDef,
	b2DefaultSurfaceMaterial,
	b2DefaultWeldJointDef,
	b2DefaultWheelJointDef,
	b2DefaultWorldDef,
	b2DestroyBody,
	b2DestroyJoint,
	b2DestroyShape,
	b2DestroyWorld,
	b2DistanceJoint_EnableSpring,
	b2DistanceJoint_GetCurrentLength,
	b2DistanceJoint_GetCurrentMotorForce,
	b2DistanceJoint_GetSpringForce,
	b2DistanceJoint_SetLimits,
	b2DistanceJoint_SetRestLength,
	b2DistanceJoint_SetTargetMotorSpeed,
	b2Dot,
	b2Filter,
	b2GetLengthAndNormalize,
	b2JointId,
	b2Joint_GetAngularSeparation,
	b2Joint_GetConstraintForce,
	b2Joint_GetConstraintTorque,
	b2Joint_GetLinearSeparation,
	b2Joint_IsValid,
	b2Joint_WakeBodies,
	b2Length,
	b2LengthSquared,
	b2MakeBox,
	b2MakeOffsetBox,
	b2MakeOffsetRoundedBox,
	b2MakePolygon,
	b2MakeProxy,
	b2MakeRot,
	b2MakeRoundedBox,
	b2MakeSquare,
	b2Manifold,
	b2MaxFloat,
	b2MinFloat,
	b2MotorJoint_SetAngularDampingRatio,
	b2MotorJoint_SetAngularHertz,
	b2MotorJoint_SetAngularVelocity,
	b2MotorJoint_SetLinearDampingRatio,
	b2MotorJoint_SetLinearHertz,
	b2MotorJoint_SetLinearVelocity,
	b2MotorJoint_SetMaxSpringForce,
	b2MotorJoint_SetMaxSpringTorque,
	b2MoverJoint_SetLinearVelocity,
	b2MoverJoint_SetMaxVelocityForce,
	b2MulSV,
	b2MulSub,
	b2Neg,
	b2OffsetPos,
	b2PlaneResult,
	b2PogoJoint_GetCurrentLength,
	b2PogoJoint_GetCurrentSpeed,
	b2PogoJoint_GetImpulse,
	b2Polygon,
	b2Pos,
	b2Pos_zero,
	b2PreSolveManifold,
	b2PrismaticJoint_EnableMotor,
	b2PrismaticJoint_GetCurrentMotorForce,
	b2PrismaticJoint_GetCurrentSpeed,
	b2PrismaticJoint_GetCurrentTranslation,
	b2PrismaticJoint_GetTargetMotorSpeed,
	b2PrismaticJoint_SetLimits,
	b2PrismaticJoint_SetTargetMotorSpeed,
	b2PrismaticJoint_SetTargetTranslation,
	b2QueryFilter,
	b2RevoluteJoint_EnableMotor,
	b2RevoluteJoint_GetCurrentAngle,
	b2RevoluteJoint_GetCurrentMotorTorque,
	b2RevoluteJoint_GetCurrentSpeed,
	b2RevoluteJoint_SetLimits,
	b2RevoluteJoint_SetSpringHertz,
	b2RevoluteJoint_SetTargetAngle,
	b2RevoluteJoint_SetTargetMotorSpeed,
	b2Rot,
	b2RotateVector,
	b2Rot_identity,
	b2Segment,
	b2ShapeId,
	b2Shape_CastRay,
	b2Shape_EnableContactEvents,
	b2Shape_EnableSensorEvents,
	b2Shape_GetBody,
	b2Shape_GetBounds,
	b2Shape_GetClosestPoint,
	b2Shape_GetContactCapacity,
	b2Shape_GetContactData,
	b2Shape_GetSensorCapacity,
	b2Shape_GetSensorData,
	b2Shape_GetUserData,
	b2Shape_IsValid,
	b2Shape_TestPoint,
	b2SolvePlanes,
	b2SpringDamper,
	b2Sub,
	b2SurfaceMaterial,
	b2Transform,
	b2Vec2,
	b2Vec2_zero,
	b2WeldJoint_SetAngularHertz,
	b2WeldJoint_SetLinearDampingRatio,
	b2WheelJoint_EnableLimit,
	b2WheelJoint_GetCurrentMotorTorque,
	b2WheelJoint_SetMaxMotorTorque,
	b2WheelJoint_SetSpringDampingRatio,
	b2WheelJoint_SetSpringHertz,
	b2WheelJoint_SetTargetMotorSpeed,
	b2WorldId,
	b2World_CastMover,
	b2World_CastRay,
	b2World_CastRayClosest,
	b2World_CastShape,
	b2World_CollideMover,
	b2World_EnableSleeping,
	b2World_Explode,
	b2World_GetBodyEvents,
	b2World_GetBounds,
	b2World_GetContactEvents,
	b2World_GetGravity,
	b2World_GetJointEvents,
	b2World_GetSensorEvents,
	b2World_IsSleepingEnabled,
	b2World_OverlapAABB,
	b2World_OverlapShape,
	b2World_SetCustomFilterCallback,
	b2World_SetGravity,
	b2World_SetHitEventThreshold,
	b2World_SetPreSolveCallback,
	b2World_Step,
	b2_dynamicBody,
	b2_kinematicBody,
	b2_nullBodyId,
	b2_nullJointId,
	b2_staticBody,
} from "../../lib/box2d";

export const SCENE_SUBSTEP_COUNT: i32 = 4;
export const SCENE_KEY_CAPACITY: i32 = 16;
export const SCENE_SPAWN_CAPACITY: i32 = 64;
export const SCENE_HIT_CAPACITY: i32 = 32;
export const SCENE_CONTACT_CAPACITY: i32 = 8;
export const SCENE_MOVER_PLANE_CAPACITY: i32 = 8;
export const SCENE_BONE_COUNT: i32 = 11;

// samples/car.h Car, and the bike built the same way
export class SceneVehicle {
	chassisId: b2BodyId = b2BodyId();
	rearWheelId: b2BodyId = b2BodyId();
	frontWheelId: b2BodyId = b2BodyId();
	rearAxleId: b2JointId = b2JointId();
	frontAxleId: b2JointId = b2JointId();
}

// samples/geometric_mover.h GeometricMover, minus drawing
export class SceneGeometricMover {
	jumpSpeed: f32 = 0;
	maxSpeed: f32 = 0;
	minSpeed: f32 = 0;
	stopSpeed: f32 = 0;
	accelerate: f32 = 0;
	airSteer: f32 = 0;
	friction: f32 = 0;
	gravity: f32 = 0;
	pogoRestLength: f32 = 0;
	pogoHertz: f32 = 0;
	pogoDampingRatio: f32 = 0;
	pogoGroundForce: f32 = 0;
	minGroundNormalY: f32 = 0;
	pushBodies: bool = false;
	capsule: b2Capsule = b2Capsule();
	filter: b2QueryFilter = b2QueryFilter();
	position: b2Pos = b2Vec2();
	velocity: b2Vec2 = b2Vec2();
	pogoVelocity: f32 = 0;
	onGround: bool = false;
	planes: StaticArray<b2CollisionPlane> = new StaticArray<b2CollisionPlane>(SCENE_MOVER_PLANE_CAPACITY);
	planePoints: StaticArray<b2Pos> = new StaticArray<b2Pos>(SCENE_MOVER_PLANE_CAPACITY);
	planeShapes: StaticArray<b2ShapeId> = new StaticArray<b2ShapeId>(SCENE_MOVER_PLANE_CAPACITY);
	planeCount: i32 = 0;
	totalIterations: i32 = 0;
	pogoFraction: f32 = 0;
	pogoHit: bool = false;
}

// samples/dynamic_mover.h DynamicMoverCastResult
export class SceneCastResult {
	point: b2Pos = b2Vec2();
	normal: b2Vec2 = b2Vec2();
	bodyId: b2BodyId = b2BodyId();
	fraction: f32 = 0;
	hit: bool = false;
}

// samples/dynamic_mover.h DynamicMover, minus drawing
export class SceneDynamicMover {
	jumpSpeed: f32 = 0;
	maxSpeed: f32 = 0;
	minSpeed: f32 = 0;
	stopSpeed: f32 = 0;
	accelerate: f32 = 0;
	airSteer: f32 = 0;
	friction: f32 = 0;
	gravityScale: f32 = 0;
	maxGroundForce: f32 = 0;
	maxAirForce: f32 = 0;
	pogoRestLength: f32 = 0;
	pogoHertz: f32 = 0;
	pogoDampingRatio: f32 = 0;
	pogoCompressionScale: f32 = 0;
	pogoTensionScale: f32 = 0;
	minGroundNormalY: f32 = 0;
	capsule: b2Capsule = b2Capsule();
	filter: b2Filter = b2Filter();
	groundId: b2BodyId = b2BodyId();
	moverId: b2BodyId = b2BodyId();
	moverJointId: b2JointId = b2JointId();
	pogoJointId: b2JointId = b2JointId();
	castResult: SceneCastResult = new SceneCastResult();
	velocity: b2Vec2 = b2Vec2();
	onGround: bool = false;
	walkable: bool = false;
	jumping: bool = false;
	jumpTicks: i32 = 0;
	pogoImpulse: f32 = 0;
	pogoVelocity: f32 = 0;
	pogoLength: f32 = 0;
	pogoFraction: f32 = 0;
	pogoHit: bool = false;
}

// shared/benchmarks.c RainData; each human's bone bodies and joints at
// ( groupIndex * group size + human ) * SCENE_BONE_COUNT + bone, the hip's joint null
export class SceneRain {
	boneBodies: StaticArray<b2BodyId> = new StaticArray<b2BodyId>(0);
	boneJoints: StaticArray<b2JointId> = new StaticArray<b2JointId>(0);
	gridSize: f32 = 0;
	gridCount: i32 = 0;
	columnCount: i32 = 0;
	columnIndex: i32 = 0;
}

export class Scene {
	name: string = "";
	index: i32 = 0;
	worldId: b2WorldId = b2WorldId();

	// Every body the scene created, in creation order. Destroyed bodies stay in the list.
	bodies: Array<b2BodyId> = new Array<b2BodyId>(0);
	bodyCount: i32 = 0;

	// Words pushed by Scene_Step for the current step, recorded after the bodies
	probes: Array<u32> = new Array<u32>(0);

	seed: u32 = 0;
	time: f32 = 0;
	jumpReleased: bool = false;

	// Handles each scene keeps for its script, slots documented per scene
	groundId: b2BodyId = b2BodyId();
	keyBodies: StaticArray<b2BodyId> = new StaticArray<b2BodyId>(SCENE_KEY_CAPACITY);
	keyShapes: StaticArray<b2ShapeId> = new StaticArray<b2ShapeId>(SCENE_KEY_CAPACITY);
	keyJoints: StaticArray<b2JointId> = new StaticArray<b2JointId>(SCENE_KEY_CAPACITY);
	keyJointCount: i32 = 0;

	// Bodies created by the script, oldest first
	spawned: StaticArray<b2BodyId> = new StaticArray<b2BodyId>(SCENE_SPAWN_CAPACITY);
	spawnCount: i32 = 0;
	spawnHead: i32 = 0;

	car: SceneVehicle = new SceneVehicle();
	bike: SceneVehicle = new SceneVehicle();
	geometricMover: SceneGeometricMover = new SceneGeometricMover();
	dynamicMover: SceneDynamicMover = new SceneDynamicMover();
	rain: SceneRain = new SceneRain();
}

const SCENE_PYRAMID: i32 = 0;
const SCENE_SHAPE_RAIN: i32 = 1;
const SCENE_CHAIN_CAR: i32 = 2;
const SCENE_JOINTS: i32 = 3;
const SCENE_DYNAMIC_MOVER: i32 = 4;
const SCENE_CHURN: i32 = 5;
const SCENE_BULLETS: i32 = 6;
const SCENE_SENSORS: i32 = 7;
const SCENE_EVENTS: i32 = 8;
const SCENE_QUERIES: i32 = 9;
const SCENE_MOVER: i32 = 10;
const SCENE_MANY_BODIES: i32 = 11;
const SCENE_LARGE_PYRAMID: i32 = 12;
const SCENE_JOINT_GRID: i32 = 13;
const SCENE_RAIN: i32 = 14;
const SCENE_SMASH: i32 = 15;
const SCENE_SPINNER: i32 = 16;
const SCENE_TUMBLER: i32 = 17;

const s_sceneNames: StaticArray<string> = [
	"pyramid",
	"shape_rain",
	"chain_car",
	"joints",
	"dynamic_mover",
	"churn",
	"bullets",
	"sensors",
	"events",
	"queries",
	"mover",
	"many_bodies",
	"large_pyramid",
	"joint_grid",
	"rain",
	"smash",
	"spinner",
	"tumbler",
];

// shared/benchmarks.c release sizes
const RAIN_ROW_COUNT: i32 = 5;
const RAIN_COLUMN_COUNT: i32 = 40;
const RAIN_GROUP_SIZE: i32 = 5;
const SPINNER_POINT_COUNT: i32 = 360;

// samples/sample_character.cpp CollisionBits; AllBits is ~0u, which zero extends
const s_staticBit: u64 = 0x0001;
const s_moverBit: u64 = 0x0002;
const s_dynamicBit: u64 = 0x0004;
const s_allBits: u64 = 0xffffffff;

// Scratch the C keeps on the stack
const s_contactData: StaticArray<b2ContactData> = new StaticArray<b2ContactData>(SCENE_CONTACT_CAPACITY);
const s_shapeIds: StaticArray<b2ShapeId> = new StaticArray<b2ShapeId>(SCENE_HIT_CAPACITY);

export function Scene_GetCount(): i32 {
	return s_sceneNames.length;
}

export function Scene_GetName(index: i32): string {
	return s_sceneNames[index];
}

// Numerical Recipes LCG
function Scene_Random(scene: Scene): u32 {
	scene.seed = scene.seed * 1664525 + 1013904223;
	return scene.seed;
}

// Uniform in [lo, hi] from the top 24 bits
function Scene_RandomFloat(scene: Scene, lo: f32, hi: f32): f32 {
	let bits: u32 = Scene_Random(scene) >> 8;
	let r: f32 = <f32>bits * (<f32>1.0 / <f32>16777216.0);
	let range: f32 = hi - lo;
	let scaled: f32 = range * r;
	return lo + scaled;
}

function Scene_AddBody(scene: Scene, def: b2BodyDef): b2BodyId {
	let bodyId = b2CreateBody(scene.worldId, def);
	scene.bodies.push(bodyId);
	scene.bodyCount += 1;
	return bodyId;
}

function Scene_AddSpawn(scene: Scene, bodyId: b2BodyId): void {
	if (scene.spawnCount < SCENE_SPAWN_CAPACITY) {
		scene.spawned[scene.spawnCount] = bodyId;
		scene.spawnCount += 1;
	}
}

function Probe_Word(scene: Scene, value: u32): void {
	scene.probes.push(value);
}

function Probe_Int(scene: Scene, value: i32): void {
	Probe_Word(scene, <u32>value);
}

function Probe_Bool(scene: Scene, value: bool): void {
	Probe_Word(scene, value ? <u32>1 : <u32>0);
}

function Probe_Float(scene: Scene, value: f32): void {
	Probe_Word(scene, reinterpret<u32>(value));
}

function Probe_Vec2(scene: Scene, v: b2Vec2): void {
	Probe_Float(scene, v.x);
	Probe_Float(scene, v.y);
}

function Probe_Rot(scene: Scene, q: b2Rot): void {
	Probe_Float(scene, q.c);
	Probe_Float(scene, q.s);
}

function Probe_ShapeId(scene: Scene, id: b2ShapeId): void {
	Probe_Int(scene, id.index1);
	Probe_Word(scene, <u32>id.generation);
}

function Probe_BodyId(scene: Scene, id: b2BodyId): void {
	Probe_Int(scene, id.index1);
	Probe_Word(scene, <u32>id.generation);
}

function Probe_JointId(scene: Scene, id: b2JointId): void {
	Probe_Int(scene, id.index1);
	Probe_Word(scene, <u32>id.generation);
}

function Probe_ContactId(scene: Scene, id: b2ContactId): void {
	Probe_Int(scene, id.index1);
	Probe_Word(scene, id.generation);
}

// Point count, normal, rolling impulse, then per point anchors, separation and impulses.
// normalVelocity is left out: the solver only computes it for hit events.
function Probe_Manifold(scene: Scene, manifold: b2Manifold): void {
	Probe_Int(scene, manifold.pointCount);
	Probe_Vec2(scene, manifold.normal);
	Probe_Float(scene, manifold.rollingImpulse);
	for (let i = 0; i < manifold.pointCount; ++i) {
		let mp = manifold.points[i];
		Probe_Vec2(scene, mp.anchorA);
		Probe_Vec2(scene, mp.anchorB);
		Probe_Float(scene, mp.separation);
		Probe_Float(scene, mp.normalImpulse);
		Probe_Float(scene, mp.tangentImpulse);
		Probe_Float(scene, mp.totalNormalImpulse);
		Probe_Int(scene, <i32>mp.id);
		Probe_Bool(scene, mp.persisted);
	}
}

function Probe_ContactData(scene: Scene, data: b2ContactData): void {
	Probe_ContactId(scene, data.contactId);
	Probe_ShapeId(scene, data.shapeIdA);
	Probe_ShapeId(scene, data.shapeIdB);
	Probe_Manifold(scene, data.manifold);
}

// Begin, end and hit events in array order, each block led by its count
function Probe_ContactEvents(scene: Scene): void {
	let events = b2World_GetContactEvents(scene.worldId);

	Probe_Int(scene, events.beginCount);
	for (let i = 0; i < events.beginCount; ++i) {
		let event = events.beginEvents[i];
		Probe_ShapeId(scene, event.shapeIdA);
		Probe_ShapeId(scene, event.shapeIdB);
		Probe_ContactId(scene, event.contactId);
	}

	Probe_Int(scene, events.endCount);
	for (let i = 0; i < events.endCount; ++i) {
		let event = events.endEvents[i];
		Probe_ShapeId(scene, event.shapeIdA);
		Probe_ShapeId(scene, event.shapeIdB);
		Probe_ContactId(scene, event.contactId);
	}

	Probe_Int(scene, events.hitCount);
	for (let i = 0; i < events.hitCount; ++i) {
		let event = events.hitEvents[i];
		Probe_ShapeId(scene, event.shapeIdA);
		Probe_ShapeId(scene, event.shapeIdB);
		Probe_ContactId(scene, event.contactId);
		Probe_Vec2(scene, event.point);
		Probe_Vec2(scene, event.normal);
		Probe_Float(scene, event.approachSpeed);
	}
}

function Probe_SensorEvents(scene: Scene): void {
	let events = b2World_GetSensorEvents(scene.worldId);

	Probe_Int(scene, events.beginCount);
	for (let i = 0; i < events.beginCount; ++i) {
		let event = events.beginEvents[i];
		Probe_ShapeId(scene, event.sensorShapeId);
		Probe_ShapeId(scene, event.visitorShapeId);
	}

	Probe_Int(scene, events.endCount);
	for (let i = 0; i < events.endCount; ++i) {
		let event = events.endEvents[i];
		Probe_ShapeId(scene, event.sensorShapeId);
		Probe_ShapeId(scene, event.visitorShapeId);
	}
}

function Probe_BodyMoveEvents(scene: Scene): void {
	let events = b2World_GetBodyEvents(scene.worldId);

	Probe_Int(scene, events.moveCount);
	for (let i = 0; i < events.moveCount; ++i) {
		let event = events.moveEvents[i];
		Probe_BodyId(scene, event.bodyId);
		Probe_Vec2(scene, event.transform.p);
		Probe_Rot(scene, event.transform.q);
		Probe_Bool(scene, event.fellAsleep);
	}
}

function Probe_JointEvents(scene: Scene): void {
	let events = b2World_GetJointEvents(scene.worldId);

	Probe_Int(scene, events.count);
	for (let i = 0; i < events.count; ++i) {
		let event = events.jointEvents[i];
		Probe_JointId(scene, event.jointId);
		Probe_Word(scene, <u32>event.userData);
	}
}

// Force, torque and separations of any joint, a valid flag first
function Probe_Joint(scene: Scene, jointId: b2JointId): void {
	let valid = b2Joint_IsValid(jointId);
	Probe_Bool(scene, valid);
	if (valid == false) {
		return;
	}

	Probe_Vec2(scene, b2Joint_GetConstraintForce(jointId));
	Probe_Float(scene, b2Joint_GetConstraintTorque(jointId));
	Probe_Float(scene, b2Joint_GetLinearSeparation(jointId));
	Probe_Float(scene, b2Joint_GetAngularSeparation(jointId));
}

function Scene_WorldStep(scene: Scene): void {
	let timeStep: f32 = <f32>1.0 / <f32>60.0;
	b2World_Step(scene.worldId, timeStep, SCENE_SUBSTEP_COUNT);
}

// A ground box and a 10 box base pyramid of unit boxes, sleeping enabled
function CreatePyramid(scene: Scene): void {
	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(0.0, -1.0);
		let groundId = Scene_AddBody(scene, bodyDef);

		let box = b2MakeBox(20.0, 1.0);
		let shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape(groundId, shapeDef, box);
	}

	let baseCount = 10;

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;

	let a: f32 = 0.5;
	let box = b2MakeSquare(a);

	let shift: f32 = 1.0 * a;

	for (let i = 0; i < baseCount; ++i) {
		let y: f32 = (2.0 * <f32>i + 1.0) * shift;

		for (let j = i; j < baseCount; ++j) {
			let x: f32 = (<f32>i + 1.0) * shift + 2.0 * <f32>(j - i) * shift - a * <f32>baseCount;

			bodyDef.position = b2Vec2(x, y);

			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);
		}
	}
}

// shape_rain: a walled bin of segments with two ramps, a spinning kinematic paddle and a sliding
// kinematic platform, and a rain of every convex shape kind with random materials.
// keyBodies: 0 paddle, 1 platform
function CreateShapeRain(scene: Scene): void {
	scene.seed = 7919;

	let bodyDef = b2DefaultBodyDef();
	let groundId = Scene_AddBody(scene, bodyDef);
	scene.groundId = groundId;

	let shapeDef = b2DefaultShapeDef();

	let segment = b2Segment(b2Vec2(-30.0, 0.0), b2Vec2(30.0, 0.0));
	b2CreateSegmentShape(groundId, shapeDef, segment);

	segment = b2Segment(b2Vec2(-30.0, 0.0), b2Vec2(-30.0, 30.0));
	b2CreateSegmentShape(groundId, shapeDef, segment);

	segment = b2Segment(b2Vec2(30.0, 0.0), b2Vec2(30.0, 30.0));
	b2CreateSegmentShape(groundId, shapeDef, segment);

	shapeDef.material.friction = 0.3;
	segment = b2Segment(b2Vec2(-24.0, 14.0), b2Vec2(-6.0, 9.0));
	b2CreateSegmentShape(groundId, shapeDef, segment);

	segment = b2Segment(b2Vec2(6.0, 12.0), b2Vec2(24.0, 16.0));
	b2CreateSegmentShape(groundId, shapeDef, segment);

	{
		let paddleDef = b2DefaultBodyDef();
		paddleDef.type = b2_kinematicBody;
		paddleDef.position = b2Vec2(0.0, 5.0);
		paddleDef.angularVelocity = 1.5;
		let paddleId = Scene_AddBody(scene, paddleDef);
		scene.keyBodies[0] = paddleId;

		let paddleShapeDef = b2DefaultShapeDef();
		let box = b2MakeBox(3.0, 0.2);
		b2CreatePolygonShape(paddleId, paddleShapeDef, box);
	}

	{
		let platformDef = b2DefaultBodyDef();
		platformDef.type = b2_kinematicBody;
		platformDef.position = b2Vec2(-18.0, 3.0);
		platformDef.linearVelocity = b2Vec2(2.0, 0.0);
		let platformId = Scene_AddBody(scene, platformDef);
		scene.keyBodies[1] = platformId;

		let platformShapeDef = b2DefaultShapeDef();
		let box = b2MakeRoundedBox(2.0, 0.25, 0.1);
		b2CreatePolygonShape(platformId, platformShapeDef, box);
	}
}

// One falling body, its kind cycling through circle, capsule, box, hull, rounded hull and a two
// polygon compound
function Scene_SpawnRainBody(scene: Scene): void {
	let kind = scene.spawnCount % 6;

	let x: f32 = Scene_RandomFloat(scene, -25.0, 25.0);
	let angle: f32 = Scene_RandomFloat(scene, -B2_PI, B2_PI);
	let friction: f32 = Scene_RandomFloat(scene, 0.1, 0.9);
	let restitution: f32 = Scene_RandomFloat(scene, 0.0, 0.5);
	let density: f32 = Scene_RandomFloat(scene, 0.5, 2.0);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(x, 26.0);
	bodyDef.rotation = b2MakeRot(angle);
	let bodyId = Scene_AddBody(scene, bodyDef);
	Scene_AddSpawn(scene, bodyId);

	let shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = friction;
	shapeDef.material.restitution = restitution;
	shapeDef.density = density;

	if (kind == 0) {
		let radius: f32 = Scene_RandomFloat(scene, 0.2, 0.5);
		shapeDef.material.rollingResistance = 0.1;
		let circle = b2Circle(b2Vec2(0.0, 0.0), radius);
		b2CreateCircleShape(bodyId, shapeDef, circle);
	} else if (kind == 1) {
		let halfLength: f32 = Scene_RandomFloat(scene, 0.2, 0.6);
		shapeDef.material.rollingResistance = 0.05;
		let capsule = b2Capsule(b2Vec2(-halfLength, 0.0), b2Vec2(halfLength, 0.0), 0.25);
		b2CreateCapsuleShape(bodyId, shapeDef, capsule);
	} else if (kind == 2) {
		let hx: f32 = Scene_RandomFloat(scene, 0.2, 0.6);
		let hy: f32 = Scene_RandomFloat(scene, 0.2, 0.6);
		let box = b2MakeBox(hx, hy);
		b2CreatePolygonShape(bodyId, shapeDef, box);
	} else if (kind == 3 || kind == 4) {
		let points = new StaticArray<b2Vec2>(8);
		let count = kind == 3 ? 6 : 5;
		for (let i = 0; i < count; ++i) {
			let px: f32 = Scene_RandomFloat(scene, -0.6, 0.6);
			let py: f32 = Scene_RandomFloat(scene, -0.6, 0.6);
			points[i] = b2Vec2(px, py);
		}

		let radius: f32 = kind == 3 ? <f32>0.0 : <f32>0.1;
		let hull = b2ComputeHull(points, count);
		let polygon = b2Polygon();
		if (hull.count == 0) {
			polygon = b2MakeRoundedBox(0.4, 0.4, radius);
		} else {
			polygon = b2MakePolygon(hull, radius);
		}
		b2CreatePolygonShape(bodyId, shapeDef, polygon);
	} else {
		let box1 = b2MakeOffsetBox(0.5, 0.15, b2Vec2(0.2, 0.1), b2MakeRot(0.3));
		b2CreatePolygonShape(bodyId, shapeDef, box1);

		let box2 = b2MakeOffsetRoundedBox(0.15, 0.45, b2Vec2(-0.35, 0.0), b2Rot_identity, 0.05);
		b2CreatePolygonShape(bodyId, shapeDef, box2);
	}
}

function StepShapeRain(scene: Scene, stepIndex: i32): void {
	if (stepIndex < 300 && stepIndex % 5 == 0) {
		Scene_SpawnRainBody(scene);
		Scene_SpawnRainBody(scene);
	}

	if (stepIndex % 120 == 60 && stepIndex < 420) {
		let v = b2Body_GetLinearVelocity(scene.keyBodies[1]);
		b2Body_SetLinearVelocity(scene.keyBodies[1], b2Neg(v));
	}

	if (stepIndex == 420) {
		b2Body_SetAngularVelocity(scene.keyBodies[0], 0.0);
		b2Body_SetLinearVelocity(scene.keyBodies[1], b2Vec2_zero);
	}

	Scene_WorldStep(scene);
}

// chain_car terrain: flat up to x = 0, then two sines with an amplitude that grows with x
function Scene_TerrainHeight(x: f32): f32 {
	if (x <= 0.0) {
		return 0.0;
	}

	let amplitude: f32 = b2MinFloat(0.04 * x, 2.0);
	let cs1 = b2ComputeCosSin(0.08 * x);
	let cs2 = b2ComputeCosSin(0.27 * x);
	let wave: f32 = 1.5 * cs1.sine + 0.6 * cs2.sine;
	return amplitude * wave;
}

// samples/car.cpp Car::Spawn
function Scene_SpawnCar(
	scene: Scene,
	car: SceneVehicle,
	position: b2Pos,
	scale: f32,
	hertz: f32,
	dampingRatio: f32,
	torque: f32,
): void {
	let worldId = scene.worldId;

	let vertices = new StaticArray<b2Vec2>(6);
	vertices[0] = b2Vec2(-1.5, -0.5);
	vertices[1] = b2Vec2(1.5, -0.5);
	vertices[2] = b2Vec2(1.5, 0.0);
	vertices[3] = b2Vec2(0.0, 0.9);
	vertices[4] = b2Vec2(-1.15, 0.9);
	vertices[5] = b2Vec2(-1.5, 0.2);

	for (let i = 0; i < 6; ++i) {
		let s: f32 = 0.85 * scale;
		vertices[i].x = vertices[i].x * s;
		vertices[i].y = vertices[i].y * s;
	}

	let hull = b2ComputeHull(vertices, 6);
	let chassis = b2MakePolygon(hull, 0.15 * scale);

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = <f32>1.0 / scale;
	shapeDef.material.friction = 0.2;
	shapeDef.filter.groupIndex = -1;

	let circle = b2Circle(b2Vec2(0.0, 0.0), 0.4 * scale);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2OffsetPos(position, b2Vec2(0.0, 1.0 * scale));
	car.chassisId = Scene_AddBody(scene, bodyDef);
	b2CreatePolygonShape(car.chassisId, shapeDef, chassis);

	shapeDef.density = <f32>2.0 / scale;
	shapeDef.material.friction = 1.5;
	shapeDef.material.rollingResistance = 0.1;

	bodyDef.position = b2OffsetPos(position, b2Vec2(-1.0 * scale, 0.35 * scale));
	bodyDef.allowFastRotation = true;
	car.rearWheelId = Scene_AddBody(scene, bodyDef);
	b2CreateCircleShape(car.rearWheelId, shapeDef, circle);

	bodyDef.position = b2OffsetPos(position, b2Vec2(1.0 * scale, 0.4 * scale));
	bodyDef.allowFastRotation = true;
	car.frontWheelId = Scene_AddBody(scene, bodyDef);
	b2CreateCircleShape(car.frontWheelId, shapeDef, circle);

	let pivot = b2Body_GetPosition(car.rearWheelId);

	let jointDef = b2DefaultWheelJointDef();

	jointDef.base.bodyIdA = car.chassisId;
	jointDef.base.bodyIdB = car.rearWheelId;
	jointDef.base.localFrameA.q = b2MakeRot(0.5 * B2_PI);
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
	jointDef.motorSpeed = 0.0;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25 * scale;
	jointDef.upperTranslation = 0.25 * scale;
	jointDef.enableLimit = true;
	car.rearAxleId = b2CreateWheelJoint(worldId, jointDef);

	pivot = b2Body_GetPosition(car.frontWheelId);
	jointDef.base.bodyIdA = car.chassisId;
	jointDef.base.bodyIdB = car.frontWheelId;
	jointDef.base.localFrameA.q = b2MakeRot(0.5 * B2_PI);
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
	jointDef.motorSpeed = 0.0;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25 * scale;
	jointDef.upperTranslation = 0.25 * scale;
	jointDef.enableLimit = true;
	car.frontAxleId = b2CreateWheelJoint(worldId, jointDef);
}

// samples/car.cpp Car::SetSpeed
function Scene_SetCarSpeed(car: SceneVehicle, speed: f32): void {
	b2WheelJoint_SetTargetMotorSpeed(car.rearAxleId, speed);
	b2WheelJoint_SetTargetMotorSpeed(car.frontAxleId, speed);
	b2Joint_WakeBodies(car.rearAxleId);
}

// A bike on the Car pattern: one frame with a rider block, two wheels on wheel joints, the rear
// one driven
function Scene_SpawnBike(scene: Scene, bike: SceneVehicle, position: b2Pos): void {
	let worldId = scene.worldId;

	let vertices = new StaticArray<b2Vec2>(5);
	vertices[0] = b2Vec2(-0.9, -0.15);
	vertices[1] = b2Vec2(0.9, -0.15);
	vertices[2] = b2Vec2(0.75, 0.25);
	vertices[3] = b2Vec2(-0.2, 0.45);
	vertices[4] = b2Vec2(-0.8, 0.2);

	let hull = b2ComputeHull(vertices, 5);
	let frame = b2MakePolygon(hull, 0.05);

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;
	shapeDef.material.friction = 0.3;
	shapeDef.filter.groupIndex = -1;

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2OffsetPos(position, b2Vec2(0.0, 0.75));
	bike.chassisId = Scene_AddBody(scene, bodyDef);
	b2CreatePolygonShape(bike.chassisId, shapeDef, frame);

	let rider = b2MakeOffsetBox(0.15, 0.35, b2Vec2(-0.15, 0.7), b2MakeRot(-0.25));
	shapeDef.density = 0.8;
	b2CreatePolygonShape(bike.chassisId, shapeDef, rider);

	shapeDef.density = 1.5;
	shapeDef.material.friction = 1.2;
	shapeDef.material.rollingResistance = 0.05;

	let circle = b2Circle(b2Vec2(0.0, 0.0), 0.35);

	bodyDef.position = b2OffsetPos(position, b2Vec2(-0.85, 0.35));
	bodyDef.allowFastRotation = true;
	bike.rearWheelId = Scene_AddBody(scene, bodyDef);
	b2CreateCircleShape(bike.rearWheelId, shapeDef, circle);

	bodyDef.position = b2OffsetPos(position, b2Vec2(0.85, 0.35));
	bike.frontWheelId = Scene_AddBody(scene, bodyDef);
	b2CreateCircleShape(bike.frontWheelId, shapeDef, circle);

	let pivot = b2Body_GetPosition(bike.rearWheelId);

	let jointDef = b2DefaultWheelJointDef();
	jointDef.base.bodyIdA = bike.chassisId;
	jointDef.base.bodyIdB = bike.rearWheelId;
	jointDef.base.localFrameA.q = b2MakeRot(0.5 * B2_PI);
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
	jointDef.motorSpeed = 0.0;
	jointDef.maxMotorTorque = 6.0;
	jointDef.enableMotor = true;
	jointDef.hertz = 5.0;
	jointDef.dampingRatio = 0.7;
	jointDef.lowerTranslation = -0.2;
	jointDef.upperTranslation = 0.2;
	jointDef.enableLimit = true;
	bike.rearAxleId = b2CreateWheelJoint(worldId, jointDef);

	pivot = b2Body_GetPosition(bike.frontWheelId);
	jointDef.base.bodyIdB = bike.frontWheelId;
	jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
	jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
	jointDef.enableMotor = false;
	jointDef.hertz = 4.0;
	bike.frontAxleId = b2CreateWheelJoint(worldId, jointDef);
}

// chain_car: samples/sample_joints.cpp Driving's car on a smooth open chain with a loop chain rock,
// and a bike behind it on the same chain
function CreateChainCar(scene: Scene): void {
	let bodyDef = b2DefaultBodyDef();
	let groundId = Scene_AddBody(scene, bodyDef);
	scene.groundId = groundId;

	{
		// Right to left so the solid side faces up
		let points = new StaticArray<b2Vec2>(171);
		for (let i = 0; i < 171; ++i) {
			let x: f32 = 300.0 - 2.0 * <f32>i;
			let y: f32 = Scene_TerrainHeight(x);
			points[i] = b2Vec2(x, y);
		}

		let ghostX: f32 = 302.0;
		let ghostY: f32 = Scene_TerrainHeight(ghostX);

		let materials = new StaticArray<b2SurfaceMaterial>(1);
		materials[0] = b2DefaultSurfaceMaterial();
		materials[0].friction = 0.8;

		let chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 171;
		chainDef.ghost1 = b2Vec2(ghostX, ghostY);
		chainDef.ghost2 = b2Vec2(-42.0, 0.0);
		chainDef.materials = materials;
		chainDef.materialCount = 1;
		b2CreateChain(groundId, chainDef);
	}

	{
		// Counter-clockwise so the solid side faces out
		let x: f32 = 40.0;
		let y: f32 = Scene_TerrainHeight(x);
		let rockDef = b2DefaultBodyDef();
		rockDef.position = b2Vec2(x, y);
		let rockId = Scene_AddBody(scene, rockDef);

		let points = new StaticArray<b2Vec2>(8);
		for (let i = 0; i < 8; ++i) {
			let angle: f32 = 0.25 * B2_PI * <f32>i;
			let cs = b2ComputeCosSin(angle);
			points[i] = b2Vec2(2.5 * cs.cosine, 0.5 * cs.sine);
		}

		let chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 8;
		chainDef.isLoop = true;
		b2CreateChain(rockId, chainDef);
	}

	Scene_SpawnCar(scene, scene.car, b2Vec2(0.0, 0.0), 1.0, 5.0, 0.7, 5.0);
	Scene_SpawnBike(scene, scene.bike, b2Vec2(-12.0, 0.0));
}

function Probe_Vehicle(scene: Scene, vehicle: SceneVehicle): void {
	Probe_Float(scene, b2WheelJoint_GetCurrentMotorTorque(vehicle.rearAxleId));
	Probe_Joint(scene, vehicle.rearAxleId);
	Probe_Float(scene, b2WheelJoint_GetCurrentMotorTorque(vehicle.frontAxleId));
	Probe_Joint(scene, vehicle.frontAxleId);

	let count = b2Body_GetContactData(vehicle.rearWheelId, s_contactData, SCENE_CONTACT_CAPACITY);
	Probe_Int(scene, count);
	for (let i = 0; i < count; ++i) {
		Probe_ContactData(scene, s_contactData[i]);
	}
}

function StepChainCar(scene: Scene, stepIndex: i32): void {
	let car = scene.car;
	let bike = scene.bike;

	// Negative motor speed drives to the right
	if (stepIndex == 30) {
		Scene_SetCarSpeed(car, -35.0);
	} else if (stepIndex == 330) {
		Scene_SetCarSpeed(car, 0.0);
	} else if (stepIndex == 420) {
		Scene_SetCarSpeed(car, 20.0);
	} else if (stepIndex == 540) {
		b2WheelJoint_SetMaxMotorTorque(car.rearAxleId, 8.0);
		b2WheelJoint_SetMaxMotorTorque(car.frontAxleId, 8.0);
		b2WheelJoint_SetSpringHertz(car.rearAxleId, 3.0);
		b2WheelJoint_SetSpringHertz(car.frontAxleId, 3.0);
		b2WheelJoint_SetSpringDampingRatio(car.rearAxleId, 1.0);
		b2WheelJoint_SetSpringDampingRatio(car.frontAxleId, 1.0);
		Scene_SetCarSpeed(car, -40.0);
	}

	if (stepIndex == 40) {
		b2WheelJoint_SetTargetMotorSpeed(bike.rearAxleId, -30.0);
		b2Joint_WakeBodies(bike.rearAxleId);
	} else if (stepIndex == 400) {
		b2WheelJoint_SetMaxMotorTorque(bike.rearAxleId, 10.0);
		b2WheelJoint_SetTargetMotorSpeed(bike.rearAxleId, 0.0);
	} else if (stepIndex == 460) {
		b2WheelJoint_SetMaxMotorTorque(bike.rearAxleId, 6.0);
		b2WheelJoint_SetTargetMotorSpeed(bike.rearAxleId, -40.0);
	} else if (stepIndex == 600) {
		b2WheelJoint_SetSpringHertz(bike.frontAxleId, 8.0);
		b2WheelJoint_EnableLimit(bike.rearAxleId, false);
	}

	Scene_WorldStep(scene);

	Probe_Vehicle(scene, car);
	Probe_Vehicle(scene, bike);
}

// shared/human.c CreateHuman, one table row per bone in the order the C creates them. Colors, names
// and user data are dropped.
struct SceneBoneDef {
	parentIndex: i32 = 0;
	positionY: f32 = 0;
	linearDamping: f32 = 0;
	frictionScale: f32 = 0;
	capsuleY1: f32 = 0;
	capsuleY2: f32 = 0;
	capsuleRadius: f32 = 0;
	pivotY: f32 = 0;
	lowerAngle: f32 = 0;
	upperAngle: f32 = 0;
	foot: bool = false;
	rotateFrameA: bool = false;

	constructor(
		parentIndex: i32,
		positionY: f32,
		linearDamping: f32,
		frictionScale: f32,
		capsuleY1: f32,
		capsuleY2: f32,
		capsuleRadius: f32,
		pivotY: f32,
		lowerAngle: f32,
		upperAngle: f32,
		foot: bool,
		rotateFrameA: bool,
	) {
		this.parentIndex = parentIndex;
		this.positionY = positionY;
		this.linearDamping = linearDamping;
		this.frictionScale = frictionScale;
		this.capsuleY1 = capsuleY1;
		this.capsuleY2 = capsuleY2;
		this.capsuleRadius = capsuleRadius;
		this.pivotY = pivotY;
		this.lowerAngle = lowerAngle;
		this.upperAngle = upperAngle;
		this.foot = foot;
		this.rotateFrameA = rotateFrameA;
	}
}

function Scene_MakeBoneDefs(): StaticArray<SceneBoneDef> {
	let defs = new StaticArray<SceneBoneDef>(SCENE_BONE_COUNT);
	// hip
	defs[0] = SceneBoneDef(-1, 0.95, 0.0, 1.0, -0.02, 0.02, 0.095, 0.0, 0.0, 0.0, false, false);
	// torso
	defs[1] = SceneBoneDef(0, 1.2, 0.0, 0.5, -0.135, 0.135, 0.09, 1.0, -0.25, 0.0, false, false);
	// head
	defs[2] = SceneBoneDef(1, 1.475, 0.1, 0.25, -0.038, 0.039, 0.075, 1.4, -0.3, 0.1, false, false);
	// upper left leg
	defs[3] = SceneBoneDef(0, 0.775, 0.0, 1.0, -0.125, 0.125, 0.06, 0.9, -0.05, 0.4, false, false);
	// lower left leg
	defs[4] = SceneBoneDef(3, 0.475, 0.0, 0.5, -0.155, 0.125, 0.045, 0.625, -0.5, -0.02, true, false);
	// upper right leg
	defs[5] = SceneBoneDef(0, 0.775, 0.0, 1.0, -0.125, 0.125, 0.06, 0.9, -0.05, 0.4, false, false);
	// lower right leg
	defs[6] = SceneBoneDef(5, 0.475, 0.0, 0.5, -0.155, 0.125, 0.045, 0.625, -0.5, -0.02, true, false);
	// upper left arm
	defs[7] = SceneBoneDef(1, 1.225, 0.0, 0.5, -0.125, 0.125, 0.035, 1.35, -0.1, 0.8, false, false);
	// lower left arm
	defs[8] = SceneBoneDef(7, 0.975, 0.1, 0.1, -0.125, 0.125, 0.03, 1.1, -0.2, 0.3, false, true);
	// upper right arm
	defs[9] = SceneBoneDef(1, 1.225, 0.0, 0.5, -0.125, 0.125, 0.035, 1.35, -0.1, 0.8, false, false);
	// lower right arm
	defs[10] = SceneBoneDef(9, 0.975, 0.1, 0.1, -0.125, 0.125, 0.03, 1.1, -0.2, 0.3, false, true);
	return defs;
}

const s_boneDefs: StaticArray<SceneBoneDef> = Scene_MakeBoneDefs();

// Returns the torso body
// boneSlot >= 0 keeps the bone bodies and joints in scene.rain from that slot, for Scene_DestroyHuman
function Scene_CreateHuman(
	scene: Scene,
	position: b2Pos,
	scale: f32,
	frictionTorque: f32,
	hertz: f32,
	dampingRatio: f32,
	groupIndex: i32,
	boneSlot: i32,
): b2BodyId {
	let worldId = scene.worldId;
	let bodyIds = new StaticArray<b2BodyId>(SCENE_BONE_COUNT);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.sleepThreshold = 0.1;

	let shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.2;
	shapeDef.filter.groupIndex = -groupIndex;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = <u64>(1 | 2);

	let footShapeDef = shapeDef;
	footShapeDef.material.friction = 0.05;

	// feet don't collide with ragdolls
	footShapeDef.filter.categoryBits = 2;
	footShapeDef.filter.maskBits = 1;

	let s: f32 = scale;
	let maxTorque: f32 = frictionTorque * s;
	let enableMotor = true;
	let enableLimit = true;
	let drawSize: f32 = 0.05;

	let points = new StaticArray<b2Vec2>(4);
	points[0] = b2Vec2(-0.03 * s, -0.185 * s);
	points[1] = b2Vec2(0.11 * s, -0.185 * s);
	points[2] = b2Vec2(0.11 * s, -0.16 * s);
	points[3] = b2Vec2(-0.03 * s, -0.14 * s);

	let footHull = b2ComputeHull(points, 4);
	let footPolygon = b2MakePolygon(footHull, 0.015 * s);

	for (let i = 0; i < SCENE_BONE_COUNT; ++i) {
		let def = s_boneDefs[i];

		bodyDef.position = b2OffsetPos(position, b2Vec2(0.0, def.positionY * s));
		bodyDef.linearDamping = def.linearDamping;
		bodyIds[i] = Scene_AddBody(scene, bodyDef);
		if (boneSlot >= 0) {
			scene.rain.boneBodies[boneSlot + i] = bodyIds[i];
			scene.rain.boneJoints[boneSlot + i] = b2_nullJointId;
		}

		let capsule = b2Capsule(b2Vec2(0.0, def.capsuleY1 * s), b2Vec2(0.0, def.capsuleY2 * s), def.capsuleRadius * s);
		b2CreateCapsuleShape(bodyIds[i], shapeDef, capsule);

		if (def.foot) {
			b2CreatePolygonShape(bodyIds[i], footShapeDef, footPolygon);
		}

		if (def.parentIndex < 0) {
			continue;
		}

		let pivot = b2OffsetPos(position, b2Vec2(0.0, def.pivotY * s));
		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = bodyIds[def.parentIndex];
		jointDef.base.bodyIdB = bodyIds[i];
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
		if (def.rotateFrameA) {
			jointDef.base.localFrameA.q = b2MakeRot(0.25 * B2_PI);
		}
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = def.lowerAngle * B2_PI;
		jointDef.upperAngle = def.upperAngle * B2_PI;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = def.frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.base.drawScale = drawSize;

		let jointId = b2CreateRevoluteJoint(worldId, jointDef);
		if (boneSlot >= 0) {
			scene.rain.boneJoints[boneSlot + i] = jointId;
		}
	}

	return bodyIds[1];
}

// joints: one rig per joint type along a floor, from samples/sample_joints.cpp, plus a ragdoll.
// keyJoints: 0 distance, 1 motor, 2 filter, 3 prismatic, 4 revolute, 5 wheel, 6..9 rigid welds,
// 10..13 soft welds. keyBodies: 0 motor body, 1 ragdoll torso, 2 rigid cantilever tip, 3 second
// filtered box. Joint user data is the key slot plus one.
function CreateJoints(scene: Scene): void {
	let worldId = scene.worldId;

	let groundDef = b2DefaultBodyDef();
	let groundId = Scene_AddBody(scene, groundDef);
	scene.groundId = groundId;

	{
		let shapeDef = b2DefaultShapeDef();
		let segment = b2Segment(b2Vec2(-40.0, 0.0), b2Vec2(50.0, 0.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);
	}

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	let shapeDef = b2DefaultShapeDef();

	// Distance joint with spring, limits and motor
	{
		bodyDef.position = b2Vec2(-30.0, 6.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let box = b2MakeSquare(0.25);
		b2CreatePolygonShape(bodyId, shapeDef, box);

		let jointDef = b2DefaultDistanceJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Vec2(-30.0, 10.0);
		jointDef.base.localFrameB.p = b2Vec2(0.0, 0.25);
		jointDef.base.forceThreshold = 12.0;
		jointDef.base.userData = 1;
		jointDef.length = 4.0;
		jointDef.enableSpring = true;
		jointDef.hertz = 2.0;
		jointDef.dampingRatio = 0.3;
		jointDef.enableLimit = true;
		jointDef.minLength = 3.0;
		jointDef.maxLength = 5.0;
		jointDef.enableMotor = true;
		jointDef.maxMotorForce = 20.0;
		jointDef.motorSpeed = 1.0;
		scene.keyJoints[0] = b2CreateDistanceJoint(worldId, jointDef);
	}

	// Motor joint, velocity driven then spring driven by the script
	{
		bodyDef.position = b2Vec2(-20.0, 3.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[0] = bodyId;
		let box = b2MakeBox(0.75, 0.25);
		b2CreatePolygonShape(bodyId, shapeDef, box);

		let jointDef = b2DefaultMotorJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Vec2(-20.0, 3.0);
		jointDef.base.userData = 2;
		jointDef.maxVelocityForce = 500.0;
		jointDef.maxVelocityTorque = 100.0;
		scene.keyJoints[1] = b2CreateMotorJoint(worldId, jointDef);
	}

	// Filter joint between two overlapping boxes, destroyed by the script
	{
		bodyDef.position = b2Vec2(-12.0, 0.5);
		let bodyIdA = Scene_AddBody(scene, bodyDef);
		let box = b2MakeSquare(0.5);
		b2CreatePolygonShape(bodyIdA, shapeDef, box);

		bodyDef.position = b2Vec2(-11.5, 1.2);
		let bodyIdB = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[3] = bodyIdB;
		b2CreatePolygonShape(bodyIdB, shapeDef, box);

		let jointDef = b2DefaultFilterJointDef();
		jointDef.base.bodyIdA = bodyIdA;
		jointDef.base.bodyIdB = bodyIdB;
		jointDef.base.userData = 3;
		scene.keyJoints[2] = b2CreateFilterJoint(worldId, jointDef);
	}

	// Prismatic joint on a diagonal axis with limit, motor and spring
	{
		bodyDef.position = b2Vec2(-5.0, 4.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let box = b2MakeBox(0.5, 0.25);
		b2CreatePolygonShape(bodyId, shapeDef, box);

		let jointDef = b2DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Vec2(-5.0, 4.0);
		jointDef.base.localFrameA.q = b2MakeRot(0.25 * B2_PI);
		jointDef.base.userData = 4;
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -2.0;
		jointDef.upperTranslation = 2.0;
		jointDef.enableMotor = true;
		jointDef.maxMotorForce = 50.0;
		jointDef.motorSpeed = 1.0;
		jointDef.enableSpring = true;
		jointDef.hertz = 1.0;
		jointDef.dampingRatio = 0.5;
		scene.keyJoints[3] = b2CreatePrismaticJoint(worldId, jointDef);
	}

	// Revolute joint with limit, motor and spring
	{
		bodyDef.position = b2Vec2(4.0, 5.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let box = b2MakeBox(1.0, 0.1);
		b2CreatePolygonShape(bodyId, shapeDef, box);

		let pivot = b2Vec2(3.0, 5.0);
		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint(groundId, pivot);
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint(bodyId, pivot);
		jointDef.base.torqueThreshold = 8.0;
		jointDef.base.userData = 5;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.5 * B2_PI;
		jointDef.upperAngle = 0.75 * B2_PI;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 50.0;
		jointDef.motorSpeed = 1.5;
		jointDef.enableSpring = true;
		jointDef.hertz = 1.0;
		jointDef.dampingRatio = 0.5;
		scene.keyJoints[4] = b2CreateRevoluteJoint(worldId, jointDef);
	}

	// Wheel joint with limit, motor and spring
	{
		bodyDef.position = b2Vec2(24.0, 3.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let circle = b2Circle(b2Vec2(0.0, 0.0), 0.5);
		b2CreateCircleShape(bodyId, shapeDef, circle);

		let jointDef = b2DefaultWheelJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Vec2(24.0, 3.0);
		jointDef.base.localFrameA.q = b2MakeRot(0.5 * B2_PI);
		jointDef.base.forceThreshold = 30.0;
		jointDef.base.userData = 6;
		jointDef.enableSpring = true;
		jointDef.hertz = 2.0;
		jointDef.dampingRatio = 0.5;
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -0.5;
		jointDef.upperTranslation = 0.5;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 10.0;
		jointDef.motorSpeed = 4.0;
		scene.keyJoints[5] = b2CreateWheelJoint(worldId, jointDef);
	}

	// Two weld cantilevers from samples/sample_joints.cpp Cantilever, rigid then soft, each with a
	// box dropped on its tip
	for (let k = 0; k < 2; ++k) {
		let x0: f32 = k == 0 ? <f32>10.0 : <f32>17.0;
		let hx: f32 = 0.5;
		let box = b2MakeBox(hx, 0.125);

		let jointDef = b2DefaultWeldJointDef();
		if (k == 1) {
			jointDef.linearHertz = 5.0;
			jointDef.angularHertz = 3.0;
			jointDef.linearDampingRatio = 0.5;
			jointDef.angularDampingRatio = 0.5;
			jointDef.base.torqueThreshold = 3.0;
		}

		let prevBodyId = groundId;
		for (let i = 0; i < 4; ++i) {
			bodyDef.position = b2Vec2(x0 + hx + 2.0 * hx * <f32>i, 4.0);
			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);

			let pivot = b2Vec2(x0 + 2.0 * hx * <f32>i, 4.0);
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
			let slot = 6 + 4 * k + i;
			jointDef.base.userData = <u64>(slot + 1);
			scene.keyJoints[slot] = b2CreateWeldJoint(worldId, jointDef);

			prevBodyId = bodyId;
		}

		if (k == 0) {
			scene.keyBodies[2] = prevBodyId;
		}

		bodyDef.position = b2Vec2(x0 + 3.5, 7.0);
		let dropId = Scene_AddBody(scene, bodyDef);
		let dropBox = b2MakeSquare(0.3);
		b2CreatePolygonShape(dropId, shapeDef, dropBox);
	}

	scene.keyJointCount = 14;

	// samples/sample_joints.cpp Ragdoll
	scene.keyBodies[1] = Scene_CreateHuman(scene, b2Vec2(34.0, 5.0), 1.0, 0.03, 5.0, 0.5, 1, -1);
}

function StepJoints(scene: Scene, stepIndex: i32): void {
	let joints = scene.keyJoints;
	let timeStep: f32 = <f32>1.0 / <f32>60.0;

	if (stepIndex == 100) {
		b2DistanceJoint_SetRestLength(joints[0], 2.5);
	} else if (stepIndex == 160) {
		b2DistanceJoint_SetLimits(joints[0], 2.0, 3.5);
		b2DistanceJoint_SetTargetMotorSpeed(joints[0], -2.0);
	} else if (stepIndex == 250) {
		b2DistanceJoint_EnableSpring(joints[0], false);
	}

	if (stepIndex < 200) {
		let cs1 = b2ComputeCosSin(scene.time);
		let angle2: f32 = 2.0 * scene.time;
		let cs2 = b2ComputeCosSin(angle2);
		let v = b2Vec2(2.0 * cs1.cosine, 1.5 * cs2.sine);
		b2MotorJoint_SetLinearVelocity(joints[1], v);
		b2MotorJoint_SetAngularVelocity(joints[1], 1.0);
	} else if (stepIndex == 200) {
		b2MotorJoint_SetLinearVelocity(joints[1], b2Vec2_zero);
		b2MotorJoint_SetAngularVelocity(joints[1], 0.0);
		b2MotorJoint_SetLinearHertz(joints[1], 2.0);
		b2MotorJoint_SetLinearDampingRatio(joints[1], 0.7);
		b2MotorJoint_SetMaxSpringForce(joints[1], 200.0);
		b2MotorJoint_SetAngularHertz(joints[1], 1.0);
		b2MotorJoint_SetAngularDampingRatio(joints[1], 0.5);
		b2MotorJoint_SetMaxSpringTorque(joints[1], 50.0);
	}

	if (stepIndex == 200) {
		b2DestroyJoint(joints[2]);
	} else if (stepIndex == 205) {
		// Moving the proxy makes the broad phase pair the boxes again
		b2Body_SetTransform(scene.keyBodies[3], b2Vec2(-11.6, 0.6), b2Rot_identity);
	}

	if (stepIndex % 100 == 50) {
		let speed: f32 = b2PrismaticJoint_GetTargetMotorSpeed(joints[3]);
		b2PrismaticJoint_SetTargetMotorSpeed(joints[3], -speed);
	}
	if (stepIndex == 250) {
		b2PrismaticJoint_SetTargetTranslation(joints[3], 1.0);
	} else if (stepIndex == 300) {
		b2PrismaticJoint_EnableMotor(joints[3], false);
	} else if (stepIndex == 350) {
		b2PrismaticJoint_SetLimits(joints[3], -1.0, 1.0);
	}

	if (stepIndex == 120) {
		b2RevoluteJoint_SetTargetMotorSpeed(joints[4], -2.0);
	} else if (stepIndex == 200) {
		b2RevoluteJoint_SetLimits(joints[4], -0.25 * B2_PI, 0.25 * B2_PI);
	} else if (stepIndex == 260) {
		b2RevoluteJoint_SetTargetAngle(joints[4], 0.5);
	} else if (stepIndex == 320) {
		b2RevoluteJoint_EnableMotor(joints[4], false);
		b2RevoluteJoint_SetSpringHertz(joints[4], 3.0);
	}

	if (stepIndex == 150) {
		b2WheelJoint_SetTargetMotorSpeed(joints[5], -4.0);
	} else if (stepIndex == 250) {
		b2WheelJoint_SetSpringHertz(joints[5], 5.0);
	} else if (stepIndex == 300) {
		b2WheelJoint_EnableLimit(joints[5], false);
	}

	if (stepIndex == 200) {
		for (let i = 10; i < 14; ++i) {
			b2WeldJoint_SetAngularHertz(joints[i], 1.0);
			b2WeldJoint_SetLinearDampingRatio(joints[i], 1.0);
		}
		b2Body_ApplyLinearImpulseToCenter(scene.keyBodies[2], b2Vec2(0.0, -4.0), true);
	}

	if (stepIndex == 100) {
		b2Body_ApplyAngularImpulse(scene.keyBodies[1], 0.5, true);
	} else if (stepIndex == 150) {
		b2Body_ApplyLinearImpulseToCenter(scene.keyBodies[1], b2Vec2(-3.0, 4.0), true);
	}

	scene.time += timeStep;

	Scene_WorldStep(scene);

	Probe_JointEvents(scene);
	for (let i = 0; i < scene.keyJointCount; ++i) {
		Probe_Joint(scene, joints[i]);
	}

	Probe_Float(scene, b2DistanceJoint_GetCurrentLength(joints[0]));
	Probe_Float(scene, b2DistanceJoint_GetSpringForce(joints[0]));
	Probe_Float(scene, b2DistanceJoint_GetCurrentMotorForce(joints[0]));
	Probe_Float(scene, b2PrismaticJoint_GetCurrentTranslation(joints[3]));
	Probe_Float(scene, b2PrismaticJoint_GetCurrentSpeed(joints[3]));
	Probe_Float(scene, b2PrismaticJoint_GetCurrentMotorForce(joints[3]));
	Probe_Float(scene, b2RevoluteJoint_GetCurrentAngle(joints[4]));
	Probe_Float(scene, b2RevoluteJoint_GetCurrentSpeed(joints[4]));
	Probe_Float(scene, b2RevoluteJoint_GetCurrentMotorTorque(joints[4]));
	Probe_Float(scene, b2WheelJoint_GetCurrentMotorTorque(joints[5]));
}

// The svg levels of samples/sample_character.cpp, run through Sample::ParsePath at scale 0.2 and
// stored as float bits, x then y
const s_geometricLevel1Bits: StaticArray<u32> = [
	0xc1870889, 0xbe5ddd9a, 0x4242f333, 0xbe5ddd9a, 0x4242f333, 0x4114eef0, 0x4240d555, 0x4114eef0,
	0x42385ddd, 0x40f71113, 0x422dc888, 0x40c44447, 0x42232fa7, 0x40a26667, 0x4209cccc, 0x405f3333,
	0x41f1bbba, 0x403d555a, 0x41cba220, 0x3f57779a, 0x41b23bba, 0x3f577767, 0x41b23bba, 0x3fd1554d,
	0x4196b775, 0x3fd15533, 0x4196b775, 0x3f8d999a, 0x41766663, 0x3f8d999a, 0x41766663, 0x3faf7780,
	0x4169b330, 0x3faf7780, 0x4169b330, 0x3fd15567, 0x415cfffd, 0x3fd15567, 0x415cfffd, 0x3ff3334d,
	0x41504cca, 0x3ff3334d, 0x41504cca, 0x400a888d, 0x41439997, 0x400a888d, 0x41439997, 0x401b7780,
	0x4136e663, 0x401b7780, 0x4136e663, 0x402c6673, 0x412a3330, 0x402c6673, 0x412a3330, 0x403d555a,
	0x411d7ffb, 0x403d555a, 0x411d7ffb, 0x404e444d, 0x4110ccc8, 0x404e444d, 0x4110ccc8, 0x405f3340,
	0x41041995, 0x405f3340, 0x41041995, 0x40702227, 0x40eeccc3, 0x40702227, 0x40eeccc3, 0x4080888d,
	0x40d5665d, 0x4080888d, 0x40d5665d, 0x40890007, 0x40bbfff7, 0x40890007, 0x40bbfff7, 0x4091777a,
	0x40a29990, 0x4091777a, 0x40a29990, 0x4099eef3, 0x4089332a, 0x4099eef3, 0x4089332a, 0x40a2666d,
	0x405f9987, 0x40a2666d, 0x405f9987, 0x40aadde0, 0x402cccba, 0x40aadde0, 0x402cccb7, 0x40b35560,
	0x3f15555a, 0x40b35560, 0xc0040000, 0x4091777a, 0xc047bbbd, 0x4091777a, 0xc096aaab, 0x405f3333,
	0xc0da6667, 0x405f3333, 0xc182cccd, 0x4158aaaa, 0xc182cccd, 0x4196aaab, 0xc1870889, 0x4196aaab,
];

const s_dynamicLevel1Bits: StaticArray<u32> = [
	0xc1870889, 0xbe5ddd9a, 0x4242f333, 0xbe5ddd9a, 0x4242f333, 0x4114eef0, 0x4240d555, 0x4114eef0,
	0x42385ddd, 0x40f71113, 0x422dc888, 0x40c44447, 0x42232fa7, 0x40a26667, 0x4209cccc, 0x405f3333,
	0x41f1bbba, 0x403d555a, 0x41cba220, 0x3e9fff9a, 0x41b23bba, 0x3e9fff33, 0x41b23bba, 0x3fd1554d,
	0x4196b775, 0x3fd15533, 0x4196b775, 0x3f8d999a, 0x41766663, 0x3f8d999a, 0x41766663, 0x3faf7780,
	0x4169b330, 0x3faf7780, 0x4169b330, 0x3fd15567, 0x415cfffd, 0x3fd15567, 0x415cfffd, 0x3ff3334d,
	0x41504cca, 0x3ff3334d, 0x41504cca, 0x400a888d, 0x41439997, 0x400a888d, 0x41439997, 0x401b7780,
	0x4136e663, 0x401b7780, 0x4136e663, 0x402c6673, 0x412a3330, 0x402c6673, 0x412a3330, 0x403d555a,
	0x411d7ffb, 0x403d555a, 0x411d7ffb, 0x404e444d, 0x4110ccc8, 0x404e444d, 0x4110ccc8, 0x405f3340,
	0x41041995, 0x405f3340, 0x41041995, 0x40702227, 0x40eeccc3, 0x40702227, 0x40eeccc3, 0x4080888d,
	0x40d5665d, 0x4080888d, 0x40d5665d, 0x40890007, 0x40bbfff7, 0x40890007, 0x40bbfff7, 0x4091777a,
	0x40a29990, 0x4091777a, 0x40a29990, 0x4099eef3, 0x4089332a, 0x4099eef3, 0x4089332a, 0x40a2666d,
	0x405f9987, 0x40a2666d, 0x405f9987, 0x40aadde0, 0x402cccba, 0x40aadde0, 0x402cccb7, 0x40b35560,
	0x3f15555a, 0x40b35560, 0xc0040000, 0x4091777a, 0xc047bbbd, 0x4091777a, 0xc096aaab, 0x405f3333,
	0xc0da6667, 0x405f3333, 0xc182cccd, 0x4158aaaa, 0xc182cccd, 0x4196aaab, 0xc1870889, 0x4196aaab,
];

const s_level2Bits: StaticArray<u32> = [
	0x3f077777, 0xbe5ddd9a, 0x429fceef, 0xbe5ddd9a, 0x429fceef, 0x4091777a, 0x42964889, 0x4091777a,
	0x429ec000, 0x3f577767, 0x42033bbb, 0x3f57779a, 0x42032ff5, 0x3ea00000, 0x41fde873, 0x3ea00000,
	0x41fde873, 0x3f577767, 0x41f1517f, 0x3f57779a, 0x41f14cd3, 0x3faf7780, 0x41ed1118, 0x3faf7780,
	0x41e8d55d, 0x3f577767, 0x41dc222a, 0x3f577767, 0x41d7e66d, 0x3ea00000, 0x41d3c202, 0x3f57779a,
	0x41c2bbbc, 0x3f577767, 0x41c2bbbc, 0x3faf7767, 0x41b60889, 0x3faf7767, 0x41b60889, 0x3ff33333,
	0x41a95556, 0x3ff33333, 0x41a95556, 0x401b7773, 0x419ca223, 0x401b7773, 0x419ca223, 0x403d554d,
	0x418feeef, 0x403d554d, 0x418feeef, 0x405f3327, 0x41833bbc, 0x405f3333, 0x41833bbc, 0x4080888d,
	0x416d1112, 0x40808887, 0x416d1112, 0x40917773, 0x4153aaab, 0x4091777a, 0x4153aaab, 0x40a26667,
	0x413a4445, 0x40a26660, 0x413a4445, 0x40b35553, 0x41295556, 0x40b35553, 0x410feeef, 0x40c44447,
	0x40ed1112, 0x40d53333, 0x40cb3333, 0x40f71113, 0x40877778, 0x40f71113, 0x40295556, 0x41040000,
	0x3f877778, 0x4114eef0, 0x3f077778, 0x4114eef0,
];

function Scene_CreateLevelChain(scene: Scene, position: b2Pos, bits: StaticArray<u32>, count: i32): b2BodyId {
	let bodyDef = b2DefaultBodyDef();
	bodyDef.position = position;
	let groundId = Scene_AddBody(scene, bodyDef);

	let points = new StaticArray<b2Vec2>(64);
	for (let i = 0; i < count; ++i) {
		let x: f32 = reinterpret<f32>(bits[2 * i]);
		let y: f32 = reinterpret<f32>(bits[2 * i + 1]);
		points[i] = b2Vec2(x, y);
	}

	let chainDef = b2DefaultChainDef();
	chainDef.points = points;
	chainDef.pointCount = count;
	chainDef.isLoop = true;
	b2CreateChain(groundId, chainDef);

	return groundId;
}

// The props both character samples place after the level: plank bridge, balls, elevator, box
// pyramid and motorized door. The geometric sample tags the elevator for the mover planes.
function Scene_CreateCharacterProps(
	scene: Scene,
	groundId1: b2BodyId,
	groundId2: b2BodyId,
	doorY: f32,
	geometric: bool,
): void {
	{
		let box = b2MakeBox(0.55, 0.125);
		let shapeDef = b2DefaultShapeDef();

		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.maxMotorTorque = 10.0;
		jointDef.enableMotor = true;
		jointDef.hertz = 3.0;
		jointDef.dampingRatio = 0.8;
		jointDef.enableSpring = true;

		let xBase: f32 = 48.7;
		let yBase: f32 = 9.2;
		let count = 50;
		let prevBodyId = groundId1;
		for (let i = 0; i < count; ++i) {
			let bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Vec2(xBase + 0.5 + 1.0 * <f32>i, yBase);
			bodyDef.angularDamping = 0.2;
			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);

			let pivot = b2Vec2(xBase + 1.0 * <f32>i, yBase);
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
			b2CreateRevoluteJoint(scene.worldId, jointDef);

			prevBodyId = bodyId;
		}

		let pivot = b2Vec2(xBase + 1.0 * <f32>count, yBase);
		jointDef.base.bodyIdA = prevBodyId;
		jointDef.base.bodyIdB = groundId2;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
		b2CreateRevoluteJoint(scene.worldId, jointDef);
	}

	for (let i = 0; i < 5; ++i) {
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(7.0, 7.0 + 0.5 * <f32>i);
		let bodyId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.filter = b2Filter(s_dynamicBit, s_allBits, 0);
		shapeDef.material.restitution = 0.7;
		shapeDef.material.rollingResistance = 0.2;

		let circle = b2Circle(b2Vec2(0.0, 0.0), 0.25);
		b2CreateCircleShape(bodyId, shapeDef, circle);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = b2Vec2(112.0, <f32>10.0 - <f32>4.0);
		let elevatorId = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[0] = elevatorId;

		let shapeDef = b2DefaultShapeDef();
		shapeDef.filter = b2Filter(s_dynamicBit, s_allBits, 0);
		if (geometric) {
			shapeDef.userData = 2;
		}

		let box = b2MakeBox(2.0, 0.1);
		b2CreatePolygonShape(elevatorId, shapeDef, box);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.x = 140.0;
		let a: f32 = 0.25;
		let square = b2MakeSquare(a);
		let shapeDef = b2DefaultShapeDef();
		shapeDef.filter = b2Filter(s_dynamicBit, s_allBits, 0);
		for (let i = 0; i < 10; ++i) {
			bodyDef.position.y = (2.0 * <f32>i + 1.0) * a + 1.0;
			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, square);
		}
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(160.0, doorY);
		bodyDef.type = b2_dynamicBody;
		let bodyId = Scene_AddBody(scene, bodyDef);

		let box = b2MakeBox(0.1, 4.0);
		let shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0;
		b2CreatePolygonShape(bodyId, shapeDef, box);

		let pivot = b2Add(bodyDef.position, b2Vec2(0.0, 4.0));
		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId2;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 1.0;
		jointDef.maxMotorTorque = 500.0;
		b2CreateRevoluteJoint(scene.worldId, jointDef);
	}
}

// The elevator script both character samples run before the step, with b2ComputeCosSin for cosf
function Scene_MoveElevator(scene: Scene, timeStep: f32): void {
	let angle: f32 = 1.0 * scene.time + B2_PI;
	let cs = b2ComputeCosSin(angle);
	let point = b2Vec2(112.0, 4.0 * cs.cosine + 10.0);
	let target = b2Transform(point, b2Rot_identity);
	let wake = true;
	b2Body_SetTargetTransform(scene.keyBodies[0], target, timeStep, wake);

	scene.time += timeStep;
}

// Scripted keys for both movers: throttle and the jump key
function Scene_MoverThrottle(stepIndex: i32): f32 {
	if (stepIndex < 60) {
		return 0.0;
	}
	if (stepIndex < 300) {
		return 1.0;
	}
	if (stepIndex < 360) {
		return -1.0;
	}
	return 1.0;
}

function Scene_MoverJumpKey(stepIndex: i32): bool {
	let phase = stepIndex % 300;
	return stepIndex >= 150 && phase >= 150 && phase < 154;
}

function Scene_ResetCastResult(result: SceneCastResult): void {
	result.point = b2Vec2(0.0, 0.0);
	result.normal = b2Vec2(0.0, 0.0);
	result.bodyId = b2BodyId();
	result.fraction = 0.0;
	result.hit = false;
}

// The cast result Scene_CastCallback fills
let s_castResult: SceneCastResult = new SceneCastResult();

function Scene_CastCallback(shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32): f32 {
	let result = s_castResult;
	result.point = point;
	result.normal = normal;
	result.bodyId = b2Shape_GetBody(shapeId);
	result.fraction = fraction;
	result.hit = true;
	return fraction;
}

// samples/dynamic_mover.cpp DynamicMover::DynamicMover and Create with the sample's def
function Scene_CreateDynamicMover(scene: Scene, m: SceneDynamicMover, position: b2Pos): void {
	m.capsule = b2Capsule(b2Vec2(0.0, -0.5), b2Vec2(0.0, 0.5), 0.3);
	m.filter = b2Filter(s_moverBit, s_staticBit | s_dynamicBit, 0);
	m.jumpSpeed = 7.0;
	m.maxSpeed = 6.0;
	m.minSpeed = 0.1;
	m.stopSpeed = 3.0;
	m.accelerate = 20.0;
	m.airSteer = 0.5;
	m.friction = 8.0;
	m.gravityScale = 1.5;
	m.maxGroundForce = 70.0;
	m.maxAirForce = 20.0;
	m.pogoRestLength = 0.9;
	m.pogoHertz = 5.0;
	m.pogoDampingRatio = 0.8;
	m.pogoCompressionScale = 100.0;
	m.pogoTensionScale = 100.0;
	m.minGroundNormalY = 0.7;
	m.velocity = b2Vec2_zero;
	m.pogoJointId = b2_nullJointId;
	m.pogoFraction = 1.0;

	let bodyDef = b2DefaultBodyDef();
	m.groundId = Scene_AddBody(scene, bodyDef);

	// Position is the center of the capsule.
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = position;
	bodyDef.gravityScale = m.gravityScale;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.enableSleep = false;
	bodyDef.name = "mover";

	m.moverId = Scene_AddBody(scene, bodyDef);

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;

	// The mover joint and the pogo joint handle all the movement, surface friction would fight them
	shapeDef.material.friction = 0.0;
	shapeDef.filter = m.filter;
	shapeDef.enablePreSolveEvents = true;

	b2CreateCapsuleShape(m.moverId, shapeDef, m.capsule);

	let moverDef = b2DefaultMoverJointDef();
	moverDef.linearVelocity = m.velocity;
	moverDef.maxVelocityForce = b2Vec2(m.maxGroundForce, 0.0);
	moverDef.base.bodyIdA = m.groundId;
	moverDef.base.bodyIdB = m.moverId;
	moverDef.base.collideConnected = true;

	m.moverJointId = b2CreateMoverJoint(scene.worldId, moverDef);
}

// samples/dynamic_mover.cpp DynamicMover::Jump
function Scene_DynamicMoverJump(m: SceneDynamicMover): bool {
	if (m.onGround == false || m.walkable == false) {
		return false;
	}

	let surfaceVelocity: f32 = 0.0;
	if (b2Body_IsValid(m.castResult.bodyId)) {
		let v = b2Body_GetWorldPointVelocity(m.castResult.bodyId, m.castResult.point);
		surfaceVelocity = v.y;
	}

	let mass: f32 = b2Body_GetMass(m.moverId);
	let vy: f32 = b2Body_GetLinearVelocity(m.moverId).y;

	// Remove the pogo constraint velocity but add the surface velocity.
	let dv: f32 = b2MaxFloat(0.0, m.jumpSpeed - vy) + surfaceVelocity;
	b2Body_ApplyLinearImpulseToCenter(m.moverId, b2Vec2(0.0, mass * dv), true);

	// This removes the pogo step down on the next update.
	m.onGround = false;
	m.walkable = false;
	m.jumping = true;
	m.jumpTicks = 0;
	return true;
}

// samples/dynamic_mover.cpp DynamicMover::Update
function Scene_DynamicMoverUpdate(scene: Scene, m: SceneDynamicMover, timeStep: f32, throttle: f32): void {
	let worldId = scene.worldId;

	// Reach further while grounded so the spring can find the ground over a step
	let stepDownLength: f32 = m.pogoRestLength;
	let rayLength: f32 = m.onGround ? m.pogoRestLength + stepDownLength : m.pogoRestLength;
	let translation = b2Vec2(0.0, -rayLength);

	let position = b2Body_GetPosition(m.moverId);
	let origin = b2Add(position, m.capsule.center1);

	let queryFilter = b2QueryFilter(m.filter.categoryBits, m.filter.maskBits);

	Scene_ResetCastResult(m.castResult);
	s_castResult = m.castResult;
	b2World_CastRay(worldId, origin, translation, queryFilter, Scene_CastCallback);

	m.pogoFraction = m.castResult.hit ? m.castResult.fraction : <f32>1.0;
	m.pogoHit = m.castResult.hit;

	// Should jumping end?
	if (m.jumping) {
		m.jumpTicks += 1;

		// Jump ticks allow time for the jump to leave the ground. Otherwise jumping ends
		// when the character approaches the current ground.
		if (m.jumpTicks > 2 && m.castResult.hit && b2Dot(m.velocity, m.castResult.normal) <= 0.0) {
			m.jumping = false;
		}
	}

	if (m.castResult.hit) {
		// The cast can still hit while jumping.
		m.onGround = m.jumping == false;

		// Is the ground too steep to walk?
		m.walkable = m.castResult.normal.y >= m.minGroundNormalY;
	} else {
		m.onGround = false;
		m.walkable = false;
	}

	m.velocity = b2Body_GetLinearVelocity(m.moverId);

	// Friction
	if (m.onGround) {
		let speed: f32 = b2Length(m.velocity);
		if (speed < m.minSpeed) {
			m.velocity.x = 0.0;
		} else {
			// Linear damping above stopSpeed and fixed reduction below stopSpeed
			let control: f32 = speed < m.stopSpeed ? m.stopSpeed : speed;

			// friction has units of 1/time
			let drop: f32 = control * m.friction * timeStep;
			let newSpeed: f32 = b2MaxFloat(0.0, speed - drop);
			m.velocity.x *= newSpeed / speed;
		}
	}

	// Mover force
	if (m.onGround) {
		let maxForce: f32 = m.walkable ? m.maxGroundForce : <f32>0.0;
		b2MoverJoint_SetMaxVelocityForce(m.moverJointId, b2Vec2(maxForce, 0.0));
	} else {
		b2MoverJoint_SetMaxVelocityForce(m.moverJointId, b2Vec2(m.maxAirForce, 0.0));
	}

	let desiredVelocity = b2Vec2(m.maxSpeed * throttle, 0.0);
	let desiredSpeedOut: FixedArray<f32, 1>;
	desiredSpeedOut[0] = 0.0;
	let desiredDirection = b2GetLengthAndNormalize(desiredSpeedOut[0], desiredVelocity);
	let desiredSpeed: f32 = desiredSpeedOut[0];

	if (desiredSpeed > m.maxSpeed) {
		desiredSpeed = m.maxSpeed;
	}

	// Accelerate
	let currentSpeed: f32 = b2Dot(m.velocity, desiredDirection);
	let addSpeed: f32 = desiredSpeed - currentSpeed;
	if (addSpeed > 0.0) {
		let steer: f32 = m.walkable ? <f32>1.0 : m.airSteer;
		let accelSpeed: f32 = steer * m.accelerate * m.maxSpeed * timeStep;
		if (accelSpeed > addSpeed) {
			accelSpeed = addSpeed;
		}

		m.velocity.x += accelSpeed * desiredDirection.x;
	}

	// The pogo joint is rebuilt every solve because it may land on a different body
	if (b2Joint_IsValid(m.pogoJointId)) {
		m.pogoLength = b2PogoJoint_GetCurrentLength(m.pogoJointId);
		m.pogoImpulse = b2PogoJoint_GetImpulse(m.pogoJointId);
		m.pogoVelocity = b2PogoJoint_GetCurrentSpeed(m.pogoJointId);

		b2DestroyJoint(m.pogoJointId);
		m.pogoJointId = b2_nullJointId;
	}

	if (m.castResult.hit == true) {
		let moverMass: f32 = b2Body_GetMass(m.moverId);
		let moverWeight: f32 = m.gravityScale * b2Length(b2World_GetGravity(worldId)) * moverMass;

		let pogoDef = b2DefaultPogoJointDef();
		pogoDef.base.localFrameA.p = b2Body_GetLocalPoint(m.castResult.bodyId, m.castResult.point);
		pogoDef.base.localFrameB.p = m.capsule.center1;
		pogoDef.normal = m.castResult.normal;
		pogoDef.base.bodyIdA = m.castResult.bodyId;
		pogoDef.base.bodyIdB = m.moverId;
		pogoDef.base.collideConnected = true;
		pogoDef.restLength = m.pogoRestLength;
		pogoDef.hertz = m.pogoHertz;
		pogoDef.dampingRatio = m.pogoDampingRatio;
		pogoDef.maxCompressionForce = m.pogoCompressionScale * moverWeight;

		// Warm start from the joint that was just destroyed
		pogoDef.impulse = m.pogoImpulse;
		pogoDef.velocity = m.pogoVelocity;

		if (m.jumping) {
			// Don't allow the pogo to pull down
			pogoDef.maxTensionForce = 0.0;
		} else {
			// The pogo can pull down at a multiple of the gravity force.
			pogoDef.maxTensionForce = m.pogoTensionScale * moverWeight;
		}

		m.pogoJointId = b2CreatePogoJoint(worldId, pogoDef);
	} else {
		m.pogoImpulse = 0.0;
		m.pogoVelocity = 0.0;
	}

	b2MoverJoint_SetLinearVelocity(m.moverJointId, m.velocity);
}

// The scene the pre-solve callbacks read
let s_preSolveScene: Scene = new Scene();

// samples/sample_character.cpp DynamicMoverSample::PreSolve: the elevator is one way for the mover
function Scene_PreSolve(shapeIdA: b2ShapeId, shapeIdB: b2ShapeId, manifold: b2PreSolveManifold): void {
	let scene = s_preSolveScene;
	let idA = b2Shape_GetBody(shapeIdA);
	let idB = b2Shape_GetBody(shapeIdB);
	let elevatorId = scene.keyBodies[0];
	let moverId = scene.dynamicMover.moverId;

	if (idA == elevatorId && idB == moverId) {
		if (manifold.manifold.normal.y < 0.0) {
			manifold.manifold.pointCount = 0;
		}
	} else if (idA == moverId && idB == elevatorId) {
		if (manifold.manifold.normal.y > 0.0) {
			manifold.manifold.pointCount = 0;
		}
	}
}

function Scene_PreContinuous(shapeIdA: b2ShapeId, shapeIdB: b2ShapeId, point: b2Pos, normal: b2Vec2): bool {
	let scene = s_preSolveScene;
	let idA = b2Shape_GetBody(shapeIdA);
	let idB = b2Shape_GetBody(shapeIdB);
	let elevatorId = scene.keyBodies[0];
	let moverId = scene.dynamicMover.moverId;

	if (idA == elevatorId && idB == moverId) {
		if (normal.y < 0.0) {
			return false;
		}

		return true;
	}

	if (idA == moverId && idB == elevatorId) {
		if (normal.y > 0.0) {
			return false;
		}

		return true;
	}

	return true;
}

// dynamic_mover: samples/sample_character.cpp DynamicMoverSample. keyBodies: 0 elevator
function CreateDynamicMover(scene: Scene): void {
	Scene_CreateDynamicMover(scene, scene.dynamicMover, b2Vec2(0.0, 8.0));

	let groundId1 = Scene_CreateLevelChain(scene, b2Vec2(0.0, 0.0), s_dynamicLevel1Bits, s_dynamicLevel1Bits.length / 2);
	let groundId2 = Scene_CreateLevelChain(scene, b2Vec2(98.0, 0.0), s_level2Bits, s_level2Bits.length / 2);

	Scene_CreateCharacterProps(scene, groundId1, groundId2, 4.0, false);

	s_preSolveScene = scene;
	b2World_SetPreSolveCallback(scene.worldId, Scene_PreSolve, Scene_PreContinuous);

	scene.jumpReleased = true;
	scene.time = 0.0;
}

function StepDynamicMover(scene: Scene, stepIndex: i32): void {
	let timeStep: f32 = <f32>1.0 / <f32>60.0;
	let m = scene.dynamicMover;

	Scene_MoveElevator(scene, timeStep);

	let throttle: f32 = Scene_MoverThrottle(stepIndex);
	if (Scene_MoverJumpKey(stepIndex)) {
		if (scene.jumpReleased && Scene_DynamicMoverJump(m)) {
			scene.jumpReleased = false;
		}
	} else {
		scene.jumpReleased = true;
	}

	Scene_DynamicMoverUpdate(scene, m, timeStep, throttle);

	Scene_WorldStep(scene);

	Probe_Bool(scene, m.castResult.hit);
	Probe_Vec2(scene, m.castResult.point);
	Probe_Vec2(scene, m.castResult.normal);
	Probe_BodyId(scene, m.castResult.bodyId);
	Probe_Float(scene, m.castResult.fraction);
	Probe_Vec2(scene, m.velocity);
	Probe_Bool(scene, m.onGround);
	Probe_Bool(scene, m.walkable);
	Probe_Bool(scene, m.jumping);
	Probe_Int(scene, m.jumpTicks);
	Probe_Float(scene, m.pogoImpulse);
	Probe_Float(scene, m.pogoVelocity);
	Probe_Float(scene, m.pogoLength);
	Probe_JointId(scene, m.pogoJointId);
	Probe_Joint(scene, m.moverJointId);
	Probe_Joint(scene, m.pogoJointId);
}

// samples/geometric_mover.cpp GeometricMover::GeometricMover and Create with the sample's def
function Scene_CreateGeometricMover(m: SceneGeometricMover, position: b2Pos): void {
	m.jumpSpeed = 10.0;
	m.maxSpeed = 6.0;
	m.minSpeed = 0.1;
	m.stopSpeed = 3.0;
	m.accelerate = 20.0;
	m.airSteer = 0.2;
	m.friction = 8.0;
	m.gravity = 30.0;
	m.pogoRestLength = 0.9;
	m.pogoHertz = 5.0;
	m.pogoDampingRatio = 0.8;
	m.pogoGroundForce = 50.0;
	m.pushBodies = true;
	m.capsule = b2Capsule(b2Vec2(0.0, -0.5), b2Vec2(0.0, 0.5), 0.3);
	m.filter = b2QueryFilter(s_moverBit, s_staticBit | s_dynamicBit | s_moverBit);
	m.minGroundNormalY = 0.7;
	m.position = position;
	m.velocity = b2Vec2_zero;
	m.pogoFraction = 1.0;
}

// samples/geometric_mover.cpp GeometricMover::Jump
function Scene_GeometricMoverJump(m: SceneGeometricMover): bool {
	if (m.onGround == false) {
		return false;
	}

	m.velocity.y = m.jumpSpeed;
	m.onGround = false;
	return true;
}

// The mover Scene_PlaneResultFcn fills
let s_planeMover: SceneGeometricMover = new SceneGeometricMover();

// samples/geometric_mover.cpp PlaneResultFcn. The sample points shape user data at a
// MoverShapeUserData; here the user data is a tag: 1 the friendly capsule, 2 the elevator.
function Scene_PlaneResultFcn(shapeId: b2ShapeId, planeResult: b2PlaneResult): bool {
	let m = s_planeMover;
	let maxPush: f32 = FLT_MAX;
	let clipVelocity = true;
	let tag: u64 = b2Shape_GetUserData(shapeId);
	if (tag == 1) {
		maxPush = 0.025;
		clipVelocity = false;
	} else if (tag == 2) {
		maxPush = 0.1;
		clipVelocity = true;
	}

	if (m.planeCount < SCENE_MOVER_PLANE_CAPACITY) {
		let index = m.planeCount;
		m.planes[index].plane = planeResult.plane;
		m.planes[index].pushLimit = maxPush;
		m.planes[index].push = 0.0;
		m.planes[index].clipVelocity = clipVelocity;

		// The plane point is relative to the origin passed to the collide call
		m.planePoints[index] = b2OffsetPos(m.position, planeResult.point);
		m.planeShapes[index] = shapeId;
		m.planeCount += 1;
	}

	return true;
}

// samples/geometric_mover.cpp GeometricMover::PushBodies
function Scene_GeometricMoverPushBodies(m: SceneGeometricMover): void {
	for (let i = 0; i < m.planeCount; ++i) {
		let bodyId = b2Shape_GetBody(m.planeShapes[i]);
		if (b2Body_GetType(bodyId) != b2_dynamicBody) {
			continue;
		}

		// Plane normals point at the mover
		let point = m.planePoints[i];
		let normal = b2Neg(m.planes[i].plane.normal);

		let invMassA: f32 = 0.0;

		let massB: f32 = b2Body_GetMass(bodyId);
		let invMassB: f32 = massB > 0.0 ? <f32>1.0 / massB : <f32>0.0;

		let inertiaB: f32 = b2Body_GetRotationalInertia(bodyId);
		let invIB: f32 = inertiaB > 0.0 ? <f32>1.0 / inertiaB : <f32>0.0;

		let centerB = b2Body_GetWorldCenter(bodyId);
		let rB = b2Sub(point, centerB);

		let rnB: f32 = b2Cross(rB, normal);
		let kNormal: f32 = invMassA + invMassB + invIB * rnB * rnB;
		let normalMass: f32 = kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0;

		let vB = b2Body_GetLinearVelocity(bodyId);
		let omegaB: f32 = b2Body_GetAngularVelocity(bodyId);
		let vrB = b2Add(vB, b2CrossSV(omegaB, rB));
		let vn: f32 = b2Dot(b2Sub(vrB, m.velocity), normal);

		// Push only, a separating body is left alone
		let impulse: f32 = b2MaxFloat(-normalMass * vn, 0.0);

		let P = b2MulSV(impulse, normal);
		m.velocity = b2MulSub(m.velocity, invMassA, P);

		b2Body_ApplyLinearImpulse(bodyId, P, point, true);
	}
}

// The ground probe of the geometric mover, a local in the C
const s_moverCast: SceneCastResult = new SceneCastResult();

// samples/geometric_mover.cpp GeometricMover::Update
function Scene_GeometricMoverUpdate(scene: Scene, m: SceneGeometricMover, timeStep: f32, throttle: f32): void {
	let worldId = scene.worldId;

	// Friction
	let speed: f32 = b2Length(m.velocity);
	if (speed < m.minSpeed) {
		m.velocity.x = 0.0;
		m.velocity.y = 0.0;
	} else if (m.onGround) {
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		let control: f32 = speed < m.stopSpeed ? m.stopSpeed : speed;

		// friction has units of 1/time
		let drop: f32 = control * m.friction * timeStep;
		let newSpeed: f32 = b2MaxFloat(0.0, speed - drop);
		let ratio: f32 = newSpeed / speed;
		m.velocity.x *= ratio;
		m.velocity.y *= ratio;
	}

	let desiredVelocity = b2Vec2(m.maxSpeed * throttle, 0.0);
	let desiredSpeedOut: FixedArray<f32, 1>;
	desiredSpeedOut[0] = 0.0;
	let desiredDirection = b2GetLengthAndNormalize(desiredSpeedOut[0], desiredVelocity);
	let desiredSpeed: f32 = desiredSpeedOut[0];

	if (desiredSpeed > m.maxSpeed) {
		desiredSpeed = m.maxSpeed;
	}

	// Accelerate
	let currentSpeed: f32 = b2Dot(m.velocity, desiredDirection);
	let addSpeed: f32 = desiredSpeed - currentSpeed;
	if (addSpeed > 0.0) {
		let steer: f32 = m.onGround ? <f32>1.0 : m.airSteer;
		let accelSpeed: f32 = steer * m.accelerate * m.maxSpeed * timeStep;
		if (accelSpeed > addSpeed) {
			accelSpeed = addSpeed;
		}

		m.velocity.x += accelSpeed * desiredDirection.x;
		m.velocity.y += accelSpeed * desiredDirection.y;
	}

	// The probe reaches past the rest length so the spring can be stretched
	let rayLength: f32 = m.onGround ? 2.0 * m.pogoRestLength : m.pogoRestLength;
	let translation = b2Vec2(0.0, -rayLength);
	let origin = b2Add(m.position, m.capsule.center1);

	let castResult = s_moverCast;
	Scene_ResetCastResult(castResult);
	s_castResult = castResult;
	b2World_CastRay(worldId, origin, translation, m.filter, Scene_CastCallback);

	m.pogoFraction = castResult.hit ? castResult.fraction : <f32>1.0;
	m.pogoHit = castResult.hit;

	// Avoid snapping to ground if still going up
	let haveGround = castResult.hit && castResult.normal.y > m.minGroundNormalY;

	if (m.onGround == false) {
		m.onGround = haveGround && m.velocity.y <= 0.01;
	} else {
		m.onGround = haveGround;
	}

	if (m.onGround == false) {
		m.velocity.y -= m.gravity * timeStep;
		m.pogoVelocity = 0.0;
	} else {
		m.velocity.y = 0.0;

		let pogoCurrentLength: f32 = castResult.fraction * rayLength;

		let offset: f32 = pogoCurrentLength - m.pogoRestLength;
		m.pogoVelocity = b2SpringDamper(m.pogoHertz, m.pogoDampingRatio, offset, m.pogoVelocity, timeStep);

		b2Body_ApplyForce(castResult.bodyId, b2Vec2(0.0, -m.pogoGroundForce), castResult.point, true);
	}

	let velocityDelta = b2MulSV(timeStep, m.velocity);
	let pogoDelta = b2MulSV(timeStep * m.pogoVelocity, b2Vec2(0.0, 1.0));
	let target = b2Add(b2Add(m.position, velocityDelta), pogoDelta);

	m.totalIterations = 0;
	let tolerance: f32 = 0.01;

	// Don't cast against other movers.
	let castFilter = b2QueryFilter(m.filter.categoryBits, m.filter.maskBits & ~m.filter.categoryBits);

	for (let iteration = 0; iteration < 5; ++iteration) {
		m.planeCount = 0;

		s_planeMover = m;
		b2World_CollideMover(worldId, m.position, m.capsule, m.filter, Scene_PlaneResultFcn);
		let targetDelta = b2Sub(target, m.position);
		let result = b2SolvePlanes(targetDelta, m.planes, m.planeCount);

		m.totalIterations += result.iterationCount;

		let fraction: f32 = b2World_CastMover(worldId, m.position, m.capsule, result.delta, castFilter);

		let delta = b2MulSV(fraction, result.delta);
		m.position = b2Add(m.position, delta);

		if (b2LengthSquared(delta) < tolerance * tolerance) {
			break;
		}
	}

	if (m.pushBodies) {
		Scene_GeometricMoverPushBodies(m);
	}

	// Stop pushing into surfaces so speed can't accumulate against a wall
	m.velocity = b2ClipVector(m.velocity, m.planes, m.planeCount);
}

// mover: samples/sample_character.cpp GeometryMoverSample. keyBodies: 0 elevator
function CreateMover(scene: Scene): void {
	Scene_CreateGeometricMover(scene.geometricMover, b2Vec2(0.0, 8.0));

	let groundId1 = Scene_CreateLevelChain(scene, b2Vec2(0.0, 0.0), s_geometricLevel1Bits, s_geometricLevel1Bits.length / 2);
	let groundId2 = Scene_CreateLevelChain(scene, b2Vec2(98.0, 0.0), s_level2Bits, s_level2Bits.length / 2);

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(32.0, 4.5);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.filter = b2Filter(s_moverBit, s_allBits, 0);
		shapeDef.userData = 1;
		let bodyId = Scene_AddBody(scene, bodyDef);
		b2CreateCapsuleShape(bodyId, shapeDef, scene.geometricMover.capsule);
	}

	Scene_CreateCharacterProps(scene, groundId1, groundId2, 5.0, true);

	scene.jumpReleased = true;
	scene.time = 0.0;
}

function StepMover(scene: Scene, stepIndex: i32): void {
	let timeStep: f32 = <f32>1.0 / <f32>60.0;
	let m = scene.geometricMover;

	Scene_MoveElevator(scene, timeStep);

	Scene_WorldStep(scene);

	let throttle: f32 = Scene_MoverThrottle(stepIndex);
	if (Scene_MoverJumpKey(stepIndex)) {
		if (scene.jumpReleased && Scene_GeometricMoverJump(m)) {
			scene.jumpReleased = false;
		}
	} else {
		scene.jumpReleased = true;
	}

	Scene_GeometricMoverUpdate(scene, m, timeStep, throttle);

	Probe_Vec2(scene, m.position);
	Probe_Vec2(scene, m.velocity);
	Probe_Float(scene, m.pogoVelocity);
	Probe_Bool(scene, m.onGround);
	Probe_Bool(scene, m.pogoHit);
	Probe_Float(scene, m.pogoFraction);
	Probe_Int(scene, m.totalIterations);
	Probe_Int(scene, m.planeCount);
	for (let i = 0; i < m.planeCount; ++i) {
		let plane = m.planes[i];
		Probe_Vec2(scene, plane.plane.normal);
		Probe_Float(scene, plane.plane.offset);
		Probe_Float(scene, plane.pushLimit);
		Probe_Float(scene, plane.push);
		Probe_Bool(scene, plane.clipVelocity);
		Probe_Vec2(scene, m.planePoints[i]);
		Probe_ShapeId(scene, m.planeShapes[i]);
	}
}

// shared/benchmarks.c CreateSmallPyramid
function Scene_CreateSmallPyramid(scene: Scene, baseCount: i32, extent: f32, centerX: f32, baseY: f32): void {
	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	let shapeDef = b2DefaultShapeDef();

	let box = b2MakeSquare(extent);

	for (let i = 0; i < baseCount; ++i) {
		let y: f32 = (2.0 * <f32>i + 1.0) * extent + baseY;

		for (let j = i; j < baseCount; ++j) {
			let x: f32 = (<f32>i + 1.0) * extent + 2.0 * <f32>(j - i) * extent + centerX - 0.5;
			bodyDef.position = b2Vec2(x, y);

			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);
		}
	}
}

// churn: two pyramids, a three shape body and a jointed pair under a script that creates, destroys,
// sleeps, wakes, disables, retypes, teleports and pushes bodies.
// keyBodies: 0 left pyramid base, 1 right pyramid top, 2 compound, 3 and 4 jointed pair, 5 right
// pyramid base. keyShapes 0..2: compound shapes. keyJoints 0: the pair's revolute.
function CreateChurn(scene: Scene): void {
	scene.seed = 104729;

	let groundDef = b2DefaultBodyDef();
	let groundId = Scene_AddBody(scene, groundDef);
	scene.groundId = groundId;

	{
		let shapeDef = b2DefaultShapeDef();
		let floor = b2MakeOffsetBox(20.0, 0.5, b2Vec2(0.0, -0.5), b2Rot_identity);
		b2CreatePolygonShape(groundId, shapeDef, floor);

		let segment = b2Segment(b2Vec2(-20.0, 0.0), b2Vec2(-20.0, 15.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);

		segment = b2Segment(b2Vec2(20.0, 0.0), b2Vec2(20.0, 15.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);
	}

	let first = scene.bodyCount;
	Scene_CreateSmallPyramid(scene, 4, 0.5, -12.0, 0.0);
	scene.keyBodies[0] = scene.bodies[first];

	first = scene.bodyCount;
	Scene_CreateSmallPyramid(scene, 4, 0.5, 8.0, 0.0);
	scene.keyBodies[5] = scene.bodies[first];
	scene.keyBodies[1] = scene.bodies[scene.bodyCount - 1];

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(0.0, 2.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[2] = bodyId;

		let shapeDef = b2DefaultShapeDef();
		let box = b2MakeBox(0.5, 0.5);
		scene.keyShapes[0] = b2CreatePolygonShape(bodyId, shapeDef, box);

		shapeDef.density = 3.0;
		let circle = b2Circle(b2Vec2(0.8, 0.0), 0.4);
		scene.keyShapes[1] = b2CreateCircleShape(bodyId, shapeDef, circle);

		let capsule = b2Capsule(b2Vec2(-0.9, -0.2), b2Vec2(-0.9, 0.2), 0.2);
		scene.keyShapes[2] = b2CreateCapsuleShape(bodyId, shapeDef, capsule);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(-3.0, 6.0);
		let bodyIdA = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[3] = bodyIdA;

		bodyDef.position = b2Vec2(-2.0, 6.0);
		let bodyIdB = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[4] = bodyIdB;

		let shapeDef = b2DefaultShapeDef();
		let box = b2MakeBox(0.5, 0.2);
		b2CreatePolygonShape(bodyIdA, shapeDef, box);
		b2CreatePolygonShape(bodyIdB, shapeDef, box);

		let pivot = b2Vec2(-2.5, 6.0);
		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = bodyIdA;
		jointDef.base.bodyIdB = bodyIdB;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint(bodyIdA, pivot);
		jointDef.base.localFrameB.p = b2Body_GetLocalPoint(bodyIdB, pivot);
		scene.keyJoints[0] = b2CreateRevoluteJoint(scene.worldId, jointDef);
	}
}

function Scene_SpawnChurnBody(scene: Scene): void {
	let kind = scene.spawnCount % 3;
	let x: f32 = Scene_RandomFloat(scene, -15.0, 15.0);
	let angle: f32 = Scene_RandomFloat(scene, -B2_PI, B2_PI);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(x, 14.0);
	bodyDef.rotation = b2MakeRot(angle);
	let bodyId = Scene_AddBody(scene, bodyDef);
	Scene_AddSpawn(scene, bodyId);

	let shapeDef = b2DefaultShapeDef();
	if (kind == 0) {
		let box = b2MakeSquare(0.4);
		b2CreatePolygonShape(bodyId, shapeDef, box);
	} else if (kind == 1) {
		let circle = b2Circle(b2Vec2(0.0, 0.0), 0.4);
		b2CreateCircleShape(bodyId, shapeDef, circle);
	} else {
		let capsule = b2Capsule(b2Vec2(-0.3, 0.0), b2Vec2(0.3, 0.0), 0.25);
		b2CreateCapsuleShape(bodyId, shapeDef, capsule);
	}
}

function StepChurn(scene: Scene, stepIndex: i32): void {
	let keys = scene.keyBodies;

	if (stepIndex % 25 == 0 && stepIndex < 450) {
		Scene_SpawnChurnBody(scene);
	}

	if (stepIndex >= 60 && stepIndex % 40 == 20 && scene.spawnHead < scene.spawnCount) {
		b2DestroyBody(scene.spawned[scene.spawnHead]);
		scene.spawnHead += 1;
	}

	if (stepIndex == 120) {
		b2DestroyShape(scene.keyShapes[1], true);
	} else if (stepIndex == 130) {
		b2Body_Disable(keys[1]);
	} else if (stepIndex == 150) {
		b2Body_SetAwake(keys[0], false);
	} else if (stepIndex == 160) {
		b2Body_SetType(keys[2], b2_staticBody);
	} else if (stepIndex == 180) {
		b2DestroyJoint(scene.keyJoints[0]);
	} else if (stepIndex == 200) {
		b2Body_Enable(keys[1]);
	} else if (stepIndex == 230) {
		b2Body_SetAwake(keys[0], true);
	} else if (stepIndex == 240) {
		b2Body_SetType(keys[2], b2_kinematicBody);
		b2Body_SetLinearVelocity(keys[2], b2Vec2(1.0, 0.0));
		b2Body_SetAngularVelocity(keys[2], 0.5);
	} else if (stepIndex == 260) {
		b2Body_SetTransform(keys[5], b2Vec2(0.0, 10.0), b2MakeRot(0.7));
	} else if (stepIndex == 300) {
		b2Body_SetType(keys[2], b2_dynamicBody);
	} else if (stepIndex == 320) {
		let point = b2Body_GetWorldPoint(keys[0], b2Vec2(0.5, 0.5));
		b2Body_ApplyLinearImpulse(keys[0], b2Vec2(0.0, 5.0), point, false);
	} else if (stepIndex == 330) {
		b2Body_ApplyLinearImpulseToCenter(keys[3], b2Vec2(3.0, 6.0), true);
	} else if (stepIndex == 340) {
		b2Body_ApplyAngularImpulse(keys[4], 2.0, true);
	} else if (stepIndex == 380) {
		b2Body_WakeTouching(scene.groundId);
	} else if (stepIndex == 390) {
		b2Body_EnableSleeping(keys[2], false);
	} else if (stepIndex == 400) {
		b2Body_SetLinearVelocity(keys[2], b2Vec2(-2.0, 4.0));
		b2Body_SetAngularVelocity(keys[2], 3.0);
	} else if (stepIndex == 420) {
		b2Body_SetGravityScale(keys[3], -0.5);
	} else if (stepIndex == 450) {
		b2Body_SetGravityScale(keys[3], 1.0);
	} else if (stepIndex == 470) {
		b2Body_EnableSleeping(keys[2], true);
	} else if (stepIndex == 480) {
		b2World_EnableSleeping(scene.worldId, false);
	} else if (stepIndex == 520) {
		b2World_EnableSleeping(scene.worldId, true);
	}

	if (350 <= stepIndex && stepIndex < 370) {
		let point = b2Body_GetWorldPoint(keys[2], b2Vec2(0.5, 0.0));
		b2Body_ApplyForce(keys[2], b2Vec2(0.0, 30.0), point, true);
		b2Body_ApplyTorque(keys[4], 5.0, true);
		b2Body_ApplyForceToCenter(keys[3], b2Vec2(20.0, 0.0), true);
	}
	if (stepIndex == 369) {
		b2Body_ClearForces(keys[3]);
	}

	Scene_WorldStep(scene);

	Probe_Float(scene, b2Body_GetMass(keys[2]));
	Probe_Float(scene, b2Body_GetRotationalInertia(keys[2]));
	Probe_Vec2(scene, b2Body_GetLocalCenter(keys[2]));
	Probe_Int(scene, b2Body_GetShapeCount(keys[2]));
	Probe_Int(scene, b2Body_GetJointCount(keys[3]));
	Probe_Bool(scene, b2Body_IsEnabled(keys[1]));
	Probe_Bool(scene, b2Body_IsSleepingEnabled(keys[2]));
	Probe_Bool(scene, b2World_IsSleepingEnabled(scene.worldId));
	Probe_Int(scene, scene.spawnCount);
	Probe_Int(scene, scene.spawnHead);
}

// bullets: samples/sample_stacking.cpp VerticalStack with its bullets fired by script, thin wall
// projectiles from samples/sample_continuous.cpp, and a resting box turned into a bullet.
// keyBodies: 0 current bullet, 1 dart
function CreateBullets(scene: Scene): void {
	scene.seed = 2718;

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(0.0, 0.0);
		let groundId = Scene_AddBody(scene, bodyDef);
		scene.groundId = groundId;

		let shapeDef = b2DefaultShapeDef();

		let segment = b2Segment(b2Vec2(10.0, 0.0), b2Vec2(10.0, 80.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);

		segment = b2Segment(b2Vec2(-100.0, 0.0), b2Vec2(100.0, 0.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);
	}

	{
		let box = b2MakeRoundedBox(0.45, 0.45, 0.05);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0;
		shapeDef.material.friction = 0.3;

		let offset: f32 = 0.01;
		let x: f32 = 8.0;

		for (let i = 0; i < 12; ++i) {
			let bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			let shift: f32 = i % 2 == 0 ? -offset : offset;
			bodyDef.position = b2Vec2(x + shift, 0.5 + 1.0 * <f32>i);
			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);
		}
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(-60.0, 5.0);
		let wallId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		let box = b2MakeBox(0.05, 5.0);
		b2CreatePolygonShape(wallId, shapeDef, box);
	}

	{
		let shapeDef = b2DefaultShapeDef();
		shapeDef.enableHitEvents = true;

		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		bodyDef.position = b2Vec2(-90.0, 2.0);
		bodyDef.linearVelocity = b2Vec2(150.0, 0.0);
		let circleId = Scene_AddBody(scene, bodyDef);
		let circle = b2Circle(b2Vec2(0.0, 0.0), 0.25);
		b2CreateCircleShape(circleId, shapeDef, circle);

		bodyDef.position = b2Vec2(-90.0, 5.0);
		bodyDef.linearVelocity = b2Vec2(120.0, 0.0);
		bodyDef.angularVelocity = 50.0;
		let spinnerId = Scene_AddBody(scene, bodyDef);
		let thinBox = b2MakeBox(0.05, 0.6);
		b2CreatePolygonShape(spinnerId, shapeDef, thinBox);

		bodyDef.position = b2Vec2(-90.0, 8.0);
		bodyDef.linearVelocity = b2Vec2(180.0, 0.0);
		bodyDef.angularVelocity = 0.0;
		let capsuleId = Scene_AddBody(scene, bodyDef);
		let capsule = b2Capsule(b2Vec2(-0.3, 0.0), b2Vec2(0.3, 0.0), 0.1);
		b2CreateCapsuleShape(capsuleId, shapeDef, capsule);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(-40.0, 0.2);
		let dartId = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[1] = dartId;

		let shapeDef = b2DefaultShapeDef();
		shapeDef.density = 4.0;
		shapeDef.enableHitEvents = true;
		let box = b2MakeBox(0.3, 0.15);
		b2CreatePolygonShape(dartId, shapeDef, box);
	}

	scene.keyBodies[0] = b2_nullBodyId;
}

// samples/sample_stacking.cpp VerticalStack::FireBullets for one bullet, destroying the previous
// one first as Launch does
function Scene_FireBullet(scene: Scene, isBox: bool, y: f32): void {
	if (scene.keyBodies[0].index1 != 0) {
		b2DestroyBody(scene.keyBodies[0]);
		scene.keyBodies[0] = b2_nullBodyId;
	}

	let circle = b2Circle(b2Vec2(0.0, 0.0), 0.25);
	let box = b2MakeBox(0.25, 0.25);

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 4.0;
	shapeDef.enableHitEvents = true;
	shapeDef.enableContactEvents = true;

	let speed: f32 = Scene_RandomFloat(scene, 200.0, 300.0);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(-26.7, y);
	bodyDef.linearVelocity = b2Vec2(speed, 0.0);
	bodyDef.isBullet = true;

	let bullet = Scene_AddBody(scene, bodyDef);

	if (isBox) {
		b2CreatePolygonShape(bullet, shapeDef, box);
	} else {
		b2CreateCircleShape(bullet, shapeDef, circle);
	}

	scene.keyBodies[0] = bullet;

	Probe_BodyId(scene, bullet);
	Probe_Float(scene, speed);
}

function StepBullets(scene: Scene, stepIndex: i32): void {
	if (stepIndex == 30) {
		Scene_FireBullet(scene, false, 6.0);
	} else if (stepIndex == 90) {
		Scene_FireBullet(scene, true, 6.0);
	} else if (stepIndex == 150) {
		Scene_FireBullet(scene, false, 2.5);
	} else if (stepIndex == 200) {
		b2Body_SetBullet(scene.keyBodies[1], true);
		b2Body_SetLinearVelocity(scene.keyBodies[1], b2Vec2(250.0, 0.0));
	}

	Scene_WorldStep(scene);

	Probe_Bool(scene, b2Body_IsBullet(scene.keyBodies[1]));
	Probe_ContactEvents(scene);
}

// sensors: samples/sample_events.cpp sensor setups: a static sensor zone, a sensor on a sweeping
// kinematic body and a sensor ring on a rolling dynamic ball, with visitors dropped through them.
// keyShapes: 0 static zone, 1 kinematic sweeper, 2 ball ring. keyBodies: 0 sweeper.
function CreateSensors(scene: Scene): void {
	scene.seed = 31337;

	let groundDef = b2DefaultBodyDef();
	let groundId = Scene_AddBody(scene, groundDef);
	scene.groundId = groundId;

	{
		let shapeDef = b2DefaultShapeDef();
		let segment = b2Segment(b2Vec2(-20.0, 0.0), b2Vec2(20.0, 0.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);

		segment = b2Segment(b2Vec2(-20.0, 0.0), b2Vec2(-20.0, 20.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);

		segment = b2Segment(b2Vec2(20.0, 0.0), b2Vec2(20.0, 20.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(-5.0, 7.0);
		let zoneId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		let box = b2MakeBox(3.0, 1.5);
		scene.keyShapes[0] = b2CreatePolygonShape(zoneId, shapeDef, box);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = b2Vec2(-15.0, 1.0);
		bodyDef.linearVelocity = b2Vec2(3.0, 0.0);
		let sweeperId = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[0] = sweeperId;

		let shapeDef = b2DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		let box = b2MakeBox(1.0, 1.0);
		scene.keyShapes[1] = b2CreatePolygonShape(sweeperId, shapeDef, box);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(12.0, 0.5);
		bodyDef.linearVelocity = b2Vec2(-4.0, 0.0);
		bodyDef.angularVelocity = 8.0;
		let ballId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = 0.5;
		shapeDef.material.rollingResistance = 0.02;
		shapeDef.enableSensorEvents = true;
		let circle = b2Circle(b2Vec2(0.0, 0.0), 0.5);
		b2CreateCircleShape(ballId, shapeDef, circle);

		let sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		let ring = b2Circle(b2Vec2(0.0, 0.0), 2.0);
		scene.keyShapes[2] = b2CreateCircleShape(ballId, sensorDef, ring);
	}
}

function Scene_SpawnVisitor(scene: Scene): void {
	let index = scene.spawnCount;
	let x: f32 = Scene_RandomFloat(scene, -9.0, 3.0);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(x, 12.0);
	let bodyId = Scene_AddBody(scene, bodyDef);
	Scene_AddSpawn(scene, bodyId);

	let shapeDef = b2DefaultShapeDef();
	shapeDef.enableSensorEvents = index != 4;
	if (index % 2 == 0) {
		let circle = b2Circle(b2Vec2(0.0, 0.0), 0.3);
		b2CreateCircleShape(bodyId, shapeDef, circle);
	} else {
		let box = b2MakeSquare(0.3);
		b2CreatePolygonShape(bodyId, shapeDef, box);
	}
}

function StepSensors(scene: Scene, stepIndex: i32): void {
	if (stepIndex < 240 && stepIndex % 12 == 0) {
		Scene_SpawnVisitor(scene);
	}

	if (stepIndex % 300 == 299) {
		let v = b2Body_GetLinearVelocity(scene.keyBodies[0]);
		b2Body_SetLinearVelocity(scene.keyBodies[0], b2Neg(v));
	}

	if (stepIndex == 400) {
		b2DestroyBody(scene.spawned[0]);
		b2DestroyBody(scene.spawned[1]);
	} else if (stepIndex == 420) {
		let count = b2Body_GetShapes(scene.spawned[2], s_shapeIds, 1);
		if (count == 1) {
			b2Shape_EnableSensorEvents(s_shapeIds[0], false);
		}
	}

	Scene_WorldStep(scene);

	Probe_SensorEvents(scene);
	for (let i = 0; i < 3; ++i) {
		Probe_Int(scene, b2Shape_GetSensorCapacity(scene.keyShapes[i]));
		let count = b2Shape_GetSensorData(scene.keyShapes[i], s_shapeIds, SCENE_HIT_CAPACITY);
		Probe_Int(scene, count);
		for (let j = 0; j < count; ++j) {
			Probe_ShapeId(scene, s_shapeIds[j]);
		}
	}
}

// events: samples/sample_events.cpp ContactEvent and BodyMove style drops with contact, hit and
// move events, a spinning kinematic paddle, and a script that changes event flags.
// keyShapes: 0 first dropped shape. keyBodies: 0 paddle. spawned: the dropped bodies.
function CreateEvents(scene: Scene): void {
	scene.seed = 4242;

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(0.0, -1.0);
		let groundId = Scene_AddBody(scene, bodyDef);
		scene.groundId = groundId;

		let shapeDef = b2DefaultShapeDef();
		let box = b2MakeBox(20.0, 1.0);
		b2CreatePolygonShape(groundId, shapeDef, box);

		let segment = b2Segment(b2Vec2(-14.0, 6.0), b2Vec2(-6.0, 1.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = b2Vec2(9.0, 2.0);
		bodyDef.angularVelocity = 2.0;
		let paddleId = Scene_AddBody(scene, bodyDef);
		scene.keyBodies[0] = paddleId;

		let shapeDef = b2DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		let box = b2MakeBox(2.0, 0.2);
		b2CreatePolygonShape(paddleId, shapeDef, box);
	}

	for (let i = 0; i < 16; ++i) {
		let column = i % 4;
		let row = i / 4;
		let jitter: f32 = Scene_RandomFloat(scene, 0.0, 1.0);
		let restitution: f32 = Scene_RandomFloat(scene, 0.2, 0.7);

		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(-9.0 + 5.0 * <f32>column, 3.0 + 2.5 * <f32>row + jitter);
		let bodyId = Scene_AddBody(scene, bodyDef);
		Scene_AddSpawn(scene, bodyId);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = restitution;
		shapeDef.enableContactEvents = true;
		shapeDef.enableHitEvents = i % 2 == 0;

		let shapeId = b2ShapeId();
		if (i % 2 == 0) {
			let circle = b2Circle(b2Vec2(0.0, 0.0), 0.4);
			shapeId = b2CreateCircleShape(bodyId, shapeDef, circle);
		} else {
			let box = b2MakeBox(0.4, 0.3);
			shapeId = b2CreatePolygonShape(bodyId, shapeDef, box);
		}

		if (i == 0) {
			scene.keyShapes[0] = shapeId;
		}
	}
}

function StepEvents(scene: Scene, stepIndex: i32): void {
	let spawned = scene.spawned;

	if (stepIndex == 150) {
		for (let i = 0; i < 4; ++i) {
			b2Body_ApplyLinearImpulseToCenter(spawned[i], b2Vec2(0.5, 6.0), true);
		}
	} else if (stepIndex == 200) {
		b2World_SetHitEventThreshold(scene.worldId, 2.0);
	} else if (stepIndex == 220) {
		b2DestroyBody(spawned[5]);
	} else if (stepIndex == 250) {
		b2Body_SetAngularVelocity(scene.keyBodies[0], 0.0);
	} else if (stepIndex == 260) {
		let count = b2Body_GetShapes(spawned[6], s_shapeIds, 1);
		if (count == 1) {
			b2Shape_EnableContactEvents(s_shapeIds[0], false);
		}
		b2Body_EnableHitEvents(spawned[8], false);
	} else if (stepIndex == 300) {
		for (let i = 8; i < 12; ++i) {
			b2Body_ApplyLinearImpulseToCenter(spawned[i], b2Vec2(-1.0, 8.0), true);
		}
	}

	Scene_WorldStep(scene);

	Probe_ContactEvents(scene);

	// Manifold summary of each begin contact and validity of each end event's shapes
	let events = b2World_GetContactEvents(scene.worldId);
	for (let i = 0; i < events.beginCount; ++i) {
		let contactId = events.beginEvents[i].contactId;
		let valid = b2Contact_IsValid(contactId);
		Probe_Bool(scene, valid);
		if (valid) {
			let data = b2Contact_GetData(contactId);
			Probe_Int(scene, data.manifold.pointCount);
			Probe_Vec2(scene, data.manifold.normal);
		}
	}
	for (let i = 0; i < events.endCount; ++i) {
		Probe_Bool(scene, b2Shape_IsValid(events.endEvents[i].shapeIdA));
		Probe_Bool(scene, b2Shape_IsValid(events.endEvents[i].shapeIdB));
	}

	Probe_BodyMoveEvents(scene);

	Probe_Int(scene, b2Shape_GetContactCapacity(scene.keyShapes[0]));
	let count = b2Shape_GetContactData(scene.keyShapes[0], s_contactData, SCENE_CONTACT_CAPACITY);
	Probe_Int(scene, count);
	for (let i = 0; i < count; ++i) {
		Probe_ContactData(scene, s_contactData[i]);
	}
}

// Query results gathered by the callbacks below, in call order
class SceneHits {
	shapeIds: StaticArray<b2ShapeId> = new StaticArray<b2ShapeId>(SCENE_HIT_CAPACITY);
	points: StaticArray<b2Pos> = new StaticArray<b2Pos>(SCENE_HIT_CAPACITY);
	normals: StaticArray<b2Vec2> = new StaticArray<b2Vec2>(SCENE_HIT_CAPACITY);
	fractions: StaticArray<f32> = new StaticArray<f32>(SCENE_HIT_CAPACITY);
	count: i32 = 0;
	calls: i32 = 0;
}

const s_hits: SceneHits = new SceneHits();

// The C clears the whole struct; the arrays past count are never read
function Scene_ResetHits(hits: SceneHits): void {
	hits.count = 0;
	hits.calls = 0;
}

function Scene_OverlapCallback(shapeId: b2ShapeId): bool {
	let hits = s_hits;
	hits.calls += 1;
	if (hits.count < SCENE_HIT_CAPACITY) {
		hits.shapeIds[hits.count] = shapeId;
		hits.count += 1;
	}
	return true;
}

// Gathers every hit and lets the ray continue
function Scene_RayAllCallback(shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32): f32 {
	let hits = s_hits;
	hits.calls += 1;
	if (hits.count < SCENE_HIT_CAPACITY) {
		let i = hits.count;
		hits.shapeIds[i] = shapeId;
		hits.points[i] = point;
		hits.normals[i] = normal;
		hits.fractions[i] = fraction;
		hits.count += 1;
	}
	return 1.0;
}

// Keeps the latest hit in slot 0 and clips the cast to it, so slot 0 ends as the closest
function Scene_CastClosestCallback(shapeId: b2ShapeId, point: b2Pos, normal: b2Vec2, fraction: f32): f32 {
	let hits = s_hits;
	hits.calls += 1;
	hits.shapeIds[0] = shapeId;
	hits.points[0] = point;
	hits.normals[0] = normal;
	hits.fractions[0] = fraction;
	hits.count = 1;
	return fraction;
}

// Shapes tagged 7 that opted into custom filtering pass through each other
function Scene_CustomFilter(shapeIdA: b2ShapeId, shapeIdB: b2ShapeId): bool {
	let tagA: u64 = b2Shape_GetUserData(shapeIdA);
	let tagB: u64 = b2Shape_GetUserData(shapeIdB);
	if (tagA == 7 && tagB == 7) {
		return false;
	}
	return true;
}

function Probe_Hits(scene: Scene, hits: SceneHits, withCast: bool): void {
	Probe_Int(scene, hits.calls);
	Probe_Int(scene, hits.count);
	for (let i = 0; i < hits.count; ++i) {
		Probe_ShapeId(scene, hits.shapeIds[i]);
		if (withCast) {
			Probe_Vec2(scene, hits.points[i]);
			Probe_Vec2(scene, hits.normals[i]);
			Probe_Float(scene, hits.fractions[i]);
		}
	}
}

function Probe_CastOutput(scene: Scene, output: b2CastOutput): void {
	Probe_Bool(scene, output.hit);
	if (output.hit) {
		Probe_Vec2(scene, output.point);
		Probe_Vec2(scene, output.normal);
		Probe_Float(scene, output.fraction);
	}
}

function Probe_AABB(scene: Scene, aabb: b2AABB): void {
	Probe_Vec2(scene, aabb.lowerBound);
	Probe_Vec2(scene, aabb.upperBound);
}

// queries: static shapes of every kind on their own bodies, a rain of dynamic bodies, some custom
// filtered, explosions, and every world, shape and body query each step.
// keyShapes: 0 static box, 1 first dynamic shape. keyBodies: 0 first dynamic body.
function CreateQueries(scene: Scene): void {
	scene.seed = 1234567;
	let worldId = scene.worldId;

	b2World_SetCustomFilterCallback(worldId, Scene_CustomFilter);

	let shapeDef = b2DefaultShapeDef();
	let bodyDef = b2DefaultBodyDef();

	{
		let groundId = Scene_AddBody(scene, bodyDef);
		scene.groundId = groundId;
		let segment = b2Segment(b2Vec2(-25.0, 0.0), b2Vec2(25.0, 0.0));
		b2CreateSegmentShape(groundId, shapeDef, segment);
	}

	{
		bodyDef.position = b2Vec2(-15.0, 3.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let circle = b2Circle(b2Vec2(0.0, 0.0), 1.0);
		b2CreateCircleShape(bodyId, shapeDef, circle);
	}

	{
		bodyDef.position = b2Vec2(-8.0, 2.0);
		bodyDef.rotation = b2MakeRot(0.4);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let capsule = b2Capsule(b2Vec2(-1.0, 0.0), b2Vec2(1.0, 0.0), 0.5);
		b2CreateCapsuleShape(bodyId, shapeDef, capsule);
	}

	{
		bodyDef.position = b2Vec2(-2.0, 4.0);
		bodyDef.rotation = b2MakeRot(0.3);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let box = b2MakeBox(1.0, 1.0);
		scene.keyShapes[0] = b2CreatePolygonShape(bodyId, shapeDef, box);
	}

	{
		bodyDef.position = b2Vec2(5.0, 3.0);
		bodyDef.rotation = b2Rot_identity;
		let bodyId = Scene_AddBody(scene, bodyDef);
		let points = new StaticArray<b2Vec2>(5);
		points[0] = b2Vec2(-1.0, -0.5);
		points[1] = b2Vec2(1.0, -0.7);
		points[2] = b2Vec2(1.2, 0.4);
		points[3] = b2Vec2(0.0, 1.0);
		points[4] = b2Vec2(-0.9, 0.6);
		let hull = b2ComputeHull(points, 5);
		let polygon = b2MakePolygon(hull, 0.2);
		b2CreatePolygonShape(bodyId, shapeDef, polygon);
	}

	{
		bodyDef.position = b2Vec2(0.0, 0.0);
		let bodyId = Scene_AddBody(scene, bodyDef);
		let segment = b2Segment(b2Vec2(10.0, 1.0), b2Vec2(14.0, 4.0));
		b2CreateSegmentShape(bodyId, shapeDef, segment);

		let points = new StaticArray<b2Vec2>(4);
		points[0] = b2Vec2(22.0, 2.0);
		points[1] = b2Vec2(20.0, 1.0);
		points[2] = b2Vec2(18.0, 1.5);
		points[3] = b2Vec2(16.0, 0.5);
		let chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 4;
		chainDef.ghost1 = b2Vec2(24.0, 2.0);
		chainDef.ghost2 = b2Vec2(14.0, 0.5);
		b2CreateChain(bodyId, chainDef);
	}

	for (let i = 0; i < 12; ++i) {
		let x: f32 = Scene_RandomFloat(scene, -20.0, 20.0);

		let dynamicDef = b2DefaultBodyDef();
		dynamicDef.type = b2_dynamicBody;
		dynamicDef.position = b2Vec2(x, 8.0 + 1.5 * <f32>i);
		let bodyId = Scene_AddBody(scene, dynamicDef);

		let dynamicShapeDef = b2DefaultShapeDef();
		if (i % 3 == 0) {
			dynamicShapeDef.enableCustomFiltering = true;
			dynamicShapeDef.userData = 7;
		}

		let shapeId = b2ShapeId();
		if (i % 2 == 0) {
			let circle = b2Circle(b2Vec2(0.0, 0.0), 0.5);
			shapeId = b2CreateCircleShape(bodyId, dynamicShapeDef, circle);
		} else {
			let box = b2MakeBox(0.5, 0.4);
			shapeId = b2CreatePolygonShape(bodyId, dynamicShapeDef, box);
		}

		if (i == 0) {
			scene.keyBodies[0] = bodyId;
			scene.keyShapes[1] = shapeId;
		}
	}
}

function StepQueries(scene: Scene, stepIndex: i32): void {
	let worldId = scene.worldId;

	if (stepIndex == 120 || stepIndex == 240) {
		let def = b2DefaultExplosionDef();
		def.position = b2Vec2(0.0, 1.0);
		def.radius = 4.0;
		def.falloff = 2.0;
		def.impulsePerLength = 4.0;
		b2World_Explode(worldId, def);
	}

	Scene_WorldStep(scene);

	let t: f32 = <f32>(stepIndex % 200);
	let filter = b2DefaultQueryFilter();
	let hits = s_hits;

	{
		let cx: f32 = -20.0 + 0.2 * t;
		let aabb = b2AABB(b2Vec2(cx - 2.0, 0.0), b2Vec2(cx + 2.0, 4.0));
		Scene_ResetHits(hits);
		b2World_OverlapAABB(worldId, b2Pos_zero, aabb, filter, Scene_OverlapCallback);
		Probe_Hits(scene, hits, false);
	}

	{
		let points: FixedArray<b2Vec2, 8>;
		points[0] = b2Vec2(-1.0, -0.5);
		points[1] = b2Vec2(1.0, -0.5);
		points[2] = b2Vec2(0.0, 1.0);
		let proxy = b2MakeProxy(points, 3, 0.1);
		let origin = b2Vec2(20.0 - 0.2 * t, 3.0);
		Scene_ResetHits(hits);
		b2World_OverlapShape(worldId, origin, proxy, filter, Scene_OverlapCallback);
		Probe_Hits(scene, hits, false);
	}

	{
		let origin = b2Vec2(-24.0, 0.5 + 0.03 * t);
		let translation = b2Vec2(48.0, 4.0);
		Scene_ResetHits(hits);
		b2World_CastRay(worldId, origin, translation, filter, Scene_RayAllCallback);
		Probe_Hits(scene, hits, true);
	}

	{
		let origin = b2Vec2(-20.0 + 0.2 * t, 15.0);
		let translation = b2Vec2(0.0, -20.0);
		let result = b2World_CastRayClosest(worldId, origin, translation, filter);
		Probe_Bool(scene, result.hit);
		if (result.hit) {
			Probe_ShapeId(scene, result.shapeId);
			Probe_Vec2(scene, result.point);
			Probe_Vec2(scene, result.normal);
			Probe_Float(scene, result.fraction);
		}
	}

	{
		let center: FixedArray<b2Vec2, 8>;
		center[0] = b2Vec2(0.0, 0.0);
		let proxy = b2MakeProxy(center, 1, 0.5);
		let origin = b2Vec2(-24.0, 6.0 - 0.025 * t);
		let translation = b2Vec2(48.0, -2.0);
		Scene_ResetHits(hits);
		b2World_CastShape(worldId, origin, proxy, translation, filter, Scene_CastClosestCallback);
		Probe_Hits(scene, hits, true);
	}

	{
		let origin = b2Vec2(-2.0, 8.0);
		let translation = b2Vec2(0.02 * t - 2.0, -8.0);
		Probe_CastOutput(scene, b2Shape_CastRay(scene.keyShapes[0], origin, translation));

		let target = b2Body_GetPosition(scene.keyBodies[0]);
		let origin2 = b2Vec2(target.x - 3.0, target.y + 1.0);
		let translation2 = b2Vec2(6.0, -2.0);
		Probe_CastOutput(scene, b2Shape_CastRay(scene.keyShapes[1], origin2, translation2));
	}

	{
		let point = b2Vec2(0.1 * t - 10.0, 6.0);
		Probe_Bool(scene, b2Shape_TestPoint(scene.keyShapes[0], point));
		Probe_Vec2(scene, b2Shape_GetClosestPoint(scene.keyShapes[0], point));
		Probe_Bool(scene, b2Shape_TestPoint(scene.keyShapes[1], b2Body_GetPosition(scene.keyBodies[0])));
	}

	{
		let bodyId = scene.keyBodies[0];
		Probe_AABB(scene, b2Body_ComputeBounds(bodyId));
		Probe_AABB(scene, b2Shape_GetBounds(scene.keyShapes[1]));
		Probe_AABB(scene, b2World_GetBounds(worldId));
		Probe_Vec2(scene, b2Body_GetWorldPoint(bodyId, b2Vec2(0.5, 0.0)));
		Probe_Vec2(scene, b2Body_GetLocalPoint(bodyId, b2Vec2(0.0, 0.0)));
		Probe_Vec2(scene, b2Body_GetWorldVector(bodyId, b2Vec2(1.0, 0.0)));
		Probe_Vec2(scene, b2Body_GetLocalVector(bodyId, b2Vec2(0.0, 1.0)));
		Probe_Vec2(scene, b2Body_GetLocalPointVelocity(bodyId, b2Vec2(0.5, 0.0)));
		Probe_Vec2(scene, b2Body_GetWorldPointVelocity(bodyId, b2Vec2(0.0, 0.0)));
		Probe_Vec2(scene, b2Body_GetWorldCenter(bodyId));
	}
}

// many_bodies: shared/benchmarks.c CreateManyPyramids at 10 x 10 pyramids
function CreateManyBodies(scene: Scene): void {
	b2World_EnableSleeping(scene.worldId, false);

	let baseCount = 10;
	let extent: f32 = 0.5;
	let rowCount = 10;
	let columnCount = 10;

	let bodyDef = b2DefaultBodyDef();
	let groundId = Scene_AddBody(scene, bodyDef);

	let groundDeltaY: f32 = 2.0 * extent * (<f32>baseCount + 1.0);
	let groundWidth: f32 = 2.0 * extent * <f32>columnCount * (<f32>baseCount + 1.0);
	let shapeDef = b2DefaultShapeDef();

	let groundY: f32 = 0.0;

	for (let i = 0; i < rowCount; ++i) {
		let segment = b2Segment(b2Vec2(-0.5 * groundWidth, groundY), b2Vec2(0.5 * groundWidth, groundY));
		b2CreateSegmentShape(groundId, shapeDef, segment);
		groundY += groundDeltaY;
	}

	let baseWidth: f32 = 2.0 * extent * <f32>baseCount;
	let baseY: f32 = 0.0;

	for (let i = 0; i < rowCount; ++i) {
		for (let j = 0; j < columnCount; ++j) {
			let centerX: f32 = -0.5 * groundWidth + <f32>j * (baseWidth + 2.0 * extent) + 2.0 * extent;
			Scene_CreateSmallPyramid(scene, baseCount, extent, centerX, baseY);
		}

		baseY += groundDeltaY;
	}
}

// large_pyramid: shared/benchmarks.c CreateLargePyramid, base 100
function CreateLargePyramid(scene: Scene): void {
	b2World_EnableSleeping(scene.worldId, false);

	let baseCount = 100;

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.position = b2Vec2(0.0, -1.0);
		let groundId = Scene_AddBody(scene, bodyDef);

		let box = b2MakeBox(100.0, 1.0);
		let shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape(groundId, shapeDef, box);
	}

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;

	let a: f32 = 0.5;
	let box = b2MakeSquare(a);

	let shift: f32 = 1.0 * a;

	for (let i = 0; i < baseCount; ++i) {
		let y: f32 = (2.0 * <f32>i + 1.0) * shift;

		for (let j = i; j < baseCount; ++j) {
			let x: f32 = (<f32>i + 1.0) * shift + 2.0 * <f32>(j - i) * shift - a * <f32>baseCount;

			bodyDef.position = b2Vec2(x, y);

			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);
		}
	}
}

// joint_grid: shared/benchmarks.c CreateJointGrid, 100 x 100 circles on revolute joints
function CreateJointGrid(scene: Scene): void {
	let worldId = scene.worldId;
	b2World_EnableSleeping(worldId, false);

	let N = 100;

	let bodies = new StaticArray<b2BodyId>(N * N);
	let index = 0;

	let shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = <u64>~(<u32>2);

	let circle = b2Circle(b2Vec2(0.0, 0.0), 0.4);

	let jointDef = b2DefaultRevoluteJointDef();
	jointDef.base.drawScale = 0.4;

	let bodyDef = b2DefaultBodyDef();

	for (let k = 0; k < N; ++k) {
		for (let i = 0; i < N; ++i) {
			let fk: f32 = <f32>k;
			let fi: f32 = <f32>i;

			if (k >= N / 2 - 3 && k <= N / 2 + 3 && i == 0) {
				bodyDef.type = b2_staticBody;
			} else {
				bodyDef.type = b2_dynamicBody;
			}

			bodyDef.position = b2Vec2(fk, -fi);

			let body = Scene_AddBody(scene, bodyDef);

			b2CreateCircleShape(body, shapeDef, circle);

			if (i > 0) {
				jointDef.base.bodyIdA = bodies[index - 1];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = b2Vec2(0.0, -0.5);
				jointDef.base.localFrameB.p = b2Vec2(0.0, 0.5);
				b2CreateRevoluteJoint(worldId, jointDef);
			}

			if (k > 0) {
				jointDef.base.bodyIdA = bodies[index - N];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = b2Vec2(0.5, 0.0);
				jointDef.base.localFrameB.p = b2Vec2(-0.5, 0.0);
				b2CreateRevoluteJoint(worldId, jointDef);
			}

			bodies[index++] = body;
		}
	}
}

// rain: shared/benchmarks.c CreateRain and StepRain, 5 rows of 40 columns of 5 humans dropped onto
// rows of boxes, then from step 320 the oldest column replaced every 8 steps
function CreateRain(scene: Scene): void {
	let boneCount = RAIN_ROW_COUNT * RAIN_COLUMN_COUNT * RAIN_GROUP_SIZE * SCENE_BONE_COUNT;
	scene.rain.boneBodies = new StaticArray<b2BodyId>(boneCount);
	scene.rain.boneJoints = new StaticArray<b2JointId>(boneCount);

	scene.rain.gridSize = 0.5;
	scene.rain.gridCount = 500;

	{
		let bodyDef = b2DefaultBodyDef();
		let groundId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		let y: f32 = 0.0;
		let width: f32 = scene.rain.gridSize;
		let height: f32 = scene.rain.gridSize;

		for (let i = 0; i < RAIN_ROW_COUNT; ++i) {
			let x: f32 = -0.5 * <f32>scene.rain.gridCount * scene.rain.gridSize;
			for (let j = 0; j <= scene.rain.gridCount; ++j) {
				let box = b2MakeOffsetBox(0.5 * width, 0.5 * height, b2Vec2(x, y), b2Rot_identity);
				b2CreatePolygonShape(groundId, shapeDef, box);

				x += scene.rain.gridSize;
			}

			y += 45.0;
		}
	}

	scene.rain.columnCount = 0;
	scene.rain.columnIndex = 0;
}

// shared/benchmarks.c CreateGroup
function Scene_CreateRainGroup(scene: Scene, rowIndex: i32, columnIndex: i32): void {
	let groupIndex = rowIndex * RAIN_COLUMN_COUNT + columnIndex;

	let span: f32 = <f32>scene.rain.gridCount * scene.rain.gridSize;
	let groupDistance: f32 = (1.0 * span) / <f32>RAIN_COLUMN_COUNT;

	let position: b2Pos = b2Pos_zero;
	position.x = -0.5 * span + groupDistance * (<f32>columnIndex + 0.5);
	position.y = 40.0 + 45.0 * <f32>rowIndex;

	let scale: f32 = 1.0;
	let jointFriction: f32 = 0.05;
	let jointHertz: f32 = 5.0;
	let jointDamping: f32 = 0.5;

	for (let i = 0; i < RAIN_GROUP_SIZE; ++i) {
		let boneSlot = (groupIndex * RAIN_GROUP_SIZE + i) * SCENE_BONE_COUNT;
		Scene_CreateHuman(scene, position, scale, jointFriction, jointHertz, jointDamping, i + 1, boneSlot);
		position.x += 0.5;
	}
}

// shared/human.c DestroyHuman
function Scene_DestroyHuman(scene: Scene, boneSlot: i32): void {
	for (let i = 0; i < SCENE_BONE_COUNT; ++i) {
		if (scene.rain.boneJoints[boneSlot + i].index1 == 0) {
			continue;
		}

		b2DestroyJoint(scene.rain.boneJoints[boneSlot + i]);
		scene.rain.boneJoints[boneSlot + i] = b2_nullJointId;
	}

	for (let i = 0; i < SCENE_BONE_COUNT; ++i) {
		if (scene.rain.boneBodies[boneSlot + i].index1 == 0) {
			continue;
		}

		b2DestroyBody(scene.rain.boneBodies[boneSlot + i]);
		scene.rain.boneBodies[boneSlot + i] = b2_nullBodyId;
	}
}

// shared/benchmarks.c DestroyGroup
function Scene_DestroyRainGroup(scene: Scene, rowIndex: i32, columnIndex: i32): void {
	let groupIndex = rowIndex * RAIN_COLUMN_COUNT + columnIndex;

	for (let i = 0; i < RAIN_GROUP_SIZE; ++i) {
		Scene_DestroyHuman(scene, (groupIndex * RAIN_GROUP_SIZE + i) * SCENE_BONE_COUNT);
	}
}

function StepRain(scene: Scene, stepIndex: i32): void {
	let delay = 0x7;

	if ((stepIndex & delay) == 0) {
		if (scene.rain.columnCount < RAIN_COLUMN_COUNT) {
			for (let i = 0; i < RAIN_ROW_COUNT; ++i) {
				Scene_CreateRainGroup(scene, i, scene.rain.columnCount);
			}

			scene.rain.columnCount += 1;
		} else {
			for (let i = 0; i < RAIN_ROW_COUNT; ++i) {
				Scene_DestroyRainGroup(scene, i, scene.rain.columnIndex);
				Scene_CreateRainGroup(scene, i, scene.rain.columnIndex);
			}

			scene.rain.columnIndex = (scene.rain.columnIndex + 1) % RAIN_COLUMN_COUNT;
		}
	}

	Scene_WorldStep(scene);
}

// smash: shared/benchmarks.c CreateSmash, a heavy box into a sleeping 120 x 80 block, no gravity
function CreateSmash(scene: Scene): void {
	b2World_SetGravity(scene.worldId, b2Vec2_zero);

	{
		let box = b2MakeBox(4.0, 4.0);

		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(-20.0, 0.0);
		bodyDef.linearVelocity = b2Vec2(40.0, 0.0);
		let bodyId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.density = 8.0;
		b2CreatePolygonShape(bodyId, shapeDef, box);
	}

	let d: f32 = 0.4;
	let box = b2MakeSquare(0.5 * d);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.isAwake = false;

	let shapeDef = b2DefaultShapeDef();

	let columns = 120;
	let rows = 80;

	for (let i = 0; i < columns; ++i) {
		for (let j = 0; j < rows; ++j) {
			bodyDef.position.x = <f32>i * d + 30.0;
			bodyDef.position.y = (<f32>j - <f32>rows / 2.0) * d;
			let bodyId = Scene_AddBody(scene, bodyDef);
			b2CreatePolygonShape(bodyId, shapeDef, box);
		}
	}
}

// spinner: shared/benchmarks.c CreateSpinner and StepSpinner, a motorized paddle in a loop chain
// stirring 6076 capsules, circles and boxes. keyJoints: 0 spinner
function CreateSpinner(scene: Scene): void {
	let worldId = scene.worldId;

	let groundId = b2_nullBodyId;
	{
		let bodyDef = b2DefaultBodyDef();
		groundId = Scene_AddBody(scene, bodyDef);

		let points = new StaticArray<b2Vec2>(SPINNER_POINT_COUNT);

		let q = b2MakeRot((-2.0 * B2_PI) / <f32>SPINNER_POINT_COUNT);
		let p = b2Vec2(40.0, 0.0);
		for (let i = 0; i < SPINNER_POINT_COUNT; ++i) {
			points[i] = b2Vec2(p.x, p.y + 32.0);
			p = b2RotateVector(q, p);
		}

		let materials = new StaticArray<b2SurfaceMaterial>(1);
		materials[0] = b2SurfaceMaterial();
		materials[0].friction = 0.1;

		let chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = SPINNER_POINT_COUNT;
		chainDef.isLoop = true;
		chainDef.materials = materials;
		chainDef.materialCount = 1;

		b2CreateChain(groundId, chainDef);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(0.0, 12.0);
		bodyDef.enableSleep = false;

		let spinnerId = Scene_AddBody(scene, bodyDef);

		let box = b2MakeRoundedBox(0.4, 20.0, 0.2);
		let shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0;
		b2CreatePolygonShape(spinnerId, shapeDef, box);

		let motorSpeed: f32 = 5.0;
		let maxMotorTorque: f32 = FLT_MAX;
		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = spinnerId;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint(groundId, bodyDef.position);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = motorSpeed;
		jointDef.maxMotorTorque = maxMotorTorque;

		scene.keyJoints[0] = b2CreateRevoluteJoint(worldId, jointDef);
	}

	let capsule = b2Capsule(b2Vec2(-0.25, 0.0), b2Vec2(0.25, 0.0), 0.25);
	let circle = b2Circle(b2Vec2(0.0, 0.0), 0.35);
	let square = b2MakeSquare(0.35);

	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	let shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.1;
	shapeDef.material.restitution = 0.1;
	shapeDef.density = 0.25;

	let bodyCount = 2 * 3038;

	let x: f32 = -23.0;
	let y: f32 = 2.0;
	for (let i = 0; i < bodyCount; ++i) {
		bodyDef.position = b2Vec2(x, y);
		let bodyId = Scene_AddBody(scene, bodyDef);

		let remainder = i % 3;
		if (remainder == 0) {
			b2CreateCapsuleShape(bodyId, shapeDef, capsule);
		} else if (remainder == 1) {
			b2CreateCircleShape(bodyId, shapeDef, circle);
		} else if (remainder == 2) {
			b2CreatePolygonShape(bodyId, shapeDef, square);
		}

		x += 0.5;

		if (x >= 23.0) {
			x = -23.0;
			y += 0.5;
		}
	}
}

// StepSpinner's angle, read before the step as the benchmark does
function StepSpinner(scene: Scene): void {
	Probe_Float(scene, b2RevoluteJoint_GetCurrentAngle(scene.keyJoints[0]));
	Scene_WorldStep(scene);
}

// tumbler: shared/benchmarks.c CreateTumbler, a motorized box of four walls turning 2025 small boxes
function CreateTumbler(scene: Scene): void {
	let worldId = scene.worldId;

	let groundId = b2_nullBodyId;
	{
		let bodyDef = b2DefaultBodyDef();
		groundId = Scene_AddBody(scene, bodyDef);
	}

	{
		let bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2(0.0, 10.0);
		let bodyId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b2DefaultShapeDef();
		shapeDef.density = 50.0;

		let polygon = b2MakeOffsetBox(0.5, 10.0, b2Vec2(10.0, 0.0), b2Rot_identity);
		b2CreatePolygonShape(bodyId, shapeDef, polygon);
		polygon = b2MakeOffsetBox(0.5, 10.0, b2Vec2(-10.0, 0.0), b2Rot_identity);
		b2CreatePolygonShape(bodyId, shapeDef, polygon);
		polygon = b2MakeOffsetBox(10.0, 0.5, b2Vec2(0.0, 10.0), b2Rot_identity);
		b2CreatePolygonShape(bodyId, shapeDef, polygon);
		polygon = b2MakeOffsetBox(10.0, 0.5, b2Vec2(0.0, -10.0), b2Rot_identity);
		b2CreatePolygonShape(bodyId, shapeDef, polygon);

		let motorSpeed: f32 = 25.0;

		let jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = b2Vec2(0.0, 10.0);
		jointDef.base.localFrameB.p = b2Vec2(0.0, 0.0);
		jointDef.motorSpeed = (B2_PI / 180.0) * motorSpeed;
		jointDef.maxMotorTorque = 1e8;
		jointDef.enableMotor = true;

		b2CreateRevoluteJoint(worldId, jointDef);
	}

	let gridCount = 45;

	let polygon = b2MakeBox(0.125, 0.125);
	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	let shapeDef = b2DefaultShapeDef();

	let y: f32 = -0.2 * <f32>gridCount + 10.0;
	for (let i = 0; i < gridCount; ++i) {
		let x: f32 = -0.2 * <f32>gridCount;

		for (let j = 0; j < gridCount; ++j) {
			bodyDef.position = b2Vec2(x, y);
			let bodyId = Scene_AddBody(scene, bodyDef);

			b2CreatePolygonShape(bodyId, shapeDef, polygon);

			x += 0.4;
		}

		y += 0.4;
	}
}

// The benchmarks' world capacities at release sizes; zero for the other scenes
function Scene_GetCapacity(index: i32): b2Capacity {
	let c = b2Capacity();
	if (index == SCENE_LARGE_PYRAMID) {
		c.staticShapeCount = 1;
		c.dynamicShapeCount = 5600;
		c.staticBodyCount = 1;
		c.dynamicBodyCount = 5600;
		c.contactCount = 19000;
		c.arenaByteCount = 2800 * 1024;
	} else if (index == SCENE_JOINT_GRID) {
		c.staticShapeCount = 7;
		c.dynamicShapeCount = 11000;
		c.staticBodyCount = 7;
		c.dynamicBodyCount = 11000;
		c.arenaByteCount = 120 * 1024;
	} else if (index == SCENE_RAIN) {
		c.staticShapeCount = 2800;
		c.dynamicShapeCount = 15000;
		c.staticBodyCount = 1;
		c.dynamicBodyCount = 13000;
		c.contactCount = 29000;
		c.arenaByteCount = 1900 * 1024;
	} else if (index == SCENE_SMASH) {
		c.dynamicShapeCount = 11000;
		c.dynamicBodyCount = 11000;
		c.contactCount = 55000;
		c.arenaByteCount = 4900 * 1024;
	} else if (index == SCENE_SPINNER) {
		c.staticShapeCount = 400;
		c.dynamicShapeCount = 6700;
		c.staticBodyCount = 1;
		c.dynamicBodyCount = 6700;
		c.contactCount = 41000;
		c.arenaByteCount = 4600 * 1024;
	} else if (index == SCENE_TUMBLER) {
		c.dynamicShapeCount = 2300;
		c.staticBodyCount = 1;
		c.dynamicBodyCount = 2300;
		c.contactCount = 13000;
		c.arenaByteCount = 1100 * 1024;
	}
	return c;
}

// Returns false for an unknown scene name
export function Scene_Create(scene: Scene, name: string, workerCount: i32): bool {
	let index = -1;
	for (let i = 0; i < Scene_GetCount(); ++i) {
		if (name == s_sceneNames[i]) {
			index = i;
		}
	}
	if (index < 0) {
		return false;
	}

	scene.name = s_sceneNames[index];
	scene.index = index;

	let worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	worldDef.capacity = Scene_GetCapacity(index);
	scene.worldId = b2CreateWorld(worldDef);

	if (index == SCENE_PYRAMID) {
		CreatePyramid(scene);
	} else if (index == SCENE_SHAPE_RAIN) {
		CreateShapeRain(scene);
	} else if (index == SCENE_CHAIN_CAR) {
		CreateChainCar(scene);
	} else if (index == SCENE_JOINTS) {
		CreateJoints(scene);
	} else if (index == SCENE_DYNAMIC_MOVER) {
		CreateDynamicMover(scene);
	} else if (index == SCENE_CHURN) {
		CreateChurn(scene);
	} else if (index == SCENE_BULLETS) {
		CreateBullets(scene);
	} else if (index == SCENE_SENSORS) {
		CreateSensors(scene);
	} else if (index == SCENE_EVENTS) {
		CreateEvents(scene);
	} else if (index == SCENE_QUERIES) {
		CreateQueries(scene);
	} else if (index == SCENE_MOVER) {
		CreateMover(scene);
	} else if (index == SCENE_MANY_BODIES) {
		CreateManyBodies(scene);
	} else if (index == SCENE_LARGE_PYRAMID) {
		CreateLargePyramid(scene);
	} else if (index == SCENE_JOINT_GRID) {
		CreateJointGrid(scene);
	} else if (index == SCENE_RAIN) {
		CreateRain(scene);
	} else if (index == SCENE_SMASH) {
		CreateSmash(scene);
	} else if (index == SCENE_SPINNER) {
		CreateSpinner(scene);
	} else if (index == SCENE_TUMBLER) {
		CreateTumbler(scene);
	}

	return true;
}

// Scripted input for this step, b2World_Step at 60 Hz with 4 sub-steps, then the probes
export function Scene_Step(scene: Scene, stepIndex: i32): void {
	scene.probes.length = 0;

	let index = scene.index;
	if (index == SCENE_SHAPE_RAIN) {
		StepShapeRain(scene, stepIndex);
	} else if (index == SCENE_CHAIN_CAR) {
		StepChainCar(scene, stepIndex);
	} else if (index == SCENE_JOINTS) {
		StepJoints(scene, stepIndex);
	} else if (index == SCENE_DYNAMIC_MOVER) {
		StepDynamicMover(scene, stepIndex);
	} else if (index == SCENE_CHURN) {
		StepChurn(scene, stepIndex);
	} else if (index == SCENE_BULLETS) {
		StepBullets(scene, stepIndex);
	} else if (index == SCENE_SENSORS) {
		StepSensors(scene, stepIndex);
	} else if (index == SCENE_EVENTS) {
		StepEvents(scene, stepIndex);
	} else if (index == SCENE_QUERIES) {
		StepQueries(scene, stepIndex);
	} else if (index == SCENE_MOVER) {
		StepMover(scene, stepIndex);
	} else if (index == SCENE_RAIN) {
		StepRain(scene, stepIndex);
	} else if (index == SCENE_SPINNER) {
		StepSpinner(scene);
	} else {
		Scene_WorldStep(scene);
	}
}

export function Scene_Destroy(scene: Scene): void {
	b2DestroyWorld(scene.worldId);
	scene.bodies = new Array<b2BodyId>(0);
	scene.bodyCount = 0;
	scene.probes = new Array<u32>(0);
	scene.rain = new SceneRain();
}
