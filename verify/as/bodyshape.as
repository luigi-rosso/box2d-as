import { vtag, vi32, vu32, vf32, vbool, vend } from "./common";
import { b2AABB, b2MakeRot, b2Transform, b2Vec2 } from "../../lib/math_functions";
import { b2Capsule, b2Circle, b2MassData, b2Polygon, b2Segment } from "../../lib/collision";
import { b2BodyId, b2ShapeId, b2WorldId } from "../../lib/id";
import {
	b2DefaultBodyDef,
	b2DefaultChainDef,
	b2DefaultFilter,
	b2DefaultShapeDef,
	b2DefaultSurfaceMaterial,
	b2DefaultWorldDef,
	b2MotionLocks,
	b2SurfaceMaterial,
	b2_dynamicBody,
	b2_kinematicBody,
	b2_staticBody,
} from "../../lib/types";
import { b2MakeBox, b2MakeOffsetRoundedBox, b2MakeRoundedBox, b2MakeSquare } from "../../lib/geometry";
import { b2Body_IsValid, b2CreateWorld, b2DestroyWorld, b2GetWorldFromId } from "../../lib/physics_world";
import {
	b2Body_ApplyAngularImpulse,
	b2Body_ApplyForce,
	b2Body_ApplyLinearImpulse,
	b2Body_ApplyTorque,
	b2Body_ComputeBounds,
	b2Body_Disable,
	b2Body_Enable,
	b2Body_EnableSleeping,
	b2Body_GetAngularVelocity,
	b2Body_GetLinearVelocity,
	b2Body_GetLocalPoint,
	b2Body_GetLocalPointVelocity,
	b2Body_GetMaxExtentFromOrigin,
	b2Body_GetName,
	b2Body_GetShapes,
	b2Body_GetWorldPoint,
	b2Body_GetWorldPointVelocity,
	b2Body_SetAwake,
	b2Body_SetBullet,
	b2Body_SetLinearVelocity,
	b2Body_SetMassData,
	b2Body_SetMotionLocks,
	b2Body_SetTargetTransform,
	b2Body_SetTransform,
	b2Body_SetType,
	b2Body_UpdateMassFromShapes,
	b2CreateBody,
	b2DestroyBody,
	b2GetBodySim,
} from "../../lib/body";
import {
	b2Chain_GetSegmentCount,
	b2Chain_GetSurfaceMaterial,
	b2Chain_SetSurfaceMaterial,
	b2CreateCapsuleShape,
	b2CreateChain,
	b2CreateCircleShape,
	b2CreatePolygonShape,
	b2CreateSegmentShape,
	b2DestroyChain,
	b2DestroyShape,
	b2Shape_ApplyWind,
	b2Shape_CastRay,
	b2Shape_ComputeMassData,
	b2Shape_GetBounds,
	b2Shape_GetClosestPoint,
	b2Shape_SetCapsule,
	b2Shape_SetDensity,
	b2Shape_SetFilter,
	b2Shape_SetFriction,
	b2Shape_SetPolygon,
	b2Shape_SetSegment,
	b2Shape_TestPoint,
} from "../../lib/shape";

export { frameBoundary } from "./common";

let worldId = b2WorldId();
let ids = new StaticArray<b2BodyId>(16);
let idCount = 0;

function vvec(v: b2Vec2): void {
	vf32(v.x);
	vf32(v.y);
}

function vaabb(a: b2AABB): void {
	vvec(a.lowerBound);
	vvec(a.upperBound);
}

function emitShape(id: b2ShapeId): void {
	let world = b2GetWorldFromId(worldId);
	let i = id.index1 - 1;
	let shapes = world.shapes;
	vtag("s");
	vi32(id.index1);
	vi32(shapes[i].type);
	vaabb(shapes[i].aabb);
	vaabb(world.fatAABBs[i]);
	vf32(shapes[i].aabbMargin);
	vi32(shapes[i].proxyKey);
	vvec(shapes[i].localCentroid);
	vi32(shapes[i].sensorIndex);
	vi32(shapes[i].prevShapeId);
	vi32(shapes[i].nextShapeId);
	vf32(shapes[i].density);
	vf32(shapes[i].material.friction);
	vu32(<u32>shapes[i].filter.categoryBits);
	vend();
}

function emitBody(id: b2BodyId): void {
	let world = b2GetWorldFromId(worldId);
	let b = id.index1 - 1;
	let bodies = world.bodies;
	let sim = b2GetBodySim(world, b);
	vtag("b");
	vi32(id.index1);
	vi32(bodies[b].setIndex);
	vi32(bodies[b].localIndex);
	vi32(bodies[b].islandId >= 0 ? 1 : 0);
	vu32(bodies[b].flags);
	vu32(sim.flags);
	vf32(bodies[b].mass);
	vf32(bodies[b].inertia);
	vf32(sim.invMass);
	vf32(sim.invInertia);
	vvec(sim.localCenter);
	vvec(sim.center);
	vvec(sim.transform.p);
	vf32(sim.transform.q.c);
	vf32(sim.transform.q.s);
	vf32(sim.minExtent);
	vf32(sim.maxExtent);
	vvec(sim.force);
	vf32(sim.torque);
	vvec(b2Body_GetLinearVelocity(id));
	vf32(b2Body_GetAngularVelocity(id));
	vi32(bodies[b].shapeCount);
	vaabb(b2Body_ComputeBounds(id));
	vend();

	let shapes = new StaticArray<b2ShapeId>(32);
	let n = b2Body_GetShapes(id, shapes, 32);
	for (let i = 0; i < n; ++i) {
		emitShape(shapes[i]);
	}
}

function emitAll(tag: string): void {
	vtag(tag);
	vend();
	for (let i = 0; i < idCount; ++i) {
		if (b2Body_IsValid(ids[i])) {
			emitBody(ids[i]);
		}
	}
}

function queries(): void {
	for (let i = 0; i < idCount; ++i) {
		if (b2Body_IsValid(ids[i]) == false) {
			continue;
		}

		let shapes = new StaticArray<b2ShapeId>(32);
		let n = b2Body_GetShapes(ids[i], shapes, 32);
		for (let j = 0; j < n; ++j) {
			let s = shapes[j];
			vtag("q");
			vbool(b2Shape_TestPoint(s, b2Vec2(1.0, 2.0)));
			vbool(b2Shape_TestPoint(s, b2Vec2(0.0, 0.0)));
			let o = b2Shape_CastRay(s, b2Vec2(-3.0, 2.1), b2Vec2(8.0, -0.5));
			vbool(o.hit);
			vvec(o.point);
			vvec(o.normal);
			vf32(o.fraction);
			o = b2Shape_CastRay(s, b2Vec2(1.0, 6.0), b2Vec2(0.2, -8.0));
			vbool(o.hit);
			vvec(o.point);
			vvec(o.normal);
			vf32(o.fraction);
			vvec(b2Shape_GetClosestPoint(s, b2Vec2(4.0, 4.0)));
			let md = b2Shape_ComputeMassData(s);
			vf32(md.mass);
			vvec(md.center);
			vf32(md.rotationalInertia);
			vaabb(b2Shape_GetBounds(s));
			vend();
		}
	}
}

function push(id: b2BodyId): void {
	let a = ids;
	a[idCount] = id;
	idCount += 1;
}

export function run(): void {
	let wd = b2DefaultWorldDef();
	worldId = b2CreateWorld(wd);

	let bd = b2DefaultBodyDef();
	bd.name = "ground";
	let ground = b2CreateBody(worldId, bd);
	push(ground);
	let sd = b2DefaultShapeDef();
	let box = b2MakeBox(5.0, 0.5);
	b2CreatePolygonShape(ground, sd, box);
	let seg = b2Segment();
	seg.point1 = b2Vec2(-6.0, 1.0);
	seg.point2 = b2Vec2(6.0, 1.5);
	b2CreateSegmentShape(ground, sd, seg);

	bd = b2DefaultBodyDef();
	bd.position = b2Vec2(0.0, -4.0);
	let chainBody = b2CreateBody(worldId, bd);
	push(chainBody);
	let loopPoints = new StaticArray<b2Vec2>(5);
	loopPoints[0] = b2Vec2(-3.0, 0.0);
	loopPoints[1] = b2Vec2(3.0, 0.0);
	loopPoints[2] = b2Vec2(3.0, 2.0);
	loopPoints[3] = b2Vec2(0.0, 3.0);
	loopPoints[4] = b2Vec2(-3.0, 2.0);
	let mats = new StaticArray<b2SurfaceMaterial>(5);
	for (let i = 0; i < 5; ++i) {
		mats[i] = b2DefaultSurfaceMaterial();
		mats[i].friction = 0.1 * <f32>(i + 1);
	}
	let cd = b2DefaultChainDef();
	cd.points = loopPoints;
	cd.pointCount = 5;
	cd.materials = mats;
	cd.materialCount = 5;
	cd.isLoop = true;
	let loop = b2CreateChain(chainBody, cd);

	let openPoints = new StaticArray<b2Vec2>(4);
	openPoints[0] = b2Vec2(-8.0, -6.0);
	openPoints[1] = b2Vec2(-4.0, -6.5);
	openPoints[2] = b2Vec2(0.0, -6.0);
	openPoints[3] = b2Vec2(4.0, -7.0);
	cd = b2DefaultChainDef();
	cd.points = openPoints;
	cd.pointCount = 4;
	cd.ghost1 = b2Vec2(-10.0, -5.0);
	cd.ghost2 = b2Vec2(6.0, -6.0);
	let open = b2CreateChain(chainBody, cd);

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = b2Vec2(1.0, 2.0);
	bd.rotation = b2MakeRot(0.3);
	bd.angularVelocity = 0.7;
	bd.linearVelocity = b2Vec2(0.5, -0.25);
	bd.name = "averyveryverylongname";
	let a = b2CreateBody(worldId, bd);
	push(a);
	let circle = b2Circle(b2Vec2(0.1, 0.2), 0.5);
	let circleId = b2CreateCircleShape(a, sd, circle);
	let capsule = b2Capsule(b2Vec2(-0.5, 0.0), b2Vec2(0.5, 0.1), 0.25);
	sd.density = 2.5;
	let capsuleId = b2CreateCapsuleShape(a, sd, capsule);
	let rbox = b2MakeOffsetRoundedBox(0.3, 0.4, b2Vec2(0.2, -0.1), b2MakeRot(0.5), 0.05);
	sd.density = 0.7;
	let polyId = b2CreatePolygonShape(a, sd, rbox);
	sd = b2DefaultShapeDef();
	sd.isSensor = true;
	sd.enableSensorEvents = true;
	let sensorCircle = b2Circle(b2Vec2(0.0, 0.0), 1.5);
	let sensorId = b2CreateCircleShape(a, sd, sensorCircle);
	sd = b2DefaultShapeDef();

	bd = b2DefaultBodyDef();
	bd.type = b2_kinematicBody;
	bd.position = b2Vec2(-2.0, 3.0);
	bd.angularVelocity = -0.4;
	let k = b2CreateBody(worldId, bd);
	push(k);
	let kbox = b2MakeBox(1.0, 0.75);
	b2CreatePolygonShape(k, sd, kbox);

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = b2Vec2(3.0, 5.0);
	bd.isAwake = false;
	let c = b2CreateBody(worldId, bd);
	push(c);
	let cc = b2Circle(b2Vec2(0.0, 0.0), 0.3);
	b2CreateCircleShape(c, sd, cc);

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = b2Vec2(-4.0, 5.0);
	bd.isEnabled = false;
	bd.motionLocks.angularZ = true;
	let d = b2CreateBody(worldId, bd);
	push(d);
	sd.updateBodyMass = false;
	let dbox = b2MakeSquare(0.4);
	b2CreatePolygonShape(d, sd, dbox);
	sd = b2DefaultShapeDef();

	emitAll("create");
	queries();

	vtag("name");
	vtag(b2Body_GetName(a));
	vtag(b2Body_GetName(ground));
	vi32(b2Chain_GetSegmentCount(loop));
	vf32(b2Chain_GetSurfaceMaterial(loop, 3).friction);
	vend();

	b2Body_SetTransform(a, b2Vec2(1.5, 2.5), b2MakeRot(-0.2));
	b2Body_ApplyLinearImpulse(a, b2Vec2(0.3, 0.6), b2Vec2(1.7, 2.2), true);
	b2Body_ApplyAngularImpulse(a, 0.25, true);
	b2Body_ApplyForce(a, b2Vec2(3.0, -1.0), b2Vec2(2.0, 2.0), true);
	b2Body_ApplyTorque(a, 1.5, true);
	b2Shape_ApplyWind(capsuleId, b2Vec2(4.0, 1.0), 1.0, 0.5, true);
	b2Shape_ApplyWind(polyId, b2Vec2(-2.0, 3.0), 1.0, 0.25, true);
	b2Shape_ApplyWind(circleId, b2Vec2(1.0, 0.0), 2.0, 0.0, true);
	emitAll("moved");

	vtag("pv");
	vvec(b2Body_GetLocalPointVelocity(a, b2Vec2(0.5, 0.5)));
	vvec(b2Body_GetWorldPointVelocity(a, b2Vec2(2.0, 3.0)));
	vvec(b2Body_GetWorldPoint(a, b2Vec2(0.5, 0.5)));
	vvec(b2Body_GetLocalPoint(a, b2Vec2(0.5, 0.5)));
	vf32(b2Body_GetMaxExtentFromOrigin(a));
	vend();

	let md = b2MassData();
	md.mass = 3.0;
	md.center = b2Vec2(0.1, -0.2);
	md.rotationalInertia = 0.9;
	b2Body_SetMassData(a, md);
	emitAll("massdata");
	b2Body_UpdateMassFromShapes(a);
	let locks = b2MotionLocks();
	locks.linearY = true;
	locks.angularZ = true;
	b2Body_SetMotionLocks(a, locks);
	emitAll("locks");
	locks = b2MotionLocks();
	b2Body_SetMotionLocks(a, locks);

	b2Body_SetType(a, b2_staticBody);
	emitAll("static");
	b2Body_SetType(a, b2_kinematicBody);
	emitAll("kinematic");
	b2Body_SetType(a, b2_dynamicBody);
	emitAll("dynamic");

	b2Body_Disable(a);
	emitAll("disabled");
	b2Body_Enable(a);
	b2Body_Enable(d);
	b2Body_UpdateMassFromShapes(d);
	emitAll("enabled");

	b2Body_SetTargetTransform(c, b2Transform(b2Vec2(3.5, 4.5), b2MakeRot(0.1)), <f32>1.0 / <f32>60.0, true);
	b2Body_SetLinearVelocity(k, b2Vec2(1.0, 0.0));
	b2Body_SetBullet(a, true);
	b2Body_EnableSleeping(c, false);
	emitAll("target");

	b2Shape_SetDensity(polyId, 3.0, true);
	let np = b2MakeBox(0.2, 0.6);
	b2Shape_SetPolygon(circleId, np);
	let nc = b2Capsule(b2Vec2(0.0, -0.3), b2Vec2(0.0, 0.3), 0.1);
	b2Shape_SetCapsule(polyId, nc);
	let ns = b2Segment();
	ns.point1 = b2Vec2(-1.0, 0.0);
	ns.point2 = b2Vec2(1.0, 0.2);
	b2Shape_SetSegment(capsuleId, ns);
	let f = b2DefaultFilter();
	f.categoryBits = 4;
	b2Shape_SetFilter(polyId, f);
	b2Shape_SetFriction(polyId, 0.9);
	b2Body_UpdateMassFromShapes(a);
	let m = b2DefaultSurfaceMaterial();
	m.friction = 0.33;
	b2Chain_SetSurfaceMaterial(loop, m, 1);
	emitAll("setters");
	queries();

	b2DestroyShape(sensorId, true);
	b2DestroyChain(open);
	emitAll("destroyed");
	b2DestroyBody(a);
	b2DestroyBody(k);

	bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = b2Vec2(0.0, 8.0);
	let e = b2CreateBody(worldId, bd);
	push(e);
	let ebox = b2MakeRoundedBox(0.5, 0.5, 0.1);
	b2CreatePolygonShape(e, sd, ebox);
	vtag("reuse");
	vi32(e.index1);
	vu32(e.generation);
	vend();
	emitAll("final");
	queries();

	b2Body_SetAwake(e, false);
	emitAll("sleep");
	b2Body_SetAwake(e, true);
	emitAll("wake");

	b2DestroyWorld(worldId);
}
