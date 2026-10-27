// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/math_functions.as, lib/core.as, lib/constants.as, lib/id.as, lib/types.as and
// lib/collision.as, mirrors verify/cref/math.c

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrand32, vrand64, vrandint, vrandf, vspecialf } from "./common";
import {
	B2_ALIGNMENT,
	B2_HASH_INIT,
	B2_NULL_INDEX,
	B2_SECRET_COOKIE,
	B2_VERSION_MAJOR,
	B2_VERSION_MINOR,
	B2_VERSION_REVISION,
	FLT_EPSILON,
	FLT_MAX,
	FLT_MIN,
	INFINITY,
	b2FloatToIntSat,
	b2FloatToUIntSat,
	b2GetLengthUnitsPerMeter,
	b2GetVersion,
	b2Hash,
	b2IsDoublePrecision,
	b2SetLengthUnitsPerMeter,
	b2SetAssertCallback,
	b2SetLogCallback,
	b2_lengthUnitsPerMeter,
} from "../../lib/core";
import {
	B2_AABB_MARGIN_FRACTION,
	B2_CONTACT_RECYCLE_COS_ANGLE,
	B2_CONTACT_RECYCLE_DISTANCE,
	B2_GRAPH_COLOR_COUNT,
	B2_HUGE,
	B2_LINEAR_SLOP,
	B2_MAX_AABB_MARGIN,
	B2_MAX_RESTITUTION_ITERATIONS,
	B2_MAX_ROTATION,
	B2_MAX_TASKS,
	B2_MAX_WORKERS,
	B2_MAX_WORLDS,
	B2_NAME_LENGTH,
	B2_SPECULATIVE_DISTANCE,
	B2_TIME_TO_SLEEP,
} from "../../lib/constants";
import {
	B2_PI,
	b2AABB,
	b2AABB_Center,
	b2AABB_Contains,
	b2AABB_Extents,
	b2AABB_Overlaps,
	b2AABB_Union,
	b2Abs,
	b2Add,
	b2Atan2,
	b2Clamp,
	b2ClampFloat,
	b2ClampInt,
	b2CeilingInt,
	b2ComputeAngularVelocity,
	b2ComputeCosSin,
	b2ComputeRotationBetweenUnitVectors,
	b2Cross,
	b2CrossSV,
	b2CrossVS,
	b2Distance,
	b2DistanceSquared,
	b2Dot,
	b2GetInverse22,
	b2GetLengthAndNormalize,
	b2IntegrateRotation,
	b2InvMulRot,
	b2InvMulTransforms,
	b2InvMulWorldTransforms,
	b2InvRotateVector,
	b2InvTransformPoint,
	b2InvTransformWorldPoint,
	b2InvertRot,
	b2IsNormalized,
	b2IsNormalizedRot,
	b2IsValidFloat,
	b2IsValidPlane,
	b2IsValidPos,
	b2IsValidRot,
	b2IsValidTransform,
	b2IsValidVec2,
	b2IsValidWorldTransform,
	b2LeftPerp,
	b2Length,
	b2LengthSquared,
	b2Lerp,
	b2LerpPos,
	b2MakeAABB,
	b2MakeRot,
	b2MakeRotFromUnitVector,
	b2MakeWorldTransform,
	b2Mat22,
	b2Mat22_zero,
	b2Max,
	b2MaxFloat,
	b2MaxInt,
	b2Min,
	b2MinFloat,
	b2MinInt,
	b2AbsFloat,
	b2AbsInt,
	b2Mul,
	b2MulAdd,
	b2MulMV,
	b2MulRot,
	b2MulSV,
	b2MulSub,
	b2MulTransforms,
	b2MulWorldTransforms,
	b2NLerp,
	b2Neg,
	b2Normalize,
	b2NormalizeRot,
	b2OffsetPos,
	b2Plane,
	b2PlaneSeparation,
	b2Pos_zero,
	b2RelativeAngle,
	b2RightPerp,
	b2Rot,
	b2Rot_GetAngle,
	b2Rot_GetXAxis,
	b2Rot_GetYAxis,
	b2Rot_identity,
	b2RotateVector,
	b2RoundDownFloat,
	b2RoundUpFloat,
	b2Solve22,
	b2SpringDamper,
	b2Sub,
	b2SubPos,
	b2ToPos,
	b2ToRelativeTransform,
	b2ToVec2,
	b2Transform,
	b2TransformPoint,
	b2TransformWorldPoint,
	b2Transform_identity,
	b2UnwindAngle,
	b2Vec2,
	b2Vec2_zero,
	b2WorldTransform_identity,
} from "../../lib/math_functions";
import { B2_MAX_POLYGON_VERTICES, b2ShapeProxy, b2_emptySimplexCache } from "../../lib/collision";
import {
	B2_DEFAULT_CATEGORY_BITS,
	B2_DEFAULT_MASK_BITS,
	b2DefaultBodyDef,
	b2DefaultChainDef,
	b2DefaultDistanceJointDef,
	b2DefaultExplosionDef,
	b2DefaultFilter,
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
	b2Filter,
	b2JointDef,
	b2SurfaceMaterial,
} from "../../lib/types";
import {
	b2BodyId,
	b2LoadBodyId,
	b2LoadChainId,
	b2LoadContactId,
	b2LoadJointId,
	b2LoadShapeId,
	b2LoadWorldId,
	b2StoreBodyId,
	b2StoreChainId,
	b2StoreContactId,
	b2StoreJointId,
	b2StoreShapeId,
	b2StoreWorldId,
	b2_nullBodyId,
	b2_nullChainId,
	b2_nullContactId,
	b2_nullJointId,
	b2_nullShapeId,
	b2_nullWorldId,
} from "../../lib/id";
import { b2GetSIMDWidth, b2IsAVX2Available, b2SetSIMDWidth } from "../../lib/simd";

export { frameBoundary } from "./common";

function silentLog(message: string): void {}

// The invalid unit inputs trip B2_CHECK_INPUT's assert in assert builds
function skipAssert(condition: string, fileName: string, lineNumber: i32): i32 {
	return 0;
}

function pv(v: b2Vec2): void {
	vf32(v.x);
	vf32(v.y);
}

function pr(q: b2Rot): void {
	vf32(q.c);
	vf32(q.s);
}

function pxf(t: b2Transform): void {
	pv(t.p);
	pr(t.q);
}

function pab(a: b2AABB): void {
	pv(a.lowerBound);
	pv(a.upperBound);
}

function pm(m: b2Mat22): void {
	pv(m.cx);
	pv(m.cy);
}

function ppl(p: b2Plane): void {
	pv(p.normal);
	vf32(p.offset);
}

// mode 0 special, 1 tiny, 2 large, 3 ordinary
function rcomp(mode: i32): f32 {
	if (mode == 0) {
		return vspecialf();
	}
	if (mode == 1) {
		return vrandf(-1.0e-3, 1.0e-3);
	}
	if (mode == 2) {
		return vrandf(-1.0e4, 1.0e4);
	}
	return vrandf(-10.0, 10.0);
}

function rscalar(): f32 {
	let mode = <i32>(vrand32() % 4);
	return rcomp(mode);
}

function rvec(): b2Vec2 {
	let mode = <i32>(vrand32() % 4);
	let v: b2Vec2;
	v.x = rcomp(mode);
	v.y = rcomp(mode);
	return v;
}

function rvecn(): b2Vec2 {
	let v: b2Vec2;
	v.x = vrandf(-10.0, 10.0);
	v.y = vrandf(-10.0, 10.0);
	return v;
}

function runit(): b2Vec2 {
	let v = rvecn();
	v = b2Normalize(v);
	if (b2LengthSquared(v) == 0.0) {
		v = b2Vec2(1.0, 0.0);
	}
	return v;
}

// Normalized rotations, raw component pairs and special values
function rrot(): b2Rot {
	let mode = vrand32() % 4;
	if (mode == 0) {
		let q: b2Rot;
		q.c = vspecialf();
		q.s = vspecialf();
		return q;
	}
	if (mode == 1) {
		let q: b2Rot;
		q.c = vrandf(-2.0, 2.0);
		q.s = vrandf(-2.0, 2.0);
		return q;
	}
	let angle = vrandf(-4.0, 4.0);
	return b2MakeRot(angle);
}

function rrotn(): b2Rot {
	let angle = vrandf(-4.0, 4.0);
	return b2MakeRot(angle);
}

function rxf(): b2Transform {
	let t: b2Transform;
	t.p = rvec();
	t.q = rrot();
	return t;
}

function raabb(): b2AABB {
	let mode = vrand32() % 6;
	let a: b2AABB;
	if (mode == 0) {
		a.lowerBound = rvec();
		a.upperBound = rvec();
		return a;
	}
	let c = rvecn();
	let e = b2Abs(rvecn());
	if (mode == 1) {
		e = b2Neg(e);
	}
	a.lowerBound = b2Sub(c, e);
	a.upperBound = b2Add(c, e);
	return a;
}

function rmat(): b2Mat22 {
	let mode = vrand32() % 5;
	let m: b2Mat22;
	if (mode == 0) {
		m.cx = rvec();
		m.cy = rvec();
		return m;
	}
	if (mode == 1) {
		m.cx = rvecn();
		let s = vrandf(-3.0, 3.0);
		m.cy = b2MulSV(s, m.cx);
		return m;
	}
	m.cx = rvecn();
	m.cy = rvecn();
	return m;
}

function rplane(): b2Plane {
	let p: b2Plane;
	let mode = vrand32() % 4;
	if (mode == 0) {
		p.normal = rvec();
		p.offset = vspecialf();
		return p;
	}
	p.normal = runit();
	p.offset = vrandf(-10.0, 10.0);
	return p;
}

function checkConstants(): void {
	vtag("const");
	vf32(B2_PI);
	vf32(FLT_MIN);
	vf32(FLT_MAX);
	vf32(FLT_EPSILON);
	vf32(INFINITY);
	vf32(B2_MAX_ROTATION);
	vf32(B2_CONTACT_RECYCLE_COS_ANGLE);
	vf32(B2_AABB_MARGIN_FRACTION);
	vf32(B2_TIME_TO_SLEEP);
	vend();

	vtag("iconst");
	vi32(B2_MAX_WORKERS);
	vi32(B2_MAX_TASKS);
	vi32(B2_GRAPH_COLOR_COUNT);
	vi32(B2_MAX_WORLDS);
	vi32(B2_NAME_LENGTH);
	vi32(B2_MAX_RESTITUTION_ITERATIONS);
	vi32(B2_MAX_POLYGON_VERTICES);
	vi32(B2_NULL_INDEX);
	vu32(B2_HASH_INIT);
	vi32(B2_SECRET_COOKIE);
	vi32(B2_ALIGNMENT);
	vu64(B2_DEFAULT_CATEGORY_BITS);
	vu64(B2_DEFAULT_MASK_BITS);
	vi32(B2_VERSION_MAJOR);
	vi32(B2_VERSION_MINOR);
	vi32(B2_VERSION_REVISION);
	vend();

	let version = b2GetVersion();
	vtag("version");
	vi32(version.major);
	vi32(version.minor);
	vi32(version.revision);
	vbool(b2IsDoublePrecision());
	vbool(b2IsAVX2Available());
	vi32(b2GetSIMDWidth());
	b2SetSIMDWidth(8);
	vi32(b2GetSIMDWidth());
	b2SetSIMDWidth(0);
	vi32(b2GetSIMDWidth());
	vend();

	vtag("statics");
	pv(b2Vec2_zero);
	pr(b2Rot_identity);
	pxf(b2Transform_identity);
	pm(b2Mat22_zero);
	pv(b2Pos_zero);
	pxf(b2WorldTransform_identity);
	vu32(b2_emptySimplexCache.count);
	vu32(b2_emptySimplexCache.indexA[0]);
	vu32(b2_emptySimplexCache.indexB[2]);
	vend();

	vtag("null");
	vu32(b2StoreWorldId(b2_nullWorldId));
	vu64(b2StoreBodyId(b2_nullBodyId));
	vu64(b2StoreShapeId(b2_nullShapeId));
	vu64(b2StoreChainId(b2_nullChainId));
	vu64(b2StoreJointId(b2_nullJointId));
	vi32(b2_nullContactId.index1);
	vu32(b2_nullContactId.generation);
	vend();
}

function checkUnitConstants(units: f32): void {
	b2SetLengthUnitsPerMeter(units);
	vtag("units");
	vf32(b2GetLengthUnitsPerMeter());
	vf32(b2_lengthUnitsPerMeter);
	vf32(B2_HUGE());
	vf32(B2_LINEAR_SLOP());
	vf32(B2_SPECULATIVE_DISTANCE());
	vf32(B2_CONTACT_RECYCLE_DISTANCE());
	vf32(B2_MAX_AABB_MARGIN());
	vend();
}

function checkInvalidUnits(): void {
	let bad: StaticArray<f32> = [0.0, -1.0, -0.0, Infinity, -Infinity, NaN];
	for (let i = 0; i < 6; ++i) {
		b2SetLengthUnitsPerMeter(bad[i]);
		vtag("badunits");
		vf32(b2GetLengthUnitsPerMeter());
		vend();
	}
}

function pfilter(f: b2Filter): void {
	vu64(f.categoryBits);
	vu64(f.maskBits);
	vi32(f.groupIndex);
}

function psurface(m: b2SurfaceMaterial): void {
	vf32(m.friction);
	vf32(m.restitution);
	vf32(m.rollingResistance);
	vf32(m.tangentSpeed);
	vu64(m.userMaterialId);
	vu32(m.customColor);
}

function pjointdef(d: b2JointDef): void {
	vu64(d.userData);
	vu64(b2StoreBodyId(d.bodyIdA));
	vu64(b2StoreBodyId(d.bodyIdB));
	pxf(d.localFrameA);
	pxf(d.localFrameB);
	vf32(d.forceThreshold);
	vf32(d.torqueThreshold);
	vf32(d.constraintHertz);
	vf32(d.constraintDampingRatio);
	vf32(d.drawScale);
	vbool(d.collideConnected);
}

function checkDefaults(units: f32): void {
	b2SetLengthUnitsPerMeter(units);

	let w = b2DefaultWorldDef();
	vtag("worlddef");
	pv(w.gravity);
	vf32(w.restitutionThreshold);
	vi32(w.restitutionIterations);
	vbool(w.enableRestitutionPropagation);
	vf32(w.hitEventThreshold);
	vf32(w.contactHertz);
	vf32(w.contactDampingRatio);
	vf32(w.contactSpeed);
	vf32(w.maximumLinearSpeed);
	vbool(w.frictionCallback == null);
	vbool(w.restitutionCallback == null);
	vbool(w.enableSleep);
	vbool(w.enableContinuous);
	vi32(w.workerCount);
	vbool(w.enqueueTask == null);
	vbool(w.finishTask == null);
	vu64(w.userTaskContext);
	vu64(w.userData);
	vi32(w.capacity.staticShapeCount);
	vi32(w.capacity.dynamicShapeCount);
	vi32(w.capacity.staticBodyCount);
	vi32(w.capacity.dynamicBodyCount);
	vi32(w.capacity.contactCount);
	vi32(w.capacity.arenaByteCount);
	vi32(w.internalValue);
	vend();

	let b = b2DefaultBodyDef();
	vtag("bodydef");
	vi32(b.type);
	pv(b.position);
	pr(b.rotation);
	pv(b.linearVelocity);
	vf32(b.angularVelocity);
	vf32(b.linearDamping);
	vf32(b.angularDamping);
	vf32(b.gravityScale);
	vf32(b.sleepThreshold);
	vf32(b.safetyFactor);
	vbool(b.name.length == 0);
	vu64(b.userData);
	vbool(b.motionLocks.linearX);
	vbool(b.motionLocks.linearY);
	vbool(b.motionLocks.angularZ);
	vbool(b.enableSleep);
	vbool(b.isAwake);
	vbool(b.isBullet);
	vbool(b.isEnabled);
	vbool(b.allowFastRotation);
	vbool(b.enableContactRecycling);
	vi32(b.internalValue);
	vend();

	vtag("filters");
	pfilter(b2DefaultFilter());
	let qf = b2DefaultQueryFilter();
	vu64(qf.categoryBits);
	vu64(qf.maskBits);
	psurface(b2DefaultSurfaceMaterial());
	vend();

	let s = b2DefaultShapeDef();
	vtag("shapedef");
	vu64(s.userData);
	psurface(s.material);
	vf32(s.density);
	pfilter(s.filter);
	vbool(s.enableCustomFiltering);
	vbool(s.isSensor);
	vbool(s.enableSensorEvents);
	vbool(s.enableContactEvents);
	vbool(s.enableHitEvents);
	vbool(s.enablePreSolveEvents);
	vbool(s.invokeContactCreation);
	vbool(s.updateBodyMass);
	vi32(s.internalValue);
	vend();

	let c = b2DefaultChainDef();
	vtag("chaindef");
	vu64(c.userData);
	vi32(c.pointCount);
	pv(c.ghost1);
	pv(c.ghost2);
	vi32(c.materialCount);
	psurface(c.materials[0]);
	pfilter(c.filter);
	vbool(c.isLoop);
	vbool(c.enableSensorEvents);
	vi32(c.internalValue);
	vend();

	let dj = b2DefaultDistanceJointDef();
	vtag("distancedef");
	pjointdef(dj.base);
	vf32(dj.length);
	vbool(dj.enableSpring);
	vf32(dj.lowerSpringForce);
	vf32(dj.upperSpringForce);
	vf32(dj.hertz);
	vf32(dj.dampingRatio);
	vbool(dj.enableLimit);
	vf32(dj.minLength);
	vf32(dj.maxLength);
	vbool(dj.enableMotor);
	vf32(dj.maxMotorForce);
	vf32(dj.motorSpeed);
	vi32(dj.internalValue);
	vend();

	let fj = b2DefaultFilterJointDef();
	vtag("filterdef");
	pjointdef(fj.base);
	vi32(fj.internalValue);
	vend();

	let mj = b2DefaultMotorJointDef();
	vtag("motordef");
	pjointdef(mj.base);
	pv(mj.linearVelocity);
	vf32(mj.maxVelocityForce);
	vf32(mj.angularVelocity);
	vf32(mj.maxVelocityTorque);
	vf32(mj.linearHertz);
	vf32(mj.linearDampingRatio);
	vf32(mj.maxSpringForce);
	vf32(mj.angularHertz);
	vf32(mj.angularDampingRatio);
	vf32(mj.maxSpringTorque);
	vi32(mj.internalValue);
	vend();

	let vj = b2DefaultMoverJointDef();
	vtag("moverdef");
	pjointdef(vj.base);
	pv(vj.linearVelocity);
	pv(vj.maxVelocityForce);
	vi32(vj.internalValue);
	vend();

	let pj = b2DefaultPogoJointDef();
	vtag("pogodef");
	pjointdef(pj.base);
	pv(pj.normal);
	vf32(pj.hertz);
	vf32(pj.dampingRatio);
	vf32(pj.restLength);
	vf32(pj.maxTensionForce);
	vf32(pj.maxCompressionForce);
	vf32(pj.impulse);
	vf32(pj.velocity);
	vi32(pj.internalValue);
	vend();

	let xj = b2DefaultPrismaticJointDef();
	vtag("prismaticdef");
	pjointdef(xj.base);
	vbool(xj.enableSpring);
	vf32(xj.hertz);
	vf32(xj.dampingRatio);
	vf32(xj.targetTranslation);
	vbool(xj.enableLimit);
	vf32(xj.lowerTranslation);
	vf32(xj.upperTranslation);
	vbool(xj.enableMotor);
	vf32(xj.maxMotorForce);
	vf32(xj.motorSpeed);
	vi32(xj.internalValue);
	vend();

	let rj = b2DefaultRevoluteJointDef();
	vtag("revolutedef");
	pjointdef(rj.base);
	vf32(rj.targetAngle);
	vbool(rj.enableSpring);
	vf32(rj.hertz);
	vf32(rj.dampingRatio);
	vbool(rj.enableLimit);
	vf32(rj.lowerAngle);
	vf32(rj.upperAngle);
	vbool(rj.enableMotor);
	vf32(rj.maxMotorTorque);
	vf32(rj.motorSpeed);
	vi32(rj.internalValue);
	vend();

	let wj = b2DefaultWeldJointDef();
	vtag("welddef");
	pjointdef(wj.base);
	vf32(wj.linearHertz);
	vf32(wj.angularHertz);
	vf32(wj.linearDampingRatio);
	vf32(wj.angularDampingRatio);
	vi32(wj.internalValue);
	vend();

	let hj = b2DefaultWheelJointDef();
	vtag("wheeldef");
	pjointdef(hj.base);
	vbool(hj.enableSpring);
	vf32(hj.hertz);
	vf32(hj.dampingRatio);
	vbool(hj.enableLimit);
	vf32(hj.lowerTranslation);
	vf32(hj.upperTranslation);
	vbool(hj.enableMotor);
	vf32(hj.maxMotorTorque);
	vf32(hj.motorSpeed);
	vi32(hj.internalValue);
	vend();

	let ex = b2DefaultExplosionDef();
	vtag("explosiondef");
	vu64(ex.maskBits);
	pv(ex.position);
	vf32(ex.radius);
	vf32(ex.falloff);
	vf32(ex.impulsePerLength);
	vend();
}

function checkScalars(): void {
	for (let i = 0; i < 6000; ++i) {
		let ia = <i32>vrand32();
		let ib = vrandint(-1000, 1000);
		let ic = vrandint(-1000, 1000);
		let num = vrandint(0, 100000);
		let den = vrandint(1, 1000);
		vtag("int");
		vi32(b2MinInt(ia, ib));
		vi32(b2MaxInt(ia, ib));
		vi32(b2AbsInt(ia));
		vi32(b2AbsInt(ib));
		vi32(b2ClampInt(ia, ib, ic));
		vi32(b2ClampInt(ib, -100, 100));
		vi32(b2CeilingInt(num, den));
		vend();

		let a = rscalar();
		let b = rscalar();
		let c = rscalar();
		vtag("f");
		vf32(a);
		vf32(b);
		vf32(c);
		vf32(b2MinFloat(a, b));
		vf32(b2MaxFloat(a, b));
		vf32(b2AbsFloat(a));
		vf32(b2ClampFloat(a, b, c));
		vbool(b2IsValidFloat(a));
		vend();

		vtag("trig");
		vf32(b2Atan2(a, b));
		let cs = b2ComputeCosSin(a);
		vf32(cs.cosine);
		vf32(cs.sine);
		vf32(b2UnwindAngle(a));
		let q = b2MakeRot(b);
		pr(q);
		vend();

		let angle = vrandf(-20.0, 20.0);
		let y = vrandf(-10.0, 10.0);
		let x = vrandf(-10.0, 10.0);
		vtag("trign");
		vf32(b2Atan2(y, x));
		vf32(b2Atan2(y, 0.0));
		vf32(b2Atan2(0.0, x));
		let csn = b2ComputeCosSin(angle);
		vf32(csn.cosine);
		vf32(csn.sine);
		vf32(b2UnwindAngle(angle));
		vend();

		let hertz = vrandf(0.0, 60.0);
		let zeta = vrandf(0.0, 10.0);
		let position = rscalar();
		let velocity = rscalar();
		let h = vrandf(0.0, 0.1);
		vtag("spring");
		vf32(b2SpringDamper(hertz, zeta, position, velocity, h));
		vend();

		vtag("cast");
		vi32(b2FloatToIntSat(a));
		vu32(b2FloatToUIntSat(a));
		vi32(b2FloatToIntSat(a * 1.0e8));
		vend();
	}

	let edges: StaticArray<f32> = [
		0.0,
		-0.0,
		1.0,
		-1.0,
		B2_PI,
		-B2_PI,
		0.5 * B2_PI,
		-0.5 * B2_PI,
		2.0 * B2_PI,
		1.0e6,
		-1.0e6,
		1.0e7,
		Infinity,
		-Infinity,
		NaN,
		FLT_MIN,
		FLT_MAX,
	];
	for (let i = 0; i < edges.length; ++i) {
		let e = edges[i];
		vtag("edge");
		vf32(e);
		vf32(b2UnwindAngle(e));
		let cs = b2ComputeCosSin(e);
		vf32(cs.cosine);
		vf32(cs.sine);
		vf32(b2Atan2(e, 1.0));
		vf32(b2Atan2(1.0, e));
		vf32(b2Atan2(e, e));
		vf32(b2Atan2(-e, e));
		vbool(b2IsValidFloat(e));
		vend();
	}
}

// A struct leaf for b2GetLengthAndNormalize's ref output
struct LengthOut {
	length: f32 = 0;
}

function checkVectors(): void {
	for (let i = 0; i < 6000; ++i) {
		let a = rvec();
		let b = rvec();
		let c = rvec();
		let s = rscalar();
		let t = vrandf(-0.5, 1.5);

		vtag("v");
		pv(a);
		pv(b);
		vf32(s);
		vf32(b2Dot(a, b));
		vf32(b2Cross(a, b));
		pv(b2CrossVS(a, s));
		pv(b2CrossSV(s, a));
		pv(b2LeftPerp(a));
		pv(b2RightPerp(a));
		pv(b2Add(a, b));
		pv(b2Sub(a, b));
		pv(b2Neg(a));
		pv(b2Lerp(a, b, t));
		pv(b2Mul(a, b));
		pv(b2MulSV(s, a));
		pv(b2MulAdd(a, s, b));
		pv(b2MulSub(a, s, b));
		vend();

		vtag("v2");
		pv(b2Abs(a));
		pv(b2Min(a, b));
		pv(b2Max(a, b));
		pv(b2Clamp(a, b, c));
		vf32(b2Length(a));
		vf32(b2Distance(a, b));
		pv(b2Normalize(a));
		vbool(b2IsNormalized(a));
		let out = LengthOut();
		out.length = -7.0;
		let n = b2GetLengthAndNormalize(out.length, a);
		pv(n);
		vf32(out.length);
		vf32(b2LengthSquared(a));
		vf32(b2DistanceSquared(a, b));
		vbool(b2IsValidVec2(a));
		vbool(b2IsNormalized(n));
		vend();

		vtag("pos");
		pv(b2ToPos(a));
		pv(b2ToVec2(a));
		pv(b2SubPos(a, b));
		pv(b2OffsetPos(a, b));
		pv(b2LerpPos(a, b, t));
		vbool(b2IsValidPos(a));
		vf32(b2RoundDownFloat(<f64>s * 1.000001));
		vf32(b2RoundUpFloat(<f64>s * 1.000001));
		vend();
	}
}

function checkRots(): void {
	for (let i = 0; i < 6000; ++i) {
		let q = rrot();
		let r = rrot();
		let v = rvec();
		let t = vrandf(-0.5, 1.5);
		let d = rscalar();
		let invH = vrandf(0.0, 240.0);

		vtag("q");
		pr(q);
		pr(r);
		pr(b2NormalizeRot(q));
		pr(b2IntegrateRotation(q, d));
		vbool(b2IsNormalizedRot(q));
		pr(b2InvertRot(q));
		pr(b2NLerp(q, r, t));
		vf32(b2ComputeAngularVelocity(q, r, invH));
		vf32(b2Rot_GetAngle(q));
		pv(b2Rot_GetXAxis(q));
		pv(b2Rot_GetYAxis(q));
		pr(b2MulRot(q, r));
		pr(b2InvMulRot(q, r));
		vf32(b2RelativeAngle(q, r));
		pv(b2RotateVector(q, v));
		pv(b2InvRotateVector(q, v));
		vbool(b2IsValidRot(q));
		vend();

		let u1 = runit();
		let u2 = runit();
		vtag("qu");
		pr(b2MakeRotFromUnitVector(u1));
		pr(b2ComputeRotationBetweenUnitVectors(u1, u2));
		let qn = rrotn();
		let dn = vrandf(-0.5, 0.5);
		pr(b2IntegrateRotation(qn, dn));
		vf32(b2RelativeAngle(qn, b2IntegrateRotation(qn, dn)));
		vend();
	}
}

function checkTransforms(): void {
	for (let i = 0; i < 6000; ++i) {
		let A = rxf();
		let B = rxf();
		let p = rvec();
		let base = rvec();

		vtag("xf");
		pv(b2TransformPoint(A, p));
		pv(b2InvTransformPoint(A, p));
		pxf(b2MulTransforms(A, B));
		pxf(b2InvMulTransforms(A, B));
		vbool(b2IsValidTransform(A));
		pv(b2TransformWorldPoint(A, p));
		pv(b2InvTransformWorldPoint(A, p));
		pxf(b2InvMulWorldTransforms(A, B));
		pxf(b2MulWorldTransforms(A, B));
		pxf(b2ToRelativeTransform(A, base));
		pxf(b2MakeWorldTransform(A));
		vbool(b2IsValidWorldTransform(A));
		vend();

		let m = rmat();
		let v = rvec();
		vtag("m");
		pv(b2MulMV(m, v));
		pm(b2GetInverse22(m));
		pv(b2Solve22(m, v));
		vend();
	}
}

function checkAABBs(): void {
	for (let i = 0; i < 6000; ++i) {
		let a = raabb();
		let b = raabb();

		vtag("ab");
		pab(a);
		pab(b);
		vbool(b2AABB_Contains(a, b));
		pv(b2AABB_Center(a));
		pv(b2AABB_Extents(a));
		pab(b2AABB_Union(a, b));
		vbool(b2AABB_Overlaps(a, b));
		vend();

		let proxy = b2ShapeProxy();
		proxy.count = vrandint(1, B2_MAX_POLYGON_VERTICES + 1);
		for (let k = 0; k < proxy.count; ++k) {
			proxy.points[k] = rvec();
		}
		proxy.radius = vrandf(0.0, 2.0);
		vtag("mk");
		pab(b2MakeAABB(proxy.points, proxy.count, proxy.radius));
		vend();

		let plane = rplane();
		let point = rvec();
		vtag("pl");
		ppl(plane);
		vbool(b2IsValidPlane(plane));
		vf32(b2PlaneSeparation(plane, point));
		vend();
	}
}

function checkHash(): void {
	let buffer = new Uint8Array(512);
	for (let i = 0; i < 400; ++i) {
		let len: i32;
		if (i < 260) {
			len = i;
		} else {
			len = vrandint(0, 512);
		}

		for (let j = 0; j < len; ++j) {
			buffer[j] = <u8>(vrand32() & 0xff);
		}

		let seed = i % 2 == 0 ? B2_HASH_INIT : vrand32();
		vtag("h");
		vi32(len);
		vu32(b2Hash(seed, buffer, len));
		vend();
	}
}

// FixedArray holders for the contact id helpers
struct ContactValues {
	values: FixedArray<u32, 3>;
}

function checkIds(): void {
	for (let i = 0; i < 2000; ++i) {
		let x32 = vrand32();
		let x64 = vrand64();
		let y64 = vrand64();
		let z64 = vrand64();
		let c64 = vrand64();
		let input: ContactValues;
		input.values[0] = vrand32();
		input.values[1] = vrand32();
		input.values[2] = vrand32();

		vtag("id");
		let w = b2LoadWorldId(x32);
		vu32(w.index1);
		vu32(w.generation);
		vu32(b2StoreWorldId(w));
		let bo = b2LoadBodyId(x64);
		vi32(bo.index1);
		vu32(bo.world0);
		vu32(bo.generation);
		vu64(b2StoreBodyId(bo));
		let sh = b2LoadShapeId(y64);
		vi32(sh.index1);
		vu32(sh.world0);
		vu32(sh.generation);
		vu64(b2StoreShapeId(sh));
		let ch = b2LoadChainId(c64);
		vi32(ch.index1);
		vu32(ch.world0);
		vu32(ch.generation);
		vu64(b2StoreChainId(ch));
		let jo = b2LoadJointId(z64);
		vi32(jo.index1);
		vu32(jo.world0);
		vu32(jo.generation);
		vu64(b2StoreJointId(jo));
		let co = b2LoadContactId(input.values);
		vi32(co.index1);
		vu32(co.world0);
		vi32(co.padding);
		vu32(co.generation);
		let out: ContactValues;
		b2StoreContactId(co, out.values);
		vu32(out.values[0]);
		vu32(out.values[1]);
		vu32(out.values[2]);
		let same = b2LoadBodyId(x64);
		let other = b2LoadBodyId(x64 ^ 1);
		vbool(bo == same);
		vbool(bo == other);
		vbool(bo.index1 == 0);
		vbool(bo.index1 != 0);
		vbool(b2_nullShapeId.index1 == 0);
		vend();
	}
}

export function run(): void {
	b2SetLogCallback(silentLog);
	b2SetAssertCallback(skipAssert);
	vseed(0x5eed);
	checkConstants();
	checkUnitConstants(1.0);
	checkDefaults(1.0);
	checkUnitConstants(0.37);
	checkDefaults(0.37);
	checkUnitConstants(100.0);
	checkInvalidUnits();
	b2SetLengthUnitsPerMeter(1.0);
	checkScalars();
	checkVectors();
	checkRots();
	checkTransforms();
	checkAABBs();
	checkHash();
	checkIds();
}
