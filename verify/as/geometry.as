// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/geometry.as, mirrors verify/cref/geometry.c

import { vtag, vi32, vbool, vf32, vend, vseed, vrand32, vrandint, vrandf, vspecialf } from "./common";
import {
	paabb,
	pcast,
	ppoly,
	pproxy,
	pv,
	randCapsule,
	randCircle,
	randPolygon,
	randRadius,
	randRot,
	randSegment,
	randVec,
	randXf,
	libraryPolygon,
} from "./collide";
import { b2Add, b2MulSV, b2Rot_identity, b2Sub, b2Transform, b2Vec2, b2Vec2_zero } from "../../lib/math_functions";
import { b2Capsule, b2Hull, b2MassData, b2PlaneResult, b2Polygon, b2RayCastInput, b2ShapeCastInput } from "../../lib/collision";
import { b2ComputeHull } from "../../lib/hull";
import { b2MakeOffsetProxy } from "../../lib/distance";
import {
	b2CastRayCapsule,
	b2CastRayCircle,
	b2CastRayPolygon,
	b2CastRaySegment,
	b2CastShapeCapsule,
	b2CastShapeCircle,
	b2CastShapePolygon,
	b2CastShapeSegment,
	b2CollideMoverAndCapsule,
	b2CollideMoverAndCircle,
	b2CollideMoverAndPolygon,
	b2CollideMoverAndSegment,
	b2ComputeCapsuleBounds,
	b2ComputeCapsuleFatAABB,
	b2ComputeCapsuleMass,
	b2ComputeCircleBounds,
	b2ComputeCircleFatAABB,
	b2ComputeCircleMass,
	b2ComputePolygonBounds,
	b2ComputePolygonFatAABB,
	b2ComputePolygonMass,
	b2ComputeSegmentBounds,
	b2ComputeSegmentFatAABB,
	b2IsValidRay,
	b2MakeBox,
	b2MakeOffsetBox,
	b2MakeOffsetPolygon,
	b2MakeOffsetRoundedBox,
	b2MakeOffsetRoundedPolygon,
	b2MakePolygon,
	b2MakeRoundedBox,
	b2MakeSquare,
	b2PointInCapsule,
	b2PointInCircle,
	b2PointInPolygon,
	b2TransformPolygon,
} from "../../lib/geometry";

export { frameBoundary } from "./common";

const hullPoints = new StaticArray<b2Vec2>(8);

function pmass(m: b2MassData): void {
	vf32(m.mass);
	pv(m.center);
	vf32(m.rotationalInertia);
}

function pplane(r: b2PlaneResult): void {
	pv(r.plane.normal);
	vf32(r.plane.offset);
	pv(r.point);
	vbool(r.hit);
}

function randPose(): b2Transform {
	let xf = randXf(4.0);
	if ((vrand32() & 3) == 0) {
		let far = vrandf(-20000.0, 20000.0);
		xf.p.x += far;
	}
	return xf;
}

function randRay(): b2RayCastInput {
	let input: b2RayCastInput;
	let kind = vrandint(0, 6);
	let origin = randVec(-2.5, 2.5);
	let target = randVec(-0.75, 0.75);
	input.origin = origin;
	input.translation = b2Sub(target, origin);
	if (kind == 0) {
		input.translation = b2Vec2_zero;
	} else if (kind == 1) {
		let scale = vrandf(0.25, 3.0);
		input.translation = b2MulSV(scale, input.translation);
	} else if (kind == 2) {
		input.origin = randVec(-0.25, 0.25);
	}
	input.maxFraction = vrandf(0.0, 1.5);
	return input;
}

function randCastInput(): b2ShapeCastInput {
	let pts: FixedArray<b2Vec2, 8>;
	let n = vrandint(1, 9);
	for (let i = 0; i < n; ++i) {
		pts[i] = randVec(-0.375, 0.375);
	}
	let radius = randRadius(0.25);
	let position = randVec(-3.0, 3.0);
	let rotation = randRot();

	let input: b2ShapeCastInput;
	input.proxy = b2MakeOffsetProxy(pts, n, radius, position, rotation);
	let target = randVec(-0.5, 0.5);
	let scale = vrandf(0.5, 2.0);
	input.translation = b2MulSV(scale, b2Sub(target, position));
	input.maxFraction = vrandf(0.25, 1.5);
	input.canEncroach = (vrand32() & 1) == 1;
	return input;
}

function randMover(): b2Capsule {
	let mover: b2Capsule;
	mover.center1 = randVec(-1.25, 1.25);
	let d = randVec(-0.5, 0.5);
	mover.center2 = b2Add(mover.center1, d);
	mover.radius = vrandf(0.0625, 0.375);
	return mover;
}

function makers(): void {
	for (let w = 0; w < 12; ++w) {
		let p = libraryPolygon(w, (w & 1) != 0 ? 0.0625 : 0.0);
		vtag("lib");
		ppoly(p);
		vend();
	}

	for (let i = 0; i < 4000; ++i) {
		let hw = vrandf(0.01, 3.0);
		let hh = vrandf(0.01, 3.0);
		let radius = vrandf(0.0, 0.5);
		let center = randVec(-5.0, 5.0);
		let rotation = randRot();

		let p = b2MakeBox(hw, hh);
		vtag("box");
		ppoly(p);
		vend();

		p = b2MakeSquare(hw);
		vtag("square");
		ppoly(p);
		vend();

		p = b2MakeRoundedBox(hw, hh, radius);
		vtag("rbox");
		ppoly(p);
		vend();

		p = b2MakeOffsetBox(hw, hh, center, rotation);
		vtag("obox");
		ppoly(p);
		vend();

		p = b2MakeOffsetRoundedBox(hw, hh, center, rotation, radius);
		vtag("orbox");
		ppoly(p);
		vend();

		let q = randPolygon();
		vtag("rpoly");
		ppoly(q);
		vend();

		let xf = randXf(5.0);
		p = b2TransformPolygon(xf, q);
		vtag("xpoly");
		ppoly(p);
		vend();

		let n = vrandint(3, 9);
		for (let j = 0; j < n; ++j) {
			hullPoints[j] = randVec(-1.0, 1.0);
		}
		let hull = b2ComputeHull(hullPoints, n);
		if (hull.count >= 3) {
			p = b2MakeOffsetPolygon(hull, center, rotation);
			vtag("opoly");
			ppoly(p);
			vend();

			p = b2MakeOffsetRoundedPolygon(hull, center, rotation, radius);
			vtag("orpoly");
			ppoly(p);
			vend();
		}
	}

	// A bad hull falls back to a square when asserts are off
	let bad: b2Hull;
	let p = b2MakePolygon(bad, 0.0);
	vtag("bad");
	ppoly(p);
	vend();
	p = b2MakeOffsetRoundedPolygon(bad, b2Vec2_zero, b2Rot_identity, 0.25);
	ppoly(p);
	vend();
}

function masses(): void {
	for (let i = 0; i < 5000; ++i) {
		let density = vrandf(0.0, 10.0);

		let circle = randCircle();
		vtag("mc");
		pmass(b2ComputeCircleMass(circle, density));
		vend();

		let capsule = randCapsule();
		vtag("mk");
		pmass(b2ComputeCapsuleMass(capsule, density));
		vend();

		let polygon = randPolygon();
		vtag("mp");
		pmass(b2ComputePolygonMass(polygon, density));
		vend();

		// One and two point polygons go through the circle and capsule paths
		let small: b2Polygon;
		small.count = 1 + (i & 1);
		small.vertices[0] = randVec(-1.0, 1.0);
		small.vertices[1] = randVec(-1.0, 1.0);
		small.radius = vrandf(0.0, 0.5);
		vtag("ms");
		pmass(b2ComputePolygonMass(small, density));
		vend();
	}
}

// C goes through b2ComputeFatShapeAABB, which dispatches on the shape type to these helpers
function bounds(): void {
	for (let i = 0; i < 5000; ++i) {
		let xf = randPose();
		let extra = vrandf(0.0, 0.25);

		let circle = randCircle();
		vtag("bc");
		paabb(b2ComputeCircleBounds(circle, xf));
		paabb(b2ComputeCircleFatAABB(circle, xf, extra));
		vend();

		let capsule = randCapsule();
		vtag("bk");
		paabb(b2ComputeCapsuleBounds(capsule, xf));
		paabb(b2ComputeCapsuleFatAABB(capsule, xf, extra));
		vend();

		let polygon = randPolygon();
		vtag("bp");
		paabb(b2ComputePolygonBounds(polygon, xf));
		paabb(b2ComputePolygonFatAABB(polygon, xf, extra));
		vend();

		let segment = randSegment();
		vtag("bs");
		paabb(b2ComputeSegmentBounds(segment, xf));
		paabb(b2ComputeSegmentFatAABB(segment, xf, extra));
		vend();
	}
}

function points(): void {
	for (let i = 0; i < 5000; ++i) {
		let circle = randCircle();
		let capsule = randCapsule();
		if ((i & 7) == 0) {
			capsule.center2 = capsule.center1;
		}
		let polygon = randPolygon();

		for (let j = 0; j < 4; ++j) {
			let point = randVec(-1.25, 1.25);
			vtag("pt");
			pv(point);
			vbool(b2PointInCircle(circle, point));
			vbool(b2PointInCapsule(capsule, point));
			vbool(b2PointInPolygon(polygon, point));
			vend();
		}
	}
}

function rays(): void {
	for (let i = 0; i < 5000; ++i) {
		let input: b2RayCastInput;
		input.origin = b2Vec2(vspecialf(), 0.0);
		input.origin.y = vspecialf();
		input.translation = randVec(-1.0, 1.0);
		input.maxFraction = vspecialf();
		vtag("valid");
		vbool(b2IsValidRay(input));
		vend();
	}

	for (let i = 0; i < 6000; ++i) {
		let circle = randCircle();
		let input = randRay();
		vtag("rc");
		pcast(b2CastRayCircle(circle, input));
		vend();

		let capsule = randCapsule();
		if ((i & 15) == 0) {
			capsule.center2 = capsule.center1;
		}
		input = randRay();
		if ((i & 7) == 1) {
			// Parallel to the capsule axis
			let scale = vrandf(-2.0, 2.0);
			input.translation = b2MulSV(scale, b2Sub(capsule.center2, capsule.center1));
		}
		vtag("rk");
		pcast(b2CastRayCapsule(capsule, input));
		vend();

		let segment = randSegment();
		input = randRay();
		vtag("rs");
		pcast(b2CastRaySegment(segment, input, false));
		pcast(b2CastRaySegment(segment, input, true));
		vend();

		let polygon = randPolygon();
		input = randRay();
		vtag("rp");
		pcast(b2CastRayPolygon(polygon, input));
		vend();
	}
}

function casts(): void {
	for (let i = 0; i < 4000; ++i) {
		let circle = randCircle();
		let input = randCastInput();
		vtag("sc");
		pproxy(input.proxy);
		pcast(b2CastShapeCircle(circle, input));
		vend();

		let capsule = randCapsule();
		input = randCastInput();
		vtag("sk");
		pcast(b2CastShapeCapsule(capsule, input));
		vend();

		let segment = randSegment();
		input = randCastInput();
		vtag("ss");
		pcast(b2CastShapeSegment(segment, input));
		vend();

		let polygon = randPolygon();
		input = randCastInput();
		vtag("sp");
		pcast(b2CastShapePolygon(polygon, input));
		vend();
	}
}

function movers(): void {
	for (let i = 0; i < 5000; ++i) {
		let mover = randMover();

		let circle = randCircle();
		vtag("mvc");
		pplane(b2CollideMoverAndCircle(mover, circle));
		vend();

		let capsule = randCapsule();
		vtag("mvk");
		pplane(b2CollideMoverAndCapsule(mover, capsule));
		vend();

		let polygon = randPolygon();
		vtag("mvp");
		pplane(b2CollideMoverAndPolygon(mover, polygon));
		vend();

		let segment = randSegment();
		vtag("mvs");
		pplane(b2CollideMoverAndSegment(mover, segment));
		vend();
	}
}

export function run(): void {
	vseed(31);
	makers();
	masses();
	bounds();
	points();
	rays();
	casts();
	movers();
}
