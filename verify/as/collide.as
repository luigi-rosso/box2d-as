// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Printers and seeded shape generators shared by the collision drivers, the twin of
// verify/cref/collide.h.

import { vi32, vbool, vf32, vrand32, vrandint, vrandf } from "./common";
import { B2_PI, b2AABB, b2ComputeCosSin, b2MakeRot, b2Rot, b2Transform, b2Vec2 } from "../../lib/math_functions";
import {
	B2_MAX_POLYGON_VERTICES,
	b2Capsule,
	b2CastOutput,
	b2Circle,
	b2DistanceOutput,
	b2LocalManifold,
	b2Polygon,
	b2Segment,
	b2ShapeProxy,
	b2SimplexCache,
} from "../../lib/collision";
import { b2ComputeHull } from "../../lib/hull";
import { b2MakePolygon, b2MakeRoundedBox } from "../../lib/geometry";
import { b2MakeProxy } from "../../lib/distance";

export function pv(v: b2Vec2): void {
	vf32(v.x);
	vf32(v.y);
}

export function pr(q: b2Rot): void {
	vf32(q.c);
	vf32(q.s);
}

export function pxf(t: b2Transform): void {
	pv(t.p);
	pr(t.q);
}

export function paabb(a: b2AABB): void {
	pv(a.lowerBound);
	pv(a.upperBound);
}

export function ppoly(p: b2Polygon): void {
	vi32(p.count);
	for (let i = 0; i < B2_MAX_POLYGON_VERTICES; ++i) {
		pv(p.vertices[i]);
	}
	for (let i = 0; i < B2_MAX_POLYGON_VERTICES; ++i) {
		pv(p.normals[i]);
	}
	pv(p.centroid);
	vf32(p.radius);
}

export function pproxy(p: b2ShapeProxy): void {
	vi32(p.count);
	for (let i = 0; i < p.count; ++i) {
		pv(p.points[i]);
	}
	vf32(p.radius);
}

export function pcast(o: b2CastOutput): void {
	pv(o.normal);
	pv(o.point);
	vf32(o.fraction);
	vi32(o.iterations);
	vbool(o.hit);
}

export function pcache(c: b2SimplexCache): void {
	vi32(<i32>c.count);
	for (let i = 0; i < 3; ++i) {
		vi32(<i32>c.indexA[i]);
	}
	for (let i = 0; i < 3; ++i) {
		vi32(<i32>c.indexB[i]);
	}
}

export function pdist(o: b2DistanceOutput): void {
	pv(o.pointA);
	pv(o.pointB);
	pv(o.normal);
	vf32(o.distance);
	vi32(o.iterations);
	vi32(o.simplexCount);
}

export function plocal(m: b2LocalManifold): void {
	vi32(m.pointCount);
	pv(m.normal);
	for (let i = 0; i < 2; ++i) {
		pv(m.points[i].point);
		vf32(m.points[i].separation);
		vi32(<i32>m.points[i].id);
	}
}

export function randVec(lo: f32, hi: f32): b2Vec2 {
	let x = vrandf(lo, hi);
	let y = vrandf(lo, hi);
	return b2Vec2(x, y);
}

export function randRot(): b2Rot {
	return b2MakeRot(vrandf(-3.25, 3.25));
}

export function randXf(range: f32): b2Transform {
	let t: b2Transform;
	t.p = randVec(-range, range);
	t.q = randRot();
	return t;
}

export function randRadius(hi: f32): f32 {
	if ((vrand32() & 1) == 0) {
		return 0.0;
	}
	return vrandf(0.0, hi);
}

// Scratch for b2ComputeHull, which takes an array where C takes a pointer
export const hullPoints: StaticArray<b2Vec2> = new StaticArray<b2Vec2>(16);

const libraryCounts: StaticArray<i32> = [3, 4, 5, 6, 7, 8];

export function libraryPolygon(which: i32, radius: f32): b2Polygon {
	let n = libraryCounts[which % 6];
	for (let i = 0; i < n; ++i) {
		let a: f32 = (2.0 * B2_PI * <f32>i) / <f32>n + 0.125 * <f32>which;
		let r: f32 = 0.4375 + 0.125 * <f32>(i % 3);
		let cs = b2ComputeCosSin(a);
		hullPoints[i] = b2Vec2(r * cs.cosine, r * cs.sine);
	}
	let hull = b2ComputeHull(hullPoints, n);
	return b2MakePolygon(hull, radius);
}

export function randPolygon(): b2Polygon {
	let n = vrandint(3, 9);
	for (let i = 0; i < n; ++i) {
		hullPoints[i] = randVec(-0.75, 0.75);
	}
	let radius = randRadius(0.25);
	let hull = b2ComputeHull(hullPoints, n);
	if (hull.count < 3) {
		return b2MakeRoundedBox(0.5, 0.25, radius);
	}
	return b2MakePolygon(hull, radius);
}

export function randProxy(): b2ShapeProxy {
	let n = vrandint(1, 9);
	let pts: FixedArray<b2Vec2, 8>;
	for (let i = 0; i < n; ++i) {
		pts[i] = randVec(-0.6, 0.6);
	}
	let radius = randRadius(0.3);
	return b2MakeProxy(pts, n, radius);
}

export function randCircle(): b2Circle {
	let c: b2Circle;
	c.center = randVec(-0.25, 0.25);
	c.radius = vrandf(0.0625, 0.5);
	return c;
}

export function randCapsule(): b2Capsule {
	let c: b2Capsule;
	c.center1 = randVec(-0.6, 0.6);
	c.center2 = randVec(-0.6, 0.6);
	c.radius = vrandf(0.03125, 0.375);
	return c;
}

export function randSegment(): b2Segment {
	let s: b2Segment;
	s.point1 = randVec(-0.8, 0.8);
	s.point2 = randVec(-0.8, 0.8);
	return s;
}

export const RING_ANGLES: i32 = 8;
export const RING_DEPTHS: i32 = 7;
export const RING_ROTS: i32 = 4;

const ringDepths: StaticArray<f32> = [-0.375, -0.125, -0.015625, 0.0, 0.015625, 0.125, 0.625];
const ringRots: StaticArray<f32> = [0.0, 0.375, -1.25, 2.875];

export function ringPose(an: i32, dn: i32, rn: i32, reach: f32): b2Transform {
	let a: f32 = (2.0 * B2_PI * <f32>an) / <f32>RING_ANGLES + 0.0625;
	let d = reach + ringDepths[dn];
	let cs = b2ComputeCosSin(a);
	let xf: b2Transform;
	xf.p = b2Vec2(d * cs.cosine, d * cs.sine);
	xf.q = b2MakeRot(ringRots[rn]);
	return xf;
}
