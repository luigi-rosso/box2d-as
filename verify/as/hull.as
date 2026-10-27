// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/hull.as, mirrors verify/cref/hull.c

import { vtag, vi32, vbool, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import { pv, randRot, randVec } from "./collide";
import { B2_PI, b2Add, b2ComputeCosSin, b2LeftPerp, b2MulSV, b2Vec2 } from "../../lib/math_functions";
import { B2_MAX_POLYGON_VERTICES, b2Hull } from "../../lib/collision";
import { b2ComputeHull, b2ValidateHull } from "../../lib/hull";

export { frameBoundary } from "./common";

const pts = new StaticArray<b2Vec2>(16);

function makePoints(): i32 {
	let kind = vrandint(0, 5);
	let count = vrandint(0, 11);
	let center = randVec(-4.0, 4.0);
	let scale = kind == 1 ? vrandf(0.0001, 0.02) : vrandf(0.125, 3.0);
	let dir = randRot();
	let axis = b2Vec2(dir.c, dir.s);
	for (let i = 0; i < count; ++i) {
		if (kind == 2) {
			let t = vrandf(-1.0, 1.0);
			let jitter = vrandf(-0.004, 0.004);
			pts[i] = b2Add(center, b2Add(b2MulSV(scale * t, axis), b2MulSV(jitter, b2LeftPerp(axis))));
		} else if (kind == 3 && i > 0 && (vrand32() & 1) == 0) {
			let j = vrandint(0, i);
			let offset = randVec(-0.004, 0.004);
			pts[i] = b2Add(pts[j], offset);
		} else if (kind == 4) {
			let q = randRot();
			pts[i] = b2Add(center, b2MulSV(scale, b2Vec2(q.c, q.s)));
		} else {
			let r = randVec(-1.0, 1.0);
			pts[i] = b2Add(center, b2MulSV(scale, r));
		}
	}
	return count;
}

function phull(hull: b2Hull): void {
	vi32(hull.count);
	for (let i = 0; i < B2_MAX_POLYGON_VERTICES; ++i) {
		pv(hull.points[i]);
	}
}

function setPoints(xs: StaticArray<f32>): i32 {
	let n = xs.length / 2;
	for (let i = 0; i < n; ++i) {
		pts[i] = b2Vec2(xs[2 * i], xs[2 * i + 1]);
	}
	return n;
}

export function run(): void {
	vseed(21);

	// Fixed cases
	{
		let n = setPoints([-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0]);
		let hull = b2ComputeHull(pts, n);
		vtag("square");
		phull(hull);
		vbool(b2ValidateHull(hull));
		vend();

		n = setPoints([0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 4.0, 0.0]);
		hull = b2ComputeHull(pts, n);
		vtag("line");
		phull(hull);
		vend();

		n = setPoints([0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
		hull = b2ComputeHull(pts, n);
		vtag("same");
		phull(hull);
		vend();
	}

	for (let i = 0; i < 30000; ++i) {
		let count = makePoints();
		let hull = b2ComputeHull(pts, count);
		vtag("h");
		vi32(count);
		phull(hull);
		vbool(b2ValidateHull(hull));
		vend();
	}

	// Raw point sets as hulls exercise the rejections of b2ValidateHull
	for (let i = 0; i < 20000; ++i) {
		let hull: b2Hull;
		hull.count = vrandint(0, 10);
		let n = hull.count < 8 ? hull.count : 8;
		let ring = (vrand32() & 1) == 0;
		let phase = vrandf(0.0, 1.0);
		for (let j = 0; j < n; ++j) {
			if (ring) {
				let a: f32 = phase + (2.0 * B2_PI * <f32>j) / <f32>n;
				let r = vrandf(0.5, 1.0);
				let cs = b2ComputeCosSin(a);
				hull.points[j] = b2Vec2(r * cs.cosine, r * cs.sine);
			} else {
				hull.points[j] = randVec(-1.0, 1.0);
			}
		}
		vtag("v");
		phull(hull);
		vbool(b2ValidateHull(hull));
		vend();
	}
}
