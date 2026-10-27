// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/mover.as, mirrors verify/cref/mover.c

import { vtag, vi32, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import { pv, randRot, randVec } from "./collide";
import { FLT_MAX } from "../../lib/core";
import { b2Vec2 } from "../../lib/math_functions";
import { b2CollisionPlane } from "../../lib/collision";
import { b2ClipVector, b2SolvePlanes } from "../../lib/mover";

export { frameBoundary } from "./common";

const planes = new StaticArray<b2CollisionPlane>(8);

export function run(): void {
	vseed(61);

	for (let i = 0; i < 20000; ++i) {
		let count = vrandint(0, 9);
		for (let j = 0; j < count; ++j) {
			let q = randRot();
			planes[j].plane.normal = b2Vec2(q.c, q.s);
			planes[j].plane.offset = vrandf(-0.25, 0.25);
			let kind = vrandint(0, 4);
			planes[j].pushLimit = kind == 0 ? FLT_MAX : vrandf(0.0, 0.5);
			planes[j].push = vrandf(-1.0, 1.0);
			planes[j].clipVelocity = (vrand32() & 1) == 0;
		}

		let target = randVec(-1.0, 1.0);
		let result = b2SolvePlanes(target, planes, count);
		vtag("solve");
		pv(result.delta);
		vi32(result.iterationCount);
		for (let j = 0; j < count; ++j) {
			vf32(planes[j].push);
		}
		vend();

		let velocity = randVec(-2.0, 2.0);
		vtag("clip");
		pv(b2ClipVector(velocity, planes, count));
		for (let j = 0; j < count; ++j) {
			planes[j].push = (vrand32() & 1) == 0 ? 0.0 : 1.0;
		}
		pv(b2ClipVector(velocity, planes, count));
		vend();
	}
}
