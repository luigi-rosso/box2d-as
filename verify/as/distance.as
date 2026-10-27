// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/distance.as, mirrors verify/cref/distance.c

import { vtag, vi32, vbool, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import {
	RING_ANGLES,
	RING_DEPTHS,
	RING_ROTS,
	libraryPolygon,
	pcache,
	pcast,
	pdist,
	pproxy,
	pr,
	pv,
	pxf,
	randProxy,
	randRadius,
	randRot,
	randVec,
	randXf,
	ringPose,
} from "./collide";
import { b2Add, b2MakeRot, b2MulSV, b2Sub, b2Vec2 } from "../../lib/math_functions";
import {
	b2DistanceInput,
	b2ShapeCastPairInput,
	b2ShapeProxy,
	b2Simplex,
	b2SimplexCache,
	b2SimplexVertex,
	b2Sweep,
	b2TOIInput,
} from "../../lib/collision";
import {
	b2CastShape,
	b2ComputeSegmentDistance,
	b2ComputeShapeDistance,
	b2ComputeTimeOfImpact,
	b2GetSweepTransform,
	b2MakeOffsetProxy,
	b2MakePointProxy,
	b2MakeProxy,
	b2MakeSegmentProxy,
} from "../../lib/distance";
import { b2MakeBox } from "../../lib/geometry";

export { frameBoundary } from "./common";

function pvertex(v: b2SimplexVertex): void {
	pv(v.wA);
	pv(v.wB);
	pv(v.w);
	vf32(v.a);
	vi32(v.indexA);
	vi32(v.indexB);
}

function psimplex(s: b2Simplex): void {
	vi32(s.count);
	if (s.count > 0) {
		pvertex(s.v1);
	}
	if (s.count > 1) {
		pvertex(s.v2);
	}
	if (s.count > 2) {
		pvertex(s.v3);
	}
}

function psweep(s: b2Sweep): void {
	pv(s.localCenter);
	pv(s.c1);
	pv(s.c2);
	pr(s.q1);
	pr(s.q2);
}

function gridProxy(which: i32): b2ShapeProxy {
	switch (which) {
		case 0: {
			return b2MakePointProxy(b2Vec2(0.03125, -0.0625), 0.25);
		}
		case 1: {
			return b2MakeSegmentProxy(b2Vec2(-0.5, 0.0625), b2Vec2(0.4375, -0.0625), 0.125);
		}
		case 2: {
			let p = libraryPolygon(2, 0.0);
			return b2MakeProxy(p.vertices, p.count, p.radius);
		}
		case 3: {
			let p = libraryPolygon(5, 0.0625);
			return b2MakeProxy(p.vertices, p.count, p.radius);
		}
		case 4: {
			let p = b2MakeBox(0.375, 0.25);
			return b2MakeProxy(p.vertices, p.count, p.radius);
		}
		default: {
			let p = libraryPolygon(3, 0.0);
			return b2MakeProxy(p.vertices, p.count, 0.1875);
		}
	}
}

function randCache(cache: ref b2SimplexCache, proxyA: b2ShapeProxy, proxyB: b2ShapeProxy): void {
	let kind = vrandint(0, 3);
	if (kind == 2) {
		for (let i = 0; i < <i32>cache.count; ++i) {
			if (<i32>cache.indexA[i] >= proxyA.count || <i32>cache.indexB[i] >= proxyB.count) {
				kind = 0;
			}
		}
	}

	if (kind == 0) {
		let empty: b2SimplexCache;
		cache = empty;
	} else if (kind == 1) {
		let fresh: b2SimplexCache;
		cache = fresh;
		cache.count = <u16>vrandint(1, 4);
		for (let i = 0; i < <i32>cache.count; ++i) {
			cache.indexA[i] = <u8>vrandint(0, proxyA.count);
			cache.indexB[i] = <u8>vrandint(0, proxyB.count);
		}
	}
}

function randSweep(reach: f32): b2Sweep {
	let s: b2Sweep;
	s.localCenter = randVec(-0.25, 0.25);
	s.c1 = randVec(-reach, reach);
	let d = randVec(-reach, reach);
	s.c2 = b2Add(s.c1, d);
	let angle1 = vrandf(-3.0, 3.0);
	let delta = vrandf(-1.5, 1.5);
	if ((vrand32() & 3) == 0) {
		delta = 0.0;
	}
	s.q1 = b2MakeRot(angle1);
	s.q2 = b2MakeRot(angle1 + delta);
	return s;
}

function sweeps(): void {
	for (let i = 0; i < 5000; ++i) {
		let sweep = randSweep(3.0);
		let t = vrandf(-0.25, 1.25);
		let pick = vrandint(0, 4);
		if (pick == 0) {
			t = 0.0;
		} else if (pick == 1) {
			t = 1.0;
		}
		vtag("sw");
		pxf(b2GetSweepTransform(sweep, t));
		vend();
	}
}

function segments(): void {
	for (let i = 0; i < 20000; ++i) {
		let p1 = randVec(-1.0, 1.0);
		let q1 = randVec(-1.0, 1.0);
		let p2 = randVec(-1.0, 1.0);
		let q2 = randVec(-1.0, 1.0);
		let kind = vrandint(0, 8);
		if (kind == 0) {
			q1 = p1;
		} else if (kind == 1) {
			q2 = p2;
		} else if (kind == 2) {
			q1 = p1;
			q2 = p2;
		} else if (kind == 3) {
			// parallel
			let offset = randVec(-0.5, 0.5);
			let scale = vrandf(-2.0, 2.0);
			p2 = b2Add(p1, offset);
			q2 = b2Add(p2, b2MulSV(scale, b2Sub(q1, p1)));
		} else if (kind == 4) {
			// touching at an end
			p2 = q1;
		}
		let r = b2ComputeSegmentDistance(p1, q1, p2, q2);
		vtag("sd");
		pv(r.closest1);
		pv(r.closest2);
		vf32(r.fraction1);
		vf32(r.fraction2);
		vf32(r.distanceSquared);
		vend();
	}
}

function proxies(): void {
	for (let i = 0; i < 2000; ++i) {
		// C draws up to 10 points and both makers keep the first 8
		let pts: FixedArray<b2Vec2, 8>;
		let n = vrandint(1, 11);
		for (let j = 0; j < n; ++j) {
			let p = randVec(-2.0, 2.0);
			if (j < 8) {
				pts[j] = p;
			}
		}
		let radius = randRadius(1.0);
		let position = randVec(-10.0, 10.0);
		let rotation = randRot();
		let a = b2MakeProxy(pts, n, radius);
		let b = b2MakeOffsetProxy(pts, n, radius, position, rotation);
		vtag("px");
		pproxy(a);
		pproxy(b);
		vend();
	}
}

const simplexes = new StaticArray<b2Simplex>(2);

function distanceQuery(tag: string, input: b2DistanceInput, cache: ref b2SimplexCache, record: bool): void {
	vtag(tag);
	pcache(cache);
	let output = b2ComputeShapeDistance(input, cache, record ? simplexes : null, record ? 2 : 0);
	pdist(output);
	pcache(cache);
	if (output.simplexCount > 0) {
		psimplex(simplexes[0]);
	}
	vend();
}

function distanceGrid(): void {
	for (let a = 0; a < 6; ++a) {
		for (let b = 0; b < 6; ++b) {
			let input: b2DistanceInput;
			input.proxyA = gridProxy(a);
			input.proxyB = gridProxy(b);

			let warm: b2SimplexCache;
			for (let an = 0; an < RING_ANGLES; ++an) {
				for (let dn = 0; dn < RING_DEPTHS; ++dn) {
					for (let rn = 0; rn < RING_ROTS; ++rn) {
						input.transform = ringPose(an, dn, rn, 1.0);
						input.useRadii = ((an + dn + rn) & 1) == 0;

						let cold: b2SimplexCache;
						distanceQuery("dg", input, cold, rn == 0);
						distanceQuery("dw", input, warm, rn == 1);
					}
				}
			}
		}
	}
}

function distanceRandom(): void {
	let cache: b2SimplexCache;
	for (let i = 0; i < 30000; ++i) {
		let input: b2DistanceInput;
		input.proxyA = randProxy();
		input.proxyB = randProxy();
		input.transform = randXf(1.5);
		input.useRadii = (vrand32() & 1) == 0;
		randCache(cache, input.proxyA, input.proxyB);
		distanceQuery("dr", input, cache, (i & 3) == 0);
	}
}

function shapeCasts(): void {
	for (let i = 0; i < 20000; ++i) {
		let input: b2ShapeCastPairInput;
		input.proxyA = randProxy();
		input.proxyB = randProxy();
		input.transform = randXf(3.0);
		let target = randVec(-0.5, 0.5);
		let scale = vrandf(0.25, 2.5);
		input.translationB = b2MulSV(scale, b2Sub(target, input.transform.p));
		input.maxFraction = vrandf(0.25, 1.25);
		input.canEncroach = (vrand32() & 1) == 0;
		vtag("cs");
		pcast(b2CastShape(input));
		vend();
	}
}

function timeOfImpact(): void {
	for (let i = 0; i < 20000; ++i) {
		let input: b2TOIInput;
		input.proxyA = randProxy();
		input.proxyB = randProxy();
		input.sweepA = randSweep(2.0);
		input.sweepB = randSweep(2.0);
		if ((i & 3) == 0) {
			// a fast mover against a nearly static one
			input.sweepA.c2 = b2Add(input.sweepA.c1, randVec(-0.125, 0.125));
			input.sweepB.c2 = b2Sub(b2MulSV(2.0, input.sweepA.c1), input.sweepB.c1);
		}
		input.maxFraction = (i & 1) != 0 ? 1.0 : vrandf(0.25, 1.0);
		let output = b2ComputeTimeOfImpact(input);
		vtag("toi");
		psweep(input.sweepA);
		psweep(input.sweepB);
		vi32(output.state);
		pv(output.point);
		pv(output.normal);
		vf32(output.fraction);
		vend();
	}
}

export function run(): void {
	vseed(41);
	sweeps();
	segments();
	proxies();
	distanceGrid();
	distanceRandom();
	shapeCasts();
	timeOfImpact();
}
