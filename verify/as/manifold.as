// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/manifold.as, mirrors verify/cref/manifold.c

import { vtag, vend, vseed, vrand32 } from "./common";
import {
	RING_ANGLES,
	RING_DEPTHS,
	RING_ROTS,
	libraryPolygon,
	pcache,
	plocal,
	randCapsule,
	randCircle,
	randPolygon,
	randRot,
	randSegment,
	randVec,
	randXf,
	ringPose,
} from "./collide";
import { b2Add, b2InvMulTransforms, b2MakeRot, b2Sub, b2Transform, b2Vec2 } from "../../lib/math_functions";
import { b2Capsule, b2ChainSegment, b2Circle, b2LocalManifold, b2Segment, b2SimplexCache } from "../../lib/collision";
import { b2MakeBox, b2MakeRoundedBox } from "../../lib/geometry";
import {
	b2CollideCapsuleAndCircle,
	b2CollideCapsules,
	b2CollideChainSegmentAndCapsule,
	b2CollideChainSegmentAndCircle,
	b2CollideChainSegmentAndPolygon,
	b2CollideCircles,
	b2CollidePolygonAndCapsule,
	b2CollidePolygonAndCircle,
	b2CollidePolygons,
	b2CollideSegmentAndCapsule,
	b2CollideSegmentAndCircle,
	b2CollideSegmentAndPolygon,
} from "../../lib/manifold";

export { frameBoundary } from "./common";

function poseA(variant: i32): b2Transform {
	let xf: b2Transform;
	xf.p = b2Vec2(0.03125 * <f32>variant, -0.015625 * <f32>variant);
	xf.q = b2MakeRot(0.25 * <f32>variant);
	return xf;
}

function gridPose(v: i32, an: i32, dn: i32, rn: i32, reach: f32): b2Transform {
	return b2InvMulTransforms(poseA(v), ringPose(an, dn, rn, reach));
}

function chainSegment(variant: i32): b2ChainSegment {
	let cs: b2ChainSegment;
	cs.segment.point1 = b2Vec2(-0.625, 0.0);
	cs.segment.point2 = b2Vec2(0.625, 0.0625);
	if (variant == 0) {
		cs.ghost1 = b2Vec2(-1.875, -0.03125);
		cs.ghost2 = b2Vec2(1.875, 0.125);
	} else if (variant == 1) {
		cs.ghost1 = b2Vec2(-1.5, 0.875);
		cs.ghost2 = b2Vec2(1.5, 1.0);
	} else if (variant == 2) {
		cs.ghost1 = b2Vec2(-1.5, -0.875);
		cs.ghost2 = b2Vec2(1.5, -1.0);
	} else {
		cs.ghost1 = b2Vec2(-1.375, 0.75);
		cs.ghost2 = b2Vec2(1.625, -0.6875);
	}
	return cs;
}

function randChainSegment(): b2ChainSegment {
	let cs: b2ChainSegment;
	cs.segment = randSegment();
	let d = b2Sub(cs.segment.point2, cs.segment.point1);
	let bend1 = randVec(-1.0, 1.0);
	let bend2 = randVec(-1.0, 1.0);
	cs.ghost1 = b2Add(b2Sub(cs.segment.point1, d), bend1);
	cs.ghost2 = b2Add(b2Add(cs.segment.point2, d), bend2);
	return cs;
}

function out(tag: string, m: b2LocalManifold): void {
	vtag(tag);
	plocal(m);
	vend();
}

function outCache(tag: string, m: b2LocalManifold, before: b2SimplexCache, after: b2SimplexCache): void {
	vtag(tag);
	pcache(before);
	plocal(m);
	pcache(after);
	vend();
}

function grids(): void {
	for (let v = 0; v < 3; ++v) {
		let circleA = b2Circle(b2Vec2(0.0625, -0.03125), 0.375);
		let circleB = b2Circle(b2Vec2(-0.03125, 0.0625), 0.3125);
		let capsuleA = b2Capsule(b2Vec2(-0.4375, 0.0625), b2Vec2(0.5, -0.0625), 0.25);
		let segmentA = b2Segment(b2Vec2(-0.625, 0.0625), b2Vec2(0.6875, -0.125));
		let polyA = libraryPolygon(v + 1, (v & 1) != 0 ? 0.0625 : 0.0);

		for (let an = 0; an < RING_ANGLES; ++an) {
			for (let dn = 0; dn < RING_DEPTHS; ++dn) {
				for (let rn = 0; rn < RING_ROTS; ++rn) {
					out("gcc", b2CollideCircles(circleA, circleB, gridPose(v, an, dn, rn, 0.6875)));
					out("gkc", b2CollideCapsuleAndCircle(capsuleA, circleB, gridPose(v, an, dn, rn, 0.5625)));
					out("gsc", b2CollideSegmentAndCircle(segmentA, circleB, gridPose(v, an, dn, rn, 0.3125)));
					out("gpc", b2CollidePolygonAndCircle(polyA, circleB, gridPose(v, an, dn, rn, 0.8125)));
				}
			}
		}
	}

	for (let v = 0; v < 4; ++v) {
		let capsuleA = b2Capsule(b2Vec2(-0.5, 0.0), b2Vec2(0.5, 0.0), 0.1875);
		let segmentA = b2Segment(b2Vec2(-0.6875, -0.0625), b2Vec2(0.625, 0.125));
		let polyA = libraryPolygon(v + 2, v % 3 == 0 ? 0.0625 : 0.0);
		// Capsule B: general, equal and parallel, perpendicular, short
		let capsuleB: b2Capsule;
		if (v == 0) {
			capsuleB = b2Capsule(b2Vec2(-0.375, 0.03125), b2Vec2(0.4375, -0.0625), 0.15625);
		} else if (v == 1) {
			capsuleB = b2Capsule(b2Vec2(-0.5, 0.0), b2Vec2(0.5, 0.0), 0.1875);
		} else if (v == 2) {
			capsuleB = b2Capsule(b2Vec2(0.0, -0.4375), b2Vec2(0.0, 0.5), 0.1875);
		} else {
			capsuleB = b2Capsule(b2Vec2(-0.125, 0.0), b2Vec2(0.125, 0.03125), 0.25);
		}

		for (let an = 0; an < RING_ANGLES; ++an) {
			for (let dn = 0; dn < RING_DEPTHS; ++dn) {
				for (let rn = 0; rn < RING_ROTS; ++rn) {
					let va = v % 3;
					out("gkk", b2CollideCapsules(capsuleA, capsuleB, gridPose(va, an, dn, rn, 0.75)));
					out("gsk", b2CollideSegmentAndCapsule(segmentA, capsuleB, gridPose(va, an, dn, rn, 0.5625)));
					out("gpk", b2CollidePolygonAndCapsule(polyA, capsuleB, gridPose(va, an, dn, rn, 0.875)));
				}
			}
		}
	}

	for (let wa = 0; wa < 6; ++wa) {
		let polyA = libraryPolygon(wa, (wa & 1) != 0 ? 0.0625 : 0.0);
		let segmentA = b2Segment(b2Vec2(-0.8125, 0.0), b2Vec2(0.75, 0.0625));
		for (let wb = 0; wb < 6; ++wb) {
			let polyB = libraryPolygon(wb + 2, wb % 3 == 0 ? 0.046875 : 0.0);
			if (wb == 5) {
				polyB = b2MakeBox(0.5, 0.25);
			}

			for (let an = 0; an < RING_ANGLES; ++an) {
				for (let dn = 0; dn < RING_DEPTHS; ++dn) {
					let rn = (an + dn + wa + wb) % RING_ROTS;
					let v = (wa + wb) % 3;
					out("gpp", b2CollidePolygons(polyA, polyB, gridPose(v, an, dn, rn, 1.0)));
					if (wa == 0) {
						out("gsp", b2CollideSegmentAndPolygon(segmentA, polyB, gridPose(v, an, dn, rn, 0.6875)));
					}
				}
			}
		}
	}

	// Box on box faces: aligned and nearly aligned edges where clipping and the vertex-vertex fallback meet
	for (let i = 0; i < 64; ++i) {
		let boxA = b2MakeBox(0.5, 0.5);
		let boxB = b2MakeRoundedBox(0.5, 0.25, (i & 1) != 0 ? 0.03125 : 0.0);
		let xf: b2Transform;
		xf.p = b2Vec2(0.0625 * <f32>(i % 8) - 0.25, 0.75 + 0.0078125 * <f32>(i / 8) - 0.03125);
		xf.q = b2MakeRot(0.001953125 * <f32>((i % 5) - 2));
		out("gbb", b2CollidePolygons(boxA, boxB, xf));
	}

	for (let v = 0; v < 4; ++v) {
		let cs = chainSegment(v);
		let circleB = b2Circle(b2Vec2(0.0, 0.03125), 0.25);
		let capsuleB = b2Capsule(b2Vec2(-0.3125, 0.0), b2Vec2(0.3125, 0.0625), 0.1875);

		for (let w = 0; w < 3; ++w) {
			let polyB = libraryPolygon(w + 1, (w & 1) != 0 ? 0.0625 : 0.0);
			let warmK: b2SimplexCache;
			let warmP: b2SimplexCache;

			for (let an = 0; an < RING_ANGLES; ++an) {
				for (let dn = 0; dn < RING_DEPTHS; ++dn) {
					for (let rn = 0; rn < RING_ROTS; ++rn) {
						let va = v % 3;
						if (w == 0) {
							out("gxc", b2CollideChainSegmentAndCircle(cs, circleB, gridPose(va, an, dn, rn, 0.5)));
						}

						let cold: b2SimplexCache;
						let before = cold;
						let xf = gridPose(va, an, dn, rn, 0.5625);
						let m = b2CollideChainSegmentAndCapsule(cs, capsuleB, xf, cold);
						outCache("gxk", m, before, cold);

						before = warmK;
						m = b2CollideChainSegmentAndCapsule(cs, capsuleB, xf, warmK);
						outCache("gxkw", m, before, warmK);

						let empty: b2SimplexCache;
						cold = empty;
						before = cold;
						xf = gridPose(va, an, dn, rn, 0.6875);
						m = b2CollideChainSegmentAndPolygon(cs, polyB, xf, cold);
						outCache("gxp", m, before, cold);

						before = warmP;
						m = b2CollideChainSegmentAndPolygon(cs, polyB, xf, warmP);
						outCache("gxpw", m, before, warmP);
					}
				}
			}
		}
	}
}

function randoms(): void {
	let cacheK: b2SimplexCache;
	let cacheP: b2SimplexCache;
	let empty: b2SimplexCache;

	for (let i = 0; i < 6000; ++i) {
		let circleA = randCircle();
		let circleB = randCircle();
		let capsuleA = randCapsule();
		let capsuleB = randCapsule();
		let segmentA = randSegment();
		let polyA = randPolygon();
		let polyB = randPolygon();
		let cs = randChainSegment();
		let xf = randXf(1.25);

		out("rcc", b2CollideCircles(circleA, circleB, xf));
		out("rkc", b2CollideCapsuleAndCircle(capsuleA, circleB, xf));
		out("rsc", b2CollideSegmentAndCircle(segmentA, circleB, xf));
		out("rpc", b2CollidePolygonAndCircle(polyA, circleB, xf));
		out("rkk", b2CollideCapsules(capsuleA, capsuleB, xf));
		out("rsk", b2CollideSegmentAndCapsule(segmentA, capsuleB, xf));
		out("rpk", b2CollidePolygonAndCapsule(polyA, capsuleB, xf));
		out("rpp", b2CollidePolygons(polyA, polyB, xf));
		out("rsp", b2CollideSegmentAndPolygon(segmentA, polyB, xf));
		out("rxc", b2CollideChainSegmentAndCircle(cs, circleB, xf));

		// The polygon cache carries over only when the next polygon still has the cached vertices
		if ((vrand32() & 3) == 0) {
			cacheK = empty;
		}
		let before = cacheK;
		let m = b2CollideChainSegmentAndCapsule(cs, capsuleB, xf, cacheK);
		outCache("rxk", m, before, cacheK);

		cacheP = empty;
		before = cacheP;
		m = b2CollideChainSegmentAndPolygon(cs, polyB, xf, cacheP);
		outCache("rxp", m, before, cacheP);

		// Warm repeat at a nearby pose
		let xf2 = xf;
		let nudge = randVec(-0.03125, 0.03125);
		xf2.p = b2Add(xf2.p, nudge);
		before = cacheP;
		m = b2CollideChainSegmentAndPolygon(cs, polyB, xf2, cacheP);
		outCache("rxpw", m, before, cacheP);
	}
}

// Polygons around the segment ends, where vertex-vertex and ghost corner cases live
function chainCorners(): void {
	for (let i = 0; i < 20000; ++i) {
		let cs = (i & 1) != 0 ? chainSegment((i >> 1) & 3) : randChainSegment();
		let polyB = randPolygon();
		let base = (vrand32() & 1) != 0 ? cs.segment.point1 : cs.segment.point2;
		let xf: b2Transform;
		xf.p = b2Add(base, randVec(-1.0, 1.0));
		xf.q = randRot();
		let cache: b2SimplexCache;
		let m = b2CollideChainSegmentAndPolygon(cs, polyB, xf, cache);
		vtag("cx");
		plocal(m);
		pcache(cache);
		vend();
	}
}

export function run(): void {
	vseed(51);
	grids();
	randoms();
	chainCorners();
}
