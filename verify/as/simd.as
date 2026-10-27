// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/simd.as, mirrors verify/cref/simd.c. Every b2FloatW result is also computed through
// the lane generic L helpers, per lane as f32 and, with --enable-simd, as v128; a lane that differs
// from the b2FloatW result prints an extra line, which C never prints.

import { vtag, vi32, vbool, vf32, vend, vseed, vrand32, vrandint, vrandf, vspecialf } from "./common";
import { b2AABB } from "../../lib/math_functions";
import { b2TreeNode } from "../../lib/collision";
import {
	b2AABBV,
	b2AddL,
	b2AddW,
	b2AllZeroW,
	b2AndL,
	b2AndW,
	b2AnyTrueL,
	b2AnyTrueW,
	b2BlendL,
	b2BlendW,
	b2CrossL,
	b2CrossW,
	b2DivL,
	b2DivW,
	b2DotL,
	b2DotW,
	b2FloatW,
	b2GreaterThanL,
	b2GreaterThanW,
	b2LessThanL,
	b2LessThanW,
	b2LoadAABBV,
	b2LoadU,
	b2MaskValueL,
	b2MaxL,
	b2MaxW,
	b2MinL,
	b2MinW,
	b2MulAddL,
	b2MulAddW,
	b2MulL,
	b2MulSubL,
	b2MulSubW,
	b2MulW,
	b2NegL,
	b2NegW,
	b2OverlapNode,
	b2OverlapNodePairMask,
	b2OverlapPairMask,
	b2OverlapV,
	b2RightPerpW,
	b2RotW,
	b2RotateVectorW,
	b2SetL,
	b2SetW,
	b2SoftMaskW,
	b2SplatW,
	b2StoreAABBV,
	b2StoreW,
	b2SubL,
	b2SubVW,
	b2SubW,
	b2SymClampL,
	b2SymClampW,
	b2UnionPairV,
	b2UnionV,
	b2UnpackHiL,
	b2UnpackHiW,
	b2UnpackLoL,
	b2UnpackLoW,
	b2Vec2W,
	b2ZeroW,
} from "../../lib/simd";

export { frameBoundary } from "./common";

function pw(a: b2FloatW): void {
	vf32(a.x);
	vf32(a.y);
	vf32(a.z);
	vf32(a.w);
}

function pab(a: b2AABB): void {
	vf32(a.lowerBound.x);
	vf32(a.lowerBound.y);
	vf32(a.upperBound.x);
	vf32(a.upperBound.y);
}

// mode 0 special, 1 tiny, 2 ordinary, 3 small integers so lanes tie and hit zero
function rlane(mode: i32): f32 {
	if (mode == 0) {
		return vspecialf();
	}
	if (mode == 1) {
		return vrandf(-1.0e-3, 1.0e-3);
	}
	if (mode == 2) {
		return vrandf(-10.0, 10.0);
	}
	return <f32>vrandint(-2, 3);
}

function rw(): b2FloatW {
	let a: b2FloatW;
	let mode = <i32>(vrand32() % 4);
	a.x = rlane(mode);
	a.y = rlane(mode);
	a.z = rlane(mode);
	a.w = rlane(mode);
	return a;
}

// The scalar path's masks are 1.0 or 0.0
function rmask(): b2FloatW {
	let m: b2FloatW;
	m.x = <f32>(vrand32() & 1);
	m.y = <f32>(vrand32() & 1);
	m.z = <f32>(vrand32() & 1);
	m.w = <f32>(vrand32() & 1);
	return m;
}

function raabb(): b2AABB {
	let a: b2AABB;
	a.lowerBound.x = vrandf(-10.0, 10.0);
	a.lowerBound.y = vrandf(-10.0, 10.0);
	let ex = vrandf(-1.0, 6.0);
	let ey = vrandf(-1.0, 6.0);
	a.upperBound.x = a.lowerBound.x + ex;
	a.upperBound.y = a.lowerBound.y + ey;
	return a;
}

function lane(a: b2FloatW, k: i32): f32 {
	return k == 0 ? a.x : k == 1 ? a.y : k == 2 ? a.z : a.w;
}

// Bits equal, or both NaN
function same(x: f32, y: f32): bool {
	return reinterpret<u32>(x) == reinterpret<u32>(y) || (isNaN(x) && isNaN(y));
}

const OP_COUNT = 16;

// One f32 lane through the L helpers, in op order
function laneOps(a: f32, b: f32, c: f32, m: f32, out: StaticArray<f32>): void {
	out[0] = b2NegL<f32>(a);
	out[1] = b2AddL<f32>(a, b);
	out[2] = b2SubL<f32>(a, b);
	out[3] = b2MulL<f32>(a, b);
	out[4] = b2DivL<f32>(a, b);
	out[5] = b2MulAddL<f32>(a, b, c);
	out[6] = b2MulSubL<f32>(a, b, c);
	out[7] = b2MinL<f32>(a, b);
	out[8] = b2MaxL<f32>(a, b);
	out[9] = b2SymClampL<f32>(a, b);
	out[10] = b2MaskValueL<f32>(b2GreaterThanL<f32>(a, b));
	out[11] = b2MaskValueL<f32>(b2LessThanL<f32>(a, b));
	out[12] = b2MaskValueL<f32>(b2AndL<f32>(b2GreaterThanL<f32>(a, b), b2LessThanL<f32>(c, b)));
	out[13] = b2BlendL<f32>(a, b, m);
	out[14] = b2DotL<f32>(a, b, c, a);
	out[15] = b2CrossL<f32>(a, b, c, a);
}

// The four lanes through the v128 L helpers, in op order
function wideOps(a: b2FloatW, b: b2FloatW, c: b2FloatW, m: b2FloatW, out: StaticArray<f32>): void {
	if (ASC_FEATURE_SIMD) {
		let va = b2SetL<v128>(a.x, a.y, a.z, a.w);
		let vb = b2SetL<v128>(b.x, b.y, b.z, b.w);
		let vc = b2SetL<v128>(c.x, c.y, c.z, c.w);
		let vm = f32x4.ne(b2SetL<v128>(m.x, m.y, m.z, m.w), f32x4.splat(0.0));
		let results = new StaticArray<v128>(OP_COUNT + 2);
		results[0] = b2NegL<v128>(va);
		results[1] = b2AddL<v128>(va, vb);
		results[2] = b2SubL<v128>(va, vb);
		results[3] = b2MulL<v128>(va, vb);
		results[4] = b2DivL<v128>(va, vb);
		results[5] = b2MulAddL<v128>(va, vb, vc);
		results[6] = b2MulSubL<v128>(va, vb, vc);
		results[7] = b2MinL<v128>(va, vb);
		results[8] = b2MaxL<v128>(va, vb);
		results[9] = b2SymClampL<v128>(va, vb);
		results[10] = b2MaskValueL<v128>(b2GreaterThanL<v128>(va, vb));
		results[11] = b2MaskValueL<v128>(b2LessThanL<v128>(va, vb));
		results[12] = b2MaskValueL<v128>(b2AndL<v128>(b2GreaterThanL<v128>(va, vb), b2LessThanL<v128>(vc, vb)));
		results[13] = b2BlendL<v128>(va, vb, vm);
		results[14] = b2DotL<v128>(va, vb, vc, va);
		results[15] = b2CrossL<v128>(va, vb, vc, va);
		results[16] = b2UnpackLoL<v128>(va, vb);
		results[17] = b2UnpackHiL<v128>(va, vb);
		for (let op = 0; op < OP_COUNT + 2; ++op) {
			let r = results[op];
			out[4 * op + 0] = f32x4.extract_lane(r, 0);
			out[4 * op + 1] = f32x4.extract_lane(r, 1);
			out[4 * op + 2] = f32x4.extract_lane(r, 2);
			out[4 * op + 3] = f32x4.extract_lane(r, 3);
		}
		out[4 * (OP_COUNT + 2)] = b2AnyTrueL<v128>(vm) ? 1.0 : 0.0;
	}
}

function checkLanes(a: b2FloatW, b: b2FloatW, c: b2FloatW, m: b2FloatW): void {
	let expected = new StaticArray<b2FloatW>(OP_COUNT + 2);
	expected[0] = b2NegW(a);
	expected[1] = b2AddW(a, b);
	expected[2] = b2SubW(a, b);
	expected[3] = b2MulW(a, b);
	expected[4] = b2DivW(a, b);
	expected[5] = b2MulAddW(a, b, c);
	expected[6] = b2MulSubW(a, b, c);
	expected[7] = b2MinW(a, b);
	expected[8] = b2MaxW(a, b);
	expected[9] = b2SymClampW(a, b);
	expected[10] = b2GreaterThanW(a, b);
	expected[11] = b2LessThanW(a, b);
	expected[12] = b2AndW(b2GreaterThanW(a, b), b2LessThanW(c, b));
	expected[13] = b2BlendW(a, b, m);
	let va = b2Vec2W(a, b);
	let vb = b2Vec2W(c, a);
	expected[14] = b2DotW(va, vb);
	expected[15] = b2CrossW(va, vb);
	expected[16] = b2UnpackLoW(a, b);
	expected[17] = b2UnpackHiW(a, b);

	let out = new StaticArray<f32>(OP_COUNT);
	for (let k = 0; k < 4; ++k) {
		laneOps(lane(a, k), lane(b, k), lane(c, k), lane(m, k), out);
		for (let op = 0; op < OP_COUNT; ++op) {
			if (!same(out[op], lane(expected[op], k))) {
				vtag("lane-mismatch-f32");
				vi32(op);
				vi32(k);
				vf32(out[op]);
				vf32(lane(expected[op], k));
				vend();
			}
		}
	}

	if (ASC_FEATURE_SIMD) {
		let wide = new StaticArray<f32>(4 * (OP_COUNT + 2) + 1);
		wideOps(a, b, c, m, wide);
		for (let op = 0; op < OP_COUNT + 2; ++op) {
			for (let k = 0; k < 4; ++k) {
				if (!same(wide[4 * op + k], lane(expected[op], k))) {
					vtag("lane-mismatch-v128");
					vi32(op);
					vi32(k);
					vf32(wide[4 * op + k]);
					vf32(lane(expected[op], k));
					vend();
				}
			}
		}
		if ((wide[4 * (OP_COUNT + 2)] != 0.0) != b2AnyTrueW(m)) {
			vtag("lane-mismatch-anytrue");
			vend();
		}
	}
}

export function run(): void {
	vseed(7);

	for (let i = 0; i < 20000; ++i) {
		let a = rw();
		let b = rw();
		let c = rw();
		let mask = rmask();
		let mask2 = rmask();

		vtag("in");
		pw(a);
		pw(b);
		pw(c);
		pw(mask);
		vend();

		vtag("arith");
		pw(b2NegW(a));
		pw(b2AddW(a, b));
		pw(b2SubW(a, b));
		pw(b2MulW(a, b));
		pw(b2DivW(a, b));
		pw(b2MulAddW(a, b, c));
		pw(b2MulSubW(a, b, c));
		vend();

		vtag("cmp");
		pw(b2MinW(a, b));
		pw(b2MaxW(a, b));
		pw(b2SymClampW(a, b));
		pw(b2GreaterThanW(a, b));
		pw(b2LessThanW(a, b));
		pw(b2AndW(mask, mask2));
		pw(b2AndW(b2GreaterThanW(a, b), b2LessThanW(c, b)));
		vbool(b2AnyTrueW(mask));
		vbool(b2AllZeroW(a));
		vbool(b2AllZeroW(mask));
		pw(b2BlendW(a, b, mask));
		pw(b2BlendW(a, b, b2GreaterThanW(a, c)));
		vend();

		vtag("lanes");
		pw(b2UnpackLoW(a, b));
		pw(b2UnpackHiW(a, b));
		pw(b2SplatW(a.y));
		pw(b2SetW(c.w, b.z, a.y, c.x));
		pw(b2ZeroW());
		let data = new StaticArray<f32>(8);
		b2StoreW(data, 0, a);
		b2StoreW(data, 4, b);
		pw(b2LoadU(data, 2));
		let indexA: FixedArray<i32, 4>;
		let indexB: FixedArray<i32, 4>;
		for (let k = 0; k < 4; ++k) {
			indexA[k] = vrandint(0, 3);
			indexB[k] = vrandint(0, 3);
		}
		pw(b2SoftMaskW(indexA, indexB));
		vend();

		let va = b2Vec2W(a, b);
		let vb = b2Vec2W(c, a);
		let q = b2RotW(b, c);
		vtag("vec");
		let d = b2SubVW(va, vb);
		pw(d.X);
		pw(d.Y);
		let p = b2RightPerpW(va);
		pw(p.X);
		pw(p.Y);
		pw(b2DotW(va, vb));
		pw(b2CrossW(va, vb));
		let r = b2RotateVectorW(q, va);
		pw(r.X);
		pw(r.Y);
		vend();

		checkLanes(a, b, c, mask);
	}

	for (let i = 0; i < 5000; ++i) {
		let nodes = new StaticArray<b2TreeNode>(4);
		for (let k = 0; k < 4; ++k) {
			nodes[k].aabb = raabb();
		}
		let a = raabb();
		let b = raabb();
		let condition = (vrand32() & 1) != 0;

		let av: b2AABBV = b2LoadAABBV(a);
		vtag("tree");
		vbool(b2OverlapNode(av, nodes[1]));
		vbool(b2OverlapV(a, b));
		pab(b2UnionV(a, b));
		pab(b2UnionPairV(nodes, 2));
		let holder = new StaticArray<b2AABB>(1);
		holder[0] = b;
		b2StoreAABBV(holder[0], av, condition);
		pab(holder[0]);
		vi32(b2OverlapPairMask(nodes, 0, nodes, 2));
		vi32(b2OverlapNodePairMask(av, nodes, 2));
		vend();
	}
}
