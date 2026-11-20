// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/simd.h and src/simd.c, plus the B2_SIMD_NONE lane math of
// src/contact_solver_wide.inl, the 4 lane path as C compiles it with BOX2D_DISABLE_SIMD

import { B2_ASSERT, B2_ASSERT_ENABLED } from "./core";
import { b2AABB, b2AABB_Union, b2ClampFloat } from "./math_functions";
import { b2TreeNode } from "./collision";

// simd.h, the scalar branch

export type b2AABBV = b2AABB;

@inline export function b2LoadAABBV(aabb: b2AABB): b2AABBV {
	return aabb;
}

@inline export function b2OverlapNode(av: b2AABBV, node: b2TreeNode): bool {
	return (
		av.lowerBound.x <= node.aabb.upperBound.x &&
		av.lowerBound.y <= node.aabb.upperBound.y &&
		node.aabb.lowerBound.x <= av.upperBound.x &&
		node.aabb.lowerBound.y <= av.upperBound.y
	);
}

@inline export function b2OverlapV(a: b2AABB, b: b2AABB): bool {
	return a.lowerBound.x <= b.upperBound.x && a.lowerBound.y <= b.upperBound.y && b.lowerBound.x <= a.upperBound.x && b.lowerBound.y <= a.upperBound.y;
}

@inline export function b2UnionV(a: b2AABB, b: b2AABB): b2AABB {
	return b2AABB_Union(a, b);
}

// C takes a pointer to the pair; here the node array plus the index of the first node
@inline export function b2UnionPairV<A>(nodes: A, pair: i32): b2AABBV {
	return b2AABB_Union(nodes[pair].aabb, nodes[pair + 1].aabb);
}

@inline export function b2StoreAABBV(aabb: ref b2AABB, value: b2AABBV, condition: bool): void {
	if (condition) {
		aabb = value;
	}
}

@inline export function b2OverlapPairMask<A, B>(nodesA: A, pairA: i32, nodesB: B, pairB: i32): i32 {
	let mask = 0;
	mask |= b2OverlapV(nodesA[pairA].aabb, nodesB[pairB].aabb) ? 1 : 0;
	mask |= b2OverlapV(nodesA[pairA].aabb, nodesB[pairB + 1].aabb) ? 2 : 0;
	mask |= b2OverlapV(nodesA[pairA + 1].aabb, nodesB[pairB].aabb) ? 4 : 0;
	mask |= b2OverlapV(nodesA[pairA + 1].aabb, nodesB[pairB + 1].aabb) ? 8 : 0;
	return mask;
}

@inline export function b2OverlapNodePairMask<A>(av: b2AABBV, nodes: A, pair: i32): i32 {
	let mask = 0;
	mask |= b2OverlapNode(av, nodes[pair]) ? 1 : 0;
	mask |= b2OverlapNode(av, nodes[pair + 1]) ? 2 : 0;
	return mask;
}

// simd.c

let b2_simdWidth: i32 = 0;

function b2DetectSIMDWidth(): i32 {
	return 4;
}

export function b2GetSIMDWidth(): i32 {
	let width = b2_simdWidth;
	if (width == 0) {
		width = b2DetectSIMDWidth();
		b2_simdWidth = width;
	}

	return width;
}

/// @return true if the current CPU has the AVX2 instruction set.
export function b2IsAVX2Available(): bool {
	return b2DetectSIMDWidth() == 8;
}

export function b2SetSIMDWidth(width: i32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(width == 0 || width == 4 || width == 8);
	if (width == 8 && b2DetectSIMDWidth() != 8) {
		width = 4;
	}

	b2_simdWidth = width;
}

// contact_solver_wide.inl at B2_SIMD_WIDTH 4, the scalar branch

export const B2_SIMD_WIDTH: i32 = 4;

// scalar math
export struct b2FloatW {
	x: f32 = 0;
	y: f32 = 0;
	z: f32 = 0;
	w: f32 = 0;

	constructor(x: f32, y: f32, z: f32, w: f32) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}
}

export struct b2Vec2W {
	X: b2FloatW = b2FloatW();
	Y: b2FloatW = b2FloatW();

	constructor(X: b2FloatW, Y: b2FloatW) {
		this.X = X;
		this.Y = Y;
	}
}

export struct b2RotW {
	C: b2FloatW = b2FloatW();
	S: b2FloatW = b2FloatW();

	constructor(C: b2FloatW, S: b2FloatW) {
		this.C = C;
		this.S = S;
	}
}

@inline export function b2ZeroW(): b2FloatW {
	return b2FloatW(0.0, 0.0, 0.0, 0.0);
}

@inline export function b2SplatW(scalar: f32): b2FloatW {
	return b2FloatW(scalar, scalar, scalar, scalar);
}

@inline export function b2SetW(a: f32, b: f32, c: f32, d: f32): b2FloatW {
	return b2FloatW(a, b, c, d);
}

@inline export function b2NegW(a: b2FloatW): b2FloatW {
	return b2FloatW(-a.x, -a.y, -a.z, -a.w);
}

@inline export function b2AddW(a: b2FloatW, b: b2FloatW): b2FloatW {
	return b2FloatW(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

@inline export function b2SubW(a: b2FloatW, b: b2FloatW): b2FloatW {
	return b2FloatW(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

@inline export function b2MulW(a: b2FloatW, b: b2FloatW): b2FloatW {
	return b2FloatW(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

@inline export function b2DivW(a: b2FloatW, b: b2FloatW): b2FloatW {
	return b2FloatW(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

@inline export function b2MulAddW(a: b2FloatW, b: b2FloatW, c: b2FloatW): b2FloatW {
	return b2FloatW(a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w);
}

@inline export function b2MulSubW(a: b2FloatW, b: b2FloatW, c: b2FloatW): b2FloatW {
	return b2FloatW(a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w);
}

@inline export function b2MinW(a: b2FloatW, b: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	return r;
}

@inline export function b2MaxW(a: b2FloatW, b: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

// a = clamp(a, -b, b)
@inline export function b2SymClampW(a: b2FloatW, b: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = b2ClampFloat(a.x, -b.x, b.x);
	r.y = b2ClampFloat(a.y, -b.y, b.y);
	r.z = b2ClampFloat(a.z, -b.z, b.z);
	r.w = b2ClampFloat(a.w, -b.w, b.w);
	return r;
}

@inline export function b2AndW(a: b2FloatW, b: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = a.x != 0.0 && b.x != 0.0 ? 1.0 : 0.0;
	r.y = a.y != 0.0 && b.y != 0.0 ? 1.0 : 0.0;
	r.z = a.z != 0.0 && b.z != 0.0 ? 1.0 : 0.0;
	r.w = a.w != 0.0 && b.w != 0.0 ? 1.0 : 0.0;
	return r;
}

@inline export function b2GreaterThanW(a: b2FloatW, b: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = a.x > b.x ? 1.0 : 0.0;
	r.y = a.y > b.y ? 1.0 : 0.0;
	r.z = a.z > b.z ? 1.0 : 0.0;
	r.w = a.w > b.w ? 1.0 : 0.0;
	return r;
}

@inline export function b2LessThanW(a: b2FloatW, b: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = a.x < b.x ? 1.0 : 0.0;
	r.y = a.y < b.y ? 1.0 : 0.0;
	r.z = a.z < b.z ? 1.0 : 0.0;
	r.w = a.w < b.w ? 1.0 : 0.0;
	return r;
}

@inline export function b2AnyTrueW(mask: b2FloatW): bool {
	return mask.x != 0.0 || mask.y != 0.0 || mask.z != 0.0 || mask.w != 0.0;
}

@inline export function b2AllZeroW(a: b2FloatW): bool {
	return a.x == 0.0 && a.y == 0.0 && a.z == 0.0 && a.w == 0.0;
}

// C takes a float pointer; here the array plus the element index it points at
@inline export function b2LoadU(data: StaticArray<f32>, index: i32): b2FloatW {
	return b2FloatW(data[index], data[index + 1], data[index + 2], data[index + 3]);
}

@inline export function b2StoreW(data: StaticArray<f32>, index: i32, a: b2FloatW): void {
	data[index] = a.x;
	data[index + 1] = a.y;
	data[index + 2] = a.z;
	data[index + 3] = a.w;
}

// component-wise returns mask ? b : a
@inline export function b2BlendW(a: b2FloatW, b: b2FloatW, mask: b2FloatW): b2FloatW {
	let r: b2FloatW;
	r.x = mask.x != 0.0 ? b.x : a.x;
	r.y = mask.y != 0.0 ? b.y : a.y;
	r.z = mask.z != 0.0 ? b.z : a.z;
	r.w = mask.w != 0.0 ? b.w : a.w;
	return r;
}

@inline export function b2UnpackLoW(a: b2FloatW, b: b2FloatW): b2FloatW {
	return b2FloatW(a.x, b.x, a.y, b.y);
}

@inline export function b2UnpackHiW(a: b2FloatW, b: b2FloatW): b2FloatW {
	return b2FloatW(a.z, b.z, a.w, b.w);
}

@inline export function b2SoftMaskW(indexA: FixedArray<i32, 4>, indexB: FixedArray<i32, 4>): b2FloatW {
	let r: b2FloatW;
	r.x = indexA[0] == 0 || indexB[0] == 0 ? 1.0 : 0.0;
	r.y = indexA[1] == 0 || indexB[1] == 0 ? 1.0 : 0.0;
	r.z = indexA[2] == 0 || indexB[2] == 0 ? 1.0 : 0.0;
	r.w = indexA[3] == 0 || indexB[3] == 0 ? 1.0 : 0.0;
	return r;
}

// a - b
@inline export function b2SubVW(a: b2Vec2W, b: b2Vec2W): b2Vec2W {
	return b2Vec2W(b2SubW(a.X, b.X), b2SubW(a.Y, b.Y));
}

@inline export function b2RightPerpW(a: b2Vec2W): b2Vec2W {
	return b2Vec2W(a.Y, b2NegW(a.X));
}

@inline export function b2DotW(a: b2Vec2W, b: b2Vec2W): b2FloatW {
	return b2AddW(b2MulW(a.X, b.X), b2MulW(a.Y, b.Y));
}

@inline export function b2CrossW(a: b2Vec2W, b: b2Vec2W): b2FloatW {
	return b2SubW(b2MulW(a.X, b.Y), b2MulW(a.Y, b.X));
}

@inline export function b2RotateVectorW(q: b2RotW, v: b2Vec2W): b2Vec2W {
	return b2Vec2W(b2SubW(b2MulW(q.C, v.X), b2MulW(q.S, v.Y)), b2AddW(b2MulW(q.S, v.X), b2MulW(q.C, v.Y)));
}

// Kernel lane math generic over T: v128 under ASC_FEATURE_SIMD, else f32 run once per lane, since v128
// cannot appear in the scalar artifact. Masks are all ones lanes in v128 and 1.0 or 0.0 in f32.

@inline export function b2SplatL<T>(a: f32): T {
	if (isFloat<T>()) {
		return a;
	} else {
		return f32x4.splat(a);
	}
}

@inline export function b2ZeroL<T>(): T {
	return b2SplatL<T>(0.0);
}

@inline export function b2LoadL<T>(p: usize): T {
	if (isFloat<T>()) {
		return load<f32>(p);
	} else {
		return v128.load(p);
	}
}

@inline export function b2StoreL<T>(p: usize, a: T): void {
	if (isFloat<T>()) {
		store<f32>(p, a);
	} else {
		v128.store(p, a);
	}
}

// The lanes a, b, c, d; a lane kernel takes a
@inline export function b2SetL<T>(a: f32, b: f32, c: f32, d: f32): T {
	if (isFloat<T>()) {
		return a;
	} else {
		return f32x4.replace_lane(f32x4.replace_lane(f32x4.replace_lane(f32x4.splat(a), 1, b), 2, c), 3, d);
	}
}

// b2SetL of the floats at four addresses, read straight into their lanes; a lane kernel reads a
@inline export function b2SetLoadL<T>(a: usize, b: usize, c: usize, d: usize): T {
	if (isFloat<T>()) {
		return load<f32>(a);
	} else {
		let v = f32x4.splat(load<f32>(a));
		v = v128.load32_lane(b, v, 1);
		v = v128.load32_lane(c, v, 2);
		return v128.load32_lane(d, v, 3);
	}
}

// ( (const float*)&w )[lane]
@inline export function b2LaneL<T>(a: T, lane: i32): f32 {
	if (isFloat<T>()) {
		return a;
	} else {
		if (lane == 0) {
			return f32x4.extract_lane(a, 0);
		}
		if (lane == 1) {
			return f32x4.extract_lane(a, 1);
		}
		if (lane == 2) {
			return f32x4.extract_lane(a, 2);
		}
		return f32x4.extract_lane(a, 3);
	}
}

@inline export function b2AddL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a + b;
	} else {
		return f32x4.add(a, b);
	}
}

@inline export function b2SubL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a - b;
	} else {
		return f32x4.sub(a, b);
	}
}

@inline export function b2MulL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a * b;
	} else {
		return f32x4.mul(a, b);
	}
}

@inline export function b2DivL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a / b;
	} else {
		return f32x4.div(a, b);
	}
}

@inline export function b2NegL<T>(a: T): T {
	if (isFloat<T>()) {
		return -a;
	} else {
		return f32x4.neg(a);
	}
}

// a + b * c, never fused
@inline export function b2MulAddL<T>(a: T, b: T, c: T): T {
	return b2AddL(a, b2MulL(b, c));
}

// a - b * c
@inline export function b2MulSubL<T>(a: T, b: T, c: T): T {
	return b2SubL(a, b2MulL(b, c));
}

// a <= b ? a : b
@inline export function b2MinL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a <= b ? a : b;
	} else {
		return v128.bitselect(a, b, f32x4.le(a, b));
	}
}

// a >= b ? a : b
@inline export function b2MaxL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a >= b ? a : b;
	} else {
		return v128.bitselect(a, b, f32x4.ge(a, b));
	}
}

// b2ClampFloat( a, -b, b ): a < -b ? -b : ( a > b ? b : a )
@inline export function b2SymClampL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return b2ClampFloat(a, -b, b);
	} else {
		let nb = f32x4.neg(b);
		let upper = v128.bitselect(b, a, f32x4.gt(a, b));
		return v128.bitselect(nb, upper, f32x4.lt(a, nb));
	}
}

@inline export function b2GreaterThanL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a > b ? <f32>1.0 : <f32>0.0;
	} else {
		return f32x4.gt(a, b);
	}
}

@inline export function b2LessThanL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a < b ? <f32>1.0 : <f32>0.0;
	} else {
		return f32x4.lt(a, b);
	}
}

@inline export function b2AndL<T>(a: T, b: T): T {
	if (isFloat<T>()) {
		return a != 0.0 && b != 0.0 ? <f32>1.0 : <f32>0.0;
	} else {
		return v128.and(a, b);
	}
}

// mask ? b : a
@inline export function b2BlendL<T>(a: T, b: T, mask: T): T {
	if (isFloat<T>()) {
		return mask != 0.0 ? b : a;
	} else {
		return v128.bitselect(b, a, mask);
	}
}

// A mask read back from memory, where it is stored as 1.0 or 0.0; true where the scalar path tests != 0
@inline export function b2MaskL<T>(a: T): T {
	if (isFloat<T>()) {
		return a;
	} else {
		return f32x4.ne(a, f32x4.splat(0.0));
	}
}

// A mask as the 1.0 or 0.0 the scalar path stores
@inline export function b2MaskValueL<T>(mask: T): T {
	if (isFloat<T>()) {
		return mask;
	} else {
		return v128.and(mask, f32x4.splat(1.0));
	}
}

// b2AnyTrueW over the lanes a kernel runs; a lane kernel tests its own lane
@inline export function b2AnyTrueL<T>(mask: T): bool {
	if (isFloat<T>()) {
		return mask != 0.0;
	} else {
		return v128.any_true(mask);
	}
}

// aX bX + aY bY
@inline export function b2DotL<T>(aX: T, aY: T, bX: T, bY: T): T {
	return b2AddL(b2MulL(aX, bX), b2MulL(aY, bY));
}

// aX bY - aY bX
@inline export function b2CrossL<T>(aX: T, aY: T, bX: T, bY: T): T {
	return b2SubL(b2MulL(aX, bY), b2MulL(aY, bX));
}

// C's unpacklo and unpackhi, the 0 4 1 5 and 2 6 3 7 zips; v128 only
@inline export function b2UnpackLoL<T>(a: T, b: T): T {
	return v128.shuffle<f32>(a, b, 0, 4, 1, 5);
}

@inline export function b2UnpackHiL<T>(a: T, b: T): T {
	return v128.shuffle<f32>(a, b, 2, 6, 3, 7);
}

// Four rows of four floats transposed into columns, as SSE2 and Neon gather body states; v128 only
export struct b2ColumnsL<T> {
	c0: T;
	c1: T;
	c2: T;
	c3: T;
}

@inline export function b2TransposeL<T>(r0: T, r1: T, r2: T, r3: T): b2ColumnsL<T> {
	let t0 = v128.shuffle<f32>(r0, r2, 0, 4, 1, 5);
	let t1 = v128.shuffle<f32>(r1, r3, 0, 4, 1, 5);
	let t2 = v128.shuffle<f32>(r0, r2, 2, 6, 3, 7);
	let t3 = v128.shuffle<f32>(r1, r3, 2, 6, 3, 7);
	let c: b2ColumnsL<T>;
	c.c0 = v128.shuffle<f32>(t0, t1, 0, 4, 1, 5);
	c.c1 = v128.shuffle<f32>(t0, t1, 2, 6, 3, 7);
	c.c2 = v128.shuffle<f32>(t2, t3, 0, 4, 1, 5);
	c.c3 = v128.shuffle<f32>(t2, t3, 2, 6, 3, 7);
	return c;
}

// b2AllZeroW over four floats in memory, for decisions a lane kernel shares across lanes
@inline export function b2AllZeroAt(p: usize): bool {
	if (ASC_FEATURE_SIMD) {
		return i32x4.all_true(f32x4.eq(v128.load(p), f32x4.splat(0.0)));
	}
	return load<f32>(p) == 0.0 && load<f32>(p + 4) == 0.0 && load<f32>(p + 8) == 0.0 && load<f32>(p + 12) == 0.0;
}

// Four floats into the 16 bytes at p; lanes only move bits, so both artifacts write the same memory.
@inline export function b2StoreRow(p: usize, x: f32, y: f32, z: f32, w: f32): void {
	if (ASC_FEATURE_SIMD) {
		v128.store(p, f32x4.replace_lane(f32x4.replace_lane(f32x4.replace_lane(f32x4.splat(x), 1, y), 2, z), 3, w));
	} else {
		store<f32>(p, x);
		store<f32>(p + 4, y);
		store<f32>(p + 8, z);
		store<f32>(p + 12, w);
	}
}
