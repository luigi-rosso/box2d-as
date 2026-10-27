// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box2d/math_types.h, include/box2d/math_functions.h and
// src/math_functions.c

import { B2_ASSERT, B2_ASSERT_ENABLED, FLT_EPSILON, FLT_MIN } from "./core";

export { b2SetLengthUnitsPerMeter, b2GetLengthUnitsPerMeter, b2_lengthUnitsPerMeter } from "./core";

// math_types.h

/// 2D vector This can be used to represent a point or free vector
export struct b2Vec2 {
	/// coordinates
	x: f32 = 0;
	y: f32 = 0;

	constructor(x: f32, y: f32) {
		this.x = x;
		this.y = y;
	}
}

/// Cosine and sine pair This uses a custom implementation designed for cross-platform determinism
export struct b2CosSin {
	/// cosine and sine
	cosine: f32 = 0;
	sine: f32 = 0;

	constructor(cosine: f32, sine: f32) {
		this.cosine = cosine;
		this.sine = sine;
	}
}

/// 2D rotation This is similar to using a complex number for rotation
export struct b2Rot {
	/// cosine and sine
	c: f32 = 0;
	s: f32 = 0;

	constructor(c: f32, s: f32) {
		this.c = c;
		this.s = s;
	}
}

/// A 2D rigid transform
export struct b2Transform {
	p: b2Vec2 = b2Vec2();
	q: b2Rot = b2Rot();

	constructor(p: b2Vec2, q: b2Rot) {
		this.p = p;
		this.q = q;
	}
}

/// Alias in single precision.
export type b2Pos = b2Vec2;

/// Alias in single precision.
export type b2WorldTransform = b2Transform;

/// A 2-by-2 Matrix
export struct b2Mat22 {
	/// columns
	cx: b2Vec2 = b2Vec2();
	cy: b2Vec2 = b2Vec2();

	constructor(cx: b2Vec2, cy: b2Vec2) {
		this.cx = cx;
		this.cy = cy;
	}
}

/// Axis-aligned bounding box
export struct b2AABB {
	lowerBound: b2Vec2 = b2Vec2();
	upperBound: b2Vec2 = b2Vec2();

	constructor(lowerBound: b2Vec2, upperBound: b2Vec2) {
		this.lowerBound = lowerBound;
		this.upperBound = upperBound;
	}
}

/// separation = dot(normal, point) - offset
export struct b2Plane {
	normal: b2Vec2 = b2Vec2();
	offset: f32 = 0;

	constructor(normal: b2Vec2, offset: f32) {
		this.normal = normal;
		this.offset = offset;
	}
}

/// https://en.wikipedia.org/wiki/Pi
export const B2_PI: f32 = 3.14159265359;

export const b2Vec2_zero: b2Vec2 = b2Vec2(0.0, 0.0);
export const b2Rot_identity: b2Rot = b2Rot(1.0, 0.0);
export const b2Transform_identity: b2Transform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(1.0, 0.0));
export const b2Mat22_zero: b2Mat22 = b2Mat22(b2Vec2(0.0, 0.0), b2Vec2(0.0, 0.0));

// Initializers valid in both modes: 0.0f promotes to double, the identity rotation is float
export const b2Pos_zero: b2Pos = b2Vec2(0.0, 0.0);
export const b2WorldTransform_identity: b2WorldTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(1.0, 0.0));

// math_functions.h

/// @return the minimum of two integers
export function b2MinInt(a: i32, b: i32): i32 {
	return a < b ? a : b;
}

/// @return the maximum of two integers
export function b2MaxInt(a: i32, b: i32): i32 {
	return a > b ? a : b;
}

/// @return the absolute value of an integer
export function b2AbsInt(a: i32): i32 {
	return a < 0 ? -a : a;
}

/// @return an integer clamped between a lower and upper bound
export function b2ClampInt(a: i32, lower: i32, upper: i32): i32 {
	return a < lower ? lower : a > upper ? upper : a;
}

/// https://en.wikipedia.org/wiki/Floor_and_ceiling_functions
export function b2CeilingInt(numerator: i32, denominator: i32): i32 {
	return (numerator + denominator - 1) / denominator;
}

/// @return the minimum of two floats
export function b2MinFloat(a: f32, b: f32): f32 {
	return a < b ? a : b;
}

/// @return the maximum of two floats
export function b2MaxFloat(a: f32, b: f32): f32 {
	return a > b ? a : b;
}

/// @return the absolute value of a float
export function b2AbsFloat(a: f32): f32 {
	return a < 0 ? -a : a;
}

/// @return a float clamped between a lower and upper bound
export function b2ClampFloat(a: f32, lower: f32, upper: f32): f32 {
	return a < lower ? lower : a > upper ? upper : a;
}

/// Vector dot product
export function b2Dot(a: b2Vec2, b: b2Vec2): f32 {
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
export function b2Cross(a: b2Vec2, b: b2Vec2): f32 {
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
export function b2CrossVS(v: b2Vec2, s: f32): b2Vec2 {
	return b2Vec2(s * v.y, -s * v.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
export function b2CrossSV(s: f32, v: b2Vec2): b2Vec2 {
	return b2Vec2(-s * v.y, s * v.x);
}

/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
export function b2LeftPerp(v: b2Vec2): b2Vec2 {
	return b2Vec2(-v.y, v.x);
}

/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
export function b2RightPerp(v: b2Vec2): b2Vec2 {
	return b2Vec2(v.y, -v.x);
}

/// Vector addition
export function b2Add(a: b2Vec2, b: b2Vec2): b2Vec2 {
	return b2Vec2(a.x + b.x, a.y + b.y);
}

/// Vector subtraction
export function b2Sub(a: b2Vec2, b: b2Vec2): b2Vec2 {
	return b2Vec2(a.x - b.x, a.y - b.y);
}

/// Vector negation
export function b2Neg(a: b2Vec2): b2Vec2 {
	return b2Vec2(-a.x, -a.y);
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
export function b2Lerp(a: b2Vec2, b: b2Vec2, t: f32): b2Vec2 {
	return b2Vec2((1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y);
}

/// Component-wise multiplication
export function b2Mul(a: b2Vec2, b: b2Vec2): b2Vec2 {
	return b2Vec2(a.x * b.x, a.y * b.y);
}

/// Multiply a scalar and vector
export function b2MulSV(s: f32, v: b2Vec2): b2Vec2 {
	return b2Vec2(s * v.x, s * v.y);
}

/// a + s * b
export function b2MulAdd(a: b2Vec2, s: f32, b: b2Vec2): b2Vec2 {
	return b2Vec2(a.x + s * b.x, a.y + s * b.y);
}

/// a - s * b
export function b2MulSub(a: b2Vec2, s: f32, b: b2Vec2): b2Vec2 {
	return b2Vec2(a.x - s * b.x, a.y - s * b.y);
}

/// Component-wise absolute vector
export function b2Abs(a: b2Vec2): b2Vec2 {
	let b: b2Vec2;
	b.x = b2AbsFloat(a.x);
	b.y = b2AbsFloat(a.y);
	return b;
}

/// Component-wise minimum vector
export function b2Min(a: b2Vec2, b: b2Vec2): b2Vec2 {
	let c: b2Vec2;
	c.x = b2MinFloat(a.x, b.x);
	c.y = b2MinFloat(a.y, b.y);
	return c;
}

/// Component-wise maximum vector
export function b2Max(a: b2Vec2, b: b2Vec2): b2Vec2 {
	let c: b2Vec2;
	c.x = b2MaxFloat(a.x, b.x);
	c.y = b2MaxFloat(a.y, b.y);
	return c;
}

/// Component-wise clamp vector v into the range [a, b]
export function b2Clamp(v: b2Vec2, a: b2Vec2, b: b2Vec2): b2Vec2 {
	let c: b2Vec2;
	c.x = b2ClampFloat(v.x, a.x, b.x);
	c.y = b2ClampFloat(v.y, a.y, b.y);
	return c;
}

/// Get the length of this vector (the norm)
export function b2Length(v: b2Vec2): f32 {
	return Mathf.sqrt(v.x * v.x + v.y * v.y);
}

/// Get the distance between two points
export function b2Distance(a: b2Vec2, b: b2Vec2): f32 {
	let dx = b.x - a.x;
	let dy = b.y - a.y;
	return Mathf.sqrt(dx * dx + dy * dy);
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
@inline
export function b2Normalize(a: b2Vec2): b2Vec2 {
	let lengthSquared = a.x * a.x + a.y * a.y;
	if (lengthSquared > 1000.0 * FLT_MIN) {
		let s: f32 = 1.0 / Mathf.sqrt(lengthSquared);
		let u = b2Vec2(s * a.x, s * a.y);
		return u;
	}
	return b2Vec2(0.0, 0.0);
}

/// Determines if the provided vector is normalized (norm(a) == 1).
export function b2IsNormalized(a: b2Vec2): bool {
	let aa = b2Dot(a, a);
	return b2AbsFloat(1.0 - aa) < 100.0 * FLT_EPSILON;
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also outputs
/// the length.
export function b2GetLengthAndNormalize(length: ref f32, a: b2Vec2): b2Vec2 {
	let lengthSquared = a.x * a.x + a.y * a.y;
	if (lengthSquared > 1000.0 * FLT_MIN) {
		length = Mathf.sqrt(lengthSquared);
		let s: f32 = 1.0 / length;
		let u = b2Vec2(s * a.x, s * a.y);
		return u;
	}

	length = 0.0;
	return b2Vec2(0.0, 0.0);
}

/// Normalize rotation
export function b2NormalizeRot(q: b2Rot): b2Rot {
	let mag = Mathf.sqrt(q.s * q.s + q.c * q.c);
	let invMag: f32 = mag > 0.0 ? 1.0 / mag : 0.0;
	let qn = b2Rot(q.c * invMag, q.s * invMag);
	return qn;
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaAngle the angular displacement in radians
export function b2IntegrateRotation(q1: b2Rot, deltaAngle: f32): b2Rot {
	// dc/dt = -omega * sin(t)
	// ds/dt = omega * cos(t)
	// c2 = c1 - omega * h * s1
	// s2 = s1 + omega * h * c1
	let q2 = b2Rot(q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c);
	let mag = Mathf.sqrt(q2.s * q2.s + q2.c * q2.c);
	let invMag: f32 = mag > 0.0 ? 1.0 / mag : 0.0;
	let qn = b2Rot(q2.c * invMag, q2.s * invMag);
	return qn;
}

/// Get the length squared of this vector
export function b2LengthSquared(v: b2Vec2): f32 {
	return v.x * v.x + v.y * v.y;
}

/// Get the distance squared between points
export function b2DistanceSquared(a: b2Vec2, b: b2Vec2): f32 {
	let c = b2Vec2(b.x - a.x, b.y - a.y);
	return c.x * c.x + c.y * c.y;
}

/// Make a rotation using an angle in radians
export function b2MakeRot(radians: f32): b2Rot {
	let cs = b2ComputeCosSin(radians);
	return b2Rot(cs.cosine, cs.sine);
}

/// Make a rotation using a unit vector
export function b2MakeRotFromUnitVector(unitVector: b2Vec2): b2Rot {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsNormalized(unitVector));
	return b2Rot(unitVector.x, unitVector.y);
}

/// Is this rotation normalized?
export function b2IsNormalizedRot(q: b2Rot): bool {
	// larger tolerance due to failure on mingw 32-bit
	let qq = q.s * q.s + q.c * q.c;
	return 1.0 - 0.0006 < qq && qq < 1.0 + 0.0006;
}

/// Get the inverse of a rotation
export function b2InvertRot(a: b2Rot): b2Rot {
	return b2Rot(a.c, -a.s);
}

/// Normalized linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
/// https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
export function b2NLerp(q1: b2Rot, q2: b2Rot, t: f32): b2Rot {
	let omt: f32 = 1.0 - t;
	let q = b2Rot(omt * q1.c + t * q2.c, omt * q1.s + t * q2.s);

	let mag = Mathf.sqrt(q.s * q.s + q.c * q.c);
	let invMag: f32 = mag > 0.0 ? 1.0 / mag : 0.0;
	let qn = b2Rot(q.c * invMag, q.s * invMag);
	return qn;
}

/// Compute the angular velocity necessary to rotate between two rotations over a given time
/// @param q1 initial rotation
/// @param q2 final rotation
/// @param inv_h inverse time step
export function b2ComputeAngularVelocity(q1: b2Rot, q2: b2Rot, inv_h: f32): f32 {
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1

	// omega * h * s1 = c1 - c2
	// omega * h * c1 = s2 - s1
	// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
	// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
	// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
	let omega = inv_h * (q2.s * q1.c - q2.c * q1.s);
	return omega;
}

/// Get the angle in radians in the range [-pi, pi]
export function b2Rot_GetAngle(q: b2Rot): f32 {
	return b2Atan2(q.s, q.c);
}

/// Get the x-axis
export function b2Rot_GetXAxis(q: b2Rot): b2Vec2 {
	let v = b2Vec2(q.c, q.s);
	return v;
}

/// Get the y-axis
export function b2Rot_GetYAxis(q: b2Rot): b2Vec2 {
	let v = b2Vec2(-q.s, q.c);
	return v;
}

/// Multiply two rotations: q * r
export function b2MulRot(q: b2Rot, r: b2Rot): b2Rot {
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s(q + r) = qs * rc + qc * rs
	// c(q + r) = qc * rc - qs * rs
	let qr: b2Rot;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: inv(a) * b This rotates a vector local in frame b into a
/// vector local in frame a
export function b2InvMulRot(a: b2Rot, b: b2Rot): b2Rot {
	// [ ac as] * [bc -bs] = [ac*bc+as*bs -ac*bs+as*bc]
	// [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
	// s(b - a) = ac * bs - as * bc
	// c(b - a) = ac * bc + as * bs
	let r: b2Rot;
	r.s = a.c * b.s - a.s * b.c;
	r.c = a.c * b.c + a.s * b.s;
	return r;
}

/// The angle of rotation b relative to rotation a. Radians.
export function b2RelativeAngle(a: b2Rot, b: b2Rot): f32 {
	// sin(b - a) = bs * ac - bc * as
	// cos(b - a) = bc * ac + bs * as
	let s = a.c * b.s - a.s * b.c;
	let c = a.c * b.c + a.s * b.s;
	return b2Atan2(s, c);
}

/// Convert any angle into the range [-pi, pi].
export function b2UnwindAngle(radians: f32): f32 {
	// This function can be implemented using remainderf.
	// However that is a libc dependency not present in WASM.
	// This function doesn't work well with huge angles. So unwind your angles regularly.
	let x = b2ClampFloat(radians, -1.0e6, 1.0e6);

	// Work in doubles. Adding and removing this constant rounds to the nearest integer.
	// https://stackoverflow.com/questions/17035464/a-fast-method-to-round-a-double-to-a-32-bit-int-explained
	let twoPi = <f64>(<f32>2.0 * B2_PI);
	let roundToNearest: f64 = 6755399441055744.0;
	let a = <f64>x;
	let k = (a / twoPi + roundToNearest) - roundToNearest;
	return <f32>(a - k * twoPi);
}

/// Rotate a vector.
export function b2RotateVector(q: b2Rot, v: b2Vec2): b2Vec2 {
	return b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector.
export function b2InvRotateVector(q: b2Rot, v: b2Vec2): b2Vec2 {
	return b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

/// Transform a point (e.g. local space to world space).
export function b2TransformPoint(t: b2Transform, p: b2Vec2): b2Vec2 {
	let x = (t.q.c * p.x - t.q.s * p.y) + t.p.x;
	let y = (t.q.s * p.x + t.q.c * p.y) + t.p.y;

	return b2Vec2(x, y);
}

/// Inverse transform a point (e.g. world space to local space).
export function b2InvTransformPoint(t: b2Transform, p: b2Vec2): b2Vec2 {
	let vx = p.x - t.p.x;
	let vy = p.y - t.p.y;
	return b2Vec2(t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy);
}

/// Multiply two transforms. If the result is applied to a point p local to frame B, the transform
/// would first convert p to a point local to frame A, then into a point in the world frame.
/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
export function b2MulTransforms(A: b2Transform, B: b2Transform): b2Transform {
	let C: b2Transform;
	C.q = b2MulRot(A.q, B.q);
	C.p = b2Add(b2RotateVector(A.q, B.p), A.p);
	return C;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
export function b2InvMulTransforms(A: b2Transform, B: b2Transform): b2Transform {
	let C: b2Transform;
	C.q = b2InvMulRot(A.q, B.q);
	C.p = b2InvRotateVector(A.q, b2Sub(B.p, A.p));
	return C;
}

/// Convert a vector to a world position. No-op in single precision.
export function b2ToPos(v: b2Vec2): b2Pos {
	return b2Vec2(v.x, v.y);
}

/// Lossy conversion of a world position to a float vector. No-op in single precision.
export function b2ToVec2(p: b2Pos): b2Vec2 {
	return b2Vec2(p.x, p.y);
}

/// Narrow a world coordinate to float, rounding toward negative infinity. With large world mode off
/// this is a plain conversion.
export function b2RoundDownFloat(x: f64): f32 {
	return <f32>x;
}

/// Narrow a world coordinate to float, rounding toward positive infinity.
export function b2RoundUpFloat(x: f64): f32 {
	return <f32>x;
}

/// a - b, demoted to float. The primary precision boundary operation.
export function b2SubPos(a: b2Pos, b: b2Pos): b2Vec2 {
	return b2Vec2(a.x - b.x, a.y - b.y);
}

/// p + d
export function b2OffsetPos(p: b2Pos, d: b2Vec2): b2Pos {
	return b2Vec2(p.x + d.x, p.y + d.y);
}

/// World position interpolation for sweeps and sampling.
export function b2LerpPos(a: b2Pos, b: b2Pos, t: f32): b2Pos {
	return b2Vec2((1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y);
}

/// Transform a local point to a world position. Rotation in float, translation in double.
export function b2TransformWorldPoint(t: b2WorldTransform, p: b2Vec2): b2Pos {
	let rx = t.q.c * p.x - t.q.s * p.y;
	let ry = t.q.s * p.x + t.q.c * p.y;
	return b2Vec2(t.p.x + rx, t.p.y + ry);
}

/// Transform a world position to a local point. One double subtraction, then float.
export function b2InvTransformWorldPoint(t: b2WorldTransform, p: b2Pos): b2Vec2 {
	let vx = p.x - t.p.x;
	let vy = p.y - t.p.y;
	return b2Vec2(t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy);
}

/// Relative transform of frame B in frame A.
export function b2InvMulWorldTransforms(A: b2WorldTransform, B: b2WorldTransform): b2Transform {
	let C: b2Transform;
	C.q = b2InvMulRot(A.q, B.q);
	let d = b2Vec2(B.p.x - A.p.x, B.p.y - A.p.y);
	C.p = b2InvRotateVector(A.q, d);
	return C;
}

/// Convert a local transform B into world space using world transform A.
export function b2MulWorldTransforms(A: b2WorldTransform, B: b2Transform): b2WorldTransform {
	let C: b2WorldTransform;
	C.q = b2MulRot(A.q, B.q);
	C.p = b2OffsetPos(A.p, b2RotateVector(A.q, B.p));
	return C;
}

/// Shift a world transform into the frame of a base position.
export function b2ToRelativeTransform(t: b2WorldTransform, base: b2Pos): b2Transform {
	let r: b2Transform;
	r.q = t.q;
	r.p = b2Vec2(t.p.x - base.x, t.p.y - base.y);
	return r;
}

/// Promote a float transform to a world transform. Lossless.
export function b2MakeWorldTransform(t: b2Transform): b2WorldTransform {
	let w: b2WorldTransform;
	w.p = b2ToPos(t.p);
	w.q = t.q;
	return w;
}

/// Multiply a 2-by-2 matrix times a 2D vector
export function b2MulMV(A: b2Mat22, v: b2Vec2): b2Vec2 {
	let u = b2Vec2(A.cx.x * v.x + A.cy.x * v.y, A.cx.y * v.x + A.cy.y * v.y);
	return u;
}

/// Get the inverse of a 2-by-2 matrix
export function b2GetInverse22(A: b2Mat22): b2Mat22 {
	let a = A.cx.x;
	let b = A.cy.x;
	let c = A.cx.y;
	let d = A.cy.y;
	let det = a * d - b * c;
	if (det != 0.0) {
		det = 1.0 / det;
	}

	let B = b2Mat22(b2Vec2(det * d, -det * c), b2Vec2(-det * b, det * a));
	return B;
}

/// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
/// in one-shot cases.
export function b2Solve22(A: b2Mat22, b: b2Vec2): b2Vec2 {
	let a11 = A.cx.x;
	let a12 = A.cy.x;
	let a21 = A.cx.y;
	let a22 = A.cy.y;
	let det = a11 * a22 - a12 * a21;
	if (det != 0.0) {
		det = 1.0 / det;
	}
	let x = b2Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
	return x;
}

/// Does a fully contain b
export function b2AABB_Contains(a: b2AABB, b: b2AABB): bool {
	let s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Get the center of the AABB.
export function b2AABB_Center(a: b2AABB): b2Vec2 {
	let b = b2Vec2(0.5 * (a.lowerBound.x + a.upperBound.x), 0.5 * (a.lowerBound.y + a.upperBound.y));
	return b;
}

/// Get the extents of the AABB (half-widths).
export function b2AABB_Extents(a: b2AABB): b2Vec2 {
	let b = b2Vec2(0.5 * (a.upperBound.x - a.lowerBound.x), 0.5 * (a.upperBound.y - a.lowerBound.y));
	return b;
}

/// Union of two AABBs
export function b2AABB_Union(a: b2AABB, b: b2AABB): b2AABB {
	let c: b2AABB;
	c.lowerBound.x = b2MinFloat(a.lowerBound.x, b.lowerBound.x);
	c.lowerBound.y = b2MinFloat(a.lowerBound.y, b.lowerBound.y);
	c.upperBound.x = b2MaxFloat(a.upperBound.x, b.upperBound.x);
	c.upperBound.y = b2MaxFloat(a.upperBound.y, b.upperBound.y);
	return c;
}

/// Do a and b overlap
export function b2AABB_Overlaps(a: b2AABB, b: b2AABB): bool {
	return !(
		b.lowerBound.x > a.upperBound.x ||
		b.lowerBound.y > a.upperBound.y ||
		a.lowerBound.x > b.upperBound.x ||
		a.lowerBound.y > b.upperBound.y
	);
}

/// Compute the bounding box of an array of circles
// Every C caller passes a proxy's points or a smaller local array, so this takes the proxy array type.
export function b2MakeAABB(points: FixedArray<b2Vec2, 8>, count: i32, radius: f32): b2AABB {
	if (B2_ASSERT_ENABLED) B2_ASSERT(count > 0);
	let a = b2AABB(points[0], points[0]);
	for (let i = 1; i < count; ++i) {
		a.lowerBound = b2Min(a.lowerBound, points[i]);
		a.upperBound = b2Max(a.upperBound, points[i]);
	}

	let r = b2Vec2(radius, radius);
	a.lowerBound = b2Sub(a.lowerBound, r);
	a.upperBound = b2Add(a.upperBound, r);

	return a;
}

/// Signed separation of a point from a plane
export function b2PlaneSeparation(plane: b2Plane, point: b2Vec2): f32 {
	return b2Dot(plane.normal, point) - plane.offset;
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and
/// time step. You can then compute the new position using:
/// position += timeStep * newVelocity
/// This drives towards a zero position. By using implicit integration we get a stable solution that
/// doesn't require transcendental functions.
export function b2SpringDamper(hertz: f32, dampingRatio: f32, position: f32, velocity: f32, timeStep: f32): f32 {
	let omega: f32 = 2.0 * B2_PI * hertz;
	let omegaH = omega * timeStep;
	return (velocity - omega * omegaH * position) / (1.0 + 2.0 * dampingRatio * omegaH + omegaH * omegaH);
}

// src/math_functions.c

@inline
function isnan(a: f32): bool {
	return isNaN(a);
}

@inline
function isinf(a: f32): bool {
	return Mathf.abs(a) == Infinity;
}

/// Is this a valid number? Not NaN or infinity.
export function b2IsValidFloat(a: f32): bool {
	if (isnan(a)) {
		return false;
	}

	if (isinf(a)) {
		return false;
	}

	return true;
}

/// Is this a valid vector? Not NaN or infinity.
export function b2IsValidVec2(v: b2Vec2): bool {
	if (isnan(v.x) || isnan(v.y)) {
		return false;
	}

	if (isinf(v.x) || isinf(v.y)) {
		return false;
	}

	return true;
}

/// Is this a valid rotation? Not NaN or infinity. Is normalized.
export function b2IsValidRot(q: b2Rot): bool {
	if (isnan(q.s) || isnan(q.c)) {
		return false;
	}

	if (isinf(q.s) || isinf(q.c)) {
		return false;
	}

	return b2IsNormalizedRot(q);
}

/// Is this a valid transform? Not NaN or infinity. Rotation is normalized.
export function b2IsValidTransform(t: b2Transform): bool {
	if (b2IsValidVec2(t.p) == false) {
		return false;
	}

	return b2IsValidRot(t.q);
}

/// Is this a valid plane? Normal is a unit vector. Not NaN or infinity.
export function b2IsValidPlane(a: b2Plane): bool {
	return b2IsValidVec2(a.normal) && b2IsNormalized(a.normal) && b2IsValidFloat(a.offset);
}

/// Is this a valid world position? Not NaN or infinity.
export function b2IsValidPos(p: b2Pos): bool {
	if (isnan(p.x) || isnan(p.y)) {
		return false;
	}

	if (isinf(p.x) || isinf(p.y)) {
		return false;
	}

	return true;
}

/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
export function b2IsValidWorldTransform(t: b2WorldTransform): bool {
	if (b2IsValidPos(t.p) == false) {
		return false;
	}

	return b2IsValidRot(t.q);
}

/// Compute an approximate arctangent in the range [-pi, pi] This is hand coded for cross-platform
/// determinism. The atan2f function in the standard library is not cross-platform deterministic.
/// Accurate to around 0.0023 degrees
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
export function b2Atan2(y: f32, x: f32): f32 {
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0.0 && y == 0.0) {
		return 0.0;
	}

	let ax = b2AbsFloat(x);
	let ay = b2AbsFloat(y);
	let mx = b2MaxFloat(ay, ax);
	let mn = b2MinFloat(ay, ax);
	let a = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	let s = a * a;
	let c = s * a;
	let q = s * s;
	let r: f32 = 0.024840285 * q + 0.18681418;
	let t: f32 = -0.094097948 * q - 0.33213072;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if (ay > ax) {
		r = 1.57079637 - r;
	}

	if (x < 0) {
		r = 3.14159274 - r;
	}

	if (y < 0) {
		r = -r;
	}

	return r;
}

/// Compute the cosine and sine of an angle in radians. Implemented for cross-platform determinism.
// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
export function b2ComputeCosSin(radians: f32): b2CosSin {
	let x = b2UnwindAngle(radians);
	let pi2: f32 = B2_PI * B2_PI;

	// cosine needs angle in [-pi/2, pi/2]
	let c: f32;
	if (x < -0.5 * B2_PI) {
		let y: f32 = x + B2_PI;
		let y2 = y * y;
		c = -(pi2 - 4.0 * y2) / (pi2 + y2);
	} else if (x > 0.5 * B2_PI) {
		let y: f32 = x - B2_PI;
		let y2 = y * y;
		c = -(pi2 - 4.0 * y2) / (pi2 + y2);
	} else {
		let y2 = x * x;
		c = (pi2 - 4.0 * y2) / (pi2 + y2);
	}

	// sine needs angle in [0, pi]
	let s: f32;
	if (x < 0.0) {
		let y: f32 = x + B2_PI;
		s = (-16.0 * y * (B2_PI - y)) / (5.0 * pi2 - 4.0 * y * (B2_PI - y));
	} else {
		s = (16.0 * x * (B2_PI - x)) / (5.0 * pi2 - 4.0 * x * (B2_PI - x));
	}

	let mag = Mathf.sqrt(s * s + c * c);
	let invMag: f32 = mag > 0.0 ? 1.0 / mag : 0.0;
	let cs = b2CosSin(c * invMag, s * invMag);
	return cs;
}

/// Compute the rotation between two unit vectors
export function b2ComputeRotationBetweenUnitVectors(v1: b2Vec2, v2: b2Vec2): b2Rot {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2AbsFloat(1.0 - b2Length(v1)) < 100.0 * FLT_EPSILON);
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2AbsFloat(1.0 - b2Length(v2)) < 100.0 * FLT_EPSILON);

	let rot: b2Rot;
	rot.c = b2Dot(v1, v2);
	rot.s = b2Cross(v1, v2);
	return b2NormalizeRot(rot);
}
