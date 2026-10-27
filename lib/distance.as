// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/distance.c

import { B2_ASSERT, B2_ASSERT_ENABLED, FLT_EPSILON, FLT_MIN } from "./core";
import { B2_LINEAR_SLOP } from "./constants";
import {
	b2AbsFloat,
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2CrossVS,
	b2Distance,
	b2DistanceSquared,
	b2Dot,
	b2InvMulTransforms,
	b2InvRotateVector,
	b2IsNormalized,
	b2IsNormalizedRot,
	b2Lerp,
	b2MaxFloat,
	b2MinInt,
	b2MulAdd,
	b2MulSub,
	b2MulSV,
	b2Neg,
	b2Normalize,
	b2NormalizeRot,
	b2Rot,
	b2RotateVector,
	b2Sub,
	b2Transform,
	b2TransformPoint,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import {
	B2_MAX_POLYGON_VERTICES,
	b2CastOutput,
	b2DistanceInput,
	b2DistanceOutput,
	b2SegmentDistanceResult,
	b2ShapeCastPairInput,
	b2ShapeProxy,
	b2Simplex,
	b2SimplexCache,
	b2Sweep,
	b2TOIInput,
	b2TOIOutput,
	b2_toiStateFailed,
	b2_toiStateHit,
	b2_toiStateOverlapped,
	b2_toiStateSeparated,
	b2_toiStateUnknown,
} from "./collision";

/// Evaluate the transform sweep at a specific time.
export function b2GetSweepTransform(sweep: b2Sweep, time: f32): b2Transform {
	// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
	let xf: b2Transform;
	xf.p = b2Add(b2MulSV(1.0 - time, sweep.c1), b2MulSV(time, sweep.c2));

	let q = b2Rot((1.0 - time) * sweep.q1.c + time * sweep.q2.c, (1.0 - time) * sweep.q1.s + time * sweep.q2.s);

	xf.q = b2NormalizeRot(q);

	// Shift to origin
	xf.p = b2Sub(xf.p, b2RotateVector(xf.q, sweep.localCenter));
	return xf;
}

/// Follows Ericson 5.1.9 Closest Points of Two Line Segments
export function b2ComputeSegmentDistance(p1: b2Vec2, q1: b2Vec2, p2: b2Vec2, q2: b2Vec2): b2SegmentDistanceResult {
	let result: b2SegmentDistanceResult;

	let d1 = b2Sub(q1, p1);
	let d2 = b2Sub(q2, p2);
	let r = b2Sub(p1, p2);
	let dd1 = b2Dot(d1, d1);
	let dd2 = b2Dot(d2, d2);
	let rd1 = b2Dot(r, d1);
	let rd2 = b2Dot(r, d2);

	const epsSqr = FLT_EPSILON * FLT_EPSILON;

	if (dd1 < epsSqr || dd2 < epsSqr) {
		// Handle all degeneracies
		if (dd1 >= epsSqr) {
			// Segment 2 is degenerate
			result.fraction1 = b2ClampFloat(-rd1 / dd1, 0.0, 1.0);
			result.fraction2 = 0.0;
		} else if (dd2 >= epsSqr) {
			// Segment 1 is degenerate
			result.fraction1 = 0.0;
			result.fraction2 = b2ClampFloat(rd2 / dd2, 0.0, 1.0);
		} else {
			result.fraction1 = 0.0;
			result.fraction2 = 0.0;
		}
	} else {
		// Non-degenerate segments
		let d12 = b2Dot(d1, d2);

		let denominator = dd1 * dd2 - d12 * d12;

		// Fraction on segment 1
		let f1: f32 = 0.0;
		if (denominator != 0.0) {
			// not parallel
			f1 = b2ClampFloat((d12 * rd2 - rd1 * dd2) / denominator, 0.0, 1.0);
		}

		// Compute point on segment 2 closest to p1 + f1 * d1
		let f2 = (d12 * f1 + rd2) / dd2;

		// Clamping of segment 2 requires a do over on segment 1
		if (f2 < 0.0) {
			f2 = 0.0;
			f1 = b2ClampFloat(-rd1 / dd1, 0.0, 1.0);
		} else if (f2 > 1.0) {
			f2 = 1.0;
			f1 = b2ClampFloat((d12 - rd1) / dd1, 0.0, 1.0);
		}

		result.fraction1 = f1;
		result.fraction2 = f2;
	}

	result.closest1 = b2MulAdd(p1, result.fraction1, d1);
	result.closest2 = b2MulAdd(p2, result.fraction2, d2);
	result.distanceSquared = b2DistanceSquared(result.closest1, result.closest2);
	return result;
}

/// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the
/// points.
// C takes a point pointer; every caller passes polygon or hull points, or uses the point and
// segment forms below.
export function b2MakeProxy(points: FixedArray<b2Vec2, 8>, count: i32, radius: f32): b2ShapeProxy {
	count = b2MinInt(count, B2_MAX_POLYGON_VERTICES);
	let proxy: b2ShapeProxy;
	for (let i = 0; i < count; ++i) {
		proxy.points[i] = points[i];
	}
	proxy.count = count;
	proxy.radius = radius;
	return proxy;
}

/// b2MakeProxy for one point, where C passes the address of a circle center.
export function b2MakePointProxy(point: b2Vec2, radius: f32): b2ShapeProxy {
	let proxy: b2ShapeProxy;
	proxy.points[0] = point;
	proxy.count = 1;
	proxy.radius = radius;
	return proxy;
}

/// b2MakeProxy for two points, where C passes the address of a capsule or segment's first point.
export function b2MakeSegmentProxy(point1: b2Vec2, point2: b2Vec2, radius: f32): b2ShapeProxy {
	let proxy: b2ShapeProxy;
	proxy.points[0] = point1;
	proxy.points[1] = point2;
	proxy.count = 2;
	proxy.radius = radius;
	return proxy;
}

/// Make a proxy with a transform. This is a deep copy of the points.
export function b2MakeOffsetProxy(points: FixedArray<b2Vec2, 8>, count: i32, radius: f32, position: b2Vec2, rotation: b2Rot): b2ShapeProxy {
	count = b2MinInt(count, B2_MAX_POLYGON_VERTICES);
	let transform = b2Transform(position, rotation);
	let proxy: b2ShapeProxy;
	for (let i = 0; i < count; ++i) {
		proxy.points[i] = b2TransformPoint(transform, points[i]);
	}
	proxy.count = count;
	proxy.radius = radius;
	return proxy;
}

@inline
function b2Weight2(a1: f32, w1: b2Vec2, a2: f32, w2: b2Vec2): b2Vec2 {
	return b2Vec2(a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y);
}

@inline
function b2Weight3(a1: f32, w1: b2Vec2, a2: f32, w2: b2Vec2, a3: f32, w3: b2Vec2): b2Vec2 {
	return b2Vec2(a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y);
}

@inline
function b2FindSupport(proxy: ref b2ShapeProxy, direction: b2Vec2): i32 {
	let count = proxy.count;

	let bestIndex = 0;
	let bestValue = b2Dot(proxy.points[0], direction);
	for (let i = 1; i < count; ++i) {
		let value = b2Dot(proxy.points[i], direction);
		if (value > bestValue) {
			bestIndex = i;
			bestValue = value;
		}
	}

	return bestIndex;
}

// C walks the vertices through an array of pointers; the three are written out here.
function b2MakeSimplexFromCache(cache: ref b2SimplexCache, proxyA: ref b2ShapeProxy, proxyB: ref b2ShapeProxy): b2Simplex {
	if (B2_ASSERT_ENABLED) B2_ASSERT(cache.count <= 3);
	let s: b2Simplex;

	// Copy data from cache.
	s.count = <i32>cache.count;

	if (s.count > 0) {
		s.v1.indexA = <i32>cache.indexA[0];
		s.v1.indexB = <i32>cache.indexB[0];
		s.v1.wA = proxyA.points[s.v1.indexA];
		s.v1.wB = proxyB.points[s.v1.indexB];
		s.v1.w = b2Sub(s.v1.wA, s.v1.wB);

		// invalid
		s.v1.a = -1.0;
	}

	if (s.count > 1) {
		s.v2.indexA = <i32>cache.indexA[1];
		s.v2.indexB = <i32>cache.indexB[1];
		s.v2.wA = proxyA.points[s.v2.indexA];
		s.v2.wB = proxyB.points[s.v2.indexB];
		s.v2.w = b2Sub(s.v2.wA, s.v2.wB);
		s.v2.a = -1.0;
	}

	if (s.count > 2) {
		s.v3.indexA = <i32>cache.indexA[2];
		s.v3.indexB = <i32>cache.indexB[2];
		s.v3.wA = proxyA.points[s.v3.indexA];
		s.v3.wB = proxyB.points[s.v3.indexB];
		s.v3.w = b2Sub(s.v3.wA, s.v3.wB);
		s.v3.a = -1.0;
	}

	// If the cache is empty or invalid ...
	if (s.count == 0) {
		s.v1.indexA = 0;
		s.v1.indexB = 0;
		s.v1.wA = proxyA.points[0];
		s.v1.wB = proxyB.points[0];
		s.v1.w = b2Sub(s.v1.wA, s.v1.wB);
		s.v1.a = 1.0;
		s.count = 1;
	}

	return s;
}

@inline
function b2MakeSimplexCache(simplex: ref b2Simplex): b2SimplexCache {
	let cache: b2SimplexCache;
	let count = simplex.count;
	cache.count = <u16>count;
	if (count > 0) {
		cache.indexA[0] = <u8>simplex.v1.indexA;
		cache.indexB[0] = <u8>simplex.v1.indexB;
	}
	if (count > 1) {
		cache.indexA[1] = <u8>simplex.v2.indexA;
		cache.indexB[1] = <u8>simplex.v2.indexB;
	}
	if (count > 2) {
		cache.indexA[2] = <u8>simplex.v3.indexA;
		cache.indexB[2] = <u8>simplex.v3.indexB;
	}

	return cache;
}

// C writes the two witness points through pointers; this writes output.pointA and output.pointB.
@inline
function b2ComputeWitnessPoints(s: ref b2Simplex, output: ref b2DistanceOutput): void {
	switch (s.count) {
		case 1:
			output.pointA = s.v1.wA;
			output.pointB = s.v1.wB;
			break;

		case 2:
			output.pointA = b2Weight2(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA);
			output.pointB = b2Weight2(s.v1.a, s.v1.wB, s.v2.a, s.v2.wB);
			break;

		case 3:
			output.pointA = b2Weight3(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA, s.v3.a, s.v3.wA);
			// todo why are these not equal?
			//*b = b2Weight3(s->v1.a, s->v1.wB, s->v2.a, s->v2.wB, s->v3.a, s->v3.wB);
			output.pointB = output.pointA;
			break;

		default:
			output.pointA = b2Vec2_zero;
			output.pointB = b2Vec2_zero;
			if (B2_ASSERT_ENABLED) B2_ASSERT(false);
			break;
	}
}

// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
//
// returns a vector that points towards the origin
@inline
function b2SolveSimplex2(s: ref b2Simplex): b2Vec2 {
	let w1 = s.v1.w;
	let w2 = s.v2.w;
	let e12 = b2Sub(w2, w1);

	// w1 region
	let d12_2 = -b2Dot(w1, e12);
	if (d12_2 <= 0.0) {
		// a2 <= 0, so we clamp it to 0
		s.v1.a = 1.0;
		s.count = 1;
		return b2Neg(w1);
	}

	// w2 region
	let d12_1 = b2Dot(w2, e12);
	if (d12_1 <= 0.0) {
		// a1 <= 0, so we clamp it to 0
		s.v2.a = 1.0;
		s.count = 1;
		s.v1 = s.v2;
		return b2Neg(w2);
	}

	// Must be in e12 region.
	let inv_d12: f32 = 1.0 / (d12_1 + d12_2);
	s.v1.a = d12_1 * inv_d12;
	s.v2.a = d12_2 * inv_d12;
	s.count = 2;
	return b2CrossSV(b2Cross(b2Add(w1, w2), e12), e12);
}

@inline
function b2SolveSimplex3(s: ref b2Simplex): b2Vec2 {
	let w1 = s.v1.w;
	let w2 = s.v2.w;
	let w3 = s.v3.w;

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	let e12 = b2Sub(w2, w1);
	let w1e12 = b2Dot(w1, e12);
	let w2e12 = b2Dot(w2, e12);
	let d12_1 = w2e12;
	let d12_2 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	let e13 = b2Sub(w3, w1);
	let w1e13 = b2Dot(w1, e13);
	let w3e13 = b2Dot(w3, e13);
	let d13_1 = w3e13;
	let d13_2 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	let e23 = b2Sub(w3, w2);
	let w2e23 = b2Dot(w2, e23);
	let w3e23 = b2Dot(w3, e23);
	let d23_1 = w3e23;
	let d23_2 = -w2e23;

	// Triangle123
	let n123 = b2Cross(e12, e13);

	let d123_1 = n123 * b2Cross(w2, w3);
	let d123_2 = n123 * b2Cross(w3, w1);
	let d123_3 = n123 * b2Cross(w1, w2);

	// w1 region
	if (d12_2 <= 0.0 && d13_2 <= 0.0) {
		s.v1.a = 1.0;
		s.count = 1;
		return b2Neg(w1);
	}

	// e12
	if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
		let inv_d12: f32 = 1.0 / (d12_1 + d12_2);
		s.v1.a = d12_1 * inv_d12;
		s.v2.a = d12_2 * inv_d12;
		s.count = 2;
		return b2CrossSV(b2Cross(b2Add(w1, w2), e12), e12);
	}

	// e13
	if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
		let inv_d13: f32 = 1.0 / (d13_1 + d13_2);
		s.v1.a = d13_1 * inv_d13;
		s.v3.a = d13_2 * inv_d13;
		s.count = 2;
		s.v2 = s.v3;
		return b2CrossSV(b2Cross(b2Add(w1, w3), e13), e13);
	}

	// w2 region
	if (d12_1 <= 0.0 && d23_2 <= 0.0) {
		s.v2.a = 1.0;
		s.count = 1;
		s.v1 = s.v2;
		return b2Neg(w2);
	}

	// w3 region
	if (d13_1 <= 0.0 && d23_1 <= 0.0) {
		s.v3.a = 1.0;
		s.count = 1;
		s.v1 = s.v3;
		return b2Neg(w3);
	}

	// e23
	if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
		let inv_d23: f32 = 1.0 / (d23_1 + d23_2);
		s.v2.a = d23_1 * inv_d23;
		s.v3.a = d23_2 * inv_d23;
		s.count = 2;
		s.v1 = s.v3;
		return b2CrossSV(b2Cross(b2Add(w2, w3), e23), e23);
	}

	// Must be in triangle123
	let inv_d123: f32 = 1.0 / (d123_1 + d123_2 + d123_3);
	s.v1.a = d123_1 * inv_d123;
	s.v2.a = d123_2 * inv_d123;
	s.v3.a = d123_3 * inv_d123;
	s.count = 3;

	// No search direction
	return b2Vec2_zero;
}

/// Compute the closest points between two shapes represented as point clouds.
/// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
/// The query runs in frame A: proxyB is placed by input.transform (the pose of B in A), and the
/// output points and normal are in frame A.
/// Pass null simplexes for no debug output. As in C's release build only the initial simplex is
/// recorded.
// Uses GJK for computing the distance between convex shapes.
// https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf
// I spent time optimizing this and could find no further significant gains 3/30/2025
export function b2ComputeShapeDistance(
	input: b2DistanceInput,
	cache: ref b2SimplexCache,
	simplexes: StaticArray<b2Simplex>?,
	simplexCapacity: i32,
): b2DistanceOutput {
	if (B2_ASSERT_ENABLED) B2_ASSERT(input.proxyA.count > 0 && input.proxyB.count > 0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(input.proxyA.radius >= 0.0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(input.proxyB.radius >= 0.0);

	let output: b2DistanceOutput;

	// Get proxyB in frame A to avoid further transforms in the main loop.
	// This is still a performance gain at 8 points.
	let localProxyB: b2ShapeProxy;
	{
		localProxyB.count = input.proxyB.count;
		localProxyB.radius = input.proxyB.radius;
		for (let i = 0; i < localProxyB.count; ++i) {
			localProxyB.points[i] = b2TransformPoint(input.transform, input.proxyB.points[i]);
		}
	}

	// Initialize the simplex.
	let simplex = b2MakeSimplexFromCache(cache, input.proxyA, localProxyB);

	let simplexIndex = 0;
	if (simplexes != null && simplexIndex < simplexCapacity) {
		simplexes[simplexIndex] = simplex;
		simplexIndex += 1;
	}

	let nonUnitNormal = b2Vec2_zero;

	// Main iteration loop. All computations are done in frame A.
	const maxIterations = 20;
	let iteration = 0;
	while (iteration < maxIterations) {
		// Copy simplex so we can identify duplicates. C keeps these in two arrays of three.
		let saveCount = simplex.count;
		let saveA0 = simplex.v1.indexA;
		let saveB0 = simplex.v1.indexB;
		let saveA1 = simplex.v2.indexA;
		let saveB1 = simplex.v2.indexB;
		let saveA2 = simplex.v3.indexA;
		let saveB2 = simplex.v3.indexB;

		let d = b2Vec2_zero;
		switch (simplex.count) {
			case 1:
				d = b2Neg(simplex.v1.w);
				break;

			case 2:
				d = b2SolveSimplex2(simplex);
				break;

			case 3:
				d = b2SolveSimplex3(simplex);
				break;

			default:
				if (B2_ASSERT_ENABLED) B2_ASSERT(false);
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.count == 3) {
			// Overlap
			cache = b2MakeSimplexCache(simplex);
			b2ComputeWitnessPoints(simplex, output);
			output.distance = 0.0;
			return output;
		}

		// Ensure the search direction is numerically fit.
		if (b2Dot(d, d) < 1000.0 * FLT_MIN) {
			// This is unlikely but could lead to bad cycling.
			// The branch predictor seems to make this check have low cost.

			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			cache = b2MakeSimplexCache(simplex);

			// Must return overlap due to invalid normal.
			b2ComputeWitnessPoints(simplex, output);
			output.distance = 0.0;
			return output;
		}

		// Save the normal
		nonUnitNormal = d;

		// Compute a tentative new simplex vertex using support points.
		// support = support(a, d) - support(b, -d)
		let indexA = b2FindSupport(input.proxyA, d);
		let wA = input.proxyA.points[indexA];
		let indexB = b2FindSupport(localProxyB, b2Neg(d));
		let wB = localProxyB.points[indexB];
		let w = b2Sub(wA, wB);
		if (simplex.count == 1) {
			simplex.v2.indexA = indexA;
			simplex.v2.wA = wA;
			simplex.v2.indexB = indexB;
			simplex.v2.wB = wB;
			simplex.v2.w = w;
		} else {
			simplex.v3.indexA = indexA;
			simplex.v3.wA = wA;
			simplex.v3.indexB = indexB;
			simplex.v3.wB = wB;
			simplex.v3.w = w;
		}

		// Iteration count is equated to the number of support point calls.
		++iteration;

		// Check for duplicate support points. This is the main termination criteria.
		let duplicate =
			(indexA == saveA0 && indexB == saveB0) ||
			(saveCount > 1 && indexA == saveA1 && indexB == saveB1) ||
			(saveCount > 2 && indexA == saveA2 && indexB == saveB2);

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate) {
			break;
		}

		// New vertex is valid and needed.
		simplex.count += 1;
	}

	// Prepare output in frame A
	let normal = b2Normalize(nonUnitNormal);
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsNormalized(normal));

	b2ComputeWitnessPoints(simplex, output);
	output.normal = normal;
	output.distance = b2Distance(output.pointA, output.pointB);
	output.iterations = iteration;
	output.simplexCount = simplexIndex;

	// Cache the simplex
	cache = b2MakeSimplexCache(simplex);

	// Apply radii if requested
	if (input.useRadii) {
		let radiusA = input.proxyA.radius;
		let radiusB = input.proxyB.radius;
		output.distance = b2MaxFloat(0.0, output.distance - radiusA - radiusB);

		// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
		output.pointA = b2MulAdd(output.pointA, radiusA, normal);
		output.pointB = b2MulSub(output.pointB, radiusB, normal);
	}

	return output;
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point,
/// normal, and translation fraction. Initially touching shapes are treated as a miss. You may
/// optionally allow encroachment. The cast runs in frame A, so the result is in frame A.
// Shape cast using conservative advancement
export function b2CastShape(input: b2ShapeCastPairInput): b2CastOutput {
	// Compute tolerance
	let linearSlop = B2_LINEAR_SLOP();
	let totalRadius = input.proxyA.radius + input.proxyB.radius;
	let target = b2MaxFloat(linearSlop, totalRadius - linearSlop);
	let tolerance: f32 = 0.25 * linearSlop;

	if (B2_ASSERT_ENABLED) B2_ASSERT(target > tolerance);

	// Prepare input for distance query
	let cache: b2SimplexCache;

	let fraction: f32 = 0.0;

	let distanceInput: b2DistanceInput;
	distanceInput.proxyA = input.proxyA;
	distanceInput.proxyB = input.proxyB;
	distanceInput.useRadii = false;

	// The whole cast runs in frame A. Advance the relative pose of B in float each iteration,
	// which keeps the math near the local origin and avoids re-relativizing world poses.
	distanceInput.transform = input.transform;

	let delta2 = input.translationB;
	let output: b2CastOutput;

	let iteration = 0;
	const maxIterations = 20;

	for (; iteration < maxIterations; ++iteration) {
		output.iterations += 1;

		let distanceOutput = b2ComputeShapeDistance(distanceInput, cache, null, 0);

		if (distanceOutput.distance < target + tolerance) {
			if (iteration == 0) {
				if (input.canEncroach && distanceOutput.distance > 2.0 * linearSlop) {
					target = distanceOutput.distance - linearSlop;
				} else {
					// Initial overlap
					output.hit = true;

					// Compute a common point
					let c1 = b2MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
					let c2 = b2MulAdd(distanceOutput.pointB, -input.proxyB.radius, distanceOutput.normal);
					output.point = b2Lerp(c1, c2, 0.5);
					return output;
				}
			} else {
				// Regular hit
				if (B2_ASSERT_ENABLED) B2_ASSERT(distanceOutput.distance > 0.0 && b2IsNormalized(distanceOutput.normal));
				output.fraction = fraction;
				output.point = b2MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
				output.normal = distanceOutput.normal;
				output.hit = true;
				return output;
			}
		}

		if (B2_ASSERT_ENABLED) B2_ASSERT(distanceOutput.distance > 0.0);
		if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsNormalized(distanceOutput.normal));

		// Check if shapes are approaching each other
		let denominator = b2Dot(delta2, distanceOutput.normal);
		if (denominator >= 0.0) {
			// Miss
			return output;
		}

		// Advance sweep
		fraction += (target - distanceOutput.distance) / denominator;
		if (fraction >= input.maxFraction) {
			// Miss
			return output;
		}

		distanceInput.transform.p = b2MulAdd(input.transform.p, fraction, delta2);
	}

	// Failure!
	return output;
}

// b2ShapeCastMerged is compiled out in C (#if 0) and is not ported.

type b2SeparationType = i32;
const b2_pointsType: b2SeparationType = 0;
const b2_faceAType: b2SeparationType = 1;
const b2_faceBType: b2SeparationType = 2;

// C keeps pointers to the two proxies here; they are passed by ref beside the function instead.
struct b2SeparationFunction {
	sweepA: b2Sweep = b2Sweep();
	sweepB: b2Sweep = b2Sweep();
	localPoint: b2Vec2 = b2Vec2();
	axis: b2Vec2 = b2Vec2();
	type: b2SeparationType = 0;
}

function b2MakeSeparationFunction(
	cache: ref b2SimplexCache,
	proxyA: ref b2ShapeProxy,
	sweepA: b2Sweep,
	proxyB: ref b2ShapeProxy,
	sweepB: b2Sweep,
	t1: f32,
): b2SeparationFunction {
	let f: b2SeparationFunction;

	let count = <i32>cache.count;
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 < count && count < 3);

	f.sweepA = sweepA;
	f.sweepB = sweepB;

	let xfA = b2GetSweepTransform(sweepA, t1);
	let xfB = b2GetSweepTransform(sweepB, t1);

	if (count == 1) {
		f.type = b2_pointsType;
		let localPointA = proxyA.points[<i32>cache.indexA[0]];
		let localPointB = proxyB.points[<i32>cache.indexB[0]];
		let pointA = b2TransformPoint(xfA, localPointA);
		let pointB = b2TransformPoint(xfB, localPointB);
		f.axis = b2Normalize(b2Sub(pointB, pointA));
		f.localPoint = b2Vec2_zero;
		return f;
	}

	if (cache.indexA[0] == cache.indexA[1]) {
		// Two points on B and one on A.
		f.type = b2_faceBType;
		let localPointB1 = proxyB.points[<i32>cache.indexB[0]];
		let localPointB2 = proxyB.points[<i32>cache.indexB[1]];

		f.axis = b2CrossVS(b2Sub(localPointB2, localPointB1), 1.0);
		f.axis = b2Normalize(f.axis);
		let normal = b2RotateVector(xfB.q, f.axis);

		f.localPoint = b2Vec2(0.5 * (localPointB1.x + localPointB2.x), 0.5 * (localPointB1.y + localPointB2.y));
		let pointB = b2TransformPoint(xfB, f.localPoint);

		let localPointA = proxyA.points[<i32>cache.indexA[0]];
		let pointA = b2TransformPoint(xfA, localPointA);

		let s = b2Dot(b2Sub(pointA, pointB), normal);
		if (s < 0.0) {
			f.axis = b2Neg(f.axis);
		}
		return f;
	}

	// Two points on A and one or two points on B.
	f.type = b2_faceAType;
	let localPointA1 = proxyA.points[<i32>cache.indexA[0]];
	let localPointA2 = proxyA.points[<i32>cache.indexA[1]];

	f.axis = b2CrossVS(b2Sub(localPointA2, localPointA1), 1.0);
	f.axis = b2Normalize(f.axis);
	let normal = b2RotateVector(xfA.q, f.axis);

	f.localPoint = b2Vec2(0.5 * (localPointA1.x + localPointA2.x), 0.5 * (localPointA1.y + localPointA2.y));
	let pointA = b2TransformPoint(xfA, f.localPoint);

	let localPointB = proxyB.points[<i32>cache.indexB[0]];
	let pointB = b2TransformPoint(xfB, localPointB);

	let s = b2Dot(b2Sub(pointB, pointA), normal);
	if (s < 0.0) {
		f.axis = b2Neg(f.axis);
	}
	return f;
}

// C returns the separation and writes the two indices through pointers.
struct b2MinSeparation {
	separation: f32 = 0;
	indexA: i32 = 0;
	indexB: i32 = 0;
}

function b2FindMinSeparation(f: ref b2SeparationFunction, proxyA: ref b2ShapeProxy, proxyB: ref b2ShapeProxy, t: f32): b2MinSeparation {
	let xfA = b2GetSweepTransform(f.sweepA, t);
	let xfB = b2GetSweepTransform(f.sweepB, t);
	let result: b2MinSeparation;

	switch (f.type) {
		case b2_pointsType: {
			let axisA = b2InvRotateVector(xfA.q, f.axis);
			let axisB = b2InvRotateVector(xfB.q, b2Neg(f.axis));

			result.indexA = b2FindSupport(proxyA, axisA);
			result.indexB = b2FindSupport(proxyB, axisB);

			let localPointA = proxyA.points[result.indexA];
			let localPointB = proxyB.points[result.indexB];

			let pointA = b2TransformPoint(xfA, localPointA);
			let pointB = b2TransformPoint(xfB, localPointB);

			result.separation = b2Dot(b2Sub(pointB, pointA), f.axis);
			return result;
		}

		case b2_faceAType: {
			let normal = b2RotateVector(xfA.q, f.axis);
			let pointA = b2TransformPoint(xfA, f.localPoint);

			let axisB = b2InvRotateVector(xfB.q, b2Neg(normal));

			result.indexA = -1;
			result.indexB = b2FindSupport(proxyB, axisB);

			let localPointB = proxyB.points[result.indexB];
			let pointB = b2TransformPoint(xfB, localPointB);

			result.separation = b2Dot(b2Sub(pointB, pointA), normal);
			return result;
		}

		case b2_faceBType: {
			let normal = b2RotateVector(xfB.q, f.axis);
			let pointB = b2TransformPoint(xfB, f.localPoint);

			let axisA = b2InvRotateVector(xfA.q, b2Neg(normal));

			result.indexB = -1;
			result.indexA = b2FindSupport(proxyA, axisA);

			let localPointA = proxyA.points[result.indexA];
			let pointA = b2TransformPoint(xfA, localPointA);

			result.separation = b2Dot(b2Sub(pointA, pointB), normal);
			return result;
		}

		default:
			if (B2_ASSERT_ENABLED) B2_ASSERT(false);
			result.indexA = -1;
			result.indexB = -1;
			result.separation = 0.0;
			return result;
	}
}

//
function b2EvaluateSeparation(f: ref b2SeparationFunction, proxyA: ref b2ShapeProxy, proxyB: ref b2ShapeProxy, indexA: i32, indexB: i32, t: f32): f32 {
	let xfA = b2GetSweepTransform(f.sweepA, t);
	let xfB = b2GetSweepTransform(f.sweepB, t);

	switch (f.type) {
		case b2_pointsType: {
			let localPointA = proxyA.points[indexA];
			let localPointB = proxyB.points[indexB];

			let pointA = b2TransformPoint(xfA, localPointA);
			let pointB = b2TransformPoint(xfB, localPointB);

			let separation = b2Dot(b2Sub(pointB, pointA), f.axis);
			return separation;
		}

		case b2_faceAType: {
			let normal = b2RotateVector(xfA.q, f.axis);
			let pointA = b2TransformPoint(xfA, f.localPoint);

			let localPointB = proxyB.points[indexB];
			let pointB = b2TransformPoint(xfB, localPointB);

			let separation = b2Dot(b2Sub(pointB, pointA), normal);
			return separation;
		}

		case b2_faceBType: {
			let normal = b2RotateVector(xfB.q, f.axis);
			let pointB = b2TransformPoint(xfB, f.localPoint);

			let localPointA = proxyA.points[indexA];
			let pointA = b2TransformPoint(xfA, localPointA);

			let separation = b2Dot(b2Sub(pointA, pointB), normal);
			return separation;
		}

		default:
			if (B2_ASSERT_ENABLED) B2_ASSERT(false);
			return 0.0;
	}
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction
/// between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function again.
// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
export function b2ComputeTimeOfImpact(input: b2TOIInput): b2TOIOutput {
	let output: b2TOIOutput;
	output.state = b2_toiStateUnknown;
	output.fraction = input.maxFraction;

	let sweepA = input.sweepA;
	let sweepB = input.sweepB;
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsNormalizedRot(sweepA.q1) && b2IsNormalizedRot(sweepA.q2));
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsNormalizedRot(sweepB.q1) && b2IsNormalizedRot(sweepB.q2));

	// todo_erin
	// c1 can be at the origin yet the points are far away
	// b2Vec2 origin = b2Add(sweepA.c1, input->proxyA.points[0]);

	let tMax = input.maxFraction;

	// Setup target distance and tolerance
	let totalRadius = input.proxyA.radius + input.proxyB.radius;
	let target = b2MaxFloat(B2_LINEAR_SLOP(), totalRadius - B2_LINEAR_SLOP());
	let tolerance: f32 = 0.25 * B2_LINEAR_SLOP();
	if (B2_ASSERT_ENABLED) B2_ASSERT(target > tolerance);

	let t1: f32 = 0.0;
	const k_maxIterations = 20;
	let distanceIterations = 0;

	// Prepare input for distance query.
	let cache: b2SimplexCache;
	let distanceInput: b2DistanceInput;
	distanceInput.proxyA = input.proxyA;
	distanceInput.proxyB = input.proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for (;;) {
		// Get the distance between shapes. We can also use the results to get a separating axis.
		let xfA = b2GetSweepTransform(sweepA, t1);
		let xfB = b2GetSweepTransform(sweepB, t1);
		distanceInput.transform = b2InvMulTransforms(xfA, xfB);
		let distanceOutput = b2ComputeShapeDistance(distanceInput, cache, null, 0);

		// The distance query runs in frame A, project the witness data back to world
		let worldNormal = b2RotateVector(xfA.q, distanceOutput.normal);
		let worldPointA = b2TransformPoint(xfA, distanceOutput.pointA);
		let worldPointB = b2TransformPoint(xfA, distanceOutput.pointB);

		distanceIterations += 1;

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0.0) {
			// Failure!
			output.state = b2_toiStateOverlapped;
			output.fraction = 0.0;
			break;
		}

		if (distanceOutput.distance <= target + tolerance) {
			// Success!
			output.state = b2_toiStateHit;
			// Averaged hit point
			let pA = b2MulAdd(worldPointA, input.proxyA.radius, worldNormal);
			let pB = b2MulAdd(worldPointB, -input.proxyB.radius, worldNormal);
			output.point = b2Lerp(pA, pB, 0.5);
			output.normal = worldNormal;
			output.fraction = t1;
			break;
		}

		// Initialize the separating axis.
		let fcn = b2MakeSeparationFunction(cache, input.proxyA, sweepA, input.proxyB, sweepB, t1);

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		let done = false;
		let t2 = tMax;
		let pushBackIterations = 0;
		for (;;) {
			// Find the deepest point at t2. Store the witness point indices.
			let minSeparation = b2FindMinSeparation(fcn, input.proxyA, input.proxyB, t2);
			let indexA = minSeparation.indexA;
			let indexB = minSeparation.indexB;
			let s2 = minSeparation.separation;

			// Is the final configuration separated?
			if (s2 > target + tolerance) {
				// Victory!
				output.state = b2_toiStateSeparated;
				output.fraction = tMax;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance) {
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			let s1 = b2EvaluateSeparation(fcn, input.proxyA, input.proxyB, indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder runs out of iterations.
			if (s1 < target - tolerance) {
				output.state = b2_toiStateFailed;
				output.fraction = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance) {
				// Success! t1 should hold the TOI (could be 0.0).
				output.state = b2_toiStateHit;
				// Averaged hit point
				let pA = b2MulAdd(worldPointA, input.proxyA.radius, worldNormal);
				let pB = b2MulAdd(worldPointB, -input.proxyB.radius, worldNormal);
				output.point = b2Lerp(pA, pB, 0.5);
				output.normal = worldNormal;
				output.fraction = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			let rootIterationCount = 0;
			let a1 = t1;
			let a2 = t2;
			for (;;) {
				// Use a mix of false position and bisection.
				let t: f32;
				if ((rootIterationCount & 1) != 0) {
					// False position to improve convergence.
					t = a1 + ((target - s1) * (a2 - a1)) / (s2 - s1);
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2);
				}

				rootIterationCount += 1;

				let s = b2EvaluateSeparation(fcn, input.proxyA, input.proxyB, indexA, indexB, t);

				// Has the separation reached tolerance?
				if (b2AbsFloat(s - target) < tolerance) {
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target) {
					a1 = t;
					s1 = s;
				} else {
					a2 = t;
					s2 = s;
				}

				if (rootIterationCount == 50) {
					break;
				}
			}

			pushBackIterations += 1;

			if (pushBackIterations == B2_MAX_POLYGON_VERTICES) {
				break;
			}
		}

		if (done) {
			break;
		}

		if (distanceIterations == k_maxIterations) {
			// Root finder got stuck. Semi-victory.
			output.state = b2_toiStateFailed;
			// Averaged hit point
			let pA = b2MulAdd(worldPointA, input.proxyA.radius, worldNormal);
			let pB = b2MulAdd(worldPointB, -input.proxyB.radius, worldNormal);
			output.point = b2Lerp(pA, pB, 0.5);
			output.normal = worldNormal;
			output.fraction = t1;
			break;
		}
	}

	return output;
}
