// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/hull.c

import { B2_ASSERT, B2_ASSERT_ENABLED, FLT_MAX } from "./core";
import { B2_LINEAR_SLOP } from "./constants";
import { b2AABB, b2AABB_Center, b2Cross, b2DistanceSquared, b2Max, b2Min, b2MinInt, b2Normalize, b2Sub, b2Vec2 } from "./math_functions";
import { B2_MAX_POLYGON_VERTICES, b2Hull } from "./collision";

// quickhull recursion
function b2RecurseHull(p1: b2Vec2, p2: b2Vec2, ps: FixedArray<b2Vec2, 8>, count: i32): b2Hull {
	let hull: b2Hull;
	hull.count = 0;

	if (count == 0) {
		return hull;
	}

	// create an edge vector pointing from p1 to p2
	let e = b2Normalize(b2Sub(p2, p1));

	// discard points left of e and find point furthest to the right of e
	let rightPoints: FixedArray<b2Vec2, 8>;
	let rightCount = 0;

	let bestIndex = 0;
	let bestDistance = b2Cross(b2Sub(ps[bestIndex], p1), e);
	if (bestDistance > 0.0) {
		rightPoints[rightCount++] = ps[bestIndex];
	}

	for (let i = 1; i < count; ++i) {
		let distance = b2Cross(b2Sub(ps[i], p1), e);
		if (distance > bestDistance) {
			bestIndex = i;
			bestDistance = distance;
		}

		if (distance > 0.0) {
			rightPoints[rightCount++] = ps[i];
		}
	}

	if (bestDistance < 2.0 * B2_LINEAR_SLOP()) {
		return hull;
	}

	let bestPoint = ps[bestIndex];

	// compute hull to the right of p1-bestPoint
	let hull1 = b2RecurseHull(p1, bestPoint, rightPoints, rightCount);

	// compute hull to the right of bestPoint-p2
	let hull2 = b2RecurseHull(bestPoint, p2, rightPoints, rightCount);

	// stitch together hulls
	for (let i = 0; i < hull1.count; ++i) {
		hull.points[hull.count++] = hull1.points[i];
	}

	hull.points[hull.count++] = bestPoint;

	for (let i = 0; i < hull2.count; ++i) {
		hull.points[hull.count++] = hull2.points[i];
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(hull.count < B2_MAX_POLYGON_VERTICES);

	return hull;
}

// quickhull algorithm
// - merges vertices based on B2_LINEAR_SLOP
// - removes collinear points using B2_LINEAR_SLOP
// - returns an empty hull if it fails
// C takes a point pointer; this takes any StaticArray or Array of b2Vec2.
export function b2ComputeHull<A>(points: A, count: i32): b2Hull {
	let hull: b2Hull;
	hull.count = 0;

	if (count < 3 || count > B2_MAX_POLYGON_VERTICES) {
		// check your data
		return hull;
	}

	count = b2MinInt(count, B2_MAX_POLYGON_VERTICES);

	let aabb = b2AABB(b2Vec2(FLT_MAX, FLT_MAX), b2Vec2(-FLT_MAX, -FLT_MAX));

	// Perform aggressive point welding. First point always remains.
	// Also compute the bounding box for later.
	let ps: FixedArray<b2Vec2, 8>;
	let n = 0;
	const linearSlop = B2_LINEAR_SLOP();
	const tolSqr = 0.25 * linearSlop * linearSlop;
	for (let i = 0; i < count; ++i) {
		let vi: b2Vec2 = points[i];
		aabb.lowerBound = b2Min(aabb.lowerBound, vi);
		aabb.upperBound = b2Max(aabb.upperBound, vi);

		let unique = true;
		for (let j = 0; j < n; ++j) {
			let vj = ps[j];

			let distSqr = b2DistanceSquared(vi, vj);
			if (distSqr < tolSqr) {
				unique = false;
				break;
			}
		}

		if (unique) {
			ps[n++] = vi;
		}
	}

	if (n < 3) {
		// all points very close together, check your data and check your scale
		return hull;
	}

	// Find an extreme point as the first point on the hull
	let c = b2AABB_Center(aabb);
	let f1 = 0;
	let dsq1 = b2DistanceSquared(c, ps[f1]);
	for (let i = 1; i < n; ++i) {
		let dsq = b2DistanceSquared(c, ps[i]);
		if (dsq > dsq1) {
			f1 = i;
			dsq1 = dsq;
		}
	}

	// remove p1 from working set
	let p1 = ps[f1];
	ps[f1] = ps[n - 1];
	n = n - 1;

	let f2 = 0;
	let dsq2 = b2DistanceSquared(p1, ps[f2]);
	for (let i = 1; i < n; ++i) {
		let dsq = b2DistanceSquared(p1, ps[i]);
		if (dsq > dsq2) {
			f2 = i;
			dsq2 = dsq;
		}
	}

	// remove p2 from working set
	let p2 = ps[f2];
	ps[f2] = ps[n - 1];
	n = n - 1;

	// split the points into points that are left and right of the line p1-p2.
	let rightPoints: FixedArray<b2Vec2, 8>;
	let rightCount = 0;

	let leftPoints: FixedArray<b2Vec2, 8>;
	let leftCount = 0;

	let e = b2Normalize(b2Sub(p2, p1));

	for (let i = 0; i < n; ++i) {
		let d = b2Cross(b2Sub(ps[i], p1), e);

		// slop used here to skip points that are very close to the line p1-p2
		if (d >= 2.0 * linearSlop) {
			rightPoints[rightCount++] = ps[i];
		} else if (d <= -2.0 * linearSlop) {
			leftPoints[leftCount++] = ps[i];
		}
	}

	// compute hulls on right and left
	let hull1 = b2RecurseHull(p1, p2, rightPoints, rightCount);
	let hull2 = b2RecurseHull(p2, p1, leftPoints, leftCount);

	if (hull1.count == 0 && hull2.count == 0) {
		// all points collinear
		return hull;
	}

	// stitch hulls together, preserving CCW winding order
	hull.points[hull.count++] = p1;

	for (let i = 0; i < hull1.count; ++i) {
		hull.points[hull.count++] = hull1.points[i];
	}

	hull.points[hull.count++] = p2;

	for (let i = 0; i < hull2.count; ++i) {
		hull.points[hull.count++] = hull2.points[i];
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(hull.count <= B2_MAX_POLYGON_VERTICES);

	// merge collinear
	let searching = true;
	while (searching && hull.count > 2) {
		searching = false;

		for (let i = 0; i < hull.count; ++i) {
			let i1 = i;
			let i2 = (i + 1) % hull.count;
			let i3 = (i + 2) % hull.count;

			let s1 = hull.points[i1];
			let s2 = hull.points[i2];
			let s3 = hull.points[i3];

			// unit edge vector for s1-s3
			let r = b2Normalize(b2Sub(s3, s1));

			let distance = b2Cross(b2Sub(s2, s1), r);
			if (distance <= 2.0 * linearSlop) {
				// remove midpoint from hull
				for (let j = i2; j < hull.count - 1; ++j) {
					hull.points[j] = hull.points[j + 1];
				}
				hull.count -= 1;

				// continue searching for collinear points
				searching = true;

				break;
			}
		}
	}

	if (hull.count < 3) {
		// all points collinear, shouldn't be reached since this was validated above
		hull.count = 0;
	}

	return hull;
}

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
export function b2ValidateHull(hull: b2Hull): bool {
	if (hull.count < 3 || B2_MAX_POLYGON_VERTICES < hull.count) {
		return false;
	}

	// test that every point is behind every edge
	for (let i = 0; i < hull.count; ++i) {
		// create an edge vector
		let i1 = i;
		let i2 = i < hull.count - 1 ? i1 + 1 : 0;
		let p = hull.points[i1];
		let e = b2Normalize(b2Sub(hull.points[i2], p));

		for (let j = 0; j < hull.count; ++j) {
			// skip points that subtend the current edge
			if (j == i1 || j == i2) {
				continue;
			}

			let distance = b2Cross(b2Sub(hull.points[j], p), e);
			if (distance >= 0.0) {
				return false;
			}
		}
	}

	// test for collinear points
	const linearSlop = B2_LINEAR_SLOP();
	for (let i = 0; i < hull.count; ++i) {
		let i1 = i;
		let i2 = (i + 1) % hull.count;
		let i3 = (i + 2) % hull.count;

		let p1 = hull.points[i1];
		let p2 = hull.points[i2];
		let p3 = hull.points[i3];

		let e = b2Normalize(b2Sub(p3, p1));

		let distance = b2Cross(b2Sub(p2, p1), e);
		if (distance <= linearSlop) {
			// p1-p2-p3 are collinear
			return false;
		}
	}

	return true;
}
