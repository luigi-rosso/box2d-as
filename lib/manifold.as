// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/manifold.c

import { B2_ASSERT, B2_ASSERT_ENABLED, FLT_EPSILON, FLT_MAX } from "./core";
import { B2_LINEAR_SLOP, B2_SPECULATIVE_DISTANCE } from "./constants";
import {
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossSV,
	b2DistanceSquared,
	b2Dot,
	b2GetLengthAndNormalize,
	b2LeftPerp,
	b2LengthSquared,
	b2Lerp,
	b2MinFloat,
	b2MulAdd,
	b2MulSub,
	b2MulSV,
	b2Neg,
	b2Normalize,
	b2RightPerp,
	b2RotateVector,
	b2Sub,
	b2Transform,
	b2TransformPoint,
	b2Transform_identity,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import {
	b2Capsule,
	b2ChainSegment,
	b2Circle,
	b2DistanceInput,
	b2LocalManifold,
	b2Polygon,
	b2Segment,
	b2SimplexCache,
} from "./collision";
import { b2ComputeSegmentDistance, b2ComputeShapeDistance, b2MakeProxy, b2MakeSegmentProxy } from "./distance";

@inline
function B2_MAKE_ID(A: i32, B: i32): u16 {
	return <u16>(((A & 0xff) << 8) | (B & 0xff));
}

function b2MakeCapsule(p1: b2Vec2, p2: b2Vec2, radius: f32): b2Polygon {
	let shape: b2Polygon;
	shape.vertices[0] = p1;
	shape.vertices[1] = p2;
	shape.centroid = b2Lerp(p1, p2, 0.5);

	let d = b2Sub(p2, p1);
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2LengthSquared(d) > FLT_EPSILON);
	let axis = b2Normalize(d);
	let normal = b2RightPerp(axis);

	shape.normals[0] = normal;
	shape.normals[1] = b2Neg(normal);
	shape.count = 2;
	shape.radius = radius;

	return shape;
}

/// Compute the contact manifold between two circles
export function b2CollideCircles(circleA: b2Circle, circleB: b2Circle, xf: b2Transform): b2LocalManifold {
	let manifold: b2LocalManifold;

	let pointA = circleA.center;
	let pointB = b2TransformPoint(xf, circleB.center);

	// rasc binds ref only to a struct leaf, so the length lands in distance.x
	let distance: b2Vec2;
	let normal = b2GetLengthAndNormalize(distance.x, b2Sub(pointB, pointA));

	let radiusA = circleA.radius;
	let radiusB = circleB.radius;

	let separation = distance.x - radiusA - radiusB;
	if (separation > B2_SPECULATIVE_DISTANCE()) {
		return manifold;
	}

	let cA = b2MulAdd(pointA, radiusA, normal);
	let cB = b2MulAdd(pointB, -radiusB, normal);

	manifold.normal = normal;
	manifold.points[0].point = b2Lerp(cA, cB, 0.5);
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

/// Compute the contact manifold between a capsule and circle
export function b2CollideCapsuleAndCircle(capsuleA: b2Capsule, circleB: b2Circle, xf: b2Transform): b2LocalManifold {
	let manifold: b2LocalManifold;

	// Compute circle position in the frame of the capsule.
	let pB = b2TransformPoint(xf, circleB.center);

	// Compute closest point
	let p1 = capsuleA.center1;
	let p2 = capsuleA.center2;

	let e = b2Sub(p2, p1);

	// dot(p - pA, e) = 0
	// dot(p - (p1 + s1 * e), e) = 0
	// s1 = dot(p - p1, e)
	let pA: b2Vec2;
	let s1 = b2Dot(b2Sub(pB, p1), e);
	let s2 = b2Dot(b2Sub(p2, pB), e);
	if (s1 < 0.0) {
		// p1 region
		pA = p1;
	} else if (s2 < 0.0) {
		// p2 region
		pA = p2;
	} else {
		// circle colliding with segment interior
		let s = s1 / b2Dot(e, e);
		pA = b2MulAdd(p1, s, e);
	}

	let distance: b2Vec2;
	let normal = b2GetLengthAndNormalize(distance.x, b2Sub(pB, pA));

	let radiusA = capsuleA.radius;
	let radiusB = circleB.radius;
	let separation = distance.x - radiusA - radiusB;
	if (separation > B2_SPECULATIVE_DISTANCE()) {
		return manifold;
	}

	let cA = b2MulAdd(pA, radiusA, normal);
	let cB = b2MulAdd(pB, -radiusB, normal);

	manifold.normal = normal;
	manifold.points[0].point = b2Lerp(cA, cB, 0.5);
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

/// Compute the contact manifold between a polygon and a circle
export function b2CollidePolygonAndCircle(polygonA: ref b2Polygon, circleB: b2Circle, xf: b2Transform): b2LocalManifold {
	let manifold: b2LocalManifold;
	const speculativeDistance = B2_SPECULATIVE_DISTANCE();

	// Compute circle position in the frame of the polygon.
	let center = b2TransformPoint(xf, circleB.center);
	let radiusA = polygonA.radius;
	let radiusB = circleB.radius;
	let radius = radiusA + radiusB;

	// Find the min separating edge.
	let normalIndex = 0;
	let separation = -FLT_MAX;
	let vertexCount = polygonA.count;

	for (let i = 0; i < vertexCount; ++i) {
		let s = b2Dot(polygonA.normals[i], b2Sub(center, polygonA.vertices[i]));
		if (s > separation) {
			separation = s;
			normalIndex = i;
		}
	}

	if (separation > radius + speculativeDistance) {
		return manifold;
	}

	// Vertices of the reference edge.
	let vertIndex1 = normalIndex;
	let vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	let v1 = polygonA.vertices[vertIndex1];
	let v2 = polygonA.vertices[vertIndex2];

	// Compute barycentric coordinates
	let u1 = b2Dot(b2Sub(center, v1), b2Sub(v2, v1));
	let u2 = b2Dot(b2Sub(center, v2), b2Sub(v1, v2));

	if (u1 < 0.0 && separation > FLT_EPSILON) {
		// Circle center is closest to v1 and safely outside the polygon
		let normal = b2Normalize(b2Sub(center, v1));
		separation = b2Dot(b2Sub(center, v1), normal);
		if (separation > radius + speculativeDistance) {
			return manifold;
		}

		let cA = b2MulAdd(v1, radiusA, normal);
		let cB = b2MulSub(center, radiusB, normal);
		let contactPointA = b2Lerp(cA, cB, 0.5);

		manifold.normal = normal;
		manifold.points[0].point = contactPointA;
		manifold.points[0].separation = b2Dot(b2Sub(cB, cA), normal);
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	} else if (u2 < 0.0 && separation > FLT_EPSILON) {
		// Circle center is closest to v2 and safely outside the polygon
		let normal = b2Normalize(b2Sub(center, v2));
		separation = b2Dot(b2Sub(center, v2), normal);
		if (separation > radius + speculativeDistance) {
			return manifold;
		}

		let cA = b2MulAdd(v2, radiusA, normal);
		let cB = b2MulSub(center, radiusB, normal);
		let contactPointA = b2Lerp(cA, cB, 0.5);

		manifold.normal = normal;
		manifold.points[0].point = contactPointA;
		manifold.points[0].separation = b2Dot(b2Sub(cB, cA), normal);
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	} else {
		// Circle center is between v1 and v2. Center may be inside polygon
		let normal = polygonA.normals[normalIndex];
		manifold.normal = normal;

		// cA is the projection of the circle center onto to the reference edge
		let cA = b2MulAdd(center, radiusA - b2Dot(b2Sub(center, v1), normal), normal);

		// cB is the deepest point on the circle with respect to the reference edge
		let cB = b2MulSub(center, radiusB, normal);

		// The contact point is the midpoint
		manifold.points[0].point = b2Lerp(cA, cB, 0.5);
		manifold.points[0].separation = separation - radius;
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}

	return manifold;
}

/// Compute the contact manifold between two capsules
// Follows Ericson 5.1.9 Closest Points of Two Line Segments
// Adds some logic to support clipping to get two contact points
export function b2CollideCapsules(capsuleA: b2Capsule, capsuleB: b2Capsule, xf: b2Transform): b2LocalManifold {
	let origin = capsuleA.center1;

	// Shift to the origin in frame A for round-off, a pure translation in A's frame
	let xfs = b2Transform(b2Sub(xf.p, origin), xf.q);

	let p1 = b2Vec2_zero;
	let q1 = b2Sub(capsuleA.center2, origin);

	let p2 = b2TransformPoint(xfs, capsuleB.center1);
	let q2 = b2TransformPoint(xfs, capsuleB.center2);

	let d1 = b2Sub(q1, p1);
	let d2 = b2Sub(q2, p2);

	let dd1 = b2Dot(d1, d1);
	let dd2 = b2Dot(d2, d2);

	const epsSqr = FLT_EPSILON * FLT_EPSILON;
	if (B2_ASSERT_ENABLED) B2_ASSERT(dd1 > epsSqr && dd2 > epsSqr);

	let r = b2Sub(p1, p2);
	let rd1 = b2Dot(r, d1);
	let rd2 = b2Dot(r, d2);

	let d12 = b2Dot(d1, d2);

	let denom = dd1 * dd2 - d12 * d12;

	// Fraction on segment 1
	let f1: f32 = 0.0;
	if (denom != 0.0) {
		// not parallel
		f1 = b2ClampFloat((d12 * rd2 - rd1 * dd2) / denom, 0.0, 1.0);
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

	let closest1 = b2MulAdd(p1, f1, d1);
	let closest2 = b2MulAdd(p2, f2, d2);
	let distanceSquared = b2DistanceSquared(closest1, closest2);

	let manifold: b2LocalManifold;
	let radiusA = capsuleA.radius;
	let radiusB = capsuleB.radius;
	let radius = radiusA + radiusB;
	let maxDistance = radius + B2_SPECULATIVE_DISTANCE();

	if (distanceSquared > maxDistance * maxDistance) {
		return manifold;
	}

	let distance = Mathf.sqrt(distanceSquared);

	// x is length1, y is length2
	let lengths: b2Vec2;
	let u1 = b2GetLengthAndNormalize(lengths.x, d1);
	let u2 = b2GetLengthAndNormalize(lengths.y, d2);
	let length1 = lengths.x;
	let length2 = lengths.y;

	// Does segment B project outside segment A?
	let fp2 = b2Dot(b2Sub(p2, p1), u1);
	let fq2 = b2Dot(b2Sub(q2, p1), u1);
	let outsideA = (fp2 <= 0.0 && fq2 <= 0.0) || (fp2 >= length1 && fq2 >= length1);

	// Does segment A project outside segment B?
	let fp1 = b2Dot(b2Sub(p1, p2), u2);
	let fq1 = b2Dot(b2Sub(q1, p2), u2);
	let outsideB = (fp1 <= 0.0 && fq1 <= 0.0) || (fp1 >= length2 && fq1 >= length2);

	if (outsideA == false && outsideB == false) {
		// attempt to clip
		// this may yield contact points with excessive separation
		// in that case the algorithm falls back to single point collision

		// find reference edge using SAT
		let normalA: b2Vec2;
		let separationA: f32;

		{
			normalA = b2LeftPerp(u1);
			let ss1 = b2Dot(b2Sub(p2, p1), normalA);
			let ss2 = b2Dot(b2Sub(q2, p1), normalA);
			let s1p = ss1 < ss2 ? ss1 : ss2;
			let s1n = -ss1 < -ss2 ? -ss1 : -ss2;

			if (s1p > s1n) {
				separationA = s1p;
			} else {
				separationA = s1n;
				normalA = b2Neg(normalA);
			}
		}

		let normalB: b2Vec2;
		let separationB: f32;
		{
			normalB = b2LeftPerp(u2);
			let ss1 = b2Dot(b2Sub(p1, p2), normalB);
			let ss2 = b2Dot(b2Sub(q1, p2), normalB);
			let s1p = ss1 < ss2 ? ss1 : ss2;
			let s1n = -ss1 < -ss2 ? -ss1 : -ss2;

			if (s1p > s1n) {
				separationB = s1p;
			} else {
				separationB = s1n;
				normalB = b2Neg(normalB);
			}
		}

		// biased to avoid feature flip-flop
		// todo more testing?
		if (separationA + 0.1 * B2_LINEAR_SLOP() >= separationB) {
			manifold.normal = normalA;

			let cp = p2;
			let cq = q2;

			// clip to p1
			if (fp2 < 0.0 && fq2 > 0.0) {
				cp = b2Lerp(p2, q2, (0.0 - fp2) / (fq2 - fp2));
			} else if (fq2 < 0.0 && fp2 > 0.0) {
				cq = b2Lerp(q2, p2, (0.0 - fq2) / (fp2 - fq2));
			}

			// clip to q1
			if (fp2 > length1 && fq2 < length1) {
				cp = b2Lerp(p2, q2, (fp2 - length1) / (fp2 - fq2));
			} else if (fq2 > length1 && fp2 < length1) {
				cq = b2Lerp(q2, p2, (fq2 - length1) / (fq2 - fp2));
			}

			let sp = b2Dot(b2Sub(cp, p1), normalA);
			let sq = b2Dot(b2Sub(cq, p1), normalA);

			if (sp <= distance + B2_LINEAR_SLOP() || sq <= distance + B2_LINEAR_SLOP()) {
				manifold.points[0].point = b2MulAdd(cp, 0.5 * (radiusA - radiusB - sp), normalA);
				manifold.points[0].separation = sp - radius;
				manifold.points[0].id = B2_MAKE_ID(0, 0);

				manifold.points[1].point = b2MulAdd(cq, 0.5 * (radiusA - radiusB - sq), normalA);
				manifold.points[1].separation = sq - radius;
				manifold.points[1].id = B2_MAKE_ID(0, 1);
				manifold.pointCount = 2;
			}
		} else {
			// normal always points from A to B
			manifold.normal = b2Neg(normalB);

			let cp = p1;
			let cq = q1;

			// clip to p2
			if (fp1 < 0.0 && fq1 > 0.0) {
				cp = b2Lerp(p1, q1, (0.0 - fp1) / (fq1 - fp1));
			} else if (fq1 < 0.0 && fp1 > 0.0) {
				cq = b2Lerp(q1, p1, (0.0 - fq1) / (fp1 - fq1));
			}

			// clip to q2
			if (fp1 > length2 && fq1 < length2) {
				cp = b2Lerp(p1, q1, (fp1 - length2) / (fp1 - fq1));
			} else if (fq1 > length2 && fp1 < length2) {
				cq = b2Lerp(q1, p1, (fq1 - length2) / (fq1 - fp1));
			}

			let sp = b2Dot(b2Sub(cp, p2), normalB);
			let sq = b2Dot(b2Sub(cq, p2), normalB);

			if (sp <= distance + B2_LINEAR_SLOP() || sq <= distance + B2_LINEAR_SLOP()) {
				manifold.points[0].point = b2MulAdd(cp, 0.5 * (radiusB - radiusA - sp), normalB);
				manifold.points[0].separation = sp - radius;
				manifold.points[0].id = B2_MAKE_ID(0, 0);
				manifold.points[1].point = b2MulAdd(cq, 0.5 * (radiusB - radiusA - sq), normalB);
				manifold.points[1].separation = sq - radius;
				manifold.points[1].id = B2_MAKE_ID(1, 0);
				manifold.pointCount = 2;
			}
		}
	}

	if (manifold.pointCount == 0) {
		// single point collision
		let normal = b2Sub(closest2, closest1);
		if (b2Dot(normal, normal) > epsSqr) {
			normal = b2Normalize(normal);
		} else {
			normal = b2LeftPerp(u1);
		}

		let c1 = b2MulAdd(closest1, radiusA, normal);
		let c2 = b2MulAdd(closest2, -radiusB, normal);

		let i1 = f1 == 0.0 ? 0 : 1;
		let i2 = f2 == 0.0 ? 0 : 1;

		manifold.normal = normal;
		manifold.points[0].point = b2Lerp(c1, c2, 0.5);
		manifold.points[0].separation = Mathf.sqrt(distanceSquared) - radius;
		manifold.points[0].id = B2_MAKE_ID(i1, i2);
		manifold.pointCount = 1;
	}

	// Undo the origin shift so points are in frame A
	for (let i = 0; i < manifold.pointCount; ++i) {
		manifold.points[i].point = b2Add(manifold.points[i].point, origin);
	}

	return manifold;
}

/// Compute the contact manifold between a segment and a capsule
export function b2CollideSegmentAndCapsule(segmentA: b2Segment, capsuleB: b2Capsule, xf: b2Transform): b2LocalManifold {
	let capsuleA = b2Capsule(segmentA.point1, segmentA.point2, 0.0);
	return b2CollideCapsules(capsuleA, capsuleB, xf);
}

/// Compute the contact manifold between a polygon and capsule
export function b2CollidePolygonAndCapsule(polygonA: ref b2Polygon, capsuleB: b2Capsule, xf: b2Transform): b2LocalManifold {
	let polyB = b2MakeCapsule(capsuleB.center1, capsuleB.center2, capsuleB.radius);
	return b2CollidePolygons(polygonA, polyB, xf);
}

// Polygon clipper used to compute contact points when there are potentially two contact points.
// C points poly1 and poly2 at the reference and incident polygons; the values they feed are read
// out up front instead.
function b2ClipPolygons(polyA: ref b2Polygon, polyB: ref b2Polygon, edgeA: i32, edgeB: i32, flip: bool): b2LocalManifold {
	let manifold: b2LocalManifold;

	// reference polygon
	let i11: i32;
	let i12: i32;

	// incident polygon
	let i21: i32;
	let i22: i32;

	let normal: b2Vec2;
	let v11: b2Vec2;
	let v12: b2Vec2;
	let v21: b2Vec2;
	let v22: b2Vec2;
	let r1: f32;
	let r2: f32;

	if (flip) {
		i11 = edgeB;
		i12 = edgeB + 1 < polyB.count ? edgeB + 1 : 0;
		i21 = edgeA;
		i22 = edgeA + 1 < polyA.count ? edgeA + 1 : 0;

		normal = polyB.normals[i11];

		// Reference edge vertices
		v11 = polyB.vertices[i11];
		v12 = polyB.vertices[i12];

		// Incident edge vertices
		v21 = polyA.vertices[i21];
		v22 = polyA.vertices[i22];

		r1 = polyB.radius;
		r2 = polyA.radius;
	} else {
		i11 = edgeA;
		i12 = edgeA + 1 < polyA.count ? edgeA + 1 : 0;
		i21 = edgeB;
		i22 = edgeB + 1 < polyB.count ? edgeB + 1 : 0;

		normal = polyA.normals[i11];

		// Reference edge vertices
		v11 = polyA.vertices[i11];
		v12 = polyA.vertices[i12];

		// Incident edge vertices
		v21 = polyB.vertices[i21];
		v22 = polyB.vertices[i22];

		r1 = polyA.radius;
		r2 = polyB.radius;
	}

	let tangent = b2CrossSV(1.0, normal);

	let lower1: f32 = 0.0;
	let upper1 = b2Dot(b2Sub(v12, v11), tangent);

	// Incident edge points opposite of tangent due to CCW winding
	let upper2 = b2Dot(b2Sub(v21, v11), tangent);
	let lower2 = b2Dot(b2Sub(v22, v11), tangent);

	// Are the segments disjoint?
	if (upper2 < lower1 || upper1 < lower2) {
		return manifold;
	}

	let vLower: b2Vec2;
	if (lower2 < lower1 && upper2 - lower2 > FLT_EPSILON) {
		vLower = b2Lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2));
	} else {
		vLower = v22;
	}

	let vUpper: b2Vec2;
	if (upper2 > upper1 && upper2 - lower2 > FLT_EPSILON) {
		vUpper = b2Lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2));
	} else {
		vUpper = v21;
	}

	// todo vLower can be very close to vUpper, reduce to one point?

	let separationLower = b2Dot(b2Sub(vLower, v11), normal);
	let separationUpper = b2Dot(b2Sub(vUpper, v11), normal);

	// Put contact points at midpoint, accounting for radii
	vLower = b2MulAdd(vLower, 0.5 * (r1 - r2 - separationLower), normal);
	vUpper = b2MulAdd(vUpper, 0.5 * (r1 - r2 - separationUpper), normal);

	let radius = r1 + r2;

	if (flip == false) {
		manifold.normal = normal;

		manifold.points[0].point = vLower;
		manifold.points[0].separation = separationLower - radius;
		manifold.points[0].id = B2_MAKE_ID(i11, i22);

		manifold.points[1].point = vUpper;
		manifold.points[1].separation = separationUpper - radius;
		manifold.points[1].id = B2_MAKE_ID(i12, i21);
		manifold.pointCount = 2;
	} else {
		manifold.normal = b2Neg(normal);

		manifold.points[0].point = vUpper;
		manifold.points[0].separation = separationUpper - radius;
		manifold.points[0].id = B2_MAKE_ID(i21, i12);

		manifold.points[1].point = vLower;
		manifold.points[1].separation = separationLower - radius;
		manifold.points[1].id = B2_MAKE_ID(i22, i11);
		manifold.pointCount = 2;
	}

	return manifold;
}

// C returns the separation and writes the edge index through a pointer.
struct b2MaxSeparation {
	separation: f32 = 0;
	edgeIndex: i32 = 0;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
// The SSE2 branch of C is not compiled in the scalar reference build.
function b2FindMaxSeparation(poly1: ref b2Polygon, poly2: ref b2Polygon): b2MaxSeparation {
	let count1 = poly1.count;
	let count2 = poly2.count;

	let bestIndex = 0;
	let maxSeparation = -FLT_MAX;
	for (let i = 0; i < count1; ++i) {
		// Get poly1 normal in frame2.
		let n = poly1.normals[i];
		let v1 = poly1.vertices[i];

		// Find the deepest point for normal i.
		let si = FLT_MAX;
		for (let j = 0; j < count2; ++j) {
			let sij = b2Dot(n, b2Sub(poly2.vertices[j], v1));
			if (sij < si) {
				si = sij;
			}
		}

		if (si > maxSeparation) {
			maxSeparation = si;
			bestIndex = i;
		}
	}

	let result: b2MaxSeparation;
	result.edgeIndex = bestIndex;
	result.separation = maxSeparation;
	return result;
}

// Due to speculation, every polygon is rounded
// Algorithm:
//
// compute edge separation using the separating axis test (SAT)
// if (separation > speculation_distance)
//   return
// find reference and incident edge
// if separation >= 0.1f * B2_LINEAR_SLOP
//   compute closest points between reference and incident edge
//   if vertices are closest
//      single vertex-vertex contact
//   else
//      clip edges
//   end
// else
//   clip edges
// end

/// Compute the contact manifold between two polygons
export function b2CollidePolygons(polygonA: ref b2Polygon, polygonB: ref b2Polygon, xf: b2Transform): b2LocalManifold {
	let origin = polygonA.vertices[0];
	let linearSlop = B2_LINEAR_SLOP();
	let speculativeDistance = B2_SPECULATIVE_DISTANCE();

	// Shift to the origin in frame A for round-off, a pure translation in A's frame
	let xfs = b2Transform(b2Sub(xf.p, origin), xf.q);

	let localPolyA: b2Polygon;
	localPolyA.count = polygonA.count;
	localPolyA.radius = polygonA.radius;
	localPolyA.vertices[0] = b2Vec2_zero;
	localPolyA.normals[0] = polygonA.normals[0];
	for (let i = 1; i < localPolyA.count; ++i) {
		localPolyA.vertices[i] = b2Sub(polygonA.vertices[i], origin);
		localPolyA.normals[i] = polygonA.normals[i];
	}

	// Put polyB in polyA's frame to reduce round-off error
	let localPolyB: b2Polygon;
	localPolyB.count = polygonB.count;
	localPolyB.radius = polygonB.radius;
	for (let i = 0; i < localPolyB.count; ++i) {
		localPolyB.vertices[i] = b2TransformPoint(xfs, polygonB.vertices[i]);
		localPolyB.normals[i] = b2RotateVector(xfs.q, polygonB.normals[i]);
	}

	let radius = localPolyA.radius + localPolyB.radius;

	let maxSeparationA = b2FindMaxSeparation(localPolyA, localPolyB);
	let edgeA = maxSeparationA.edgeIndex;
	let separationA = maxSeparationA.separation;
	if (separationA > speculativeDistance + radius) {
		let empty: b2LocalManifold;
		return empty;
	}

	let maxSeparationB = b2FindMaxSeparation(localPolyB, localPolyA);
	let edgeB = maxSeparationB.edgeIndex;
	let separationB = maxSeparationB.separation;
	if (separationB > speculativeDistance + radius) {
		let empty: b2LocalManifold;
		return empty;
	}

	// Find incident edge
	let flip: bool;
	if (separationA >= separationB) {
		flip = false;

		let searchDirection = localPolyA.normals[edgeA];

		// Find the incident edge on polyB
		let count = localPolyB.count;
		edgeB = 0;
		let minDot = FLT_MAX;
		for (let i = 0; i < count; ++i) {
			let dot = b2Dot(searchDirection, localPolyB.normals[i]);
			if (dot < minDot) {
				minDot = dot;
				edgeB = i;
			}
		}
	} else {
		flip = true;

		let searchDirection = localPolyB.normals[edgeB];

		// Find the incident edge on polyA
		let count = localPolyA.count;
		edgeA = 0;
		let minDot = FLT_MAX;
		for (let i = 0; i < count; ++i) {
			let dot = b2Dot(searchDirection, localPolyA.normals[i]);
			if (dot < minDot) {
				minDot = dot;
				edgeA = i;
			}
		}
	}

	let manifold: b2LocalManifold;

	// Using slop here to ensure vertex-vertex normal vectors can be safely normalized
	// todo this means edge clipping needs to handle slightly non-overlapping edges.
	if (separationA > 0.1 * linearSlop || separationB > 0.1 * linearSlop) {
		// Edges are disjoint. Find closest points between reference edge and incident edge
		// Reference edge on polygon A
		let i11 = edgeA;
		let i12 = edgeA + 1 < localPolyA.count ? edgeA + 1 : 0;
		let i21 = edgeB;
		let i22 = edgeB + 1 < localPolyB.count ? edgeB + 1 : 0;

		let v11 = localPolyA.vertices[i11];
		let v12 = localPolyA.vertices[i12];
		let v21 = localPolyB.vertices[i21];
		let v22 = localPolyB.vertices[i22];

		let result = b2ComputeSegmentDistance(v11, v12, v21, v22);
		if (B2_ASSERT_ENABLED) B2_ASSERT(result.distanceSquared > 0.0);
		let distance = Mathf.sqrt(result.distanceSquared);
		let separation = distance - radius;

		if (distance - radius > speculativeDistance) {
			// This can happen in the vertex-vertex case
			return manifold;
		}

		// Attempt to clip edges
		manifold = b2ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);

		let minSeparation = FLT_MAX;
		for (let i = 0; i < manifold.pointCount; ++i) {
			minSeparation = b2MinFloat(minSeparation, manifold.points[i].separation);
		}

		// Does vertex-vertex have substantially larger separation?
		if (separation + 0.1 * linearSlop < minSeparation) {
			if (result.fraction1 == 0.0 && result.fraction2 == 0.0) {
				// v11 - v21
				let normal = b2Sub(v21, v11);
				let invDistance: f32 = 1.0 / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				let c1 = b2MulAdd(v11, localPolyA.radius, normal);
				let c2 = b2MulAdd(v21, -localPolyB.radius, normal);

				manifold.normal = normal;
				manifold.points[0].point = b2Lerp(c1, c2, 0.5);
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID(i11, i21);
				manifold.pointCount = 1;
			} else if (result.fraction1 == 0.0 && result.fraction2 == 1.0) {
				// v11 - v22
				let normal = b2Sub(v22, v11);
				let invDistance: f32 = 1.0 / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				let c1 = b2MulAdd(v11, localPolyA.radius, normal);
				let c2 = b2MulAdd(v22, -localPolyB.radius, normal);

				manifold.normal = normal;
				manifold.points[0].point = b2Lerp(c1, c2, 0.5);
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID(i11, i22);
				manifold.pointCount = 1;
			} else if (result.fraction1 == 1.0 && result.fraction2 == 0.0) {
				// v12 - v21
				let normal = b2Sub(v21, v12);
				let invDistance: f32 = 1.0 / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				let c1 = b2MulAdd(v12, localPolyA.radius, normal);
				let c2 = b2MulAdd(v21, -localPolyB.radius, normal);

				manifold.normal = normal;
				manifold.points[0].point = b2Lerp(c1, c2, 0.5);
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID(i12, i21);
				manifold.pointCount = 1;
			} else if (result.fraction1 == 1.0 && result.fraction2 == 1.0) {
				// v12 - v22
				let normal = b2Sub(v22, v12);
				let invDistance: f32 = 1.0 / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;

				let c1 = b2MulAdd(v12, localPolyA.radius, normal);
				let c2 = b2MulAdd(v22, -localPolyB.radius, normal);

				manifold.normal = normal;
				manifold.points[0].point = b2Lerp(c1, c2, 0.5);
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID(i12, i22);
				manifold.pointCount = 1;
			}
		}
	} else {
		// Polygons overlap
		manifold = b2ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);
	}

	// Undo the origin shift so points are in frame A
	for (let i = 0; i < manifold.pointCount; ++i) {
		manifold.points[i].point = b2Add(manifold.points[i].point, origin);
	}

	return manifold;
}

/// Compute the contact manifold between a segment and a circle
export function b2CollideSegmentAndCircle(segmentA: b2Segment, circleB: b2Circle, xf: b2Transform): b2LocalManifold {
	let capsuleA = b2Capsule(segmentA.point1, segmentA.point2, 0.0);
	return b2CollideCapsuleAndCircle(capsuleA, circleB, xf);
}

/// Compute the contact manifold between a segment and a polygon
export function b2CollideSegmentAndPolygon(segmentA: b2Segment, polygonB: ref b2Polygon, xf: b2Transform): b2LocalManifold {
	let polygonA = b2MakeCapsule(segmentA.point1, segmentA.point2, 0.0);
	return b2CollidePolygons(polygonA, polygonB, xf);
}

/// Compute the contact manifold between a chain segment and a circle
export function b2CollideChainSegmentAndCircle(segmentA: b2ChainSegment, circleB: b2Circle, xf: b2Transform): b2LocalManifold {
	let manifold: b2LocalManifold;

	// Compute circle in frame of segment
	let pB = b2TransformPoint(xf, circleB.center);

	let p1 = segmentA.segment.point1;
	let p2 = segmentA.segment.point2;
	let e = b2Sub(p2, p1);

	// Normal points to the right
	let offset = b2Dot(b2RightPerp(e), b2Sub(pB, p1));
	if (offset < 0.0) {
		// collision is one-sided
		return manifold;
	}

	// Barycentric coordinates
	let u = b2Dot(e, b2Sub(p2, pB));
	let v = b2Dot(e, b2Sub(pB, p1));

	let pA: b2Vec2;

	if (v <= 0.0) {
		// Behind point1?
		// Is pB in the Voronoi region of the previous edge?
		let prevEdge = b2Sub(p1, segmentA.ghost1);
		let uPrev = b2Dot(prevEdge, b2Sub(pB, p1));
		if (uPrev <= 0.0) {
			return manifold;
		}

		pA = p1;
	} else if (u <= 0.0) {
		// Ahead of point2?
		let nextEdge = b2Sub(segmentA.ghost2, p2);
		let vNext = b2Dot(nextEdge, b2Sub(pB, p2));

		// Is pB in the Voronoi region of the next edge?
		if (vNext > 0.0) {
			return manifold;
		}

		pA = p2;
	} else {
		let ee = b2Dot(e, e);
		pA = b2Vec2(u * p1.x + v * p2.x, u * p1.y + v * p2.y);
		pA = ee > 0.0 ? b2MulSV(1.0 / ee, pA) : p1;
	}

	let distance: b2Vec2;
	let normal = b2GetLengthAndNormalize(distance.x, b2Sub(pB, pA));

	let radius = circleB.radius;
	let separation = distance.x - radius;
	if (separation > B2_SPECULATIVE_DISTANCE()) {
		return manifold;
	}

	let cA = pA;
	let cB = b2MulAdd(pB, -radius, normal);

	manifold.normal = normal;

	manifold.points[0].point = b2Lerp(cA, cB, 0.5);
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

/// Compute the contact manifold between a chain segment and a capsule
export function b2CollideChainSegmentAndCapsule(
	segmentA: b2ChainSegment,
	capsuleB: b2Capsule,
	xf: b2Transform,
	cache: ref b2SimplexCache,
): b2LocalManifold {
	let polyB = b2MakeCapsule(capsuleB.center1, capsuleB.center2, capsuleB.radius);
	return b2CollideChainSegmentAndPolygon(segmentA, polyB, xf, cache);
}

function b2ClipSegments(
	a1: b2Vec2,
	a2: b2Vec2,
	b1: b2Vec2,
	b2: b2Vec2,
	normal: b2Vec2,
	ra: f32,
	rb: f32,
	id1: u16,
	id2: u16,
): b2LocalManifold {
	let manifold: b2LocalManifold;

	let tangent = b2LeftPerp(normal);

	// Barycentric coordinates of each point relative to a1 along tangent
	let lower1: f32 = 0.0;
	let upper1 = b2Dot(b2Sub(a2, a1), tangent);

	// Incident edge points opposite of tangent due to CCW winding
	let upper2 = b2Dot(b2Sub(b1, a1), tangent);
	let lower2 = b2Dot(b2Sub(b2, a1), tangent);

	// Do segments overlap?
	if (upper2 < lower1 || upper1 < lower2) {
		return manifold;
	}

	let vLower: b2Vec2;
	if (lower2 < lower1 && upper2 - lower2 > FLT_EPSILON) {
		vLower = b2Lerp(b2, b1, (lower1 - lower2) / (upper2 - lower2));
	} else {
		vLower = b2;
	}

	let vUpper: b2Vec2;
	if (upper2 > upper1 && upper2 - lower2 > FLT_EPSILON) {
		vUpper = b2Lerp(b2, b1, (upper1 - lower2) / (upper2 - lower2));
	} else {
		vUpper = b1;
	}

	// todo vLower can be very close to vUpper, reduce to one point?

	let separationLower = b2Dot(b2Sub(vLower, a1), normal);
	let separationUpper = b2Dot(b2Sub(vUpper, a1), normal);

	// Put contact points at midpoint, accounting for radii
	vLower = b2MulAdd(vLower, 0.5 * (ra - rb - separationLower), normal);
	vUpper = b2MulAdd(vUpper, 0.5 * (ra - rb - separationUpper), normal);

	let radius = ra + rb;

	manifold.normal = normal;
	manifold.points[0].point = vLower;
	manifold.points[0].separation = separationLower - radius;
	manifold.points[0].id = id1;

	manifold.points[1].point = vUpper;
	manifold.points[1].separation = separationUpper - radius;
	manifold.points[1].id = id2;

	manifold.pointCount = 2;

	return manifold;
}

type b2NormalType = i32;

// This means the normal points in a direction that is non-smooth relative to a convex vertex and should be skipped
const b2_normalSkip: b2NormalType = 0;

// This means the normal points in a direction that is smooth relative to a convex vertex and should be used for collision
const b2_normalAdmit: b2NormalType = 1;

// This means the normal is in a region of a concave vertex and should be snapped to the segment normal
const b2_normalSnap: b2NormalType = 2;

struct b2ChainSegmentParams {
	edge1: b2Vec2 = b2Vec2();
	normal0: b2Vec2 = b2Vec2();
	normal2: b2Vec2 = b2Vec2();
	convex1: bool = false;
	convex2: bool = false;
}

// Evaluate Gauss map
// See https://box2d.org/posts/2020/06/ghost-collisions/
function b2ClassifyNormal(params: b2ChainSegmentParams, normal: b2Vec2): b2NormalType {
	const sinTol: f32 = 0.01;

	if (b2Dot(normal, params.edge1) <= 0.0) {
		// Normal points towards the segment tail
		if (params.convex1) {
			if (b2Cross(normal, params.normal0) > sinTol) {
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		} else {
			return b2_normalSnap;
		}
	} else {
		// Normal points towards segment head
		if (params.convex2) {
			if (b2Cross(params.normal2, normal) > sinTol) {
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		} else {
			return b2_normalSnap;
		}
	}
}

/// Compute the contact manifold between a chain segment and a rounded polygon
export function b2CollideChainSegmentAndPolygon(
	segmentA: b2ChainSegment,
	polygonB: ref b2Polygon,
	xf: b2Transform,
	cache: ref b2SimplexCache,
): b2LocalManifold {
	let manifold: b2LocalManifold;

	let centroidB = b2TransformPoint(xf, polygonB.centroid);
	let radiusB = polygonB.radius;

	let p1 = segmentA.segment.point1;
	let p2 = segmentA.segment.point2;

	let edge1 = b2Normalize(b2Sub(p2, p1));

	let smoothParams: b2ChainSegmentParams;
	smoothParams.edge1 = edge1;

	const convexTol: f32 = 0.01;
	let edge0 = b2Normalize(b2Sub(p1, segmentA.ghost1));
	smoothParams.normal0 = b2RightPerp(edge0);
	smoothParams.convex1 = b2Cross(edge0, edge1) >= convexTol;

	let edge2 = b2Normalize(b2Sub(segmentA.ghost2, p2));
	smoothParams.normal2 = b2RightPerp(edge2);
	smoothParams.convex2 = b2Cross(edge1, edge2) >= convexTol;

	// Normal points to the right
	let normal1 = b2RightPerp(edge1);
	let behind1 = b2Dot(normal1, b2Sub(centroidB, p1)) < 0.0;
	let behind0 = true;
	let behind2 = true;
	if (smoothParams.convex1) {
		behind0 = b2Dot(smoothParams.normal0, b2Sub(centroidB, p1)) < 0.0;
	}

	if (smoothParams.convex2) {
		behind2 = b2Dot(smoothParams.normal2, b2Sub(centroidB, p2)) < 0.0;
	}

	if (behind1 && behind0 && behind2) {
		// one-sided collision
		return manifold;
	}

	// Get polygonB in frameA
	let count = polygonB.count;
	let vertices: FixedArray<b2Vec2, 8>;
	let normals: FixedArray<b2Vec2, 8>;
	for (let i = 0; i < count; ++i) {
		vertices[i] = b2TransformPoint(xf, polygonB.vertices[i]);
		normals[i] = b2RotateVector(xf.q, polygonB.normals[i]);
	}

	// Distance doesn't work correctly with partial polygons
	let input: b2DistanceInput;
	input.proxyA = b2MakeSegmentProxy(segmentA.segment.point1, segmentA.segment.point2, 0.0);
	input.proxyB = b2MakeProxy(vertices, count, 0.0);
	input.transform = b2Transform_identity;
	input.useRadii = false;

	let output = b2ComputeShapeDistance(input, cache, null, 0);

	if (output.distance > radiusB + B2_SPECULATIVE_DISTANCE()) {
		return manifold;
	}

	// Snap concave normals for partial polygon
	let n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
	let n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;

	// Index of incident vertex on polygon
	let incidentIndex = -1;
	let incidentNormal = -1;

	if (behind1 == false && output.distance > 0.1 * B2_LINEAR_SLOP()) {
		// The closest features may be two vertices or an edge and a vertex even when there should
		// be face contact

		if (cache.count == 1) {
			// vertex-vertex collision
			let pA = output.pointA;
			let pB = output.pointB;

			let normal = b2Normalize(b2Sub(pB, pA));

			let type = b2ClassifyNormal(smoothParams, normal);
			if (type == b2_normalSkip) {
				return manifold;
			}

			if (type == b2_normalAdmit) {
				manifold.normal = normal;
				manifold.points[0].point = pA;
				manifold.points[0].separation = output.distance - radiusB;
				manifold.points[0].id = B2_MAKE_ID(<i32>cache.indexA[0], <i32>cache.indexB[0]);
				manifold.pointCount = 1;
				return manifold;
			}

			// fall through b2_normalSnap
			incidentIndex = <i32>cache.indexB[0];
		} else {
			// vertex-edge collision
			if (B2_ASSERT_ENABLED) B2_ASSERT(cache.count == 2);

			let ia1 = <i32>cache.indexA[0];
			let ia2 = <i32>cache.indexA[1];
			let ib1 = <i32>cache.indexB[0];
			let ib2 = <i32>cache.indexB[1];

			if (ia1 == ia2) {
				// 1 point on A, expect 2 points on B
				if (B2_ASSERT_ENABLED) B2_ASSERT(ib1 != ib2);

				// Find polygon normal most aligned with vector between closest points.
				// This effectively sorts ib1 and ib2
				let normalB = b2Sub(output.pointA, output.pointB);
				let dot1 = b2Dot(normalB, normals[ib1]);
				let dot2 = b2Dot(normalB, normals[ib2]);
				let ib = dot1 > dot2 ? ib1 : ib2;

				// Use accurate normal
				normalB = normals[ib];

				let type = b2ClassifyNormal(smoothParams, b2Neg(normalB));
				if (type == b2_normalSkip) {
					return manifold;
				}

				if (type == b2_normalAdmit) {
					// Get polygon edge associated with normal
					ib1 = ib;
					ib2 = ib < count - 1 ? ib + 1 : 0;

					let b1 = vertices[ib1];
					let b2 = vertices[ib2];

					// Find incident segment vertex
					dot1 = b2Dot(normalB, b2Sub(p1, b1));
					dot2 = b2Dot(normalB, b2Sub(p2, b1));

					if (dot1 < dot2) {
						if (b2Dot(n0, normalB) < b2Dot(normal1, normalB)) {
							// Neighbor is incident
							return manifold;
						}
					} else {
						if (b2Dot(n2, normalB) < b2Dot(normal1, normalB)) {
							// Neighbor is incident
							return manifold;
						}
					}

					manifold = b2ClipSegments(b1, b2, p1, p2, normalB, radiusB, 0.0, B2_MAKE_ID(ib1, 1), B2_MAKE_ID(ib2, 0));

					if (B2_ASSERT_ENABLED) B2_ASSERT(manifold.pointCount == 0 || manifold.pointCount == 2);
					if (manifold.pointCount == 2) {
						manifold.normal = b2Neg(normalB);
					}
					return manifold;
				}

				// fall through b2_normalSnap
				incidentNormal = ib;
			} else {
				// Get index of incident polygonB vertex
				let dot1 = b2Dot(normal1, b2Sub(vertices[ib1], p1));
				let dot2 = b2Dot(normal1, b2Sub(vertices[ib2], p2));
				incidentIndex = dot1 < dot2 ? ib1 : ib2;
			}
		}
	} else {
		// SAT edge normal
		let edgeSeparation = FLT_MAX;

		for (let i = 0; i < count; ++i) {
			let s = b2Dot(normal1, b2Sub(vertices[i], p1));
			if (s < edgeSeparation) {
				edgeSeparation = s;
				incidentIndex = i;
			}
		}

		// Check convex neighbor for edge separation
		if (smoothParams.convex1) {
			let s0 = FLT_MAX;

			for (let i = 0; i < count; ++i) {
				let s = b2Dot(smoothParams.normal0, b2Sub(vertices[i], p1));
				if (s < s0) {
					s0 = s;
				}
			}

			if (s0 > edgeSeparation) {
				edgeSeparation = s0;

				// Indicate neighbor owns edge separation
				incidentIndex = -1;
			}
		}

		// Check convex neighbor for edge separation
		if (smoothParams.convex2) {
			let s2 = FLT_MAX;

			for (let i = 0; i < count; ++i) {
				let s = b2Dot(smoothParams.normal2, b2Sub(vertices[i], p2));
				if (s < s2) {
					s2 = s;
				}
			}

			if (s2 > edgeSeparation) {
				edgeSeparation = s2;

				// Indicate neighbor owns edge separation
				incidentIndex = -1;
			}
		}

		// SAT polygon normals
		let polygonSeparation = -FLT_MAX;
		let referenceIndex = -1;

		for (let i = 0; i < count; ++i) {
			let n = normals[i];

			let type = b2ClassifyNormal(smoothParams, b2Neg(n));
			if (type != b2_normalAdmit) {
				continue;
			}

			// Check the infinite sides of the partial polygon
			// if ((smoothParams.convex1 && b2Cross(n0, n) > 0.0f) || (smoothParams.convex2 && b2Cross(n, n2) > 0.0f))
			//{
			//	continue;
			//}

			let p = vertices[i];
			let s = b2MinFloat(b2Dot(n, b2Sub(p2, p)), b2Dot(n, b2Sub(p1, p)));

			if (s > polygonSeparation) {
				polygonSeparation = s;
				referenceIndex = i;
			}
		}

		if (polygonSeparation > edgeSeparation) {
			let ia1 = referenceIndex;
			let ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
			let a1 = vertices[ia1];
			let a2 = vertices[ia2];

			let n = normals[ia1];

			let dot1 = b2Dot(n, b2Sub(p1, a1));
			let dot2 = b2Dot(n, b2Sub(p2, a1));

			if (dot1 < dot2) {
				if (b2Dot(n0, n) < b2Dot(normal1, n)) {
					// Neighbor is incident
					return manifold;
				}
			} else {
				if (b2Dot(n2, n) < b2Dot(normal1, n)) {
					// Neighbor is incident
					return manifold;
				}
			}

			manifold = b2ClipSegments(a1, a2, p1, p2, normals[ia1], radiusB, 0.0, B2_MAKE_ID(ia1, 1), B2_MAKE_ID(ia2, 0));

			if (B2_ASSERT_ENABLED) B2_ASSERT(manifold.pointCount == 0 || manifold.pointCount == 2);
			if (manifold.pointCount == 2) {
				manifold.normal = b2Neg(normals[ia1]);
			}

			return manifold;
		}

		if (incidentIndex == -1) {
			// neighboring segment is the separating axis
			return manifold;
		}

		// fall through segment normal axis
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(incidentNormal != -1 || incidentIndex != -1);

	// Segment normal

	// Find incident polygon normal: normal adjacent to deepest vertex that is most anti-parallel to segment normal
	let b1: b2Vec2;
	let b2: b2Vec2;
	let ib1: i32;
	let ib2: i32;

	if (incidentNormal != -1) {
		ib1 = incidentNormal;
		ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
		b1 = vertices[ib1];
		b2 = vertices[ib2];
	} else {
		let i2 = incidentIndex;
		let i1 = i2 > 0 ? i2 - 1 : count - 1;
		let d1 = b2Dot(normal1, normals[i1]);
		let d2 = b2Dot(normal1, normals[i2]);
		if (d1 < d2) {
			ib1 = i1;
			ib2 = i2;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		} else {
			ib1 = i2;
			ib2 = i2 < count - 1 ? i2 + 1 : 0;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
	}

	manifold = b2ClipSegments(p1, p2, b1, b2, normal1, 0.0, radiusB, B2_MAKE_ID(0, ib2), B2_MAKE_ID(1, ib1));

	if (B2_ASSERT_ENABLED) B2_ASSERT(manifold.pointCount == 0 || manifold.pointCount == 2);

	return manifold;
}
