// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/geometry.c

import { B2_ASSERT, B2_ASSERT_ENABLED, FLT_EPSILON } from "./core";
import { B2_HUGE } from "./constants";
import {
	B2_PI,
	b2AABB,
	b2Add,
	b2ClampFloat,
	b2Cross,
	b2CrossVS,
	b2DistanceSquared,
	b2Dot,
	b2GetLengthAndNormalize,
	b2IsValidFloat,
	b2IsValidVec2,
	b2Length,
	b2LengthSquared,
	b2Lerp,
	b2MulAdd,
	b2MulSub,
	b2MulSV,
	b2Neg,
	b2Normalize,
	b2Plane,
	b2RightPerp,
	b2Rot,
	b2RotateVector,
	b2RoundDownFloat,
	b2RoundUpFloat,
	b2Sub,
	b2Transform,
	b2TransformPoint,
	b2TransformWorldPoint,
	b2Transform_identity,
	b2Vec2,
	b2Vec2_zero,
	b2WorldTransform,
} from "./math_functions";
import {
	b2Capsule,
	b2CastOutput,
	b2Circle,
	b2DistanceInput,
	b2Hull,
	b2MassData,
	b2PlaneResult,
	b2Polygon,
	b2RayCastInput,
	b2Segment,
	b2ShapeCastInput,
	b2ShapeCastPairInput,
	b2SimplexCache,
} from "./collision";
import { b2CastShape, b2ComputeShapeDistance, b2MakePointProxy, b2MakeProxy, b2MakeSegmentProxy } from "./distance";
import { b2ValidateHull } from "./hull";

/// Validate ray cast input data (NaN, etc)
export function b2IsValidRay(input: b2RayCastInput): bool {
	let isValid =
		b2IsValidVec2(input.origin) &&
		b2IsValidVec2(input.translation) &&
		b2IsValidFloat(input.maxFraction) &&
		0.0 <= input.maxFraction &&
		input.maxFraction < B2_HUGE();
	return isValid;
}

function b2ComputePolygonCentroid(vertices: ref FixedArray<b2Vec2, 8>, count: i32): b2Vec2 {
	let center = b2Vec2(0.0, 0.0);
	let area: f32 = 0.0;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	let origin = vertices[0];

	const inv3: f32 = 1.0 / 3.0;

	for (let i = 1; i < count - 1; ++i) {
		// Triangle edges
		let e1 = b2Sub(vertices[i], origin);
		let e2 = b2Sub(vertices[i + 1], origin);
		let a: f32 = 0.5 * b2Cross(e1, e2);

		// Area weighted centroid
		center = b2MulAdd(center, a * inv3, b2Add(e1, e2));
		area += a;
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(area > FLT_EPSILON);
	let invArea: f32 = 1.0 / area;
	center.x *= invArea;
	center.y *= invArea;

	// Restore offset
	center = b2Add(origin, center);

	return center;
}

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
export function b2MakePolygon(hull: b2Hull, radius: f32): b2Polygon {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2ValidateHull(hull));

	if (hull.count < 3) {
		// Handle a bad hull when assertions are disabled
		return b2MakeSquare(0.5);
	}

	let shape: b2Polygon;
	shape.count = hull.count;
	shape.radius = radius;

	// Copy vertices
	for (let i = 0; i < shape.count; ++i) {
		shape.vertices[i] = hull.points[i];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (let i = 0; i < shape.count; ++i) {
		let i1 = i;
		let i2 = i + 1 < shape.count ? i + 1 : 0;
		let edge = b2Sub(shape.vertices[i2], shape.vertices[i1]);
		if (B2_ASSERT_ENABLED) B2_ASSERT(b2Dot(edge, edge) > FLT_EPSILON * FLT_EPSILON);
		shape.normals[i] = b2Normalize(b2CrossVS(edge, 1.0));
	}

	shape.centroid = b2ComputePolygonCentroid(shape.vertices, shape.count);

	return shape;
}

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
export function b2MakeOffsetPolygon(hull: b2Hull, position: b2Vec2, rotation: b2Rot): b2Polygon {
	return b2MakeOffsetRoundedPolygon(hull, position, rotation, 0.0);
}

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
export function b2MakeOffsetRoundedPolygon(hull: b2Hull, position: b2Vec2, rotation: b2Rot, radius: f32): b2Polygon {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2ValidateHull(hull));

	if (hull.count < 3) {
		// Handle a bad hull when assertions are disabled
		return b2MakeSquare(0.5);
	}

	let transform = b2Transform(position, rotation);

	let shape: b2Polygon;
	shape.count = hull.count;
	shape.radius = radius;

	// Copy vertices
	for (let i = 0; i < shape.count; ++i) {
		shape.vertices[i] = b2TransformPoint(transform, hull.points[i]);
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (let i = 0; i < shape.count; ++i) {
		let i1 = i;
		let i2 = i + 1 < shape.count ? i + 1 : 0;
		let edge = b2Sub(shape.vertices[i2], shape.vertices[i1]);
		if (B2_ASSERT_ENABLED) B2_ASSERT(b2Dot(edge, edge) > FLT_EPSILON * FLT_EPSILON);
		shape.normals[i] = b2Normalize(b2CrossVS(edge, 1.0));
	}

	shape.centroid = b2ComputePolygonCentroid(shape.vertices, shape.count);

	return shape;
}

/// Make a square polygon, bypassing the need for a convex hull.
export function b2MakeSquare(halfWidth: f32): b2Polygon {
	return b2MakeBox(halfWidth, halfWidth);
}

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
export function b2MakeBox(halfWidth: f32, halfHeight: f32): b2Polygon {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidFloat(halfWidth) && halfWidth > 0.0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidFloat(halfHeight) && halfHeight > 0.0);

	let shape: b2Polygon;
	shape.count = 4;
	shape.vertices[0] = b2Vec2(-halfWidth, -halfHeight);
	shape.vertices[1] = b2Vec2(halfWidth, -halfHeight);
	shape.vertices[2] = b2Vec2(halfWidth, halfHeight);
	shape.vertices[3] = b2Vec2(-halfWidth, halfHeight);
	shape.normals[0] = b2Vec2(0.0, -1.0);
	shape.normals[1] = b2Vec2(1.0, 0.0);
	shape.normals[2] = b2Vec2(0.0, 1.0);
	shape.normals[3] = b2Vec2(-1.0, 0.0);
	shape.radius = 0.0;
	shape.centroid = b2Vec2_zero;
	return shape;
}

/// Make a rounded box, bypassing the need for a convex hull.
export function b2MakeRoundedBox(halfWidth: f32, halfHeight: f32, radius: f32): b2Polygon {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidFloat(radius) && radius >= 0.0);
	let shape = b2MakeBox(halfWidth, halfHeight);
	shape.radius = radius;
	return shape;
}

/// Make an offset box, bypassing the need for a convex hull.
export function b2MakeOffsetBox(halfWidth: f32, halfHeight: f32, center: b2Vec2, rotation: b2Rot): b2Polygon {
	let xf = b2Transform(center, rotation);

	let shape: b2Polygon;
	shape.count = 4;
	shape.vertices[0] = b2TransformPoint(xf, b2Vec2(-halfWidth, -halfHeight));
	shape.vertices[1] = b2TransformPoint(xf, b2Vec2(halfWidth, -halfHeight));
	shape.vertices[2] = b2TransformPoint(xf, b2Vec2(halfWidth, halfHeight));
	shape.vertices[3] = b2TransformPoint(xf, b2Vec2(-halfWidth, halfHeight));
	shape.normals[0] = b2RotateVector(xf.q, b2Vec2(0.0, -1.0));
	shape.normals[1] = b2RotateVector(xf.q, b2Vec2(1.0, 0.0));
	shape.normals[2] = b2RotateVector(xf.q, b2Vec2(0.0, 1.0));
	shape.normals[3] = b2RotateVector(xf.q, b2Vec2(-1.0, 0.0));
	shape.radius = 0.0;
	shape.centroid = xf.p;
	return shape;
}

/// Make an offset rounded box, bypassing the need for a convex hull.
export function b2MakeOffsetRoundedBox(halfWidth: f32, halfHeight: f32, center: b2Vec2, rotation: b2Rot, radius: f32): b2Polygon {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidFloat(radius) && radius >= 0.0);
	let xf = b2Transform(center, rotation);

	let shape: b2Polygon;
	shape.count = 4;
	shape.vertices[0] = b2TransformPoint(xf, b2Vec2(-halfWidth, -halfHeight));
	shape.vertices[1] = b2TransformPoint(xf, b2Vec2(halfWidth, -halfHeight));
	shape.vertices[2] = b2TransformPoint(xf, b2Vec2(halfWidth, halfHeight));
	shape.vertices[3] = b2TransformPoint(xf, b2Vec2(-halfWidth, halfHeight));
	shape.normals[0] = b2RotateVector(xf.q, b2Vec2(0.0, -1.0));
	shape.normals[1] = b2RotateVector(xf.q, b2Vec2(1.0, 0.0));
	shape.normals[2] = b2RotateVector(xf.q, b2Vec2(0.0, 1.0));
	shape.normals[3] = b2RotateVector(xf.q, b2Vec2(-1.0, 0.0));
	shape.radius = radius;
	shape.centroid = xf.p;
	return shape;
}

/// Transform a polygon. This is useful for transferring a shape from one body to another.
export function b2TransformPolygon(transform: b2Transform, polygon: b2Polygon): b2Polygon {
	let p = polygon;

	for (let i = 0; i < p.count; ++i) {
		p.vertices[i] = b2TransformPoint(transform, p.vertices[i]);
		p.normals[i] = b2RotateVector(transform.q, p.normals[i]);
	}

	p.centroid = b2TransformPoint(transform, p.centroid);

	return p;
}

/// Compute mass properties of a circle
export function b2ComputeCircleMass(shape: b2Circle, density: f32): b2MassData {
	let rr = shape.radius * shape.radius;

	let massData: b2MassData;
	massData.mass = density * B2_PI * rr;
	massData.center = shape.center;

	// inertia about the center of mass
	massData.rotationalInertia = massData.mass * 0.5 * rr;

	return massData;
}

/// Compute mass properties of a capsule
export function b2ComputeCapsuleMass(shape: b2Capsule, density: f32): b2MassData {
	let radius = shape.radius;
	let rr = radius * radius;
	let p1 = shape.center1;
	let p2 = shape.center2;
	let length = b2Length(b2Sub(p2, p1));
	let ll = length * length;

	let circleMass = density * (B2_PI * radius * radius);
	let boxMass = density * (2.0 * radius * length);

	let massData: b2MassData;
	massData.mass = circleMass + boxMass;
	massData.center.x = 0.5 * (p1.x + p2.x);
	massData.center.y = 0.5 * (p1.y + p2.y);

	// two offset half circles, both halves add up to full circle and each half is offset by half length
	// semi-circle centroid = 4 r / 3 pi
	// Need to apply parallel-axis theorem twice:
	// 1. shift semi-circle centroid to origin
	// 2. shift semi-circle to box end
	// m * ((h + lc)^2 - lc^2) = m * (h^2 + 2 * h * lc)
	// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
	// I verified this formula by computing the convex hull of a 128 vertex capsule

	// half circle centroid
	let lc: f32 = (4.0 * radius) / (3.0 * B2_PI);

	// half length of rectangular portion of capsule
	let h: f32 = 0.5 * length;

	let circleInertia = circleMass * (0.5 * rr + h * h + 2.0 * h * lc);
	let boxInertia = (boxMass * (4.0 * rr + ll)) / 12.0;
	massData.rotationalInertia = circleInertia + boxInertia;

	return massData;
}

/// Compute mass properties of a polygon
export function b2ComputePolygonMass(shape: b2Polygon, density: f32): b2MassData {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	if (B2_ASSERT_ENABLED) B2_ASSERT(shape.count > 0);

	if (shape.count == 1) {
		let circle = b2Circle(shape.vertices[0], shape.radius);
		return b2ComputeCircleMass(circle, density);
	}

	if (shape.count == 2) {
		let capsule = b2Capsule(shape.vertices[0], shape.vertices[1], shape.radius);
		return b2ComputeCapsuleMass(capsule, density);
	}

	let vertices: FixedArray<b2Vec2, 8>;
	let count = shape.count;
	let radius = shape.radius;

	if (radius > 0.0) {
		// Approximate mass of rounded polygons by pushing out the vertices.
		let sqrt2: f32 = 1.412;
		for (let i = 0; i < count; ++i) {
			let j = i == 0 ? count - 1 : i - 1;
			let n1 = shape.normals[j];
			let n2 = shape.normals[i];

			let mid = b2Normalize(b2Add(n1, n2));
			vertices[i] = b2MulAdd(shape.vertices[i], sqrt2 * radius, mid);
		}
	} else {
		for (let i = 0; i < count; ++i) {
			vertices[i] = shape.vertices[i];
		}
	}

	let center = b2Vec2(0.0, 0.0);
	let area: f32 = 0.0;
	let rotationalInertia: f32 = 0.0;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	let r = vertices[0];

	const inv3: f32 = 1.0 / 3.0;

	for (let i = 1; i < count - 1; ++i) {
		// Triangle edges
		let e1 = b2Sub(vertices[i], r);
		let e2 = b2Sub(vertices[i + 1], r);

		let D = b2Cross(e1, e2);

		let triangleArea: f32 = 0.5 * D;
		area += triangleArea;

		// Area weighted centroid, r at origin
		center = b2MulAdd(center, triangleArea * inv3, b2Add(e1, e2));

		let ex1 = e1.x;
		let ey1 = e1.y;
		let ex2 = e2.x;
		let ey2 = e2.y;

		let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
		let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

		rotationalInertia += 0.25 * inv3 * D * (intx2 + inty2);
	}

	let massData: b2MassData;

	// Total mass
	massData.mass = density * area;

	// Center of mass, shift back from origin at r
	if (B2_ASSERT_ENABLED) B2_ASSERT(area > FLT_EPSILON);
	let invArea: f32 = 1.0 / area;
	center.x *= invArea;
	center.y *= invArea;
	massData.center = b2Add(r, center);

	// Inertia tensor relative to the local origin (point s).
	massData.rotationalInertia = density * rotationalInertia;

	// Shift inertia to center of mass
	massData.rotationalInertia -= massData.mass * b2Dot(center, center);

	// If this goes negative we are hosed
	if (B2_ASSERT_ENABLED) B2_ASSERT(massData.rotationalInertia >= 0.0);

	return massData;
}

// AABBs are built in double and narrowed to float once. In large world mode the narrowing rounds
// outward so the box always contains the shape, and the inflation (speculative margin) folds into
// the double step, otherwise it vanishes into a float ULP far from the origin. In float mode the
// rounding helpers are plain casts.

// The four fat AABB helpers are static in C. They are exported for b2ComputeFatShapeAABB, which
// takes a b2Shape and so lives in lib/shape.as.

export function b2ComputeCircleFatAABB(shape: b2Circle, xf: b2WorldTransform, extra: f32): b2AABB {
	let c = b2TransformWorldPoint(xf, shape.center);
	let r = <f64>shape.radius + <f64>extra;
	let aabb = b2AABB(
		b2Vec2(b2RoundDownFloat(<f64>c.x - r), b2RoundDownFloat(<f64>c.y - r)),
		b2Vec2(b2RoundUpFloat(<f64>c.x + r), b2RoundUpFloat(<f64>c.y + r)),
	);
	return aabb;
}

export function b2ComputeCapsuleFatAABB(shape: b2Capsule, xf: b2WorldTransform, extra: f32): b2AABB {
	let v1 = b2TransformWorldPoint(xf, shape.center1);
	let v2 = b2TransformWorldPoint(xf, shape.center2);
	let r = <f64>shape.radius + <f64>extra;
	let aabb = b2AABB(
		b2Vec2(b2RoundDownFloat(<f64>(v1.x < v2.x ? v1.x : v2.x) - r), b2RoundDownFloat(<f64>(v1.y < v2.y ? v1.y : v2.y) - r)),
		b2Vec2(b2RoundUpFloat(<f64>(v1.x > v2.x ? v1.x : v2.x) + r), b2RoundUpFloat(<f64>(v1.y > v2.y ? v1.y : v2.y) + r)),
	);
	return aabb;
}

export function b2ComputePolygonFatAABB(shape: b2Polygon, xf: b2WorldTransform, extra: f32): b2AABB {
	if (B2_ASSERT_ENABLED) B2_ASSERT(shape.count > 0);
	let v = b2TransformWorldPoint(xf, shape.vertices[0]);
	let lx = <f64>v.x;
	let ly = <f64>v.y;
	let ux = <f64>v.x;
	let uy = <f64>v.y;

	for (let i = 1; i < shape.count; ++i) {
		v = b2TransformWorldPoint(xf, shape.vertices[i]);
		lx = <f64>v.x < lx ? <f64>v.x : lx;
		ly = <f64>v.y < ly ? <f64>v.y : ly;
		ux = <f64>v.x > ux ? <f64>v.x : ux;
		uy = <f64>v.y > uy ? <f64>v.y : uy;
	}

	let r = <f64>shape.radius + <f64>extra;
	let aabb = b2AABB(
		b2Vec2(b2RoundDownFloat(lx - r), b2RoundDownFloat(ly - r)),
		b2Vec2(b2RoundUpFloat(ux + r), b2RoundUpFloat(uy + r)),
	);
	return aabb;
}

export function b2ComputeSegmentFatAABB(shape: b2Segment, xf: b2WorldTransform, extra: f32): b2AABB {
	let v1 = b2TransformWorldPoint(xf, shape.point1);
	let v2 = b2TransformWorldPoint(xf, shape.point2);
	let e = <f64>extra;
	let aabb = b2AABB(
		b2Vec2(b2RoundDownFloat(<f64>(v1.x < v2.x ? v1.x : v2.x) - e), b2RoundDownFloat(<f64>(v1.y < v2.y ? v1.y : v2.y) - e)),
		b2Vec2(b2RoundUpFloat(<f64>(v1.x > v2.x ? v1.x : v2.x) + e), b2RoundUpFloat(<f64>(v1.y > v2.y ? v1.y : v2.y) + e)),
	);
	return aabb;
}

/// Compute the bounding box of a transformed circle
export function b2ComputeCircleBounds(shape: b2Circle, xf: b2WorldTransform): b2AABB {
	return b2ComputeCircleFatAABB(shape, xf, 0.0);
}

/// Compute the bounding box of a transformed capsule
export function b2ComputeCapsuleBounds(shape: b2Capsule, xf: b2WorldTransform): b2AABB {
	return b2ComputeCapsuleFatAABB(shape, xf, 0.0);
}

/// Compute the bounding box of a transformed polygon
export function b2ComputePolygonBounds(shape: b2Polygon, xf: b2WorldTransform): b2AABB {
	return b2ComputePolygonFatAABB(shape, xf, 0.0);
}

/// Compute the bounding box of a transformed line segment
export function b2ComputeSegmentBounds(shape: b2Segment, xf: b2WorldTransform): b2AABB {
	return b2ComputeSegmentFatAABB(shape, xf, 0.0);
}

/// Test a point for overlap with a circle in local space
export function b2PointInCircle(shape: b2Circle, point: b2Vec2): bool {
	let center = shape.center;
	return b2DistanceSquared(point, center) <= shape.radius * shape.radius;
}

/// Test a point for overlap with a capsule in local space
export function b2PointInCapsule(shape: b2Capsule, point: b2Vec2): bool {
	let rr = shape.radius * shape.radius;
	let p1 = shape.center1;
	let p2 = shape.center2;

	let d = b2Sub(p2, p1);
	let dd = b2Dot(d, d);
	if (dd == 0.0) {
		// Capsule is really a circle
		return b2DistanceSquared(point, p1) <= rr;
	}

	// Get closest point on capsule segment
	// c = p1 + t * d
	// dot(point - c, d) = 0
	// dot(point - p1 - t * d, d) = 0
	// t = dot(point - p1, d) / dot(d, d)
	let t = b2Dot(b2Sub(point, p1), d) / dd;
	t = b2ClampFloat(t, 0.0, 1.0);
	let c = b2MulAdd(p1, t, d);

	// Is query point within radius around closest point?
	return b2DistanceSquared(point, c) <= rr;
}

/// Test a point for overlap with a convex polygon in local space
export function b2PointInPolygon(shape: b2Polygon, point: b2Vec2): bool {
	let input: b2DistanceInput;
	input.proxyA = b2MakeProxy(shape.vertices, shape.count, 0.0);
	input.proxyB = b2MakePointProxy(point, 0.0);
	input.transform = b2Transform_identity;
	input.useRadii = false;

	let cache: b2SimplexCache;
	let output = b2ComputeShapeDistance(input, cache, null, 0);

	return output.distance <= shape.radius;
}

/// Ray cast versus circle shape in local space.
// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
export function b2CastRayCircle(shape: b2Circle, input: b2RayCastInput): b2CastOutput {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidRay(input));

	let p = shape.center;

	let output: b2CastOutput;

	// Shift ray so circle center is the origin
	let s = b2Sub(input.origin, p);

	let r = shape.radius;
	let rr = r * r;

	let length: b2Vec2;
	let d = b2GetLengthAndNormalize(length.x, input.translation);
	if (length.x == 0.0) {
		// zero length ray

		if (b2LengthSquared(s) < rr) {
			// initial overlap
			output.point = input.origin;
			output.hit = true;
		}

		return output;
	}

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	let t = -b2Dot(s, d);

	// c is the closest point on the line to the origin
	let c = b2MulAdd(s, t, d);

	let cc = b2Dot(c, c);

	if (cc > rr) {
		// closest point is outside the circle
		return output;
	}

	// Pythagoras
	let h = Mathf.sqrt(rr - cc);

	let fraction = t - h;

	if (fraction < 0.0 || input.maxFraction * length.x < fraction) {
		// intersection is point outside the range of the ray segment

		if (b2LengthSquared(s) < rr) {
			// initial overlap
			output.point = input.origin;
			output.hit = true;
		}

		return output;
	}

	// hit point relative to center
	let hitPoint = b2MulAdd(s, fraction, d);

	output.fraction = fraction / length.x;
	output.normal = b2Normalize(hitPoint);
	output.point = b2MulAdd(p, shape.radius, output.normal);
	output.hit = true;

	return output;
}

/// Ray cast versus capsule shape in local space.
export function b2CastRayCapsule(shape: b2Capsule, input: b2RayCastInput): b2CastOutput {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidRay(input));

	let output: b2CastOutput;

	let v1 = shape.center1;
	let v2 = shape.center2;

	let e = b2Sub(v2, v1);

	// x is the capsule length, y the ray length; rasc binds ref only to a struct leaf.
	let lengths: b2Vec2;
	let a = b2GetLengthAndNormalize(lengths.x, e);
	let capsuleLength = lengths.x;

	if (capsuleLength < FLT_EPSILON) {
		// Capsule is really a circle
		let circle = b2Circle(v1, shape.radius);
		return b2CastRayCircle(circle, input);
	}

	let p1 = input.origin;
	let d = input.translation;

	// Ray from capsule start to ray start
	let q = b2Sub(p1, v1);
	let qa = b2Dot(q, a);

	// Vector to ray start that is perpendicular to capsule axis
	let qp = b2MulAdd(q, -qa, a);

	let radius = shape.radius;

	// Does the ray start within the infinite length capsule?
	if (b2Dot(qp, qp) < radius * radius) {
		if (qa < 0.0) {
			// start point behind capsule segment
			let circle = b2Circle(v1, shape.radius);
			return b2CastRayCircle(circle, input);
		}

		if (qa > capsuleLength) {
			// start point ahead of capsule segment
			let circle = b2Circle(v2, shape.radius);
			return b2CastRayCircle(circle, input);
		}

		// ray starts inside capsule -> no hit
		output.point = input.origin;
		output.hit = true;
		return output;
	}

	// Perpendicular to capsule axis, pointing right
	let n = b2Vec2(a.y, -a.x);

	let u = b2GetLengthAndNormalize(lengths.y, d);
	let rayLength = lengths.y;

	// Intersect ray with infinite length capsule
	// v1 + radius * n + s1 * a = p1 + s2 * u
	// v1 - radius * n + s1 * a = p1 + s2 * u

	// s1 * a - s2 * u = b
	// b = q - radius * ap
	// or
	// b = q + radius * ap

	// Cramer's rule [a -u]
	let den = -a.x * u.y + u.x * a.y;
	if (-FLT_EPSILON < den && den < FLT_EPSILON) {
		// Ray is parallel to capsule and outside infinite length capsule
		return output;
	}

	let b1 = b2MulSub(q, radius, n);
	let b2 = b2MulAdd(q, radius, n);

	let invDen: f32 = 1.0 / den;

	// Cramer's rule [a b1]
	let s21 = (a.x * b1.y - b1.x * a.y) * invDen;

	// Cramer's rule [a b2]
	let s22 = (a.x * b2.y - b2.x * a.y) * invDen;

	let s2: f32;
	let b: b2Vec2;
	if (s21 < s22) {
		s2 = s21;
		b = b1;
	} else {
		s2 = s22;
		b = b2;
		n = b2Neg(n);
	}

	if (s2 < 0.0 || input.maxFraction * rayLength < s2) {
		return output;
	}

	// Cramer's rule [b -u]
	let s1 = (-b.x * u.y + u.x * b.y) * invDen;

	if (s1 < 0.0) {
		// ray passes behind capsule segment
		let circle = b2Circle(v1, shape.radius);
		return b2CastRayCircle(circle, input);
	} else if (capsuleLength < s1) {
		// ray passes ahead of capsule segment
		let circle = b2Circle(v2, shape.radius);
		return b2CastRayCircle(circle, input);
	} else {
		// ray hits capsule side
		output.fraction = s2 / rayLength;
		output.point = b2Add(b2Lerp(v1, v2, s1 / capsuleLength), b2MulSV(shape.radius, n));
		output.normal = n;
		output.hit = true;
		return output;
	}
}

/// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits
/// from the left side being treated as a miss.
// Ray vs line segment
export function b2CastRaySegment(shape: b2Segment, input: b2RayCastInput, oneSided: bool): b2CastOutput {
	if (oneSided) {
		// Skip left-side collision
		let offset = b2Cross(b2Sub(input.origin, shape.point1), b2Sub(shape.point2, shape.point1));
		if (offset < 0.0) {
			let output: b2CastOutput;
			return output;
		}
	}

	// Put the ray into the edge's frame of reference.
	let p1 = input.origin;
	let d = input.translation;

	let v1 = shape.point1;
	let v2 = shape.point2;
	let e = b2Sub(v2, v1);

	let output: b2CastOutput;

	let length: b2Vec2;
	let eUnit = b2GetLengthAndNormalize(length.x, e);
	if (length.x == 0.0) {
		return output;
	}

	// Normal points to the right, looking from v1 towards v2
	let normal = b2RightPerp(eUnit);

	// Intersect ray with infinite segment using normal
	// Similar to intersecting a ray with an infinite plane
	// p = p1 + t * d
	// dot(normal, p - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	let numerator = b2Dot(normal, b2Sub(v1, p1));
	let denominator = b2Dot(normal, d);

	if (denominator == 0.0) {
		// parallel
		return output;
	}

	let t = numerator / denominator;
	if (t < 0.0 || input.maxFraction < t) {
		// out of ray range
		return output;
	}

	// Intersection point on infinite segment
	let p = b2MulAdd(p1, t, d);

	// Compute position of p along segment
	// p = v1 + s * e
	// s = dot(p - v1, e) / dot(e, e)

	let s = b2Dot(b2Sub(p, v1), eUnit);
	if (s < 0.0 || length.x < s) {
		// out of segment range
		return output;
	}

	if (numerator > 0.0) {
		normal = b2Neg(normal);
	}

	output.fraction = t;
	output.point = p;
	output.normal = normal;
	output.hit = true;

	return output;
}

/// Ray cast versus polygon shape in local space.
export function b2CastRayPolygon(shape: b2Polygon, input: b2RayCastInput): b2CastOutput {
	if (B2_ASSERT_ENABLED) B2_ASSERT(b2IsValidRay(input));

	if (shape.radius == 0.0) {
		// Shift all math to first vertex since the polygon may be far
		// from the origin.
		let base = shape.vertices[0];

		let p1 = b2Sub(input.origin, base);
		let d = input.translation;

		let lower: f32 = 0.0;
		let upper = input.maxFraction;

		let index = -1;

		let output: b2CastOutput;

		for (let edgeIndex = 0; edgeIndex < shape.count; ++edgeIndex) {
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			let vertex = b2Sub(shape.vertices[edgeIndex], base);
			let numerator = b2Dot(shape.normals[edgeIndex], b2Sub(vertex, p1));
			let denominator = b2Dot(shape.normals[edgeIndex], d);

			if (denominator == 0.0) {
				// Parallel and runs outside edge
				if (numerator < 0.0) {
					return output;
				}
			} else {
				// Note: we want this predicate without division:
				// lower < numerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < numerator / denominator <==> denominator * lower > numerator.
				if (denominator < 0.0 && numerator < lower * denominator) {
					// Increase lower.
					// The segment enters this half-space.
					lower = numerator / denominator;
					index = edgeIndex;
				} else if (denominator > 0.0 && numerator < upper * denominator) {
					// Decrease upper.
					// The segment exits this half-space.
					upper = numerator / denominator;
				}
			}

			if (upper < lower) {
				// Ray misses
				return output;
			}
		}

		if (B2_ASSERT_ENABLED) B2_ASSERT(0.0 <= lower && lower <= input.maxFraction);

		if (index >= 0) {
			output.fraction = lower;
			output.normal = shape.normals[index];
			output.point = b2MulAdd(input.origin, lower, d);
			output.hit = true;
		} else {
			// initial overlap
			output.point = input.origin;
			output.hit = true;
		}

		return output;
	}

	let castInput: b2ShapeCastPairInput;
	castInput.proxyA = b2MakeProxy(shape.vertices, shape.count, shape.radius);
	castInput.proxyB = b2MakePointProxy(input.origin, 0.0);
	castInput.transform = b2Transform_identity;
	castInput.translationB = input.translation;
	castInput.maxFraction = input.maxFraction;
	castInput.canEncroach = false;
	return b2CastShape(castInput);
}

/// Shape cast versus a circle.
export function b2CastShapeCircle(shape: b2Circle, input: b2ShapeCastInput): b2CastOutput {
	let pairInput: b2ShapeCastPairInput;
	pairInput.proxyA = b2MakePointProxy(shape.center, shape.radius);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b2Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	return b2CastShape(pairInput);
}

/// Shape cast versus a capsule.
export function b2CastShapeCapsule(shape: b2Capsule, input: b2ShapeCastInput): b2CastOutput {
	let pairInput: b2ShapeCastPairInput;
	pairInput.proxyA = b2MakeSegmentProxy(shape.center1, shape.center2, shape.radius);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b2Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	return b2CastShape(pairInput);
}

/// Shape cast versus a line segment.
export function b2CastShapeSegment(shape: b2Segment, input: b2ShapeCastInput): b2CastOutput {
	let pairInput: b2ShapeCastPairInput;
	pairInput.proxyA = b2MakeSegmentProxy(shape.point1, shape.point2, 0.0);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b2Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	return b2CastShape(pairInput);
}

/// Shape cast versus a convex polygon.
export function b2CastShapePolygon(shape: b2Polygon, input: b2ShapeCastInput): b2CastOutput {
	let pairInput: b2ShapeCastPairInput;
	pairInput.proxyA = b2MakeProxy(shape.vertices, shape.count, shape.radius);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b2Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	return b2CastShape(pairInput);
}

export function b2CollideMoverAndCircle(mover: b2Capsule, shape: b2Circle): b2PlaneResult {
	let distanceInput: b2DistanceInput;
	distanceInput.proxyA = b2MakePointProxy(shape.center, 0.0);
	distanceInput.proxyB = b2MakeSegmentProxy(mover.center1, mover.center2, mover.radius);
	distanceInput.transform = b2Transform_identity;
	distanceInput.useRadii = false;

	let totalRadius = mover.radius + shape.radius;

	let cache: b2SimplexCache;
	let distanceOutput = b2ComputeShapeDistance(distanceInput, cache, null, 0);

	let result: b2PlaneResult;
	if (distanceOutput.distance <= totalRadius) {
		result.plane = b2Plane(distanceOutput.normal, totalRadius - distanceOutput.distance);
		result.point = distanceOutput.pointA;
		result.hit = true;
	}

	return result;
}

export function b2CollideMoverAndCapsule(mover: b2Capsule, shape: b2Capsule): b2PlaneResult {
	let distanceInput: b2DistanceInput;
	distanceInput.proxyA = b2MakeSegmentProxy(shape.center1, shape.center2, 0.0);
	distanceInput.proxyB = b2MakeSegmentProxy(mover.center1, mover.center2, mover.radius);
	distanceInput.transform = b2Transform_identity;
	distanceInput.useRadii = false;

	let totalRadius = mover.radius + shape.radius;

	let cache: b2SimplexCache;
	let distanceOutput = b2ComputeShapeDistance(distanceInput, cache, null, 0);

	let result: b2PlaneResult;
	if (distanceOutput.distance <= totalRadius) {
		result.plane = b2Plane(distanceOutput.normal, totalRadius - distanceOutput.distance);
		result.point = distanceOutput.pointA;
		result.hit = true;
	}

	return result;
}

export function b2CollideMoverAndPolygon(mover: b2Capsule, shape: b2Polygon): b2PlaneResult {
	let distanceInput: b2DistanceInput;
	distanceInput.proxyA = b2MakeProxy(shape.vertices, shape.count, shape.radius);
	distanceInput.proxyB = b2MakeSegmentProxy(mover.center1, mover.center2, mover.radius);
	distanceInput.transform = b2Transform_identity;
	distanceInput.useRadii = false;

	let totalRadius = mover.radius + shape.radius;

	let cache: b2SimplexCache;
	let distanceOutput = b2ComputeShapeDistance(distanceInput, cache, null, 0);

	let result: b2PlaneResult;
	if (distanceOutput.distance <= totalRadius) {
		result.plane = b2Plane(distanceOutput.normal, totalRadius - distanceOutput.distance);
		result.point = distanceOutput.pointA;
		result.hit = true;
	}

	return result;
}

export function b2CollideMoverAndSegment(mover: b2Capsule, shape: b2Segment): b2PlaneResult {
	let distanceInput: b2DistanceInput;
	distanceInput.proxyA = b2MakeSegmentProxy(shape.point1, shape.point2, 0.0);
	distanceInput.proxyB = b2MakeSegmentProxy(mover.center1, mover.center2, mover.radius);
	distanceInput.transform = b2Transform_identity;
	distanceInput.useRadii = false;

	let totalRadius = mover.radius;

	let cache: b2SimplexCache;
	let distanceOutput = b2ComputeShapeDistance(distanceInput, cache, null, 0);

	let result: b2PlaneResult;
	if (distanceOutput.distance <= totalRadius) {
		result.plane = b2Plane(distanceOutput.normal, totalRadius - distanceOutput.distance);
		result.point = distanceOutput.pointA;
		result.hit = true;
	}

	return result;
}
