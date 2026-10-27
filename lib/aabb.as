// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/aabb.h and src/aabb.c

import { FLT_EPSILON, FLT_MAX } from "./core";
import {
	b2AABB,
	b2Abs,
	b2IsValidVec2,
	b2Lerp,
	b2MinFloat,
	b2Pos,
	b2RoundDownFloat,
	b2RoundUpFloat,
	b2Sub,
	b2Vec2,
	b2Vec2_zero,
} from "./math_functions";
import { b2CastOutput } from "./collision";

// aabb.h

// Get surface area of an AABB (the perimeter length)
@inline export function b2Perimeter(a: b2AABB): f32 {
	let wx = a.upperBound.x - a.lowerBound.x;
	let wy = a.upperBound.y - a.lowerBound.y;
	return 2.0 * (wx + wy);
}

/// Enlarge a to contain b
/// @return true if the AABB grew
export function b2EnlargeAABB(a: ref b2AABB, b: b2AABB): bool {
	let changed = false;
	if (b.lowerBound.x < a.lowerBound.x) {
		a.lowerBound.x = b.lowerBound.x;
		changed = true;
	}

	if (b.lowerBound.y < a.lowerBound.y) {
		a.lowerBound.y = b.lowerBound.y;
		changed = true;
	}

	if (a.upperBound.x < b.upperBound.x) {
		a.upperBound.x = b.upperBound.x;
		changed = true;
	}

	if (a.upperBound.y < b.upperBound.y) {
		a.upperBound.y = b.upperBound.y;
		changed = true;
	}

	return changed;
}

// Translate a relative AABB into world space, rounding outward so the float box always contains
// the true box far from the origin where a float coordinate cannot resolve the shape extent.
// Float ULP at 1e8 dwarfs the AABB margin, plain truncation could clip a shape out of its own
// box. The broadphase pair order rides on the deterministic directed rounding.
export function b2OffsetAABB(box: b2AABB, origin: b2Pos): b2AABB {
	let result: b2AABB;
	result.lowerBound.x = b2RoundDownFloat(<f64>origin.x + <f64>box.lowerBound.x);
	result.lowerBound.y = b2RoundDownFloat(<f64>origin.y + <f64>box.lowerBound.y);
	result.upperBound.x = b2RoundUpFloat(<f64>origin.x + <f64>box.upperBound.x);
	result.upperBound.y = b2RoundUpFloat(<f64>origin.y + <f64>box.upperBound.y);
	return result;
}

// aabb.c

export function b2IsValidAABB(a: b2AABB): bool {
	let d = b2Sub(a.upperBound, a.lowerBound);
	let valid = d.x >= 0.0 && d.y >= 0.0;
	valid = valid && b2IsValidVec2(a.lowerBound) && b2IsValidVec2(a.upperBound);
	return valid;
}

// From Real-time Collision Detection, p179.
export function b2AABB_RayCast(a: b2AABB, p1: b2Vec2, p2: b2Vec2): b2CastOutput {
	// Radius not handled
	let output = b2CastOutput();

	let tMin = -FLT_MAX;
	let tMax = FLT_MAX;

	let p = p1;
	let d = b2Sub(p2, p1);
	let absD = b2Abs(d);

	let normal = b2Vec2_zero;

	// x-coordinate
	if (absD.x < FLT_EPSILON) {
		// parallel
		if (p.x < a.lowerBound.x || a.upperBound.x < p.x) {
			return output;
		}
	} else {
		let inv_d: f32 = 1.0 / d.x;
		let t1 = (a.lowerBound.x - p.x) * inv_d;
		let t2 = (a.upperBound.x - p.x) * inv_d;

		// Sign of the normal vector.
		let s: f32 = -1.0;

		if (t1 > t2) {
			let tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0;
		}

		// Push the min up
		if (t1 > tMin) {
			normal.y = 0.0;
			normal.x = s;
			tMin = t1;
		}

		// Pull the max down
		tMax = b2MinFloat(tMax, t2);

		if (tMin > tMax) {
			return output;
		}
	}

	// y-coordinate
	if (absD.y < FLT_EPSILON) {
		// parallel
		if (p.y < a.lowerBound.y || a.upperBound.y < p.y) {
			return output;
		}
	} else {
		let inv_d: f32 = 1.0 / d.y;
		let t1 = (a.lowerBound.y - p.y) * inv_d;
		let t2 = (a.upperBound.y - p.y) * inv_d;

		// Sign of the normal vector.
		let s: f32 = -1.0;

		if (t1 > t2) {
			let tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0;
		}

		// Push the min up
		if (t1 > tMin) {
			normal.x = 0.0;
			normal.y = s;
			tMin = t1;
		}

		// Pull the max down
		tMax = b2MinFloat(tMax, t2);

		if (tMin > tMax) {
			return output;
		}
	}

	// Does the ray start inside the box?
	if (tMin < 0.0) {
		return output;
	}

	// Does the ray intersect beyond the segment length?
	if (1.0 < tMin) {
		return output;
	}

	// Intersection.
	output.fraction = tMin;
	output.normal = normal;
	output.point = b2Lerp(p1, p2, tMin);
	output.hit = true;
	return output;
}
