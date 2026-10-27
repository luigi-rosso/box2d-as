// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/mover.c

import { B2_LINEAR_SLOP } from "./constants";
import { b2AbsFloat, b2ClampFloat, b2Dot, b2MinFloat, b2MulAdd, b2MulSub, b2PlaneSeparation, b2Vec2 } from "./math_functions";
import { b2PlaneSolverResult } from "./collision";

/// Solves the position of a mover that satisfies the given collision planes.
/// @param targetDelta the desired translation from the position used to generate the collision
/// planes
/// @param planes the collision planes
/// @param count the number of collision planes
// C takes a plane pointer; this takes any StaticArray or Array of b2CollisionPlane.
export function b2SolvePlanes<A>(targetDelta: b2Vec2, planes: A, count: i32): b2PlaneSolverResult {
	for (let i = 0; i < count; ++i) {
		planes[i].push = 0.0;
	}

	let delta = targetDelta;
	let tolerance = B2_LINEAR_SLOP();

	let iteration: i32;
	for (iteration = 0; iteration < 20; ++iteration) {
		let totalPush: f32 = 0.0;
		for (let planeIndex = 0; planeIndex < count; ++planeIndex) {
			// Add slop to prevent jitter
			let separation = b2PlaneSeparation(planes[planeIndex].plane, delta) + B2_LINEAR_SLOP();
			// if (separation > 0.0f)
			//{
			//	continue;
			// }

			let push = -separation;

			// Clamp accumulated push
			let accumulatedPush: f32 = planes[planeIndex].push;
			planes[planeIndex].push = b2ClampFloat(accumulatedPush + push, 0.0, planes[planeIndex].pushLimit);
			push = planes[planeIndex].push - accumulatedPush;
			delta = b2MulAdd(delta, push, planes[planeIndex].plane.normal);

			// Track maximum push for convergence
			totalPush += b2AbsFloat(push);
		}

		if (totalPush < tolerance) {
			break;
		}
	}

	let result: b2PlaneSolverResult;
	result.delta = delta;
	result.iterationCount = iteration;
	return result;
}

/// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity set
/// to false are skipped.
// C takes a plane pointer; this takes any StaticArray or Array of b2CollisionPlane.
export function b2ClipVector<A>(vector: b2Vec2, planes: A, count: i32): b2Vec2 {
	let v = vector;

	for (let planeIndex = 0; planeIndex < count; ++planeIndex) {
		if (planes[planeIndex].push == 0.0 || planes[planeIndex].clipVelocity == false) {
			continue;
		}

		let normal: b2Vec2 = planes[planeIndex].plane.normal;
		v = b2MulSub(v, b2MinFloat(0.0, b2Dot(v, normal)), normal);
	}

	return v;
}
