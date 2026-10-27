// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box2d/constants.h

import { b2_lengthUnitsPerMeter } from "./core";
import { B2_PI } from "./math_functions";

/// Used to detect bad values. Positions greater than about 16km will have precision problems in
/// single precision, so 100km as a limit should be fine in all cases.
@inline
export function B2_HUGE(): f32 {
	return (100.0 * 1000.0) * b2_lengthUnitsPerMeter;
}

/// Maximum parallel workers. Used for some fixed size arrays.
export const B2_MAX_WORKERS: i32 = 32;

/// Maximum number of tasks queued per world step. b2EnqueueTaskCallbackFcn will never be called
/// more than this per world step.
export const B2_MAX_TASKS: i32 = 256;

/// Maximum number of colors in the constraint graph. Constraints that cannot find a color are added
/// to the overflow set, which is solved single-threaded. The compound barrel benchmark has minor
/// overflow with 24 colors
export const B2_GRAPH_COLOR_COUNT: i32 = 24;

/// A small length used as a collision and constraint tolerance. Usually it is chosen to be
/// numerically significant, but visually insignificant. In meters. Normally this is 0.5cm.
/// @warning modifying this can have a significant impact on stability
@inline
export function B2_LINEAR_SLOP(): f32 {
	return 0.005 * b2_lengthUnitsPerMeter;
}

/// Maximum number of simultaneous worlds that can be allocated
export const B2_MAX_WORLDS: i32 = 128;

/// Maximum length of the body name. Can be 0 if you don't need names.
export const B2_NAME_LENGTH: i32 = 10;

/// The maximum rotation of a body per time step. This limit is very large and is used to prevent
/// numerical problems. You shouldn't need to adjust this.
/// @warning increasing this to 0.5f * B2_PI or greater will break continuous collision.
export const B2_MAX_ROTATION: f32 = 0.25 * B2_PI;

/// Box2D uses limited speculative collision. This reduces jitter. Normally this is 2cm.
/// @warning modifying this can have a significant impact on performance and stability
@inline
export function B2_SPECULATIVE_DISTANCE(): f32 {
	return 4.0 * B2_LINEAR_SLOP();
}

/// The default contact recycling distance.
@inline
export function B2_CONTACT_RECYCLE_DISTANCE(): f32 {
	return 10.0 * B2_LINEAR_SLOP();
}

/// The default contact recycling world angle threshold. 0.98 ~= 11.5 degrees
export const B2_CONTACT_RECYCLE_COS_ANGLE: f32 = 0.98;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies to move by a small amount
/// without triggering a tree adjustment. This is in meters. Normally this is 5cm.
/// @warning modifying this can have a significant impact on performance
@inline
export function B2_MAX_AABB_MARGIN(): f32 {
	return 0.05 * b2_lengthUnitsPerMeter;
}

/// For small objects the margin is limited to this fraction times the maximum extent
export const B2_AABB_MARGIN_FRACTION: f32 = 0.125;

/// The time that a body must be still before it will go to sleep. In seconds.
export const B2_TIME_TO_SLEEP: f32 = 0.5;

/// The maximum number of restitution iterations. Needed to avoid a solver overflow.
export const B2_MAX_RESTITUTION_ITERATIONS: i32 = 63;
