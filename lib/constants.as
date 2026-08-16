// Ported from box2d src/constants.h. The macros that scale with
// b2_lengthUnitsPerMeter become functions; fixed values stay consts.

import { B2_PI } from "./math_functions";
import { b2_lengthUnitsPerMeter } from "./core";

// Used to detect bad values. Positions greater than about 16km will have precision
// problems, so 100km as a limit should be fine in all cases.
export function B2_HUGE(): f32 {
    return <f32>100000.0 * b2_lengthUnitsPerMeter;
}

// Maximum parallel workers. Used to size some static arrays.
export const B2_MAX_WORKERS: i32 = 64;

// Maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
export const B2_GRAPH_COLOR_COUNT: i32 = 24;

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
export function B2_LINEAR_SLOP(): f32 {
    return <f32>0.005 * b2_lengthUnitsPerMeter;
}

// Maximum number of simultaneous worlds that can be allocated
export const B2_MAX_WORLDS: i32 = 128;

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
export const B2_MAX_ROTATION: f32 = <f32>0.25 * B2_PI;

// Box2D uses limited speculative collision. This reduces jitter.
export function B2_SPECULATIVE_DISTANCE(): f32 {
    return <f32>4.0 * B2_LINEAR_SLOP();
}

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment. This is in meters.
export function B2_AABB_MARGIN(): f32 {
    return <f32>0.05 * b2_lengthUnitsPerMeter;
}

// The time that a body must be still before it will go to sleep. In seconds.
export const B2_TIME_TO_SLEEP: f32 = 0.5;

// enum b2TreeNodeFlags
export const b2_allocatedNode: u32 = 0x0001;
export const b2_enlargedNode: u32 = 0x0002;
export const b2_leafNode: u32 = 0x0004;
