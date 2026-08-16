// Ported from box2d src/aabb.h/.c.

import {
    b2AABB, b2Vec2, b2Sub, b2Abs, b2Lerp, b2MinFloat, b2IsValidVec2, b2Vec2_zero, FLT_EPSILON, FLT_MAX,
} from "./math_functions";
import { b2CastOutput } from "./collision";

export function b2IsValidAABB(a: b2AABB): bool {
    let d = b2Sub(a.upperBound, a.lowerBound);
    let valid = d.x >= 0.0 && d.y >= 0.0;
    valid = valid && b2IsValidVec2(a.lowerBound) && b2IsValidVec2(a.upperBound);
    return valid;
}

export function b2Perimeter(a: b2AABB): f32 {
    let wx: f32 = a.upperBound.x - a.lowerBound.x;
    let wy: f32 = a.upperBound.y - a.lowerBound.y;
    return <f32>2.0 * (wx + wy);
}

// Enlarge a to contain b; mutates a like the C pointer version.
export function b2EnlargeAABB(a: b2AABB, b: b2AABB): bool {
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

// From Real-time Collision Detection, p179.
export function b2AABB_RayCast(a: b2AABB, p1: b2Vec2, p2: b2Vec2): b2CastOutput {
    // Radius not handled
    let output = new b2CastOutput();

    let tmin: f32 = -FLT_MAX;
    let tmax: f32 = FLT_MAX;

    let p = p1;
    let d = b2Sub(p2, p1);
    let absD = b2Abs(d);

    let normal = b2Vec2_zero();

    // x-coordinate
    if (absD.x < FLT_EPSILON) {
        // parallel
        if (p.x < a.lowerBound.x || a.upperBound.x < p.x) {
            return output;
        }
    } else {
        let inv_d: f32 = <f32>1.0 / d.x;
        let t1: f32 = (a.lowerBound.x - p.x) * inv_d;
        let t2: f32 = (a.upperBound.x - p.x) * inv_d;

        // Sign of the normal vector.
        let s: f32 = -1.0;

        if (t1 > t2) {
            let tmp = t1;
            t1 = t2;
            t2 = tmp;
            s = 1.0;
        }

        // Push the min up
        if (t1 > tmin) {
            normal.y = 0.0;
            normal.x = s;
            tmin = t1;
        }

        // Pull the max down
        tmax = b2MinFloat(tmax, t2);

        if (tmin > tmax) {
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
        let inv_d: f32 = <f32>1.0 / d.y;
        let t1: f32 = (a.lowerBound.y - p.y) * inv_d;
        let t2: f32 = (a.upperBound.y - p.y) * inv_d;

        // Sign of the normal vector.
        let s: f32 = -1.0;

        if (t1 > t2) {
            let tmp = t1;
            t1 = t2;
            t2 = tmp;
            s = 1.0;
        }

        // Push the min up
        if (t1 > tmin) {
            normal.x = 0.0;
            normal.y = s;
            tmin = t1;
        }

        // Pull the max down
        tmax = b2MinFloat(tmax, t2);

        if (tmin > tmax) {
            return output;
        }
    }

    // Does the ray start inside the box?
    if (tmin < 0.0) {
        return output;
    }

    // Does the ray intersect beyond the segment length?
    if (<f32>1.0 < tmin) {
        return output;
    }

    // Intersection.
    output.fraction = tmin;
    output.normal = normal;
    output.point = b2Lerp(p1, p2, tmin);
    output.hit = true;
    return output;
}
