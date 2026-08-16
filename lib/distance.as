// Ported from box2d src/distance.c: GJK distance, conservative-advancement
// shape cast, and the TOI machinery. The debug simplex capture (NDEBUG-only
// in C) is omitted; output.simplexCount is always 0 like the reference build.

import {
    b2Vec2, b2Rot, b2Transform,
    b2Add, b2Sub, b2Neg, b2Dot, b2Cross, b2CrossSV, b2CrossVS, b2MulAdd, b2MulSub, b2MulSV, b2Lerp,
    b2Normalize, b2Distance, b2DistanceSquared, b2ClampFloat, b2MaxFloat, b2MinInt, b2AbsFloat,
    b2NormalizeRot, b2RotateVector, b2InvRotateVector, b2TransformPoint, b2InvMulTransforms,
    b2IsNormalized, b2IsNormalizedRot, b2Vec2_zero, FLT_EPSILON,
} from "./math_functions";
import {
    b2ShapeProxy, b2SegmentDistanceResult, b2SimplexCache, b2DistanceInput, b2DistanceOutput,
    b2Simplex, b2SimplexVertex, b2ShapeCastPairInput, b2CastOutput, b2Sweep, b2TOIInput, b2TOIOutput,
    b2_toiStateUnknown, b2_toiStateFailed, b2_toiStateOverlapped, b2_toiStateHit, b2_toiStateSeparated,
    B2_MAX_POLYGON_VERTICES,
} from "./collision";
import { B2_LINEAR_SLOP } from "./constants";

export function b2GetSweepTransform(sweep: b2Sweep, time: f32): b2Transform {
    let xf = new b2Transform();
    xf.p = b2Add(b2MulSV(<f32>1.0 - time, sweep.c1), b2MulSV(time, sweep.c2));

    let q = new b2Rot(
        (<f32>1.0 - time) * sweep.q1.c + time * sweep.q2.c,
        (<f32>1.0 - time) * sweep.q1.s + time * sweep.q2.s);

    xf.q = b2NormalizeRot(q);

    // Shift to origin
    xf.p = b2Sub(xf.p, b2RotateVector(xf.q, sweep.localCenter));
    return xf;
}

// Follows Ericson 5.1.9 Closest Points of Two Line Segments
export function b2SegmentDistance(p1: b2Vec2, q1: b2Vec2, p2: b2Vec2, q2: b2Vec2): b2SegmentDistanceResult {
    let result = new b2SegmentDistanceResult();

    let d1 = b2Sub(q1, p1);
    let d2 = b2Sub(q2, p2);
    let r = b2Sub(p1, p2);
    let dd1: f32 = b2Dot(d1, d1);
    let dd2: f32 = b2Dot(d2, d2);
    let rd1: f32 = b2Dot(r, d1);
    let rd2: f32 = b2Dot(r, d2);

    let epsSqr: f32 = FLT_EPSILON * FLT_EPSILON;

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
        let d12: f32 = b2Dot(d1, d2);

        let denominator: f32 = dd1 * dd2 - d12 * d12;

        // Fraction on segment 1
        let f1: f32 = 0.0;
        if (denominator != 0.0) {
            // not parallel
            f1 = b2ClampFloat((d12 * rd2 - rd1 * dd2) / denominator, 0.0, 1.0);
        }

        // Compute point on segment 2 closest to p1 + f1 * d1
        let f2: f32 = (d12 * f1 + rd2) / dd2;

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

export function b2MakeProxy(points: StaticArray<b2Vec2>, count: i32, radius: f32): b2ShapeProxy {
    count = b2MinInt(count, B2_MAX_POLYGON_VERTICES);
    let proxy = new b2ShapeProxy();
    for (let i = 0; i < count; ++i) {
        proxy.points[i] = points[i].clone();
    }
    proxy.count = count;
    proxy.radius = radius;
    return proxy;
}

export function b2MakeOffsetProxy(points: StaticArray<b2Vec2>, count: i32, radius: f32,
    position: b2Vec2, rotation: b2Rot): b2ShapeProxy {
    count = b2MinInt(count, B2_MAX_POLYGON_VERTICES);
    let transform = new b2Transform(position, rotation);
    let proxy = new b2ShapeProxy();
    for (let i = 0; i < count; ++i) {
        proxy.points[i] = b2TransformPoint(transform, points[i]);
    }
    proxy.count = count;
    proxy.radius = radius;
    return proxy;
}

function b2Weight2(a1: f32, w1: b2Vec2, a2: f32, w2: b2Vec2): b2Vec2 {
    return new b2Vec2(a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y);
}

function b2Weight3(a1: f32, w1: b2Vec2, a2: f32, w2: b2Vec2, a3: f32, w3: b2Vec2): b2Vec2 {
    return new b2Vec2(a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y);
}

function b2FindSupport(proxy: b2ShapeProxy, direction: b2Vec2): i32 {
    let points = proxy.points;
    let count = proxy.count;

    let bestIndex = 0;
    let bestValue: f32 = b2Dot(unchecked(points[0]), direction);
    for (let i = 1; i < count; ++i) {
        let value: f32 = b2Dot(unchecked(points[i]), direction);
        if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
        }
    }

    return bestIndex;
}

function simplexVertex(s: b2Simplex, i: i32): b2SimplexVertex {
    if (i == 0) {
        return s.v1;
    }
    if (i == 1) {
        return s.v2;
    }
    return s.v3;
}

function b2MakeSimplexFromCache(cache: b2SimplexCache, proxyA: b2ShapeProxy, proxyB: b2ShapeProxy): b2Simplex {
    assert(cache.count <= 3);
    let s = new b2Simplex();

    // Copy data from cache.
    s.count = <i32>cache.count;

    for (let i = 0; i < s.count; ++i) {
        let v = simplexVertex(s, i);
        v.indexA = <i32>cache.indexA[i];
        v.indexB = <i32>cache.indexB[i];
        v.wA = proxyA.points[v.indexA].clone();
        v.wB = proxyB.points[v.indexB].clone();
        v.w = b2Sub(v.wA, v.wB);

        // invalid
        v.a = -1.0;
    }

    // If the cache is empty or invalid ...
    if (s.count == 0) {
        let v = s.v1;
        v.indexA = 0;
        v.indexB = 0;
        v.wA = proxyA.points[0].clone();
        v.wB = proxyB.points[0].clone();
        v.w = b2Sub(v.wA, v.wB);
        v.a = 1.0;
        s.count = 1;
    }

    return s;
}

function b2MakeSimplexCache(cache: b2SimplexCache, simplex: b2Simplex): void {
    cache.count = <u16>simplex.count;
    for (let i = 0; i < simplex.count; ++i) {
        let v = simplexVertex(simplex, i);
        cache.indexA[i] = <u8>v.indexA;
        cache.indexB[i] = <u8>v.indexB;
    }
}

// C writes both witness points through pointers; the pair travels in a class.
class b2WitnessPoints {
    a: b2Vec2 = new b2Vec2();
    b: b2Vec2 = new b2Vec2();
}

function b2ComputeWitnessPoints(s: b2Simplex): b2WitnessPoints {
    let out = new b2WitnessPoints();
    switch (s.count) {
        case 1:
            out.a = s.v1.wA.clone();
            out.b = s.v1.wB.clone();
            break;

        case 2:
            out.a = b2Weight2(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA);
            out.b = b2Weight2(s.v1.a, s.v1.wB, s.v2.a, s.v2.wB);
            break;

        case 3:
            out.a = b2Weight3(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA, s.v3.a, s.v3.wA);
            // todo why are these not equal?
            out.b = out.a.clone();
            break;

        default:
            assert(false);
            break;
    }
    return out;
}

// Solve a line segment using barycentric coordinates; returns a vector that
// points towards the origin. See the C source for the derivation.
function b2SolveSimplex2(s: b2Simplex): b2Vec2 {
    let w1 = s.v1.w;
    let w2 = s.v2.w;
    let e12 = b2Sub(w2, w1);

    // w1 region
    let d12_2: f32 = -b2Dot(w1, e12);
    if (d12_2 <= 0.0) {
        // a2 <= 0, so we clamp it to 0
        s.v1.a = 1.0;
        s.count = 1;
        return b2Neg(w1);
    }

    // w2 region
    let d12_1: f32 = b2Dot(w2, e12);
    if (d12_1 <= 0.0) {
        // a1 <= 0, so we clamp it to 0
        s.v2.a = 1.0;
        s.count = 1;
        s.v1.copyFrom(s.v2);
        return b2Neg(w2);
    }

    // Must be in e12 region.
    let inv_d12: f32 = <f32>1.0 / (d12_1 + d12_2);
    s.v1.a = d12_1 * inv_d12;
    s.v2.a = d12_2 * inv_d12;
    s.count = 2;
    return b2CrossSV(b2Cross(b2Add(w1, w2), e12), e12);
}

function b2SolveSimplex3(s: b2Simplex): b2Vec2 {
    let w1 = s.v1.w;
    let w2 = s.v2.w;
    let w3 = s.v3.w;

    // Edge12
    let e12 = b2Sub(w2, w1);
    let w1e12: f32 = b2Dot(w1, e12);
    let w2e12: f32 = b2Dot(w2, e12);
    let d12_1: f32 = w2e12;
    let d12_2: f32 = -w1e12;

    // Edge13
    let e13 = b2Sub(w3, w1);
    let w1e13: f32 = b2Dot(w1, e13);
    let w3e13: f32 = b2Dot(w3, e13);
    let d13_1: f32 = w3e13;
    let d13_2: f32 = -w1e13;

    // Edge23
    let e23 = b2Sub(w3, w2);
    let w2e23: f32 = b2Dot(w2, e23);
    let w3e23: f32 = b2Dot(w3, e23);
    let d23_1: f32 = w3e23;
    let d23_2: f32 = -w2e23;

    // Triangle123
    let n123: f32 = b2Cross(e12, e13);

    let d123_1: f32 = n123 * b2Cross(w2, w3);
    let d123_2: f32 = n123 * b2Cross(w3, w1);
    let d123_3: f32 = n123 * b2Cross(w1, w2);

    // w1 region
    if (d12_2 <= 0.0 && d13_2 <= 0.0) {
        s.v1.a = 1.0;
        s.count = 1;
        return b2Neg(w1);
    }

    // e12
    if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
        let inv_d12: f32 = <f32>1.0 / (d12_1 + d12_2);
        s.v1.a = d12_1 * inv_d12;
        s.v2.a = d12_2 * inv_d12;
        s.count = 2;
        return b2CrossSV(b2Cross(b2Add(w1, w2), e12), e12);
    }

    // e13
    if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
        let inv_d13: f32 = <f32>1.0 / (d13_1 + d13_2);
        s.v1.a = d13_1 * inv_d13;
        s.v3.a = d13_2 * inv_d13;
        s.count = 2;
        s.v2.copyFrom(s.v3);
        return b2CrossSV(b2Cross(b2Add(w1, w3), e13), e13);
    }

    // w2 region
    if (d12_1 <= 0.0 && d23_2 <= 0.0) {
        s.v2.a = 1.0;
        s.count = 1;
        s.v1.copyFrom(s.v2);
        return b2Neg(w2);
    }

    // w3 region
    if (d13_1 <= 0.0 && d23_1 <= 0.0) {
        s.v3.a = 1.0;
        s.count = 1;
        s.v1.copyFrom(s.v3);
        return b2Neg(w3);
    }

    // e23
    if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
        let inv_d23: f32 = <f32>1.0 / (d23_1 + d23_2);
        s.v2.a = d23_1 * inv_d23;
        s.v3.a = d23_2 * inv_d23;
        s.count = 2;
        s.v1.copyFrom(s.v3);
        return b2CrossSV(b2Cross(b2Add(w2, w3), e23), e23);
    }

    // Must be in triangle123
    let inv_d123: f32 = <f32>1.0 / (d123_1 + d123_2 + d123_3);
    s.v1.a = d123_1 * inv_d123;
    s.v2.a = d123_2 * inv_d123;
    s.v3.a = d123_3 * inv_d123;
    s.count = 3;

    // No search direction
    return b2Vec2_zero();
}

let s_saveA = new StaticArray<i32>(3);
let s_saveB = new StaticArray<i32>(3);
let s_localProxyB = new b2ShapeProxy();

// Uses GJK for computing the distance between convex shapes.
// https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf
export function b2ShapeDistance(input: b2DistanceInput, cache: b2SimplexCache): b2DistanceOutput {
    assert(input.proxyA.count > 0 && input.proxyB.count > 0);
    assert(input.proxyA.radius >= 0.0);
    assert(input.proxyB.radius >= 0.0);

    let output = new b2DistanceOutput();

    let proxyA = input.proxyA;

    // Get proxyB in frame A to avoid further transforms in the main loop.
    let localProxyB = s_localProxyB;
    {
        let transform = b2InvMulTransforms(input.transformA, input.transformB);
        localProxyB.count = input.proxyB.count;
        localProxyB.radius = input.proxyB.radius;
        for (let i = 0; i < localProxyB.count; ++i) {
            unchecked(localProxyB.points[i] = b2TransformPoint(transform, input.proxyB.points[i]));
        }
    }

    // Initialize the simplex.
    let simplex = b2MakeSimplexFromCache(cache, proxyA, localProxyB);

    let nonUnitNormal = b2Vec2_zero();

    // These store the vertices of the last simplex so that we can check for duplicates and prevent cycling.
    let saveA = s_saveA;
    let saveB = s_saveB;

    // Main iteration loop. All computations are done in frame A.
    const maxIterations = 20;
    let iteration = 0;
    while (iteration < maxIterations) {
        // Copy simplex so we can identify duplicates.
        let saveCount = simplex.count;
        for (let i = 0; i < saveCount; ++i) {
            let v = simplexVertex(simplex, i);
            unchecked(saveA[i] = v.indexA);
            unchecked(saveB[i] = v.indexB);
        }

        let d = b2Vec2_zero();
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
                assert(false);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.count == 3) {
            // Overlap
            let wp = b2ComputeWitnessPoints(simplex);
            output.pointA = b2TransformPoint(input.transformA, wp.a);
            output.pointB = b2TransformPoint(input.transformA, wp.b);
            return output;
        }

        // Ensure the search direction is numerically fit.
        if (b2Dot(d, d) < FLT_EPSILON * FLT_EPSILON) {
            // The origin is probably contained by a line segment or triangle.
            // Thus the shapes are overlapped. Must return overlap due to invalid normal.
            let wp = b2ComputeWitnessPoints(simplex);
            output.pointA = b2TransformPoint(input.transformA, wp.a);
            output.pointB = b2TransformPoint(input.transformA, wp.b);
            return output;
        }

        // Save the normal
        nonUnitNormal = d;

        // Compute a tentative new simplex vertex using support points.
        let vertex = simplexVertex(simplex, simplex.count);
        vertex.indexA = b2FindSupport(proxyA, d);
        vertex.wA = proxyA.points[vertex.indexA].clone();
        vertex.indexB = b2FindSupport(localProxyB, b2Neg(d));
        vertex.wB = localProxyB.points[vertex.indexB].clone();
        vertex.w = b2Sub(vertex.wA, vertex.wB);

        // Iteration count is equated to the number of support point calls.
        ++iteration;

        // Check for duplicate support points. This is the main termination criteria.
        let duplicate = false;
        for (let i = 0; i < saveCount; ++i) {
            if (vertex.indexA == unchecked(saveA[i]) && vertex.indexB == unchecked(saveB[i])) {
                duplicate = true;
                break;
            }
        }

        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate) {
            break;
        }

        // New vertex is valid and needed.
        simplex.count += 1;
    }

    // Prepare output
    let normal = b2Normalize(nonUnitNormal);
    assert(b2IsNormalized(normal));
    normal = b2RotateVector(input.transformA.q, normal);

    let wp = b2ComputeWitnessPoints(simplex);
    output.normal = normal;
    output.distance = b2Distance(wp.a, wp.b);
    output.pointA = b2TransformPoint(input.transformA, wp.a);
    output.pointB = b2TransformPoint(input.transformA, wp.b);
    output.iterations = iteration;
    output.simplexCount = 0;

    // Cache the simplex
    b2MakeSimplexCache(cache, simplex);

    // Apply radii if requested
    if (input.useRadii) {
        let radiusA: f32 = input.proxyA.radius;
        let radiusB: f32 = input.proxyB.radius;
        output.distance = b2MaxFloat(0.0, output.distance - radiusA - radiusB);

        // Keep closest points on perimeter even if overlapped, this way the points move smoothly.
        output.pointA = b2MulAdd(output.pointA, radiusA, normal);
        output.pointB = b2MulSub(output.pointB, radiusB, normal);
    }

    return output;
}

// Shape cast using conservative advancement
export function b2ShapeCast(input: b2ShapeCastPairInput): b2CastOutput {
    // Compute tolerance
    let linearSlop: f32 = B2_LINEAR_SLOP();
    let totalRadius: f32 = input.proxyA.radius + input.proxyB.radius;
    let target: f32 = b2MaxFloat(linearSlop, totalRadius - linearSlop);
    let tolerance: f32 = <f32>0.25 * linearSlop;

    assert(target > tolerance);

    // Prepare input for distance query
    let cache = new b2SimplexCache();

    let fraction: f32 = 0.0;

    let distanceInput = new b2DistanceInput();
    distanceInput.proxyA = input.proxyA;
    distanceInput.proxyB = input.proxyB;
    distanceInput.transformA = input.transformA.clone();
    distanceInput.transformB = input.transformB.clone();
    distanceInput.useRadii = false;

    let delta2 = input.translationB;
    let output = new b2CastOutput();

    let iteration = 0;
    const maxIterations = 20;

    for (; iteration < maxIterations; ++iteration) {
        output.iterations += 1;

        let distanceOutput = b2ShapeDistance(distanceInput, cache);

        if (distanceOutput.distance < target + tolerance) {
            if (iteration == 0) {
                if (input.canEncroach && distanceOutput.distance > <f32>2.0 * linearSlop) {
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
                assert(distanceOutput.distance > 0.0 && b2IsNormalized(distanceOutput.normal));
                output.fraction = fraction;
                output.point = b2MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
                output.normal = distanceOutput.normal;
                output.hit = true;
                return output;
            }
        }

        assert(distanceOutput.distance > 0.0);
        assert(b2IsNormalized(distanceOutput.normal));

        // Check if shapes are approaching each other
        let denominator: f32 = b2Dot(delta2, distanceOutput.normal);
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

        distanceInput.transformB.p = b2MulAdd(input.transformB.p, fraction, delta2);
    }

    // Failure!
    return output;
}

// enum b2SeparationType
const b2_pointsType: i32 = 0;
const b2_faceAType: i32 = 1;
const b2_faceBType: i32 = 2;

class b2SeparationFunction {
    proxyA: b2ShapeProxy = new b2ShapeProxy();
    proxyB: b2ShapeProxy = new b2ShapeProxy();
    sweepA: b2Sweep = new b2Sweep();
    sweepB: b2Sweep = new b2Sweep();
    localPoint: b2Vec2 = new b2Vec2();
    axis: b2Vec2 = new b2Vec2();
    type: i32 = b2_pointsType;
}

function b2MakeSeparationFunction(cache: b2SimplexCache, proxyA: b2ShapeProxy, sweepA: b2Sweep,
    proxyB: b2ShapeProxy, sweepB: b2Sweep, t1: f32): b2SeparationFunction {
    let f = new b2SeparationFunction();

    f.proxyA = proxyA;
    f.proxyB = proxyB;
    let count = <i32>cache.count;
    assert(0 < count && count < 3);

    f.sweepA = sweepA.clone();
    f.sweepB = sweepB.clone();

    let xfA = b2GetSweepTransform(sweepA, t1);
    let xfB = b2GetSweepTransform(sweepB, t1);

    if (count == 1) {
        f.type = b2_pointsType;
        let localPointA = proxyA.points[cache.indexA[0]];
        let localPointB = proxyB.points[cache.indexB[0]];
        let pointA = b2TransformPoint(xfA, localPointA);
        let pointB = b2TransformPoint(xfB, localPointB);
        f.axis = b2Normalize(b2Sub(pointB, pointA));
        f.localPoint = b2Vec2_zero();
        return f;
    }

    if (cache.indexA[0] == cache.indexA[1]) {
        // Two points on B and one on A.
        f.type = b2_faceBType;
        let localPointB1 = proxyB.points[cache.indexB[0]];
        let localPointB2 = proxyB.points[cache.indexB[1]];

        f.axis = b2CrossVS(b2Sub(localPointB2, localPointB1), 1.0);
        f.axis = b2Normalize(f.axis);
        let normal = b2RotateVector(xfB.q, f.axis);

        f.localPoint = new b2Vec2(<f32>0.5 * (localPointB1.x + localPointB2.x), <f32>0.5 * (localPointB1.y + localPointB2.y));
        let pointB = b2TransformPoint(xfB, f.localPoint);

        let localPointA = proxyA.points[cache.indexA[0]];
        let pointA = b2TransformPoint(xfA, localPointA);

        let s: f32 = b2Dot(b2Sub(pointA, pointB), normal);
        if (s < 0.0) {
            f.axis = b2Neg(f.axis);
        }
        return f;
    }

    // Two points on A and one or two points on B.
    f.type = b2_faceAType;
    let localPointA1 = proxyA.points[cache.indexA[0]];
    let localPointA2 = proxyA.points[cache.indexA[1]];

    f.axis = b2CrossVS(b2Sub(localPointA2, localPointA1), 1.0);
    f.axis = b2Normalize(f.axis);
    let normal = b2RotateVector(xfA.q, f.axis);

    f.localPoint = new b2Vec2(<f32>0.5 * (localPointA1.x + localPointA2.x), <f32>0.5 * (localPointA1.y + localPointA2.y));
    let pointA = b2TransformPoint(xfA, f.localPoint);

    let localPointB = proxyB.points[cache.indexB[0]];
    let pointB = b2TransformPoint(xfB, localPointB);

    let s: f32 = b2Dot(b2Sub(pointB, pointA), normal);
    if (s < 0.0) {
        f.axis = b2Neg(f.axis);
    }
    return f;
}

// C returns the witness indices through pointers; the trio travels in a class.
class b2MinSeparation {
    indexA: i32 = -1;
    indexB: i32 = -1;
    separation: f32 = 0;
}

function b2FindMinSeparation(f: b2SeparationFunction, t: f32): b2MinSeparation {
    let out = new b2MinSeparation();
    let xfA = b2GetSweepTransform(f.sweepA, t);
    let xfB = b2GetSweepTransform(f.sweepB, t);

    switch (f.type) {
        case b2_pointsType: {
            let axisA = b2InvRotateVector(xfA.q, f.axis);
            let axisB = b2InvRotateVector(xfB.q, b2Neg(f.axis));

            out.indexA = b2FindSupport(f.proxyA, axisA);
            out.indexB = b2FindSupport(f.proxyB, axisB);

            let localPointA = f.proxyA.points[out.indexA];
            let localPointB = f.proxyB.points[out.indexB];

            let pointA = b2TransformPoint(xfA, localPointA);
            let pointB = b2TransformPoint(xfB, localPointB);

            out.separation = b2Dot(b2Sub(pointB, pointA), f.axis);
            return out;
        }

        case b2_faceAType: {
            let normal = b2RotateVector(xfA.q, f.axis);
            let pointA = b2TransformPoint(xfA, f.localPoint);

            let axisB = b2InvRotateVector(xfB.q, b2Neg(normal));

            out.indexA = -1;
            out.indexB = b2FindSupport(f.proxyB, axisB);

            let localPointB = f.proxyB.points[out.indexB];
            let pointB = b2TransformPoint(xfB, localPointB);

            out.separation = b2Dot(b2Sub(pointB, pointA), normal);
            return out;
        }

        case b2_faceBType: {
            let normal = b2RotateVector(xfB.q, f.axis);
            let pointB = b2TransformPoint(xfB, f.localPoint);

            let axisA = b2InvRotateVector(xfA.q, b2Neg(normal));

            out.indexB = -1;
            out.indexA = b2FindSupport(f.proxyA, axisA);

            let localPointA = f.proxyA.points[out.indexA];
            let pointA = b2TransformPoint(xfA, localPointA);

            out.separation = b2Dot(b2Sub(pointA, pointB), normal);
            return out;
        }

        default:
            assert(false);
            return out;
    }
}

function b2EvaluateSeparation(f: b2SeparationFunction, indexA: i32, indexB: i32, t: f32): f32 {
    let xfA = b2GetSweepTransform(f.sweepA, t);
    let xfB = b2GetSweepTransform(f.sweepB, t);

    switch (f.type) {
        case b2_pointsType: {
            let localPointA = f.proxyA.points[indexA];
            let localPointB = f.proxyB.points[indexB];

            let pointA = b2TransformPoint(xfA, localPointA);
            let pointB = b2TransformPoint(xfB, localPointB);

            return b2Dot(b2Sub(pointB, pointA), f.axis);
        }

        case b2_faceAType: {
            let normal = b2RotateVector(xfA.q, f.axis);
            let pointA = b2TransformPoint(xfA, f.localPoint);

            let localPointB = f.proxyB.points[indexB];
            let pointB = b2TransformPoint(xfB, localPointB);

            return b2Dot(b2Sub(pointB, pointA), normal);
        }

        case b2_faceBType: {
            let normal = b2RotateVector(xfB.q, f.axis);
            let pointB = b2TransformPoint(xfB, f.localPoint);

            let localPointA = f.proxyA.points[indexA];
            let pointA = b2TransformPoint(xfA, localPointA);

            return b2Dot(b2Sub(pointA, pointB), normal);
        }

        default:
            assert(false);
            return 0.0;
    }
}

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
export function b2TimeOfImpact(input: b2TOIInput): b2TOIOutput {
    let output = new b2TOIOutput();
    output.state = b2_toiStateUnknown;
    output.fraction = input.maxFraction;

    let sweepA = input.sweepA;
    let sweepB = input.sweepB;
    assert(b2IsNormalizedRot(sweepA.q1) && b2IsNormalizedRot(sweepA.q2));
    assert(b2IsNormalizedRot(sweepB.q1) && b2IsNormalizedRot(sweepB.q2));

    let proxyA = input.proxyA;
    let proxyB = input.proxyB;

    let tMax: f32 = input.maxFraction;

    let totalRadius: f32 = proxyA.radius + proxyB.radius;
    let target: f32 = b2MaxFloat(B2_LINEAR_SLOP(), totalRadius - B2_LINEAR_SLOP());
    let tolerance: f32 = <f32>0.25 * B2_LINEAR_SLOP();
    assert(target > tolerance);

    let t1: f32 = 0.0;
    const k_maxIterations = 20;
    let distanceIterations = 0;

    // Prepare input for distance query.
    let cache = new b2SimplexCache();
    let distanceInput = new b2DistanceInput();
    distanceInput.proxyA = input.proxyA;
    distanceInput.proxyB = input.proxyB;
    distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (; ;) {
        let xfA = b2GetSweepTransform(sweepA, t1);
        let xfB = b2GetSweepTransform(sweepB, t1);

        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA = xfA;
        distanceInput.transformB = xfB;
        let distanceOutput = b2ShapeDistance(distanceInput, cache);

        distanceIterations += 1;

        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0.0) {
            // Failure!
            output.state = b2_toiStateOverlapped;
            output.fraction = 0.0;
            break;
        }

        if (distanceOutput.distance <= target + tolerance) {
            // Victory!
            output.state = b2_toiStateHit;
            // Averaged hit point
            let pA = b2MulAdd(distanceOutput.pointA, proxyA.radius, distanceOutput.normal);
            let pB = b2MulAdd(distanceOutput.pointB, -proxyB.radius, distanceOutput.normal);
            output.point = b2Lerp(pA, pB, 0.5);
            output.normal = distanceOutput.normal;
            output.fraction = t1;
            break;
        }

        // Initialize the separating axis.
        let fcn = b2MakeSeparationFunction(cache, proxyA, sweepA, proxyB, sweepB, t1);

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        let done = false;
        let t2: f32 = tMax;
        let pushBackIterations = 0;
        for (; ;) {
            // Find the deepest point at t2. Store the witness point indices.
            let min2 = b2FindMinSeparation(fcn, t2);
            let indexA = min2.indexA;
            let indexB = min2.indexB;
            let s2: f32 = min2.separation;

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
            let s1: f32 = b2EvaluateSeparation(fcn, indexA, indexB, t1);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < target - tolerance) {
                output.state = b2_toiStateFailed;
                output.fraction = t1;
                done = true;
                break;
            }

            // Check for touching
            if (s1 <= target + tolerance) {
                // Victory! t1 should hold the TOI (could be 0.0).
                output.state = b2_toiStateHit;
                // Averaged hit point
                let pA = b2MulAdd(distanceOutput.pointA, proxyA.radius, distanceOutput.normal);
                let pB = b2MulAdd(distanceOutput.pointB, -proxyB.radius, distanceOutput.normal);
                output.point = b2Lerp(pA, pB, 0.5);
                output.normal = distanceOutput.normal;
                output.fraction = t1;
                done = true;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            let rootIterationCount = 0;
            let a1: f32 = t1;
            let a2: f32 = t2;
            for (; ;) {
                // Use a mix of the secant rule and bisection.
                let t: f32;
                if (rootIterationCount & 1) {
                    // Secant rule to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                } else {
                    // Bisection to guarantee progress.
                    t = <f32>0.5 * (a1 + a2);
                }

                rootIterationCount += 1;

                let s: f32 = b2EvaluateSeparation(fcn, indexA, indexB, t);

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
            let pA = b2MulAdd(distanceOutput.pointA, proxyA.radius, distanceOutput.normal);
            let pB = b2MulAdd(distanceOutput.pointB, -proxyB.radius, distanceOutput.normal);
            output.point = b2Lerp(pA, pB, 0.5);
            output.normal = distanceOutput.normal;
            output.fraction = t1;
            break;
        }
    }

    return output;
}
