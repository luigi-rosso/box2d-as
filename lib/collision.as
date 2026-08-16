// Ported from box2d include/box2d/collision.h — the shape types Phase A
// needs (hull, primitive shapes, proxy, cast IO, mass data) plus the Phase B
// distance/manifold/tree IO types. Fixed C arrays become StaticArrays
// pre-filled with fresh instances so slots are never null; counts gate the
// live range exactly as in C.

import { b2Vec2, b2Rot, b2Transform, b2Plane } from "./math_functions";

// The maximum number of vertices on a convex polygon.
export const B2_MAX_POLYGON_VERTICES: i32 = 8;

export function makeVecArray(n: i32): StaticArray<b2Vec2> {
    let a = new StaticArray<b2Vec2>(n);
    for (let i = 0; i < n; i++) {
        a[i] = new b2Vec2();
    }
    return a;
}

// A convex hull. Used to create convex polygons.
export class b2Hull {
    points: StaticArray<b2Vec2> = makeVecArray(B2_MAX_POLYGON_VERTICES);
    count: i32 = 0;
}

// Low level ray cast input data
export class b2RayCastInput {
    origin: b2Vec2 = new b2Vec2();
    translation: b2Vec2 = new b2Vec2();
    maxFraction: f32 = 0;
}

// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
export class b2ShapeProxy {
    points: StaticArray<b2Vec2> = makeVecArray(B2_MAX_POLYGON_VERTICES);
    count: i32 = 0;
    radius: f32 = 0;
}

// Low level shape cast input in generic form.
export class b2ShapeCastInput {
    proxy: b2ShapeProxy = new b2ShapeProxy();
    translation: b2Vec2 = new b2Vec2();
    maxFraction: f32 = 0;
    canEncroach: bool = false;
}

// Low level ray cast or shape-cast output data.
export class b2CastOutput {
    normal: b2Vec2 = new b2Vec2();
    point: b2Vec2 = new b2Vec2();
    fraction: f32 = 0;
    iterations: i32 = 0;
    hit: bool = false;
}

// This holds the mass data computed for a shape.
export class b2MassData {
    mass: f32 = 0;
    center: b2Vec2 = new b2Vec2();
    rotationalInertia: f32 = 0;
}

// A solid circle
export class b2Circle {
    center: b2Vec2;
    radius: f32;

    constructor(center: b2Vec2 = new b2Vec2(), radius: f32 = 0) {
        this.center = center;
        this.radius = radius;
    }
}

// A solid capsule: two semicircles connected by a rectangle.
export class b2Capsule {
    center1: b2Vec2;
    center2: b2Vec2;
    radius: f32;

    constructor(center1: b2Vec2 = new b2Vec2(), center2: b2Vec2 = new b2Vec2(), radius: f32 = 0) {
        this.center1 = center1;
        this.center2 = center2;
        this.radius = radius;
    }
}

// A solid convex polygon. Use b2MakePolygon or b2MakeBox, do not fill by hand.
export class b2Polygon {
    vertices: StaticArray<b2Vec2> = makeVecArray(B2_MAX_POLYGON_VERTICES);
    normals: StaticArray<b2Vec2> = makeVecArray(B2_MAX_POLYGON_VERTICES);
    centroid: b2Vec2 = new b2Vec2();
    radius: f32 = 0;
    count: i32 = 0;
}

// A line segment with two-sided collision.
export class b2Segment {
    point1: b2Vec2;
    point2: b2Vec2;

    constructor(point1: b2Vec2 = new b2Vec2(), point2: b2Vec2 = new b2Vec2()) {
        this.point1 = point1;
        this.point2 = point2;
    }
}

// A line segment with one-sided collision, owned by a chain shape.
export class b2ChainSegment {
    ghost1: b2Vec2 = new b2Vec2();
    segment: b2Segment = new b2Segment();
    ghost2: b2Vec2 = new b2Vec2();
    chainId: i32 = 0;
}

// Result returned by the mover collide functions.
export class b2PlaneResult {
    plane: b2Plane = new b2Plane();
    point: b2Vec2 = new b2Vec2();
    hit: bool = false;
}

// Result of computing the distance between two line segments.
export class b2SegmentDistanceResult {
    closest1: b2Vec2 = new b2Vec2();
    closest2: b2Vec2 = new b2Vec2();
    fraction1: f32 = 0;
    fraction2: f32 = 0;
    distanceSquared: f32 = 0;
}

// Used to warm start the GJK simplex.
export class b2SimplexCache {
    count: u16 = 0;
    indexA: StaticArray<u8> = new StaticArray<u8>(3);
    indexB: StaticArray<u8> = new StaticArray<u8>(3);

    clone(): b2SimplexCache {
        let c = new b2SimplexCache();
        c.count = this.count;
        for (let i = 0; i < 3; i++) {
            c.indexA[i] = this.indexA[i];
            c.indexB[i] = this.indexB[i];
        }
        return c;
    }
}

// Input for b2ShapeDistance.
export class b2DistanceInput {
    proxyA: b2ShapeProxy = new b2ShapeProxy();
    proxyB: b2ShapeProxy = new b2ShapeProxy();
    transformA: b2Transform = new b2Transform();
    transformB: b2Transform = new b2Transform();
    useRadii: bool = false;
}

// Output for b2ShapeDistance.
export class b2DistanceOutput {
    pointA: b2Vec2 = new b2Vec2();
    pointB: b2Vec2 = new b2Vec2();
    normal: b2Vec2 = new b2Vec2();
    distance: f32 = 0;
    iterations: i32 = 0;
    simplexCount: i32 = 0;
}

// Simplex vertex; the simplex holds three fixed slots so vertex copies keep
// C's value semantics (copyFrom, never slot reassignment).
export class b2SimplexVertex {
    wA: b2Vec2 = new b2Vec2();
    wB: b2Vec2 = new b2Vec2();
    w: b2Vec2 = new b2Vec2();
    a: f32 = 0;
    indexA: i32 = 0;
    indexB: i32 = 0;

    copyFrom(v: b2SimplexVertex): void {
        this.wA = v.wA.clone();
        this.wB = v.wB.clone();
        this.w = v.w.clone();
        this.a = v.a;
        this.indexA = v.indexA;
        this.indexB = v.indexB;
    }
}

// Simplex from the GJK algorithm.
export class b2Simplex {
    v1: b2SimplexVertex = new b2SimplexVertex();
    v2: b2SimplexVertex = new b2SimplexVertex();
    v3: b2SimplexVertex = new b2SimplexVertex();
    count: i32 = 0;
}

// Input parameters for b2ShapeCast.
export class b2ShapeCastPairInput {
    proxyA: b2ShapeProxy = new b2ShapeProxy();
    proxyB: b2ShapeProxy = new b2ShapeProxy();
    transformA: b2Transform = new b2Transform();
    transformB: b2Transform = new b2Transform();
    translationB: b2Vec2 = new b2Vec2();
    maxFraction: f32 = 0;
    canEncroach: bool = false;
}

// Describes the motion of a body/shape for TOI computation.
export class b2Sweep {
    localCenter: b2Vec2 = new b2Vec2();
    c1: b2Vec2 = new b2Vec2();
    c2: b2Vec2 = new b2Vec2();
    q1: b2Rot = new b2Rot();
    q2: b2Rot = new b2Rot();

    clone(): b2Sweep {
        let s = new b2Sweep();
        s.localCenter = this.localCenter.clone();
        s.c1 = this.c1.clone();
        s.c2 = this.c2.clone();
        s.q1 = this.q1.clone();
        s.q2 = this.q2.clone();
        return s;
    }
}

// Time of impact input.
export class b2TOIInput {
    proxyA: b2ShapeProxy = new b2ShapeProxy();
    proxyB: b2ShapeProxy = new b2ShapeProxy();
    sweepA: b2Sweep = new b2Sweep();
    sweepB: b2Sweep = new b2Sweep();
    maxFraction: f32 = 0;
}

// enum b2TOIState
export const b2_toiStateUnknown: i32 = 0;
export const b2_toiStateFailed: i32 = 1;
export const b2_toiStateOverlapped: i32 = 2;
export const b2_toiStateHit: i32 = 3;
export const b2_toiStateSeparated: i32 = 4;

// Time of impact output.
export class b2TOIOutput {
    state: i32 = b2_toiStateUnknown;
    point: b2Vec2 = new b2Vec2();
    normal: b2Vec2 = new b2Vec2();
    fraction: f32 = 0;
}

// A manifold point is a contact point belonging to a contact manifold.
export class b2ManifoldPoint {
    point: b2Vec2 = new b2Vec2();
    anchorA: b2Vec2 = new b2Vec2();
    anchorB: b2Vec2 = new b2Vec2();
    separation: f32 = 0;
    normalImpulse: f32 = 0;
    tangentImpulse: f32 = 0;
    totalNormalImpulse: f32 = 0;
    normalVelocity: f32 = 0;
    id: u16 = 0;
    persisted: bool = false;
}

function makeManifoldPoints(): StaticArray<b2ManifoldPoint> {
    let a = new StaticArray<b2ManifoldPoint>(2);
    a[0] = new b2ManifoldPoint();
    a[1] = new b2ManifoldPoint();
    return a;
}

// A contact manifold describes the contact points between colliding shapes.
export class b2Manifold {
    normal: b2Vec2 = new b2Vec2();
    rollingImpulse: f32 = 0;
    points: StaticArray<b2ManifoldPoint> = makeManifoldPoints();
    pointCount: i32 = 0;
}

// Performance results returned by dynamic tree queries.
export class b2TreeStats {
    nodeVisits: i32 = 0;
    leafVisits: i32 = 0;
}
