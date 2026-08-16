// Ported from box2d src/geometry.c. Phase B completed the pieces that need
// the GJK distance module: b2PointInPolygon, the rounded-polygon ray cast
// tail, and the b2ShapeCast* wrappers. b2CollideMover* stays deferred; the
// Luau subset does not use the mover.

import {
    b2Vec2, b2Rot, b2Transform, b2AABB, b2Vec2_zero,
    b2Add, b2Sub, b2Neg, b2Dot, b2Cross, b2CrossVS, b2MulAdd, b2MulSub, b2MulSV, b2Min, b2Max, b2Lerp,
    b2Normalize, b2GetLengthAndNormalize, b2Length, b2LengthSquared, b2DistanceSquared, b2ClampFloat,
    b2TransformPoint, b2RotateVector, b2RightPerp, b2IsValidVec2, b2IsValidFloat,
    B2_PI, FLT_EPSILON,
} from "./math_functions";
import {
    b2Hull, b2Polygon, b2Circle, b2Capsule, b2Segment, b2MassData, b2RayCastInput, b2CastOutput,
    b2ShapeCastInput, b2ShapeCastPairInput, b2DistanceInput, b2SimplexCache,
    B2_MAX_POLYGON_VERTICES, makeVecArray,
} from "./collision";
import { b2Transform_identity } from "./math_functions";
import { B2_HUGE } from "./constants";
import { b2ValidateHull } from "./hull";
import { b2MakeProxy, b2ShapeDistance, b2ShapeCast } from "./distance";

export function b2IsValidRay(input: b2RayCastInput): bool {
    let isValid = b2IsValidVec2(input.origin) && b2IsValidVec2(input.translation) &&
        b2IsValidFloat(input.maxFraction) && 0.0 <= input.maxFraction && input.maxFraction < B2_HUGE();
    return isValid;
}

function b2ComputePolygonCentroid(vertices: StaticArray<b2Vec2>, count: i32): b2Vec2 {
    let center = new b2Vec2(0, 0);
    let area: f32 = 0.0;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    let origin = vertices[0];

    let inv3: f32 = <f32>1.0 / <f32>3.0;

    for (let i = 1; i < count - 1; ++i) {
        // Triangle edges
        let e1 = b2Sub(vertices[i], origin);
        let e2 = b2Sub(vertices[i + 1], origin);
        let a: f32 = <f32>0.5 * b2Cross(e1, e2);

        // Area weighted centroid
        center = b2MulAdd(center, a * inv3, b2Add(e1, e2));
        area += a;
    }

    assert(area > FLT_EPSILON);
    let invArea: f32 = <f32>1.0 / area;
    center.x *= invArea;
    center.y *= invArea;

    // Restore offset
    center = b2Add(origin, center);

    return center;
}

export function b2MakePolygon(hull: b2Hull, radius: f32): b2Polygon {
    assert(b2ValidateHull(hull));

    if (hull.count < 3) {
        // Handle a bad hull when assertions are disabled
        return b2MakeSquare(0.5);
    }

    let shape = new b2Polygon();
    shape.count = hull.count;
    shape.radius = radius;

    // Copy vertices
    for (let i = 0; i < shape.count; ++i) {
        shape.vertices[i] = hull.points[i].clone();
    }

    // Compute normals. Ensure the edges have non-zero length.
    for (let i = 0; i < shape.count; ++i) {
        let i1 = i;
        let i2 = i + 1 < shape.count ? i + 1 : 0;
        let edge = b2Sub(shape.vertices[i2], shape.vertices[i1]);
        assert(b2Dot(edge, edge) > FLT_EPSILON * FLT_EPSILON);
        shape.normals[i] = b2Normalize(b2CrossVS(edge, 1.0));
    }

    shape.centroid = b2ComputePolygonCentroid(shape.vertices, shape.count);

    return shape;
}

export function b2MakeOffsetPolygon(hull: b2Hull, position: b2Vec2, rotation: b2Rot): b2Polygon {
    return b2MakeOffsetRoundedPolygon(hull, position, rotation, 0.0);
}

export function b2MakeOffsetRoundedPolygon(hull: b2Hull, position: b2Vec2, rotation: b2Rot, radius: f32): b2Polygon {
    assert(b2ValidateHull(hull));

    if (hull.count < 3) {
        // Handle a bad hull when assertions are disabled
        return b2MakeSquare(0.5);
    }

    let transform = new b2Transform(position, rotation);

    let shape = new b2Polygon();
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
        assert(b2Dot(edge, edge) > FLT_EPSILON * FLT_EPSILON);
        shape.normals[i] = b2Normalize(b2CrossVS(edge, 1.0));
    }

    shape.centroid = b2ComputePolygonCentroid(shape.vertices, shape.count);

    return shape;
}

export function b2MakeSquare(halfWidth: f32): b2Polygon {
    return b2MakeBox(halfWidth, halfWidth);
}

export function b2MakeBox(halfWidth: f32, halfHeight: f32): b2Polygon {
    assert(b2IsValidFloat(halfWidth) && halfWidth > 0.0);
    assert(b2IsValidFloat(halfHeight) && halfHeight > 0.0);

    let shape = new b2Polygon();
    shape.count = 4;
    shape.vertices[0] = new b2Vec2(-halfWidth, -halfHeight);
    shape.vertices[1] = new b2Vec2(halfWidth, -halfHeight);
    shape.vertices[2] = new b2Vec2(halfWidth, halfHeight);
    shape.vertices[3] = new b2Vec2(-halfWidth, halfHeight);
    shape.normals[0] = new b2Vec2(0.0, -1.0);
    shape.normals[1] = new b2Vec2(1.0, 0.0);
    shape.normals[2] = new b2Vec2(0.0, 1.0);
    shape.normals[3] = new b2Vec2(-1.0, 0.0);
    shape.radius = 0.0;
    shape.centroid = b2Vec2_zero();
    return shape;
}

export function b2MakeRoundedBox(halfWidth: f32, halfHeight: f32, radius: f32): b2Polygon {
    assert(b2IsValidFloat(radius) && radius >= 0.0);
    let shape = b2MakeBox(halfWidth, halfHeight);
    shape.radius = radius;
    return shape;
}

export function b2MakeOffsetBox(halfWidth: f32, halfHeight: f32, center: b2Vec2, rotation: b2Rot): b2Polygon {
    let xf = new b2Transform(center, rotation);

    let shape = new b2Polygon();
    shape.count = 4;
    shape.vertices[0] = b2TransformPoint(xf, new b2Vec2(-halfWidth, -halfHeight));
    shape.vertices[1] = b2TransformPoint(xf, new b2Vec2(halfWidth, -halfHeight));
    shape.vertices[2] = b2TransformPoint(xf, new b2Vec2(halfWidth, halfHeight));
    shape.vertices[3] = b2TransformPoint(xf, new b2Vec2(-halfWidth, halfHeight));
    shape.normals[0] = b2RotateVector(xf.q, new b2Vec2(0.0, -1.0));
    shape.normals[1] = b2RotateVector(xf.q, new b2Vec2(1.0, 0.0));
    shape.normals[2] = b2RotateVector(xf.q, new b2Vec2(0.0, 1.0));
    shape.normals[3] = b2RotateVector(xf.q, new b2Vec2(-1.0, 0.0));
    shape.radius = 0.0;
    shape.centroid = xf.p.clone();
    return shape;
}

export function b2MakeOffsetRoundedBox(halfWidth: f32, halfHeight: f32, center: b2Vec2, rotation: b2Rot,
    radius: f32): b2Polygon {
    assert(b2IsValidFloat(radius) && radius >= 0.0);
    let shape = b2MakeOffsetBox(halfWidth, halfHeight, center, rotation);
    shape.radius = radius;
    return shape;
}

export function b2TransformPolygon(transform: b2Transform, polygon: b2Polygon): b2Polygon {
    let p = new b2Polygon();
    p.count = polygon.count;
    p.radius = polygon.radius;

    for (let i = 0; i < p.count; ++i) {
        p.vertices[i] = b2TransformPoint(transform, polygon.vertices[i]);
        p.normals[i] = b2RotateVector(transform.q, polygon.normals[i]);
    }

    p.centroid = b2TransformPoint(transform, polygon.centroid);

    return p;
}

export function b2ComputeCircleMass(shape: b2Circle, density: f32): b2MassData {
    let rr: f32 = shape.radius * shape.radius;

    let massData = new b2MassData();
    massData.mass = density * B2_PI * rr;
    massData.center = shape.center.clone();

    // inertia about the center of mass
    massData.rotationalInertia = massData.mass * <f32>0.5 * rr;

    return massData;
}

export function b2ComputeCapsuleMass(shape: b2Capsule, density: f32): b2MassData {
    let radius: f32 = shape.radius;
    let rr: f32 = radius * radius;
    let p1 = shape.center1;
    let p2 = shape.center2;
    let length: f32 = b2Length(b2Sub(p2, p1));
    let ll: f32 = length * length;

    let circleMass: f32 = density * (B2_PI * radius * radius);
    let boxMass: f32 = density * (<f32>2.0 * radius * length);

    let massData = new b2MassData();
    massData.mass = circleMass + boxMass;
    massData.center = new b2Vec2(<f32>0.5 * (p1.x + p2.x), <f32>0.5 * (p1.y + p2.y));

    // two offset half circles, both halves add up to full circle and each half is offset by half length
    // See the parallel-axis derivation in the C source.

    // half circle centroid
    let lc: f32 = <f32>4.0 * radius / (<f32>3.0 * B2_PI);

    // half length of rectangular portion of capsule
    let h: f32 = <f32>0.5 * length;

    let circleInertia: f32 = circleMass * (<f32>0.5 * rr + h * h + <f32>2.0 * h * lc);
    let boxInertia: f32 = boxMass * (<f32>4.0 * rr + ll) / <f32>12.0;
    massData.rotationalInertia = circleInertia + boxInertia;

    return massData;
}

export function b2ComputePolygonMass(shape: b2Polygon, density: f32): b2MassData {
    // See the C source for the full derivation; the code is kept verbatim.
    assert(shape.count > 0);

    if (shape.count == 1) {
        let circle = new b2Circle(shape.vertices[0].clone(), shape.radius);
        return b2ComputeCircleMass(circle, density);
    }

    if (shape.count == 2) {
        let capsule = new b2Capsule(shape.vertices[0].clone(), shape.vertices[1].clone(), shape.radius);
        return b2ComputeCapsuleMass(capsule, density);
    }

    let vertices = makeVecArray(B2_MAX_POLYGON_VERTICES);
    let count = shape.count;
    let radius: f32 = shape.radius;

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
            vertices[i] = shape.vertices[i].clone();
        }
    }

    let center = new b2Vec2(0, 0);
    let area: f32 = 0.0;
    let rotationalInertia: f32 = 0.0;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    let r = vertices[0];

    let inv3: f32 = <f32>1.0 / <f32>3.0;

    for (let i = 1; i < count - 1; ++i) {
        // Triangle edges
        let e1 = b2Sub(vertices[i], r);
        let e2 = b2Sub(vertices[i + 1], r);

        let D: f32 = b2Cross(e1, e2);

        let triangleArea: f32 = <f32>0.5 * D;
        area += triangleArea;

        // Area weighted centroid, r at origin
        center = b2MulAdd(center, triangleArea * inv3, b2Add(e1, e2));

        let ex1: f32 = e1.x;
        let ey1: f32 = e1.y;
        let ex2: f32 = e2.x;
        let ey2: f32 = e2.y;

        let intx2: f32 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
        let inty2: f32 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

        rotationalInertia += (<f32>0.25 * inv3 * D) * (intx2 + inty2);
    }

    let massData = new b2MassData();

    // Total mass
    massData.mass = density * area;

    // Center of mass, shift back from origin at r
    assert(area > FLT_EPSILON);
    let invArea: f32 = <f32>1.0 / area;
    center.x *= invArea;
    center.y *= invArea;
    massData.center = b2Add(r, center);

    // Inertia tensor relative to the local origin (point s).
    massData.rotationalInertia = density * rotationalInertia;

    // Shift inertia to center of mass
    massData.rotationalInertia -= massData.mass * b2Dot(center, center);

    // If this goes negative we are hosed
    assert(massData.rotationalInertia >= 0.0);

    return massData;
}

export function b2ComputeCircleAABB(shape: b2Circle, xf: b2Transform): b2AABB {
    let p = b2TransformPoint(xf, shape.center);
    let r: f32 = shape.radius;

    return new b2AABB(new b2Vec2(p.x - r, p.y - r), new b2Vec2(p.x + r, p.y + r));
}

export function b2ComputeCapsuleAABB(shape: b2Capsule, xf: b2Transform): b2AABB {
    let v1 = b2TransformPoint(xf, shape.center1);
    let v2 = b2TransformPoint(xf, shape.center2);

    let r = new b2Vec2(shape.radius, shape.radius);
    let lower = b2Sub(b2Min(v1, v2), r);
    let upper = b2Add(b2Max(v1, v2), r);

    return new b2AABB(lower, upper);
}

export function b2ComputePolygonAABB(shape: b2Polygon, xf: b2Transform): b2AABB {
    assert(shape.count > 0);
    let lower = b2TransformPoint(xf, shape.vertices[0]);
    let upper = lower.clone();

    for (let i = 1; i < shape.count; ++i) {
        let v = b2TransformPoint(xf, shape.vertices[i]);
        lower = b2Min(lower, v);
        upper = b2Max(upper, v);
    }

    let r = new b2Vec2(shape.radius, shape.radius);
    lower = b2Sub(lower, r);
    upper = b2Add(upper, r);

    return new b2AABB(lower, upper);
}

export function b2ComputeSegmentAABB(shape: b2Segment, xf: b2Transform): b2AABB {
    let v1 = b2TransformPoint(xf, shape.point1);
    let v2 = b2TransformPoint(xf, shape.point2);

    let lower = b2Min(v1, v2);
    let upper = b2Max(v1, v2);

    return new b2AABB(lower, upper);
}

export function b2PointInPolygon(shape: b2Polygon, point: b2Vec2): bool {
    let input = new b2DistanceInput();
    input.proxyA = b2MakeProxy(shape.vertices, shape.count, 0.0);
    let pointArr = new StaticArray<b2Vec2>(1);
    pointArr[0] = point;
    input.proxyB = b2MakeProxy(pointArr, 1, 0.0);
    input.transformA = b2Transform_identity();
    input.transformB = b2Transform_identity();
    input.useRadii = false;

    let cache = new b2SimplexCache();
    let output = b2ShapeDistance(input, cache);

    return output.distance <= shape.radius;
}

export function b2PointInCircle(shape: b2Circle, point: b2Vec2): bool {
    let center = shape.center;
    return b2DistanceSquared(point, center) <= shape.radius * shape.radius;
}

export function b2PointInCapsule(shape: b2Capsule, point: b2Vec2): bool {
    let rr: f32 = shape.radius * shape.radius;
    let p1 = shape.center1;
    let p2 = shape.center2;

    let d = b2Sub(p2, p1);
    let dd: f32 = b2Dot(d, d);
    if (dd == 0.0) {
        // Capsule is really a circle
        return b2DistanceSquared(point, p1) <= rr;
    }

    // Get closest point on capsule segment
    let t: f32 = b2Dot(b2Sub(point, p1), d) / dd;
    t = b2ClampFloat(t, 0.0, 1.0);
    let c = b2MulAdd(p1, t, d);

    // Is query point within radius around closest point?
    return b2DistanceSquared(point, c) <= rr;
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
export function b2RayCastCircle(shape: b2Circle, input: b2RayCastInput): b2CastOutput {
    assert(b2IsValidRay(input));

    let p = shape.center;

    let output = new b2CastOutput();

    // Shift ray so circle center is the origin
    let s = b2Sub(input.origin, p);

    let r: f32 = shape.radius;
    let rr: f32 = r * r;

    let ln = b2GetLengthAndNormalize(input.translation);
    let length: f32 = ln.length;
    let d = ln.normal;
    if (length == 0.0) {
        // zero length ray

        if (b2LengthSquared(s) < rr) {
            // initial overlap
            output.point = input.origin.clone();
            output.hit = true;
        }

        return output;
    }

    // Find closest point on ray to origin

    // solve: dot(s + t * d, d) = 0
    let t: f32 = -b2Dot(s, d);

    // c is the closest point on the line to the origin
    let c = b2MulAdd(s, t, d);

    let cc: f32 = b2Dot(c, c);

    if (cc > rr) {
        // closest point is outside the circle
        return output;
    }

    // Pythagoras
    let h: f32 = sqrt<f32>(rr - cc);

    let fraction: f32 = t - h;

    if (fraction < 0.0 || input.maxFraction * length < fraction) {
        // intersection is point outside the range of the ray segment

        if (b2LengthSquared(s) < rr) {
            // initial overlap
            output.point = input.origin.clone();
            output.hit = true;
        }

        return output;
    }

    // hit point relative to center
    let hitPoint = b2MulAdd(s, fraction, d);

    output.fraction = fraction / length;
    output.normal = b2Normalize(hitPoint);
    output.point = b2MulAdd(p, shape.radius, output.normal);
    output.hit = true;

    return output;
}

export function b2RayCastCapsule(shape: b2Capsule, input: b2RayCastInput): b2CastOutput {
    assert(b2IsValidRay(input));

    let output = new b2CastOutput();

    let v1 = shape.center1;
    let v2 = shape.center2;

    let e = b2Sub(v2, v1);

    let lnA = b2GetLengthAndNormalize(e);
    let capsuleLength: f32 = lnA.length;
    let a = lnA.normal;

    if (capsuleLength < FLT_EPSILON) {
        // Capsule is really a circle
        let circle = new b2Circle(v1.clone(), shape.radius);
        return b2RayCastCircle(circle, input);
    }

    let p1 = input.origin;
    let d = input.translation;

    // Ray from capsule start to ray start
    let q = b2Sub(p1, v1);
    let qa: f32 = b2Dot(q, a);

    // Vector to ray start that is perpendicular to capsule axis
    let qp = b2MulAdd(q, -qa, a);

    let radius: f32 = shape.radius;

    // Does the ray start within the infinite length capsule?
    if (b2Dot(qp, qp) < radius * radius) {
        if (qa < 0.0) {
            // start point behind capsule segment
            let circle = new b2Circle(v1.clone(), shape.radius);
            return b2RayCastCircle(circle, input);
        }

        if (qa > capsuleLength) {
            // start point ahead of capsule segment
            let circle = new b2Circle(v2.clone(), shape.radius);
            return b2RayCastCircle(circle, input);
        }

        // ray starts inside capsule -> no hit
        output.point = input.origin.clone();
        output.hit = true;
        return output;
    }

    // Perpendicular to capsule axis, pointing right
    let n = new b2Vec2(a.y, -a.x);

    let lnU = b2GetLengthAndNormalize(d);
    let rayLength: f32 = lnU.length;
    let u = lnU.normal;

    // Intersect ray with infinite length capsule; Cramer's rule [a -u]
    let den: f32 = -a.x * u.y + u.x * a.y;
    if (-FLT_EPSILON < den && den < FLT_EPSILON) {
        // Ray is parallel to capsule and outside infinite length capsule
        return output;
    }

    let b1 = b2MulSub(q, radius, n);
    let b2v = b2MulAdd(q, radius, n);

    let invDen: f32 = <f32>1.0 / den;

    // Cramer's rule [a b1]
    let s21: f32 = (a.x * b1.y - b1.x * a.y) * invDen;

    // Cramer's rule [a b2]
    let s22: f32 = (a.x * b2v.y - b2v.x * a.y) * invDen;

    let s2: f32;
    let b: b2Vec2;
    if (s21 < s22) {
        s2 = s21;
        b = b1;
    } else {
        s2 = s22;
        b = b2v;
        n = b2Neg(n);
    }

    if (s2 < 0.0 || input.maxFraction * rayLength < s2) {
        return output;
    }

    // Cramer's rule [b -u]
    let s1: f32 = (-b.x * u.y + u.x * b.y) * invDen;

    if (s1 < 0.0) {
        // ray passes behind capsule segment
        let circle = new b2Circle(v1.clone(), shape.radius);
        return b2RayCastCircle(circle, input);
    } else if (capsuleLength < s1) {
        // ray passes ahead of capsule segment
        let circle = new b2Circle(v2.clone(), shape.radius);
        return b2RayCastCircle(circle, input);
    } else {
        // ray hits capsule side
        output.fraction = s2 / rayLength;
        output.point = b2Add(b2Lerp(v1, v2, s1 / capsuleLength), b2MulSV(shape.radius, n));
        output.normal = n;
        output.hit = true;
        return output;
    }
}

// Ray vs line segment
export function b2RayCastSegment(shape: b2Segment, input: b2RayCastInput, oneSided: bool): b2CastOutput {
    if (oneSided) {
        // Skip left-side collision
        let offset: f32 = b2Cross(b2Sub(input.origin, shape.point1), b2Sub(shape.point2, shape.point1));
        if (offset < 0.0) {
            return new b2CastOutput();
        }
    }

    // Put the ray into the edge's frame of reference.
    let p1 = input.origin;
    let d = input.translation;

    let v1 = shape.point1;
    let v2 = shape.point2;
    let e = b2Sub(v2, v1);

    let output = new b2CastOutput();

    let ln = b2GetLengthAndNormalize(e);
    let length: f32 = ln.length;
    let eUnit = ln.normal;
    if (length == 0.0) {
        return output;
    }

    // Normal points to the right, looking from v1 towards v2
    let normal = b2RightPerp(eUnit);

    // Intersect ray with infinite segment using normal
    let numerator: f32 = b2Dot(normal, b2Sub(v1, p1));
    let denominator: f32 = b2Dot(normal, d);

    if (denominator == 0.0) {
        // parallel
        return output;
    }

    let t: f32 = numerator / denominator;
    if (t < 0.0 || input.maxFraction < t) {
        // out of ray range
        return output;
    }

    // Intersection point on infinite segment
    let p = b2MulAdd(p1, t, d);

    // Compute position of p along segment
    let s: f32 = b2Dot(b2Sub(p, v1), eUnit);
    if (s < 0.0 || length < s) {
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

export function b2RayCastPolygon(shape: b2Polygon, input: b2RayCastInput): b2CastOutput {
    assert(b2IsValidRay(input));

    if (shape.radius != 0.0) {
        let castInput = new b2ShapeCastPairInput();
        castInput.proxyA = b2MakeProxy(shape.vertices, shape.count, shape.radius);
        let originArr = new StaticArray<b2Vec2>(1);
        originArr[0] = input.origin;
        castInput.proxyB = b2MakeProxy(originArr, 1, 0.0);
        castInput.transformA = b2Transform_identity();
        castInput.transformB = b2Transform_identity();
        castInput.translationB = input.translation.clone();
        castInput.maxFraction = input.maxFraction;
        castInput.canEncroach = false;
        return b2ShapeCast(castInput);
    }

    // Shift all math to first vertex since the polygon may be far
    // from the origin.
    let base = shape.vertices[0];

    let p1 = b2Sub(input.origin, base);
    let d = input.translation;

    let lower: f32 = 0.0;
    let upper: f32 = input.maxFraction;

    let index = -1;

    let output = new b2CastOutput();

    for (let edgeIndex = 0; edgeIndex < shape.count; ++edgeIndex) {
        // p = p1 + a * d
        // dot(normal, p - v) = 0
        // dot(normal, p1 - v) + a * dot(normal, d) = 0
        let vertex = b2Sub(shape.vertices[edgeIndex], base);
        let numerator: f32 = b2Dot(shape.normals[edgeIndex], b2Sub(vertex, p1));
        let denominator: f32 = b2Dot(shape.normals[edgeIndex], d);

        if (denominator == 0.0) {
            // Parallel and runs outside edge
            if (numerator < 0.0) {
                return output;
            }
        } else {
            // Division-free ordering predicate, see the C source.
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

    assert(0.0 <= lower && lower <= input.maxFraction);

    if (index >= 0) {
        output.fraction = lower;
        output.normal = shape.normals[index].clone();
        output.point = b2MulAdd(input.origin, lower, d);
        output.hit = true;
    } else {
        // initial overlap
        output.point = input.origin.clone();
        output.hit = true;
    }

    return output;
}

export function b2ShapeCastCircle(shape: b2Circle, input: b2ShapeCastInput): b2CastOutput {
    let pairInput = new b2ShapeCastPairInput();
    let centerArr = new StaticArray<b2Vec2>(1);
    centerArr[0] = shape.center;
    pairInput.proxyA = b2MakeProxy(centerArr, 1, shape.radius);
    pairInput.proxyB = input.proxy;
    pairInput.transformA = b2Transform_identity();
    pairInput.transformB = b2Transform_identity();
    pairInput.translationB = input.translation.clone();
    pairInput.maxFraction = input.maxFraction;
    pairInput.canEncroach = input.canEncroach;

    return b2ShapeCast(pairInput);
}

export function b2ShapeCastCapsule(shape: b2Capsule, input: b2ShapeCastInput): b2CastOutput {
    let pairInput = new b2ShapeCastPairInput();
    let centers = new StaticArray<b2Vec2>(2);
    centers[0] = shape.center1;
    centers[1] = shape.center2;
    pairInput.proxyA = b2MakeProxy(centers, 2, shape.radius);
    pairInput.proxyB = input.proxy;
    pairInput.transformA = b2Transform_identity();
    pairInput.transformB = b2Transform_identity();
    pairInput.translationB = input.translation.clone();
    pairInput.maxFraction = input.maxFraction;
    pairInput.canEncroach = input.canEncroach;

    return b2ShapeCast(pairInput);
}

export function b2ShapeCastSegment(shape: b2Segment, input: b2ShapeCastInput): b2CastOutput {
    let pairInput = new b2ShapeCastPairInput();
    let points = new StaticArray<b2Vec2>(2);
    points[0] = shape.point1;
    points[1] = shape.point2;
    pairInput.proxyA = b2MakeProxy(points, 2, 0.0);
    pairInput.proxyB = input.proxy;
    pairInput.transformA = b2Transform_identity();
    pairInput.transformB = b2Transform_identity();
    pairInput.translationB = input.translation.clone();
    pairInput.maxFraction = input.maxFraction;
    pairInput.canEncroach = input.canEncroach;

    return b2ShapeCast(pairInput);
}

export function b2ShapeCastPolygon(shape: b2Polygon, input: b2ShapeCastInput): b2CastOutput {
    let pairInput = new b2ShapeCastPairInput();
    pairInput.proxyA = b2MakeProxy(shape.vertices, shape.count, shape.radius);
    pairInput.proxyB = input.proxy;
    pairInput.transformA = b2Transform_identity();
    pairInput.transformB = b2Transform_identity();
    pairInput.translationB = input.translation.clone();
    pairInput.maxFraction = input.maxFraction;
    pairInput.canEncroach = input.canEncroach;

    return b2ShapeCast(pairInput);
}
