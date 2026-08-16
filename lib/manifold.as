// Ported from box2d src/manifold.c. Feature ids (B2_MAKE_ID) and anchor
// packing are bit-exact; they feed contact warm starting in Phase C.

import {
    b2Vec2, b2Transform,
    b2Add, b2Sub, b2Neg, b2Dot, b2Cross, b2CrossSV, b2MulAdd, b2MulSub, b2MulSV, b2Lerp,
    b2Normalize, b2GetLengthAndNormalize, b2LengthSquared, b2DistanceSquared, b2ClampFloat,
    b2MinFloat, b2LeftPerp, b2RightPerp, b2RotateVector, b2TransformPoint, b2InvMulTransforms,
    b2Transform_identity, b2Vec2_zero, FLT_EPSILON, FLT_MAX,
} from "./math_functions";
import {
    b2Circle, b2Capsule, b2Polygon, b2Segment, b2ChainSegment, b2Manifold, b2ManifoldPoint,
    b2SimplexCache, b2DistanceInput,
    B2_MAX_POLYGON_VERTICES, makeVecArray,
} from "./collision";
import { B2_LINEAR_SLOP, B2_SPECULATIVE_DISTANCE } from "./constants";
import { b2MakeProxy, b2ShapeDistance, b2SegmentDistance } from "./distance";

function B2_MAKE_ID(a: i32, b: i32): u16 {
    return <u16>(((a & 0xff) << 8) | (b & 0xff));
}

function b2MakeCapsule(p1: b2Vec2, p2: b2Vec2, radius: f32): b2Polygon {
    let shape = new b2Polygon();
    shape.vertices[0] = p1.clone();
    shape.vertices[1] = p2.clone();
    shape.centroid = b2Lerp(p1, p2, 0.5);

    let d = b2Sub(p2, p1);
    assert(b2LengthSquared(d) > FLT_EPSILON);
    let axis = b2Normalize(d);
    let normal = b2RightPerp(axis);

    shape.normals[0] = normal;
    shape.normals[1] = b2Neg(normal);
    shape.count = 2;
    shape.radius = radius;

    return shape;
}

export function b2CollideCircles(circleA: b2Circle, xfA: b2Transform, circleB: b2Circle, xfB: b2Transform): b2Manifold {
    let manifold = new b2Manifold();

    let xf = b2InvMulTransforms(xfA, xfB);

    let pointA = circleA.center;
    let pointB = b2TransformPoint(xf, circleB.center);

    let ln = b2GetLengthAndNormalize(b2Sub(pointB, pointA));
    let distance: f32 = ln.length;
    let normal = ln.normal;

    let radiusA: f32 = circleA.radius;
    let radiusB: f32 = circleB.radius;

    let separation: f32 = distance - radiusA - radiusB;
    if (separation > B2_SPECULATIVE_DISTANCE()) {
        return manifold;
    }

    let cA = b2MulAdd(pointA, radiusA, normal);
    let cB = b2MulAdd(pointB, -radiusB, normal);
    let contactPointA = b2Lerp(cA, cB, 0.5);

    manifold.normal = b2RotateVector(xfA.q, normal);
    let mp = manifold.points[0];
    mp.anchorA = b2RotateVector(xfA.q, contactPointA);
    mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
    mp.point = b2Add(mp.anchorA, xfA.p);
    mp.separation = separation;
    mp.id = 0;
    manifold.pointCount = 1;
    return manifold;
}

export function b2CollideCapsuleAndCircle(capsuleA: b2Capsule, xfA: b2Transform, circleB: b2Circle,
    xfB: b2Transform): b2Manifold {
    let manifold = new b2Manifold();

    let xf = b2InvMulTransforms(xfA, xfB);

    // Compute circle position in the frame of the capsule.
    let pB = b2TransformPoint(xf, circleB.center);

    // Compute closest point
    let p1 = capsuleA.center1;
    let p2 = capsuleA.center2;

    let e = b2Sub(p2, p1);

    let pA: b2Vec2;
    let s1: f32 = b2Dot(b2Sub(pB, p1), e);
    let s2: f32 = b2Dot(b2Sub(p2, pB), e);
    if (s1 < 0.0) {
        // p1 region
        pA = p1;
    } else if (s2 < 0.0) {
        // p2 region
        pA = p2;
    } else {
        // circle colliding with segment interior
        let s: f32 = s1 / b2Dot(e, e);
        pA = b2MulAdd(p1, s, e);
    }

    let ln = b2GetLengthAndNormalize(b2Sub(pB, pA));
    let distance: f32 = ln.length;
    let normal = ln.normal;

    let radiusA: f32 = capsuleA.radius;
    let radiusB: f32 = circleB.radius;
    let separation: f32 = distance - radiusA - radiusB;
    if (separation > B2_SPECULATIVE_DISTANCE()) {
        return manifold;
    }

    let cA = b2MulAdd(pA, radiusA, normal);
    let cB = b2MulAdd(pB, -radiusB, normal);
    let contactPointA = b2Lerp(cA, cB, 0.5);

    manifold.normal = b2RotateVector(xfA.q, normal);
    let mp = manifold.points[0];
    mp.anchorA = b2RotateVector(xfA.q, contactPointA);
    mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
    mp.point = b2Add(xfA.p, mp.anchorA);
    mp.separation = separation;
    mp.id = 0;
    manifold.pointCount = 1;
    return manifold;
}

export function b2CollidePolygonAndCircle(polygonA: b2Polygon, xfA: b2Transform, circleB: b2Circle,
    xfB: b2Transform): b2Manifold {
    let manifold = new b2Manifold();
    let speculativeDistance: f32 = B2_SPECULATIVE_DISTANCE();

    let xf = b2InvMulTransforms(xfA, xfB);

    // Compute circle position in the frame of the polygon.
    let center = b2TransformPoint(xf, circleB.center);
    let radiusA: f32 = polygonA.radius;
    let radiusB: f32 = circleB.radius;
    let radius: f32 = radiusA + radiusB;

    // Find the min separating edge.
    let normalIndex = 0;
    let separation: f32 = -FLT_MAX;
    let vertexCount = polygonA.count;
    let vertices = polygonA.vertices;
    let normals = polygonA.normals;

    for (let i = 0; i < vertexCount; ++i) {
        let s: f32 = b2Dot(unchecked(normals[i]), b2Sub(center, unchecked(vertices[i])));
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
    let v1 = vertices[vertIndex1];
    let v2 = vertices[vertIndex2];

    // Compute barycentric coordinates
    let u1: f32 = b2Dot(b2Sub(center, v1), b2Sub(v2, v1));
    let u2: f32 = b2Dot(b2Sub(center, v2), b2Sub(v1, v2));

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

        manifold.normal = b2RotateVector(xfA.q, normal);
        let mp = manifold.points[0];
        mp.anchorA = b2RotateVector(xfA.q, contactPointA);
        mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
        mp.point = b2Add(xfA.p, mp.anchorA);
        mp.separation = b2Dot(b2Sub(cB, cA), normal);
        mp.id = 0;
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

        manifold.normal = b2RotateVector(xfA.q, normal);
        let mp = manifold.points[0];
        mp.anchorA = b2RotateVector(xfA.q, contactPointA);
        mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
        mp.point = b2Add(xfA.p, mp.anchorA);
        mp.separation = b2Dot(b2Sub(cB, cA), normal);
        mp.id = 0;
        manifold.pointCount = 1;
    } else {
        // Circle center is between v1 and v2. Center may be inside polygon
        let normal = normals[normalIndex];
        manifold.normal = b2RotateVector(xfA.q, normal);

        // cA is the projection of the circle center onto to the reference edge
        let cA = b2MulAdd(center, radiusA - b2Dot(b2Sub(center, v1), normal), normal);

        // cB is the deepest point on the circle with respect to the reference edge
        let cB = b2MulSub(center, radiusB, normal);

        let contactPointA = b2Lerp(cA, cB, 0.5);

        // The contact point is the midpoint in world space
        let mp = manifold.points[0];
        mp.anchorA = b2RotateVector(xfA.q, contactPointA);
        mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
        mp.point = b2Add(xfA.p, mp.anchorA);
        mp.separation = separation - radius;
        mp.id = 0;
        manifold.pointCount = 1;
    }

    return manifold;
}

// Follows Ericson 5.1.9 Closest Points of Two Line Segments
// Adds some logic to support clipping to get two contact points
export function b2CollideCapsules(capsuleA: b2Capsule, xfA: b2Transform, capsuleB: b2Capsule,
    xfB: b2Transform): b2Manifold {
    let origin = capsuleA.center1;

    // Shift polyA to origin
    let sfA = new b2Transform(b2Add(xfA.p, b2RotateVector(xfA.q, origin)), xfA.q.clone());
    let xf = b2InvMulTransforms(sfA, xfB);

    let p1 = b2Vec2_zero();
    let q1 = b2Sub(capsuleA.center2, origin);

    let p2 = b2TransformPoint(xf, capsuleB.center1);
    let q2 = b2TransformPoint(xf, capsuleB.center2);

    let d1 = b2Sub(q1, p1);
    let d2 = b2Sub(q2, p2);

    let dd1: f32 = b2Dot(d1, d1);
    let dd2: f32 = b2Dot(d2, d2);

    let epsSqr: f32 = FLT_EPSILON * FLT_EPSILON;
    assert(dd1 > epsSqr && dd2 > epsSqr);

    let r = b2Sub(p1, p2);
    let rd1: f32 = b2Dot(r, d1);
    let rd2: f32 = b2Dot(r, d2);

    let d12: f32 = b2Dot(d1, d2);

    let denom: f32 = dd1 * dd2 - d12 * d12;

    // Fraction on segment 1
    let f1: f32 = 0.0;
    if (denom != 0.0) {
        // not parallel
        f1 = b2ClampFloat((d12 * rd2 - rd1 * dd2) / denom, 0.0, 1.0);
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

    let closest1 = b2MulAdd(p1, f1, d1);
    let closest2 = b2MulAdd(p2, f2, d2);
    let distanceSquared: f32 = b2DistanceSquared(closest1, closest2);

    let manifold = new b2Manifold();
    let radiusA: f32 = capsuleA.radius;
    let radiusB: f32 = capsuleB.radius;
    let radius: f32 = radiusA + radiusB;
    let maxDistance: f32 = radius + B2_SPECULATIVE_DISTANCE();

    if (distanceSquared > maxDistance * maxDistance) {
        return manifold;
    }

    let distance: f32 = sqrt<f32>(distanceSquared);

    let ln1 = b2GetLengthAndNormalize(d1);
    let length1: f32 = ln1.length;
    let u1 = ln1.normal;
    let ln2 = b2GetLengthAndNormalize(d2);
    let length2: f32 = ln2.length;
    let u2 = ln2.normal;

    // Does segment B project outside segment A?
    let fp2: f32 = b2Dot(b2Sub(p2, p1), u1);
    let fq2: f32 = b2Dot(b2Sub(q2, p1), u1);
    let outsideA = (fp2 <= 0.0 && fq2 <= 0.0) || (fp2 >= length1 && fq2 >= length1);

    // Does segment A project outside segment B?
    let fp1: f32 = b2Dot(b2Sub(p1, p2), u2);
    let fq1: f32 = b2Dot(b2Sub(q1, p2), u2);
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
            let ss1: f32 = b2Dot(b2Sub(p2, p1), normalA);
            let ss2: f32 = b2Dot(b2Sub(q2, p1), normalA);
            let s1p: f32 = ss1 < ss2 ? ss1 : ss2;
            let s1n: f32 = -ss1 < -ss2 ? -ss1 : -ss2;

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
            let ss1: f32 = b2Dot(b2Sub(p1, p2), normalB);
            let ss2: f32 = b2Dot(b2Sub(q1, p2), normalB);
            let s1p: f32 = ss1 < ss2 ? ss1 : ss2;
            let s1n: f32 = -ss1 < -ss2 ? -ss1 : -ss2;

            if (s1p > s1n) {
                separationB = s1p;
            } else {
                separationB = s1n;
                normalB = b2Neg(normalB);
            }
        }

        // biased to avoid feature flip-flop
        if (separationA + <f32>0.1 * B2_LINEAR_SLOP() >= separationB) {
            manifold.normal = normalA;

            let cp = p2;
            let cq = q2;

            // clip to p1
            if (fp2 < 0.0 && fq2 > 0.0) {
                cp = b2Lerp(p2, q2, (<f32>0.0 - fp2) / (fq2 - fp2));
            } else if (fq2 < 0.0 && fp2 > 0.0) {
                cq = b2Lerp(q2, p2, (<f32>0.0 - fq2) / (fp2 - fq2));
            }

            // clip to q1
            if (fp2 > length1 && fq2 < length1) {
                cp = b2Lerp(p2, q2, (fp2 - length1) / (fp2 - fq2));
            } else if (fq2 > length1 && fp2 < length1) {
                cq = b2Lerp(q2, p2, (fq2 - length1) / (fq2 - fp2));
            }

            let sp: f32 = b2Dot(b2Sub(cp, p1), normalA);
            let sq: f32 = b2Dot(b2Sub(cq, p1), normalA);

            if (sp <= distance + B2_LINEAR_SLOP() || sq <= distance + B2_LINEAR_SLOP()) {
                let mp = manifold.points[0];
                mp.anchorA = b2MulAdd(cp, <f32>0.5 * (radiusA - radiusB - sp), normalA);
                mp.separation = sp - radius;
                mp.id = B2_MAKE_ID(0, 0);

                mp = manifold.points[1];
                mp.anchorA = b2MulAdd(cq, <f32>0.5 * (radiusA - radiusB - sq), normalA);
                mp.separation = sq - radius;
                mp.id = B2_MAKE_ID(0, 1);
                manifold.pointCount = 2;
            }
        } else {
            // normal always points from A to B
            manifold.normal = b2Neg(normalB);

            let cp = p1;
            let cq = q1;

            // clip to p2
            if (fp1 < 0.0 && fq1 > 0.0) {
                cp = b2Lerp(p1, q1, (<f32>0.0 - fp1) / (fq1 - fp1));
            } else if (fq1 < 0.0 && fp1 > 0.0) {
                cq = b2Lerp(q1, p1, (<f32>0.0 - fq1) / (fp1 - fq1));
            }

            // clip to q2
            if (fp1 > length2 && fq1 < length2) {
                cp = b2Lerp(p1, q1, (fp1 - length2) / (fp1 - fq1));
            } else if (fq1 > length2 && fp1 < length2) {
                cq = b2Lerp(q1, p1, (fq1 - length2) / (fq1 - fp1));
            }

            let sp: f32 = b2Dot(b2Sub(cp, p2), normalB);
            let sq: f32 = b2Dot(b2Sub(cq, p2), normalB);

            if (sp <= distance + B2_LINEAR_SLOP() || sq <= distance + B2_LINEAR_SLOP()) {
                let mp = manifold.points[0];
                mp.anchorA = b2MulAdd(cp, <f32>0.5 * (radiusB - radiusA - sp), normalB);
                mp.separation = sp - radius;
                mp.id = B2_MAKE_ID(0, 0);
                mp = manifold.points[1];
                mp.anchorA = b2MulAdd(cq, <f32>0.5 * (radiusB - radiusA - sq), normalB);
                mp.separation = sq - radius;
                mp.id = B2_MAKE_ID(1, 0);
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
        manifold.points[0].anchorA = b2Lerp(c1, c2, 0.5);
        manifold.points[0].separation = sqrt<f32>(distanceSquared) - radius;
        manifold.points[0].id = B2_MAKE_ID(i1, i2);
        manifold.pointCount = 1;
    }

    // Convert manifold to world space
    manifold.normal = b2RotateVector(xfA.q, manifold.normal);
    for (let i = 0; i < manifold.pointCount; ++i) {
        let mp = manifold.points[i];

        // anchor points relative to shape origin in world space
        mp.anchorA = b2RotateVector(xfA.q, b2Add(mp.anchorA, origin));
        mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
        mp.point = b2Add(xfA.p, mp.anchorA);
    }

    return manifold;
}

export function b2CollideSegmentAndCapsule(segmentA: b2Segment, xfA: b2Transform, capsuleB: b2Capsule,
    xfB: b2Transform): b2Manifold {
    let capsuleA = new b2Capsule(segmentA.point1.clone(), segmentA.point2.clone(), 0.0);
    return b2CollideCapsules(capsuleA, xfA, capsuleB, xfB);
}

export function b2CollidePolygonAndCapsule(polygonA: b2Polygon, xfA: b2Transform, capsuleB: b2Capsule,
    xfB: b2Transform): b2Manifold {
    let polyB = b2MakeCapsule(capsuleB.center1, capsuleB.center2, capsuleB.radius);
    return b2CollidePolygons(polygonA, xfA, polyB, xfB);
}

// Polygon clipper used to compute contact points when there are potentially two contact points.
function b2ClipPolygons(polyA: b2Polygon, polyB: b2Polygon, edgeA: i32, edgeB: i32, flip: bool): b2Manifold {
    let manifold = new b2Manifold();

    // reference polygon
    let poly1: b2Polygon;
    let i11: i32, i12: i32;

    // incident polygon
    let poly2: b2Polygon;
    let i21: i32, i22: i32;

    if (flip) {
        poly1 = polyB;
        poly2 = polyA;
        i11 = edgeB;
        i12 = edgeB + 1 < polyB.count ? edgeB + 1 : 0;
        i21 = edgeA;
        i22 = edgeA + 1 < polyA.count ? edgeA + 1 : 0;
    } else {
        poly1 = polyA;
        poly2 = polyB;
        i11 = edgeA;
        i12 = edgeA + 1 < polyA.count ? edgeA + 1 : 0;
        i21 = edgeB;
        i22 = edgeB + 1 < polyB.count ? edgeB + 1 : 0;
    }

    let normal = poly1.normals[i11];

    // Reference edge vertices
    let v11 = poly1.vertices[i11];
    let v12 = poly1.vertices[i12];

    // Incident edge vertices
    let v21 = poly2.vertices[i21];
    let v22 = poly2.vertices[i22];

    let tangent = b2CrossSV(1.0, normal);

    let lower1: f32 = 0.0;
    let upper1: f32 = b2Dot(b2Sub(v12, v11), tangent);

    // Incident edge points opposite of tangent due to CCW winding
    let upper2: f32 = b2Dot(b2Sub(v21, v11), tangent);
    let lower2: f32 = b2Dot(b2Sub(v22, v11), tangent);

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

    let separationLower: f32 = b2Dot(b2Sub(vLower, v11), normal);
    let separationUpper: f32 = b2Dot(b2Sub(vUpper, v11), normal);

    let r1: f32 = poly1.radius;
    let r2: f32 = poly2.radius;

    // Put contact points at midpoint, accounting for radii
    vLower = b2MulAdd(vLower, <f32>0.5 * (r1 - r2 - separationLower), normal);
    vUpper = b2MulAdd(vUpper, <f32>0.5 * (r1 - r2 - separationUpper), normal);

    let radius: f32 = r1 + r2;

    if (flip == false) {
        manifold.normal = normal.clone();

        {
            let cp = manifold.points[0];
            cp.anchorA = vLower;
            cp.separation = separationLower - radius;
            cp.id = B2_MAKE_ID(i11, i22);
            manifold.pointCount += 1;
        }

        {
            let cp = manifold.points[1];
            cp.anchorA = vUpper;
            cp.separation = separationUpper - radius;
            cp.id = B2_MAKE_ID(i12, i21);
            manifold.pointCount += 1;
        }
    } else {
        manifold.normal = b2Neg(normal);

        {
            let cp = manifold.points[0];
            cp.anchorA = vUpper;
            cp.separation = separationUpper - radius;
            cp.id = B2_MAKE_ID(i21, i12);
            manifold.pointCount += 1;
        }

        {
            let cp = manifold.points[1];
            cp.anchorA = vLower;
            cp.separation = separationLower - radius;
            cp.id = B2_MAKE_ID(i22, i11);
            manifold.pointCount += 1;
        }
    }

    return manifold;
}

// C returns the edge index through a pointer; the pair travels in a class.
class b2MaxSeparation {
    edgeIndex: i32 = 0;
    separation: f32 = 0;
}

// Scratch for the polygon collide hot path; single threaded and never
// escaping: the world-space conversion leaves only fresh objects in the
// returned manifold.
const s_localPolyA = new b2Polygon();
const s_localPolyB = new b2Polygon();
const s_collideXf = b2Transform_identity();
const s_maxSepOut = new b2MaxSeparation();

// Find the max separation between poly1 and poly2 using edge normals from poly1.
function b2FindMaxSeparation(poly1: b2Polygon, poly2: b2Polygon): b2MaxSeparation {
    let count1 = poly1.count;
    let count2 = poly2.count;
    let n1s = poly1.normals;
    let v1s = poly1.vertices;
    let v2s = poly2.vertices;

    let bestIndex = 0;
    let maxSeparation: f32 = -FLT_MAX;
    for (let i = 0; i < count1; ++i) {
        // Get poly1 normal in frame2.
        let n = unchecked(n1s[i]);
        let v1 = unchecked(v1s[i]);

        // Find the deepest point for normal i.
        let si: f32 = FLT_MAX;
        for (let j = 0; j < count2; ++j) {
            let v2 = unchecked(v2s[j]);
            let sij: f32 = n.x * (v2.x - v1.x) + n.y * (v2.y - v1.y);
            if (sij < si) {
                si = sij;
            }
        }

        if (si > maxSeparation) {
            maxSeparation = si;
            bestIndex = i;
        }
    }

    // Callers extract both fields before the next call.
    let out = s_maxSepOut;
    out.edgeIndex = bestIndex;
    out.separation = maxSeparation;
    return out;
}

// Due to speculation, every polygon is rounded. See the C source for the
// clipping strategy outline.
export function b2CollidePolygons(polygonA: b2Polygon, xfA: b2Transform, polygonB: b2Polygon,
    xfB: b2Transform): b2Manifold {
    let origin = polygonA.vertices[0];
    let linearSlop: f32 = B2_LINEAR_SLOP();
    let speculativeDistance: f32 = B2_SPECULATIVE_DISTANCE();

    // Shift polyA to origin; expressions mirror b2InvMulTransforms of the
    // origin-shifted frame exactly, written into scratch.
    let xf = s_collideXf;
    {
        let qc = xfA.q.c;
        let qs = xfA.q.s;
        let sfpx: f32 = xfA.p.x + (qc * origin.x - qs * origin.y);
        let sfpy: f32 = xfA.p.y + (qs * origin.x + qc * origin.y);
        xf.q.s = qc * xfB.q.s - qs * xfB.q.c;
        xf.q.c = qc * xfB.q.c + qs * xfB.q.s;
        let dx: f32 = xfB.p.x - sfpx;
        let dy: f32 = xfB.p.y - sfpy;
        xf.p.x = qc * dx + qs * dy;
        xf.p.y = -qs * dx + qc * dy;
    }

    let localPolyA = s_localPolyA;
    localPolyA.count = polygonA.count;
    localPolyA.radius = polygonA.radius;
    let va0 = unchecked(localPolyA.vertices[0]);
    va0.x = 0.0;
    va0.y = 0.0;
    let na0 = unchecked(localPolyA.normals[0]);
    na0.x = polygonA.normals[0].x;
    na0.y = polygonA.normals[0].y;
    for (let i = 1; i < localPolyA.count; ++i) {
        let v = unchecked(localPolyA.vertices[i]);
        let src = unchecked(polygonA.vertices[i]);
        v.x = src.x - origin.x;
        v.y = src.y - origin.y;
        let n = unchecked(localPolyA.normals[i]);
        let ns = unchecked(polygonA.normals[i]);
        n.x = ns.x;
        n.y = ns.y;
    }

    // Put polyB in polyA's frame to reduce round-off error
    let localPolyB = s_localPolyB;
    localPolyB.count = polygonB.count;
    localPolyB.radius = polygonB.radius;
    for (let i = 0; i < localPolyB.count; ++i) {
        let v = unchecked(localPolyB.vertices[i]);
        let src = unchecked(polygonB.vertices[i]);
        v.x = (xf.q.c * src.x - xf.q.s * src.y) + xf.p.x;
        v.y = (xf.q.s * src.x + xf.q.c * src.y) + xf.p.y;
        let n = unchecked(localPolyB.normals[i]);
        let ns = unchecked(polygonB.normals[i]);
        n.x = xf.q.c * ns.x - xf.q.s * ns.y;
        n.y = xf.q.s * ns.x + xf.q.c * ns.y;
    }

    let maxSepA = b2FindMaxSeparation(localPolyA, localPolyB);
    let edgeA = maxSepA.edgeIndex;
    let separationA: f32 = maxSepA.separation;

    let maxSepB = b2FindMaxSeparation(localPolyB, localPolyA);
    let edgeB = maxSepB.edgeIndex;
    let separationB: f32 = maxSepB.separation;

    let radius: f32 = localPolyA.radius + localPolyB.radius;

    if (separationA > speculativeDistance + radius || separationB > speculativeDistance + radius) {
        return new b2Manifold();
    }

    // Find incident edge
    let flip: bool;
    if (separationA >= separationB) {
        flip = false;

        let searchDirection = localPolyA.normals[edgeA];

        // Find the incident edge on polyB
        let count = localPolyB.count;
        let normals = localPolyB.normals;
        edgeB = 0;
        let minDot: f32 = FLT_MAX;
        for (let i = 0; i < count; ++i) {
            let dot: f32 = b2Dot(searchDirection, unchecked(normals[i]));
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
        let normals = localPolyA.normals;
        edgeA = 0;
        let minDot: f32 = FLT_MAX;
        for (let i = 0; i < count; ++i) {
            let dot: f32 = b2Dot(searchDirection, unchecked(normals[i]));
            if (dot < minDot) {
                minDot = dot;
                edgeA = i;
            }
        }
    }

    let manifold = new b2Manifold();

    // Using slop here to ensure vertex-vertex normal vectors can be safely normalized
    if (separationA > <f32>0.1 * linearSlop || separationB > <f32>0.1 * linearSlop) {
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

        let result = b2SegmentDistance(v11, v12, v21, v22);
        assert(result.distanceSquared > 0.0);
        let distance: f32 = sqrt<f32>(result.distanceSquared);
        let separation: f32 = distance - radius;

        if (distance - radius > speculativeDistance) {
            // This can happen in the vertex-vertex case
            return manifold;
        }

        // Attempt to clip edges
        manifold = b2ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);

        let minSeparation: f32 = FLT_MAX;
        for (let i = 0; i < manifold.pointCount; ++i) {
            minSeparation = b2MinFloat(minSeparation, manifold.points[i].separation);
        }

        // Does vertex-vertex have substantially larger separation?
        if (separation + <f32>0.1 * linearSlop < minSeparation) {
            if (result.fraction1 == 0.0 && result.fraction2 == 0.0) {
                // v11 - v21
                let normal = b2Sub(v21, v11);
                let invDistance: f32 = <f32>1.0 / distance;
                normal.x *= invDistance;
                normal.y *= invDistance;

                let c1 = b2MulAdd(v11, localPolyA.radius, normal);
                let c2 = b2MulAdd(v21, -localPolyB.radius, normal);

                manifold.normal = normal;
                manifold.points[0].anchorA = b2Lerp(c1, c2, 0.5);
                manifold.points[0].separation = distance - radius;
                manifold.points[0].id = B2_MAKE_ID(i11, i21);
                manifold.pointCount = 1;
            } else if (result.fraction1 == 0.0 && result.fraction2 == 1.0) {
                // v11 - v22
                let normal = b2Sub(v22, v11);
                let invDistance: f32 = <f32>1.0 / distance;
                normal.x *= invDistance;
                normal.y *= invDistance;

                let c1 = b2MulAdd(v11, localPolyA.radius, normal);
                let c2 = b2MulAdd(v22, -localPolyB.radius, normal);

                manifold.normal = normal;
                manifold.points[0].anchorA = b2Lerp(c1, c2, 0.5);
                manifold.points[0].separation = distance - radius;
                manifold.points[0].id = B2_MAKE_ID(i11, i22);
                manifold.pointCount = 1;
            } else if (result.fraction1 == 1.0 && result.fraction2 == 0.0) {
                // v12 - v21
                let normal = b2Sub(v21, v12);
                let invDistance: f32 = <f32>1.0 / distance;
                normal.x *= invDistance;
                normal.y *= invDistance;

                let c1 = b2MulAdd(v12, localPolyA.radius, normal);
                let c2 = b2MulAdd(v21, -localPolyB.radius, normal);

                manifold.normal = normal;
                manifold.points[0].anchorA = b2Lerp(c1, c2, 0.5);
                manifold.points[0].separation = distance - radius;
                manifold.points[0].id = B2_MAKE_ID(i12, i21);
                manifold.pointCount = 1;
            } else if (result.fraction1 == 1.0 && result.fraction2 == 1.0) {
                // v12 - v22
                let normal = b2Sub(v22, v12);
                let invDistance: f32 = <f32>1.0 / distance;
                normal.x *= invDistance;
                normal.y *= invDistance;

                let c1 = b2MulAdd(v12, localPolyA.radius, normal);
                let c2 = b2MulAdd(v22, -localPolyB.radius, normal);

                manifold.normal = normal;
                manifold.points[0].anchorA = b2Lerp(c1, c2, 0.5);
                manifold.points[0].separation = distance - radius;
                manifold.points[0].id = B2_MAKE_ID(i12, i22);
                manifold.pointCount = 1;
            }
        }
    } else {
        // Polygons overlap
        manifold = b2ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);
    }

    // Convert manifold to world space
    if (manifold.pointCount > 0) {
        manifold.normal = b2RotateVector(xfA.q, manifold.normal);
        for (let i = 0; i < manifold.pointCount; ++i) {
            let mp = manifold.points[i];

            // anchor points relative to shape origin in world space
            mp.anchorA = b2RotateVector(xfA.q, b2Add(mp.anchorA, origin));
            mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
            mp.point = b2Add(xfA.p, mp.anchorA);
        }
    }

    return manifold;
}

export function b2CollideSegmentAndCircle(segmentA: b2Segment, xfA: b2Transform, circleB: b2Circle,
    xfB: b2Transform): b2Manifold {
    let capsuleA = new b2Capsule(segmentA.point1.clone(), segmentA.point2.clone(), 0.0);
    return b2CollideCapsuleAndCircle(capsuleA, xfA, circleB, xfB);
}

export function b2CollideSegmentAndPolygon(segmentA: b2Segment, xfA: b2Transform, polygonB: b2Polygon,
    xfB: b2Transform): b2Manifold {
    let polygonA = b2MakeCapsule(segmentA.point1, segmentA.point2, 0.0);
    return b2CollidePolygons(polygonA, xfA, polygonB, xfB);
}

export function b2CollideChainSegmentAndCircle(segmentA: b2ChainSegment, xfA: b2Transform, circleB: b2Circle,
    xfB: b2Transform): b2Manifold {
    let manifold = new b2Manifold();

    let xf = b2InvMulTransforms(xfA, xfB);

    // Compute circle in frame of segment
    let pB = b2TransformPoint(xf, circleB.center);

    let p1 = segmentA.segment.point1;
    let p2 = segmentA.segment.point2;
    let e = b2Sub(p2, p1);

    // Normal points to the right
    let offset: f32 = b2Dot(b2RightPerp(e), b2Sub(pB, p1));
    if (offset < 0.0) {
        // collision is one-sided
        return manifold;
    }

    // Barycentric coordinates
    let u: f32 = b2Dot(e, b2Sub(p2, pB));
    let v: f32 = b2Dot(e, b2Sub(pB, p1));

    let pA: b2Vec2;

    if (v <= 0.0) {
        // Behind point1?
        // Is pB in the Voronoi region of the previous edge?
        let prevEdge = b2Sub(p1, segmentA.ghost1);
        let uPrev: f32 = b2Dot(prevEdge, b2Sub(pB, p1));
        if (uPrev <= 0.0) {
            return manifold;
        }

        pA = p1;
    } else if (u <= 0.0) {
        // Ahead of point2?
        let nextEdge = b2Sub(segmentA.ghost2, p2);
        let vNext: f32 = b2Dot(nextEdge, b2Sub(pB, p2));

        // Is pB in the Voronoi region of the next edge?
        if (vNext > 0.0) {
            return manifold;
        }

        pA = p2;
    } else {
        let ee: f32 = b2Dot(e, e);
        pA = new b2Vec2(u * p1.x + v * p2.x, u * p1.y + v * p2.y);
        pA = ee > 0.0 ? b2MulSV(<f32>1.0 / ee, pA) : p1;
    }

    let ln = b2GetLengthAndNormalize(b2Sub(pB, pA));
    let distance: f32 = ln.length;
    let normal = ln.normal;

    let radius: f32 = circleB.radius;
    let separation: f32 = distance - radius;
    if (separation > B2_SPECULATIVE_DISTANCE()) {
        return manifold;
    }

    let cA = pA;
    let cB = b2MulAdd(pB, -radius, normal);
    let contactPointA = b2Lerp(cA, cB, 0.5);

    manifold.normal = b2RotateVector(xfA.q, normal);

    let mp = manifold.points[0];
    mp.anchorA = b2RotateVector(xfA.q, contactPointA);
    mp.anchorB = b2Add(mp.anchorA, b2Sub(xfA.p, xfB.p));
    mp.point = b2Add(xfA.p, mp.anchorA);
    mp.separation = separation;
    mp.id = 0;
    manifold.pointCount = 1;
    return manifold;
}

export function b2CollideChainSegmentAndCapsule(segmentA: b2ChainSegment, xfA: b2Transform, capsuleB: b2Capsule,
    xfB: b2Transform, cache: b2SimplexCache): b2Manifold {
    let polyB = b2MakeCapsule(capsuleB.center1, capsuleB.center2, capsuleB.radius);
    return b2CollideChainSegmentAndPolygon(segmentA, xfA, polyB, xfB, cache);
}

function b2ClipSegments(a1: b2Vec2, a2: b2Vec2, b1: b2Vec2, b2v: b2Vec2, normal: b2Vec2, ra: f32, rb: f32,
    id1: u16, id2: u16): b2Manifold {
    let manifold = new b2Manifold();

    let tangent = b2LeftPerp(normal);

    // Barycentric coordinates of each point relative to a1 along tangent
    let lower1: f32 = 0.0;
    let upper1: f32 = b2Dot(b2Sub(a2, a1), tangent);

    // Incident edge points opposite of tangent due to CCW winding
    let upper2: f32 = b2Dot(b2Sub(b1, a1), tangent);
    let lower2: f32 = b2Dot(b2Sub(b2v, a1), tangent);

    // Do segments overlap?
    if (upper2 < lower1 || upper1 < lower2) {
        return manifold;
    }

    let vLower: b2Vec2;
    if (lower2 < lower1 && upper2 - lower2 > FLT_EPSILON) {
        vLower = b2Lerp(b2v, b1, (lower1 - lower2) / (upper2 - lower2));
    } else {
        vLower = b2v;
    }

    let vUpper: b2Vec2;
    if (upper2 > upper1 && upper2 - lower2 > FLT_EPSILON) {
        vUpper = b2Lerp(b2v, b1, (upper1 - lower2) / (upper2 - lower2));
    } else {
        vUpper = b1;
    }

    let separationLower: f32 = b2Dot(b2Sub(vLower, a1), normal);
    let separationUpper: f32 = b2Dot(b2Sub(vUpper, a1), normal);

    // Put contact points at midpoint, accounting for radii
    vLower = b2MulAdd(vLower, <f32>0.5 * (ra - rb - separationLower), normal);
    vUpper = b2MulAdd(vUpper, <f32>0.5 * (ra - rb - separationUpper), normal);

    let radius: f32 = ra + rb;

    manifold.normal = normal.clone();
    {
        let cp = manifold.points[0];
        cp.anchorA = vLower;
        cp.separation = separationLower - radius;
        cp.id = id1;
    }

    {
        let cp = manifold.points[1];
        cp.anchorA = vUpper;
        cp.separation = separationUpper - radius;
        cp.id = id2;
    }

    manifold.pointCount = 2;

    return manifold;
}

// enum b2NormalType
// Skip: non-smooth relative to a convex vertex. Admit: smooth, use for
// collision. Snap: concave region, snap to the segment normal.
const b2_normalSkip: i32 = 0;
const b2_normalAdmit: i32 = 1;
const b2_normalSnap: i32 = 2;

class b2ChainSegmentParams {
    edge1: b2Vec2 = new b2Vec2();
    normal0: b2Vec2 = new b2Vec2();
    normal2: b2Vec2 = new b2Vec2();
    convex1: bool = false;
    convex2: bool = false;
}

// Evaluate Gauss map
// See https://box2d.org/posts/2020/06/ghost-collisions/
function b2ClassifyNormal(params: b2ChainSegmentParams, normal: b2Vec2): i32 {
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

export function b2CollideChainSegmentAndPolygon(segmentA: b2ChainSegment, xfA: b2Transform, polygonB: b2Polygon,
    xfB: b2Transform, cache: b2SimplexCache): b2Manifold {
    let manifold = new b2Manifold();

    let xf = b2InvMulTransforms(xfA, xfB);

    let centroidB = b2TransformPoint(xf, polygonB.centroid);
    let radiusB: f32 = polygonB.radius;

    let p1 = segmentA.segment.point1;
    let p2 = segmentA.segment.point2;

    let edge1 = b2Normalize(b2Sub(p2, p1));

    let smoothParams = new b2ChainSegmentParams();
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
    let vertices = makeVecArray(B2_MAX_POLYGON_VERTICES);
    let normals = makeVecArray(B2_MAX_POLYGON_VERTICES);
    for (let i = 0; i < count; ++i) {
        unchecked(vertices[i] = b2TransformPoint(xf, polygonB.vertices[i]));
        unchecked(normals[i] = b2RotateVector(xf.q, polygonB.normals[i]));
    }

    // Distance doesn't work correctly with partial polygons
    let segPts = new StaticArray<b2Vec2>(2);
    segPts[0] = segmentA.segment.point1;
    segPts[1] = segmentA.segment.point2;
    let input = new b2DistanceInput();
    input.proxyA = b2MakeProxy(segPts, 2, 0.0);
    input.proxyB = b2MakeProxy(vertices, count, 0.0);
    input.transformA = b2Transform_identity();
    input.transformB = b2Transform_identity();
    input.useRadii = false;

    let output = b2ShapeDistance(input, cache);

    if (output.distance > radiusB + B2_SPECULATIVE_DISTANCE()) {
        return manifold;
    }

    // Snap concave normals for partial polygon
    let n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
    let n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;

    // Index of incident vertex on polygon
    let incidentIndex = -1;
    let incidentNormal = -1;

    if (behind1 == false && output.distance > <f32>0.1 * B2_LINEAR_SLOP()) {
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
                manifold.normal = b2RotateVector(xfA.q, normal);
                let cp = manifold.points[0];
                cp.anchorA = b2RotateVector(xfA.q, pA);
                cp.anchorB = b2Add(cp.anchorA, b2Sub(xfA.p, xfB.p));
                cp.point = b2Add(xfA.p, cp.anchorA);
                cp.separation = output.distance - radiusB;
                cp.id = B2_MAKE_ID(<i32>cache.indexA[0], <i32>cache.indexB[0]);
                manifold.pointCount = 1;
                return manifold;
            }

            // fall through b2_normalSnap
            incidentIndex = <i32>cache.indexB[0];
        } else {
            // vertex-edge collision
            assert(cache.count == 2);

            let ia1 = <i32>cache.indexA[0];
            let ia2 = <i32>cache.indexA[1];
            let ib1 = <i32>cache.indexB[0];
            let ib2 = <i32>cache.indexB[1];

            if (ia1 == ia2) {
                // 1 point on A, expect 2 points on B
                assert(ib1 != ib2);

                // Find polygon normal most aligned with vector between closest points.
                // This effectively sorts ib1 and ib2
                let normalB = b2Sub(output.pointA, output.pointB);
                let dot1: f32 = b2Dot(normalB, normals[ib1]);
                let dot2: f32 = b2Dot(normalB, normals[ib2]);
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
                    let b2v = vertices[ib2];

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

                    manifold = b2ClipSegments(b1, b2v, p1, p2, normalB, radiusB, 0.0,
                        B2_MAKE_ID(ib1, 1), B2_MAKE_ID(ib2, 0));

                    assert(manifold.pointCount == 0 || manifold.pointCount == 2);
                    if (manifold.pointCount == 2) {
                        manifold.normal = b2RotateVector(xfA.q, b2Neg(normalB));
                        manifold.points[0].anchorA = b2RotateVector(xfA.q, manifold.points[0].anchorA);
                        manifold.points[1].anchorA = b2RotateVector(xfA.q, manifold.points[1].anchorA);
                        let pAB = b2Sub(xfA.p, xfB.p);
                        manifold.points[0].anchorB = b2Add(manifold.points[0].anchorA, pAB);
                        manifold.points[1].anchorB = b2Add(manifold.points[1].anchorA, pAB);
                        manifold.points[0].point = b2Add(xfA.p, manifold.points[0].anchorA);
                        manifold.points[1].point = b2Add(xfA.p, manifold.points[1].anchorA);
                    }
                    return manifold;
                }

                // fall through b2_normalSnap
                incidentNormal = ib;
            } else {
                // Get index of incident polygonB vertex
                let dot1: f32 = b2Dot(normal1, b2Sub(vertices[ib1], p1));
                let dot2: f32 = b2Dot(normal1, b2Sub(vertices[ib2], p2));
                incidentIndex = dot1 < dot2 ? ib1 : ib2;
            }
        }
    } else {
        // SAT edge normal
        let edgeSeparation: f32 = FLT_MAX;

        for (let i = 0; i < count; ++i) {
            let s: f32 = b2Dot(normal1, b2Sub(unchecked(vertices[i]), p1));
            if (s < edgeSeparation) {
                edgeSeparation = s;
                incidentIndex = i;
            }
        }

        // Check convex neighbor for edge separation
        if (smoothParams.convex1) {
            let s0: f32 = FLT_MAX;

            for (let i = 0; i < count; ++i) {
                let s: f32 = b2Dot(smoothParams.normal0, b2Sub(unchecked(vertices[i]), p1));
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
            let s2: f32 = FLT_MAX;

            for (let i = 0; i < count; ++i) {
                let s: f32 = b2Dot(smoothParams.normal2, b2Sub(unchecked(vertices[i]), p2));
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
        let polygonSeparation: f32 = -FLT_MAX;
        let referenceIndex = -1;

        for (let i = 0; i < count; ++i) {
            let n = unchecked(normals[i]);

            let type = b2ClassifyNormal(smoothParams, b2Neg(n));
            if (type != b2_normalAdmit) {
                continue;
            }

            let p = unchecked(vertices[i]);
            let s: f32 = b2MinFloat(b2Dot(n, b2Sub(p2, p)), b2Dot(n, b2Sub(p1, p)));

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

            let dot1: f32 = b2Dot(n, b2Sub(p1, a1));
            let dot2: f32 = b2Dot(n, b2Sub(p2, a1));

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

            manifold = b2ClipSegments(a1, a2, p1, p2, normals[ia1], radiusB, 0.0,
                B2_MAKE_ID(ia1, 1), B2_MAKE_ID(ia2, 0));

            assert(manifold.pointCount == 0 || manifold.pointCount == 2);
            if (manifold.pointCount == 2) {
                manifold.normal = b2RotateVector(xfA.q, b2Neg(normals[ia1]));
                manifold.points[0].anchorA = b2RotateVector(xfA.q, manifold.points[0].anchorA);
                manifold.points[1].anchorA = b2RotateVector(xfA.q, manifold.points[1].anchorA);
                let pAB = b2Sub(xfA.p, xfB.p);
                manifold.points[0].anchorB = b2Add(manifold.points[0].anchorA, pAB);
                manifold.points[1].anchorB = b2Add(manifold.points[1].anchorA, pAB);
                manifold.points[0].point = b2Add(xfA.p, manifold.points[0].anchorA);
                manifold.points[1].point = b2Add(xfA.p, manifold.points[1].anchorA);
            }

            return manifold;
        }

        if (incidentIndex == -1) {
            // neighboring segment is the separating axis
            return manifold;
        }

        // fall through segment normal axis
    }

    assert(incidentNormal != -1 || incidentIndex != -1);

    // Segment normal

    // Find incident polygon normal: normal adjacent to deepest vertex that is most anti-parallel to segment normal
    let b1: b2Vec2;
    let b2v: b2Vec2;
    let ib1: i32;
    let ib2: i32;

    if (incidentNormal != -1) {
        ib1 = incidentNormal;
        ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
        b1 = vertices[ib1];
        b2v = vertices[ib2];
    } else {
        let i2 = incidentIndex;
        let i1 = i2 > 0 ? i2 - 1 : count - 1;
        let d1: f32 = b2Dot(normal1, normals[i1]);
        let d2: f32 = b2Dot(normal1, normals[i2]);
        if (d1 < d2) {
            ib1 = i1;
            ib2 = i2;
            b1 = vertices[ib1];
            b2v = vertices[ib2];
        } else {
            ib1 = i2;
            ib2 = i2 < count - 1 ? i2 + 1 : 0;
            b1 = vertices[ib1];
            b2v = vertices[ib2];
        }
    }

    manifold = b2ClipSegments(p1, p2, b1, b2v, normal1, 0.0, radiusB, B2_MAKE_ID(0, ib2), B2_MAKE_ID(1, ib1));

    assert(manifold.pointCount == 0 || manifold.pointCount == 2);
    if (manifold.pointCount == 2) {
        manifold.normal = b2RotateVector(xfA.q, manifold.normal);
        manifold.points[0].anchorA = b2RotateVector(xfA.q, manifold.points[0].anchorA);
        manifold.points[1].anchorA = b2RotateVector(xfA.q, manifold.points[1].anchorA);
        let pAB = b2Sub(xfA.p, xfB.p);
        manifold.points[0].anchorB = b2Add(manifold.points[0].anchorA, pAB);
        manifold.points[1].anchorB = b2Add(manifold.points[1].anchorA, pAB);
        manifold.points[0].point = b2Add(xfA.p, manifold.points[0].anchorA);
        manifold.points[1].point = b2Add(xfA.p, manifold.points[1].anchorA);
    }

    return manifold;
}
