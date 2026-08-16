// Ported from box2d include/box2d/math_functions.h + src/math_functions.c.
// C structs pass by value; here every function returns a fresh object and
// never mutates its inputs. All arithmetic is f32 to match box2d exactly.

export const B2_PI: f32 = 3.14159265359;

// f32 FLT_EPSILON / FLT_MAX from <float.h>.
export const FLT_EPSILON: f32 = 1.1920928955078125e-7;
export const FLT_MAX: f32 = 3.40282346638528859812e38;

export class b2Vec2 {
    x: f32;
    y: f32;

    constructor(x: f32 = 0, y: f32 = 0) {
        this.x = x;
        this.y = y;
    }

    clone(): b2Vec2 {
        return new b2Vec2(this.x, this.y);
    }
}

export class b2CosSin {
    cosine: f32;
    sine: f32;

    constructor(cosine: f32 = 1, sine: f32 = 0) {
        this.cosine = cosine;
        this.sine = sine;
    }
}

export class b2Rot {
    c: f32;
    s: f32;

    constructor(c: f32 = 1, s: f32 = 0) {
        this.c = c;
        this.s = s;
    }

    clone(): b2Rot {
        return new b2Rot(this.c, this.s);
    }
}

export class b2Transform {
    p: b2Vec2;
    q: b2Rot;

    constructor(p: b2Vec2 = new b2Vec2(), q: b2Rot = new b2Rot()) {
        this.p = p;
        this.q = q;
    }

    clone(): b2Transform {
        return new b2Transform(this.p.clone(), this.q.clone());
    }
}

export class b2Mat22 {
    cx: b2Vec2;
    cy: b2Vec2;

    constructor(cx: b2Vec2 = new b2Vec2(), cy: b2Vec2 = new b2Vec2()) {
        this.cx = cx;
        this.cy = cy;
    }
}

export class b2AABB {
    lowerBound: b2Vec2;
    upperBound: b2Vec2;

    constructor(lowerBound: b2Vec2 = new b2Vec2(), upperBound: b2Vec2 = new b2Vec2()) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    clone(): b2AABB {
        return new b2AABB(this.lowerBound.clone(), this.upperBound.clone());
    }
}

export class b2Plane {
    normal: b2Vec2;
    offset: f32;

    constructor(normal: b2Vec2 = new b2Vec2(), offset: f32 = 0) {
        this.normal = normal;
        this.offset = offset;
    }
}

// C exposes these as shared const statics; fresh instances keep them immutable.
export function b2Vec2_zero(): b2Vec2 {
    return new b2Vec2(0, 0);
}

export function b2Rot_identity(): b2Rot {
    return new b2Rot(1, 0);
}

export function b2Transform_identity(): b2Transform {
    return new b2Transform(new b2Vec2(0, 0), new b2Rot(1, 0));
}

export function b2Mat22_zero(): b2Mat22 {
    return new b2Mat22(new b2Vec2(0, 0), new b2Vec2(0, 0));
}

export function b2IsValidFloat(a: f32): bool {
    if (isNaN<f32>(a)) {
        return false;
    }
    if (!isFinite<f32>(a)) {
        return false;
    }
    return true;
}

export function b2IsValidVec2(v: b2Vec2): bool {
    if (isNaN<f32>(v.x) || isNaN<f32>(v.y)) {
        return false;
    }
    if (!isFinite<f32>(v.x) || !isFinite<f32>(v.y)) {
        return false;
    }
    return true;
}

export function b2IsValidRotation(q: b2Rot): bool {
    if (isNaN<f32>(q.s) || isNaN<f32>(q.c)) {
        return false;
    }
    if (!isFinite<f32>(q.s) || !isFinite<f32>(q.c)) {
        return false;
    }
    return b2IsNormalizedRot(q);
}

export function b2IsValidTransform(t: b2Transform): bool {
    if (b2IsValidVec2(t.p) == false) {
        return false;
    }
    return b2IsValidRotation(t.q);
}

export function b2IsValidPlane(a: b2Plane): bool {
    return b2IsValidVec2(a.normal) && b2IsNormalized(a.normal) && b2IsValidFloat(a.offset);
}

export function b2MinInt(a: i32, b: i32): i32 {
    return a < b ? a : b;
}

export function b2MaxInt(a: i32, b: i32): i32 {
    return a > b ? a : b;
}

export function b2AbsInt(a: i32): i32 {
    return a < 0 ? -a : a;
}

export function b2ClampInt(a: i32, lower: i32, upper: i32): i32 {
    return a < lower ? lower : (a > upper ? upper : a);
}

export function b2MinFloat(a: f32, b: f32): f32 {
    return a < b ? a : b;
}

export function b2MaxFloat(a: f32, b: f32): f32 {
    return a > b ? a : b;
}

export function b2AbsFloat(a: f32): f32 {
    return a < 0 ? -a : a;
}

export function b2ClampFloat(a: f32, lower: f32, upper: f32): f32 {
    return a < lower ? lower : (a > upper ? upper : a);
}

// Hand coded for cross-platform determinism; do not swap for libm atan2f.
export function b2Atan2(y: f32, x: f32): f32 {
    // Added check for (0,0) to match atan2f and avoid NaN
    if (x == 0.0 && y == 0.0) {
        return 0.0;
    }

    let ax: f32 = b2AbsFloat(x);
    let ay: f32 = b2AbsFloat(y);
    let mx: f32 = b2MaxFloat(ay, ax);
    let mn: f32 = b2MinFloat(ay, ax);
    let a: f32 = mn / mx;

    // Minimax polynomial approximation to atan(a) on [0,1]
    let s: f32 = a * a;
    let c: f32 = s * a;
    let q: f32 = s * s;
    let r: f32 = <f32>0.024840285 * q + <f32>0.18681418;
    let t: f32 = <f32>-0.094097948 * q - <f32>0.33213072;
    r = r * s + t;
    r = r * c + a;

    // Map to full circle
    if (ay > ax) {
        r = <f32>1.57079637 - r;
    }

    if (x < 0) {
        r = <f32>3.14159274 - r;
    }

    if (y < 0) {
        r = -r;
    }

    return r;
}

// Bhaskara approximation, kept verbatim for cross-platform determinism.
export function b2ComputeCosSin(radians: f32): b2CosSin {
    let x: f32 = b2UnwindAngle(radians);
    let pi2: f32 = B2_PI * B2_PI;

    // cosine needs angle in [-pi/2, pi/2]
    let c: f32;
    if (x < <f32>-0.5 * B2_PI) {
        let y: f32 = x + B2_PI;
        let y2: f32 = y * y;
        c = -(pi2 - <f32>4.0 * y2) / (pi2 + y2);
    } else if (x > <f32>0.5 * B2_PI) {
        let y: f32 = x - B2_PI;
        let y2: f32 = y * y;
        c = -(pi2 - <f32>4.0 * y2) / (pi2 + y2);
    } else {
        let y2: f32 = x * x;
        c = (pi2 - <f32>4.0 * y2) / (pi2 + y2);
    }

    // sine needs angle in [0, pi]
    let s: f32;
    if (x < 0.0) {
        let y: f32 = x + B2_PI;
        s = <f32>-16.0 * y * (B2_PI - y) / (<f32>5.0 * pi2 - <f32>4.0 * y * (B2_PI - y));
    } else {
        s = <f32>16.0 * x * (B2_PI - x) / (<f32>5.0 * pi2 - <f32>4.0 * x * (B2_PI - x));
    }

    let mag: f32 = sqrt<f32>(s * s + c * c);
    let invMag: f32 = mag > 0.0 ? <f32>1.0 / mag : <f32>0.0;
    return new b2CosSin(c * invMag, s * invMag);
}

export function b2ComputeRotationBetweenUnitVectors(v1: b2Vec2, v2: b2Vec2): b2Rot {
    let rot = new b2Rot();
    rot.c = b2Dot(v1, v2);
    rot.s = b2Cross(v1, v2);
    return b2NormalizeRot(rot);
}

export function b2Dot(a: b2Vec2, b: b2Vec2): f32 {
    return a.x * b.x + a.y * b.y;
}

export function b2Cross(a: b2Vec2, b: b2Vec2): f32 {
    return a.x * b.y - a.y * b.x;
}

export function b2CrossVS(v: b2Vec2, s: f32): b2Vec2 {
    return new b2Vec2(s * v.y, -s * v.x);
}

export function b2CrossSV(s: f32, v: b2Vec2): b2Vec2 {
    return new b2Vec2(-s * v.y, s * v.x);
}

export function b2LeftPerp(v: b2Vec2): b2Vec2 {
    return new b2Vec2(-v.y, v.x);
}

export function b2RightPerp(v: b2Vec2): b2Vec2 {
    return new b2Vec2(v.y, -v.x);
}

export function b2Add(a: b2Vec2, b: b2Vec2): b2Vec2 {
    return new b2Vec2(a.x + b.x, a.y + b.y);
}

export function b2Sub(a: b2Vec2, b: b2Vec2): b2Vec2 {
    return new b2Vec2(a.x - b.x, a.y - b.y);
}

export function b2Neg(a: b2Vec2): b2Vec2 {
    return new b2Vec2(-a.x, -a.y);
}

export function b2Lerp(a: b2Vec2, b: b2Vec2, t: f32): b2Vec2 {
    return new b2Vec2((<f32>1.0 - t) * a.x + t * b.x, (<f32>1.0 - t) * a.y + t * b.y);
}

export function b2Mul(a: b2Vec2, b: b2Vec2): b2Vec2 {
    return new b2Vec2(a.x * b.x, a.y * b.y);
}

export function b2MulSV(s: f32, v: b2Vec2): b2Vec2 {
    return new b2Vec2(s * v.x, s * v.y);
}

export function b2MulAdd(a: b2Vec2, s: f32, b: b2Vec2): b2Vec2 {
    return new b2Vec2(a.x + s * b.x, a.y + s * b.y);
}

export function b2MulSub(a: b2Vec2, s: f32, b: b2Vec2): b2Vec2 {
    return new b2Vec2(a.x - s * b.x, a.y - s * b.y);
}

export function b2Abs(a: b2Vec2): b2Vec2 {
    return new b2Vec2(b2AbsFloat(a.x), b2AbsFloat(a.y));
}

export function b2Min(a: b2Vec2, b: b2Vec2): b2Vec2 {
    return new b2Vec2(b2MinFloat(a.x, b.x), b2MinFloat(a.y, b.y));
}

export function b2Max(a: b2Vec2, b: b2Vec2): b2Vec2 {
    return new b2Vec2(b2MaxFloat(a.x, b.x), b2MaxFloat(a.y, b.y));
}

export function b2Clamp(v: b2Vec2, a: b2Vec2, b: b2Vec2): b2Vec2 {
    return new b2Vec2(b2ClampFloat(v.x, a.x, b.x), b2ClampFloat(v.y, a.y, b.y));
}

export function b2Length(v: b2Vec2): f32 {
    return sqrt<f32>(v.x * v.x + v.y * v.y);
}

export function b2Distance(a: b2Vec2, b: b2Vec2): f32 {
    let dx: f32 = b.x - a.x;
    let dy: f32 = b.y - a.y;
    return sqrt<f32>(dx * dx + dy * dy);
}

export function b2Normalize(v: b2Vec2): b2Vec2 {
    let length: f32 = sqrt<f32>(v.x * v.x + v.y * v.y);
    if (length < FLT_EPSILON) {
        return new b2Vec2(0, 0);
    }

    let invLength: f32 = <f32>1.0 / length;
    return new b2Vec2(invLength * v.x, invLength * v.y);
}

export function b2IsNormalized(a: b2Vec2): bool {
    let aa: f32 = b2Dot(a, a);
    return b2AbsFloat(<f32>1.0 - aa) < <f32>100.0 * FLT_EPSILON;
}

// C returns the length through a float*; AS returns the pair.
export class b2LengthAndNormal {
    length: f32 = 0;
    normal: b2Vec2 = new b2Vec2();
}

export function b2GetLengthAndNormalize(v: b2Vec2): b2LengthAndNormal {
    let out = new b2LengthAndNormal();
    out.length = sqrt<f32>(v.x * v.x + v.y * v.y);
    if (out.length < FLT_EPSILON) {
        return out;
    }

    let invLength: f32 = <f32>1.0 / out.length;
    out.normal = new b2Vec2(invLength * v.x, invLength * v.y);
    return out;
}

export function b2NormalizeRot(q: b2Rot): b2Rot {
    let mag: f32 = sqrt<f32>(q.s * q.s + q.c * q.c);
    let invMag: f32 = mag > 0.0 ? <f32>1.0 / mag : <f32>0.0;
    return new b2Rot(q.c * invMag, q.s * invMag);
}

export function b2IntegrateRotation(q1: b2Rot, deltaAngle: f32): b2Rot {
    let q2c: f32 = q1.c - deltaAngle * q1.s;
    let q2s: f32 = q1.s + deltaAngle * q1.c;
    let mag: f32 = sqrt<f32>(q2s * q2s + q2c * q2c);
    let invMag: f32 = mag > 0.0 ? <f32>1.0 / mag : <f32>0.0;
    return new b2Rot(q2c * invMag, q2s * invMag);
}

export function b2LengthSquared(v: b2Vec2): f32 {
    return v.x * v.x + v.y * v.y;
}

export function b2DistanceSquared(a: b2Vec2, b: b2Vec2): f32 {
    let cx: f32 = b.x - a.x;
    let cy: f32 = b.y - a.y;
    return cx * cx + cy * cy;
}

export function b2MakeRot(radians: f32): b2Rot {
    let cs = b2ComputeCosSin(radians);
    return new b2Rot(cs.cosine, cs.sine);
}

export function b2MakeRotFromUnitVector(unitVector: b2Vec2): b2Rot {
    return new b2Rot(unitVector.x, unitVector.y);
}

export function b2IsNormalizedRot(q: b2Rot): bool {
    // larger tolerance due to failure on mingw 32-bit
    let qq: f32 = q.s * q.s + q.c * q.c;
    return <f32>1.0 - <f32>0.0006 < qq && qq < <f32>1.0 + <f32>0.0006;
}

export function b2InvertRot(a: b2Rot): b2Rot {
    return new b2Rot(a.c, -a.s);
}

export function b2NLerp(q1: b2Rot, q2: b2Rot, t: f32): b2Rot {
    let omt: f32 = <f32>1.0 - t;
    let qc: f32 = omt * q1.c + t * q2.c;
    let qs: f32 = omt * q1.s + t * q2.s;

    let mag: f32 = sqrt<f32>(qs * qs + qc * qc);
    let invMag: f32 = mag > 0.0 ? <f32>1.0 / mag : <f32>0.0;
    return new b2Rot(qc * invMag, qs * invMag);
}

export function b2ComputeAngularVelocity(q1: b2Rot, q2: b2Rot, inv_h: f32): f32 {
    return inv_h * (q2.s * q1.c - q2.c * q1.s);
}

export function b2Rot_GetAngle(q: b2Rot): f32 {
    return b2Atan2(q.s, q.c);
}

export function b2Rot_GetXAxis(q: b2Rot): b2Vec2 {
    return new b2Vec2(q.c, q.s);
}

export function b2Rot_GetYAxis(q: b2Rot): b2Vec2 {
    return new b2Vec2(-q.s, q.c);
}

export function b2MulRot(q: b2Rot, r: b2Rot): b2Rot {
    let out = new b2Rot();
    out.s = q.s * r.c + q.c * r.s;
    out.c = q.c * r.c - q.s * r.s;
    return out;
}

export function b2InvMulRot(a: b2Rot, b: b2Rot): b2Rot {
    let out = new b2Rot();
    out.s = a.c * b.s - a.s * b.c;
    out.c = a.c * b.c + a.s * b.s;
    return out;
}

export function b2RelativeAngle(a: b2Rot, b: b2Rot): f32 {
    let s: f32 = a.c * b.s - a.s * b.c;
    let c: f32 = a.c * b.c + a.s * b.s;
    return b2Atan2(s, c);
}

// remainderf; NativeMathf.rem is the musl remquof port, IEEE-exact.
export function b2UnwindAngle(radians: f32): f32 {
    return NativeMathf.rem(radians, <f32>2.0 * B2_PI);
}

export function b2RotateVector(q: b2Rot, v: b2Vec2): b2Vec2 {
    return new b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

export function b2InvRotateVector(q: b2Rot, v: b2Vec2): b2Vec2 {
    return new b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

export function b2TransformPoint(t: b2Transform, p: b2Vec2): b2Vec2 {
    let x: f32 = (t.q.c * p.x - t.q.s * p.y) + t.p.x;
    let y: f32 = (t.q.s * p.x + t.q.c * p.y) + t.p.y;
    return new b2Vec2(x, y);
}

export function b2InvTransformPoint(t: b2Transform, p: b2Vec2): b2Vec2 {
    let vx: f32 = p.x - t.p.x;
    let vy: f32 = p.y - t.p.y;
    return new b2Vec2(t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy);
}

export function b2MulTransforms(A: b2Transform, B: b2Transform): b2Transform {
    let C = new b2Transform();
    C.q = b2MulRot(A.q, B.q);
    C.p = b2Add(b2RotateVector(A.q, B.p), A.p);
    return C;
}

export function b2InvMulTransforms(A: b2Transform, B: b2Transform): b2Transform {
    let C = new b2Transform();
    C.q = b2InvMulRot(A.q, B.q);
    C.p = b2InvRotateVector(A.q, b2Sub(B.p, A.p));
    return C;
}

export function b2MulMV(A: b2Mat22, v: b2Vec2): b2Vec2 {
    return new b2Vec2(A.cx.x * v.x + A.cy.x * v.y, A.cx.y * v.x + A.cy.y * v.y);
}

export function b2GetInverse22(A: b2Mat22): b2Mat22 {
    let a: f32 = A.cx.x;
    let b: f32 = A.cy.x;
    let c: f32 = A.cx.y;
    let d: f32 = A.cy.y;
    let det: f32 = a * d - b * c;
    if (det != 0.0) {
        det = <f32>1.0 / det;
    }

    return new b2Mat22(new b2Vec2(det * d, -det * c), new b2Vec2(-det * b, det * a));
}

export function b2Solve22(A: b2Mat22, b: b2Vec2): b2Vec2 {
    let a11: f32 = A.cx.x;
    let a12: f32 = A.cy.x;
    let a21: f32 = A.cx.y;
    let a22: f32 = A.cy.y;
    let det: f32 = a11 * a22 - a12 * a21;
    if (det != 0.0) {
        det = <f32>1.0 / det;
    }
    return new b2Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
}

export function b2AABB_Contains(a: b2AABB, b: b2AABB): bool {
    let s = true;
    s = s && a.lowerBound.x <= b.lowerBound.x;
    s = s && a.lowerBound.y <= b.lowerBound.y;
    s = s && b.upperBound.x <= a.upperBound.x;
    s = s && b.upperBound.y <= a.upperBound.y;
    return s;
}

export function b2AABB_Center(a: b2AABB): b2Vec2 {
    return new b2Vec2(<f32>0.5 * (a.lowerBound.x + a.upperBound.x), <f32>0.5 * (a.lowerBound.y + a.upperBound.y));
}

export function b2AABB_Extents(a: b2AABB): b2Vec2 {
    return new b2Vec2(<f32>0.5 * (a.upperBound.x - a.lowerBound.x), <f32>0.5 * (a.upperBound.y - a.lowerBound.y));
}

export function b2AABB_Union(a: b2AABB, b: b2AABB): b2AABB {
    let c = new b2AABB();
    c.lowerBound.x = b2MinFloat(a.lowerBound.x, b.lowerBound.x);
    c.lowerBound.y = b2MinFloat(a.lowerBound.y, b.lowerBound.y);
    c.upperBound.x = b2MaxFloat(a.upperBound.x, b.upperBound.x);
    c.upperBound.y = b2MaxFloat(a.upperBound.y, b.upperBound.y);
    return c;
}

export function b2AABB_Overlaps(a: b2AABB, b: b2AABB): bool {
    return !(b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x ||
        a.lowerBound.y > b.upperBound.y);
}

export function b2MakeAABB(points: StaticArray<b2Vec2>, count: i32, radius: f32): b2AABB {
    let a = new b2AABB(points[0].clone(), points[0].clone());
    for (let i = 1; i < count; ++i) {
        a.lowerBound = b2Min(a.lowerBound, points[i]);
        a.upperBound = b2Max(a.upperBound, points[i]);
    }

    let r = new b2Vec2(radius, radius);
    a.lowerBound = b2Sub(a.lowerBound, r);
    a.upperBound = b2Add(a.upperBound, r);

    return a;
}

export function b2PlaneSeparation(plane: b2Plane, point: b2Vec2): f32 {
    return b2Dot(plane.normal, point) - plane.offset;
}

export function b2SpringDamper(hertz: f32, dampingRatio: f32, position: f32, velocity: f32, timeStep: f32): f32 {
    let omega: f32 = <f32>2.0 * B2_PI * hertz;
    let omegaH: f32 = omega * timeStep;
    return (velocity - omega * omegaH * position) / (<f32>1.0 + <f32>2.0 * dampingRatio * omegaH + omegaH * omegaH);
}
