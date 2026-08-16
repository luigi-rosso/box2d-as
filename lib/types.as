// Ported from box2d include/box2d/types.h and src/types.c. Definition structs
// and their default factories. Joint defs live in joint.as like C's joint.c.

import { b2Vec2, b2Rot, b2Transform } from "./math_functions";
import { b2_lengthUnitsPerMeter, B2_SECRET_COOKIE } from "./core";

export const B2_DEFAULT_CATEGORY_BITS: u64 = 1;
export const B2_DEFAULT_MASK_BITS: u64 = 0xffffffffffffffff;

// b2BodyType lives in broad_phase.as (Phase B), re-exported here for callers.
export { b2_staticBody, b2_kinematicBody, b2_dynamicBody, b2_bodyTypeCount } from "./broad_phase";

// b2ShapeType
export const b2_circleShape: i32 = 0;
export const b2_capsuleShape: i32 = 1;
export const b2_segmentShape: i32 = 2;
export const b2_polygonShape: i32 = 3;
export const b2_chainSegmentShape: i32 = 4;
export const b2_shapeTypeCount: i32 = 5;

// b2JointType
export const b2_distanceJoint: i32 = 0;
export const b2_filterJoint: i32 = 1;
export const b2_motorJoint: i32 = 2;
export const b2_prismaticJoint: i32 = 3;
export const b2_revoluteJoint: i32 = 4;
export const b2_weldJoint: i32 = 5;
export const b2_wheelJoint: i32 = 6;

export class b2WorldDef {
    gravity: b2Vec2 = new b2Vec2();
    restitutionThreshold: f32 = 0;
    hitEventThreshold: f32 = 0;
    contactHertz: f32 = 0;
    contactDampingRatio: f32 = 0;
    contactSpeed: f32 = 0;
    maximumLinearSpeed: f32 = 0;
    enableSleep: bool = false;
    enableContinuous: bool = false;
    enableContactSoftening: bool = false;
    workerCount: i32 = 0;
    userData: u64 = 0;
    internalValue: i32 = 0;
}

export function b2DefaultWorldDef(): b2WorldDef {
    let def = new b2WorldDef();
    def.gravity.x = 0.0;
    def.gravity.y = -10.0;
    def.hitEventThreshold = <f32>1.0 * b2_lengthUnitsPerMeter;
    def.restitutionThreshold = <f32>1.0 * b2_lengthUnitsPerMeter;
    def.contactSpeed = <f32>3.0 * b2_lengthUnitsPerMeter;
    def.contactHertz = 30.0;
    def.contactDampingRatio = 10.0;
    // 400 meters per second, faster than the speed of sound
    def.maximumLinearSpeed = <f32>400.0 * b2_lengthUnitsPerMeter;
    def.enableSleep = true;
    def.enableContinuous = true;
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2MotionLocks {
    linearX: bool = false;
    linearY: bool = false;
    angularZ: bool = false;
}

export class b2BodyDef {
    type: i32 = 0;
    position: b2Vec2 = new b2Vec2();
    rotation: b2Rot = new b2Rot();
    linearVelocity: b2Vec2 = new b2Vec2();
    angularVelocity: f32 = 0;
    linearDamping: f32 = 0;
    angularDamping: f32 = 0;
    gravityScale: f32 = 0;
    sleepThreshold: f32 = 0;
    name: string = "";
    userData: u64 = 0;
    motionLocks: b2MotionLocks = new b2MotionLocks();
    enableSleep: bool = false;
    isAwake: bool = false;
    isBullet: bool = false;
    isEnabled: bool = false;
    allowFastRotation: bool = false;
    internalValue: i32 = 0;
}

export function b2DefaultBodyDef(): b2BodyDef {
    let def = new b2BodyDef();
    def.type = 0; // b2_staticBody
    def.rotation = new b2Rot(1, 0);
    def.sleepThreshold = <f32>0.05 * b2_lengthUnitsPerMeter;
    def.gravityScale = 1.0;
    def.enableSleep = true;
    def.isAwake = true;
    def.isEnabled = true;
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2Filter {
    categoryBits: u64 = 0;
    maskBits: u64 = 0;
    groupIndex: i32 = 0;

    clone(): b2Filter {
        let f = new b2Filter();
        f.categoryBits = this.categoryBits;
        f.maskBits = this.maskBits;
        f.groupIndex = this.groupIndex;
        return f;
    }
}

export function b2DefaultFilter(): b2Filter {
    let filter = new b2Filter();
    filter.categoryBits = B2_DEFAULT_CATEGORY_BITS;
    filter.maskBits = B2_DEFAULT_MASK_BITS;
    filter.groupIndex = 0;
    return filter;
}

export class b2QueryFilter {
    categoryBits: u64 = 0;
    maskBits: u64 = 0;
}

export function b2DefaultQueryFilter(): b2QueryFilter {
    let filter = new b2QueryFilter();
    filter.categoryBits = B2_DEFAULT_CATEGORY_BITS;
    filter.maskBits = B2_DEFAULT_MASK_BITS;
    return filter;
}

export class b2SurfaceMaterial {
    friction: f32 = 0;
    restitution: f32 = 0;
    rollingResistance: f32 = 0;
    tangentSpeed: f32 = 0;
    userMaterialId: u64 = 0;
    customColor: u32 = 0;

    clone(): b2SurfaceMaterial {
        let m = new b2SurfaceMaterial();
        m.friction = this.friction;
        m.restitution = this.restitution;
        m.rollingResistance = this.rollingResistance;
        m.tangentSpeed = this.tangentSpeed;
        m.userMaterialId = this.userMaterialId;
        m.customColor = this.customColor;
        return m;
    }
}

export function b2DefaultSurfaceMaterial(): b2SurfaceMaterial {
    let material = new b2SurfaceMaterial();
    material.friction = 0.6;
    return material;
}

export class b2ShapeDef {
    userData: u64 = 0;
    material: b2SurfaceMaterial = new b2SurfaceMaterial();
    density: f32 = 0;
    filter: b2Filter = new b2Filter();
    enableCustomFiltering: bool = false;
    isSensor: bool = false;
    enableSensorEvents: bool = false;
    enableContactEvents: bool = false;
    enableHitEvents: bool = false;
    enablePreSolveEvents: bool = false;
    invokeContactCreation: bool = false;
    updateBodyMass: bool = false;
    internalValue: i32 = 0;
}

export function b2DefaultShapeDef(): b2ShapeDef {
    let def = new b2ShapeDef();
    def.material.friction = 0.6;
    def.density = 1.0;
    def.filter = b2DefaultFilter();
    def.updateBodyMass = true;
    def.invokeContactCreation = true;
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2ChainDef {
    userData: u64 = 0;
    points: Array<b2Vec2> = new Array<b2Vec2>();
    count: i32 = 0;
    materials: Array<b2SurfaceMaterial> = new Array<b2SurfaceMaterial>();
    materialCount: i32 = 0;
    filter: b2Filter = new b2Filter();
    isLoop: bool = false;
    enableSensorEvents: bool = false;
    internalValue: i32 = 0;
}

export function b2DefaultChainDef(): b2ChainDef {
    let def = new b2ChainDef();
    def.materials.push(b2DefaultSurfaceMaterial());
    def.materialCount = 1;
    def.filter = b2DefaultFilter();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

export class b2ExplosionDef {
    maskBits: u64 = 0;
    position: b2Vec2 = new b2Vec2();
    radius: f32 = 0;
    falloff: f32 = 0;
    impulsePerLength: f32 = 0;
}

export function b2DefaultExplosionDef(): b2ExplosionDef {
    let def = new b2ExplosionDef();
    def.maskBits = B2_DEFAULT_MASK_BITS;
    return def;
}
