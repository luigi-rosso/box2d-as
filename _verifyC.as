// Phase C verification entry: builds the same dynamics scenes as
// cppref/ref_dynamics.c, steps them, and exposes per-step body states and
// bookkeeping hashes for the node comparison. Scene construction and the
// FNV-1a hash streams mirror the C generator statement for statement.

import { b2Vec2, b2Rot, b2Transform, b2MakeRot } from "./lib/math_functions";
import { b2Circle, b2Capsule, b2Polygon } from "./lib/collision";
import { b2MakeBox, b2MakeOffsetBox } from "./lib/geometry";
import { b2WorldId, b2BodyId, b2JointId } from "./lib/id";
import {
    b2DefaultWorldDef, b2DefaultBodyDef, b2DefaultShapeDef, b2_dynamicBody,
} from "./lib/types";
import {
    b2World, b2CreateWorld, b2DestroyWorld, b2World_Step, b2GetWorldFromId,
    b2World_GetBodyEvents, b2World_GetContactEvents, b2World_GetJointEvents,
    b2_awakeSet,
} from "./lib/physics_world";
import { b2CreateBody, b2DestroyBody, b2Body_IsValid, b2Body_IsAwake, b2Body_SetAwake } from "./lib/body";
import { b2CreatePolygonShape, b2CreateCircleShape, b2CreateCapsuleShape } from "./lib/shape";
import {
    b2DefaultDistanceJointDef, b2DefaultRevoluteJointDef, b2DefaultPrismaticJointDef,
    b2DefaultWeldJointDef, b2DefaultMotorJointDef, b2DefaultWheelJointDef,
    b2CreateDistanceJoint, b2CreateRevoluteJoint, b2CreatePrismaticJoint, b2CreateWeldJoint,
    b2CreateMotorJoint, b2CreateWheelJoint, b2DestroyJoint,
} from "./lib/joint";
import { b2Joint_IsValid } from "./lib/physics_world";
import { b2GetContactSim, b2_contactTouchingFlag, b2_contactEnableContactEvents } from "./lib/contact";
import { B2_NULL_INDEX } from "./lib/core";
import { B2_GRAPH_COLOR_COUNT } from "./lib/constants";

function B(x: f32): u32 {
    return reinterpret<u32>(x);
}

// FNV-1a 64 over u32 words
function hashInit(): u64 {
    return 14695981039346656037;
}

function hashU32(h: u64, w: u32): u64 {
    for (let i = 0; i < 4; ++i) {
        h ^= <u64>((w >> (8 * i)) & 0xff);
        h *= 1099511628211;
    }
    return h;
}

function hashI32(h: u64, w: i32): u64 {
    return hashU32(h, <u32>w);
}

function hashF32(h: u64, x: f32): u64 {
    return hashU32(h, B(x));
}

// ---------------------------------------------------------------------------
// deterministic rng (mirrors ref_dynamics.c)
// ---------------------------------------------------------------------------

let g_rng: u32 = 42;

function nextRand(): u32 {
    let x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}

function randFloat(): f32 {
    return <f32>(nextRand() >> 8) * (<f32>1.0 / <f32>16777216.0);
}

// ---------------------------------------------------------------------------
// scene builders (mirror ref_dynamics.c)
// ---------------------------------------------------------------------------

let g_worldId = new b2WorldId(0, 0);
let g_joints = new Array<b2JointId>();

function makeGround(worldId: b2WorldId, halfWidth: f32): b2BodyId {
    let bodyDef = b2DefaultBodyDef();
    bodyDef.position = new b2Vec2(0.0, -1.0);
    let groundId = b2CreateBody(worldId, bodyDef);

    let shapeDef = b2DefaultShapeDef();
    let box = b2MakeBox(halfWidth, 1.0);
    b2CreatePolygonShape(groundId, shapeDef, box);
    return groundId;
}

function buildPyramid(worldId: b2WorldId, rows: i32): void {
    makeGround(worldId, 40.0);

    let bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;

    let shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0;
    shapeDef.material.friction = 0.6;

    let h: f32 = 0.5;
    let box = b2MakeBox(h, h);

    let shift: f32 = <f32>1.0 * h;

    for (let i = 0; i < rows; ++i) {
        let y = (<f32>2.0 * <f32>i + <f32>1.0) * shift;

        for (let j = i; j < rows; ++j) {
            let x = (<f32>i + <f32>1.0) * shift + <f32>2.0 * <f32>(j - i) * shift - h * <f32>rows;

            bodyDef.position = new b2Vec2(x, y);
            let bodyId = b2CreateBody(worldId, bodyDef);
            b2CreatePolygonShape(bodyId, shapeDef, box);
        }
    }
}

function buildRain(worldId: b2WorldId, count: i32): void {
    let bodyDef = b2DefaultBodyDef();
    bodyDef.position = new b2Vec2(0.0, -1.0);
    let groundId = b2CreateBody(worldId, bodyDef);
    let shapeDef = b2DefaultShapeDef();
    let floor = b2MakeBox(10.0, 1.0);
    b2CreatePolygonShape(groundId, shapeDef, floor);

    bodyDef.position = new b2Vec2(-10.0, 5.0);
    let leftId = b2CreateBody(worldId, bodyDef);
    let wall = b2MakeBox(1.0, 6.0);
    b2CreatePolygonShape(leftId, shapeDef, wall);

    bodyDef.position = new b2Vec2(10.0, 5.0);
    let rightId = b2CreateBody(worldId, bodyDef);
    b2CreatePolygonShape(rightId, shapeDef, wall);

    let dynDef = b2DefaultBodyDef();
    dynDef.type = b2_dynamicBody;

    for (let i = 0; i < count; ++i) {
        let x = <f32>-8.0 + <f32>16.0 * randFloat();
        let y = <f32>2.0 + <f32>20.0 * randFloat();
        dynDef.position = new b2Vec2(x, y);
        dynDef.angularVelocity = <f32>2.0 * randFloat() - <f32>1.0;
        dynDef.linearVelocity = new b2Vec2(<f32>2.0 * randFloat() - <f32>1.0, 0.0);
        let bodyId = b2CreateBody(worldId, dynDef);

        let sd = b2DefaultShapeDef();
        sd.density = 1.0;
        sd.material.friction = 0.3;
        sd.material.restitution = 0.1;

        let kind = i % 3;
        if (kind == 0) {
            let circle = new b2Circle(new b2Vec2(0.0, 0.0), 0.35);
            b2CreateCircleShape(bodyId, sd, circle);
        } else if (kind == 1) {
            let capsule = new b2Capsule(new b2Vec2(-0.25, 0.0), new b2Vec2(0.25, 0.0), 0.25);
            b2CreateCapsuleShape(bodyId, sd, capsule);
        } else {
            let box = b2MakeBox(0.35, 0.35);
            b2CreatePolygonShape(bodyId, sd, box);
        }
    }
}

function makeDynamicBox(worldId: b2WorldId, x: f32, y: f32, hx: f32, hy: f32): b2BodyId {
    let bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = new b2Vec2(x, y);
    let bodyId = b2CreateBody(worldId, bodyDef);
    let shapeDef = b2DefaultShapeDef();
    let box = b2MakeBox(hx, hy);
    b2CreatePolygonShape(bodyId, shapeDef, box);
    return bodyId;
}

function buildDistanceScene(worldId: b2WorldId): void {
    let groundId = makeGround(worldId, 20.0);

    let prev = groundId;
    for (let i = 0; i < 3; ++i) {
        let bodyId = makeDynamicBox(worldId, <f32>1.0 * <f32>i + <f32>1.0, <f32>6.0 - <f32>1.0 * <f32>i, 0.25, 0.25);

        let jointDef = b2DefaultDistanceJointDef();
        jointDef.base.bodyIdA = prev;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = new b2Vec2(<f32>1.0 * <f32>i, i == 0 ? <f32>8.0 : <f32>0.0);
        jointDef.base.localFrameB.p = new b2Vec2(0.0, 0.25);
        jointDef.length = 1.5;

        if (i == 0) {
            jointDef.enableSpring = true;
            jointDef.hertz = 3.0;
            jointDef.dampingRatio = 0.5;
        } else if (i == 1) {
            jointDef.enableLimit = true;
            jointDef.minLength = 1.0;
            jointDef.maxLength = 2.0;
            jointDef.enableSpring = true;
            jointDef.hertz = 2.0;
            jointDef.dampingRatio = 0.3;
        } else {
            jointDef.enableMotor = true;
            jointDef.maxMotorForce = 50.0;
            jointDef.motorSpeed = 0.5;
        }

        g_joints.push(b2CreateDistanceJoint(worldId, jointDef));
        prev = bodyId;
    }
}

function buildRevoluteScene(worldId: b2WorldId): void {
    let groundId = makeGround(worldId, 20.0);

    let prev = groundId;
    for (let i = 0; i < 3; ++i) {
        let bodyId = makeDynamicBox(worldId, <f32>1.0 * <f32>(i + 1), 6.0, 0.5, 0.125);

        let jointDef = b2DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = prev;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = i == 0 ? new b2Vec2(0.5, 7.0) : new b2Vec2(0.5, 0.0);
        jointDef.base.localFrameB.p = new b2Vec2(-0.5, 0.0);

        if (i == 0) {
            jointDef.enableLimit = true;
            jointDef.lowerAngle = -0.5;
            jointDef.upperAngle = 0.5;
        } else if (i == 1) {
            jointDef.enableMotor = true;
            jointDef.maxMotorTorque = 20.0;
            jointDef.motorSpeed = 1.0;
        } else {
            jointDef.enableSpring = true;
            jointDef.hertz = 2.0;
            jointDef.dampingRatio = 0.2;
            jointDef.targetAngle = 0.25;
        }

        g_joints.push(b2CreateRevoluteJoint(worldId, jointDef));
        prev = bodyId;
    }
}

function buildPrismaticScene(worldId: b2WorldId): void {
    let groundId = makeGround(worldId, 20.0);

    {
        let bodyId = makeDynamicBox(worldId, 0.0, 2.0, 0.5, 0.5);

        let jointDef = b2DefaultPrismaticJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = new b2Vec2(0.0, 3.0);
        jointDef.base.localFrameB.p = new b2Vec2(0.0, 0.0);
        jointDef.enableLimit = true;
        jointDef.lowerTranslation = -2.0;
        jointDef.upperTranslation = 2.0;
        jointDef.enableMotor = true;
        jointDef.maxMotorForce = 30.0;
        jointDef.motorSpeed = 1.5;
        g_joints.push(b2CreatePrismaticJoint(worldId, jointDef));
    }

    {
        let bodyId = makeDynamicBox(worldId, 5.0, 4.0, 0.5, 0.5);

        let jointDef = b2DefaultPrismaticJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = new b2Vec2(5.0, 3.0);
        jointDef.base.localFrameA.q = b2MakeRot(<f32>0.5 * <f32>3.14159265359);
        jointDef.base.localFrameB.p = new b2Vec2(0.0, 0.0);
        jointDef.base.localFrameB.q = b2MakeRot(<f32>0.5 * <f32>3.14159265359);
        jointDef.enableSpring = true;
        jointDef.hertz = 2.0;
        jointDef.dampingRatio = 0.3;
        jointDef.targetTranslation = 1.0;
        jointDef.enableLimit = true;
        jointDef.lowerTranslation = -3.0;
        jointDef.upperTranslation = 3.0;
        g_joints.push(b2CreatePrismaticJoint(worldId, jointDef));
    }
}

function buildWeldScene(worldId: b2WorldId): void {
    let groundId = makeGround(worldId, 20.0);

    let prev = groundId;
    for (let i = 0; i < 4; ++i) {
        let bodyId = makeDynamicBox(worldId, <f32>1.0 * <f32>(i + 1), 6.0, 0.5, 0.125);

        let jointDef = b2DefaultWeldJointDef();
        jointDef.base.bodyIdA = prev;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = i == 0 ? new b2Vec2(0.5, 7.0) : new b2Vec2(0.5, 0.0);
        jointDef.base.localFrameB.p = new b2Vec2(-0.5, 0.0);

        if (i % 2 == 0) {
            jointDef.linearHertz = 5.0;
            jointDef.linearDampingRatio = 0.7;
            jointDef.angularHertz = 1.0;
            jointDef.angularDampingRatio = 0.5;
        }

        g_joints.push(b2CreateWeldJoint(worldId, jointDef));
        prev = bodyId;
    }
}

function buildMotorScene(worldId: b2WorldId): void {
    makeGround(worldId, 20.0);

    let bodyId = makeDynamicBox(worldId, 0.0, 4.0, 0.5, 0.5);

    let jointDef = b2DefaultMotorJointDef();
    let anchorDef = b2DefaultBodyDef();
    anchorDef.position = new b2Vec2(0.0, 4.0);
    let anchorId = b2CreateBody(worldId, anchorDef);

    jointDef.base.bodyIdA = anchorId;
    jointDef.base.bodyIdB = bodyId;
    jointDef.linearVelocity = new b2Vec2(1.0, 0.0);
    jointDef.maxVelocityForce = 20.0;
    jointDef.angularVelocity = 2.0;
    jointDef.maxVelocityTorque = 5.0;
    jointDef.linearHertz = 1.0;
    jointDef.linearDampingRatio = 0.5;
    jointDef.maxSpringForce = 50.0;
    jointDef.angularHertz = 1.0;
    jointDef.angularDampingRatio = 0.5;
    jointDef.maxSpringTorque = 20.0;
    g_joints.push(b2CreateMotorJoint(worldId, jointDef));
}

function buildWheelScene(worldId: b2WorldId): void {
    let groundId = makeGround(worldId, 40.0);

    {
        let shapeDef = b2DefaultShapeDef();
        let bump = b2MakeOffsetBox(1.0, 0.25, new b2Vec2(4.0, 1.0), b2MakeRot(0.0));
        b2CreatePolygonShape(groundId, shapeDef, bump);
    }

    let chassisId = makeDynamicBox(worldId, 0.0, 1.0, 1.0, 0.25);

    for (let i = 0; i < 2; ++i) {
        let x: f32 = i == 0 ? <f32>-0.75 : <f32>0.75;

        let wheelDef = b2DefaultBodyDef();
        wheelDef.type = b2_dynamicBody;
        wheelDef.position = new b2Vec2(x, 0.5);
        wheelDef.allowFastRotation = true;
        let wheelId = b2CreateBody(worldId, wheelDef);
        let sd = b2DefaultShapeDef();
        sd.density = 1.0;
        sd.material.friction = 0.8;
        let circle = new b2Circle(new b2Vec2(0.0, 0.0), 0.3);
        b2CreateCircleShape(wheelId, sd, circle);

        let jointDef = b2DefaultWheelJointDef();
        jointDef.base.bodyIdA = chassisId;
        jointDef.base.bodyIdB = wheelId;
        jointDef.base.localFrameA.p = new b2Vec2(x, -0.5);
        jointDef.base.localFrameA.q = b2MakeRot(<f32>0.5 * <f32>3.14159265359);
        jointDef.base.localFrameB.q = b2MakeRot(<f32>0.5 * <f32>3.14159265359);
        jointDef.enableSpring = true;
        jointDef.hertz = 4.0;
        jointDef.dampingRatio = 0.7;
        jointDef.enableLimit = true;
        jointDef.lowerTranslation = -0.25;
        jointDef.upperTranslation = 0.25;

        if (i == 1) {
            jointDef.enableMotor = true;
            jointDef.maxMotorTorque = 10.0;
            jointDef.motorSpeed = -10.0;
        }

        g_joints.push(b2CreateWheelJoint(worldId, jointDef));
    }
}

function buildBulletScene(worldId: b2WorldId): void {
    makeGround(worldId, 40.0);

    {
        let bodyDef = b2DefaultBodyDef();
        bodyDef.position = new b2Vec2(6.0, 3.0);
        let wallId = b2CreateBody(worldId, bodyDef);
        let shapeDef = b2DefaultShapeDef();
        let wall = b2MakeBox(0.05, 3.0);
        b2CreatePolygonShape(wallId, shapeDef, wall);
    }

    {
        let bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = new b2Vec2(-5.0, 3.0);
        bodyDef.isBullet = true;
        bodyDef.linearVelocity = new b2Vec2(80.0, 0.0);
        bodyDef.gravityScale = 0.0;
        let bulletId = b2CreateBody(worldId, bodyDef);
        let sd = b2DefaultShapeDef();
        sd.density = 5.0;
        let circle = new b2Circle(new b2Vec2(0.0, 0.0), 0.1);
        b2CreateCircleShape(bulletId, sd, circle);
    }

    {
        let bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = new b2Vec2(-5.0, 1.0);
        bodyDef.linearVelocity = new b2Vec2(60.0, 0.0);
        let fastId = b2CreateBody(worldId, bodyDef);
        let sd = b2DefaultShapeDef();
        let box = b2MakeBox(0.25, 0.25);
        b2CreatePolygonShape(fastId, sd, box);
    }
}

function churnAction(worldId: b2WorldId, step: i32): void {
    let world0 = <u16>(worldId.index1 - 1);

    if (step == 60) {
        let id = new b2BodyId(9, world0, 1);
        if (b2Body_IsValid(id)) {
            b2DestroyBody(id);
        }
    } else if (step == 90) {
        let a = new b2BodyId(3, world0, 1);
        let c = new b2BodyId(5, world0, 1);
        if (b2Body_IsValid(a) && b2Body_IsValid(c)) {
            let jointDef = b2DefaultDistanceJointDef();
            jointDef.base.bodyIdA = a;
            jointDef.base.bodyIdB = c;
            jointDef.length = 2.0;
            g_joints.push(b2CreateDistanceJoint(worldId, jointDef));
        }
    } else if (step == 120) {
        let id = new b2BodyId(2, world0, 1);
        if (b2Body_IsValid(id)) {
            b2DestroyBody(id);
        }
    } else if (step == 150) {
        makeDynamicBox(worldId, 0.0, 9.0, 0.5, 0.5);
    } else if (step == 180 && g_joints.length > 0) {
        let jid = g_joints[g_joints.length - 1];
        if (b2Joint_IsValid(jid)) {
            b2DestroyJoint(jid, true);
            g_joints.pop();
        }
    } else if (step == 210) {
        let id = new b2BodyId(5, world0, 1);
        if (b2Body_IsValid(id) && b2Body_IsAwake(id)) {
            b2Body_SetAwake(id, false);
        }
    } else if (step == 240) {
        let id = new b2BodyId(5, world0, 1);
        if (b2Body_IsValid(id)) {
            b2Body_SetAwake(id, true);
        }
    }
}

let g_sceneId = -1;

export function dynStart(sceneId: i32): void {
    g_rng = 42;
    g_joints = new Array<b2JointId>();
    g_sceneId = sceneId;

    let worldDef = b2DefaultWorldDef();
    g_worldId = b2CreateWorld(worldDef);

    if (sceneId == 0) {
        buildPyramid(g_worldId, 5);
    } else if (sceneId == 1) {
        buildPyramid(g_worldId, 10);
    } else if (sceneId == 2) {
        buildPyramid(g_worldId, 15);
    } else if (sceneId == 3) {
        buildRain(g_worldId, 30);
    } else if (sceneId == 4) {
        buildRain(g_worldId, 80);
    } else if (sceneId == 5) {
        buildDistanceScene(g_worldId);
    } else if (sceneId == 6) {
        buildRevoluteScene(g_worldId);
    } else if (sceneId == 7) {
        buildPrismaticScene(g_worldId);
    } else if (sceneId == 8) {
        buildWeldScene(g_worldId);
    } else if (sceneId == 9) {
        buildMotorScene(g_worldId);
    } else if (sceneId == 10) {
        buildWheelScene(g_worldId);
    } else if (sceneId == 11) {
        buildPyramid(g_worldId, 6);
    } else if (sceneId == 12) {
        buildBulletScene(g_worldId);
    }
}

export function dynEnd(): void {
    b2DestroyWorld(g_worldId);
    g_sceneId = -1;
}

// staged per-step results
let g_bodyIds = new Array<i32>();
let g_bodySets = new Array<i32>();
let g_bodyFloats = new Array<u32>();
let g_contactCount = 0;
let g_touchingCount = 0;
let g_contactHash: u64 = 0;
let g_islandHash: u64 = 0;
let g_setHash: u64 = 0;
let g_colorHash: u64 = 0;
let g_bookHash: u64 = 0;
let g_eventCounts = new StaticArray<i32>(5);
let g_eventHash: u64 = 0;

export function dynStep(step: i32): void {
    if (g_sceneId == 11) {
        churnAction(g_worldId, step);
    }

    b2World_Step(g_worldId, <f32>1.0 / <f32>60.0, 4);

    // dump
    let world = b2GetWorldFromId(g_worldId);

    g_bodyIds = new Array<i32>();
    g_bodySets = new Array<i32>();
    g_bodyFloats = new Array<u32>();

    let bookHash = hashInit();
    for (let i = 0; i < world.bodies.count; ++i) {
        let body = world.bodies.get(i);
        if (body.id == B2_NULL_INDEX) {
            continue;
        }

        let set = world.solverSets.get(body.setIndex);
        let sim = set.bodySims.get(body.localIndex);

        let vx: f32 = 0.0;
        let vy: f32 = 0.0;
        let w: f32 = 0.0;
        if (body.setIndex == b2_awakeSet) {
            let state = set.bodyStates.get(body.localIndex);
            vx = state.linearVelocity.x;
            vy = state.linearVelocity.y;
            w = state.angularVelocity;
        }

        g_bodyIds.push(i);
        g_bodySets.push(body.setIndex);
        g_bodyFloats.push(B(sim.transform.p.x));
        g_bodyFloats.push(B(sim.transform.p.y));
        g_bodyFloats.push(B(sim.transform.q.c));
        g_bodyFloats.push(B(sim.transform.q.s));
        g_bodyFloats.push(B(vx));
        g_bodyFloats.push(B(vy));
        g_bodyFloats.push(B(w));

        bookHash = hashI32(bookHash, body.id);
        bookHash = hashI32(bookHash, body.setIndex);
        bookHash = hashI32(bookHash, body.localIndex);
        bookHash = hashI32(bookHash, body.islandId);
        bookHash = hashI32(bookHash, body.islandPrev);
        bookHash = hashI32(bookHash, body.islandNext);
        bookHash = hashF32(bookHash, body.sleepTime);
        bookHash = hashF32(bookHash, body.mass);
        bookHash = hashF32(bookHash, body.inertia);
        bookHash = hashF32(bookHash, sim.center.x);
        bookHash = hashF32(bookHash, sim.center.y);
    }
    g_bookHash = bookHash;

    // contacts
    let liveContacts = 0;
    let touching = 0;
    let contactHash = hashInit();
    for (let i = 0; i < world.contacts.count; ++i) {
        let contact = world.contacts.get(i);
        if (contact.contactId == B2_NULL_INDEX) {
            continue;
        }

        liveContacts += 1;
        if ((contact.flags & b2_contactTouchingFlag) != 0) {
            touching += 1;
        }

        contactHash = hashI32(contactHash, contact.contactId);
        contactHash = hashI32(contactHash, contact.setIndex);
        contactHash = hashI32(contactHash, contact.colorIndex);
        contactHash = hashI32(contactHash, contact.shapeIdA);
        contactHash = hashI32(contactHash, contact.shapeIdB);
        contactHash = hashU32(contactHash, contact.flags & (b2_contactTouchingFlag | b2_contactEnableContactEvents));
        contactHash = hashI32(contactHash, contact.islandId);

        let sim = b2GetContactSim(world, contact);
        let m = sim.manifold;
        contactHash = hashI32(contactHash, m.pointCount);
        if (m.pointCount > 0) {
            contactHash = hashF32(contactHash, m.normal.x);
            contactHash = hashF32(contactHash, m.normal.y);
            contactHash = hashF32(contactHash, m.rollingImpulse);
            for (let k = 0; k < m.pointCount; ++k) {
                let mp = m.points[k];
                contactHash = hashU32(contactHash, mp.id);
                contactHash = hashF32(contactHash, mp.anchorA.x);
                contactHash = hashF32(contactHash, mp.anchorA.y);
                contactHash = hashF32(contactHash, mp.anchorB.x);
                contactHash = hashF32(contactHash, mp.anchorB.y);
                contactHash = hashF32(contactHash, mp.separation);
                contactHash = hashF32(contactHash, mp.normalImpulse);
                contactHash = hashF32(contactHash, mp.tangentImpulse);
                contactHash = hashF32(contactHash, mp.totalNormalImpulse);
                contactHash = hashF32(contactHash, mp.normalVelocity);
            }
        }
    }
    g_contactCount = liveContacts;
    g_touchingCount = touching;
    g_contactHash = contactHash;

    // islands
    let islandHash = hashInit();
    for (let i = 0; i < world.islands.count; ++i) {
        let island = world.islands.get(i);
        if (island.islandId == B2_NULL_INDEX || island.setIndex == B2_NULL_INDEX) {
            continue;
        }

        islandHash = hashI32(islandHash, island.islandId);
        islandHash = hashI32(islandHash, island.setIndex);
        islandHash = hashI32(islandHash, island.localIndex);
        islandHash = hashI32(islandHash, island.headBody);
        islandHash = hashI32(islandHash, island.tailBody);
        islandHash = hashI32(islandHash, island.bodyCount);
        islandHash = hashI32(islandHash, island.headContact);
        islandHash = hashI32(islandHash, island.tailContact);
        islandHash = hashI32(islandHash, island.contactCount);
        islandHash = hashI32(islandHash, island.headJoint);
        islandHash = hashI32(islandHash, island.tailJoint);
        islandHash = hashI32(islandHash, island.jointCount);
        islandHash = hashI32(islandHash, island.constraintRemoveCount);
    }
    g_islandHash = islandHash;

    // solver sets
    let setHash = hashInit();
    for (let i = 0; i < world.solverSets.count; ++i) {
        let set = world.solverSets.get(i);
        if (set.setIndex == B2_NULL_INDEX) {
            continue;
        }

        setHash = hashI32(setHash, set.setIndex);
        setHash = hashI32(setHash, set.bodySims.count);
        setHash = hashI32(setHash, set.contactSims.count);
        setHash = hashI32(setHash, set.jointSims.count);
        setHash = hashI32(setHash, set.islandSims.count);
        for (let k = 0; k < set.bodySims.count; ++k) {
            setHash = hashI32(setHash, set.bodySims.get(k).bodyId);
        }
        for (let k = 0; k < set.contactSims.count; ++k) {
            setHash = hashI32(setHash, set.contactSims.get(k).contactId);
        }
        for (let k = 0; k < set.jointSims.count; ++k) {
            setHash = hashI32(setHash, set.jointSims.get(k).jointId);
        }
        for (let k = 0; k < set.islandSims.count; ++k) {
            setHash = hashI32(setHash, set.islandSims.get(k).islandId);
        }
    }
    g_setHash = setHash;

    // graph colors
    let colorHash = hashInit();
    for (let i = 0; i < B2_GRAPH_COLOR_COUNT; ++i) {
        let color = world.constraintGraph.colors[i];
        colorHash = hashI32(colorHash, color.contactSims.count);
        colorHash = hashI32(colorHash, color.jointSims.count);
        for (let k = 0; k < color.contactSims.count; ++k) {
            colorHash = hashI32(colorHash, color.contactSims.get(k).contactId);
        }
        for (let k = 0; k < color.jointSims.count; ++k) {
            colorHash = hashI32(colorHash, color.jointSims.get(k).jointId);
        }
    }
    g_colorHash = colorHash;

    // events
    let bodyEvents = b2World_GetBodyEvents(g_worldId);
    let contactEvents = b2World_GetContactEvents(g_worldId);
    let jointEvents = b2World_GetJointEvents(g_worldId);

    let eventHash = hashInit();
    for (let i = 0; i < bodyEvents.moveCount; ++i) {
        let ev = bodyEvents.moveEvents[i]!;
        eventHash = hashI32(eventHash, ev.bodyId.index1);
        eventHash = hashF32(eventHash, ev.transform.p.x);
        eventHash = hashF32(eventHash, ev.transform.p.y);
        eventHash = hashF32(eventHash, ev.transform.q.c);
        eventHash = hashF32(eventHash, ev.transform.q.s);
        eventHash = hashI32(eventHash, ev.fellAsleep ? 1 : 0);
    }
    for (let i = 0; i < contactEvents.beginCount; ++i) {
        let ev = contactEvents.beginEvents[i]!;
        eventHash = hashI32(eventHash, ev.shapeIdA.index1);
        eventHash = hashI32(eventHash, ev.shapeIdB.index1);
        eventHash = hashI32(eventHash, ev.contactId.index1);
    }
    for (let i = 0; i < contactEvents.endCount; ++i) {
        let ev = contactEvents.endEvents[i]!;
        eventHash = hashI32(eventHash, ev.shapeIdA.index1);
        eventHash = hashI32(eventHash, ev.shapeIdB.index1);
        eventHash = hashI32(eventHash, ev.contactId.index1);
    }
    for (let i = 0; i < contactEvents.hitCount; ++i) {
        let ev = contactEvents.hitEvents[i]!;
        eventHash = hashI32(eventHash, ev.shapeIdA.index1);
        eventHash = hashI32(eventHash, ev.shapeIdB.index1);
        eventHash = hashF32(eventHash, ev.point.x);
        eventHash = hashF32(eventHash, ev.point.y);
        eventHash = hashF32(eventHash, ev.normal.x);
        eventHash = hashF32(eventHash, ev.normal.y);
        eventHash = hashF32(eventHash, ev.approachSpeed);
    }
    for (let i = 0; i < jointEvents.count; ++i) {
        eventHash = hashI32(eventHash, jointEvents.jointEvents[i]!.jointId.index1);
    }
    g_eventHash = eventHash;

    g_eventCounts[0] = bodyEvents.moveCount;
    g_eventCounts[1] = contactEvents.beginCount;
    g_eventCounts[2] = contactEvents.endCount;
    g_eventCounts[3] = contactEvents.hitCount;
    g_eventCounts[4] = jointEvents.count;
}

export function dynBodyCount(): i32 {
    return g_bodyIds.length;
}

export function dynBodyId(i: i32): i32 {
    return g_bodyIds[i];
}

export function dynBodySet(i: i32): i32 {
    return g_bodySets[i];
}

export function dynBodyF(i: i32, field: i32): u32 {
    return g_bodyFloats[7 * i + field];
}

export function dynContactCount(): i32 {
    return g_contactCount;
}

export function dynTouchingCount(): i32 {
    return g_touchingCount;
}

export function dynContactHash(): u64 {
    return g_contactHash;
}

export function dynIslandHash(): u64 {
    return g_islandHash;
}

export function dynSetHash(): u64 {
    return g_setHash;
}

export function dynColorHash(): u64 {
    return g_colorHash;
}

export function dynBookHash(): u64 {
    return g_bookHash;
}

export function dynEventCount(i: i32): i32 {
    return g_eventCounts[i];
}

export function dynEventHash(): u64 {
    return g_eventHash;
}

// TCMS collection is only safe with an unwound stack, so the harness calls
// this between steps from the host side.
export function dynCollect(): void {
    __collect();
}
