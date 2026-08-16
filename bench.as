// Pyramid bench scene replicating luau_wasm/tests/box2d/driver.luau: 5-row
// pyramid of 1x1 dynamic boxes on a 100x2 ground, dt=1/60 with 4 substeps,
// checksum = sum(p.x + p.y * 3) over the dynamic bodies in creation order.
// Runtime-neutral; the stub-runtime iteration wrappers live in bench_stub.as.

import { b2Vec2 } from "./lib/math_functions";
import { b2MakeBox } from "./lib/geometry";
import { b2DefaultWorldDef, b2DefaultBodyDef, b2DefaultShapeDef, b2_dynamicBody } from "./lib/types";
import { b2CreateWorld, b2DestroyWorld, b2World_Step } from "./lib/physics_world";
import { b2CreateBody, b2Body_GetPosition } from "./lib/body";
import { b2CreatePolygonShape } from "./lib/shape";
import { b2BodyId, b2WorldId } from "./lib/id";

export const BENCH_STEPS: i32 = 240;
export const BENCH_ITERS: i32 = 5;
export const BENCH_ROWS: i32 = 5;
export const BENCH_BODIES: i32 = (BENCH_ROWS * (BENCH_ROWS + 1)) / 2;

let s_worldId: b2WorldId = new b2WorldId(0, 0);
let s_bodies: Array<b2BodyId> = new Array<b2BodyId>();

export function benchStart(): void {
    let worldDef = b2DefaultWorldDef();
    worldDef.gravity = new b2Vec2(0, -10);
    worldDef.enableSleep = false;
    s_worldId = b2CreateWorld(worldDef);

    let groundDef = b2DefaultBodyDef();
    groundDef.position = new b2Vec2(0, -1);
    let groundId = b2CreateBody(s_worldId, groundDef);
    let groundBox = b2MakeBox(50, 1);
    let groundShapeDef = b2DefaultShapeDef();
    b2CreatePolygonShape(groundId, groundShapeDef, groundBox);

    s_bodies = new Array<b2BodyId>();
    let box = b2MakeBox(0.5, 0.5);
    for (let row = 1; row <= BENCH_ROWS; row++) {
        let cols = BENCH_ROWS - row + 1;
        for (let col = 1; col <= cols; col++) {
            let def = b2DefaultBodyDef();
            def.type = b2_dynamicBody;
            def.position = new b2Vec2(
                <f32>(col - 1) * <f32>1.1 - <f32>(cols - 1) * <f32>0.55,
                <f32>row * <f32>1.1 + <f32>0.5);
            let id = b2CreateBody(s_worldId, def);
            let shapeDef = b2DefaultShapeDef();
            shapeDef.density = 1;
            b2CreatePolygonShape(id, shapeDef, box);
            s_bodies.push(id);
        }
    }
}

export function benchStep(count: i32): void {
    for (let i = 0; i < count; i++) {
        b2World_Step(s_worldId, <f32>1.0 / <f32>60.0, 4);
    }
}

export function benchFinish(): f64 {
    let sum: f64 = 0;
    for (let i = 0; i < s_bodies.length; i++) {
        let p = b2Body_GetPosition(s_bodies[i]);
        sum += <f64>p.x + <f64>p.y * 3.0;
    }
    b2DestroyWorld(s_worldId);
    s_bodies = new Array<b2BodyId>();
    return sum;
}

export function benchRun(steps: i32): f64 {
    benchStart();
    benchStep(steps);
    return benchFinish();
}
