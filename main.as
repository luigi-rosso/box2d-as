// Phase A+B smoke entry: exercises the modules once and logs a sentinel line
// per family so a bake proves the module links and runs.

import { Protocol, Context } from "rive/host";
import { b2MakeRot, b2TransformPoint, b2Transform, b2Vec2, b2Rot, b2AABB } from "./lib/math_functions";
import { b2MakeBox, b2ComputePolygonMass } from "./lib/geometry";
import { b2ComputeHull, b2ValidateHull } from "./lib/hull";
import { makeVecArray } from "./lib/collision";
import { b2CreateIdPool, b2AllocId, b2FreeId, b2GetIdCount } from "./lib/id_pool";
import { b2CollidePolygons } from "./lib/manifold";
import {
    b2DynamicTree_Create, b2DynamicTree_CreateProxy, b2DynamicTree_Query, b2TreeQueryCallback,
} from "./lib/dynamic_tree";
import { b2DefaultWorldDef, b2DefaultBodyDef, b2DefaultShapeDef, b2_dynamicBody } from "./lib/types";
import { b2CreateWorld, b2DestroyWorld, b2World_Step, b2GetWorldFromId } from "./lib/physics_world";
import { b2CreateBody } from "./lib/body";
import { b2CreatePolygonShape } from "./lib/shape";
import { b2GetContactSim } from "./lib/contact";
import { BENCH_STEPS, BENCH_ITERS, BENCH_BODIES } from "./bench";
import { benchWarm, benchIter } from "./bench_stub";

declare function emscripten_get_now(): f64;

// "%.<digits>f" fixed-point formatting (context.log has no printf).
function fmtFixed(x: f64, digits: i32): string {
    let scale: f64 = 1;
    for (let i = 0; i < digits; i++) {
        scale *= 10;
    }
    let scaled = <i64>Math.round(x * scale);
    let neg = scaled < 0;
    if (neg) {
        scaled = -scaled;
    }
    let whole = scaled / <i64>scale;
    let out = (neg ? "-" : "") + whole.toString();
    if (digits > 0) {
        let frac = (scaled % <i64>scale).toString();
        while (frac.length < digits) {
            frac = "0" + frac;
        }
        out += "." + frac;
    }
    return out;
}

class Main extends Protocol {
    ran: bool = false;

    implementsInit(): bool {
        return true;
    }

    init(context: Context): bool {
        if (this.ran) {
            return true;
        }
        this.ran = true;

        let q = b2MakeRot(0.5);
        let xf = new b2Transform(new b2Vec2(1, 2), q);
        let p = b2TransformPoint(xf, new b2Vec2(3, 4));
        context.log("BOX2D-A: transform " + p.x.toString() + " " + p.y.toString());

        let box = b2MakeBox(0.5, 1.5);
        let mass = b2ComputePolygonMass(box, 1.0);
        context.log("BOX2D-A: box mass " + mass.mass.toString() + " I " + mass.rotationalInertia.toString());

        let pts = makeVecArray(8);
        pts[0] = new b2Vec2(-1, -1);
        pts[1] = new b2Vec2(1, -1);
        pts[2] = new b2Vec2(1, 1);
        pts[3] = new b2Vec2(-1, 1);
        pts[4] = new b2Vec2(0, 0);
        let hull = b2ComputeHull(pts, 5);
        context.log("BOX2D-A: hull " + hull.count.toString() + " valid " + b2ValidateHull(hull).toString());

        let pool = b2CreateIdPool();
        let a = b2AllocId(pool);
        let b = b2AllocId(pool);
        b2FreeId(pool, a);
        let c = b2AllocId(pool);
        context.log("BOX2D-A: idpool " + a.toString() + b.toString() + c.toString() +
            " count " + b2GetIdCount(pool).toString());

        context.log("BOX2D-A: done");

        let boxA = b2MakeBox(0.5, 0.5);
        let boxB = b2MakeBox(0.5, 0.5);
        let xfA = new b2Transform(new b2Vec2(0, 0), new b2Rot(1, 0));
        let xfB = new b2Transform(new b2Vec2(0.9, 0.1), new b2Rot(1, 0));
        let manifold = b2CollidePolygons(boxA, xfA, boxB, xfB);
        context.log("BOX2D-B: manifold points " + manifold.pointCount.toString() +
            " normal " + manifold.normal.x.toString() + " " + manifold.normal.y.toString() +
            " sep " + manifold.points[0].separation.toString());

        let tree = b2DynamicTree_Create();
        for (let i = 0; i < 8; i++) {
            let x = <f32>i;
            let aabb = new b2AABB(new b2Vec2(x, 0), new b2Vec2(x + <f32>0.8, 1));
            b2DynamicTree_CreateProxy(tree, aabb, 1, <u64>i);
        }
        let counter = new CountingQuery();
        b2DynamicTree_Query(tree, new b2AABB(new b2Vec2(1.5, 0.2), new b2Vec2(4.2, 0.8)), 1, counter);
        context.log("BOX2D-B: tree query hits " + counter.hits.toString());

        context.log("BOX2D-B: done");

        // Phase C sentinel: step a small pyramid and print state checksums.
        {
            let worldDef = b2DefaultWorldDef();
            let worldId = b2CreateWorld(worldDef);

            let groundDef = b2DefaultBodyDef();
            groundDef.position = new b2Vec2(0, -1);
            let groundId = b2CreateBody(worldId, groundDef);
            let groundShapeDef = b2DefaultShapeDef();
            let ground = b2MakeBox(20, 1);
            b2CreatePolygonShape(groundId, groundShapeDef, ground);

            let bodyDef = b2DefaultBodyDef();
            bodyDef.type = b2_dynamicBody;
            let shapeDef = b2DefaultShapeDef();
            let pyBox = b2MakeBox(0.5, 0.5);
            let rows = 5;
            for (let i = 0; i < rows; i++) {
                for (let j = i; j < rows; j++) {
                    let px = <f32>(i + 1) * <f32>0.5 + <f32>(j - i) - <f32>0.5 * <f32>rows;
                    let py = <f32>i + <f32>0.5;
                    bodyDef.position = new b2Vec2(px, py);
                    let bodyId = b2CreateBody(worldId, bodyDef);
                    b2CreatePolygonShape(bodyId, shapeDef, pyBox);
                }
            }

            for (let step = 0; step < 60; step++) {
                b2World_Step(worldId, <f32>1.0 / <f32>60.0, 4);
            }

            let world = b2GetWorldFromId(worldId);
            let posSum: f32 = 0;
            for (let i = 0; i < world.bodies.count; i++) {
                let body = world.bodies.get(i);
                if (body.id == -1) {
                    continue;
                }
                let set = world.solverSets.get(body.setIndex);
                let sim = set.bodySims.get(body.localIndex);
                posSum += sim.transform.p.x + sim.transform.p.y;
            }

            let impulseSum: f32 = 0;
            let touching = 0;
            for (let i = 0; i < world.contacts.count; i++) {
                let contact = world.contacts.get(i);
                if (contact.contactId == -1) {
                    continue;
                }
                let sim = b2GetContactSim(world, contact);
                for (let k = 0; k < sim.manifold.pointCount; k++) {
                    impulseSum += sim.manifold.points[k].normalImpulse;
                }
                if ((contact.flags & 1) != 0) {
                    touching += 1;
                }
            }

            context.log("BOX2D-C: pyramid bodies " + world.bodies.count.toString() +
                " touching " + touching.toString());
            context.log("BOX2D-C: posSum " + posSum.toString() + " impulseSum " + impulseSum.toString());

            b2DestroyWorld(worldId);
            context.log("BOX2D-C: done");
        }

        {
            let warm = benchWarm(BENCH_STEPS);
            let bestMs: f64 = 1e30;
            let checksum: f64 = 0;
            let stable = true;
            for (let it = 0; it < BENCH_ITERS; it++) {
                let t0 = emscripten_get_now();
                let sum = benchIter(BENCH_STEPS);
                let ms = emscripten_get_now() - t0;
                if (it == 0) {
                    checksum = sum;
                } else if (sum != checksum) {
                    stable = false;
                }
                if (ms < bestMs) {
                    bestMs = ms;
                }
            }
            if (warm != checksum || !stable) {
                context.log("BOX2D-BENCH: UNSTABLE checksum across iterations");
            }
            context.log("BOX2D-BENCH: pyramid" + BENCH_STEPS.toString() + " | " +
                BENCH_BODIES.toString() + " bodies | " + fmtFixed(bestMs, 3) +
                " ms/run best | " + checksum.toString() + " | " +
                BENCH_ITERS.toString() + " iters");
            context.log("BOX2D-BENCH: checksum bits " + reinterpret<u64>(checksum).toString(16));
        }
        return true;
    }
}

class CountingQuery extends b2TreeQueryCallback {
    hits: i32 = 0;

    queryCallback(proxyId: i32, userData: u64): bool {
        this.hits += 1;
        return true;
    }
}

export function generator(): Protocol {
    return new Main();
}
