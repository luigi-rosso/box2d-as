# box2d-as

Box2D v3.2, Erin Catto's 2D physics engine, ported to AnimaScript as a Rive scripting library. It keeps Box2D's C API
names, and every result matches native Box2D bit for bit, step for step.

Upstream: [erincatto/box2d](https://github.com/erincatto/box2d) at `178495ba35695aefe9c321e569d2277f06db3b90` (v3.2.0).
Box2D is MIT licensed. The port keeps that licence and Erin Catto's copyright on every file (`LICENSE`).

## What it is

- **All of Box2D's simulation.** Upstream `src/` is ported to about 31,800 lines of AnimaScript in 42 files under
  `lib/`, one file per C source, with functions in C's order:
  - shapes: circles, capsules, polygons with Box2D's hull builder, segments and chains;
  - the dynamic tree broad phase;
  - the narrow phase: GJK, shape casts, time of impact and every manifold function;
  - the soft step solver, with graph coloring and the 4 wide contact solver on wasm SIMD;
  - all nine joint types, the mover and pogo joints included;
  - islands and sleeping, continuous collision and bullets, sensors and events;
  - ray casts, shape casts, overlaps, explosions and the character mover.
- **Box2D's API, under the same names.** `lib/box2d.as` re-exports 545 of the 584 public functions. Ids are handles
  (`b2BodyId`), plain C structs are AnimaScript structs, and callbacks are closures. `CONVENTIONS.md` has the full
  C to AnimaScript mapping.
- **Bit exact.** The same float operations in the same order, checked against native Box2D on every module and on
  every step of 18 scenes (see Parity below).
- **Allocation free per step.** Once a scene has settled, a step allocates nothing: step buffers grow only, and
  contacts, manifolds, sims and joints are structs in arrays.
- **Left out:** debug draw; recording, replay and the world snapshots they use; the built in thread scheduler (tasks
  run inline on one worker, as C's serial fallback does); and the SSE2, AVX2 and NEON backends. The 4 lane SIMD
  layer runs on wasm `v128`, or on scalar lanes when the build has no SIMD.

## Quick start

```as
import { b2Body_GetPosition, b2CreateBody, b2CreatePolygonShape, b2CreateWorld, b2DefaultBodyDef, b2DefaultShapeDef,
	b2DefaultWorldDef, b2DestroyWorld, b2MakeBox, b2Vec2, b2World_Step, b2_dynamicBody } from "./lib/box2d";

let worldId = b2CreateWorld(b2DefaultWorldDef());

let groundDef = b2DefaultBodyDef();
groundDef.position = b2Vec2(0.0, -1.0);
let groundId = b2CreateBody(worldId, groundDef);
b2CreatePolygonShape(groundId, b2DefaultShapeDef(), b2MakeBox(10.0, 1.0));

let bodyDef = b2DefaultBodyDef();
bodyDef.type = b2_dynamicBody;
bodyDef.position = b2Vec2(0.0, 4.0);
let bodyId = b2CreateBody(worldId, bodyDef);
b2CreatePolygonShape(bodyId, b2DefaultShapeDef(), b2MakeBox(0.5, 0.5));

let timeStep: f32 = <f32>(1.0 / 60.0);
for (let i = 0; i < 180; ++i) {
	b2World_Step(worldId, timeStep, 4);
}
let y = b2Body_GetPosition(bodyId).y; // 0.5: the box rests on the ground
b2DestroyWorld(worldId);
```

rive-cli resolves imports only inside a project, so copy or sync `lib/` into yours. As in C, write float constants
as `f32` (`<f32>(1.0 / 60.0)`; a constant expression of float literals is `f64`). The library runs under the frame
collector: keep the world id and every callback you register reachable from module globals or your objects, and step
from `advance`, so a collection between frames keeps them. `verify/scenes/scenes.as` has 18 larger examples, one per
scene below, each the statement for statement twin of `scenes.c` over the C API.

## API availability

Upstream's public headers (`box2d.h`, `collision.h`, `math_functions.h`, `types.h`, `base.h` and `id.h`: every
`B2_API` function plus the inline ones), against what `lib/box2d.as` exports:

| area | ported | not ported, and why |
|---|---|---|
| world | 53 / 54 | `b2World_DumpMemoryStats` (prints C allocator stats) |
| bodies | 75 / 75 |  |
| shapes | 54 / 54 |  |
| chains | 9 / 9 |  |
| contacts | 2 / 2 |  |
| joints, common | 34 / 34 |  |
| distance joint | 24 / 24 |  |
| motor joint | 20 / 20 |  |
| mover joint | 4 / 4 |  |
| pogo joint | 9 / 9 |  |
| prismatic joint | 22 / 22 |  |
| revolute joint | 22 / 22 |  |
| weld joint | 8 / 8 |  |
| wheel joint | 18 / 18 |  |
| geometry, collision and dynamic tree | 70 / 70 |  |
| math | 91 / 91 |  |
| defaults and defs | 17 / 17 |  |
| base | 13 / 13 |  |
| debug draw | 0 / 3 | `b2World_Draw`, `b2DefaultDebugDraw`, `b2GetGraphColor` |
| recording, replay and snapshots | 0 / 35 | `b2World_StartRecording`, `b2Replay_*`, `b2World_GetSnapshot`, ... |
| **all** | **545 / 584** | |

Exported but with nothing to do in a runtime that owns memory and threads: `b2SetAllocator`, `b2GetByteCount` (0),
`b2IsAVX2Available` (false), `b2Yield`, and `b2World_SetWorkerCount` (it sizes the per worker storage; tasks still
run on one). Signatures that differ from C (a `const T*` input becomes a value or a `ref`, an out pointer a `ref` or
the return value, a callback's `void* context` the closure) are listed per module in `notes/`.

## Parity

The reference is native Box2D's scalar build (`BOX2D_DISABLE_SIMD`, `-ffp-contract=off`). Native Box2D gives the
same bits for its scalar and NEON builds and for any worker count (`notes/scenes.md`), so the port is held to bit
exactness, with no tolerance.

| gate | what it checks | covers |
|---|---|---|
| `verify/run.sh` | each module's outputs against a C driver over fixed and seeded random inputs, every value's bits | 16 modules, about 16 million values |
| `verify/traj.sh` | every body's state, the world counters and per scene probes (joints, manifolds, events, queries), every step, against a native recording | 18 scenes |
| `verify/heap.mjs` | wasm pages after step 20 and step 200 under the frame collector: a settled scene must not grow | 11 scenes |
| `bench/web/verify.sh` | the state hash of every step on V8, for the port and for C Box2D on wasm | 18 scenes, 6 builds |

The 18 scenes are pyramid, shape rain, chain car, joints, dynamic mover, churn (create, destroy and every body
setter), bullets, sensors, events, queries, the geometric mover, many bodies, and upstream's benchmarks large
pyramid, joint grid, rain, smash, spinner and tumbler. Module and trajectory gates pass on the scalar and the SIMD
artifact, and every bench run below checks its hash against the recording.

## Performance compared to C

ms per step, single threaded. Machine: Apple M5 Max (6 performance and 12 efficiency cores), macOS 26.5, shared with
other work, so compare ratios within a table, not milliseconds across tables. `notes/perf.md` has the stage
breakdowns and what each change bought.

### V8: the port against C Box2D compiled to wasm

The same scenes in node 26.3.1 (V8 14.6): C Box2D at v3.2.0 through emscripten 6.0.13 (`-O3 -ffp-contract=off`; SIMD
is Box2D's SSE2 path on `-msimd128`, scalar is `BOX2D_DISABLE_SIMD`), and the port's release bake (rasc `-O2`,
binaryen 133, as rive-cli bakes a release). Native ARM64 Box2D for scale. Each run's mean over the steps after the
first 10%, min over 3 interleaved rounds, 252 runs, load 8.6 to 62, 2026-10-27; every run's hash matched the
recording on every step.

| scene | native NEON | C wasm SIMD | AS SIMD | AS / C SIMD | C wasm scalar | AS scalar | AS / C scalar |
|---|---|---|---|---|---|---|---|
| large_pyramid | 1.17 | 1.44 | 1.47 | 1.02 | 3.13 | 2.43 | 0.78 |
| many_bodies | 0.925 | 1.12 | 1.11 | 0.99 | 2.75 | 2.00 | 0.73 |
| joint_grid | 2.22 | 3.19 | 3.21 | 1.01 | 3.13 | 3.06 | 0.98 |
| rain | 2.30 | 2.66 | 3.07 | 1.15 | 3.09 | 3.62 | 1.17 |
| smash | 1.06 | 1.42 | 1.66 | 1.17 | 1.99 | 1.90 | 0.96 |
| spinner | 3.38 | 4.32 | 4.70 | 1.09 | 6.33 | 6.01 | 0.95 |
| tumbler | 0.647 | 0.883 | 0.945 | 1.07 | 1.32 | 1.17 | 0.89 |
| shape_rain | 0.0179 | 0.0259 | 0.0278 | 1.07 | 0.0434 | 0.0379 | 0.87 |
| chain_car | 0.0027 | 0.0079 | 0.0069 | 0.87 | 0.0083 | 0.0073 | 0.87 |
| joints | 0.0068 | 0.0143 | 0.0137 | 0.96 | 0.0147 | 0.0145 | 0.99 |
| queries | 0.0025 | 0.0114 | 0.0129 | 1.14 | 0.0170 | 0.0135 | 0.79 |

- With SIMD the port runs at 0.99 to 1.17x C on the same engine; C on wasm is itself 1.16 to 1.44x native. After
  the second pass (d24f4a3; 5 rounds at load 6 to 8, `notes/perf.md`) the benchmark scenes run at 0.97 to 1.11x C:
  rain 1.08, smash 1.11, spinner 1.09, tumbler 1.05.
- The solver stages run at C's speed; what is left is the contact churn of rain and smash, where pairs and collide
  are 1.26 to 1.36x C.
- Without SIMD it beats C's scalar wasm on all but rain: C's scalar build gets no autovectorization on wasm, while
  the port's scalar artifact runs one lane at a time and skips empty lanes.

Reproduce: `bench/web/build.sh` (`EMSDK`, `WASM_OPT` = binaryen 133), `bench/web/build_native.sh`, then
`ROUNDS=3 node bench/web/bench.mjs`.

### WAMR AOT: the port against native Box2D

rive-cli's own runtime: the release bake of `bench/as` on WAMR AOT (`--aot`, `RIVE_WASM_AOT_SYNC=o3`, the SIMD
artifact), interleaved with native NEON Box2D. Software bounds is rive master's CLI (8e7360ec48) as shipped; hardware
bounds is the CLI of draft PR #15153, which makes guard page bounds the default on 64-bit macOS and Linux. Both use a
wamrc with rive's patch 0037. 4 rounds, 112 runs, load 6.4 to 12.5, 2026-10-27; means from step 1, every hash matched.

| scene | native NEON | AS AOT, software bounds | AOT / native | AS AOT, hardware bounds | AOT / native |
|---|---|---|---|---|---|
| large_pyramid | 1.14 | 1.31 | 1.15 | 1.24 | 1.09 |
| many_bodies | 0.884 | 1.06 | 1.20 | 1.00 | 1.13 |
| joint_grid | 2.22 | 2.92 | 1.32 | 2.41 | 1.09 |
| rain | 2.11 | 2.74 | 1.30 | 2.36 | 1.12 |
| smash | 0.957 | 1.32 | 1.38 | 1.17 | 1.22 |
| spinner | 3.40 | 4.64 | 1.37 | 4.14 | 1.22 |
| tumbler | 0.587 | 0.786 | 1.34 | 0.698 | 1.19 |

- As shipped, WAMR AOT runs the port at 1.15 to 1.38x native: every load and store carries a software bounds check.
- With hardware bounds it runs at 1.09 to 1.22x native, 5 to 18% faster; the narrow phase gains most (collide 1.5 to
  1.8x native with software bounds, 1.0 to 1.4x with hardware bounds).
- Use a wamrc built from rive's current patches: one without patch 0037 reloads the memory base after every store in
  any function that calls, up to 10% slower here.
- The scalar artifact on AOT (first pass, older wamrc): 1.3 to 1.35x native scalar on the pyramids and joint_grid.

Reproduce: `ROUNDS=3 RIVE_WAMRC=<wamrc> node bench/web/aot.mjs`, then `node bench/web/bench.mjs --table=<log>`, or
`bench/as/run.sh` for the AOT tier alone against `bench/native`.

### WAMR's fast interpreter

The same CLI without wamrc, ms per step, 2 rounds at a high load (9 to 80), so read the ratios. Both artifacts run
every scene bit exact.

| scene | native scalar | AS scalar artifact, AOT | AS scalar artifact, interpreter | interpreter / native scalar | AS SIMD artifact, interpreter |
|---|---|---|---|---|---|
| many_bodies | 2.07 | 3.20 | 44.5 | 21.5 | 26.3 |
| joint_grid | 3.42 | 4.84 | 58.1 | 17.0 | 71.5 |
| tumbler | 1.10 | 1.58 | 22.2 | 20.2 | 14.3 |
| shape_rain | 0.011 | 0.031 | 0.402 | 36 | 0.371 |
| joints | 0.007 | 0.013 | 0.277 | 40 | 0.249 |

The interpreter is 12 to 21x slower than AOT: fine for a few hundred bodies, not for the benchmark sizes. The SIMD
release bake is safe on it: WAMR 2.4.5's fast interpreter traps on a v128 local past frame cell 256, and the bake's
largest is cell 200 (`bench/as/tools/v128cells.py`). A build without the binaryen post pass (plain `-O`) is not.

## Layout and tooling

| path | what |
|---|---|
| `lib/` | the port; `box2d.as` is the public API |
| `verify/` | parity gates: `cref/` C drivers, `as/` AnimaScript drivers, `scenes/` the 18 scenes in C and AnimaScript |
| `bench/native`, `bench/as`, `bench/web` | native timings, rive-cli AOT and interpreter timings, V8 against C on wasm |
| `notes/` | porting notes per module, the perf pass |

Toolchain: rasc, its std and rive-cli from a rive master checkout (`verify/toolchain.sh` looks in
`~/Projects/rive-pinefall`, then `~/Projects/rive-master`; `RASC`, `AS_STD`, `RIVE` and `RIVE_WAMRC` override), and a
wamrc for `--aot` built from the patched WAMR the CLI build downloads (`~/Projects/tools/wamrc` by default,
which predates patch 0037; point `RIVE_WAMRC` at a current build).

The gates need upstream Box2D at `~/Projects/box2d` (`BOX2D`), built in `build/scalar` (`BOX2D_DISABLE_SIMD=ON`,
`-ffp-contract=off`) and `build/release`. The native recordings in `verify/scenes/out` are not committed:
`NATIVE=1 verify/traj.sh` records them.

```sh
KEEP_GOING=1 RASC_FLAGS="-O" verify/run.sh
KEEP_GOING=1 RASC_FLAGS="-O --enable-simd" verify/run.sh
RASC_FLAGS="-O" verify/traj.sh
RASC_FLAGS="-O --enable-simd" WASM=verify/scenes/out/as/traj_simd.wasm verify/traj.sh
node verify/heap.mjs many_bodies --wasm=verify/scenes/out/as/traj_simd.wasm
```

Benches: `bench/native/run.sh`, `bench/as/run.sh` (WAMR AOT and interpreter against native), and `bench/web/`
(`build.sh` builds C Box2D with emscripten and the port's release bake, `bench.mjs` runs the V8 rounds, `aot.mjs`
WAMR AOT against V8 and native, `verify.sh` the hash sweep). `EMSDK` and `WASM_OPT` name emsdk and binaryen 133.

## Notes

| note | what |
|---|---|
| `CONVENTIONS.md` | how C maps to AnimaScript: files, names, types, structs and classes, pointers, callbacks, the performance rules |
| `notes/perf.md` | the performance pass: results, changes in order, what did not help, what is left |
| `notes/world.md` | the internal API contract between the simulation files, deviations, `lib/box2d.as` |
| `notes/solver.md`, `joints.md`, `collision.md`, `tree.md`, `bodyshape.md` | per module porting notes: deviations, skipped functions |
| `notes/foundations.md` | the base modules, the parity harness and the struct or class choice |
| `notes/scenes.md` | the trajectory scenes, the recording format and the probes |

rasc findings from the port (each with a minimal repro, the observed and expected behaviour and the workaround) are
in the Obsidian note "rasc — Box2D v3.2 Port Findings (Handoff)".
