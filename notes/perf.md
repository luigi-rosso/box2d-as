# perf: the first performance pass

Scope: all of `lib/`, from 22e40eb (the finished, bit exact port) to 30f56b6. The bench harness (`bench/native`,
`bench/as`, `bench/web`, adapted from box3d-as) and Box2D's six benchmark scenes (large_pyramid, joint_grid, rain,
smash, spinner, tumbler, from upstream's `shared/benchmarks.c`) landed in the middle of it (162adf6, 6dbf6e7).

## Results

Machine: Apple M5 Max (6 + 12 cores), macOS 26.5, shared with other sessions all day (one minute load from 4 to 200),
so read ratios within a table. ms per step, single threaded, one worker. Each run's mean over the steps after the
first 10%, min over interleaved rounds: the benchmark scenes vary a lot step to step (rain and smash have long quiet
stretches), so the mean says more than the median there. Every run's state hash matched the native recording.

### V8 against C Box2D on wasm

node 26.3.1 (V8 14.6), `bench/web/bench.mjs`, 3 rounds over 7 builds, 252 runs, load 8.6 to 62. C Box2D at v3.2.0
through emscripten 6.0.13 (`-O3 -ffp-contract=off`; SIMD is Box2D's own SSE2 path on `-msimd128`, scalar is
`BOX2D_DISABLE_SIMD`), against the port's release bake (rasc `-O2`, binaryen 133) before (22e40eb) and after.

| scene | native NEON | C wasm SIMD | AS SIMD before | AS SIMD after | AS / C before | AS / C after | C wasm scalar | AS scalar before | AS scalar after | AS / C scalar before | after |
|---|---|---|---|---|---|---|---|---|---|---|---|
| large_pyramid | 1.17 | 1.44 | 1.81 | 1.47 | 1.26 | 1.02 | 3.13 | 2.83 | 2.43 | 0.90 | 0.78 |
| many_bodies | 0.925 | 1.12 | 1.46 | 1.11 | 1.30 | 0.99 | 2.75 | 2.32 | 2.00 | 0.84 | 0.73 |
| joint_grid | 2.22 | 3.19 | 5.05 | 3.21 | 1.58 | 1.01 | 3.13 | 4.60 | 3.06 | 1.47 | 0.98 |
| rain | 2.30 | 2.66 | 3.57 | 3.07 | 1.34 | 1.15 | 3.09 | 4.17 | 3.62 | 1.35 | 1.17 |
| smash | 1.06 | 1.42 | 1.86 | 1.66 | 1.31 | 1.17 | 1.99 | 2.28 | 1.90 | 1.15 | 0.96 |
| spinner | 3.38 | 4.32 | 5.62 | 4.70 | 1.30 | 1.09 | 6.33 | 6.84 | 6.01 | 1.08 | 0.95 |
| tumbler | 0.647 | 0.883 | 1.18 | 0.945 | 1.34 | 1.07 | 1.32 | 1.43 | 1.17 | 1.08 | 0.89 |
| shape_rain | 0.0179 | 0.0259 | 0.0336 | 0.0278 | 1.30 | 1.07 | 0.0434 | 0.0439 | 0.0379 | 1.01 | 0.87 |
| chain_car | 0.0027 | 0.0079 | 0.0067 | 0.0069 | 0.85 | 0.87 | 0.0083 | 0.0075 | 0.0073 | 0.90 | 0.87 |
| joints | 0.0068 | 0.0143 | 0.0129 | 0.0137 | 0.90 | 0.96 | 0.0147 | 0.0132 | 0.0145 | 0.90 | 0.99 |
| queries | 0.0025 | 0.0114 | 0.0144 | 0.0129 | 1.26 | 1.14 | 0.0170 | 0.0157 | 0.0135 | 0.92 | 0.79 |

- The SIMD artifact runs at 0.99 to 1.17x C's SIMD build on the benchmark scenes, from 1.26 to 1.58x before.
- The scalar artifact beats C's scalar build on all but rain: C's scalar wasm runs Box2D's four lane structs as
  straight scalar code, the port runs one lane at a time and skips empty lanes (box3d-as notes/web.md).
- The first rough number (about 6x native on many_bodies) was whole process time with scene creation, V8's tier up
  and a load near 100; measured per step, many_bodies was 1.6x native and 1.3x C wasm before this pass.

Stages, AS SIMD after over C SIMD, same session: the solver stages are at C's speed (many_bodies solve 0.963 against
0.953 ms, joint_grid 2.64 against 2.67), collide 0.82 to 1.36x (C's narrow phase loses on the resting pyramids, the
port's on rain and smash), pairs 1.11 to 1.33x on the churn scenes.

### WAMR AOT against native

rive-cli's own runtime, the release bake of `bench/as` on WAMR AOT (`--aot`, `RIVE_WASM_AOT_SYNC=o3`, wamrc
`~/Projects/tools/wamrc`, software bounds checks as shipped), the SIMD artifact, interleaved with native NEON Box2D and
with the same module on V8: `bench/web/aot.mjs`, 3 rounds, 96 runs, load 4.3 to 8.2, rive master (rive-pinefall
8e7360ec48).

| scene | native NEON | AS WAMR AOT | AOT / native | AS on V8 | V8 / native | C wasm on V8 |
|---|---|---|---|---|---|---|
| large_pyramid | 1.10 | 1.33 | 1.21 | 1.29 | 1.17 | 1.28 |
| many_bodies | 0.874 | 1.01 | 1.16 | 1.06 | 1.21 | 1.02 |
| joint_grid | 2.09 | 2.73 | 1.31 | 2.88 | 1.38 | 2.84 |
| rain | 2.00 | 2.73 | 1.37 | 2.62 | 1.31 | 2.39 |
| smash | 0.897 | 1.33 | 1.48 | 1.28 | 1.43 | 1.11 |
| spinner | 3.19 | 4.87 | 1.53 | 4.27 | 1.34 | 3.85 |
| tumbler | 0.570 | 0.881 | 1.55 | 0.792 | 1.39 | 0.734 |

Before and after on AOT, both artifacts, one session of 2 rounds (`aotab.py` in the scratchpad: native, then each
rive-cli project in turn, median ms per step; spinner and tumbler ran into a load spike and are left out):

| scene | native NEON | before | after | after / native | native scalar | scalar artifact before | after | after / native scalar |
|---|---|---|---|---|---|---|---|---|
| large_pyramid | 1.10 | 2.12 | 1.32 | 1.20 | 1.63 | 3.13 | 2.20 | 1.35 |
| many_bodies | 0.862 | 1.81 | 1.05 | 1.22 | 1.39 | 2.72 | 1.82 | 1.31 |
| joint_grid | 2.13 | 4.22 | 2.82 | 1.32 | 2.14 | 4.26 | 2.82 | 1.32 |
| rain | 1.69 | 3.37 | 2.86 | 1.69 | 1.69 | 3.35 | 2.86 | 1.69 |
| smash (mean) | 0.927 | 2.14 | 1.67 | 1.80 | 1.34 | 3.80 | 2.99 | 2.23 |

AOT over native by stage (the first table's session): solve 1.07 to 1.27x, collide 1.6 to 2.4x (spinner 1.86 against
0.94 ms, tumbler 0.26 against 0.11), pairs and the rest about 1.45x on spinner.

### WAMR's fast interpreter

Same CLI without wamrc, `wasmSimd: off` for the scalar artifact; 2 rounds at load 9 to 80, so only the ratios hold.
Both artifacts finish every scene bit exact (final hash against the recording, checked on many_bodies, joint_grid,
tumbler and shape_rain).

| scene | native scalar | AOT, scalar artifact | interp, scalar artifact before | after | interp / native scalar | interp, SIMD artifact before | after |
|---|---|---|---|---|---|---|---|
| many_bodies | 2.07 | 3.20 | 50.3 | 44.5 | 21.5 | 28.2 | 26.3 |
| joint_grid | 3.42 | 4.84 | 76.9 | 58.1 | 17.0 | 83.8 | 71.5 |
| tumbler | 1.10 | 1.58 | 23.6 | 22.2 | 20.2 | 15.8 | 14.3 |
| shape_rain | 0.011 | 0.031 | 0.424 | 0.402 | 36 | 0.356 | 0.371 |
| chain_car | 0.003 | 0.0074 | 0.111 | 0.115 | 38 | 0.105 | 0.109 |
| joints | 0.007 | 0.013 | 0.261 | 0.277 | 40 | 0.238 | 0.249 |

- The interpreter runs the scalar artifact 12 to 21x slower than AOT and 17 to 40x native scalar: playable for small
  scenes (a few hundred bodies at well under a millisecond a step), not for the benchmark sizes.
- **The SIMD release bake is safe on the interpreter.** WAMR 2.4.5's fast interpreter traps on a v128 local past
  frame cell 256; the bake's largest is cell 200 (`b2PushContactsL<v128>`, 45 v128 locals; solve 190, restitution
  171, prepare 111, warm start 93; `bench/as/tools/v128cells.py`), and the SIMD artifact runs bit exact on the
  interpreter, faster than the scalar one where the contact solver dominates (many_bodies 26.3 against 44.5 ms). It is
  not safe for a build without the binaryen post pass: plain `-O` leaves hundreds of v128 locals (finding 10).

## Changes, in commit order

Every change kept `KEEP_GOING=1 verify/run.sh` (16 modules) and `verify/traj.sh` (12 scenes, 18 from 3f4165b on) bit
exact with `RASC_FLAGS="-O"` and `RASC_FLAGS="-O --enable-simd"`, and the heap gate flat (pages after step 20 and 200
of 8 scenes, 11 from 3f4165b on), checked on the worktree before each commit. The A/B numbers below are interleaved
rounds of the same scene, min over rounds, taken through the session at loads from 5 to 200, so compare each pair,
not numbers across pairs.

### 1. The collide task binds each contact sim in place (f2c6d1b)

The narrow phase copied each `b2ContactSim` (200 bytes, about 50 leaves) out of its color array and stored it back,
since rasc refuses a `ref` into an `Array<T>` element. A `StaticArray` view over the array's buffer, as joints
already used, binds the sim by `ref` (`b2ContactSimView` then, `b2ArrayView` since change 8). C works through the pointer the same way. Most contacts
of a resting scene are recycled, which touches four fields and the manifold points, so the copies were most of the
task. many_bodies on V8 (3 rounds, load about 200): collide 1.02 to 0.28 ms, step median 2.54 to 2.05 ms on the SIMD
artifact and 4.51 to 3.38 on the scalar one.

### 2. Polygons, shapes and bodies by ref; one copy into the graph (88991c5)

- The polygon collide functions, `b2ComputePolygonFatAABB` and `b2ComputePolygonBounds` take `ref b2Polygon` (C's
  `const b2Polygon*`), and their world callers bind the shape through the view. A polygon by value was 36 wasm
  parameters per argument, read leaf by leaf out of the shape array.
- `b2GetShapeRadius` and `b2EncodeBodySimIndex` take `ref` (an 86 and a 26 leaf struct by value before).
- A contact that starts touching was copied out of the awake set, copied again in `b2AddContactToGraph` and pushed:
  it is now pushed once and its encoded body indices are written in place.

many_bodies' first step, where every contact is created and collided (V8, 7 rounds): collide 6.71 to 5.12 ms, pairs
5.18 to 4.68. Resting steps do not move.

### 3. Joint prepares read body sims in place (af8e3d5)

Each joint type's prepare copied both body sims (96 bytes each). Within the noise on the small joint scenes, kept as
C's access.

### 4. Kernels out of line, call free (468ce8b)

WAMR AOT keeps the memory base and bound in registers only in a function that makes no calls (`mem_space_unchanged`
in aot_llvm.c); in any other function every access reloads them. The release bake inlined the contact kernels and the
integrations into `b2ExecuteBlock`, which also calls the joint dispatchers, so every kernel access paid the reload
(223 reloads in its AOT code). `@noinline` on the five kernels, store impulses and the two integrations leaves each a
call free function of its own. many_bodies on AOT (4 rounds): 1.126 to 1.035 ms against native 0.851 (1.32 to
1.22x): prepare 0.077 to 0.064, warm start 0.165 to 0.140, store impulses 0.053 to 0.040, the integrations 25%. V8
unchanged (C 1.065, before 1.073, after 1.043 ms; 6 rounds).

### 5. Finalize bodies in a call free helper (48d2de3)

`b2FinalizeBodiesTask` calls `b2SolveContinuous`, the unstable body log and the fat AABB helpers, so its stores
reloaded the base too. The per body work is now `b2FinalizeBody` (`@noinline`), with the fat AABB helpers and
`b2DynamicTree_MarkProxyMoved` inlined into it; it returns whether the body is unstable or needs the continuous
solve, and the task does those two after it. Both touch only that body and static data (non bullet bodies sweep the
static tree only), and the order of the continuous solves is C's. AOT transforms 0.275 to 0.210 ms on many_bodies,
V8 unchanged.

### 6. Joint sims as C's union (ef3b039)

`b2JointSim` held one field per union member, 972 bytes against C's 252, and joint_grid (20,000 revolute joints)
ran 2.5x C wasm on V8 under load and 1.3x on a quiet machine. Padding C's own `b2JointSim` by 720 bytes slowed native
joint_grid from 4.7 to 11.1 ms, so the layout was the cost, not the code: V8 runs `b2SolveRevoluteJoint` in 509
instructions. The joint types now overlay one 160 byte payload, 256 bytes a sim. A type binds by `ref` through a view
at the sim's address (`b2RevoluteJointAt(sim)` and the like, joint.as); the solver and API functions take the sim's
address where they took `ref b2JointSim`. rasc range checks such a bind even under `unchecked` against the word
before the element read as a byte length (finding 11), so the payload is the first field and a guard word set to
0x7fffffff the last, and `b2CreateJoint` sets the guard on every new sim. Every joint type's prepare, warm start and
solve is also `@noinline`, as C keeps them, instead of one 12k instruction `b2SolveJoint`.

joint_grid on V8 (3 rounds, quiet): 3.80 to 2.86 ms against C's 2.88; rain (ragdolls) 2.96 to 2.72 against 2.46.

### 7. Contact churn and continuous (f7b778e)

- `b2CreateContact` builds the contact and its sim in place (it copied the contact out and back and pushed a sim
  built on the stack).
- `b2Array_RemoveSwap` moves a struct element with one `memory.copy`, as C's memcpy, instead of leaf by leaf.
- `b2ComputeShapeDistance` and `b2ComputeTimeOfImpact` take their inputs by `ref` (C's `const T*`; 45 and 59 leaves).
- The continuous query reads the hit body's sim in place (`b2MakeRelativeSweepAt`), where it copied it.
- `b2Collide`, `b2UpdateBroadPhasePairs`, `b2Solve` and `b2OverlapSensors` are `@noinline`, so a profile tells them
  apart and each keeps its own register state.

spinner on AOT (3 rounds): 4.91 to 4.79 ms (native 3.19), smash 0.0724 to 0.0697 (native 0.035); rain and tumbler
unchanged. In the AOT profile of spinner `b2Array_RemoveSwap` went from 2.55% to 0.31% of samples and
`b2CreateContact` from 5.45% to 3.96%.

### 8. One generic array view (30f56b6)

The per type views (`b2ContactSimView`, `b2ShapeView`, `b2BodyView`, ...) became one `b2ArrayView<A>` in body.as.
No change in code.

## What did not help

- **A call free prologue for the collide task** (the bounds test, inverse masses and recycling in a `@noinline`
  helper, the narrow phase call after it): AOT collide 0.230 to 0.214 ms on many_bodies, 1.94 to 1.99 on spinner, in
  4 rounds at load 45 to 58; not kept. The AOT profile of the collide task is flat (403 samples over 19.5 KB of code),
  each load behind its software bounds check.
- **The joint sim layout without the union**: putting `revoluteJoint` right after the header (same 972 bytes) bought
  15% on joint_grid; the union bought all of it.
- **`@noinline` on the joint types alone** (before the union): within the noise on V8 (17.5 against 18.4 ms at load
  20 to 30).

## What is left, and why

- **WAMR AOT on the churn scenes, 1.4 to 1.55x native** (smash, spinner, tumbler, rain): the narrow phase is 1.6 to
  2.4x native on AOT while it is 0.8 to 1.4x C on V8. The collide task, `b2CreateContact`, `b2DestroyContact` and the
  pair update call out (manifold functions, user callbacks, the allocator for array growth), so WAMR reloads the
  memory base and bound around their accesses, and every access carries a software bounds check. Calls between AOT
  functions go through WAMR's entry stub. Splitting out a call free part did not pay (above); hardware bounds checks
  (box3d-as perf_4: 5 to 10% on its collide heavy scenes) are not shippable yet.
- **V8, rain and smash 1.15 to 1.17x C**: pairs (1.26 to 1.33x) and collide (1.27 to 1.36x). The contact creation
  and destruction paths still copy `b2Contact` and edge structs through checked `Array` places, and the pair callbacks
  run through closures; C's are static functions inlined into the flush.
- **Prepare on AOT**, 1.3x native: the kernel is call free; what is left is the lane gathers on WAMR's code
  generation.
- **The interpreter**: 17 to 40x native scalar, the cost of WAMR's fast interpreter, not of the port's code (the
  same module is at C's speed on V8).

## How it was measured

- V8: `bench/web/bench.mjs` with per change bakes (`bench/web/out/as_simd_<tag>.wasm`, built with the release bake
  flags), and a scratchpad `ab.mjs` for quick interleaved A/B of two bakes in one process.
- WAMR AOT and interpreter: a scratchpad copy of `bench/as` per label with its own AOT cache, interleaved with the
  native driver (`aotab.py`); `bench/web/aot.mjs` for the final table.
- Static AOT code: `wamrc --format=object` with the ladder's flags (`--bounds-checks=1 --opt-level=3
  --cpu-features=+reserve-x18,+lse --tune-cpu=apple-m1`, which reproduce the cached `.aot` byte for byte) and
  `llvm-objdump`, counting instructions, calls and the `[x19, #0x178]` / `[x19, #0x1b8]` base and bound reloads per
  function.
- Profiles: `node --cpu-prof` on the release bake and on C's named build (`PROFILE=1 bench/web/build.sh`),
  `bench/as/profile.sh` (xctrace attached to rive-cli, symbolized by `tools/aotsym.py`), and xctrace on the native
  driver for the per function comparison.
- `node --print-wasm-code --no-liftoff` for V8's instruction counts per function.
