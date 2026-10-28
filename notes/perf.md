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

# perf: the second pass

Scope: aabe2a7 to d24f4a3 on the port, and a read only look at how our rive runtime builds and configures WAMR
(rive-pinefall, master 8e7360ec48), with an experimental hardware bounds CLI built from draft PR #15153 in a scratch
worktree. Same machine as the first pass.

## Results

### V8 against C Box2D on wasm

`bench/web/bench.mjs`, 5 rounds, 140 runs, load 5.7 to 8.4, the release bakes of 13747d0 (before) and d24f4a3 (after),
mean ms per step after warmup, min over rounds; every hash matched.

| scene | native NEON | C wasm SIMD | AS SIMD before | AS SIMD after | AS / C before | AS / C after |
|---|---|---|---|---|---|---|
| large_pyramid | 1.13 | 1.33 | 1.35 | 1.32 | 1.02 | 0.99 |
| many_bodies | 0.862 | 1.04 | 1.02 | 1.01 | 0.98 | 0.97 |
| joint_grid | 2.19 | 2.94 | 2.97 | 2.91 | 1.01 | 0.99 |
| rain | 2.26 | 2.70 | 3.00 | 2.92 | 1.11 | 1.08 |
| smash | 1.04 | 1.28 | 1.50 | 1.42 | 1.18 | 1.11 |
| spinner | 3.26 | 3.96 | 4.48 | 4.32 | 1.13 | 1.09 |
| tumbler | 0.633 | 0.813 | 0.883 | 0.852 | 1.09 | 1.05 |

Stages of the fastest run, AS before, after and C: rain pairs 0.813, 0.744, 0.636 and collide 0.198, 0.188, 0.154;
spinner pairs 1.19, 1.10, 0.980 and collide 1.58, 1.53, 1.32; smash pairs 0.408, 0.400, 0.350.

### WAMR AOT against native

rive-cli's `--bench` release bake of `bench/as` with `--aot` and `RIVE_WASM_AOT_SYNC=o3`, the SIMD artifact, the live
game's wamrc (with patch 0037), interleaved with native NEON (`bench/web/run.mjs native_simd`): 4 rounds, 112 runs,
load 6.4 to 12.5, mean ms per step from step 1, min over rounds; every hash matched. Software bounds is master's CLI
(8e7360ec48) as shipped; hardware bounds is the CLI of draft PR #15153 (below).

| scene | native NEON | software bounds, before | software bounds, after | hardware bounds, after | before / native | after / native | hardware / native |
|---|---|---|---|---|---|---|---|
| large_pyramid | 1.14 | 1.36 | 1.31 | 1.24 | 1.19 | 1.15 | 1.09 |
| many_bodies | 0.884 | 1.07 | 1.06 | 1.00 | 1.20 | 1.20 | 1.13 |
| joint_grid | 2.22 | 2.89 | 2.92 | 2.41 | 1.30 | 1.32 | 1.09 |
| rain | 2.11 | 2.82 | 2.74 | 2.36 | 1.34 | 1.30 | 1.12 |
| smash | 0.957 | 1.37 | 1.32 | 1.17 | 1.44 | 1.38 | 1.22 |
| spinner | 3.40 | 4.91 | 4.64 | 4.14 | 1.45 | 1.37 | 1.22 |
| tumbler | 0.587 | 0.825 | 0.786 | 0.698 | 1.40 | 1.34 | 1.19 |

- The port's changes bought 3 to 6% on AOT where pairs and collide matter (rain, smash, spinner, tumbler).
- Hardware bounds take another 5 to 18%: AOT then runs at 1.09 to 1.22x native, from 1.15 to 1.38x.
- By stage, hardware bounds over native: collide 1.03 to 1.44x (software 1.47 to 1.79x), pairs 1.10 to 1.23x
  (software 1.16 to 1.51x), solve 1.06 to 1.14x (software 1.11 to 1.32x).

## WAMR bounds checks on AOT

### What is configured today

- **Hardware bounds are off everywhere.** `runtime/scripting/premake5_wamr.lua` defines `WASM_DISABLE_HW_BOUND_CHECK=1`
  unless premake gets `--wasm_hw_bounds`, which no build passes, so WAMR's darwin and linux `platform_internal.h` never
  define `OS_ENABLE_HW_BOUND_CHECK`. `WASM_DISABLE_STACK_HW_BOUND_CHECK=1` is always set: native stack overflow is
  checked in software too.
- **The tier ladder compiles software checked code.** `ModuleTierLadder::runWamrc` (runtime/src/wasm/
  module_tier_ladder.cpp) passes `--bounds-checks=1 --opt-level=3`, `--cpu=generic --cpu-features=+reserve-x18
  --tune-cpu=apple-m1` on Apple arm64, and `--rive-interrupt` for modules that import the host budget;
  `RIVE_WASM_AOT_SYNC=o0` swaps in `--opt-level=0`. Only a `RIVE_WASM_HW_BOUNDS` build asks for the `hw` species
  (`--bounds-checks=0 --stack-bounds-checks=1`, cached as `.hw.aot`). The iOS and Switch prelinks pass
  `--bounds-checks=1` as well.
- **Draft PR #15153 (`hw-bounds-default`) makes hardware bounds the default** on 64-bit macOS and Linux desktops
  (rive-cli, editor_native, evi), keeps the macOS editor on software bounds, adds `--wasm_sw_bounds`, and fixes what
  broke hardware bounds before (nested calls from another exec env, signal state on threads that boot files, the
  handler on the alt stack, `_setjmp` on Darwin).
- **The first pass measured a stale wamrc.** `~/Projects/tools/wamrc` (built 2026-10-20) predates patch 0037, which
  keeps the memory base and bound in registers across linear stores in functions that call (#15091, 2026-10-26). The
  "base reloads around the accesses of any function that makes calls" the first pass found were that missing patch.

### Static code

The release bake of `bench/as` through `wamrc --format=object` with the ladder's flags: instructions, memory base and
bound reloads (`[xN, #0x178]`, `#0x1b8`) and unsigned compare branches (the software checks).

| | old wamrc, software | current wamrc, software | current wamrc, hardware |
|---|---|---|---|
| module, instructions | 220,066 | 200,362 | 127,508 |
| module, base and bound reloads | 20,461 | 2,865 | 2,151 |
| module, check branches | 24,600 | 23,036 | 4,086 |
| collide task, instructions / reloads / checks | 4655 / 563 / 543 | 3917 / 12 / 491 | 2249 / 16 / 41 |
| `b2CollidePolygons` | 1710 / 234 / 198 | 1454 / 6 / 183 | 840 / 3 / 11 |
| `b2CreateContact` | 1866 / 309 / 300 | 1556 / 7 / 266 | 818 / 7 / 47 |
| `b2SolveContinuous` | 2238 / 172 / 277 | 2012 / 13 / 247 | 1169 / 6 / 40 |
| `b2PrepareContactsL<v128>` (call free) | 1068 / 2 / 126 | 1068 / 2 / 126 | 654 / 1 / 3 |

Hardware bounds take 36% of the module's instructions and 82% of its check branches; patch 0037 took 86% of the
reloads. Every function also keeps a 9 instruction entry stub that checks the native stack
(`--stack-bounds-checks=1`, since the runtime has no stack guard page).

### Measured

Hardware bounds against software bounds on the same port and wamrc, mean ms per step, min over rounds:

| scene | session 3 (after, load 6 to 13) | session 2 (before, load 7 to 51) | session 1 (before, load 17 to 69) |
|---|---|---|---|
| large_pyramid | -5% | -4% | -10% |
| many_bodies | -6% | -3% | -12% |
| joint_grid | -18% | -11% | -25% |
| rain | -14% | -6% | -1% |
| smash | -12% | -22% | -12% |
| spinner | -11% | -7% | -20% |
| tumbler | -11% | -13% | -15% |

Session 3 is the AOT table above; the louder sessions agree on the sign everywhere and scatter on the size. The
narrow phase gains most (collide 1.47 to 1.79x native with software bounds, 1.03 to 1.44x with hardware bounds), the
call free contact kernels least (solve 1.11 to 1.32x against 1.06 to 1.14x).

### Other wamrc knobs

- `--cpu=apple-m1` instead of `--cpu=generic --tune-cpu=apple-m1`: byte identical code.
- `--disable-aux-stack-check`: byte identical (rasc modules keep no C shadow stack).
- `--size-level=0` (large code model): 0.8% more instructions; `--size-level=1` aborts on arm64 ("Only small, tiny
  and large code models are allowed on AArch64"); 3, the default, is the small model.
- `--stack-bounds-checks=0` on top of hardware bounds (the native stack unchecked, so not shippable without a stack
  guard page): no wrapper stubs, but LLVM then inlines more and the module grows 9% (139,419 instructions).
  Against hardware bounds with the stubs (session 2, load 7 to 51): -9 to -10% on four scenes, +3 to +9% on three,
  so not resolved at that load.
- Patch 0037 alone (old against current wamrc, software bounds, 3 rounds at load 17 to 69): -2 to -9% on six
  scenes, +2% on large_pyramid.

### Constraints

- **Address space per memory.** Every module memory reserves 8 GB under hardware bounds, the main module and each
  lane, job, worker and processor instance. 64-bit only: 32-bit targets and memory64 modules keep software checks
  (wamrc forces them for memory64).
- **Per call cost.** Every call into wasm sets up the trap state: about 16 ns against 8 for a call into compiled code
  on an M4 Pro, once `_setjmp` replaces `setjmp` on Darwin (plain `setjmp` saves the signal mask with two system calls,
  about 170 ns, which doubled the draw visitor's cost per shape). Hosts that cross into wasm per shape or per event
  pay this; a physics step crosses once.
- **Signal handlers.** WAMR installs its SIGSEGV and SIGBUS handler once per process (patch 0036) and passes on every
  fault that is not a guard page. A handler installed later must chain to WAMR's, and nothing may put an older
  handler back over it. Catch2 does that around every test, so the PR turns Catch2's signals off on Linux; lldb
  stops on every guard page fault unless told to pass EXC_BAD_ACCESS through.
- **macOS editor.** Sentry's breakpad backend takes the Mach exception before WAMR's signal handler sees it and
  records a crash. Sentry has to move to crashpad first.
- **Windows.** Upstream WAMR's handler skips the faulting instruction (x86-64 only) instead of unwinding the call;
  it needs to unwind as the POSIX handler does. Windows on Arm cannot even skip.
- **Android.** ART keeps the app's handlers behind its own and a crash reporter installed after WAMR (Crashpad,
  Crashlytics) sees guard page faults first. Many arm64 devices give a process 512 GB of address space: about 60
  memories at 8 GB.
- **iOS.** Needs the extended virtual addressing entitlement and a count of how many 8 GB memories fit; the shipped
  modules are prelinked into the app with `--bounds-checks=1` today, so iOS also needs a prelink species change.
- **Switch (Apollo, Artemis).** nnSdk has no signal API (`wamr_nx/platform_internal.h`), so software bounds stay.
- **The interpreter** keeps software bounds in every build (patch 0009), and its memories reserve 8 GB too in a
  hardware bounds build.

## Changes, in commit order

Every change kept `KEEP_GOING=1 verify/run.sh` (16 modules) and `verify/traj.sh` (18 scenes) bit exact with
`RASC_FLAGS="-O"` and `RASC_FLAGS="-O --enable-simd"`, and the heap gate flat (11 scenes), checked on a snapshot of the
tree before each commit. The A/B numbers are interleaved runs of the release bake on V8 (`ab.mjs`, mean ms per step
after warmup), so compare within a pair.

### 1. The pair filter reads in place (aabe2a7)

`b2WorldPairFilter.shouldCreatePair` read every shape field through a checked `Array` place (`shapes[id].bodyId`, then
`.sensorIndex`, `.filter`, `.type`, ...); it binds both shapes by `ref` through `b2ArrayView` once, as C's pointers.
`b2ShouldBodiesCollide` does the same for the two bodies and each joint of the joint walk. The candidate buffers of
`b2FlushCandidatePairs` (32 entries, filled up to `batchCount`) and the hash set slots (`index` masked by
`capacity - 1`, `items` holding `capacity` items) read `unchecked`. AOT code of the flush 635 to 475 instructions,
87 to 51 software checks. Rain on V8: pairs 1.747 to 1.676 ms at load 30 to 40, within the noise on smash.

### 2. Tree nodes are C's 32 bytes (f912d71)

`b2TreeNode` held C's `height`/`shapeIndex` union as two fields after an 8 byte padding: 40 bytes, so a sibling pair
(80 bytes) no longer fit the 64 byte cache line C lays them out for. `shapeIndex` now takes half of the padding (unused
in 2D), and a node is 32 bytes. Rain on V8, same session as 1: pairs 1.676 to 1.608, step 6.56 to 6.39 ms (load 30
to 40).

### 3. Polygon collide temporaries in module scratch (d24f4a3)

`let localPolyA: b2Polygon;` is zeroed on every call (36 words, written to the shadow stack), and so was each
`b2MakeCapsule` result, which also came back as 36 return values. C leaves its two local polygons uninitialized and
reads only the first `count` entries. `b2CollidePolygons` now builds them in `b2_localPolygons`, and `b2MakeCapsule`
fills `b2_capsulePolygon` for its three callers; none of these is reentrant. C's wasm build also autovectorizes the
transform loops (38 `f32x4.mul`) where the port's stay scalar, but most of the cost was the zeroing. AOT code of `b2CollidePolygons` 1454 to 1157 instructions (183 to 115 software checks), the collide task 3944
to 3290. Smash on V8: collide 0.960 to 0.878, step 2.57 to 2.47 ms. With 1 and 2, before against after all three: rain step
3.16 to 2.88 against C's 2.71, spinner 4.90 to 4.73 against 4.37 (load 10 to 20).

## What did not help

- **v128 pair masks** (`b2OverlapPairMask` and `b2OverlapNodePairMask` as C's SSE2 and NEON code: four loads, zips,
  `f32x4.le`, `i32x4.bitmask`): bit exact, but slower on V8, rain pairs 1.279 to 1.356 and smash 0.607 to 0.631.
  V8's arm64 lowering of the shuffles and `bitmask` costs more than 16 scalar compares; C's own SSE2 build pays the
  same (its `b2CollideCrossPairs` is 3000 instructions on V8 against the port's 1744).
- **Module scratch for the chain segment and GJK temporaries** (`b2CollideChainSegmentAndPolygon`'s vertex and normal
  arrays and distance input, `b2ComputeShapeDistance`'s local proxy): spinner on V8 4.84 to 4.94 ms, within the noise;
  on AOT the chain function lost 480 instructions but gained 67 software checks (scratch reads are memory reads).
- **Unchecked pair stacks and inlined friction and restitution mixing** (the cross pair and proxy stacks are bounded
  by `B2_TREE_STACK_SIZE`; C inlines `b2MixFriction` into `b2UpdateContact`): spinner 4.473 to 4.446 ms, rain 3.044 to
  3.023 on V8 (4 rounds, load 7), under 1%; not kept.
- **C's `b2Shape` union**: the port's `b2Shape` is 368 bytes against C's 280 (the union is one field per member).
  Padding C's `b2Shape` by the same 88 bytes moved native rain, smash, spinner, tumbler and many_bodies by -1 to +3%
  (5 interleaved rounds), so the 250 call site refactor the joint union needed is not worth it here.
- **No per step hashing or growth in the pair stage**: v3.2 has no move buffer (moved flags live in the tree), the
  pair set is hashed once per created or destroyed contact as in C, and every pair buffer grows only.

## What is left, and why

- **V8, rain and smash 1.08 to 1.11x C**: pairs 1.12 to 1.17x C (`b2DynamicTree_Rebuild` at 1.7x C's samples:
  `b2TreeNode` copies go field by field where C moves 32 bytes; the cross pair stack and tree node reads stay checked)
  and collide 1.02 to 1.22x (C's wasm build vectorizes the polygon transforms; the collide task itself is at C's
  cost).
- **WAMR AOT**: see the bounds section above; with hardware bounds and the current wamrc the port runs at 1.09 to 1.22x
  native. What remains is WAMR's code generation (every call between AOT functions goes through a native stack check
  stub) and the narrow phase, whose profile is flat.

## How it was measured

- V8: `bench/web/bench.mjs` (`IMPLS`, `SCENES`, `ROUNDS`), and a scratchpad `ab.mjs` with the run mean.
- WAMR AOT: a scratchpad driver (`aotvar.mjs`) interleaving native NEON (`bench/web/run.mjs native_simd`) with rive-cli
  variants, each a CLI, a wamrc, an AOT cache and a project copy of `bench/as` (old and new `lib/`).
- The hardware bounds CLI: `git worktree add --detach <scratch> origin/hw-bounds-default` (8eca0f0153), then
  `packages/rive-cli/build.sh release`; it caches `.hw.aot` artifacts. The live game's wamrc
  (`~/Projects/rive-parallel-calls/wamrc-build/wamrc`, patches through 0041 less 0039) for both modes.
- Static AOT code: `wamrc --format=object` with the ladder's flags, `llvm-objdump`; `bench/as/profile.sh` now uses the
  ladder's flags and handles `.hw.aot`.
