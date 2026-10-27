# solver: the step solver and the contact solver (wave 2)

| C | AnimaScript |
|---|---|
| `src/solver.h`, `src/solver.c` | `lib/solver.as` |
| `src/contact_solver.h`, `src/contact_solver.c` | `lib/contact_solver.as` (overflow kernels, the `_Wide` dispatchers) |
| `src/contact_solver_wide.inl` at width 4 under `B2_SIMD_NONE` (`contact_solver_w4.c` with `BOX2D_DISABLE_SIMD`) | `lib/contact_solver.as` (the `_WideW4` kernels) |

Functions are in C order. solver.c is ported in full: integrate velocities and positions, the continuous
context and query callback, `b2SolveContinuous`, `b2FinalizeBodiesTask` (move events, sleep candidates, enlarged
sim bits, fast and bullet flags), block dims, blocks, stages, `b2ExecuteBlock`, `GetWorkerStartIndex`,
`b2ExecuteStage`, `b2ExecuteMainStage`, `b2SolverTask` (orchestrator and spinner), `b2BulletBodyTask`, `b2Solve`
(setup, island split hand off, solve, finalize, joint events, hit events, refit, bullets, sensor hits, island
sleep).

## Gates

- `verify/run.sh contact_solver`: 400 synthetic scenarios (two colors with tail lanes, kinematic bodies shared
  across lanes, static lanes, one and two point manifolds with a stale second point, restitution, rolling
  resistance, tangent speed, hit events, warm starting and restitution propagation on and off, prepare and
  store split into two blocks at a random wide index), each dumping states, every wide constraint word, every
  overflow constraint word and the manifolds after prepare, 4 sub-steps of warm start / push / position
  integration / relax, 2 restitution iterations and store. 70361 lines, 1.94 M values, 0 mismatches with `-O`,
  `-O --enable-simd`, debug, and `-D B2_ENABLE_ASSERT` with and without SIMD. A planted reorder in the solve
  kernel shows up in both artifacts, a planted change in the v128 scatter only in the SIMD one.
- `verify/traj.sh` (all 12 scenes, once the world landed): pass with `-O`, `-O --enable-simd` and debug.
  With `-D B2_ENABLE_ASSERT` dynamic_mover traps in `b2GetJointConstraintTorque` (joint.as) from the scene's
  probe; the other 11 pass.
- `verify/heap.mjs`: pyramid, shape_rain, bullets, joints 993 pages after step 20 and step 200, many_bodies
  1490 and 1490 (also with SIMD).

## Kernel design

- One source for both artifacts: `b2PrepareContactsL<T>`, `b2WarmStartContactsL<T>`, `b2PushContactsL<T>`,
  `b2SolveContactsL<T>`, `b2ApplyRestitutionL<T>`, generic over `v128` (four lanes per call) or `f32` (one
  lane per call, the wide field addresses offset by 4 lane bytes). The `_WideW4` entry points dispatch with
  `if (ASC_FEATURE_SIMD)`.
- Lanes of one wide constraint touch distinct dynamic bodies (graph coloring), static lanes read
  `b2_identityStates` and kinematic lanes are never written, so one lane at a time gives C's scalar bits.
- C's decisions across lanes are taken before the lane loop, from memory: prepare's hit event and restitution
  test from the lane contacts, solve's `b2AllZeroW( rollingResistance )` and restitution's
  `b2AllZeroW( restitution )` through `b2AllZeroAt`. Empty tail lanes are computed, not skipped, as C does.
- Body states gather as C's SSE2 `b2GatherBodies`: the 32 byte state is rows `[vx vy w flags]` and
  `[dpx dpy dqc dqs]`, loaded and transposed with the 0 4 1 5 zips (`b2TransposeL`). The scatter zips v, w
  and the gathered flags back and stores the first row of each lane whose gathered flags have the dynamic bit,
  so the flags lane rewrites the bits it read. The f32 scatter stores the three floats as C's scalar path.
- The manifold point gather reads `[anchorA anchorB]` and `[separation normalImpulse tangentImpulse ...]` as
  two rows per lane, as C's NEON/SSE2 prepare does; the layout is asserted under `B2_ENABLE_ASSERT`.
- Store impulses reads `( (const float*)&c->field )[lane]` as `load<f32>` at field plus 4 lane and writes the
  manifold through its address; one code path for both artifacts.
- Exact v128 forms come from `lib/simd.as` (mul then add, compare plus bitselect). `b2RightPerpW` is a
  negate and warm start's tangent is `0 - x`, as in C; they differ for a zero normal component.

## Scratch

`b2SolverScratch` (module global `b2_solverScratch`) owns C's stack allocations, grow only
(`count + count / 4 + 16`), shared by every world since one `b2Solve` runs at a time: bullet bodies, wide
constraints, overflow constraints, stages (objects reused), body / contact / joint / graph sync blocks, the
per color arrays C keeps on `b2Solve`'s stack, the prepare spans (`B2_GRAPH_COLOR_COUNT + 1` objects each),
and the worker contexts. Every reader stays inside the counts the step writes, so stale contents are never
read. The tail wide slot of each color is zeroed as C's memset does. A stage keeps C's block pointer as the
block array plus `blockStart`; a color keeps its wide constraint pointer as `wideConstraintStart` into
`b2StepContext.wideContactConstraints` (field added, written by `b2Solve` before any reader).

## Raw addresses and why they are in range

- Wide constraints: block ranges cover `[0, wideConstraintCount)` of a color or `[0, wideContactCount)` of the
  step, and `b2Solve` sized the scratch for `wideContactCount`; lane offsets are below 16 bytes in a 16 byte
  field, index lanes below 16 in a `FixedArray<i32, 4>`.
- Body states: base 1 indices come from prepare, which decodes the contacts' encoded awake indices, so they
  are below the awake state count; 0 selects `b2_identityStates`. The overflow kernels use `b2StateAddress`.
- Contact sims: span counts are the colors' `contactSims.length`, unchanged during the solve; empty lanes
  read `b2_zeroContactSims`, which is never written (normal velocities only go to lanes with points).
- Integrations and finalize: indices below the awake body count the body blocks cover; `bodyId` of an awake
  sim is a live body; move events were resized to the awake count. Bullet indices were recorded by finalize.
- `b2SolveContinuous` holds the fast body sim address across the tree queries; nothing grows the awake sims
  during a step.

## Deviations

- Tasks run inline as C's default `enqueueTaskFcn` does: worker 0 claims the orchestrator slot and runs every
  stage, the other workers see the finish sentinel at once and return, the caller's `b2SolverTask` loses the
  claim. `taskCount` counts as C's. The island split runs inline at C's enqueue point; the user tree task is
  never pending here.
- Atomics are plain fields behind C named stand-ins (`b2AtomicFetchAddInt`, ...); `b2Pause`, `b2Yield` empty.
- Profile timers use `b2GetMillisecondsAndReset` through a `FixedArray<u64, 1>` holder (no `ref` to a local).
- The continuous context is a module object with a `visit` method, so the tree queries allocate nothing.
- `b2LogUnstableBody` builds the `%s` name from the body's zero terminated `name` bytes.
- Skipped: Tracy zones, `B2_VALIDATE` code, `b2ValidateSolverSets` (validation only). `b2BodyStateW` is
  declared but the kernels hold its lanes in locals. Width 8 is not ported (`b2GetSIMDWidth` is 4).

## rasc findings

- Plain `-O` runs rasc's inliner without binaryen's local coalescing, so the v128 kernels carry hundreds of
  v128 locals (`b2SolveContactsL<v128>`: 793, frame offset 3320 by box3d-as `bench/as/tools/v128cells.py`).
  WAMR's fast interpreter traps past cell 256, so the SIMD artifact must be checked on the release bake
  (binaryen post-pass), as box3d-as did; no `wasm-opt` is installed here, so that number is still open. The
  2D kernels are smaller than box3d-as's, which landed at 53 v128 locals after round 2.
- Nothing else new: the known findings (no `ref` to a primitive local, `T?` instead of `T | null`, whole
  element stores into an imported module's struct `StaticArray` through a local alias) were applied up front.
