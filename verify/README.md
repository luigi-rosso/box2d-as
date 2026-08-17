# Phase A+B+C verification harness

`./run.sh` rebuilds everything and runs the node comparison twice: once with
the default (debug) rasc build and once with `-O2`. Results compare
**bit-exact for f32** (hex bit patterns; both-NaN counts as equal to absorb
platform NaN payload differences — box2d itself never branches on payloads).

## Pinned reference build flags (determinism contract for Phases B/C)

All differential reference builds of box2d MUST use:

```
clang -O2 -ffp-contract=off -DNDEBUG -DBOX2D_DISABLE_SIMD
```

- `-DBOX2D_DISABLE_SIMD` is box2d v3.1's scalar knob (CMake option
  `BOX2D_DISABLE_SIMD`, `src/core.h` maps it to `B2_SIMD_NONE` with
  `B2_SIMD_WIDTH 4`). This pins the scalar contact-solver path — the
  AVX2/SSE2/NEON wide paths in `contact_solver.c` reorder float ops and are
  not reproducible in scalar wasm. Phase B/C differential work must compare
  against this configuration only.
- `-ffp-contract=off` forbids fma contraction; wasm has no scalar fma, so a
  contracted reference build would diverge in the last bit on arm64/x86-fma.
- `-DNDEBUG` matches release semantics (`B2_ASSERT` off); vectors only feed
  inputs that also pass the AS debug asserts.

C sources pinned at `~/Projects/box2d` `v3.1.1-19-gc05c487`
(commit c05c48738fbe5c27625e36c5f0cfbdaddfc8359a).

## Files

- `cppref/ref_math.c` — math_functions/constants/id sweeps (grids, branch
  boundaries, denormals, fixed-seed xorshift fuzz).
- `cppref/ref_state.c` — id pool / b2IntArray growth / bitset / arena op
  traces with expected observable state per op.
- `cppref/ref_geom.c` — hull, polygon makers, mass, shape AABBs, point tests,
  ray casts, aabb.c ops. Lines are self-contained (inputs + expected).
- `cppref/ref_gjk.c` — GJK distance over dense shape-pair grids (deep
  overlap, touching, far apart; cold and warm cache with the cache contents
  compared), shape casts (grazing and initial-overlap included), TOI over
  seeded sweeps, and the geometry.c distance-dependent tail (point-in-polygon,
  rounded-polygon ray cast, `b2ShapeCast*` wrappers).
- `cppref/ref_manifold.c` — every collide pair function over pose ring x
  depth x rotation grids; compares point counts, world normal, point/anchorA/
  anchorB, separations, and packed feature ids bit-exactly; the chain-segment
  functions also compare the simplex cache out-state.
- `cppref/ref_tree.c` — seeded dynamic tree op traces (create/destroy/move/
  enlarge/rebuild) with a full-structure FNV hash after every op (parents,
  children, heights, flags, category bits, AABB bits, leaf user data, free
  list), query/ray/shape-cast visit orders (plus a clip-mode callback that
  shrinks maxFraction via `b2AABB_RayCast`), hash-set traces with slot-exact
  state hashes, and broad phase traces (move buffers, pair sweeps in creation
  order, pair set state). It `#include`s `dynamic_tree.c` for node internals,
  so run.sh must not link that file into it twice.
- `cppref/ref_dynamics.c` — Phase C step-trajectory generator. Builds 13
  scenes with the public API (pyramids, shape rain, six joint scenes, churn
  with mid-run create/destroy/sleep/wake, bullets, a determinism double-run)
  and steps them at dt=1/60 with 4 substeps, dumping per step: every live
  body's transform and velocity (hex f32), contact and touching counts, and
  FNV-1a 64 hashes over contact manifolds+impulses+feature-ids, island/
  solver-set/graph-color bookkeeping, per-body bookkeeping, and the event
  buffers. Links the whole box2d library.
- `verify.mjs` — replays every line against `verifyA.wasm`/`verifyB.wasm`
  (compiled from `../_verifyA.as`/`../_verifyB.as` with rasc, stub runtime).
- `verifyC.mjs` — drives `verifyC.wasm` (from `../_verifyC.as`) through the
  same scenes: `dynStart(scene)` builds it, `dynStep(step)` advances and
  stages the dump, getters return states and hashes for bit-exact comparison
  against `dynamics_vectors.txt`. The AS scene builders and hash streams
  mirror ref_dynamics.c statement for statement — keep them in lockstep.
  Phase C compiles with `--runtime minimal` (TCMS) and the harness calls
  `dynCollect()` between steps: the solver's transient objects exhaust memory
  under the stub bump allocator, and collection is only safe from an unwound
  stack (rasc wall: the incremental runtime crashed in `__visit` during
  mid-step collection, so Phase C sticks to minimal + explicit collect).

Phase D bench files: `cppref/ref_bench.c` prints the pyramid bench checksum
(decimal + f64 bits) from the C reference build; `bench_v8.mjs` runs
`bench.wasm` (from `../bench_stub.as`, stub runtime, the bake's configuration)
under node; `bench_cadence.mjs` sweeps collect cadence on `bench_minimal.wasm`
(from `bench_minimal.as`, minimal runtime). run.sh rebuilds both wasm modules
but does not run the benches.

TOI caveat: C's `b2TimeOfImpact` leaves `b2TOIOutput.point/normal`
uninitialized on the non-hit paths (and the s1-overlap failed path), so the
reference emits stack garbage there; the comparison checks point/normal only
for `b2_toiStateHit` and state+fraction always.

## Relaxed-fma flavor (opt-in artifact)

`--enable-relaxed-simd` (bake env `RIVE_RASC_RELAXED=1`) turns the 52 solver
velocity-update sites — exactly C's `b2MulAddW`/`b2MulSubW` call sites — into
`f32x4.relaxed_madd`/`relaxed_nmadd`. Fused fma is single-rounding
deterministic, so the flavor keeps a bit-exact differential: run.sh generates
`cppref/contact_solver_fma.c` from the pinned source (only those two scalar
bodies become `fmaf`), rebuilds `ref_dynamics_fma`/`ref_bench_fma`, and
replays phase C against `dynamics_vectors_fma.txt`. The fused reference
checksum differs from the deterministic lane's by design; the relaxed bake's
bits line must match it in every lane (verified identical across WAMR AOT
sw/hw and V8). Phases A/B carry no relaxed sites and are not re-run. The
interpreter refuses relaxed modules ("invalid opcode 0xfd 105/106") — the
relaxed artifact is AOT/V8 only, and wamrc needs wamr patch 0012
(fields_as `packages/runtime/scripting/wamr_patches/`). This leg requires
fma hardware (every supported target has it).
