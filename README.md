# box2d_as — Box2D v3 on AssemblyScript (Phases A+B+C: foundations + collision + dynamics)

Port of Box2D v3 (canonical C sources at `~/Projects/box2d`, pinned at
`v3.1.1-19-gc05c487`) to AssemblyScript compiled with rasc. The Luau port at
`../box2d/` defines the feature subset only; every algorithm, constant, and
layout decision comes from the C sources.

Layout mirrors the C `src/` decomposition 1:1 (`lib/<name>.as` per
`src/<name>.c/.h`); shared public types from `include/box2d/collision.h` and
`include/box2d/id.h` live in `lib/collision.as` and `lib/id.as`.

## Porting conventions (decided in Phase A, binding for B/C)

- **All arithmetic f32.** Box2D v3 is float32 throughout; AS native f32
  reproduces C exactly (proved by the draco campaign and re-proved by this
  phase's differential harness). Float literals are written with `<f32>` casts
  where the context could promote to f64.
- **C by-value structs become AS classes with pure functions.** Every function
  returns a fresh object and never mutates its inputs, matching C's copy
  semantics. Struct assignment in C (`a = b`) is `b.clone()` in AS whenever
  the target outlives the statement. C pointer-out parameters become small
  result classes (`b2LengthAndNormal`) or documented mutation
  (`b2EnlargeAABB` mutates its first argument like the C pointer version).
- **C shared const statics become factory functions** (`b2Vec2_zero()`,
  `b2Rot_identity()`, ...) because AS class instances are mutable references.
- **`array.h` macro arrays**: `b2IntArray` + free functions carry the int
  instantiation with the exact C growth rule (`capacity < 2 ? 2 :
  capacity + (capacity >> 1)`; `reserve` only grows; `create(n)` allocates
  exactly `n`). `b2Array<T>` is the generic instantiation for struct element
  types with identical semantics; `add()` returns a default-constructed
  element where C returns a raw slot pointer. Later phases declare
  `b2BodyArray`-style aliases as `b2Array<b2Body>` uses.
- **Fixed C arrays** (`points[B2_MAX_POLYGON_VERTICES]`) become
  `StaticArray<b2Vec2>` pre-filled with fresh instances via `makeVecArray`,
  so slots are never null; `count` gates the live range as in C.
- **Macros that scale with `b2_lengthUnitsPerMeter`** (`B2_LINEAR_SLOP`,
  `B2_HUGE`, ...) become functions in `constants.as`; fixed-value macros stay
  consts.
- **Arena allocator** returns raw `usize` pointers into an ArrayBuffer block
  (heap-fallback entries pin their own ArrayBuffer). Offset math including the
  32-byte size rounding is verbatim; base-pointer 32-byte alignment is a SIMD
  concern the scalar lane does not need.
- **`B2_ASSERT` → `assert()`** (checked in debug rasc builds, elided under -O).
- **remainderf** maps to `NativeMathf.rem` (musl remquof port, IEEE-exact);
  never use `%` on floats (that is fmod semantics).

## Phase A contents

`math_functions`, `constants`, `core` (+ `ctz.h` helpers), `array`, `bitset`
(incl. `b2CountSetBits`, which upstream parks in `table.c`), `arena_allocator`,
`id`, `id_pool`, `aabb`, `hull`, `geometry` (everything not needing the GJK
distance module), `collision` (Phase A types).

## Phase B contents (collision layer)

- `distance` — GJK core: `b2ShapeDistance` (+ simplex cache), `b2ShapeCast`
  (conservative advancement), `b2SegmentDistance`, `b2MakeProxy`,
  `b2GetSweepTransform`, and the full TOI machinery
  (`b2MakeSeparationFunction` / `b2TimeOfImpact`); the Luau subset uses TOI so
  it is in scope. The C debug-simplex capture is omitted (`simplexCount` is
  always 0, matching the NDEBUG reference).
- `manifold` — every collide pair function incl. the chain-segment family;
  feature-id packing (`B2_MAKE_ID`) and anchors are bit-exact.
- `dynamic_tree` — full tree: SAH insert with rotations, remove, move,
  enlarge, category bits, queries, ray/shape casts, and the median-split
  bottom-up `b2DynamicTree_Rebuild` (the compiled default heuristic; the SAH
  bin path is `#if`'d out upstream and not ported). The C node unions become
  separate fields; every read site only sees the member last written for that
  node role. Callbacks are abstract classes (no C function pointers in wasm);
  the traversal stack is a shared static, so callbacks must not re-enter tree
  queries (C uses per-call stack locals; box2d itself never nests).
- `table` — `b2HashSet` with the exact murmur hash and backward-shift
  deletion; slot layout is deterministic and compared exactly.
- `broad_phase` — move buffers, proxy lifecycle, `b2UpdateBroadPhasePairs`
  pair sweep with the exact de-duplication and creation order (per-move pair
  lists are prepend-linked in C, so creation order is reversed query order).
  The world-dependent parts of `b2PairQueryCallback` (same body, sensors,
  filters, joints, custom filter) hook in through `b2PairFilter` for Phase C;
  adding created pairs to `pairSet` stays with the caller like
  `b2CreateContact` in C.
- `geometry` completions: `b2PointInPolygon`, rounded-polygon ray cast tail,
  `b2ShapeCast{Circle,Capsule,Segment,Polygon}`.

Still deferred: `b2CollideMover*` (the Luau subset does not use the mover) and
`b2DynamicTree_GetByteCount` (allocation bookkeeping with no AS equivalent).

## Phase C contents (dynamics core)

Ported 1:1 from `src/`: `types` (def structs + defaults), `physics_world`
(world lifecycle, `b2World_Step` orchestration, narrow phase + contact state
transitions, queries, events; C's `world.c`), `body`, `shape`, `contact`,
`island` (merge/split), `solver_set` (sleep/wake/merge/transfer),
`constraint_graph` (color assignment), `joint` + the six subset joints
(`distance_joint`, `revolute_joint`, `prismatic_joint`, `weld_joint`,
`motor_joint`, `wheel_joint`), `contact_solver` (TGS soft solver), `solver`
(stage machinery, continuous collision, sleeping), `sensor` (bookkeeping only).

Phase C decisions:

- **Single threaded.** `workerCount` is always 1 and every C task enqueue runs
  inline over its full range (exactly C's default `b2DefaultAddTaskFcn`). The
  solver stage/block machinery is ported structurally; the block sync-index
  CAS reduces to a plain compare-and-set, so worker 0 claims blocks in the
  same forward order as a single-threaded C run. Atomics are plain ints.
- **Scalar 4-lane contact solver.** The reference build pins
  `-DBOX2D_DISABLE_SIMD` (`B2_SIMD_NONE`, width 4); `contact_solver.as` ports
  that lane-packed scalar path verbatim (b2FloatW as a 4-float class, exact
  per-lane op order, gather/scatter with per-lane null/dynamic guards). C's
  pointer-sliced per-color constraint views become per-color StaticArrays of
  reference copies into the step's shared constraint array.
- **Continuous collision is in scope** (the Luau subset uses bullets):
  `b2SolveContinuous`, the bullet pass, and the enlarged-AABB refit are ported.
- **Sensors are deferred.** Shapes keep the `sensorIndex` plumbing (so pair
  filtering and mass are exact) and sensor create/destroy bookkeeping exists,
  but `b2OverlapSensors` is a no-op and no sensor events are generated. The
  Luau subset never consumes them.
- **Debug draw, the mover queries, and memory stats are not ported.**
  Validation matches the NDEBUG reference build (no-ops).
- **C unions become nullable fields** (`b2Shape` geometry, `b2JointSim`
  per-type data, `b2GraphColor` transient constraints), exactly one set.
- **memcpy transfers become reference moves.** Every solver-set/graph transfer
  site removes or destroys the source slot right after the C memcpy, so
  pushing the same reference is observationally identical; in-place slot
  resets keep the object identity (parents hold references).
- **Function-pointer callbacks become base classes** with overridable methods
  (friction/restitution mixing, custom filter, pre-solve, query callbacks);
  the world-dependent pair filtering plugs into Phase B's `b2PairFilter` hook
  and contact creation consumes the pair sweep in C's creation order.
- The world table is `b2_worlds[128]` like C, ids/generations included.

## Verification

`verify/run.sh` compiles C reference generators against the canonical box2d
sources (`clang -O2 -ffp-contract=off`, see `verify/README.md` for the pinned
determinism flags), regenerates the vector files, compiles the AS drivers with
rasc, and runs the node comparison. f32 results compare **bit-exact** (hex bit
patterns, both-NaN allowed as equal for payload differences).

Phase C compares full step trajectories: 13 scene families (pyramids of 5/10/15
rows through sleep, mixed-shape rain, one scene per joint type, sleep/wake/
destroy churn, bullets/continuous, plus a determinism double-run) stepped for
120-320 steps each, comparing every body's transform and velocity per step and
FNV-1a hashes of contact manifolds+impulses, island/solver-set/graph-color
bookkeeping, and event buffers. Current totals: A+B 813,926 checks and C
1,046,110 checks, all green on both the debug and -O2 rasc builds.

## Phase D: benchmark and performance pass

`bench.as` replicates the matrix's Luau pyramid workload
(`luau_wasm/tests/box2d/driver.luau`) exactly: 5-row pyramid of 1x1 dynamic
boxes (spacing 1.1) on a 100x2 static ground at y=-1, gravity (0,-10), sleep
off, 240 steps of dt=1/60 with 4 substeps, checksum = sum(p.x + p.y * 3) over
the 15 dynamic bodies in creation order (f64 accumulation). The in-bake bench
in `main.as` prints a best-of-5 `BOX2D-BENCH:` sentinel plus the checksum's
f64 bit pattern; `verify/cppref/ref_bench.c` prints the same scene's checksum
from the C reference build, and the two match bit-exactly
(`40549f023f991c00`). `bench_stub.as` carries the stub-runtime iteration
protocol: one warmup run lands lazy statics before an `__mark`, then every
timed run rewinds with `__resetTo` after restoring the world-table slot to a
pre-mark object.

The performance pass (bit-exactness gated by the full differential harness
after every family) landed: `unchecked()` in the array containers and the
solver/contact inner-loop index sites; destructive lane-wise wide ops on
module-static scratch in `b2SolveContactsTask` / `b2WarmStartContactsTask` /
`b2GatherBodiesTo` (removes the per-op `b2FloatW` churn; identical per-lane
arithmetic in identical order); scratch local polygons and transforms in
`b2CollidePolygons` / `b2FindMaxSeparation` (the escaping manifold stays
freshly allocated because contact update keeps the old manifold for warm-start
matching); and in-place scalar integrate velocity/position tasks. Net effect:
V8 24.7 -> 8.7 ms/run, WAMR interp 196 -> 97 ms/run.

Collect cadence (V8, minimal runtime, `verify/bench_cadence.mjs`): collecting
every step costs 98 ms/run, every 4/16/64 steps 42/27/23 ms, between runs only
17 ms — versus 8.7 ms for the stub runtime with mark/reset. The bake compiles
AS with the stub runtime (and `-O2` + wasm-opt via `dangerouslyFast`), so the
shipped configuration is stub + arena rewind; TCMS + periodic collect remains
the verify-harness configuration where memory, not speed, is the constraint.
