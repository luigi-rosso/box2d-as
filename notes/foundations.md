# foundations: wave 1

The base modules every later wave imports, and the verification harness.

## What is ported

| C | AnimaScript |
|---|---|
| `include/box2d/math_types.h`, `math_functions.h`, `src/math_functions.c` | `lib/math_functions.as` |
| `include/box2d/base.h`, `src/core.h`, `src/core.c`, `b2Hash` and the tick functions of `src/timer.c` | `lib/core.as` |
| `include/box2d/constants.h` | `lib/constants.as` |
| `include/box2d/id.h` | `lib/id.as` |
| `include/box2d/types.h`, `src/types.c`, the joint and explosion defaults of `src/joint.c` | `lib/types.as` |
| the types of `include/box2d/collision.h`, the tree types of `src/dynamic_tree.h` | `lib/collision.as` |
| the bit helpers of `src/platform.h` (v3.1's `ctz.h`) | `lib/ctz.as` |
| `src/container.h` | `lib/container.as` |
| `src/bitset.h`, `src/bitset.c`, `b2CountSetBits` from `src/table.c` | `lib/bitset.as` |
| `src/id_pool.h`, `src/id_pool.c` | `lib/id_pool.as` |
| `src/table.h`, `src/table.c` | `lib/table.as` |
| `src/simd.h` and `src/simd.c` scalar branches, the `B2_SIMD_NONE` lane math of `src/contact_solver_wide.inl` | `lib/simd.as` |

## Gates

`verify/run.sh`, bit exact, with `RASC_FLAGS="-O"`, `"-O --enable-simd"`, `""` (debug) and
`"-O -D B2_ENABLE_ASSERT"` (no assert fires):

| module | lines | values |
|---|---|---|
| bitset | 4307 | 48451 |
| container | 1255 | 5361 |
| ctz | 40009 | 239389 |
| id_pool | 8144 | 16514 |
| math | 98461 | 1362178 |
| simd | 105000 | 3045000 |
| table | 190497 | 592776 |

math (183 lines) and simd (20 lines) run with `nanEqual`: those lines differ only in NaN sign or
payload, where clang at -O2 turns `-s * v` into `-(s * v)` (fnmul) or commutes an operation with
two NaN operands. `NAN_EXACT=1 node verify/math.mjs` lists them.

The simd driver also runs every `b2FloatW` op through the lane generic `L` helpers, per lane as
f32 and, under `--enable-simd`, as v128, and prints an extra line on any lane that differs from the
`b2FloatW` result (checked: a planted bug in either path shows up).

`verify/traj.sh`: plumbing only, the world is not ported. `NATIVE=1 verify/traj.sh` builds
`verify/scenes/bin/traj_scalar` and `traj_simd` and records `pyramid` (56 bodies, 300 steps,
settles and sleeps). The scalar and NEON recordings are identical. The script stops with a
message until `lib/box2d.as` exists; `verify/scenes/scenes.as` and `traj.as` are written against
the public API names (`b2CreateWorld(def)`, `b2CreateBody(worldId, def)`,
`b2CreatePolygonShape(bodyId, shapeDef, polygon)`, `b2MakeBox`, `b2MakeSquare`, `b2World_Step`,
the `b2Body_Get*` and `b2World_Get*` readers) and have never been compiled. The wave that ports the
world fixes whatever signature differs.

## Struct or class, for the whole port

A struct when the C type is plain data. A class when it holds a pointer, a `b2Array`, a string or
a callback, or when C only reaches it through a pointer it owns. rasc structs cannot hold strings,
arrays or closures (E: "value class fields must be primitives or value classes").

Structs:

- math: `b2Vec2`, `b2CosSin`, `b2Rot`, `b2Transform`, `b2Mat22`, `b2AABB`, `b2Plane`
  (`b2Pos`, `b2WorldTransform` are type aliases)
- ids: `b2WorldId`, `b2BodyId`, `b2ShapeId`, `b2ChainId`, `b2JointId`, `b2ContactId`
- geometry: `b2RayCastInput`, `b2ShapeProxy`, `b2ShapeCastInput`, `b2CastOutput`, `b2MassData`,
  `b2Circle`, `b2Capsule`, `b2Polygon`, `b2Segment`, `b2ChainSegment`, `b2Hull`,
  `b2SegmentDistanceResult`, `b2SimplexCache`, `b2DistanceInput`, `b2DistanceOutput`,
  `b2SimplexVertex`, `b2Simplex`, `b2ShapeCastPairInput`, `b2Sweep`, `b2TOIInput`, `b2TOIOutput`,
  `b2ManifoldPoint`, `b2Manifold`, `b2LocalManifoldPoint`, `b2LocalManifold`, `b2TreeStats`,
  `b2BoxCastInput`, `b2TreeNode`, `b2TreeProxy`, `b2PlaneResult`, `b2CollisionPlane`,
  `b2PlaneSolverResult`
- defs and results: `b2RayResult`, `b2Capacity`, `b2MotionLocks`, `b2Filter`, `b2QueryFilter`,
  `b2SurfaceMaterial`, `b2ShapeDef`, `b2Profile`, `b2Counters`, `b2JointDef` and every joint def,
  `b2ExplosionDef`, every event struct, `b2BodyMoveEvent`, `b2JointEvent`, `b2ContactData`,
  `b2ReplayInfo`, `b2ReplayQueryInfo`, `b2ReplayQueryHit`, `b2Version`, `b2Atomic*`
- internals: `b2Body` (`name` is `FixedArray<u8, 11>`), `b2BodySim`, `b2BodyState`, `b2Shape` (the
  union is one field per member), `b2ShapeExtent`, `b2Contact`, `b2ContactEdge`, `b2ContactSim`,
  `b2Joint`, `b2JointEdge`, `b2JointSim` and each joint type (union, one field per member),
  `b2ContactLink`, `b2JointLink`, `b2IslandSim`, `b2SensorHit`, `b2Visitor`, `b2SetItem`,
  `b2Softness`, `b2SolverBlock`, `b2SyncBlock` (cache line padding dropped),
  `b2ContactConstraint`, `b2ContactConstraintPoint`, `b2FloatW`, `b2Vec2W`, `b2RotW`,
  `b2ContactConstraintWide`, `b2BodyStateW`, `b2Candidate`, `b2NodePair`, `b2IndexPair`,
  `b2TreeBin`, `b2TreePlane`, `b2RebuildItem`, `b2CopyItem`, `b2BlockDim`, `b2ShapeCastData`

Classes:

- `b2WorldDef` (callbacks), `b2BodyDef` (`name` string), `b2ChainDef` (point and material arrays),
  `b2SensorEvents`, `b2ContactEvents`, `b2BodyEvents`, `b2JointEvents` (they hold the world's event
  arrays plus C's counts), `b2PreSolveManifold` (see deviations)
- `b2DynamicTree`, `b2BitSet`, `b2IdPool`, `b2HashSet`, `b2ChainShape`, `b2Sensor`,
  `b2SensorTaskContext`, `b2SensorOverlaps`, `b2Island`, `b2SolverSet`, `b2GraphColor`,
  `b2ConstraintGraph` (colors in a `StaticArray<b2GraphColor>`), `b2BroadPhase` (trees in a
  `StaticArray<b2DynamicTree>`), `b2TaskContext`, `b2World`, `b2StepContext`, `b2SolverStage`,
  `b2WorkerContext`, `b2ContactPrepareSpan`, `b2ContactCollideSpan`, `b2JointPrepareSpan`
  (C pointers into sim arrays; keep the array plus an offset), `b2PairContext`, `b2CrossContext`
  and the query contexts in `physics_world.c`
- `b2SeparationFunction` holds pointers to the two proxies: a struct that copies both proxies is
  also correct, since C never writes through them; pick by profile.

Where this differs from box3d-as: Box2D's `b2ShapeProxy`, `b2DistanceInput`, `b2ShapeCastInput`,
`b2ShapeCastPairInput`, `b2TOIInput`, `b2LocalManifold`, `b2ShapeDef`, `b2ContactData`, `b2Shape`,
`b2Contact` and `b2JointSim` are plain data in C (fixed arrays, no hull or mesh pointers), so they
are structs here where box3d-as has classes. A class assignment aliases and a struct assignment
copies, so do not copy box3d-as code that relies on aliasing these.

## Helpers later waves use

- `core.as`: `B2_ASSERT(cond)` plus `B2_ASSERT_ENABLED`; write `if (B2_ASSERT_ENABLED)
  B2_ASSERT(x);` in hot code, since rasc evaluates the argument of a call that folds away.
  `B2_CHECK_INPUT` has no function form: write `if (!(cond)) { b2Log(msg); B2_ASSERT(false);
  return; }` as `b2SetLengthUnitsPerMeter` does; note C returns early even in release.
  `B2_CHECK_DEF(def)` is `B2_ASSERT(def.internalValue == B2_SECRET_COOKIE)`.
  `b2FloatToIntSat`, `b2FloatToUIntSat` are C's float to int casts on arm64 (verified).
  `FLT_MAX`, `FLT_MIN`, `FLT_EPSILON`, `INFINITY`, `INT32_*`, `UINT*_MAX`.
  `b2_lengthUnitsPerMeter` is an exported mutable global, read live.
  `b2Log(message)` takes a finished string.
- `constants.as`: `B2_HUGE()`, `B2_LINEAR_SLOP()`, `B2_SPECULATIVE_DISTANCE()`,
  `B2_CONTACT_RECYCLE_DISTANCE()`, `B2_MAX_AABB_MARGIN()` are inline functions.
- `math_functions.as`: `b2GetLengthAndNormalize(length: ref f32, a)` (bind a struct leaf or a
  `FixedArray` element, not a plain local), `b2MakeAABB(points: FixedArray<b2Vec2, 8>, count,
  radius)` (every C caller passes proxy points or a local array of up to 8).
- `id.as`: `B2_IS_NULL(id)` is `id.index1 == 0`, `B2_IS_NON_NULL` is `!= 0`, `B2_ID_EQUALS(a, b)`
  is `a == b` (struct `==` compares every field; ids have nothing else). `b2StoreContactId(id,
  ref FixedArray<u32, 3>)`, `b2LoadContactId(FixedArray<u32, 3>)`.
- `container.as`: `b2Array_AddIndex` / `b2Array_Emplace` (push a zeroed element, return its
  index), `b2Array_Resize`, `b2Array_ResizeAndSetZero`, `b2Array_Append(a, src, n)`,
  `b2Array_RemoveSwap`, `b2SwapElements(a, i, j)`; push, pop, `length`, `length = 0` for the
  rest. `b2Array_Reserve` and `b2Array_ByteCount` have no equivalent.
- `table.as`: `B2_SHAPE_PAIR_KEY(k1, k2)` is a function; `b2KeyHash` returns u64 as in C.
- `simd.as`: the tree helpers take the node array plus an index where C takes a node pointer
  (`b2UnionPairV(nodes, pair)`, `b2OverlapPairMask(nodesA, pairA, nodesB, pairB)`,
  `b2OverlapNodePairMask(av, nodes, pair)`), `b2StoreAABBV(ref aabb, value, condition)`.
  `b2FloatW` and its ops are the scalar reference; kernels use the `L` helpers generic over `T`
  (`f32` per lane, or `v128` under `ASC_FEATURE_SIMD`): `b2SplatL`, `b2ZeroL`, `b2LoadL`,
  `b2StoreL`, `b2SetL`, `b2SetLoadL`, `b2LaneL`, `b2AddL`, `b2SubL`, `b2MulL`, `b2DivL`, `b2NegL`,
  `b2MulAddL`, `b2MulSubL`, `b2MinL`, `b2MaxL`, `b2SymClampL` (Box2D's clamp, not Box3D's),
  `b2GreaterThanL`, `b2LessThanL`, `b2AndL`, `b2BlendL`, `b2MaskL`, `b2MaskValueL`,
  `b2AnyTrueL`, `b2DotL`, `b2CrossL`, and v128 only `b2UnpackLoL`, `b2UnpackHiL`,
  `b2TransposeL`, plus `b2AllZeroAt`, `b2StoreRow`. The contact solver wave imports these rather
  than redefining the `.inl` helpers; it owns `b2GatherBodies`, `b2ScatterBodies` and the wide
  constraint struct.
- `types.as` holds the joint defaults; `lib/joint.as` re-exports them.

## Deviations

- `b2PreSolveCallbackFcn` takes a `b2PreSolveManifold` holder: C passes a `b2Manifold*` the callback
  may edit, and rasc closures cannot take `ref`. The caller copies the manifold in and back out.
- Callbacks drop `void* context`; the closure carries it. Const pointer inputs become values
  (`b2PlaneResultCallbackFcn(shapeId, plane)`, the tree callbacks take the input struct).
- `b2BodyDef.name` is a string, empty for none. `b2ChainDef.points` and `materials` are
  `StaticArray`s; the default materials array is one shared module array, like C's function
  static.
- The b2TreeNode union is two fields that do not alias: where C writes `height` and reads
  `shapeIndex` (or the reverse), write both.
- `b2ValidateFreeId` and `b2ValidateUsedId` are empty, as C without `BOX2D_VALIDATE`.
  `b2IdPool.freeCapacity` tracks C's array capacity so `b2GetIdBytes` matches.
- `b2CountSetBits` lives in bitset.as (C: table.c), as in box3d-as.
- The allocator hooks are accepted and ignored, `b2GetByteCount` reads 0, `b2Alloc`/`b2Free`/
  `b2GrowAlloc`, mutexes, semaphores, threads, atomics functions, `b2Pause`, `b2Prefetch` and
  `b2IsDenormalFlushEnabled` are not ported. The `b2Atomic*` structs exist.
- `b2GetTicks` is `clock()` from `rive/os` in nanoseconds; the default assert handler returns 1 and
  traps; the default log handler is silent.
- Skipped: `b2HexColor`, `b2DebugDraw`, `b2DefaultDebugDraw`, `b2GetGraphColor` (debug draw),
  `B2_SNOOP_*` counters, double precision paths, C++ operators. The replay structs and enum are
  declared; the replay functions are not ported.
- `b2InvMagRot`, named in the task, does not exist in v3.2.0.

## Harness

- `verify/toolchain.sh` picks `RASC` and `AS_STD` from the environment, else the first rasc build
  in `~/Projects/rive-pinefall`, then `~/Projects/rive-master`. `run.sh` prints which.
- `verify/run.sh [module...]` as in CONVENTIONS. C drivers include `~/Projects/box2d/src`
  internals freely; `verify/cref/simd.c` shows how to test a static helper of a `.inl` (include it
  with `B2_SIMD_WIDTH` defined; the archive's `contact_solver_w4.o` is then never pulled).
- C drivers that call anything using `B2_CHECK_INPUT` must silence the log callback
  (`b2SetLogCallback`), since C's default log prints to stdout and corrupts the vectors.
- Stateful AS drivers keep state in module globals and let the host call `frameBoundary()`
  between exports (table.mjs shows it).
- Scenes: add the scene to `verify/scenes/scenes.c` and `scenes.as` (same statements, explicit
  `<f32>` on every int to float), to `sceneNames` in `verify/traj.mjs` and to the default list and
  step count in `verify/traj.sh`; then `NATIVE=1 verify/traj.sh <scene>`. Body records are 11
  words (index, flags, id, position, rotation, velocities); counters from `envCounterStart` on
  (task count, stack, bytes, workers) are not compared. `node verify/scenes/dump.mjs print|diff`.

## rasc findings

Toolchain: rive-pinefall rasc (fresh master); rive-master rasc behaves the same on everything here.

### FixedArray length must be a literal

```as
export const N: i32 = 8;
export struct P { v: FixedArray<f32, N>; } // error: FixedArray length must be an integer literal
```

The port spells `B2_MAX_POLYGON_VERTICES` as 8 in every FixedArray type.

### A FixedArray cannot be a generic argument

```as
function sumX<A>(pts: A, n: i32): f32 { let s: f32 = 0; for (let i = 0; i < n; ++i) s += pts[i].x; return s; }
struct P { v: FixedArray<V, 8>; }
let p: P; sumX(p.v, 2); // error: value classes cannot be generic arguments yet
```

`StaticArray<V>` and `Array<V>` work. A plain `FixedArray<V, 8>` parameter (by value or `ref`)
works.

### Raising an Array's length within capacity keeps stale elements

```as
let a = new Array<i32>(); a.push(5); a.push(6);
a.length = 0; a.length = 2; // a[0] == 5, a[1] == 6
```

box3d-as notes say growing through length zero fills; it does only past the capacity. Zero with
`let z: valueof<A>;` and assign or push it (`valueof<A>` works on struct arrays).

### Function types and closures cannot take ref parameters

```as
type Pre = (a: i32, m: ref M) => bool; // error: ref is only allowed on function parameters and ref bindings
let f = (a: i32, m: ref M): bool => true; // error: closures cannot take ref parameters
```

### Smaller notes

- `T | null` is refused; write `T?`.
- A literal on the left of an f32 operation types as f32 (`0.5 * x`, `1.0 - 0.0006 < q`), checked
  in the wat.
- Module constants are still `global.get` under `-O` on this toolchain when initialized through
  `reinterpret` (`FLT_EPSILON`).
