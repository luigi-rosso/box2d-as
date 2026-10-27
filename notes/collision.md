# collision: wave 2

The collision layer: `geometry.c`, `hull.c`, `distance.c`, `manifold.c`, `mover.c`, ported from
v3.2.0's C (not from box3d-as, whose GJK and manifolds differ in algorithm; the legacy v3.1 port
supplied only the test grids).

## What is ported

| C | AnimaScript |
|---|---|
| `src/hull.c` | `lib/hull.as` |
| `src/geometry.c` | `lib/geometry.as` |
| `src/distance.c` | `lib/distance.as` |
| `src/manifold.c` | `lib/manifold.as` |
| `src/mover.c` | `lib/mover.as` |

Every function of those files, in C's order, except `b2ShapeCastMerged` and
`b2ComputeSimplexClosestPoint` (inside `#if 0` in C) and `b2ComputeFatShapeAABB` (below).

## Gates

`verify/run.sh hull geometry distance manifold mover`, bit exact under `RASC_FLAGS="-O"`,
`"-O --enable-simd"` and `""`:

| module | lines | values | what |
|---|---|---|---|
| hull | 50003 | 980055 | fixed sets; 30000 seeded clouds (scattered, tiny for welding, near collinear, near duplicates, on a circle, counts 0 to 10); 20000 raw point sets through `b2ValidateHull` |
| geometry | 160986 | 2287329 | every polygon maker, transform, bad hull fallback; circle, capsule, polygon mass (rounded, one and two point polygons); bounds and fat AABBs near and 20 km out; point tests; ray casts (zero length, parallel, inside start, one sided, rounded polygon via shape cast); `b2IsValidRay` on special floats; the four shape casts with offset proxies and encroachment; the four mover collides |
| distance | 113128 | 2175442 | sweep transforms; segment distance with every degeneracy, parallel, touching; proxies past 8 points; GJK over a 6 x 6 proxy grid x ring of 8 angles x 7 depths x 4 rotations, cold and warm cache, cache before and after and the recorded simplex compared; 30000 random GJK queries with empty, carried and random caches; 20000 shape casts; 20000 TOI over seeded sweeps (overlapped, hit, separated) |
| manifold | 117440 | 1951808 | all twelve collide functions over the ring grid with A at three poses, capsule variants (parallel, perpendicular, short), 6 x 6 polygon library, box on box near alignment, chain segments (straight, convex, concave, mixed) with cold and warm caches; 6000 seeded random rounds of every function; 20000 chain segment vs polygon around the segment ends. Every manifold field: count, normal, both points, separations, feature ids, and the simplex cache in and out |
| mover | 40000 | 260665 | `b2SolvePlanes` (pushes, iteration count, rigid and soft limits) and `b2ClipVector` |

`verify/cref/collide.h` and `verify/as/collide.as` hold the shared printers and seeded shape
generators. Inputs are built with the ported `b2ComputeCosSin`, `b2MakeRot`, `b2ComputeHull` and
`b2MakePolygon` on both sides, and only dyadic constants are written as literals.

With `-D B2_ENABLE_ASSERT` the geometry driver traps at its `bad` line: it feeds `b2MakePolygon` an
empty hull to test the release fallback, and the hull assert fires as it does in C.

## Signatures that differ from C

- Struct inputs that C passes as `const T*` are values: `b2CollidePolygons(polygonA: b2Polygon,
  polygonB: b2Polygon, xf)`, `b2ComputeShapeDistance(input: b2DistanceInput, ...)`,
  `b2CastShape(input: b2ShapeCastPairInput)`, `b2ComputeTimeOfImpact(input: b2TOIInput)`,
  `b2GetSweepTransform(sweep: b2Sweep, time)`, all `b2Cast*`, `b2Compute*Mass`,
  `b2Compute*Bounds`, `b2PointIn*`. World callers read shapes out of an `Array<b2Shape>`, which
  cannot be a `ref`.
- Out and in/out pointers are `ref`: `b2ComputeShapeDistance(input, cache: ref b2SimplexCache,
  simplexes: StaticArray<b2Simplex>?, simplexCapacity)`,
  `b2CollideChainSegmentAndCapsule(segmentA, capsuleB, xf, cache: ref b2SimplexCache)`,
  `b2CollideChainSegmentAndPolygon(segmentA, polygonB, xf, cache: ref b2SimplexCache)`.
  As in C's release build, `simplexes` receives only the initial simplex.
- `b2MakeProxy(points: FixedArray<b2Vec2, 8>, count, radius)` and `b2MakeOffsetProxy(points:
  FixedArray<b2Vec2, 8>, count, radius, position, rotation)`: every C caller passes polygon or
  hull points, or the address of one or two points, so two helpers that C does not have replace
  the latter: `b2MakePointProxy(point, radius)` (C: `b2MakeProxy(&circle.center, 1, r)`) and
  `b2MakeSegmentProxy(point1, point2, radius)` (C: `b2MakeProxy(&capsule.center1, 2, r)`).
- `b2ComputeHull<A>(points: A, count)`, `b2SolvePlanes<A>(targetDelta, planes: A, count)` and
  `b2ClipVector<A>(vector, planes: A, count)` take any `StaticArray` or `Array` where C takes a
  pointer (a `FixedArray` cannot be a generic argument).
- `b2ComputeFatShapeAABB(const b2Shape*, ...)` takes a `b2Shape`, declared in `lib/shape.as`, which
  imports geometry, so it lives in `lib/shape.as`. Its four static helpers are exported from
  geometry for it: `b2ComputeCircleFatAABB`, `b2ComputeCapsuleFatAABB`,
  `b2ComputePolygonFatAABB`, `b2ComputeSegmentFatAABB(shape, xf, extra)`. The geometry check runs
  C's `b2ComputeFatShapeAABB` against them.
- `b2CollideMoverAndCircle`, `...Capsule`, `...Polygon`, `...Segment(mover: b2Capsule, shape)` are
  exported from geometry as in C (declared in C's shape.h).

## Internal deviations

- `b2SeparationFunction` does not hold the two proxy pointers; `b2FindMinSeparation` and
  `b2EvaluateSeparation` take the proxies by `ref` beside the function. `b2FindMinSeparation`
  returns a `{ separation, indexA, indexB }` struct where C writes two indices through pointers;
  `b2FindMaxSeparation` likewise returns `{ separation, edgeIndex }`.
- GJK addresses the simplex vertices by name where C walks `{ &v1, &v2, &v3 }`, and keeps the
  saved indices in six locals; `b2ComputeWitnessPoints` writes `output.pointA/pointB` directly.
  The simplex helpers are `@inline`, so `b2ComputeShapeDistance` calls only
  `b2MakeSimplexFromCache` (box3d-as perf_3 "GJK without calls").
- `b2ClipPolygons` reads the reference and incident edge values out of the two polygons up front
  where C points `poly1` and `poly2` at them.
- `b2GetLengthAndNormalize(&length, v)` binds `length` to a field of a local `b2Vec2`, since rasc
  refuses `ref` to a primitive local.
- None of the five files allocates: the wat of the hot functions has no allocator call. rasc passes
  a struct by value as its leaves (`b2CollidePolygons` takes 76 wasm params).

## rasc findings

### Whole element store into an imported StaticArray global

```as
// ra.as
export struct V { x: f32 = 0; }
export const arr: StaticArray<V> = new StaticArray<V>(4);
// rb.as
import { V, arr } from "./ra";
export function set(i: i32): void { arr[i] = V(); } // error: indexed stores of value classes only work on StaticArray
```

The same store compiles inside `ra.as`, a field store `arr[i].x = 1.0` compiles in `rb.as`, and so
does `let a = arr; a[i] = V();`. The geometry driver keeps its own scratch array.
