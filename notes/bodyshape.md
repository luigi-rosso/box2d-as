# bodyshape: body.c and shape.c

## What is ported

| C | AnimaScript |
|---|---|
| `src/body.c` | `lib/body.as`, below the `body.h` declarations |
| `src/shape.c` | `lib/shape.as`, below the `shape.h` declarations |
| `b2ComputeFatShapeAABB` (C: `geometry.c`) | `lib/shape.as`, it reads `b2Shape` |

Every function of both files in C order: body create and destroy, every `b2Body_*` function of
`box2d.h` (transforms, velocities, target transform, forces and impulses, type changes, names, user
data, mass data, damping, gravity scale, sleep, enable and disable, motion locks, bullets, contact
recycling, contact and hit events, shape and joint lists, extents, contact data, bounds), every
`b2Shape_*` and `b2Chain_*` function (create for circle, capsule, polygon, segment and chain
segment; chains with ghost vertices and per segment materials; destroy; materials, filters, event
flags, geometry getters and setters, parent chain, contact and sensor data, bounds, mass, closest
point, ray cast, test point, wind), and the internal functions of the contract.

v3.2.0 has no body queries (`b2Body_CastRay` and friends are Box3D only); the shape queries are
`b2Shape_TestPoint`, `b2Shape_CastRay`, `b2Shape_GetClosestPoint`.

The declarations at the top of both files are unchanged; only their imports grew.

## Signatures

Internal, as notes/world.md: `bodyId: i32` / `shapeId: i32` for C's `b2Body*` / `b2Shape*`.

- body.as: `b2GetBodyFullId(world, b2BodyId): i32`, `b2GetBodyTransformQuick(world, bodyId)`,
  `b2GetBodyTransform(world, bodyId)`, `b2MakeBodyId(world, bodyId)`,
  `b2ShouldBodiesCollide(world, bodyIdA, bodyIdB)`, `b2RefreshBodyContactIndices(world, bodyId)`,
  `b2WakeBody(world, bodyId): bool`, `b2UpdateBodyMassData(world, bodyId)`,
  `b2SyncBodyFlags(world, bodyId)`.
- shape.as: `b2CreateShapeProxy(world, shapeId, type, transform, forcePairCreation)`,
  `b2DestroyShapeProxy(world, shapeId)` (C passes the broad-phase, always the world's),
  `b2FreeChainData(chain)`, `b2ComputeShapeMass(world, shapeId)`,
  `b2ComputeShapeExtent(world, shapeId, localCenter)`, `b2ComputeShapeAABB(world, shapeId, xf)`,
  `b2ComputeFatShapeAABB(world, shapeId, xf, extra)`, `b2GetShapeCentroid(world, shapeId)`,
  `b2GetShapePerimeter(world, shapeId)`, `b2GetShapeProjectedPerimeter(world, shapeId, line)`,
  `b2MakeShapeDistanceProxy(world, shapeId)`, `b2RayCastShape(input, world, shapeId, xf)`,
  `b2ShapeCastShape(input: b2ShapeCastInput, world, shapeId, xf)` (input by value),
  `b2CollideMover(mover: b2Capsule, world, shapeId, xf)`.
- Public: const struct pointers pass by value (`b2CreatePolygonShape(bodyId, def, polygon)`,
  `b2Shape_SetPolygon(shapeId, polygon)`, `b2Chain_SetSurfaceMaterial(chainId, material, index)`).
  Output arrays are `StaticArray`: `b2Body_GetContactData` / `b2Shape_GetContactData`
  (`StaticArray<b2ContactData>`), `b2Body_GetShapes`, `b2Chain_GetSegments`,
  `b2Shape_GetSensorData` (`StaticArray<b2ShapeId>`), `b2Body_GetJoints`
  (`StaticArray<b2JointId>`).
- `b2Body_SetName(bodyId, name: string)`: UTF-8 bytes into `b2Body.name` with strncpy's rules (at
  most `B2_NAME_LENGTH` bytes, zero padded, stops at a zero). `b2Body_GetName` decodes up to the
  first zero.

## Deviations

- Static helpers: `b2GetShape` returns the shape index; `b2GetChainShape` returns the chain object.
  `b2CreateShapeInternal` returns the shape index and takes a private `b2ShapeGeometry` struct by
  `ref` in place of `const void* geometry` (structs cannot be generic arguments).
  `b2UpdateShapeAABBs(world, shapeId, transform, proxyType)` writes `world.fatAABBs[shapeId]`, which
  is the pointer every C caller passes. `b2ComputeShapeMargin(world, shapeId)`.
- Two private helpers name loops C repeats: `b2InvalidateContactTransforms` (the "center of mass
  changed" loop of `b2UpdateBodyMassData` and `b2Body_SetMassData`) and `b2DestroyShapeContacts`
  (the contact loop of `b2DestroyShapeInternal` and `b2ResetProxy`). `b2SetBodyName` is the name
  copy of `b2CreateBody` and `b2Body_SetName`.
- `b2UpdateBodyMassData` accumulates mass and inertia in locals and stores them where C's running
  sums end; nothing reads them in between. Its `masses` stack allocation is a module scratch that
  only grows.
- `b2DestroyChain` walks the singly linked chain list with a previous index in place of C's
  `int*` into the list.
- `b2Shape_GetBounds`, `b2Shape_ComputeMassData`, `b2Shape_GetClosestPoint`, `b2Shape_ApplyWind`
  drop C's `world == NULL` checks: `b2GetWorld` never returns null.
- `B2_CHECK_INPUT` / `B2_CHECK_INPUT_RETURN` log C's exact message and return early, also in
  release. `B2_CHECK_DEF` and plain asserts sit behind `B2_ASSERT_ENABLED`.
- Sensors: `b2Sensor` is a class, created with empty arrays (C's capacity hints dropped); C's
  `b2Array_Destroy` of its arrays is the remove swap dropping the object.

## Hazards (C behaviour kept or made safe)

1. `b2Shape_ApplyWind` takes the body sim pointer before waking a sleeping body. The wake moves the
   sims to the awake set and frees the sleeping set, so C reads the transform from freed memory
   (same values) and writes the force there: the force is lost. The port reads a sim copy before
   the wake and stores the force only when the body was already awake.
2. `b2Shape_ApplyWind` on a disabled dynamic body dereferences a null state in C; the port traps on
   the out of range state index.
3. `b2UpdateBodyMassData` clears `b2_dirtyMass` on the body only, not on the sim (as C).
4. A sim copy is always stored back before any call that can touch the same element (wakes,
   transfers, `b2SyncBodyFlags`, `b2UpdateBodyMassData`).

## Skipped

- `B2_REC` / `B2_REC_CREATE` (recording), `b2ValidateSolverSets`, `b2ValidateIsland`,
  `b2ValidateConnectivity`, `B2_VALIDATE`, the 32 byte alignment assert on body states.

## Verification

A scratch parity driver (C `verify/cref/bodyshape.c`, AS `verify/as/bodyshape.as`, not committed:
these files are outside this task) builds a world through the public API with a static ground
(box and segment), a chain loop with five materials and an open chain with ghosts, a dynamic body
with circle, capsule, rounded offset box and a sensor, a kinematic body, a sleeping body and a
disabled body with deferred mass. It prints body internals (set, local index, flags, sim flags,
mass, inertia, centers, extents, forces, velocities, bounds) and shape internals (AABB, fat AABB,
margin, proxy key, centroid, sensor index, list links, material, filter) after creation and after
each of: set transform, impulses, forces, wind on three shape types, point velocities, mass data,
motion locks, type changes static, kinematic, dynamic, disable and enable, target transform on a
sleeping body, shape density, geometry setters, filter, chain material, destroy shape and chain,
destroy bodies, id reuse, sleep and wake; plus per shape test point, two ray casts, closest point,
mass and bounds. 379 lines, 8747 values, 0 mismatches with `-O`, without `-O`,
`-O -D B2_ENABLE_ASSERT` and `-O --enable-simd` (no step: `b2Solve` had not landed).

## rasc findings

1. Structs cannot be generic arguments, so `b2CreateShapeInternal<G>(geometry: G)` with
   `instanceof` dispatch is refused:
   ```as
   struct A { x: f32 = 0; }
   function put<G>(g: G): void {}
   export function f(): void { put<A>(A()); } // error: value classes cannot be generic arguments yet
   ```
   Workaround: the `b2ShapeGeometry` holder passed by `ref`.
