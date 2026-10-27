# world: the simulation (wave 2)

## Internal API contract

Every wave 2 simulation file (body, shape, contact, island, solver_set, constraint_graph, sensor,
physics_world, solver, contact_solver, joint and the joint files) follows these rules, so the files
meet without negotiation. The types are declared in commit bdb3848 at the top of each owner file.

### Why

- World arrays are `Array<T>` of structs (CONVENTIONS). rasc refuses `ref` into an `Array<T>`
  element ("may reallocate"), so a C pointer into `world.bodies`, `world.shapes`,
  `world.contacts`, `world.joints` or a sim array cannot be a `ref`.
- rasc passes a struct by value as its leaves: `b2Shape` by value is 86 wasm params, `b2Polygon`
  36. A `ref` is one i32. So large structs never travel by value in hot code.

### Rules

1. A C pointer to a world array element becomes its index, with `world` first when C lacks it:
   `b2Body*` -> `bodyId: i32` (`world.bodies` index, C's `body->id`), `b2Shape*` -> `shapeId: i32`,
   `b2Contact*` -> `contactId: i32`, `b2Joint*` -> `jointId: i32`. A C function returning such a
   pointer returns the index (`B2_NULL_INDEX` for NULL).
2. Class objects pass as objects: `b2World`, `b2SolverSet`, `b2Island`, `b2ChainShape`,
   `b2Sensor`, `b2GraphColor`, `b2StepContext`, `b2BroadPhase`, `b2DynamicTree`. C's int ids stay
   ints (`b2DestroyIsland(world, islandId)`).
3. Sims (`b2BodySim*`, `b2BodyState*`, `b2ContactSim*`, `b2JointSim*`) that a function mutates are
   `ref` parameters; the caller binds a local copy and stores it back, or (hot loops) works through
   raw addresses (`b2ArrayData(array) + i * sizeof<T>()`, `load`/`store` at `offsetof`). Read only
   access copies (`b2GetBodySim(world, bodyId): b2BodySim`). Single field writes go through the
   place: `world.solverSets[setIndex].bodySims[localIndex].flags |= x`.
4. Locators for the arrays a sim lives in, where C returns a pointer:
   - `b2GetBodySim(world, bodyId): b2BodySim` (copy), `b2GetBodySimAddress(world, bodyId): usize`,
     `b2GetBodyState(world, bodyId): i32` (awake state index or `B2_NULL_INDEX`) - body.as.
   - `b2GetContactSim(world, contactId): b2ContactSim` (copy) and
     `b2GetContactSimArray(world, contactId): Array<b2ContactSim>` (the color's or the set's array;
     the element is `world.contacts[contactId].localIndex`) - contact.as.
   - `b2GetJointSim(world, jointId): b2JointSim` (copy) and
     `b2GetJointSimArray(world, jointId): Array<b2JointSim>` (element at
     `world.joints[jointId].localIndex`) - joint.as.
5. Other C out pointers become `ref` parameters or the return value. `const T*` inputs of small
   structs pass by value; large ones (`b2Polygon`, `b2Manifold`, `b2Shape`, sims, defs with fixed
   arrays) pass by `ref` when the caller has a ref-able place, else by value.
6. Callbacks plus `void* context` become closures (public API) or, for internal per step work,
   plain functions taking a typed context argument so nothing allocates per step
   (`b2ParallelFor<C>(world, fn, count, minRange, context: C)` in lib/parallel_for.as; `fn` is a
   top level function `(start, end, workerIndex, context: C) => void`).
7. Step memory (C's `b2StackAlloc` and arena) lives in a module scratch that only grows
   (`count + count / 4 + 16`), as box3d-as `b3SolverScratch`. A settled scene allocates nothing per
   step.

### Cross file signatures (C name -> AnimaScript)

body.as: `b2GetBodyFullId(world, b2BodyId): i32`, `b2GetBodyTransformQuick(world, bodyId)`,
`b2GetBodyTransform(world, bodyId)`, `b2MakeBodyId(world, bodyId)`,
`b2ShouldBodiesCollide(world, bodyIdA, bodyIdB)`, `b2RefreshBodyContactIndices(world, bodyId)`,
`b2WakeBody(world, bodyId): bool`, `b2UpdateBodyMassData(world, bodyId)`,
`b2SyncBodyFlags(world, bodyId)`, plus the header helpers already in body.as.

shape.as: `b2CreateShapeProxy(world, shapeId, type, transform, forcePairCreation)`,
`b2DestroyShapeProxy(world, shapeId)` (C: shape, bp), `b2FreeChainData(chain)`,
`b2ComputeShapeMass(world, shapeId)`, `b2ComputeShapeExtent(world, shapeId, localCenter)`,
`b2ComputeShapeAABB(world, shapeId, transform)`, `b2ComputeFatShapeAABB(world, shapeId, transform,
extra)`, `b2GetShapeCentroid(world, shapeId)`, `b2GetShapePerimeter(world, shapeId)`,
`b2GetShapeProjectedPerimeter(world, shapeId, line)`, `b2MakeShapeDistanceProxy(world, shapeId)`,
`b2RayCastShape(input, world, shapeId, transform)`, `b2ShapeCastShape(input, world, shapeId,
transform)`, `b2CollideMover(mover, world, shapeId, transform)`. The header helpers
(`b2GetShapeRadius`, `b2ShouldShapesCollide`, `b2ShouldQueryCollide`) take values as declared.

contact.as: `b2CanCollide(typeA, typeB)`, `b2CreateContact(world, shapeIdA, shapeIdB)`,
`b2DestroyContact(world, contactId)`, `b2UpdateContact(world, contactSim: ref b2ContactSim,
shapeIdA, transformA, centerOffsetA, shapeIdB, transformB, centerOffsetB): bool`, the locators
above.

island.as: `b2CreateIsland(world, setIndex): b2Island`, `b2DestroyIsland(world, islandId)`,
`b2LinkContact(world, contactId)`, `b2UnlinkContact(world, contactId)`,
`b2LinkJoint(world, jointId)`, `b2UnlinkJoint(world, jointId)`, `b2SplitIsland(world, baseId)`,
`b2SplitIslandTask(world)`.

solver_set.as: `b2DestroySolverSet(world, setIndex)`, `b2WakeSolverSet(world, setIndex)`,
`b2TrySleepIsland(world, islandId)`, `b2MergeSolverSets(world, setIndex1, setIndex2)`,
`b2RemoveBodySim(world, set, localIndex): i32` (moved body id or `B2_NULL_INDEX`),
`b2TransferBody(world, targetSet, sourceSet, bodyId)`,
`b2TransferJoint(world, targetSet, sourceSet, jointId)`.

constraint_graph.as: `b2CreateGraph(graph, capacity: b2Capacity)`, `b2DestroyGraph(graph)`,
`b2AddContactToGraph(world, contactSim: ref b2ContactSim, contactId)`,
`b2RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex)`,
`b2CreateJointInGraph(world, jointId): i32` (the new local index; the color is
`world.joints[jointId].colorIndex`), `b2AddJointToGraph(world, jointSim: ref b2JointSim, jointId)`,
`b2RemoveJointFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex)`.

sensor.as: `b2OverlapSensors(world)`, `b2DestroySensor(world, shapeId)`.

joint.as: `b2DestroyJointInternal(world, jointId)`, `b2GetJointFullId(world, b2JointId): i32`,
the locators above, `b2GetJointSimCheckType(jointId: b2JointId, type)` returns the joint index
(i32), `b2PrepareJoint(joint: ref b2JointSim, context)`, `b2WarmStartJoint(...)`,
`b2SolveJoint(joint: ref b2JointSim, context, useBias)`, the `_Overflow` and `Task` functions with
C's other parameters, `b2GetJointReaction(sim: ref b2JointSim, invTimeStep, force: ref f32,
torque: ref f32)`, per type `b2Prepare<X>Joint(base: ref b2JointSim, context)` etc.

solver.as: `b2Solve(world, stepContext)`, `b2MakeSoft`, the stage types.

contact_solver.as: C's signatures (`(context)` and `(block, context[, workerIndex])`).

physics_world.as: `b2GetWorldFromId(id): b2World`, `b2GetWorld(index): b2World`,
`b2GetWorldLocked(index): b2World?`, `b2ComputeWorldBounds(world, bounds: ref b2AABB): bool`, and
the public `b2World_*` functions.

parallel_for.as: `b2ParallelFor<C>(world, callback, itemCount, minRange, context: C)`.

## What is ported (parent agent's files)

| C | AnimaScript |
|---|---|
| `src/physics_world.h`, `src/physics_world.c` (minus debug draw, recording, memory stats, validation) | `lib/physics_world.as` |
| `src/contact.h`, `src/contact.c` | `lib/contact.as` |
| `src/island.h`, `src/island.c` | `lib/island.as` |
| `src/solver_set.h`, `src/solver_set.c` | `lib/solver_set.as` |
| `src/constraint_graph.h`, `src/constraint_graph.c` (minus `b2GetGraphColor`) | `lib/constraint_graph.as` |
| `src/sensor.h`, `src/sensor.c` | `lib/sensor.as` |
| `src/parallel_for.h`, `src/parallel_for.c` | `lib/parallel_for.as` |
| `include/box2d/*.h` public names | `lib/box2d.as` (generated, see below) |

## Deviations

- Tasks run inline on worker 0. C's serial fallback (`workerCount` 1, `b2DefaultAddTaskFcn`) does the same;
  the built in scheduler and external task callbacks are not ported, so `workerCount` is always 1 and
  `b2World_SetWorkerCount` only resizes the per worker storage. `b2ParallelFor` runs every block on worker 0
  in block order, which is what C's first task does when the task callback runs it synchronously.
- `b2World_Step` reuses one `b2StepContext` per world (C's is a zeroed stack local); `b2ResetStepContext`
  zeroes the scalar fields. The collide spans are a module array of `B2_GRAPH_COLOR_COUNT + 2` span objects.
- The event getters return one object per world, refreshed by each call, instead of a struct by value. The
  arrays they hold are the world's live event arrays; the counts say how many are valid.
- Query contexts (C stack structs) are classes with a `visit` method the tree calls directly. The types whose
  callback reaches user code keep one context per nesting depth, so a callback may query again; the mover
  cast and the explosion keep one context each. Nothing allocates per query once a depth was reached.
- `b2World_CastRayClosest` runs `b2RayCastClosestFcn` as a method of the ray context.
- The narrow phase copies each contact sim out of its color array, updates it through `ref`, and stores it
  back (rasc refuses a `ref` into an `Array<T>` element). Body sims are read through addresses
  (`b2ResolveContactBodySim` returns the sim's address), never copied.
- Destroying a world replaces its slot with a fresh `b2World` holding only the bumped generation, as C's
  `*world = (b2World){ 0 }` plus generation; the old arrays go with the old object.
- Islands, solver sets and sensors keep their emptied arrays when destroyed (C frees them) and reuse them
  when the slot is reused; `b2SplitIsland` detaches the base island's arrays by giving it fresh ones.
- `b2SplitIsland`'s stack arrays are module arrays that only grow.
- `b2CountSetBits`, `b2World_DumpMemoryStats`, `b2World_Draw`, recording (`b2World_StartRecording`,
  `b2World_StopRecording`, every `B2_REC*`), validation (`b2Validate*`) are not ported.
- `b2Counters.stackUsed` is 0: there is no stack allocator. It is a build dependent counter the trajectory
  gate does not compare.
- Contact registry: two bool tables (has a function, is primary) and a switch over the shape types replace
  C's table of function pointers.
- `B2_CHECK_INPUT` failures log and return, as C does in release.

## lib/box2d.as

Generated by a script from the headers (`include/box2d/{base,collision,id,math_functions,math_types,types,box2d,constants}.h`):
every function, struct, enum, enumerator, callback type, macro constant and `static const` of the public headers that a module
exports is re-exported under its C name from the module that defines it, plus `b2PreSolveManifold` (the port's pre-solve
holder). 724 names. `export * from "lib/box2d"` compiles under `-O`, `-O --enable-simd`, debug and `-D B2_ENABLE_ASSERT`, which
also type checks every public function (rasc skips functions nothing calls; Obsidian finding 7).

Not exported, by design:

- Debug draw: `b2World_Draw`, `b2DebugDraw`, `b2DefaultDebugDraw`, `b2HexColor` and its 149 `b2_color*` enumerators,
  `b2GetGraphColor`.
- Recording, replay and snapshots (`b2CreateRecording*`, `b2Recording_*`, `b2Replay_*`, `b2ValidateRecording`,
  `b2World_StartRecording`, `b2World_StopRecording`, `b2World_GetStateHash`, `b2World_GetSnapshot`,
  `b2World_RestoreFromSnapshot`, `b2CreateWorldFromSnapshot`, the `b2Recording` and `b2Replay` handles):
  the replay structs and enum are declared in types.as, the functions are not ported.
- `b2World_DumpMemoryStats` (prints C allocator stats).
- Macros with no AnimaScript form: `B2_ALIGN_AS`, `B2_FORCE_INLINE`, `B2_VALIDATE`, `B2_IS_NULL`, `B2_IS_NON_NULL`,
  `B2_ID_EQUALS`, `B2_NULL_ID` (write `id.index1 == 0`, `a == b`, `b2_null*Id`).

## Gates (wave 2 end)

- `verify/run.sh`: 16 modules (bitset, bodyshape, contact_solver, container, ctz, distance, geometry, hull, id_pool, joints,
  manifold, math, mover, simd, table, tree), 0 mismatches with `RASC_FLAGS="-O"`, `"-O --enable-simd"` and `""`.
  `bodyshape` is the body/shape sub-agent's create/mutate/query/destroy driver without stepping (379 lines, 8747 values).
- `verify/traj.sh`: 12 scenes, every step bit exact (bodies, counters, probes) in all three builds.
- `verify/heap.mjs <scene>`: wasm pages after step 20 and after step 200 under `--runtime frame`, frame boundary after each
  step. Equal for pyramid, shape_rain, chain_car, joints, churn, sensors, events (993) and many_bodies (1490), scalar and SIMD
  artifacts.
- `-D B2_ENABLE_ASSERT`: dynamic_mover traps in `b2GetJointConstraintTorque`, which asserts for mover and pogo joints in C
  too (C's recording is built with `NDEBUG`). The scene's probe calls it; the other 11 scenes pass with asserts on.

## First rough timings

many_bodies (5501 bodies), node (V8) on an M5 Max at load 80 to 125, so only the order of magnitude means anything:
AS scalar artifact about 21 ms per step, SIMD artifact about 14 ms, native scalar about 3.3 ms and NEON about 2.1 ms (whole
recorder process time over 100 steps, creation included). About 6x native. Not profiled yet; the narrow phase copies each
contact sim in and out (notes above) and the collision layer takes polygons by value as 36 wasm parameters, both candidates.
