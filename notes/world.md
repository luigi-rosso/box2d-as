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
