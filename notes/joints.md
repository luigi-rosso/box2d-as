# joints: joint.c and every *_joint.c (wave 2)

## Ported

| C | AnimaScript |
|---|---|
| `src/joint.h`, `src/joint.c` | `lib/joint.as` (declarations from bdb3848, functions below them) |
| `src/distance_joint.c` | `lib/distance_joint.as` |
| `src/motor_joint.c` | `lib/motor_joint.as` |
| `src/mover_joint.c` | `lib/mover_joint.as` |
| `src/pogo_joint.c` | `lib/pogo_joint.as` |
| `src/prismatic_joint.c` | `lib/prismatic_joint.as` |
| `src/revolute_joint.c` | `lib/revolute_joint.as` |
| `src/weld_joint.c` | `lib/weld_joint.as` |
| `src/wheel_joint.c` | `lib/wheel_joint.as` |

Every function in C order: the nine create functions (filter included), `b2DestroyJointInternal`,
`b2DestroyJoint`, every `b2Joint_*`, every per type getter and setter (all 161 joint functions of
`box2d.h` exist), `b2GetJointReaction`, the per type force and torque getters, prepare / warm start /
solve for every type, the `_Overflow` loops and the three tasks. The joint defaults live in
`lib/types.as` and `lib/joint.as` re-exports them. Joint events are built in solver.c from the bits
`b2SolveJointsTask` sets in each worker's `jointStateBitSet`.

Skipped: debug draw (`b2DrawJoint`, `b2Draw*Joint`), every `B2_REC` / `B2_REC_CREATE` line (setters
whose only world use was the recording no longer look the world up), `b2ValidateSolverSets` calls,
Tracy zones, the `B2_WELD_BLOCK_SOLVE` branch (0 in C) and `#if 0` `b2WheelJoint_Dump`.

## Sim access

`b2JointSim` is a struct in growable `Array<b2JointSim>` (graph colors and solver sets), and rasc
refuses a `ref` into an `Array` element. The port binds C's `b2JointSim*` in place through a view:

```as
@inline export function b2JointSimView(sims: Array<b2JointSim>): StaticArray<b2JointSim> {
	return changetype<StaticArray<b2JointSim>>(b2ArrayData(sims));
}
let joints = b2JointSimView(color.jointSims);
b2SolveJoint(joints[i], context, useBias); // ref parameter, works in place
```

An `Array`'s data pointer is its backing `ArrayBuffer` object, which has the same layout as a
`StaticArray`, so the view's length is the buffer capacity and indexing stays bounds checked against
real memory. The view is valid until the array grows (no solver stage grows one; API functions bind
it after any placement). Nothing is copied: every prepare, warm start and solve function takes
`base: ref b2JointSim` exactly as C takes the pointer, with `ref joint = base.revoluteJoint` for
`&base->revoluteJoint`.

Body states: `b2StateAddress(context.states, joint.indexA)` (identity state for a static end), one
`load<b2BodyState>` copy, then velocities written back through the address with
`b2WriteStateVelocity(address, v, w)` / `b2WriteStateLinearVelocity` (joint.as), only under C's
`flags & b2_dynamicFlag` test. Box2D (unlike Box3D) never writes a state without that test, so the
shared identity state is never written. Prepare copies the two body sims once (read only).

API functions: `b2GetJointSimViewCheckType(jointId, type)` plus `b2GetJointLocalIndex(jointId)`
replace C's `b2GetJointSimCheckType` pointer:

```as
let sims = b2GetJointSimViewCheckType(jointId, b2_wheelJoint);
ref joint = sims[b2GetJointLocalIndex(jointId)];
```

## Signatures others match

- `b2GetJointFullId(world, b2JointId): i32`, `b2GetJointSimArray(world, jointId): Array<b2JointSim>`,
  `b2GetJointSim(world, jointId): b2JointSim` (copy), `b2GetJointSimView(world, jointId)`,
  `b2GetJointSimCheckType(jointId, type): i32` (joint index, `B2_NULL_INDEX` where C returns NULL).
- `b2DestroyJointInternal(world, jointId)` (C always wakes both bodies; no flag).
- `b2PrepareJoint(joint: ref b2JointSim, context)`, `b2WarmStartJoint(joint: ref, context)`,
  `b2SolveJoint(joint: ref, context, useBias)`, `b2PrepareJoints_Overflow(context)`,
  `b2WarmStartJoints_Overflow(context)`, `b2SolveJoints_Overflow(context, useBias)`,
  `b2PrepareJointsTask(block: b2SolverBlock, context)`, `b2WarmStartJointsTask(block, context)`,
  `b2SolveJointsTask(block, context, useBias, workerIndex)`.
- `b2GetJointReaction(sim: ref b2JointSim, invTimeStep, force: ref f32, torque: ref f32)`; pass struct
  leaves (`r.x`, `r.y` of a local `b2Vec2`), since rasc refuses a ref to a primitive local.
- Per type `b2Get<X>JointForce(world, base: ref b2JointSim): b2Vec2`, `b2Get<X>JointTorque(...): f32`.
- `float*` out pairs are `ref f32` (`b2Joint_GetConstraintTuning`, `b2DistanceJoint_GetSpringForceRange`).
- `void* userData` is `u64`. Defs pass by value.

## Deviations

- `b2CreateJoint` returns the joint index (C: a joint and sim pointer pair); the create functions bind
  the sim with `b2GetJointSimView` after placement, which also covers C's "sim orphaned by the set
  merge" re-fetch. The new `b2Joint` is filled as a local copy and stored once before the neighbour
  edges are relinked (they never alias it).
- `B2_CHECK_INPUT` / `B2_CHECK_INPUT_RETURN` are `if (!b2CheckJointInput(cond, "text", fn)) return`;
  the message is built only on failure.
- `b2Joint_SetCollideConnected` and `b2Joint_WakeBodies` take `b2GetWorldLocked`'s `b2World?`.

## Upstream behaviour kept

- `b2Joint_GetLinearSeparation`, `b2Joint_GetAngularSeparation` and `b2Joint_GetConstraintTorque`
  reach `B2_ASSERT(false)` for the mover and pogo joints (no case); the driver skips them there.
- `b2PrismaticJoint_SetLimits`, `b2RevoluteJoint_SetLimits`, `b2WheelJoint_SetLimits` assert
  `lower <= upper` before swapping.

## Verification

- `rasc check` on all nine files: no diagnostics in them (a planted error confirmed coverage).
  Compiled to wat under `--runtime frame -O`: the only `f64` ops in joint code are the inlined
  `b2IsValidFloat` infinity compare and `b2UnwindAngle`'s remainder, both from math_functions.
- `verify/run.sh joints` (`verify/cref/joints.c`, `verify/as/joints.as`, `verify/joints.mjs`): 652
  lines, 5536 values, bit exact with `-O`, no flags, `-O -D B2_ENABLE_ASSERT` and
  `-O --enable-simd`. The world is built through the public API (ground plus eight dynamic boxes, all
  nine joint types with springs, limits, motors and thresholds), seeded body velocities, then the
  driver runs the joint stages by hand with its own step context: `b2PrepareJointsTask` in two blocks
  over prepare spans, then four substeps of warm start, solve with bias, a position update and relax,
  per color through the tasks plus the overflow loops. It prints every awake state after each phase,
  each sim's reaction and softness, the joint event bits, and every getter of every joint; then runs
  every setter, a pass without warm starting, destroys two joints and turns a body static.

## rasc findings

No new refusals. The `StaticArray` view over an `Array`'s buffer is the way to bind a struct element
of a growable array by `ref` (minimal form in "Sim access"); it relies on `dataStart` equalling the
buffer, which holds while the array is never `shift`ed.
