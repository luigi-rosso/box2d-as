# Trajectory parity scenes

The scenes the trajectory gate (`verify/traj.sh`) runs. Native side in C over the public Box2D
v3.2.0 API, AnimaScript mirror statement for statement.

| path | what |
|---|---|
| `verify/scenes/scenes.h`, `scenes.c` | scene builders, scripted per step input, probes |
| `verify/scenes/scenes.as` | the AnimaScript mirror over `lib/box2d.as` |
| `verify/scenes/traj.c`, `traj.as` | recorder: `traj <scene> <steps> <workers> <out.bin>`; the wasm driver |
| `verify/scenes/dump.mjs` | prints and diffs recordings, probes included |
| `verify/scenes/build.sh` | builds `bin/traj_scalar` (parity build) and `bin/traj_simd` (NEON build) |
| `verify/traj.sh`, `verify/traj.mjs` | records natively (`NATIVE=1`), runs the wasm, reports the first divergence |

Every scene steps at `dt = 1.0f / 60.0f` with 4 sub-steps, world from `b2DefaultWorldDef()` with only
`workerCount` changed.

## Scenes

Counts at creation; bodies created by the script are in the final column. Every body is recorded in
creation order, destroyed ones keep their slot.

| scene | source | bodies | shapes | joints | steps | exercises |
|---|---|---|---|---|---|---|
| `pyramid` | the wave 1 pyramid (benchmarks.c CreateLargePyramid, base 10) | 56 | 56 | 0 | 300 | box stacking, warm start, graph coloring, island sleep (all asleep by step 35) |
| `shape_rain` | own; segment bin with ramps | 3 (+120) | 7 (+140) | 0 | 1000 | circles and capsules with rolling resistance, `b2MakeBox`, hulls from `b2ComputeHull` + `b2MakePolygon` with radius 0 and 0.1, `b2MakeRoundedBox`, two shape bodies from `b2MakeOffsetBox` + `b2MakeOffsetRoundedBox`, random friction, restitution and density, segment ground, a spinning and a sliding kinematic body reversed and stopped by script, bodies created mid run; all asleep by step 999 |
| `chain_car` | sample_joints.cpp Driving, car.cpp Car | 8 | 185 | 4 | 720 | open chain of 171 points along a smooth curve with ghost points and a chain material, a loop chain rock, the Car (wheel joints with motor, spring and limit), a bike (frame + rider block, rear wheel driven); script: drive, brake, reverse, torque and spring changes, `b2WheelJoint_EnableLimit`; the car and bike ride the hills and the rock to x 67 and 96 |
| `joints` | sample_joints.cpp rigs, Cantilever, Ragdoll with shared/human.c | 29 | 31 | 24 | 500 | distance (spring, limits, motor), motor joint (velocity then spring), filter joint (destroyed, then the boxes separate), prismatic, revolute and wheel (limit, motor, spring), rigid and soft weld cantilevers, ragdoll; `b2*Joint_Set*` changes mid run; force and torque thresholds fire 310 joint events |
| `dynamic_mover` | sample_character.cpp DynamicMoverSample, dynamic_mover.cpp | 71 | 166 | 53 | 900 | mover joint, pogo joint rebuilt every step with warm start, `b2World_CastRay` probe, motion locks, pre-solve and pre-continuous callbacks (one way elevator), plank bridge of 50 revolute springs, kinematic elevator by `b2Body_SetTargetTransform`, two loop chain levels |
| `churn` | own | 24 (+18) | 28 | 1 | 800 | create and destroy by script (index reuse), `b2DestroyShape`, `b2DestroyJoint`, `b2Body_SetAwake`, `Disable`/`Enable`, `SetType` static/kinematic/dynamic, `SetTransform`, velocity setters, forces, torques, impulses with and without wake, `ClearForces`, `WakeTouching`, `EnableSleeping` per body and per world, gravity scale; everything asleep at the end |
| `bullets` | sample_stacking.cpp VerticalStack, sample_continuous.cpp thin wall style | 18 (+3) | 19 (+3) | 0 | 400 | `isBullet` bullets at 200-300 m/s through a 12 box stack, each launch destroying the previous bullet (id `19.1`, `19.2`, `19.3`), circle, spinning thin box and capsule at 120-180 m/s against a 0.1 thick static wall (all stopped), a resting box made a bullet by `b2Body_SetBullet`; hit and contact events |
| `sensors` | sample_events.cpp sensor setups | 4 (+20) | 7 (+20) | 0 | 600 | static sensor zone, sensor on a kinematic sweeper, sensor ring on a rolling ball, visitors dropped through them, one visitor without sensor events, visitors destroyed, `b2Shape_EnableSensorEvents`; 49 begin, 43 end events |
| `events` | sample_events.cpp ContactEvent and BodyMove style | 18 | 19 | 0 | 600 | contact begin, end and hit events, `b2World_SetHitEventThreshold`, body move events with `fellAsleep`, `b2Contact_GetData`, `b2Contact_IsValid`, `b2Shape_GetContactData`, `b2Shape_EnableContactEvents`, `b2Body_EnableHitEvents`, body destroy |
| `queries` | own | 18 | 21 | 0 | 300 | `b2World_OverlapAABB`, `OverlapShape`, `CastRay` (all hits), `CastRayClosest`, `CastShape` (closest by clipping), `b2World_Explode`, `b2Shape_CastRay`, `TestPoint`, `GetClosestPoint`, `GetBounds`, `b2Body_ComputeBounds`, `b2World_GetBounds`, body point and vector helpers, custom filter callback |
| `mover` | sample_character.cpp GeometryMoverSample, geometric_mover.cpp | 70 | 166 | 52 | 900 | `b2World_CollideMover`, `b2SolvePlanes`, `b2World_CastMover`, `b2ClipVector`, `b2SpringDamper` pogo, soft planes from shape user data, pushing bodies with `b2Body_ApplyLinearImpulse`, ground force with `b2Body_ApplyForce` |
| `many_bodies` | benchmarks.c CreateManyPyramids, 10 x 10 | 5501 | 5510 | 0 | 100 | 100 pyramids of 55 boxes, sleep off; the perf scene (24 MB recording) |
| `large_pyramid` | benchmarks.c CreateLargePyramid | 5051 | 5051 | 0 | 100 | base 100 pyramid, sleep off |
| `joint_grid` | benchmarks.c CreateJointGrid | 10000 | 10000 | 19800 | 100 | 100 x 100 circles on revolute joints hanging from 7 static bodies, filter mask `~2u`, sleep off |
| `rain` | benchmarks.c CreateRain, StepRain | 1 (+12375) | 2505 (+14625) | 0 (+11250) | 360 | a column of 5 x 5 ragdolls every 8 steps onto 5 rows of 501 offset boxes; from step 320 the oldest column is destroyed (joints, then bodies, as human.c DestroyHuman) and rebuilt |
| `smash` | benchmarks.c CreateSmash | 9601 | 9601 | 0 | 120 | zero gravity, a dense box at 40 m/s into a 120 x 80 block created asleep (`isAwake = false`), impact near step 60 |
| `spinner` | benchmarks.c CreateSpinner, StepSpinner | 6078 | 6437 | 1 | 300 | 360 point loop chain with a zero material, motorized rounded box paddle (`maxMotorTorque = FLT_MAX`), 6076 capsules, circles and boxes |
| `tumbler` | benchmarks.c CreateTumbler | 2027 | 2029 | 1 | 200 | four wall box on a 25 deg/s revolute motor turning 2025 small boxes |

### Deviations from the samples

- Random values come from the scene LCG (`seed * 1664525 + 1013904223`, float from the top 24 bits)
  instead of `rand()`, each draw its own statement.
- `chain_car`: Driving's track is replaced by the smooth chain; car and bike shapes use
  `groupIndex = -1` so they never collide with each other. The bike is ours, built like the Car.
- `joints`: `CreateHuman` is table driven, one row per bone in the C's creation order, without
  colors, names and user data; the joint frames and limits are the C's.
- Character samples: the svg levels go through `Sample::ParsePath` offline and are stored as float
  bits, so neither side parses floats. `cosf` in the elevator script is
  `b2ComputeCosSin( angle ).cosine`. The geometric sample points shape user data at a
  `MoverShapeUserData`; here user data is a tag (1 friendly capsule, 2 elevator) mapped back to the
  same push limit and clip flag. Keys are scripted: throttle 0 for steps 0-59, +1 to 299, -1 to
  359, +1 after; jump held for 4 steps from 150 every 300.
- `dynamic_mover` keeps the sample order (mover update, then world step); `mover` steps the world
  first, as `GeometryMoverSample::Step` does.
- `many_bodies`: 10 x 10 pyramids instead of 20 x 20.
- The six benchmarks.c scenes use the release (`NDEBUG`) sizes and the benchmark's `b2Capacity` as
  `worldDef.capacity` (`Scene_GetCapacity`); every other scene keeps a zero capacity. The scripted
  steps run before `b2World_Step` with the step index, as benchmark/main.c calls a step function.
- `rain`: the humans come from the table driven `Scene_CreateHuman` (the one `joints` uses) with a bone
  slot, so `Scene_DestroyHuman` finds each bone's body and joint as human.c's `Human` does. 360 steps
  so the destroy and rebuild phase (from step 320) is covered; its heap keeps growing with the bodies
  the script adds, and the recorder's per step buffer grows with the body list.
- `smash` and `tumbler` grow the heap until the contact count peaks (steps 150 and 100) and are flat
  from there to step 800; `large_pyramid`, `joint_grid` and `spinner` are flat between steps 20 and 200.

## Format, version 2

As version 1 (header in `traj.c`), with header word 1 = 2 and, after the bodies of each step, a word
count P followed by P probe words the scene pushed during `Scene_Step`. Floats are bit patterns,
ids are two words (`index1`, `generation`), bools 0 or 1. `dump.mjs` reads versions 1 and 2,
`print --probes` shows the words, `diff` reports the first differing probe word with context, and
`traj.mjs` folds it into the first divergence.

## Probe layouts

Shared blocks: **joint** = valid, then if valid force xy, torque, linear separation, angular
separation. **manifold** = pointCount, normal xy, rollingImpulse, per point anchorA xy, anchorB xy,
separation, normalImpulse, tangentImpulse, totalNormalImpulse, id, persisted (normalVelocity is left
out, the solver computes it only for hit events). **contact data** = contactId, shapeIdA, shapeIdB,
manifold. **contact events** = beginCount, (shapeA, shapeB, contactId)*, endCount, (same)*, hitCount,
(shapeA, shapeB, contactId, point xy, normal xy, approachSpeed)*. **hits** = callback calls, count,
shapeId* (with point xy, normal xy, fraction for casts).

| scene | after `b2World_Step` |
|---|---|
| `pyramid`, `shape_rain`, `many_bodies`, `large_pyramid`, `joint_grid`, `rain`, `smash`, `tumbler` | none |
| `spinner` | before `b2World_Step`, as the benchmark's StepSpinner: `b2RevoluteJoint_GetCurrentAngle` of the spinner |
| `chain_car` | car then bike: rear axle motor torque, rear joint, front motor torque, front joint, rear wheel contact count, contact data* |
| `joints` | joint events (count, (jointId, userData)*), joint block for key slots 0..13, then distance length, spring force, motor force, prismatic translation, speed, motor force, revolute angle, speed, motor torque, wheel motor torque |
| `dynamic_mover` | cast hit, point, normal, bodyId, fraction, velocity, onGround, walkable, jumping, jumpTicks, pogo impulse, velocity, length, pogo jointId, mover joint, pogo joint |
| `churn` | compound mass, inertia, local center, shape count, pair joint count, disabled body enabled flag, compound sleep enabled, world sleep enabled, spawnCount, spawnHead |
| `bullets` | on a launch step first bulletId, speed; then dart isBullet, contact events |
| `sensors` | begin count, (sensor, visitor)*, end count, (sensor, visitor)*, then per sensor (zone, sweeper, ring) capacity, count, visitorId* |
| `events` | contact events, per begin event valid then pointCount and normal, per end event shapeA valid and shapeB valid, move events (count, (bodyId, p, q, fellAsleep)*), first shape contact capacity, count, contact data* |
| `queries` | overlap AABB hits, overlap shape hits, ray all hits, closest ray (hit, shape, point, normal, fraction), shape cast hits, two `b2Shape_CastRay` outputs (hit, point, normal, fraction), test point, closest point, test point, body bounds, shape bounds, world bounds, world point, local point, world vector, local vector, local point velocity, world point velocity, world center |
| `mover` | position, velocity, pogoVelocity, onGround, pogoHit, pogoFraction, totalIterations, planeCount, per plane normal, offset, pushLimit, push, clipVelocity, point, shapeId |

Queries run after the step, on the state that step recorded. Nothing is probed from inside
callbacks that run during `b2World_Step` (custom filter, pre-solve), whose call order follows the
worker split.

## Native determinism

`NATIVE=1 verify/traj.sh` records `out/<scene>_scalar_w1.bin` and `out/<scene>_simd_w1.bin`. On
2026-10-27 the NEON recording of every scene is byte identical to the scalar one, and scalar and
NEON at 4 and 8 workers match scalar w1 in bodies, physics counters and every probe word (only the
environment counters differ). Body states, events and query results do not depend on the build or
the worker count.

## Notes for the mirror

- The mirror type checks with `rasc check` against the committed lib plus stubs for the joint API
  (`b2Create*Joint`, `b2*Joint_*`), which `lib/` does not export yet, and compiles to wasm. With
  `b2Solve` stubbed, the `mover` scene's probes match native for 60 steps and the static queries of
  `queries` match word for word, so the builders, the level data and the probe layouts line up.
- Assumed shapes of the joint API, unported when this was written: `b2Create*Joint(worldId, def)`
  by value, setters `(jointId, value)`, getters return `f32`, `b2Joint_GetConstraintForce` returns
  `b2Vec2`, `b2*Joint_SetLimits(jointId, lower, upper)`.
- Callbacks carry no context, so `Scene_CastCallback`, `Scene_PlaneResultFcn`, the query callbacks
  and the pre-solve callbacks read module globals (`s_castResult`, `s_planeMover`, `s_hits`,
  `s_preSolveScene`) set right before the call.
- `b2MakeProxy` takes a `FixedArray<b2Vec2, 8>`; `b2GetLengthAndNormalize` writes through a
  `FixedArray<f32, 1>` element, as in box3d-as.
