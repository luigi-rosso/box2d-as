# threading: Box2D's tasks on parallel calls

C Box2D threads by handing tasks to the user's `enqueueTask` and joining them with `finishTask`: `b2ParallelFor`
(narrow phase, finalize bodies, pair finding, sensors, bullets), the solver's stage workers, the tree update and the
island split. An AnimaScript function value cannot cross to another thread, so the port ran every task inline on
worker 0. Rive's parallel calls (`rive/parallel`, on rive master since 4a861ff147) run one `@parallel` function on
several threads at once over the main lane's own memory, which is what C's workers do, so the tasks map onto them
without copying any world state. The 3D port did this first (box3d-as, `notes/threading.md` there); this is the same
design on Box2D v3.2, except for the solver stages (see "No sync per stage").

## Use

```as
import { b2World_EnableParallel, b2SetParallelLimit, b2_parallelAll } from "./lib/parallel_step";

b2World_EnableParallel(worldId, b2_parallelAll); // from init or later, not from top level code
b2SetParallelLimit(4);                           // optional: at most 4 parts, 0 for every part
```

- `tasks` picks among `b2_parallelSolve`, `b2_parallelCollide`, `b2_parallelFinalize` and `b2_parallelPairs`; 0 turns it
  off. The setting is per world.
- The world gets a task context per part: its worker count becomes `Parallel.size()`, the job pool plus the main lane
  (`--job-threads=n` gives n + 1, `--parallel-safe` 1). Results do not depend on it, as C's do not.
- A build that never imports `lib/parallel_step` has no parallel call and no parallel copies: `Parallel.inPart()` is
  the constant false there and the serial path is what it was.
- It needs a runtime with parallel calls. One from before 4a861ff147 traps on the first call.
- Call it between steps: a locked world refuses it as it refuses `b2World_SetWorkerCount`. `Parallel.size()` is 1 in
  a file's top level code and under a debugger, and a world enabled there stays on one part until it is enabled again.
- A step that fails, in a part or not, leaves the world locked, as it does without parallel calls.

## Results

Bit exact at every part count, on rive master at 5d0e608da3 (rasc, rive-cli and a wamrc built from its WAMR patches):

| gate | runs | passed |
|---|---|---|
| `verify/threads.sh --large`: 18 scenes at 1 part (`--parallel-safe`), 2, 4, 8 (`--job-threads=1/3/7`) and the default pool, AOT, SIMD artifact | 90 | 90 |
| the same on the scalar artifact (`WASM_SIMD=off`), 5 scenes at 1 and 4 parts | 10 | 10 |
| the same on the interpreter (`TIER=interp`), 5 scenes at 1 and 4 parts | 10 | 10 |
| rain, smash, spinner, joint_grid and churn at 8 parts, three runs each | 15 | 15 |
| `CALLBACKS=63 verify/threads.sh`: 11 scenes at the same part counts, with stateful callbacks and bullets, against the serial port's run | 55 | 55 |
| `verify/parts.sh`: pyramid and large_pyramid at 2, 4 and 8 parts with part 0 running no block, and again with a sync every 8 polls; a trap in a worker and one in part 0 | 14 | 14 |

In browsers (`bench/web/parallel.mjs`: the same bakes on a cross origin isolated page over librive's web scripting
harness, built with threads from the same master; headless Chrome 157, WebKit 27.2 and Firefox 157): pyramid, joints,
churn, bullets, sensors, events, large_pyramid and rain at 1, 2, 4 and 8 parts and the default pass 40 of 40 in each
browser, and no run hung. Only parts other than 0 sleep, since a page's thread may not wait.

Each run logs, for every step, the hash of the bodies' state and the hash of the whole trajectory record but its
build and worker dependent counters (`verify/scenes/record.as`), so contact, sensor, joint and body events, counters,
sleep flags and the scenes' probes are held to the native recording too, not only poses. The checker also fails a
run whose `Parallel.size()` or whose solver calls' part count is not what it was given, or that never ran a task on
parts that it was asked to. All of it ran on the final lib with the machine at a load of 200 to 600 from other work,
which shakes out races better than a quiet one. The serial gates (`run.sh`, `traj.sh`, `heap.mjs`) pass unchanged on
both artifacts.

Whole step, ms per step (`bench/as/threads.sh`, session `bench/as/out/threads_20261115_200056.log`, AOT -O3 with
hardware bounds as master's CLI ships, SIMD artifact, Apple M5 Max (6 performance and 12 efficiency cores), 2 rounds interleaved with native
SIMD at the same worker count, each run the best of 3 repeats, min over rounds, 1 minute load 5 to 10; x is the same
row's 1 over the cell, and the last column is AS parallel over native at 1 / 2 / 4 / 8):

| scene | impl | 1 | 2 | 4 | 8 | parallel / native |
|---|---|---|---|---|---|---|
| large_pyramid | native simd | 1.155 | 0.643 (x1.80) | 0.398 (x2.90) | 0.494 (x2.34) | |
| large_pyramid | AS serial | **1.270** | | | | |
| large_pyramid | AS parallel | 1.259 | 0.691 (x1.82) | 0.428 (x2.94) | 0.493 (x2.56) | 1.09 / 1.07 / 1.08 / 1.00 |
| many_bodies | native simd | 0.900 | 0.488 (x1.85) | 0.300 (x3.00) | 0.464 (x1.94) | |
| many_bodies | AS serial | **1.014** | | | | |
| many_bodies | AS parallel | 1.004 | 0.553 (x1.81) | 0.332 (x3.03) | 0.417 (x2.41) | 1.12 / 1.14 / 1.11 / 0.90 |
| joint_grid | native simd | 2.208 | 1.241 (x1.78) | 0.758 (x2.91) | 0.656 (x3.36) | |
| joint_grid | AS serial | **2.421** | | | | |
| joint_grid | AS parallel | 2.404 | 1.299 (x1.85) | 0.788 (x3.05) | 0.699 (x3.44) | 1.09 / 1.05 / 1.04 / 1.06 |
| rain | native simd | 2.066 | 1.211 (x1.71) | 0.789 (x2.62) | 0.818 (x2.53) | |
| rain | AS serial | **2.232** | | | | |
| rain | AS parallel | 2.215 | 1.313 (x1.69) | 0.853 (x2.60) | 0.790 (x2.80) | 1.07 / 1.08 / 1.08 / 0.97 |
| smash | native simd | 0.893 | 0.655 (x1.36) | 0.503 (x1.78) | 0.633 (x1.41) | |
| smash | AS serial | **1.028** | | | | |
| smash | AS parallel | 1.040 | 0.731 (x1.42) | 0.603 (x1.72) | 0.693 (x1.50) | 1.16 / 1.12 / 1.20 / 1.09 |
| spinner | native simd | 3.155 | 2.163 (x1.46) | 1.445 (x2.18) | 1.667 (x1.89) | |
| spinner | AS serial | **3.714** | | | | |
| spinner | AS parallel | 3.670 | 2.456 (x1.49) | 1.736 (x2.11) | 1.691 (x2.17) | 1.16 / 1.14 / 1.20 / 1.01 |
| tumbler | native simd | 0.589 | 0.417 (x1.41) | 0.319 (x1.84) | 0.470 (x1.25) | |
| tumbler | AS serial | **0.659** | | | | |
| tumbler | AS parallel | 0.660 | 0.451 (x1.46) | 0.344 (x1.92) | 0.379 (x1.74) | 1.12 / 1.08 / 1.08 / 0.81 |

- **Parallel calls track native**: 1.04 to 1.20x native from 1 to 4 parts, the ratio the single lane already has
  (1.07 to 1.16x), and the step gets 1.7 to 3.05x faster from 1 to 4 parts where native gets 1.8 to 3.0x. One part
  runs at the serial path's speed (0.99 to 1.01x).
- **8 parts is no faster than 4** on most scenes, for native too: the machine has 6 performance cores. The port loses
  less there (0.81 to 1.09x native).
- **Stage by stage at 4 parts**, ms per step against native: the solver stages, with the tree build in their call,
  0.302 against 0.281 on large_pyramid, 0.275 against 0.256 on many_bodies, 0.629 against 0.597 on joint_grid, 0.518
  against 0.501 on rain, 0.186 against 0.173 on tumbler, and furthest on smash (0.293 against 0.246) and spinner
  (0.766 against 0.649); the narrow phase 0.031 against 0.036 on large_pyramid, 0.055 against 0.050 on rain, 0.495
  against 0.433 on spinner; finalize 0.029 against 0.023, 0.095 against 0.079, 0.158 against 0.138; pairs, whose
  sorting and contact creation stay serial, 0.051 against 0.046, 0.131 against 0.100, 0.286 against 0.196.
- **The tree update belongs in the solver's call.** It first ran in the narrow phase's, which is too short to hide
  it: in a session of that build (`threads_20261115_162831.log`, load 6 to 13) collide read 0.099 ms at 4 parts on
  large_pyramid where native's was 0.034, 0.237 on rain (0.047) and 0.342 on joint_grid, which has no contacts at
  all, and joint_grid ran at 1.39 and 1.55x native at 4 and 8 parts where it now runs at 1.04 and 1.06x.

Timings need a quiet machine: native large_pyramid at 1 worker reads about 1.1 ms when it is quiet enough. Sessions
taken while other work held the load at 15 and above are not used.

## No sync per stage

The first version walked the stages as box3d-as does: every part claims blocks with `part.claim` and calls
`part.sync()` after each stage, about a hundred syncs a step. A sync waits for every part of the call. C waits for a
stage's blocks instead: whoever is running claims them, and a worker that is not scheduled holds nobody up unless it
is in the middle of a block.

- **Under load the difference is the whole step.** With the machine at a load of about 180 (other work, 18 cores),
  the synced solver at 8 parts took 19.1 ms a step on large_pyramid (17.8 of it in the solver stages), 22.8 on
  many_bodies and 18.6 on joint_grid, while native at 8 workers took 1.38, 1.60 and 4.27 in the same round. One call
  that waits for every part costs a fraction of a millisecond there (collide and finalize each read 0.3 to 0.5 ms);
  a hundred of them cost the step.
- **On a quiet machine C's scheme is faster too.** Interleaved A/B of the two builds, 3 rounds at load 8 to 10, mean
  ms per step and the solver stages:

| scene | parts | sync per stage | C's scheme | solver stages |
|---|---|---|---|---|
| large_pyramid | 2 | 0.737 to 0.749 | 0.669 to 0.716 | 0.495 to 0.498 -> 0.428 to 0.454 |
| large_pyramid | 4 | 0.544 to 0.564 | 0.440 to 0.480 | 0.355 to 0.372 -> 0.257 to 0.285 |
| large_pyramid | 8 | 0.608 to 0.638 | 0.517 to 0.589 | 0.414 to 0.433 -> 0.323 to 0.378 |
| rain | 4 | 0.939 to 0.997 | 0.883 to 0.924 | 0.472 to 0.500 -> 0.417 to 0.437 |

So the solver runs C's `b2SolverTask` as it is, and nothing in the backend syncs: the narrow phase, finalize and pair
finding are single claim loops. Each call still returns only when every part has, which is the runtime's.

With the machine far oversubscribed, spinning workers hold their cores against the parts that have work, where
native's waiting workers yield. large_pyramid again, the three builds interleaved, mean ms per step (median in
parentheses where the mean is mostly a few stalled steps):

| load | parts | sync per stage | C's scheme, spinning | C's scheme, sleeping |
|---|---|---|---|---|
| 250 to 330 | 8 | 29.1 to 33.2 (16.6 to 17.7) | 10.3 to 21.8 (3.0 to 8.2) | |
| 350 to 430 | 8 | 31.1 to 37.4 (17.7 to 18.2) | 8.2 to 12.8 (3.4 to 3.6) | 5.6 to 6.8 (2.5 to 2.7) |
| 42 to 80 | 8 | 1.00 to 1.13 | 0.66 to 0.98 | 0.71 to 0.85 |
| 42 to 80 | 4 | 0.93 to 0.98 | 0.68 to 0.72 | 0.70 to 0.75 |

The 350 to 430 row slept on a timer alone; the wake by part 0 came after it, so that a sleeping worker never holds
the call's return for the length of a coarse timer. Native was not measured at those loads; at a load of 180 it had
read 1.4 ms at 8 workers, so the port is still behind it there.

Sleeping costs nothing visible at a load of 42 to 80, and little when quiet. Spinning against sleeping, interleaved,
3 rounds at load 7 to 12, mean ms per step:

| scene | parts | spinning | sleeping |
|---|---|---|---|
| large_pyramid | 2 | 0.688 to 0.710 | 0.679 to 0.720 |
| large_pyramid | 4 | 0.401 to 0.428 | 0.406 to 0.434 |
| large_pyramid | 8 | 0.484 to 0.497 | 0.476 to 0.509 |
| rain | 2 | 1.280 to 1.330 | 1.296 to 1.340 |
| rain | 4 | 0.818 to 0.844 | 0.828 to 0.888 |
| rain | 8 | 0.743 to 0.789 | 0.802 to 0.823 |
| many_bodies | 2 | 0.577 to 0.595 | 0.546 to 0.601 |
| many_bodies | 4 | 0.333 to 0.348 | 0.343 to 0.362 |
| many_bodies | 8 | 0.390 to 0.514 | 0.419 to 0.431 |

Within the spread at 2 and 4 parts; rain at 8 parts, more than the machine has fast cores, is about 6% slower.

## What runs on parallel calls

| task | C | here |
|---|---|---|
| solver stages | `b2SolverTask`: worker 0 orchestrates, the others spin on the sync bits and CAS claim blocks from their home index | the same function, one `Parallel.runUpTo` per step with a part as each worker (`b2SolverWorker(context, part.index, part)`): part 0 orchestrates, and C's atomics are real atomics inside a part. Where C's worker yields, a part sleeps on the sync bits once it has spun for a while, and part 0 wakes it with the next stage |
| narrow phase | `b2ParallelFor(b2CollideTask)` | one call claiming C's block sizes |
| tree update | a task of its own that overlaps the narrow phase and the solver | the last part of the solver's call builds the trees before it joins the stages; the main lane reserves the arrays when C would queue the task and swaps the built nodes in after the call, where C finishes it |
| finalize bodies | `b2ParallelFor(b2FinalizeBodiesTask)`, continuous collision inline | one call, continuous included |
| pair finding | two `b2ParallelFor`s, cross seeds and self pairs | one call with one claim loop over the cross seeds and then the self pair blocks; keys go to one `SharedList` |
| bullets, sensors, island split | parallel for or task | on the main lane, as before |

A task answers false and runs inline, as before, when the world cannot run it on parts:

- narrow phase: a pre-solve, friction or restitution callback is set (parts cannot call a function value);
- pair finding: a custom filter is set, or the key list filled up (it is then made four times larger for the next step);
- continuous collision inside finalize: the world has sensors (a sweep pushes sensor hits), a custom filter or a
  pre-continuous callback. The parts then finalize every body but leave the fast ones in their task context, and the
  main lane sweeps those after the call.

Callbacks so run only on the main lane, and in the serial step's order: the main lane sorts the fast bodies the parts
left it, and the bullets they listed, by sim index before it sweeps them, since parts list them in the order they
claimed blocks. A callback that keeps state of its own, a counter or a random number, then answers as it does in the
serial step. `CALLBACKS=63 verify/threads.sh` holds that with callbacks whose answers depend on how many calls came
before, and with about half the dynamic bodies bullets so both sweeps make them. What a callback reads from the world
inside a sweep is not the same: the left over sweeps run once every body is finalized, where the serial step has
finalized only the bodies before the one it sweeps. C's workers finalize while its callbacks run, so it promises
neither.

## What the port needed

- **A task context per part.** `b2World_EnableParallel` sets the worker count to `Parallel.size()`, and parts use
  `part.index` as C's worker index.
- **Per worker scratch**, where the perf passes had put C's stack arrays in module globals: the three polygons the
  manifold functions build (`b2_workerPolygons`, reached through `b2CollidePolygonsAt` and the other `At` variants, which
  take the worker index; the public functions pass 0), the continuous context and its tree query stack (now in the
  task context), the pair contexts (already one per worker).
- **`Parallel.inPart()`** around what a part may not do: the user callbacks, pushing sensor hits, logging an unstable
  body, binding a pair context to its world.
- **Storage that does not grow in a part**: pair keys go to a `SharedList`, deferred fast bodies to a `StaticArray`
  sized for every awake body, and the tree rebuild's allocations and array swap moved out of the build
  (`b2DynamicTree_ReserveRebuild`, `BuildRebuild`, `FinishRebuild`).
- **C's atomics, real inside a part**: the solver's sync bits, block sync indices, completion counts and main claim,
  and the bullet count. solver.as's stand-ins take the value's address and pick `atomic.*` under `Parallel.inPart()`,
  a plain load and store elsewhere, so the serial build has no atomic instruction. The spin loops need the atomic
  loads: a plain load in a loop may be hoisted out of it. The tree's moved flags stay plain: every writer ors in the
  same bit and nothing else changes a node's flags during finalize, so a lost update writes the value the winner wrote.
- **A sleep where C yields.** A waiting C worker calls `sched_yield` every few spins; a part has nothing like it, and
  a worker that only spins holds its core against the parts that have work when there are more threads than cores.
  After 2048 yields in one wait (some tens of microseconds) a worker now sleeps with `atomic.wait` on the sync bits,
  and part 0 calls `atomic.notify` when it stores new ones and a worker sleeps (`b2YieldWorker`,
  `b2PublishSyncBits`). Only parts other than 0 wait, as a page's thread may not.
- **A sync when a stage wait runs long.** With C's scheme a part that traps would hang the call: the trapped worker
  never adds its completion count, part 0 spins on it, and no part can read the call's stop flag, which only
  `part.sync()` checks. So each time part 0 has polled one stage 2^24 times it bumps a count in the step context and
  syncs, and a worker that finds the count changed in its wait loop syncs too (`b2RequestSync`, `b2AnswerSync`). If
  a part trapped the sync fails the call with its error; otherwise every part goes on once all have arrived, which
  costs one of the old barriers, a part that has not started yet included. A run that needed none reports
  `parStageSyncs=0`, as the gate's runs did with the machine at a load of 600. The other way round, a worker that
  has slept 500 times in one wait returns, so the call ends once the workers have, and part 0 can finish any step
  without them: one that part 0 leaves alone for half a second finishes on part 0.
  - How long a trap takes to end the call depends on the tier, since 2^24 polls do. With a trap injected at 4 and 8
    parts on AOT, a worker's trap ends the call in about 30 ms and part 0's in about 0.6 s. On the interpreter a
    worker's took 2.4 to 2.9 s on a loaded machine, and part 0's 26 ms, since the host stops the other parts there.
    Before this a worker's trap hung the call until it was killed.
  - On a page a requested sync is also where the host gives up on a team thread that has not started its part
    within a second, which it otherwise does at the join.
  - `verify/parts.sh` holds all of it on edited copies: part 0 running no block (the other parts must run them all),
    a sync every 8 polls, and a trap in a worker and in part 0.
  - It costs nothing that shows. Interleaved with the lib before it on large_pyramid and rain at 1, 2, 4 and 8 parts
    (load 9 to 15, min over 4 to 8 rounds of the best of 3 repeats), the step is within 3% either way, 8% faster in
    one row, and no run made a stage sync. The tables above were measured before it.
- **Marked `@parallel.shared`**: the identity body state and the zero contact sim (stand-ins nothing writes), the
  worker polygons and pair contexts (indexed by worker), the tree rebuild stacks (one part rebuilds), the stats
  (among the parts only part 0 writes them).

The build check found every one of these but the atomics. The solver stages compiled on the first build with two
warnings, both for stand-ins that are never written; the narrow phase named three callbacks and three scratch globals;
finalize named the logging, the shared continuous context, the query stack, the sensor hit push and two callbacks. What
it cannot see is a race through an object passed to the call, which is where the bullet count and the solver's sync
state are: found by reading every write in the four tasks, not by the build.

Size: `lib/parallel_step.as` is about 200 lines, and making the rest of lib parallel safe changed about 600.

## What is left

- **Friction and restitution callbacks** could be `@parallel` function types, which the build checks where they are
  stored, so a world with custom mixing keeps a parallel narrow phase. It changes the public callback types.
- **Sensor hits from parts**: a fixed buffer per worker, 8 hits a sweep at most, would keep continuous on the parts
  in a world with sensors.
- **Bullets, sensor overlaps and the island split** still run on the main lane. Only scenes with many bullets or
  sensors would notice.
- **A part cannot read its call's stop flag**, which is why a trap is found by a sync after a long wait and not at
  once. A std accessor for it would let both wait loops poll it and drop the sync requests and the worker's idle
  limit.
- **The gates do not hold speed.** A build that stopped using the parts after the first step, or whose workers
  parked where they should work, would pass them as long as its results were right; `bench/as/threads.sh` shows it
  in the scaling.
