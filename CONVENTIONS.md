# Porting conventions

Box2D (C17, github.com/erincatto/box2d) ported to AnimaScript, compiled by rasc.
Upstream pin: `178495ba35695aefe9c321e569d2277f06db3b90` (v3.2.0),
checked out at `~/Projects/box2d`. The parity reference is the scalar build at
`~/Projects/box2d/build/scalar` (`BOX2D_DISABLE_SIMD=ON`, `-ffp-contract=off`,
`BOX2D_VALIDATE=OFF`, Release).

The goal is a faithful port: same algorithms, same operation order, same float
results bit for bit. A difference is either float drift we can explain or a bug.

## Toolchain

```sh
RASC=~/Projects/rive-pinefall/packages/rasc/animascript/out/release/rasc
AS_STD=~/Projects/rive-pinefall/packages/rasc/animascript/std
$RASC entry.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -O -o out.wasm
```

- `--runtime stub` or `--runtime frame`; `minimal` and `incremental` reject
  arrays of structs.
- `rasc check <file> --runtime stub --lib "$AS_STD/assembly"` reports
  diagnostics without output.
- Never edit the compiler or std. When rasc refuses something it should
  accept, or miscompiles, write a minimal repro into your notes file under
  "rasc findings" and work around it.

## Files

- One `.as` per C source, same basename, under `lib/`. A private header
  merges into the `.as` of the same name (`shape.h` + `shape.c` ->
  `lib/shape.as`). Headers without a `.c` get their own file
  (`math_internal.h` -> `lib/math_internal.as`).
- Public headers: `include/box2d/math_functions.h` + `math_types.h` + `src/math_functions.c`
  -> `lib/math_functions.as`; `types.h` + `types.c` -> `lib/types.as`;
  `collision.h` -> `lib/collision.as`; `constants.h` -> `lib/constants.as`;
  `base.h` + `core.h` + `core.c` -> `lib/core.as`; `box2d.h` is implemented
  across the module files as in C and re-exported from `lib/box2d.as`.
- Keep the order of functions in each file as in C, so the two diff side by
  side.
- Start each file with the C file's own SPDX copyright line, then:
  ```
  // SPDX-FileCopyrightText: <year from the C file> Erin Catto
  // SPDX-License-Identifier: MIT
  // AnimaScript port of src/<file>.c
  ```
- Keep upstream comments where the code they describe survives. Our own
  comments only when a constraint is not visible in the code, one line.
- Each porting task owns a set of files. Edit only those, never commit;
  write deviations, skipped functions and rasc findings to
  `notes/<task>.md`.

## Names

C identifiers stay verbatim: `b2Dot`, `b2MakeBoxHull`, `b2World_Step`,
`b2_dynamicBody`, `B2_LINEAR_SLOP`, struct field names. Static C functions
stay unexported.

## Types

| C | AnimaScript |
|---|---|
| `float` | `f32` |
| `double` | `f64` |
| `int` | `i32` |
| `uint32_t` / `uint16_t` / `uint8_t` / `uint64_t` / `int64_t` | `u32` / `u16` / `u8` / `u64` / `i64` |
| `bool` | `bool` |
| `void* userData` | `u64` |
| `const char* name` | `string` |
| `typedef enum b2X { b2_a, b2_b }` | `export type b2X = i32;` plus `export const b2_a: b2X = 0;` for each enumerator, so call sites read as in C |
| `#define B2_X 4` | `export const B2_X: i32 = 4;` |
| `#define` that reads a runtime global (`B2_LINEAR_SLOP` uses `b2_lengthUnitsPerMeter`) | `@inline export function` with the same name, so it is evaluated at each use |
| fixed array member `b2Vec2 v[8]` | `FixedArray<b2Vec2, 8>` |
| `b2Array(T)` | builtin `Array<T>`; `.count` is `.length` |

Float literals: AnimaScript types a bare literal local as `f64`. Annotate every
float local, `let h: f32 = 0.5;`, and cast integer to float explicitly,
`<f32>i`. A literal inside an `f32` expression (`h * 0.5`) is already `f32`.
An `f64` leaking into float math changes results, so the parity checks will
find it, but avoid it up front. C float to int casts truncate; wasm traps on
NaN or out of range, so where C could see such a value use the saturating
helper from `lib/core.as`.

`sqrtf` is `Mathf.sqrt` (one instruction, exact). `floorf` is `Mathf.floor`.
`fabsf` is `Mathf.abs`. `remainderf` is `Mathf.rem`. Box2D has its own
`b2ComputeCosSin` and `b2Atan2`; port them verbatim and never use
`Mathf.sin`/`cos`/`atan2`, which round differently from libm.

## Structs and classes

- A C struct whose members are all plain data (numbers, other plain structs,
  fixed arrays) becomes a rasc `struct`: `b2Vec2`, `b2Rot`, `b2Transform`,
  `b2AABB`, `b2Polygon`, `b2Circle`, `b2Capsule`, `b2Segment`, `b2ChainSegment`,
  ids, `b2Manifold`, `b2ManifoldPoint`, `b2Body`, `b2BodySim`, `b2BodyState`,
  `b2Contact`, `b2ContactSim`, `b2Joint`, `b2JointSim`, `b2Shape` (its union
  becomes one field per member), joint defs, shape defs.
  Structs copy on assignment, never allocate, and return by multi-value.
- A C struct that holds a pointer, a `b2Array`, or that C only ever handles
  through a pointer it owns, becomes a reference `class`: `b2World`,
  `b2Island`, `b2SolverSet`, `b2GraphColor`, `b2Sensor`, `b2BroadPhase`,
  `b2DynamicTree`, `b2ChainShape`. Follow what box3d-as chose for the twin
  type (below) unless Box2D's C differs.
- Unions become one field per member.
- Structs cannot be generic arguments of our own classes, so there is no
  container class: use `Array<T>` / `StaticArray<T>` directly. Functions
  generic over the array type do work, so `lib/container.as` has
  `b2Array_RemoveSwap`, `b2Array_AddIndex`, `b2Array_Emplace` (returns an
  index), `b2Array_Resize` and `b2Array_Append` for any `Array<V>`.
- rasc refuses `ref` to a primitive local or field, so `B2_SWAP` is written
  inline (`let t = x; x = y; y = t;`); `b2SwapElements(a, i, j)` swaps array
  elements.
- Arena, stack and block allocators are not ported: each `b2Bump` or
  `b2StackAlloc` becomes a local `StaticArray<T>` and its free disappears,
  `b2Arena` parameters are dropped. `notes/containers.md` lists every site.
- Struct methods are allowed but keep C's free function style
  (`b2Add(a, b)`) for diffability.

## Pointers

- `const T*` input that C only reads: pass `T` (a copy) for small structs,
  `ref T` when the struct is large or C writes through it.
- `T* out` parameters: `ref T`, or return the value when C's function only
  fills one output.
- `T* p = array + i` then reads and writes through `p`: when the array is a
  `StaticArray<T>` or `FixedArray`, `ref p = array[i]`. When it is a
  growable `Array<T>`, rasc refuses a `ref` (push may move it); write through
  the place directly (`arr[i].v.x = ...`) or copy out, mutate, store back.
- A function that returns a pointer into an array returns the index instead,
  and the caller binds the element.
- Pointers stored inside other structs (`b2ContactSim**`) become indices.
- `memcpy` / `memset` of struct arrays become loops or `Array` copies.

## Callbacks

A C callback plus `void* context` becomes a function typed parameter; the
closure carries the context. Keep the callback's C name as the type alias.

## Asserts and validation

`B2_ASSERT(x)` stays as a call to `B2_ASSERT` from `lib/core.as`, compiled out
unless built with `-D B2_ENABLE_ASSERT`. `B2_VALIDATE` code is skipped.

## Skipped

Debug draw (`b2World_Draw` and `b2DebugDraw`), the internal scheduler and the
SSE2, AVX2 and NEON backends. Profiling timers are ported on
`clock()` from `rive/os` (notes/bench.md). The `B2_SIMD_NONE` path of `contact_solver.c` is ported: with `BOX2D_DISABLE_SIMD` the
contact solver still runs its 4 lane structure through it, and the
port keeps that structure so it matches bit for bit. Wasm SIMD later replaces
the scalar lane type behind `ASC_FEATURE_SIMD`.

## Verification

Each module gets checks against the native scalar build:

- `verify/cref/<module>.c`: links `~/Projects/box2d/build/scalar/src/libbox2d.a`
  (include `~/Projects/box2d/src` for internals), built with
  `clang -O2 -ffp-contract=off -DBOX2D_DISABLE_SIMD -DNDEBUG`. It prints
  vectors through `verify/cref/common.h`: `vtag`, `vi32`, `vu32`, `vbool`,
  `vf32` (`%08x` bits), `vu64` and `vf64` (`%016llx`), `vend` ends a line.
- `verify/as/<module>.as`: exports driver functions over `lib/` that print the
  same lines through `verify/as/common.as`, which has the same functions.
  Floats cross to the host as bits, so NaN payloads survive.
- `verify/<module>.mjs`: usually three lines, `runModule(name, drive)` from
  `verify/lib.mjs`. It loads the wasm, lets `drive` call the exports, then
  compares line by line with the C vectors, exactly, and prints the first
  mismatches with context and decoded floats, then a total.
- `verify/run.sh [module...]` builds and runs each module and stops at the
  first failure (`KEEP_GOING=1` to continue). Modules are found by file name;
  optional `cref/<m>.flags` and `as/<m>.flags` add compiler arguments.
  `RASC_FLAGS` replaces the default `-O`; run once with `RASC_FLAGS=` too.

Inputs: a fixed set plus a seeded sweep from `vseed`/`vrand32`/`vrandint`/
`vrandf`/`vspecialf` (splitmix64, identical on both sides). C leaves the
evaluation order of arguments and initializers open, so in C drivers draw each
random value in its own statement. A check passes only when the bits match.

Runtime: drivers compile with `--runtime frame`. The stub never frees, and its
`__mark`/`__resetTo` rewind would free a world that outlives the rewind. Under
frame, a minor collection runs only at a frame boundary, where only globals are
roots, so a long run keeps its world in module globals, steps it from the host
one export call at a time, and calls `ctx.frameBoundary()` between calls. Every
driver re-exports `frameBoundary` from `verify/as/common.as`. Never collect from
inside a driver call.

NaN payloads: when both operands of an operation are NaN, clang may commute it
and return the other payload, so C at `-O2` is not source exact there. Pass
`{ nanEqual: true }` to `runModule` to let any NaN match any NaN; the summary
counts those lines. Leave it off unless a mismatch shows only NaN tokens.

## Port from box3d-as, not from scratch

Box3D was forked from Box2D v3, and Box2D v3.2.0 still shares most of its
structure: the world, bodies, shapes, contacts, islands, solver sets, the
constraint graph, the solver stages, the soft step contact solver, the dynamic
tree, the broad phase, the hash set, the id pool, the bitset, sensors, the
joints and the mover. Our Box3D port (`~/Projects/box3d-rive-as`, branch
`main`) has already solved how each of those maps onto AnimaScript and how to
make it fast. For each file:

1. Diff `~/Projects/box2d/src/<f>.c` against `~/Projects/box3d/src/<f>.c`
   (Box3D pinned at `f36d061`).
2. Start from `~/Projects/box3d-rive-as/lib/<f>.as` and apply the same
   differences, in 2D: `b2Vec2`/`b2Rot` for `b3Vec3`/`b3Quat`, a scalar
   angular velocity and inertia for the 3D ones, Box2D's own shapes.
3. Where Box2D has code Box3D lacks (chain shapes, `b2Rot` math, polygon
   radius, the 2D manifold functions, joint specifics, surface materials),
   port it from C following the same conventions.

Never copy Box3D behaviour that Box2D's C does not have. The parity gates are
against Box2D's C, not Box3D's.

## Runtime contract: a game steps this every frame

- No allocation per step once a scene has settled. Per step buffers live in a
  module scratch that only grows (box3d-as `b3SolverScratch`,
  `count + count / 4 + 16`). Under `--runtime frame` the heap must not grow
  from step to step of a settled scene; check pages after 20 and 200 steps.
- Contacts, manifolds, sims and states are structs in arrays, never objects
  per contact. Hot loops read struct arrays through raw addresses
  (`load<f32>(base + offsetof<T>("field"))`) where box3d-as does, with the
  range argument written once per file as box3d-as notes/perf_solver.md does.
- The world, its arrays and every closure the game registers stay reachable
  from module globals or the game's objects, so a frame boundary collection
  keeps them.

## Performance rules from box3d-as

These were each measured there (`notes/perf_*.md` in box3d-rive-as); apply them
up front instead of rediscovering them.

- One kernel source for both artifacts: the contact solver kernels are generic
  over a lane type `T` (`v128` under `ASC_FEATURE_SIMD`, `f32` without), with
  `isFloat<T>()` picking each helper's body, dispatched by
  `if (ASC_FEATURE_SIMD)`, never a constant ternary (rasc compiles both arms).
- Exact v128 forms only: mul then add (no fma), compare plus bitselect for
  min/max/abs and blends, masks all ones in a kernel, stored as
  `and(mask, 1.0)`.
- Gather and scatter body states with C style unpack zips (`0,4,1,5` shuffles),
  not `0,4,0,4`, which V8 arm64 has no single op for.
- No `FixedArray` lane locals in kernels (rasc lowers their indexing to jump
  tables); unroll the four lanes.
- Helpers that would return several values are written per component so each
  returns one value and inlines under `-O`.
- Store impulses lane by lane through raw `load<f32>` at field plus `4 * lane`;
  the v128 form of it was slower.
- `unchecked()` only where the index is proven in range, with the proof a line
  above it.
- Tree queries keep a stack per call depth (box3d-as `lane_tree` / perf_4
  change 7), so a callback may query again.
- No `Mathf.sin`/`cos`/`atan2`; Box2D's `b2ComputeCosSin` and `b2Atan2`.
- Joints split prepare, warm start and solve as C does, and read sims in place.
- WAMR's fast interpreter traps on v128 locals past frame cell 256: a SIMD
  artifact needs AOT. The scalar artifact must stay good on the interpreter.

## Gates

Every change keeps all of these bit exact, with `RASC_FLAGS="-O"` and
`RASC_FLAGS="-O --enable-simd"`:

- `verify/run.sh`: per module checks against the scalar C build.
- `verify/traj.sh`: every body's transform and velocity and the world counters,
  every step, of each scene in `verify/scenes`, against a native recording
  (`NATIVE=1 verify/traj.sh` records them; recordings are gitignored).
