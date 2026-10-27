# tree: aabb, dynamic tree, broad phase (wave 2)

## Ported

| C | AnimaScript |
|---|---|
| `src/aabb.h`, `src/aabb.c` | `lib/aabb.as`: `b2Perimeter`, `b2EnlargeAABB`, `b2OffsetAABB`, `b2IsValidAABB`, `b2AABB_RayCast` |
| `src/dynamic_tree.h`, `src/dynamic_tree.c` | `lib/dynamic_tree.as`, everything but validation in release (see below) and the SAH partition C compiles out (`B2_TREE_HEURISTIC 0`) |
| `src/broad_phase.h`, the world independent part of `src/broad_phase.c` | `lib/broad_phase_core.as` |
| the parts of `src/broad_phase.c` that read the world | `lib/broad_phase.as`, which also re-exports `broad_phase_core` |

Started from box3d-as `lib/dynamic_tree.as` and `lib/broad_phase.as` with the box2d/box3d C diff
applied. Box2D's tree differs from Box3D's: no tree view, no closest query, no `requireAllBits`, an
`OverlapAABBAll`, `b2CreateDynamicTreeInternal` filling a tree in place, a box cast that clips with
`<` where the ray cast clips with `<=`, a rebuild that always clears the moved flags, and a pair
search that walks children by overlap and moved masks (`b2OverlapPairMask`, `b2CTZ32`).

## Why broad_phase is two files

rasc resolves every import of every reachable file, with named exports, even when nothing used comes
from it (unreachable function bodies are not checked). `b2UpdateBroadPhasePairs` needs
`physics_world`, `body`, `contact`, `shape` and `parallel_for`, whose import closure does not
compile until the world wave lands, so the parity driver could not import a single broad phase
file. `broad_phase_core.as` holds everything C's file has that does not read the world;
`broad_phase.as` holds the rest and `export * from "./broad_phase_core"`, so callers import C's
names from `./broad_phase` as in C. `b2UpdateBroadPhasePairs` was type checked and run against
stub world files (three proxies, three pairs, task count 3).

## Signatures that differ from C

```as
b2CreateDynamicTreeInternal(tree: b2DynamicTree, proxyCapacity)   // fills the object, as C's memset + init
b2CreateDynamicTree(proxyCapacity): b2DynamicTree
b2DestroyDynamicTree(tree: b2DynamicTree?)

// V is the C callback type (closure or top level function) or a class with a visit method, which
// is called directly; context is dropped either way
b2DynamicTree_OverlapAABB<V>(tree, aabb, maskBits: u64, callback: V): b2TreeStats     // visit(proxyId, userData): bool
b2DynamicTree_OverlapAABBAll<V>(tree, aabb, callback: V): b2TreeStats
b2DynamicTree_CastRay<V>(tree, input: b2RayCastInput, maskBits, callback: V): b2TreeStats   // visit(input, proxyId, userData): f32
b2DynamicTree_CastBox<V>(tree, input: b2BoxCastInput, maskBits, callback: V): b2TreeStats   // same shape with b2BoxCastInput
b2DynamicTree_GatherMovedProxies<A>(tree, proxyIds: A): i32       // any i32 array with room
b2DynamicTree_GetByteCount(tree): i32                             // C size_t; C's element sizes, sizeof(b2DynamicTree) 104

b2CreateBroadPhase(bp, capacity: b2Capacity)                      // by value
b2GatherCrossSeeds(treeA, treeB, seeds: StaticArray<b2NodePair>, seedBase): i32
b2SelfPairsTask<F>(startIndex, endIndex, bp, pairContext: b2PairContext<F>)
b2CrossPairsTask<F>(startIndex, endIndex, bp, seeds, staticSeedCount, pairContext: b2PairContext<F>)
b2RadixSortKeys(keys: StaticArray<u64>, tempKeys: StaticArray<u64>, count)
```

- `b2PairContext<F>` holds C's batch plus its stack buffers and, where C holds the world, a filter
  `F` with `shouldCreatePair(shapeIdA, shapeIdB): bool`, the pair set and the worker's pair keys.
  `broad_phase.as` has `b2WorldPairFilter`, C's same body, sensor, filter, `b2CanCollide`, joint and
  custom filter tests verbatim. One context per worker lives in a module array and is reused.
- The world tasks go through `b2ParallelFor<C>` with top level functions, as notes/world.md asks.
  `b2EnqueueTreeUpdate` runs the rebuild inline and counts the task, which is what C's default
  enqueue does (it returns no task, so `activeTaskCount` and `userTreeTask` stay untouched); the
  world has no enqueue callback fields.
- `b2ValidateSolverSets` at the end of `b2UpdateBroadPhasePairs` is not called (validation).
- `b2PrefetchHash` and the shape prefetches are cache hints, not ported.
- `b2BroadPhase_TestOverlap` and `b2BroadPhase_GetShapeIndex` do not exist in v3.2.0.
- Not allocating per step: the query stacks are one per nesting depth (box3d-as perf_4 change 7),
  so a callback may query again; the rebuild, copy, walk and refit stacks are module statics (they
  never call out); the moved siblings, the pair keys and the sort scratch are `b2BroadPhase`
  buffers that grow by `count + count / 4 + 16`; the cross seeds and search queue are module
  statics; the radix digit counts are a module static.
- The `b2TreeNode` union is two fields. Every node is built whole by `b2MakeLeafNode`,
  `b2MakeInternalNode` or `b2MakeEmptyNode`, so the unused one stays zero, and C's union value is
  `isLeaf ? shapeIndex : height`. Code outside the tree that writes nodes must keep that.
- `b2DynamicTree_MarkProxyMoved` is single threaded: the atomic load and fetch-or are a read and an
  or.
- Validation: `-D B2_VALIDATE_TREE` with `-D B2_ENABLE_ASSERT` turns on `b2DynamicTree_Validate` and
  `ValidateNoMoved` (C builds them out); `verify/as/tree.flags` sets both, so every rebuild in the
  parity run validates the whole tree, including the union rule.

## Verification

`verify/run.sh tree`: **130856 lines, 716744 values, 0 mismatches** with `RASC_FLAGS` `-O`,
`-O --enable-simd` and `""`, and by hand without `tree.flags` at `-O` and debug. 9 lines (the
`b2OffsetAABB` sweep over special floats) differ only in NaN payload: it adds in double, where wasm
returns the canonical NaN and arm64 keeps the payload; `tree.mjs` passes `nanEqual`.

- aabb: 4000 lines of perimeter, validity, enlarge and offset over special floats; 6000
  `b2AABB_RayCast` lines with axis aligned and zero rays.
- tree: 18 seeded rounds over six box kinds (uniform, integer grid with many equal boxes, points
  and flat boxes, far coordinates near 1e5, identical boxes, a line), capacities 0 to 100, 400 or
  900 ops each: create (public and internal, marked or not), destroy, move (both), enlarge, mark,
  serial mark, set category, refit, partial and full rebuild, clear, gather, and queries. An FNV
  hash of the whole tree after every op (nodes to `nodeEnd` with the union value, parents, every
  proxy, counters), a full dump at each round end, then refit, rebuilds and a drain to empty.
- queries in callback order, each hit with its input `maxFraction` and the returned value:
  overlap with masks, overlap all, ray and box casts with record, clip (a real AABB cast, so the
  fraction shrinks), terminate after k hits, skip odd ids (-1), and nested overlap queries inside
  overlap, ray and box callbacks up to depth 3. Every query randomly runs through a closure or a
  visitor, so both dispatch paths are covered.
- broad phase: 8 rounds of 30 or 60 updates. Between updates: proxies of all three types created
  (static with and without `forcePairCreation`), destroyed (their pairs leave the pair set),
  moved, marked, enlarged, serial marked, contacts ended, then the solver's refit. Each update
  prints the moved siblings, the static and kinematic seeds, the pair keys in creation order, the
  rebuild leaf counts, the sorted keys, and the three tree hashes. The C side `#include`s
  `broad_phase.c` and runs its static tasks against a fake world whose shapes come in pairs on one
  body; the AnimaScript side runs the same tasks with a filter that rejects the same body pairs.
- `b2RadixSortKeys` on random, pair, few distinct and all equal keys, 2 to 4999 of them.

Planted bugs (the closer child first order of the box cast, the moved mask of the cross pair search) both show.

## rasc findings

### Imports resolve eagerly, bodies lazily

```as
// m.as
import { foo } from "./missing";      // error: cannot resolve './missing', though nothing calls b
export function a(): i32 { return 1; }
export function b(): i32 { return foo(); }
```

An existing import with a broken body that nothing reaches compiles. This is why broad_phase is two
files; a lazy import (resolve on first use) would let one file serve both.
