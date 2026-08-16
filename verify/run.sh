#!/bin/zsh
# Phase A+B+C differential verification: compiles the C reference generators
# against the canonical box2d sources, regenerates the vectors, compiles the
# AS drivers with rasc (debug and -O2), and runs the node comparison against
# both builds.
set -e
cd "$(dirname "$0")"
BOX2D_SRC=${BOX2D_SRC:-$HOME/Projects/box2d}
RASC=${RASC:-$HOME/Projects/rive-fields_as/packages/assemblyscript/out/release/rasc}
AS_STD=${AS_STD:-$HOME/Projects/rive-fields_as/packages/assemblyscript/std}

# Pinned determinism flags (see README.md): scalar SIMD path, no fp
# contraction so the reference matches wasm float semantics exactly.
CFLAGS=(-O2 -ffp-contract=off -DNDEBUG -DBOX2D_DISABLE_SIMD -w
        -I "$BOX2D_SRC/include" -I "$BOX2D_SRC/src" -I cppref)

clang "${CFLAGS[@]}" cppref/ref_math.c \
    "$BOX2D_SRC/src/math_functions.c" "$BOX2D_SRC/src/core.c" \
    -o cppref/ref_math
clang "${CFLAGS[@]}" cppref/ref_state.c \
    "$BOX2D_SRC/src/core.c" "$BOX2D_SRC/src/array.c" "$BOX2D_SRC/src/bitset.c" \
    "$BOX2D_SRC/src/id_pool.c" "$BOX2D_SRC/src/arena_allocator.c" "$BOX2D_SRC/src/table.c" \
    "$BOX2D_SRC/src/math_functions.c" \
    -o cppref/ref_state
clang "${CFLAGS[@]}" cppref/ref_geom.c \
    "$BOX2D_SRC/src/math_functions.c" "$BOX2D_SRC/src/core.c" "$BOX2D_SRC/src/hull.c" \
    "$BOX2D_SRC/src/geometry.c" "$BOX2D_SRC/src/aabb.c" "$BOX2D_SRC/src/distance.c" \
    -o cppref/ref_geom

clang "${CFLAGS[@]}" cppref/ref_gjk.c \
    "$BOX2D_SRC/src/math_functions.c" "$BOX2D_SRC/src/core.c" "$BOX2D_SRC/src/hull.c" \
    "$BOX2D_SRC/src/geometry.c" "$BOX2D_SRC/src/aabb.c" "$BOX2D_SRC/src/distance.c" \
    -o cppref/ref_gjk
clang "${CFLAGS[@]}" cppref/ref_manifold.c \
    "$BOX2D_SRC/src/math_functions.c" "$BOX2D_SRC/src/core.c" "$BOX2D_SRC/src/hull.c" \
    "$BOX2D_SRC/src/geometry.c" "$BOX2D_SRC/src/aabb.c" "$BOX2D_SRC/src/distance.c" \
    "$BOX2D_SRC/src/manifold.c" \
    -o cppref/ref_manifold
# ref_tree.c #includes dynamic_tree.c for node internals; do not link it twice.
clang "${CFLAGS[@]}" cppref/ref_tree.c \
    "$BOX2D_SRC/src/math_functions.c" "$BOX2D_SRC/src/core.c" "$BOX2D_SRC/src/aabb.c" \
    "$BOX2D_SRC/src/array.c" "$BOX2D_SRC/src/table.c" "$BOX2D_SRC/src/bitset.c" \
    -o cppref/ref_tree
# Phase C steps whole worlds, so it links the full library.
clang "${CFLAGS[@]}" cppref/ref_dynamics.c "$BOX2D_SRC"/src/*.c -o cppref/ref_dynamics
clang "${CFLAGS[@]}" cppref/ref_bench.c "$BOX2D_SRC"/src/*.c -o cppref/ref_bench

./cppref/ref_math > vectors/math_vectors.txt
./cppref/ref_state > vectors/state_vectors.txt
./cppref/ref_geom > vectors/geom_vectors.txt
./cppref/ref_gjk > vectors/gjk_vectors.txt
./cppref/ref_manifold > vectors/manifold_vectors.txt
./cppref/ref_tree > vectors/tree_vectors.txt
./cppref/ref_dynamics > vectors/dynamics_vectors.txt
# Bench reference checksum; the bake's BOX2D-BENCH bits line must match.
./cppref/ref_bench

"$RASC" ../_verifyA.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -o verifyA.wasm
"$RASC" ../_verifyB.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -o verifyB.wasm
# Phase C needs a collecting runtime: the solver's transient objects would
# exhaust memory under the stub bump allocator.
"$RASC" ../_verifyC.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime minimal -o verifyC.wasm

echo "== debug rasc build =="
node verify.mjs
node verifyC.mjs

# Bench lane modules (not run here; see bench_v8.mjs / bench_cadence.mjs).
"$RASC" ../bench_stub.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -O2 -o bench.wasm
"$RASC" bench_minimal.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime minimal -O2 -o bench_minimal.wasm

"$RASC" ../_verifyA.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -O2 -o verifyA.wasm
"$RASC" ../_verifyB.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -O2 -o verifyB.wasm
"$RASC" ../_verifyC.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime minimal -O2 -o verifyC.wasm

echo "== -O2 rasc build =="
node verify.mjs
node verifyC.mjs

echo "phase A+B+C verification: all green"
