#!/bin/bash
# Builds the four single threaded web modules into bench/web/out:
#   c_simd, c_scalar     native Box2D through emscripten (Release, -O3, wasm SIMD or BOX2D_DISABLE_SIMD)
#   as_simd.wasm, as_scalar.wasm   the port through rasc -O2 (release bake: binaryen -O3 post pass)
# PROFILE=1 adds the named builds below.
# EMSDK names the emsdk root, WASM_OPT the binaryen 133 wasm-opt that rive-cli links; RASC and AS_STD as
# verify/toolchain.sh picks them.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
root="$here/../.."
scratch=/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad
EMSDK=${EMSDK:-$scratch/emsdk}
WASM_OPT=${WASM_OPT:-$scratch/binaryen133/bin/wasm-opt}
source "$root/verify/toolchain.sh"
out="$here/out"
mkdir -p "$out"

source "$EMSDK/emsdk_env.sh" > /dev/null 2>&1

c_build() {
	local name=$1
	shift
	local dir="$out/build_$name"
	emcmake cmake -S "$here/c" -B "$dir" -G Ninja -DCMAKE_BUILD_TYPE=Release "$@" > "$dir.log" 2>&1 ||
		{ cat "$dir.log"; exit 1; }
	cmake --build "$dir" >> "$dir.log" 2>&1 || { tail -30 "$dir.log"; exit 1; }
	cp "$dir/box2d_web.js" "$out/$name.mjs"
	cp "$dir/box2d_web.wasm" "$out/box2d_web_$name.wasm"
	# The glue names its wasm after the target, so each variant points at its own
	sed -i '' -e "s/box2d_web\.wasm/box2d_web_$name.wasm/g" "$out/$name.mjs"
}

c_build c_simd -DBOX2D_DISABLE_SIMD=OFF
c_build c_scalar -DBOX2D_DISABLE_SIMD=ON
# PROFILE=1 adds c_simd_prof and c_scalar_prof, which keep function names for V8 profiles
if [ "${PROFILE:-0}" = "1" ]; then
	c_build c_simd_prof -DBOX2D_DISABLE_SIMD=OFF -DCMAKE_EXE_LINKER_FLAGS=--profiling-funcs
	c_build c_scalar_prof -DBOX2D_DISABLE_SIMD=ON -DCMAKE_EXE_LINKER_FLAGS=--profiling-funcs
fi

# Release bake flags, as rive-cli's --bench: optimized, no fuel (dangerouslyFast), throws stripped, release checks,
# frame collector
as_build() {
	local name=$1
	shift
	RASC_WASM_OPT="$WASM_OPT" "$RASC" "$here/as/web.as" --path "$AS_STD" --lib "$AS_STD/assembly" \
		--runtime frame -O2 --dangerously-fast --no-throw-messages --release "$@" -o "$out/$name.wasm"
}

as_build as_scalar
as_build as_simd --enable-simd
ls -la "$out"/*.wasm
