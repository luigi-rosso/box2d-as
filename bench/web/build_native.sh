#!/bin/sh
# Builds out/bin/native_simd and native_scalar: the runner's twin against the native NEON and scalar Box2D builds
set -e
here="$(cd "$(dirname "$0")" && pwd)"
BOX2D=${BOX2D:-$HOME/Projects/box2d}
scenes="$here/../../verify/scenes"
mkdir -p "$here/out/bin"
for flavor in release scalar; do
	name=native_scalar
	[ "$flavor" = release ] && name=native_simd
	cmake --build "$BOX2D/build/$flavor" --target box2d > /dev/null
	cc -std=c17 -O3 -DNDEBUG -ffp-contract=off -Wall -Wextra -I "$BOX2D/include" -I "$scenes" "$here/c/native.c" \
		"$here/c/driver.c" "$scenes/scenes.c" "$BOX2D/build/$flavor/src/libbox2d.a" -lpthread -lm -o "$here/out/bin/$name"
done
