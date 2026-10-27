#!/bin/sh
# Builds bench against the SIMD and scalar builds of Box2D
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
BOX2D=${BOX2D:-$HOME/Projects/box2d}
SCENES="$HERE/../../verify/scenes"
mkdir -p "$HERE/bin"
for flavor in scalar release; do
	cmake --build "$BOX2D/build/$flavor" --target box2d > /dev/null
	name=$flavor
	[ "$flavor" = release ] && name=simd
	cc -std=c17 -O3 -DNDEBUG -ffp-contract=off -Wall -Wextra -I "$BOX2D/include" -I "$SCENES" \
		"$HERE/bench.c" "$SCENES/scenes.c" "$BOX2D/build/$flavor/src/libbox2d.a" -lpthread -lm \
		-o "$HERE/bin/bench_$name"
done
