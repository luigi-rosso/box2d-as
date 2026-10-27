#!/bin/sh
# Builds the trajectory recorder against the scalar parity build and the NEON build of Box2D
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
BOX2D=${BOX2D:-$HOME/Projects/box2d}
mkdir -p "$HERE/bin"
for flavor in scalar release; do
	cmake --build "$BOX2D/build/$flavor" --target box2d > /dev/null
	name=$flavor
	[ "$flavor" = release ] && name=simd
	cc -std=c17 -O2 -DNDEBUG -ffp-contract=off -Wall -Wextra -I "$BOX2D/include" \
		"$HERE/traj.c" "$HERE/scenes.c" "$BOX2D/build/$flavor/src/libbox2d.a" -lpthread -lm \
		-o "$HERE/bin/traj_$name"
done
