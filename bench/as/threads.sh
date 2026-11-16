#!/bin/sh
# Step timings on parallel calls, interleaved with native Box2D at the same worker counts. Each round runs, scene by
# scene, native SIMD at every count, the serial port, then the port on parallel calls at every part count, recording
# uptime before each run since the machine may be shared; compare min over rounds. Every run builds and steps its
# scene REPEATS times and keeps the best. AOT, SIMD artifact.
#   ROUNDS=3 REPEATS=3 bench/as/threads.sh
#   COUNTS="1 4" TASKS=1 SCENES="large_pyramid:100:12" bench/as/threads.sh
# n parts is --job-threads=n-1, one part --parallel-safe. TASKS picks lib/parallel_step's task bits. A scene is
# name:steps:index, the index main.as lists. Writes out/threads_<timestamp>.log and prints the tables.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
. "$HERE/../../verify/toolchain.sh"
ROUNDS=${ROUNDS:-3}
REPEATS=${REPEATS:-3}
COUNTS=${COUNTS:-1 2 4 8}
TASKS=${TASKS:-15}
SCENES=${SCENES:-large_pyramid:100:12 many_bodies:100:11 joint_grid:100:13 rain:360:14 smash:120:15 spinner:300:16 tumbler:200:17}
NATIVE=$HERE/../native/bin/bench_simd
CACHE=${RIVE_AOT_CACHE_DIR:-$HERE/out/aotcache}
mkdir -p "$HERE/out" "$CACHE"
"$HERE/../native/build.sh"
"$HERE/sync.sh"
LOG="$HERE/out/threads_$(date +%Y%m%d_%H%M%S).log"

# as_run <config> <parts> <steps> <scene index> <tasks>
as_run() {
	echo "UPTIME $(uptime)"
	echo "AS config=$1 parts=$2"
	threads=--parallel-safe
	[ "$2" -gt 1 ] && threads=--job-threads=$(($2 - 1))
	echo "SCENE=$4 STEPS=$3 PARALLEL=$5 REPEATS=$REPEATS" > "$HERE/options.txt"
	(cd "$HERE" && RIVE_WAMRC="$RIVE_WAMRC" RIVE_WASM_AOT_SYNC=o3 RIVE_AOT_CACHE_DIR="$CACHE" \
		"$RIVE" . --aot --bench=$((REPEATS * $3 + 10)) $threads 2>&1) |
		grep -E "script tier|BENCH RESULT|trapped|failed|gave up" | grep -v "warning"
}

{
	echo "MACHINE $(sysctl -n machdep.cpu.brand_string) cores $(sysctl -n hw.ncpu)"
	round=1
	while [ $round -le "$ROUNDS" ]; do
		echo "ROUND $round"
		for entry in $SCENES; do
			scene=$(echo "$entry" | cut -d: -f1)
			steps=$(echo "$entry" | cut -d: -f2)
			index=$(echo "$entry" | cut -d: -f3)
			for n in $COUNTS; do
				echo "UPTIME $(uptime)"
				"$NATIVE" "$scene" "$steps" "$n" "$REPEATS" | grep "^RESULT"
			done
			as_run serial 1 "$steps" "$index" 0
			for n in $COUNTS; do
				as_run parallel "$n" "$steps" "$index" "$TASKS"
			done
		done
		round=$((round + 1))
	done
} | tee "$LOG"

python3 "$HERE/threads_table.py" "$LOG"
