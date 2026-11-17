#!/bin/bash
# Threaded trajectory parity: runs the trajectory scenes in rive-cli with the step on parallel calls at each part
# count and compares every step's state hash with the native scalar recording in verify/scenes/out (verify/traj.sh
# records them with NATIVE=1). node cannot run parallel calls, so this is the gate for threaded runs; verify/traj.sh
# stays the gate for the serial port.
#   verify/threads.sh                 every scene but the benchmark sized ones
#   verify/threads.sh --large         all scenes
#   verify/threads.sh rain spinner    only those
# PARTS="1 2 4 8 default" picks the part counts: 1 is --parallel-safe, n is --job-threads=n-1, default the CLI's pool.
# TASKS picks lib/parallel_step's task bits, every task by default. TIER=interp runs without wamrc. WASM_SIMD=off
# takes the scalar artifact. CALLBACKS=63 sets bench/as's callbacks, any of its mask's bits, and holds each run to the
# serial port's run with them instead; a list of masks runs the scenes once per mask. A scene whose serial run calls
# none of them fails, as joint_grid does, which has no contacts. All at once hides a task that forgot one of several
# callbacks that keep it off the parts, so the gate is two commands:
#   CALLBACKS=63 verify/threads.sh
#   CALLBACKS="33 34 36 40 48" PARTS="4 8" verify/threads.sh shape_rain bullets    each callback alone, where it is called
# A run gets TIMEOUT seconds, bake included, 1800 by default. RIVE and RIVE_WAMRC name the CLI and its wamrc. PROJECT
# names a copy of bench/as to run as it is, which is how verify/parts.sh runs its edited ones.
here="$(cd "$(dirname "$0")" && pwd)"
source "$here/toolchain.sh"
project=${PROJECT:-$here/../bench/as}
parts=${PARTS:-1 2 4 8 default}
tasks=${TASKS:-15}
tier=${TIER:-aot}
timeout=${TIMEOUT:-1800}

scenes=()
large=0
for a in "$@"; do
	if [ "$a" = "--large" ]; then large=1; else scenes+=("$a"); fi
done
if [ ${#scenes[@]} -eq 0 ]; then
	scenes=(pyramid shape_rain chain_car joints dynamic_mover churn bullets sensors events queries mover)
	[ $large -eq 1 ] && scenes+=(many_bodies large_pyramid joint_grid rain smash spinner tumbler)
fi

# The scene's index in verify/scenes, as bench/as/main.as lists them
scene_define() {
	case $1 in
		pyramid) echo 0 ;; shape_rain) echo 1 ;; chain_car) echo 2 ;; joints) echo 3 ;; dynamic_mover) echo 4 ;;
		churn) echo 5 ;; bullets) echo 6 ;; sensors) echo 7 ;; events) echo 8 ;; queries) echo 9 ;; mover) echo 10 ;;
		many_bodies) echo 11 ;; large_pyramid) echo 12 ;; joint_grid) echo 13 ;; rain) echo 14 ;; smash) echo 15 ;;
		spinner) echo 16 ;; tumbler) echo 17 ;;
	esac
}

[ -n "$PROJECT" ] || "$project/sync.sh"
run_dir="$project"
if [ "${WASM_SIMD:-on}" = off ]; then
	# The scalar artifact bakes from a copy with wasmSimd: off, outside the project so the CLI does not compile it
	run_dir=${TMPDIR:-/tmp}/box2d_threads_scalar_$(echo "$project" | md5 | cut -c1-8)
	mkdir -p "$run_dir"
	rsync -a --delete --exclude out --exclude build "$project/" "$run_dir/"
	echo "wasmSimd: off" >> "$run_dir/rive.yaml"
fi
cache="$project/out/aotcache"
mkdir -p "$cache"
log=$(mktemp)
serial=$(mktemp)

# run_bench <scene> <steps> <tasks> [thread flag]: one run of the bake, which the alarm ends and names if it hangs
run_bench() (
	cd "$run_dir"
	# In the options blob, so every run shares one compiled bake
	echo "SCENE=$(scene_define "$1") STEPS=$2 PARALLEL=$3 CALLBACKS=$callbacks HASHLOG=1" > options.txt
	args=("$RIVE" . --bench=$(($2 + 10)) $4)
	if [ "$tier" = aot ]; then
		RIVE_WAMRC="$RIVE_WAMRC" RIVE_WASM_AOT_SYNC=o3 RIVE_AOT_CACHE_DIR="$cache" perl -e 'alarm shift; exec @ARGV' "$timeout" "${args[@]}" --aot
	else
		env -u RIVE_WAMRC perl -e 'alarm shift; exec @ARGV' "$timeout" "${args[@]}"
	fi
	[ $? -ne 142 ] || echo "the run timed out after $timeout s"
)

results=()
failed=0
for callbacks in ${CALLBACKS:-0}; do
	tag="T$tasks"
	[ "$callbacks" = 0 ] || tag="${tag}C$callbacks"
	for s in "${scenes[@]}"; do
		ref="$here/scenes/out/${s}_scalar_w1.bin"
		if [ ! -f "$ref" ]; then
			echo "missing $ref; record it with NATIVE=1 verify/traj.sh $s"
			failed=$((failed + 1))
			continue
		fi
		steps=$(node "$here/threads.mjs" "$ref" --steps)
		if [ "$callbacks" != 0 ]; then
			run_bench "$s" "$steps" 0 --parallel-safe > "$serial" 2>&1
			ref=$serial
		fi
		for p in $parts; do
			case $p in
				1) threads=--parallel-safe ;;
				default) threads= ;;
				*) threads=--job-threads=$((p - 1)) ;;
			esac
			run_bench "$s" "$steps" "$tasks" $threads > "$log" 2>&1
			r=$(node "$here/threads.mjs" "$ref" "$log" "$s P$p $tag" "$p" "$tasks")
			status=$?
			echo "$r"
			results+=("$r")
			[ $status -eq 0 ] || failed=$((failed + 1))
		done
	done
done
rm -f "$log" "$serial"

echo
printf "%-26s %6s  %-5s %s\n" run steps pass "first divergence"
for r in "${results[@]}"; do
	set -- $r
	label="$2 $3 $4"; steps=$5; verdict=$6; shift 6
	printf "%-26s %6s  %-5s %s\n" "$label" "$steps" "$verdict" "$*"
done
echo "passed: $(( ${#results[@]} - failed ))  failed: $failed"
[ $failed -eq 0 ]
