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
# takes the scalar artifact. RIVE and RIVE_WAMRC name the CLI and its wamrc.
here="$(cd "$(dirname "$0")" && pwd)"
source "$here/toolchain.sh"
project="$here/../bench/as"
parts=${PARTS:-1 2 4 8 default}
tasks=${TASKS:-15}
tier=${TIER:-aot}

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

"$project/sync.sh"
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

results=()
failed=0
for s in "${scenes[@]}"; do
	ref="$here/scenes/out/${s}_scalar_w1.bin"
	if [ ! -f "$ref" ]; then
		echo "missing $ref; record it with NATIVE=1 verify/traj.sh $s"
		failed=$((failed + 1))
		continue
	fi
	steps=$(node "$here/threads.mjs" "$ref" --steps)
	for p in $parts; do
		case $p in
			1) threads=--parallel-safe ;;
			default) threads= ;;
			*) threads=--job-threads=$((p - 1)) ;;
		esac
		label="$s P$p T$tasks"
		(
			cd "$run_dir"
			# In the options blob, so every run shares one compiled bake
			echo "SCENE=$(scene_define "$s") STEPS=$steps PARALLEL=$tasks HASHLOG=1" > options.txt
			args=(. --bench=$((steps + 10)) $threads)
			if [ "$tier" = aot ]; then
				RIVE_WAMRC="$RIVE_WAMRC" RIVE_WASM_AOT_SYNC=o3 RIVE_AOT_CACHE_DIR="$cache" "$RIVE" "${args[@]}" --aot
			else
				env -u RIVE_WAMRC "$RIVE" "${args[@]}"
			fi
		) > "$log" 2>&1
		r=$(node "$here/threads.mjs" "$ref" "$log" "$label")
		status=$?
		size=$(sed -n 's/.*BENCH parallel size=\([0-9]*\).*/\1/p' "$log" | head -1)
		if [ "$p" != default ] && [ "$size" != "$p" ]; then
			r="RESULT $label 0 fail Parallel.size() is ${size:-missing}"
			status=1
		fi
		echo "$r"
		results+=("$r")
		[ $status -eq 0 ] || failed=$((failed + 1))
	done
done
rm -f "$log"

echo
printf "%-26s %6s  %-5s %s\n" run steps pass "first divergence"
for r in "${results[@]}"; do
	set -- $r
	label="$2 $3 $4"; steps=$5; verdict=$6; shift 6
	printf "%-26s %6s  %-5s %s\n" "$label" "$steps" "$verdict" "$*"
done
echo "passed: $(( ${#results[@]} - failed ))  failed: $failed"
[ $failed -eq 0 ]
