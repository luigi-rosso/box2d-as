#!/bin/bash
# End to end trajectory parity: builds verify/scenes/traj.as, runs each scene and diffs it against
# the native scalar recording in verify/scenes/out (1 worker).
#   verify/traj.sh            every scene
#   verify/traj.sh pyramid    only those
# RASC_FLAGS replaces -O. WASM sets the driver path. NATIVE=1 re-records the native references first.
# The driver imports lib/box2d.as, so this runs once the world is ported.

here="$(cd "$(dirname "$0")" && pwd)"
source "$here/toolchain.sh"
RASC_FLAGS=${RASC_FLAGS--O}
out="$here/scenes/out"
mkdir -p "$out/as"
wasm=${WASM:-$out/as/traj.wasm}

scenes=("$@")
if [ ${#scenes[@]} -eq 0 ]; then
	scenes=(pyramid shape_rain chain_car joints dynamic_mover churn bullets sensors events queries mover many_bodies)
fi

# Steps each scene records; long enough for its script to finish and its bodies to settle
steps_for() {
	case "$1" in
		pyramid) echo 300 ;;
		shape_rain) echo 1000 ;;
		chain_car) echo 720 ;;
		joints) echo 500 ;;
		dynamic_mover) echo 900 ;;
		churn) echo 800 ;;
		bullets) echo 400 ;;
		sensors) echo 600 ;;
		events) echo 600 ;;
		queries) echo 300 ;;
		mover) echo 900 ;;
		many_bodies) echo 100 ;;
		*) echo 300 ;;
	esac
}

if [ "${NATIVE:-0}" = "1" ]; then
	"$here/scenes/build.sh" || exit 2
	for s in "${scenes[@]}"; do
		steps=$(steps_for "$s")
		"$here/scenes/bin/traj_scalar" "$s" $steps 1 "$out/${s}_scalar_w1.bin" > /dev/null || exit 2
		"$here/scenes/bin/traj_simd" "$s" $steps 1 "$out/${s}_simd_w1.bin" > /dev/null || exit 2
	done
fi

if [ ! -f "$here/../lib/box2d.as" ]; then
	echo "lib/box2d.as is not ported yet; native recordings are ready in $out"
	exit 2
fi

if ! "$RASC" "$here/scenes/traj.as" --path "$AS_STD" --lib "$AS_STD/assembly" --runtime frame \
		$RASC_FLAGS -o "$wasm"; then
	echo "rasc failed"
	exit 2
fi

results=()
failed=0
for s in "${scenes[@]}"; do
	echo "== $s"
	log=$(node "$here/traj.mjs" "$s" --wasm="$wasm" 2>&1)
	status=$?
	echo "$log" | grep -v '^RESULT '
	r=$(echo "$log" | grep '^RESULT ' | tail -1)
	[ -z "$r" ] && r="RESULT $s 0 fail node-exited-$status"
	results+=("$r")
	[ $status -eq 0 ] || failed=$((failed + 1))
done

echo
printf "%-14s %6s  %-5s %s\n" scene steps pass "first divergence"
for r in "${results[@]}"; do
	set -- $r
	name=$2; steps=$3; verdict=$4; shift 4
	printf "%-14s %6s  %-5s %s\n" "$name" "$steps" "$verdict" "$*"
done
echo "passed: $(( ${#results[@]} - failed ))  failed: $failed"
[ $failed -eq 0 ]
