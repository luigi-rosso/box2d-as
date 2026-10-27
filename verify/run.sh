#!/bin/bash
# Parity checks against the scalar C build.
#   verify/run.sh            every module, stops at the first failure
#   verify/run.sh table simd only those modules, in that order
# A module <m> is verify/cref/<m>.c, verify/as/<m>.as and verify/<m>.mjs.
# Optional verify/cref/<m>.flags and verify/as/<m>.flags add compiler arguments.
# KEEP_GOING=1 runs the rest after a failure. RASC_FLAGS replaces -O.

here="$(cd "$(dirname "$0")" && pwd)"
source "$here/toolchain.sh"
BOX2D=${BOX2D:-$HOME/Projects/box2d}
BOX2D_BUILD=${BOX2D_BUILD:-$BOX2D/build/scalar}
RASC_FLAGS=${RASC_FLAGS--O}
CC=${CC:-clang}
LIBBOX2D="$BOX2D_BUILD/src/libbox2d.a"

if [ ! -f "$LIBBOX2D" ]; then
	echo "missing $LIBBOX2D; configure $BOX2D_BUILD with BOX2D_DISABLE_SIMD=ON -DCMAKE_C_FLAGS=-ffp-contract=off and build it" >&2
	exit 2
fi

cd "$here"
mkdir -p out vectors

modules=("$@")
if [ ${#modules[@]} -eq 0 ]; then
	for c in cref/*.c; do
		m=$(basename "$c" .c)
		if [ -f "as/$m.as" ] && [ -f "$m.mjs" ]; then
			modules+=("$m")
		fi
	done
fi

echo "rasc $RASC, flags '$RASC_FLAGS'"

passed=()
failed=()
summary=()

for m in "${modules[@]}"; do
	echo "== $m"
	cflags=()
	if [ -f "cref/$m.flags" ]; then read -r -a cflags < "cref/$m.flags"; fi
	aflags=()
	if [ -f "as/$m.flags" ]; then read -r -a aflags < "as/$m.flags"; fi

	ok=1
	if ! "$CC" -O2 -ffp-contract=off -DBOX2D_DISABLE_SIMD -DNDEBUG -std=gnu17 -w \
			-I "$BOX2D/include" -I "$BOX2D/src" -I cref "${cflags[@]}" \
			"cref/$m.c" "$LIBBOX2D" -lm -o "out/$m.cref"; then
		echo "$m: C driver failed to build"; ok=0
	elif ! "out/$m.cref" > "vectors/$m.txt"; then
		echo "$m: C driver failed"; ok=0
	elif ! "$RASC" "as/$m.as" --path "$AS_STD" --lib "$AS_STD/assembly" --runtime frame \
			$RASC_FLAGS "${aflags[@]}" -o "out/$m.wasm"; then
		echo "$m: rasc failed"; ok=0
	else
		log=$(node "$m.mjs" 2>&1)
		status=$?
		echo "$log" | grep -v '^RESULT '
		result=$(echo "$log" | grep '^RESULT ' | tail -1)
		[ -n "$result" ] && summary+=("$result")
		[ $status -eq 0 ] || ok=0
	fi

	if [ $ok -eq 1 ]; then
		passed+=("$m")
	else
		failed+=("$m")
		[ "${KEEP_GOING:-0}" = "1" ] || break
	fi
done

echo
echo "summary"
for r in "${summary[@]}"; do
	set -- $r
	printf "  %-24s %10s lines  %s mismatches\n" "$2" "$3" "$4"
done
echo "passed: ${#passed[@]}  failed: ${#failed[@]} ${failed[*]}"
[ ${#failed[@]} -eq 0 ]
