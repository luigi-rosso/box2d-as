#!/bin/bash
# Every scene on every implementation, each step's state hash against the native scalar recording
here="$(cd "$(dirname "$0")" && pwd)"
IMPLS=${IMPLS:-native_scalar native_simd c_scalar c_simd as_scalar as_simd}
WORKERS=${WORKERS:-1}
# The recording's step count, as verify/traj.sh records it
steps_of() {
	case $1 in
		shape_rain) echo 1000 ;; chain_car) echo 720 ;; joints) echo 500 ;; dynamic_mover | mover) echo 900 ;;
		churn) echo 800 ;; bullets) echo 400 ;; sensors | events) echo 600 ;; many_bodies | large_pyramid | joint_grid) echo 100 ;;
		rain) echo 360 ;; smash) echo 120 ;; tumbler) echo 200 ;; *) echo 300 ;;
	esac
}
failed=0
for s in ${SCENES:-pyramid shape_rain chain_car joints dynamic_mover churn bullets sensors events queries mover many_bodies large_pyramid joint_grid rain smash spinner tumbler}; do
	for i in $IMPLS; do
		for w in $WORKERS; do
			line=$(node "$here/run.mjs" "$i" "$s" "$(steps_of "$s")" --workers="$w" 2>&1 | tail -1)
			echo "$line" | python3 -c "
import json,sys
l=sys.stdin.read()
try:
    r=json.loads(l)
    print(f\"{r['scene']:14} {r['impl']:14} w{r['workers']} compared {r['compared']:4} {'pass' if r['mismatch']<0 and r['compared']>0 else 'FAIL at step '+str(r['mismatch'])} final {r['finalHash']:08x}\")
    sys.exit(0 if r['mismatch']<0 and r['compared']>0 else 1)
except (ValueError, KeyError):
    print('$s $i w$w ERROR', l[:300]); sys.exit(1)
" || failed=$((failed + 1))
		done
	done
done
echo "errors: $failed"
[ $failed -eq 0 ]
