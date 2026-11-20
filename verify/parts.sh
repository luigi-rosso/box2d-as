#!/bin/bash
# What verify/threads.sh cannot see, each held on a copy of bench/as with lib/solver.as edited:
#   workers  part 0 runs no block of a stage it published, so a step ends only if the others can run every block
#   reads    part 0 reads the stop flag at every poll of a stage wait, not once in 65536, so healthy steps run it
#   trap     every worker traps inside a block from step 20 on, and the call has to end with an error, not hang
#   trap0    part 0 traps after it published a stage in step 20, the same
# workers and reads must stay bit exact against the native recording at 2, 4 and 8 parts (PARTS, none of them 1).
#   verify/parts.sh [scene...]    pyramid and large_pyramid by default
# A scene needs stages of more than one block, as those two have: a small one publishes none, so it reaches no
# trap. A hang fails at verify/threads.sh's TIMEOUT. It runs every task on the SIMD artifact and on AOT: the
# interpreter's host ends the workers of a failed call itself, so trap0 would pass there without their read. It does
# not hold speed: workers that leave the work to part 0 in an unedited build pass, and so does a trap that takes
# minutes to end the call.
here="$(cd "$(dirname "$0")" && pwd)"
project="$here/../bench/as"
# The caller's would take the runs off the solver's task, the native recording, the SIMD artifact or AOT
unset TASKS CALLBACKS WASM_SIMD TIER
for p in $PARTS; do
	if [ "$p" = 1 ]; then
		echo "PARTS cannot hold 1: with part 0 running no block, a step on one part never ends" >&2
		exit 2
	fi
done
scenes=("$@")
[ ${#scenes[@]} -gt 0 ] || scenes=(pyramid large_pyramid)
"$project/sync.sh"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# variant <name>: the copy with that edit, refused when solver.as no longer has the lines it replaces
variant() {
	rsync -a --exclude out --exclude build "$project/" "$work/$1/"
	python3 - "$work/$1/lib/solver.as" "$1" <<'PY'
import sys
path, name = sys.argv[1:]
trap = '''if (Parallel.inPart() && %s) {
				unreachable();
			}
'''
edits = {
	'workers': [('''		b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		// Spin waiting for thieves to finish
''', '''		if (Parallel.inPart() == false) {
			b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);
		}

		// Spin waiting for thieves to finish
''')],
	'reads': [('const B2_STAGE_STOP_POLLS: i32 = 1 << 16;', 'const B2_STAGE_STOP_POLLS: i32 = 1;')],
	'trap': [('''			b2ExecuteBlock(stage, context, syncBlock.block, workerIndex);
			completedCount += 1;
''', '			' + trap % 'workerIndex != 0 && context.world.stepIndex >= 20' + '''			b2ExecuteBlock(stage, context, syncBlock.block, workerIndex);
			completedCount += 1;
''')],
	'trap0': [('''		b2PublishSyncBits(context, syncBits);

		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
''', '''		b2PublishSyncBits(context, syncBits);
		''' + trap.replace('\t\t\t', '\t\t') % 'context.world.stepIndex == 20' + '''
		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
''')],
}
source = open(path).read()
for old, new in edits[name]:
	if source.count(old) != 1:
		sys.exit(name + ': lib/solver.as no longer has\n' + old)
	source = source.replace(old, new)
open(path, 'w').write(source)
PY
}

failed=0
for v in workers reads; do
	variant $v || exit 2
	PROJECT="$work/$v" PARTS="${PARTS:-2 4 8}" "$here/threads.sh" "${scenes[@]}" > "$work/$v.out" || failed=$((failed + 1))
	grep "^RESULT" "$work/$v.out" | sed "s/^RESULT/$v/"
done

for v in trap trap0; do
	variant $v || exit 2
	r=$(PROJECT="$work/$v" PARTS=4 "$here/threads.sh" "${scenes[0]}" | grep "^RESULT")
	case "$r" in
		*trapped*) echo "$v pass: the call ended with the trap" ;;
		*) echo "$v fail: ${r#RESULT }"; failed=$((failed + 1)) ;;
	esac
done
echo "failed: $failed"
[ $failed -eq 0 ]
