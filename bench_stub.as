// Stub-runtime iteration protocol: the warmup run lands lazy statics and all
// steady-state growth before the mark, then every timed run rewinds to it.
// The world slot must return to a pre-mark object or the rewind leaves the
// world table dangling.

import { benchRun } from "./bench";
import { b2BenchGetWorldSlot, b2BenchSetWorldSlot, b2World } from "./lib/physics_world";

let s_savedSlot: b2World | null = null;
let s_mark: usize = 0;

export function benchWarm(steps: i32): f64 {
    let saved = b2BenchGetWorldSlot(0);
    s_savedSlot = saved;
    let sum = benchRun(steps);
    b2BenchSetWorldSlot(0, saved);
    s_mark = __mark();
    return sum;
}

export function benchIter(steps: i32): f64 {
    let sum = benchRun(steps);
    b2BenchSetWorldSlot(0, s_savedSlot!);
    __resetTo(s_mark);
    return sum;
}
