// Collect-cadence sweep for the minimal runtime: run the pyramid bench
// collecting every N steps (plus never/stub baselines run separately) and
// report ms/run best-of-5 per cadence.

import { readFileSync } from 'fs';

const WASM = process.argv[2] ?? new URL('./bench_minimal.wasm', import.meta.url).pathname;
const STEPS = 240, ITERS = 5;
const CADENCES = [1, 4, 16, 64, 0]; // 0 = only between runs

const wasmBytes = readFileSync(WASM);
const imports = new Proxy({}, { get: () => new Proxy({}, { get: () => () => 0 }) });
const module = await WebAssembly.compile(wasmBytes);
const instance = await WebAssembly.instantiate(module, imports);
const e = instance.exports;

function run(cadence) {
    const t0 = performance.now();
    e.benchStart();
    if (cadence > 0) {
        for (let s = 0; s < STEPS; s += cadence) {
            e.benchStep(Math.min(cadence, STEPS - s));
            e.benchCollect();
        }
    } else {
        e.benchStep(STEPS);
    }
    const sum = e.benchFinish();
    const ms = performance.now() - t0;
    e.benchCollect();
    return { ms, sum };
}

let reference = null;
for (const cadence of CADENCES) {
    const times = [];
    let sum = null;
    run(cadence); // warm
    for (let i = 0; i < ITERS; i++) {
        const r = run(cadence);
        times.push(r.ms);
        sum = r.sum;
    }
    if (reference === null) reference = sum;
    if (sum !== reference) throw new Error(`checksum diverged at cadence ${cadence}`);
    const label = cadence > 0 ? `every ${cadence}` : 'between runs only';
    console.log(`cadence ${label}: ${Math.min(...times).toFixed(3)} ms/run best | checksum ${sum}`);
}
