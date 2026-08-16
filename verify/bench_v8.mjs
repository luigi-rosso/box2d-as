// Pyramid bench on V8/node: loads bench.wasm (bench.as compiled by rasc with
// the stub runtime, matching the bake), warmup via benchWarm (lands lazy
// statics and the heap mark), then best-of-5 timed benchIter runs.

import { readFileSync } from 'fs';

const WASM = process.argv[2] ?? new URL('./bench.wasm', import.meta.url).pathname;
const STEPS = 240, ITERS = 5, WARMUP = 5;

const wasmBytes = readFileSync(WASM);
const imports = new Proxy({}, { get: () => new Proxy({}, { get: () => () => 0 }) });

const t0 = performance.now();
const module = await WebAssembly.compile(wasmBytes);
const instance = await WebAssembly.instantiate(module, imports);
const loadMs = performance.now() - t0;
const e = instance.exports;

const checksum = e.benchWarm(STEPS);
for (let i = 1; i < WARMUP; i++) {
    if (e.benchIter(STEPS) !== checksum) throw new Error('warmup checksum diverged');
}
const times = [];
for (let i = 0; i < ITERS; i++) {
    const t = performance.now();
    const sum = e.benchIter(STEPS);
    times.push(performance.now() - t);
    if (sum !== checksum) throw new Error('checksum diverged');
}
const best = Math.min(...times);
const bits = new BigUint64Array(new Float64Array([checksum]).buffer)[0];
console.log(`BOX2D-BENCH: pyramid${STEPS} | 15 bodies | ${best.toFixed(3)} ms/run best | ${checksum} | ${ITERS} iters`);
console.log(`BOX2D-BENCH: checksum bits ${bits.toString(16)}`);
console.log(JSON.stringify({ lane: 'v8', wasmBytes: wasmBytes.length, loadMs: +loadMs.toFixed(2), bestMs: +best.toFixed(3) }));
