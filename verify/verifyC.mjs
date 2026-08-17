// Phase C differential comparison: replays the dynamics scenes against the
// AS world and compares per-step body states and bookkeeping hashes
// bit-exactly against the C reference trajectory.

import { readFileSync } from 'fs';

const stubImports = () => new Proxy({}, { get: () => new Proxy({}, { get: () => () => 0 }) });

const wasmName = process.env.VERIFYC_WASM || './verifyC.wasm';
const bytes = readFileSync(new URL(wasmName, import.meta.url));
const { instance } = await WebAssembly.instantiate(bytes, stubImports());
const d = instance.exports;

let failures = 0, checks = 0;
const sceneChecks = new Map();
let currentScene = '';

function bump() {
    checks++;
    sceneChecks.set(currentScene, (sceneChecks.get(currentScene) || 0) + 1);
}

function fail(msg) {
    failures++;
    if (failures <= 40) console.log('FAIL:', currentScene, msg);
}

function isNaNBits(u) {
    u = u >>> 0;
    return (u & 0x7f800000) === 0x7f800000 && (u & 0x007fffff) !== 0;
}

function ckBits(got, wantTok, msg) {
    bump();
    const want = parseInt(wantTok, 16) >>> 0;
    const g = got >>> 0;
    if (g === want) return;
    if (isNaNBits(g) && isNaNBits(want)) return;
    fail(`${msg} got ${g.toString(16).padStart(8, '0')} want ${wantTok}`);
}

function ckInt(got, want, msg) {
    bump();
    if (got !== want) fail(`${msg} got ${got} want ${want}`);
}

function ckHash(got, wantTok, msg) {
    bump();
    const want = BigInt('0x' + wantTok);
    if (got !== want) fail(`${msg} got ${got.toString(16).padStart(16, '0')} want ${wantTok}`);
}

const vectorsName = process.env.VERIFYC_VECTORS || './vectors/dynamics_vectors.txt';
const lines = readFileSync(new URL(vectorsName, import.meta.url), 'utf8').split('\n');

let sceneRun = 0;
let stepsLeft = 0;
let step = -1;
let bodyIndex = 0;
let inScene = false;

for (const line of lines) {
    if (line.length === 0) continue;
    const t = line.split(' ');
    const tag = t[0];

    if (tag === 'SCENE') {
        if (inScene) d.dynEnd();
        const sceneId = parseInt(t[1], 10);
        stepsLeft = parseInt(t[2], 10);
        sceneRun += 1;
        currentScene = `scene${sceneId}#${sceneRun}`;
        d.dynStart(sceneId);
        inScene = true;
        step = -1;
    } else if (tag === 'S') {
        step = parseInt(t[1], 10);
        d.dynStep(step);
        d.dynCollect();
        ckInt(d.dynBodyCount(), parseInt(t[2], 10), `step ${step} body count`);
        bodyIndex = 0;
    } else if (tag === 'B') {
        const i = bodyIndex++;
        const id = parseInt(t[1], 10);
        const setIndex = parseInt(t[2], 10);
        ckInt(d.dynBodyId(i), id, `step ${step} body[${i}] id`);
        ckInt(d.dynBodySet(i), setIndex, `step ${step} body ${id} set`);
        for (let f = 0; f < 7; ++f) {
            ckBits(d.dynBodyF(i, f), t[3 + f], `step ${step} body ${id} f${f}`);
        }
    } else if (tag === 'C') {
        ckInt(d.dynContactCount(), parseInt(t[1], 10), `step ${step} contact count`);
        ckInt(d.dynTouchingCount(), parseInt(t[2], 10), `step ${step} touching count`);
        ckHash(BigInt.asUintN(64, d.dynContactHash()), t[3], `step ${step} contact hash`);
    } else if (tag === 'I') {
        ckHash(BigInt.asUintN(64, d.dynIslandHash()), t[1], `step ${step} island hash`);
        ckHash(BigInt.asUintN(64, d.dynSetHash()), t[2], `step ${step} set hash`);
        ckHash(BigInt.asUintN(64, d.dynColorHash()), t[3], `step ${step} color hash`);
        ckHash(BigInt.asUintN(64, d.dynBookHash()), t[4], `step ${step} book hash`);
    } else if (tag === 'E') {
        for (let i = 0; i < 5; ++i) {
            ckInt(d.dynEventCount(i), parseInt(t[1 + i], 10), `step ${step} event count ${i}`);
        }
        ckHash(BigInt.asUintN(64, d.dynEventHash()), t[6], `step ${step} event hash`);
    }
}

if (inScene) d.dynEnd();

for (const [scene, count] of sceneChecks) {
    console.log(`${scene}: ${count} checks`);
}
console.log(`phase C total: ${checks} checks, ${failures} failures`);
if (failures > 0) process.exit(1);
