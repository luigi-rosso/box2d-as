// Phase A+B differential comparison: replays the C reference vectors against
// the AS modules and compares f32 results bit-exactly (both-NaN counts as
// equal to allow platform NaN payload differences).

import { readFileSync } from 'fs';

const stubImports = () => new Proxy({}, { get: () => new Proxy({}, { get: () => () => 0 }) });

const bytes = readFileSync(new URL('./verifyA.wasm', import.meta.url));
const { instance } = await WebAssembly.instantiate(bytes, stubImports());
const e = instance.exports;

const bytesB = readFileSync(new URL('./verifyB.wasm', import.meta.url));
const { instance: instanceB } = await WebAssembly.instantiate(bytesB, stubImports());
const b = instanceB.exports;

let failures = 0, checks = 0;
const famChecks = new Map();
let currentFam = '';

function bump() {
    checks++;
    famChecks.set(currentFam, (famChecks.get(currentFam) || 0) + 1);
}

function fail(msg) {
    failures++;
    if (failures <= 25) console.log('FAIL:', currentFam, msg);
}

// hex token -> i32 for wasm param passing
function h(t) {
    return parseInt(t, 16) | 0;
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

function ckInt(got, wantTok, msg) {
    bump();
    const want = parseInt(wantTok, 10);
    if (got !== want) fail(`${msg} got ${got} want ${want}`);
}

// packed u64 (BigInt) vs two hex tokens: high then low
function ckPacked(got, hiTok, loTok, msg) {
    ckBits(Number((got >> 32n) & 0xffffffffn), hiTok, msg + ' [hi]');
    ckBits(Number(got & 0xffffffffn), loTok, msg + ' [lo]');
}

function ckRes(count, toks, at, msg) {
    for (let i = 0; i < count; i++) {
        ckBits(e.resGet(i), toks[at + i], `${msg} [res${i}]`);
    }
}

function ckCast(toks, at, msg) {
    bump();
    const wantHit = parseInt(toks[at], 10);
    if (e.resGet(0) !== wantHit) fail(`${msg} hit got ${e.resGet(0)} want ${wantHit}`);
    for (let i = 0; i < 5; i++) {
        ckBits(e.resGet(1 + i), toks[at + 1 + i], `${msg} [cast${i}]`);
    }
}

function setPts(toks, at, n) {
    for (let i = 0; i < n; i++) {
        e.setPt(i, h(toks[at + 2 * i]), h(toks[at + 2 * i + 1]));
    }
}

function ckPolyDump(toks, at, msg) {
    bump();
    const wantCount = parseInt(toks[at], 10);
    if (e.polyGet(0) !== wantCount) fail(`${msg} count got ${e.polyGet(0)} want ${wantCount}`);
    for (let i = 1; i < 36; i++) {
        ckBits(e.polyGet(i), toks[at + i], `${msg} [poly${i}]`);
    }
}

function runMath(lines) {
    for (const line of lines) {
        const t = line.split(' ');
        const tag = t[0];
        currentFam = tag;
        switch (tag) {
            case 'F1': {
                const op = parseInt(t[1], 10);
                const got = e.mF1(op, h(t[2]));
                if (op === 2) ckInt(got, t[3], line); else ckBits(got, t[3], line);
                break;
            }
            case 'F2':
                ckBits(e.mF2(parseInt(t[1], 10), h(t[2]), h(t[3])), t[4], line);
                break;
            case 'F3':
                ckBits(e.mF3(h(t[1]), h(t[2]), h(t[3])), t[4], line);
                break;
            case 'AT':
                ckBits(e.mAtan2(h(t[1]), h(t[2])), t[3], line);
                break;
            case 'CS':
                ckPacked(BigInt.asUintN(64, e.mCosSin(h(t[1]))), t[2], t[3], line);
                break;
            case 'SPR':
                ckBits(e.mSpring(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5])), t[6], line);
                break;
            case 'V':
                ckPacked(BigInt.asUintN(64, e.mV(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]))), t[7], t[8], line);
                break;
            case 'VC':
                ckPacked(BigInt.asUintN(64, e.mVClamp(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]))), t[7], t[8], line);
                break;
            case 'VF': {
                const op = parseInt(t[1], 10);
                const got = e.mVF(op, h(t[2]), h(t[3]), h(t[4]), h(t[5]));
                if (op === 6 || op === 8) ckInt(got, t[6], line); else ckBits(got, t[6], line);
                break;
            }
            case 'R':
                ckPacked(BigInt.asUintN(64, e.mR(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]))), t[6], t[7], line);
                break;
            case 'NL':
                ckPacked(BigInt.asUintN(64, e.mNLerp(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]))), t[6], t[7], line);
                break;
            case 'RF': {
                const op = parseInt(t[1], 10);
                const got = e.mRF(op, h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]));
                if (op === 3 || op === 4) ckInt(got, t[7], line); else ckBits(got, t[7], line);
                break;
            }
            case 'RV':
                ckPacked(BigInt.asUintN(64, e.mRV(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]))), t[6], t[7], line);
                break;
            case 'TP':
                ckPacked(BigInt.asUintN(64, e.mTP(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]))), t[8], t[9], line);
                break;
            case 'TT':
                e.mTT(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]), h(t[9]));
                ckRes(4, t, 10, line);
                break;
            case 'TV':
                ckInt(e.mTValid(h(t[1]), h(t[2]), h(t[3]), h(t[4])), t[5], line);
                break;
            case 'M':
                ckPacked(BigInt.asUintN(64, e.mM(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]))), t[8], t[9], line);
                break;
            case 'MI':
                e.mMInv(h(t[1]), h(t[2]), h(t[3]), h(t[4]));
                ckRes(4, t, 5, line);
                break;
            case 'AB':
                ckInt(e.mAB2(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]), h(t[9])), t[10], line);
                break;
            case 'ABU':
                e.mABUnion(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                ckRes(4, t, 9, line);
                break;
            case 'ABC':
                ckPacked(BigInt.asUintN(64, e.mAB1(0, h(t[1]), h(t[2]), h(t[3]), h(t[4]))), t[5], t[6], line);
                break;
            case 'ABX':
                ckPacked(BigInt.asUintN(64, e.mAB1(1, h(t[1]), h(t[2]), h(t[3]), h(t[4]))), t[5], t[6], line);
                break;
            case 'MKAABB': {
                const n = parseInt(t[1], 10);
                const radius = t[2];
                setPts(t, 3, n);
                e.mMakeAABB(n, h(radius));
                ckRes(4, t, 3 + 2 * n, line);
                break;
            }
            case 'PL':
                ckBits(e.mPlane(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5])), t[6], line);
                break;
            case 'PLV':
                ckInt(e.mPlaneValid(h(t[1]), h(t[2]), h(t[3])), t[4], line);
                break;
            case 'K':
                ckBits(e.kConst(parseInt(t[1], 10)), t[2], line);
                break;
            case 'LUPM': {
                e.kSetLUPM(h(t[1]));
                ckBits(e.kGetLUPM(), t[1], line + ' [get]');
                for (let i = 0; i < 4; i++) {
                    ckBits(e.kConst(i), t[2 + i], line + ` [k${i}]`);
                }
                e.kSetLUPM(0x3f800000); // restore 1.0
                break;
            }
            case 'IDW': {
                const idx = parseInt(t[1], 10), gen = parseInt(t[2], 10);
                ckBits(e.idWorldStore(idx, gen), t[3], line + ' [store]');
                e.idWorldLoad(h(t[3]));
                ckInt(e.resGet(0), t[4], line + ' [idx]');
                ckInt(e.resGet(1), t[5], line + ' [gen]');
                break;
            }
            case 'IDB': {
                const idx = parseInt(t[1], 10), w = parseInt(t[2], 10), g = parseInt(t[3], 10);
                const packed = BigInt.asUintN(64, e.idBodyStore(idx, w, g));
                bump();
                if (packed !== BigInt('0x' + t[4])) fail(`${line} store got ${packed.toString(16)}`);
                e.idBodyLoad(BigInt.asIntN(64, BigInt('0x' + t[4])));
                ckInt(e.resGet(0) | 0, t[5], line + ' [idx]');
                ckInt(e.resGet(1), t[6], line + ' [w]');
                ckInt(e.resGet(2), t[7], line + ' [g]');
                break;
            }
            case 'IDC': {
                const idx = parseInt(t[1], 10), w = parseInt(t[2], 10), g = parseInt(t[3], 10);
                e.idContactStore(idx, w, g | 0);
                ckBits(e.resGet(0), t[4], line + ' [v0]');
                ckBits(e.resGet(1), t[5], line + ' [v1]');
                ckBits(e.resGet(2), t[6], line + ' [v2]');
                e.idContactLoad(h(t[4]), h(t[5]), h(t[6]));
                ckInt(e.resGet(0) | 0, t[7], line + ' [idx]');
                ckInt(e.resGet(1), t[8], line + ' [w]');
                ckInt(e.resGet(2) >>> 0, t[9], line + ' [g]');
                break;
            }
            default:
                fail('unknown math tag ' + tag);
        }
    }
}

function runState(lines) {
    for (const line of lines) {
        const t = line.split(' ');
        currentFam = t[0] + '.' + t[1];
        if (t[0] === 'IP') {
            if (t[1] === 'RESET') { e.idpReset(); continue; }
            if (t[1] === 'A') {
                ckInt(e.idpAlloc(), t[2], line + ' [id]');
            } else {
                e.idpFree(parseInt(t[2], 10));
            }
            ckInt(e.idpCount(), t[3], line + ' [count]');
            ckInt(e.idpCapacity(), t[4], line + ' [cap]');
            ckInt(e.idpFreeCount(), t[5], line + ' [freecount]');
            ckInt(e.idpFreeCap(), t[6], line + ' [freecap]');
        } else if (t[0] === 'IA') {
            switch (t[1]) {
                case 'RESET':
                    e.iaReset(parseInt(t[2], 10));
                    ckInt(e.iaCount(), t[3], line);
                    ckInt(e.iaCapacity(), t[4], line);
                    break;
                case 'PUSH':
                    e.iaPush(parseInt(t[2], 10));
                    ckInt(e.iaCount(), t[3], line);
                    ckInt(e.iaCapacity(), t[4], line);
                    break;
                case 'POP':
                    ckInt(e.iaPop(), t[2], line);
                    ckInt(e.iaCount(), t[3], line);
                    break;
                case 'RS':
                    ckInt(e.iaRemoveSwap(parseInt(t[2], 10)), t[3], line);
                    ckInt(e.iaCount(), t[4], line);
                    break;
                case 'RES':
                    e.iaReserve(parseInt(t[2], 10));
                    ckInt(e.iaCapacity(), t[3], line);
                    break;
                case 'GET':
                    ckInt(e.iaGet(parseInt(t[2], 10)), t[3], line);
                    break;
                case 'RZ':
                    e.iaResize(parseInt(t[2], 10));
                    ckInt(e.iaCount(), t[3], line);
                    ckInt(e.iaCapacity(), t[4], line);
                    break;
                default:
                    fail('unknown IA op ' + t[1]);
            }
        } else if (t[0] === 'BS') {
            const which = t[2] === undefined ? 0 : parseInt(t[2], 10);
            switch (t[1]) {
                case 'RESET': e.bsReset(which, parseInt(t[3], 10)); break;
                case 'SG': e.bsSetBitGrow(which, parseInt(t[3], 10)); break;
                case 'SB': e.bsSetBit(which, parseInt(t[3], 10)); break;
                case 'CB': e.bsClearBit(which, parseInt(t[3], 10)); break;
                case 'SCC': e.bsSetBitCountAndClear(which, parseInt(t[3], 10)); break;
                case 'U': e.bsUnion(); break;
                case 'GB': ckInt(e.bsGetBit(which, parseInt(t[3], 10)), t[4], line); break;
                case 'CNT': ckInt(e.bsCount(which), t[3], line); break;
                case 'ST': {
                    ckInt(e.bsBlockCount(which), t[3], line + ' [bc]');
                    ckInt(e.bsBlockCapacity(which), t[4], line + ' [bcap]');
                    bump();
                    const got = BigInt.asUintN(64, e.bsHash(which));
                    if (got !== BigInt('0x' + t[5])) fail(`${line} hash got ${got.toString(16)}`);
                    break;
                }
                default:
                    fail('unknown BS op ' + t[1]);
            }
        } else if (t[0] === 'AR') {
            switch (t[1]) {
                case 'RESET': e.arReset(parseInt(t[2], 10)); break;
                case 'AL': ckInt(e.arAlloc(parseInt(t[2], 10)), t[3], line + ' [off]'); break;
                case 'FR': e.arFree(); break;
                case 'GROW': e.arGrow(); break;
                case 'ST':
                    ckInt(e.arIndex(), t[2], line + ' [index]');
                    ckInt(e.arAllocation(), t[3], line + ' [alloc]');
                    ckInt(e.arMaxAllocation(), t[4], line + ' [max]');
                    ckInt(e.arCapacity(), t[5], line + ' [cap]');
                    ckInt(e.arEntryCount(), t[6], line + ' [entries]');
                    break;
                default:
                    fail('unknown AR op ' + t[1]);
            }
        } else {
            fail('unknown state tag ' + t[0]);
        }
    }
}

function runGeom(lines) {
    for (const line of lines) {
        const t = line.split(' ');
        const tag = t[0];
        currentFam = tag;
        switch (tag) {
            case 'HULL': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                const at = 2 + 2 * n;
                const wantCount = parseInt(t[at], 10);
                ckInt(e.gComputeHull(n), t[at], line + ' [count]');
                ckInt(e.gValidateCurHull(), t[at + 1], line + ' [valid]');
                for (let i = 0; i < 2 * wantCount; i++) {
                    ckBits(e.hullGet(i), t[at + 2 + i], line + ` [pt${i}]`);
                }
                break;
            }
            case 'VH': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                ckInt(e.gStageHull(n), t[2 + 2 * n], line);
                break;
            }
            case 'MAKEP': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                e.gComputeHull(n);
                const at = 2 + 2 * n;
                e.gMakePolygon(h(t[at]));
                ckPolyDump(t, at + 1, line);
                break;
            }
            case 'ORPOLY': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                e.gComputeHull(n);
                const at = 2 + 2 * n;
                e.gMakeOffsetRoundedPolygon(h(t[at]), h(t[at + 1]), h(t[at + 2]), h(t[at + 3]), h(t[at + 4]));
                ckPolyDump(t, at + 5, line);
                break;
            }
            case 'TPOLY': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                e.gComputeHull(n);
                const at = 2 + 2 * n;
                e.gMakePolygon(h(t[at]));
                e.gTransformPolygon(h(t[at + 1]), h(t[at + 2]), h(t[at + 3]), h(t[at + 4]));
                ckPolyDump(t, at + 5, line);
                break;
            }
            case 'PMASS': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                e.gComputeHull(n);
                const at = 2 + 2 * n;
                e.gMakePolygon(h(t[at]));
                e.gPolygonMass(h(t[at + 1]));
                ckRes(4, t, at + 2, line);
                break;
            }
            case 'PAABB': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                e.gComputeHull(n);
                const at = 2 + 2 * n;
                e.gMakePolygon(h(t[at]));
                e.gPolygonAABB(h(t[at + 1]), h(t[at + 2]), h(t[at + 3]), h(t[at + 4]));
                ckRes(4, t, at + 5, line);
                break;
            }
            case 'RCP': {
                const n = parseInt(t[1], 10);
                setPts(t, 2, n);
                e.gComputeHull(n);
                const at = 2 + 2 * n;
                e.gMakePolygon(0);
                e.gRayCastPolygon(h(t[at]), h(t[at + 1]), h(t[at + 2]), h(t[at + 3]), h(t[at + 4]));
                ckCast(t, at + 5, line);
                break;
            }
            case 'BOX':
                e.gMakeBox(h(t[1]), h(t[2]));
                ckPolyDump(t, 3, line);
                break;
            case 'SQ':
                e.gMakeSquare(h(t[1]));
                ckPolyDump(t, 2, line);
                break;
            case 'RBOX':
                e.gMakeRoundedBox(h(t[1]), h(t[2]), h(t[3]));
                ckPolyDump(t, 4, line);
                break;
            case 'OBOX':
                e.gMakeOffsetBox(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]));
                ckPolyDump(t, 7, line);
                break;
            case 'ORBOX':
                e.gMakeOffsetRoundedBox(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]));
                ckPolyDump(t, 8, line);
                break;
            case 'CMASS':
                e.gCircleMass(h(t[1]), h(t[2]), h(t[3]), h(t[4]));
                ckRes(4, t, 5, line);
                break;
            case 'KMASS':
                e.gCapsuleMass(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]));
                ckRes(4, t, 7, line);
                break;
            case 'CAABB':
                e.gCircleAABB(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]));
                ckRes(4, t, 8, line);
                break;
            case 'KAABB':
                e.gCapsuleAABB(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]), h(t[9]));
                ckRes(4, t, 10, line);
                break;
            case 'SAABB':
                e.gSegmentAABB(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                ckRes(4, t, 9, line);
                break;
            case 'PIC':
                ckInt(e.gPointInCircle(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5])), t[6], line);
                break;
            case 'PIK':
                ckInt(e.gPointInCapsule(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7])), t[8], line);
                break;
            case 'RCC':
                e.gRayCastCircle(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                ckCast(t, 9, line);
                break;
            case 'RCK':
                e.gRayCastCapsule(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]), h(t[9]), h(t[10]));
                ckCast(t, 11, line);
                break;
            case 'RCS':
                e.gRayCastSegment(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]), h(t[9]), parseInt(t[10], 10));
                ckCast(t, 11, line);
                break;
            case 'ENL':
                ckInt(e.mEnlarge(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8])), t[9], line + ' [changed]');
                ckRes(4, t, 10, line);
                break;
            case 'ABV':
                ckInt(Number(BigInt.asUintN(64, e.mAB1(2, h(t[1]), h(t[2]), h(t[3]), h(t[4])))), t[5], line);
                break;
            case 'ABP':
                ckBits(Number(BigInt.asUintN(64, e.mAB1(3, h(t[1]), h(t[2]), h(t[3]), h(t[4])))), t[5], line);
                break;
            case 'ARC':
                e.mAabbRayCast(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                ckCast(t, 9, line);
                break;
            default:
                fail('unknown geom tag ' + tag);
        }
    }
}

// ---- phase B helpers ----

function u64(tok) {
    return BigInt('0x' + tok);
}

function ckU64(got, tok, msg) {
    bump();
    const want = BigInt.asUintN(64, u64(tok));
    const g = BigInt.asUintN(64, got);
    if (g !== want) fail(`${msg} got ${g.toString(16)} want ${tok}`);
}

// hex-bit compare against module B's res array
function ckResB(count, toks, at, msg) {
    for (let i = 0; i < count; i++) {
        ckBits(b.resGet(i), toks[at + i], `${msg} [res${i}]`);
    }
}

// Stages proxy points from tokens: n, radius, then 2n point tokens.
// Returns the token index just past the proxy spec, and the (n, radius).
function stageProxy(toks, at, setter) {
    const n = parseInt(toks[at], 10);
    const radius = toks[at + 1];
    for (let i = 0; i < n; i++) {
        setter(i, h(toks[at + 2 + 2 * i]), h(toks[at + 2 + 2 * i + 1]));
    }
    return { next: at + 2 + 2 * n, n, radius };
}

// Polygon spec: n, 2n pts, radius (radius comes after the points).
function stagePolySpec(toks, at, setter, stager) {
    const n = parseInt(toks[at], 10);
    for (let i = 0; i < n; i++) {
        setter(i, h(toks[at + 1 + 2 * i]), h(toks[at + 1 + 2 * i + 1]));
    }
    const radius = toks[at + 1 + 2 * n];
    stager(n, h(radius));
    return at + 2 + 2 * n;
}

// Simplex cache tokens: count, a0..a2, b0..b2. Returns index past them.
function stageCache(toks, at) {
    b.dSetCache(parseInt(toks[at], 10),
        parseInt(toks[at + 1], 10), parseInt(toks[at + 2], 10), parseInt(toks[at + 3], 10),
        parseInt(toks[at + 4], 10), parseInt(toks[at + 5], 10), parseInt(toks[at + 6], 10));
    return at + 7;
}

function ckCacheB(toks, at, msg) {
    for (let i = 0; i < 7; i++) {
        ckInt(b.resGet(8 + i) | 0, toks[at + i], `${msg} [cache${i}]`);
    }
    return at + 7;
}

// Manifold: count, normal(2), then per point: point(2) anchorA(2) anchorB(2)
// separation id. Returns index past the manifold tokens.
function ckManifoldB(toks, at, msg) {
    const count = parseInt(toks[at], 10);
    ckInt(b.resGet(0) | 0, toks[at], `${msg} [count]`);
    ckBits(b.resGet(1), toks[at + 1], `${msg} [nx]`);
    ckBits(b.resGet(2), toks[at + 2], `${msg} [ny]`);
    let idx = at + 3;
    for (let i = 0; i < count; i++) {
        const base = 3 + 8 * i;
        for (let k = 0; k < 7; k++) {
            ckBits(b.resGet(base + k), toks[idx + k], `${msg} [p${i}.${k}]`);
        }
        ckInt(b.resGet(base + 7) | 0, toks[idx + 7], `${msg} [p${i}.id]`);
        idx += 8;
    }
    return idx;
}

function ckChainCacheB(toks, at, msg) {
    // generator writes a '|' separator token before the cache
    if (toks[at] !== '|') { bump(); fail(`${msg} missing cache separator`); return at; }
    at += 1;
    for (let i = 0; i < 7; i++) {
        ckInt(b.resGet(20 + i) | 0, toks[at + i], `${msg} [cacheOut${i}]`);
    }
    return at + 7;
}

function xfArgs(toks, at) {
    return [h(toks[at]), h(toks[at + 1]), h(toks[at + 2]), h(toks[at + 3])];
}

function runGjk(lines) {
    for (const line of lines) {
        const t = line.split(' ');
        const tag = t[0];
        currentFam = tag;
        switch (tag) {
            case 'DIST': {
                let p = stageProxy(t, 1, (i, x, y) => b.dSetPtA(i, x, y));
                b.dStageProxyA(p.n, h(p.radius));
                let q = stageProxy(t, p.next, (i, x, y) => b.dSetPtB(i, x, y));
                b.dStageProxyB(q.n, h(q.radius));
                let at = q.next;
                const xfA = xfArgs(t, at); at += 4;
                const xfB = xfArgs(t, at); at += 4;
                const useRadii = parseInt(t[at], 10); at += 1;
                at = stageCache(t, at);
                b.dDistance(...xfA, ...xfB, useRadii);
                ckResB(7, t, at, line);
                ckInt(b.resGet(7) | 0, t[at + 7], line + ' [iters]');
                for (let i = 0; i < 7; i++) {
                    ckInt(b.resGet(8 + i) | 0, t[at + 8 + i], line + ` [cache${i}]`);
                }
                break;
            }
            case 'SC': {
                let p = stageProxy(t, 1, (i, x, y) => b.dSetPtA(i, x, y));
                b.dStageProxyA(p.n, h(p.radius));
                let q = stageProxy(t, p.next, (i, x, y) => b.dSetPtB(i, x, y));
                b.dStageProxyB(q.n, h(q.radius));
                let at = q.next;
                const xfA = xfArgs(t, at); at += 4;
                const xfB = xfArgs(t, at); at += 4;
                const tx = h(t[at]), ty = h(t[at + 1]), mf = h(t[at + 2]);
                const enc = parseInt(t[at + 3], 10);
                at += 4;
                b.dShapeCast(...xfA, ...xfB, tx, ty, mf, enc);
                ckInt(b.resGet(0) | 0, t[at], line + ' [hit]');
                for (let i = 0; i < 5; i++) {
                    ckBits(b.resGet(1 + i), t[at + 1 + i], line + ` [c${i}]`);
                }
                ckInt(b.resGet(6) | 0, t[at + 6], line + ' [iters]');
                break;
            }
            case 'TOI': {
                let p = stageProxy(t, 1, (i, x, y) => b.dSetPtA(i, x, y));
                b.dStageProxyA(p.n, h(p.radius));
                let q = stageProxy(t, p.next, (i, x, y) => b.dSetPtB(i, x, y));
                b.dStageProxyB(q.n, h(q.radius));
                let at = q.next;
                b.dSetSweepA(...t.slice(at, at + 10).map(h)); at += 10;
                b.dSetSweepB(...t.slice(at, at + 10).map(h)); at += 10;
                const mf = h(t[at]); at += 1;
                b.dTOI(mf);
                ckInt(b.resGet(0) | 0, t[at], line + ' [state]');
                ckBits(b.resGet(1), t[at + 1], line + ' [fraction]');
                // C leaves b2TOIOutput.point/normal uninitialized on the
                // non-hit paths (and one failed path); only the hit state
                // reliably writes them.
                if (parseInt(t[at], 10) === 3) {
                    for (let i = 1; i < 5; i++) {
                        ckBits(b.resGet(1 + i), t[at + 1 + i], line + ` [o${i}]`);
                    }
                }
                break;
            }
            case 'PIP': {
                let at = stagePolySpec(t, 1, (i, x, y) => b.dSetPtA(i, x, y), (n, r) => b.mStagePolyA(n, r));
                const px = h(t[at]), py = h(t[at + 1]);
                ckInt(b.pPointInPolygon(px, py) | 0, t[at + 2], line);
                break;
            }
            case 'RCRP': {
                let at = stagePolySpec(t, 1, (i, x, y) => b.dSetPtA(i, x, y), (n, r) => b.mStagePolyA(n, r));
                b.pRayCastPolygon(h(t[at]), h(t[at + 1]), h(t[at + 2]), h(t[at + 3]), h(t[at + 4]));
                at += 5;
                ckInt(b.resGet(0) | 0, t[at], line + ' [hit]');
                for (let i = 0; i < 5; i++) {
                    ckBits(b.resGet(1 + i), t[at + 1 + i], line + ` [c${i}]`);
                }
                ckInt(b.resGet(6) | 0, t[at + 6], line + ' [iters]');
                break;
            }
            case 'SCW': {
                const which = parseInt(t[1], 10);
                let p = stageProxy(t, 2, (i, x, y) => b.dSetPtB(i, x, y));
                b.dStageProxyB(p.n, h(p.radius));
                let at = p.next;
                b.wShapeCastWrapper(which, h(t[at]), h(t[at + 1]), h(t[at + 2]), parseInt(t[at + 3], 10));
                at += 4;
                ckInt(b.resGet(0) | 0, t[at], line + ' [hit]');
                for (let i = 0; i < 5; i++) {
                    ckBits(b.resGet(1 + i), t[at + 1 + i], line + ` [c${i}]`);
                }
                ckInt(b.resGet(6) | 0, t[at + 6], line + ' [iters]');
                break;
            }
            default:
                fail('unknown gjk tag ' + tag);
        }
    }
}

function runManifold(lines) {
    for (const line of lines) {
        const t = line.split(' ');
        const tag = t[0];
        currentFam = tag;
        switch (tag) {
            case 'MCC': {
                b.mCollideCircles(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]),
                    ...xfArgs(t, 7), ...xfArgs(t, 11));
                ckManifoldB(t, 15, line);
                break;
            }
            case 'MKC': {
                b.mCollideCapsuleAndCircle(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]),
                    h(t[6]), h(t[7]), h(t[8]), ...xfArgs(t, 9), ...xfArgs(t, 13));
                ckManifoldB(t, 17, line);
                break;
            }
            case 'MKK': {
                b.mCollideCapsules(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]),
                    h(t[6]), h(t[7]), h(t[8]), h(t[9]), h(t[10]), ...xfArgs(t, 11), ...xfArgs(t, 15));
                ckManifoldB(t, 19, line);
                break;
            }
            case 'MSK': {
                b.mCollideSegmentAndCapsule(h(t[1]), h(t[2]), h(t[3]), h(t[4]),
                    h(t[5]), h(t[6]), h(t[7]), h(t[8]), h(t[9]), ...xfArgs(t, 10), ...xfArgs(t, 14));
                ckManifoldB(t, 18, line);
                break;
            }
            case 'MPC': {
                let at = stagePolySpec(t, 1, (i, x, y) => b.dSetPtA(i, x, y), (n, r) => b.mStagePolyA(n, r));
                b.mCollidePolygonAndCircle(h(t[at]), h(t[at + 1]), h(t[at + 2]),
                    ...xfArgs(t, at + 3), ...xfArgs(t, at + 7));
                ckManifoldB(t, at + 11, line);
                break;
            }
            case 'MPK': {
                let at = stagePolySpec(t, 1, (i, x, y) => b.dSetPtA(i, x, y), (n, r) => b.mStagePolyA(n, r));
                b.mCollidePolygonAndCapsule(h(t[at]), h(t[at + 1]), h(t[at + 2]), h(t[at + 3]), h(t[at + 4]),
                    ...xfArgs(t, at + 5), ...xfArgs(t, at + 9));
                ckManifoldB(t, at + 13, line);
                break;
            }
            case 'MPP': {
                let at = stagePolySpec(t, 1, (i, x, y) => b.dSetPtA(i, x, y), (n, r) => b.mStagePolyA(n, r));
                at = stagePolySpec(t, at, (i, x, y) => b.dSetPtB(i, x, y), (n, r) => b.mStagePolyB(n, r));
                b.mCollidePolygons(...xfArgs(t, at), ...xfArgs(t, at + 4));
                ckManifoldB(t, at + 8, line);
                break;
            }
            case 'MSC': {
                b.mCollideSegmentAndCircle(h(t[1]), h(t[2]), h(t[3]), h(t[4]),
                    h(t[5]), h(t[6]), h(t[7]), ...xfArgs(t, 8), ...xfArgs(t, 12));
                ckManifoldB(t, 16, line);
                break;
            }
            case 'MSP': {
                let at = stagePolySpec(t, 5, (i, x, y) => b.dSetPtB(i, x, y), (n, r) => b.mStagePolyB(n, r));
                b.mCollideSegmentAndPolygon(h(t[1]), h(t[2]), h(t[3]), h(t[4]),
                    ...xfArgs(t, at), ...xfArgs(t, at + 4));
                ckManifoldB(t, at + 8, line);
                break;
            }
            case 'MXC': {
                b.mSetChain(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                b.mCollideChainSegmentAndCircle(h(t[9]), h(t[10]), h(t[11]),
                    ...xfArgs(t, 12), ...xfArgs(t, 16));
                ckManifoldB(t, 20, line);
                break;
            }
            case 'MXK': {
                b.mSetChain(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                let at = 14;
                const xfA = xfArgs(t, at); at += 4;
                const xfB = xfArgs(t, at); at += 4;
                at = stageCache(t, at);
                b.mCollideChainSegmentAndCapsule(h(t[9]), h(t[10]), h(t[11]), h(t[12]), h(t[13]),
                    ...xfA, ...xfB);
                at = ckManifoldB(t, at, line);
                ckChainCacheB(t, at, line);
                break;
            }
            case 'MXP': {
                b.mSetChain(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), h(t[6]), h(t[7]), h(t[8]));
                let at = stagePolySpec(t, 9, (i, x, y) => b.dSetPtB(i, x, y), (n, r) => b.mStagePolyB(n, r));
                const xfA = xfArgs(t, at); at += 4;
                const xfB = xfArgs(t, at); at += 4;
                at = stageCache(t, at);
                b.mCollideChainSegmentAndPolygon(...xfA, ...xfB);
                at = ckManifoldB(t, at, line);
                ckChainCacheB(t, at, line);
                break;
            }
            default:
                fail('unknown manifold tag ' + tag);
        }
    }
}

function ckTreeState(toks, at, msg) {
    b.trState();
    const names = ['root', 'nodeCount', 'nodeCap', 'freeList', 'proxyCount'];
    for (let i = 0; i < 5; i++) {
        ckInt(b.resGet(i) | 0, toks[at + i], `${msg} [${names[i]}]`);
    }
    ckU64(b.trHash(), toks[at + 5], msg + ' [hash]');
    return at + 6;
}

function ckVisits(toks, at, msg) {
    ckInt(b.resGet(0) | 0, toks[at], msg + ' [nodeVisits]');
    ckInt(b.resGet(1) | 0, toks[at + 1], msg + ' [leafVisits]');
    const n = parseInt(toks[at + 2], 10);
    ckInt(b.resGet(2) | 0, toks[at + 2], msg + ' [visitCount]');
    for (let i = 0; i < n; i++) {
        ckInt(b.vGet(i), toks[at + 3 + i], `${msg} [v${i}]`);
    }
}

function ckBPState(toks, at, msg) {
    const moveCount = parseInt(toks[at], 10);
    ckInt(b.bpMoveCount(), toks[at], msg + ' [moveCount]');
    for (let i = 0; i < moveCount; i++) {
        ckInt(b.bpMoveGet(i), toks[at + 1 + i], `${msg} [move${i}]`);
    }
    at += 1 + moveCount;
    ckInt(b.bpMoveSetCount(), toks[at], msg + ' [moveSetCount]');
    ckInt(b.bpMoveSetCapacity(), toks[at + 1], msg + ' [moveSetCap]');
    ckU64(b.bpMoveSetHash(), toks[at + 2], msg + ' [moveSetHash]');
}

function runTree(lines) {
    for (const line of lines) {
        const t = line.split(' ');
        const tag = t[0];
        currentFam = tag;
        switch (tag) {
            case 'TRR':
                b.trReset();
                break;
            case 'TRC': {
                const proxyId = b.trCreate(h(t[1]), h(t[2]), h(t[3]), h(t[4]), u64(t[5]), u64(t[6]));
                ckInt(proxyId, t[7], line + ' [proxyId]');
                ckTreeState(t, 8, line);
                break;
            }
            case 'TRD':
                b.trDestroy(parseInt(t[1], 10));
                ckTreeState(t, 2, line);
                break;
            case 'TRM':
                b.trMove(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]));
                ckTreeState(t, 6, line);
                break;
            case 'TRE':
                b.trEnlarge(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]));
                ckTreeState(t, 6, line);
                break;
            case 'TRB': {
                const leafCount = b.trRebuild(parseInt(t[1], 10));
                ckInt(leafCount, t[2], line + ' [leafCount]');
                ckTreeState(t, 3, line);
                break;
            }
            case 'TRQ':
                b.trQuery(h(t[1]), h(t[2]), h(t[3]), h(t[4]), u64(t[5]));
                ckVisits(t, 6, line);
                break;
            case 'TRRC':
                b.trRayCast(h(t[1]), h(t[2]), h(t[3]), h(t[4]), h(t[5]), parseInt(t[6], 10));
                ckVisits(t, 7, line);
                break;
            case 'TRSC': {
                let p = stageProxy(t, 1, (i, x, y) => b.dSetPtA(i, x, y));
                let at = p.next;
                b.trShapeCast(p.n, h(p.radius), h(t[at]), h(t[at + 1]), h(t[at + 2]), parseInt(t[at + 3], 10));
                ckVisits(t, at + 4, line);
                break;
            }
            case 'HSR': {
                const cap = b.hsReset(parseInt(t[1], 10));
                ckInt(cap, t[2], line + ' [cap]');
                break;
            }
            case 'HSA': {
                const found = b.hsAdd(u64(t[1]));
                ckInt(found, t[2], line + ' [found]');
                ckInt(b.hsCount(), t[3], line + ' [count]');
                ckInt(b.hsCapacity(), t[4], line + ' [cap]');
                ckU64(b.hsHash(), t[5], line + ' [hash]');
                break;
            }
            case 'HSD': {
                const found = b.hsRemove(u64(t[1]));
                ckInt(found, t[2], line + ' [found]');
                ckInt(b.hsCount(), t[3], line + ' [count]');
                ckInt(b.hsCapacity(), t[4], line + ' [cap]');
                ckU64(b.hsHash(), t[5], line + ' [hash]');
                break;
            }
            case 'HSC':
                ckInt(b.hsContains(u64(t[1])), t[2], line);
                break;
            case 'HSCL':
                b.hsClear();
                ckInt(b.hsCount(), t[1], line + ' [count]');
                ckInt(b.hsCapacity(), t[2], line + ' [cap]');
                ckU64(b.hsHash(), t[3], line + ' [hash]');
                break;
            case 'BPR':
                b.bpReset();
                break;
            case 'BPC': {
                const key = b.bpCreate(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]),
                    u64(t[6]), parseInt(t[7], 10), parseInt(t[8], 10));
                ckInt(key, t[9], line + ' [proxyKey]');
                ckBPState(t, 10, line);
                break;
            }
            case 'BPD':
                b.bpDestroy(parseInt(t[1], 10));
                ckBPState(t, 2, line);
                break;
            case 'BPM':
                b.bpMove(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]));
                ckBPState(t, 6, line);
                break;
            case 'BPE':
                b.bpEnlarge(parseInt(t[1], 10), h(t[2]), h(t[3]), h(t[4]), h(t[5]));
                ckBPState(t, 6, line);
                break;
            case 'BPU': {
                const n = b.bpUpdate();
                ckInt(n, t[1], line + ' [pairCount]');
                const wantN = parseInt(t[1], 10);
                for (let i = 0; i < 2 * wantN; i++) {
                    ckInt(b.bpPairGet(i), t[2 + i], `${line} [pair${i}]`);
                }
                const at = 2 + 2 * wantN;
                ckInt(b.bpPairSetCount(), t[at], line + ' [pairSetCount]');
                ckInt(b.bpPairSetCapacity(), t[at + 1], line + ' [pairSetCap]');
                break;
            }
            case 'BPT':
                ckInt(b.bpTest(parseInt(t[1], 10), parseInt(t[2], 10)), t[3], line);
                break;
            case 'BPRB':
                b.bpRebuild();
                ckU64(b.bpTreeHash(0), t[1], line + ' [h0]');
                ckU64(b.bpTreeHash(1), t[2], line + ' [h1]');
                ckU64(b.bpTreeHash(2), t[3], line + ' [h2]');
                break;
            default:
                fail('unknown tree tag ' + tag);
        }
    }
}

function loadLines(name) {
    return readFileSync(new URL(`./vectors/${name}`, import.meta.url), 'utf8').trim().split('\n');
}

runMath(loadLines('math_vectors.txt'));
runState(loadLines('state_vectors.txt'));
runGeom(loadLines('geom_vectors.txt'));
runGjk(loadLines('gjk_vectors.txt'));
runManifold(loadLines('manifold_vectors.txt'));
runTree(loadLines('tree_vectors.txt'));

const fams = [...famChecks.entries()].map(([k, v]) => `${k}:${v}`).join(' ');
console.log('families:', fams);
console.log(`checks: ${checks}, failures: ${failures}`);
process.exit(failures === 0 ? 0 : 1);
