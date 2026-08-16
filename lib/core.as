// Ported from box2d src/core.h/.c and src/ctz.h essentials. Allocation goes
// through the AS runtime instead of b2Alloc/b2Free, so only the length-units
// global and the bit helpers cross over.

import { b2IsValidFloat } from "./math_functions";

export const B2_NULL_INDEX: i32 = -1;

export const B2_SECRET_COOKIE: i32 = 1152023;

// This allows the user to change the length units at runtime
export let b2_lengthUnitsPerMeter: f32 = 1.0;

export function b2SetLengthUnitsPerMeter(lengthUnits: f32): void {
    assert(b2IsValidFloat(lengthUnits) && lengthUnits > 0.0);
    b2_lengthUnitsPerMeter = lengthUnits;
}

export function b2GetLengthUnitsPerMeter(): f32 {
    return b2_lengthUnitsPerMeter;
}

// ctz.h
export function b2CTZ32(block: u32): u32 {
    return ctz<u32>(block);
}

export function b2CLZ32(value: u32): u32 {
    return clz<u32>(value);
}

export function b2CTZ64(block: u64): u32 {
    return <u32>ctz<u64>(block);
}

export function b2PopCount64(block: u64): i32 {
    return <i32>popcnt<u64>(block);
}

export function b2IsPowerOf2(x: i32): bool {
    return (x & (x - 1)) == 0;
}

export function b2BoundingPowerOf2(x: i32): i32 {
    if (x <= 1) {
        return 1;
    }
    return 32 - <i32>b2CLZ32(<u32>x - 1);
}

export function b2RoundUpPowerOf2(x: i32): i32 {
    if (x <= 1) {
        return 1;
    }
    return 1 << (32 - <i32>b2CLZ32(<u32>x - 1));
}
