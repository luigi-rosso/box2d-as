// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of the bit helpers of src/platform.h, which earlier versions kept in src/ctz.h

@inline export function b2CTZ32(block: u32): u32 {
	return ctz<u32>(block);
}

@inline export function b2CLZ32(value: u32): u32 {
	return clz<u32>(value);
}

@inline export function b2CTZ64(block: u64): u32 {
	return <u32>ctz<u64>(block);
}

@inline export function b2PopCount64(block: u64): i32 {
	return <i32>popcnt<u64>(block);
}

@inline export function b2IsPowerOf2(x: i32): bool {
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
