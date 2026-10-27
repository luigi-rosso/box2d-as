// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/bitset.c

import { B2_ASSERT, B2_ASSERT_ENABLED } from "./core";
import { b2PopCount64 } from "./ctz";

// Bit set provides fast operations on large arrays of bits.
export class b2BitSet {
	bits: StaticArray<u64> = new StaticArray<u64>(0);
	blockCapacity: u32 = 0;
	blockCount: u32 = 0;
}

// C reassigns the struct by value; the class is refilled in place.
function b2InitBitSet(bitSet: b2BitSet, bitCapacity: u32): void {
	bitSet.blockCapacity = (bitCapacity + sizeof<u64>() * 8 - 1) / (sizeof<u64>() * 8);
	bitSet.blockCount = 0;
	bitSet.bits = new StaticArray<u64>(<i32>bitSet.blockCapacity);
}

export function b2CreateBitSet(bitCapacity: u32): b2BitSet {
	let bitSet = new b2BitSet();
	b2InitBitSet(bitSet, bitCapacity);
	return bitSet;
}

export function b2DestroyBitSet(bitSet: b2BitSet): void {
	bitSet.blockCapacity = 0;
	bitSet.blockCount = 0;
	bitSet.bits = new StaticArray<u64>(0);
}

export function b2SetBitCountAndClear(bitSet: b2BitSet, bitCount: u32): void {
	let blockCount: u32 = (bitCount + sizeof<u64>() * 8 - 1) / (sizeof<u64>() * 8);
	if (bitSet.blockCapacity < blockCount) {
		b2DestroyBitSet(bitSet);
		let newBitCapacity: u32 = bitCount + (bitCount >> 1);
		b2InitBitSet(bitSet, newBitCapacity);
	}

	bitSet.blockCount = blockCount;
	let bits = bitSet.bits;
	for (let i: u32 = 0; i < blockCount; ++i) {
		bits[<i32>i] = 0;
	}
}

export function b2GrowBitSet(bitSet: b2BitSet, blockCount: u32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(blockCount > bitSet.blockCount);
	if (blockCount > bitSet.blockCapacity) {
		let oldCapacity = bitSet.blockCapacity;
		bitSet.blockCapacity = blockCount + blockCount / 2;
		let newBits = new StaticArray<u64>(<i32>bitSet.blockCapacity);
		let oldBits = bitSet.bits;
		for (let i: u32 = 0; i < oldCapacity; ++i) {
			newBits[<i32>i] = oldBits[<i32>i];
		}
		bitSet.bits = newBits;
	}

	bitSet.blockCount = blockCount;
}

export function b2InPlaceUnion(setA: b2BitSet, setB: b2BitSet): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(setA.blockCount == setB.blockCount);
	let blockCount = setA.blockCount;
	let bitsA = setA.bits;
	let bitsB = setB.bits;
	for (let i: u32 = 0; i < blockCount; ++i) {
		bitsA[<i32>i] |= bitsB[<i32>i];
	}
}

// In C this lives in table.c because that file includes the bit helpers
export function b2CountSetBits(bitSet: b2BitSet): i32 {
	let popCount = 0;
	let blockCount = bitSet.blockCount;
	let bits = bitSet.bits;
	for (let i: u32 = 0; i < blockCount; ++i) {
		popCount += b2PopCount64(bits[<i32>i]);
	}

	return popCount;
}

@inline export function b2SetBit(bitSet: b2BitSet, bitIndex: u32): void {
	let blockIndex = bitIndex / 64;
	if (B2_ASSERT_ENABLED) B2_ASSERT(blockIndex < bitSet.blockCount);
	bitSet.bits[<i32>blockIndex] |= (<u64>1) << <u64>(bitIndex % 64);
}

@inline export function b2SetBitGrow(bitSet: b2BitSet, bitIndex: u32): void {
	let blockIndex = bitIndex / 64;
	if (blockIndex >= bitSet.blockCount) {
		b2GrowBitSet(bitSet, blockIndex + 1);
	}
	bitSet.bits[<i32>blockIndex] |= (<u64>1) << <u64>(bitIndex % 64);
}

@inline export function b2ClearBit(bitSet: b2BitSet, bitIndex: u32): void {
	let blockIndex = bitIndex / 64;
	if (blockIndex >= bitSet.blockCount) {
		return;
	}
	bitSet.bits[<i32>blockIndex] &= ~((<u64>1) << <u64>(bitIndex % 64));
}

@inline export function b2GetBit(bitSet: b2BitSet, bitIndex: u32): bool {
	let blockIndex = bitIndex / 64;
	if (blockIndex >= bitSet.blockCount) {
		return false;
	}
	return (bitSet.bits[<i32>blockIndex] & ((<u64>1) << <u64>(bitIndex % 64))) != 0;
}

@inline export function b2GetBitSetBytes(bitSet: b2BitSet): i32 {
	return <i32>bitSet.blockCapacity * sizeof<u64>();
}
