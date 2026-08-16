// Ported from box2d src/bitset.h/.c (b2CountSetBits lives in table.c upstream
// only because of a header quirk; it belongs here).

import { b2PopCount64 } from "./core";

export class b2BitSet {
    bits: Uint64Array;
    blockCapacity: u32;
    blockCount: u32;

    constructor() {
        this.bits = new Uint64Array(0);
        this.blockCapacity = 0;
        this.blockCount = 0;
    }
}

export function b2CreateBitSet(bitCapacity: u32): b2BitSet {
    let bitSet = new b2BitSet();
    bitSet.blockCapacity = (bitCapacity + 64 - 1) / 64;
    bitSet.blockCount = 0;
    bitSet.bits = new Uint64Array(<i32>bitSet.blockCapacity);
    return bitSet;
}

export function b2DestroyBitSet(bitSet: b2BitSet): void {
    bitSet.blockCapacity = 0;
    bitSet.blockCount = 0;
    bitSet.bits = new Uint64Array(0);
}

export function b2SetBitCountAndClear(bitSet: b2BitSet, bitCount: u32): void {
    let blockCount: u32 = (bitCount + 64 - 1) / 64;
    if (bitSet.blockCapacity < blockCount) {
        b2DestroyBitSet(bitSet);
        let newBitCapacity: u32 = bitCount + (bitCount >> 1);
        let fresh = b2CreateBitSet(newBitCapacity);
        bitSet.bits = fresh.bits;
        bitSet.blockCapacity = fresh.blockCapacity;
        bitSet.blockCount = fresh.blockCount;
    }

    bitSet.blockCount = blockCount;
    for (let i: u32 = 0; i < blockCount; i++) {
        bitSet.bits[i] = 0;
    }
}

export function b2GrowBitSet(bitSet: b2BitSet, blockCount: u32): void {
    assert(blockCount > bitSet.blockCount);
    if (blockCount > bitSet.blockCapacity) {
        let oldCapacity = bitSet.blockCapacity;
        bitSet.blockCapacity = blockCount + blockCount / 2;
        let newBits = new Uint64Array(<i32>bitSet.blockCapacity);
        for (let i: u32 = 0; i < oldCapacity; i++) {
            newBits[i] = bitSet.bits[i];
        }
        bitSet.bits = newBits;
    }

    bitSet.blockCount = blockCount;
}

export function b2InPlaceUnion(setA: b2BitSet, setB: b2BitSet): void {
    assert(setA.blockCount == setB.blockCount);
    let blockCount = setA.blockCount;
    for (let i: u32 = 0; i < blockCount; ++i) {
        setA.bits[i] |= setB.bits[i];
    }
}

export function b2CountSetBits(bitSet: b2BitSet): i32 {
    let popCount = 0;
    let blockCount = bitSet.blockCount;
    for (let i: u32 = 0; i < blockCount; ++i) {
        popCount += b2PopCount64(bitSet.bits[i]);
    }
    return popCount;
}

export function b2SetBit(bitSet: b2BitSet, bitIndex: u32): void {
    let blockIndex: u32 = bitIndex / 64;
    assert(blockIndex < bitSet.blockCount);
    bitSet.bits[blockIndex] |= (<u64>1 << (bitIndex % 64));
}

export function b2SetBitGrow(bitSet: b2BitSet, bitIndex: u32): void {
    let blockIndex: u32 = bitIndex / 64;
    if (blockIndex >= bitSet.blockCount) {
        b2GrowBitSet(bitSet, blockIndex + 1);
    }
    bitSet.bits[blockIndex] |= (<u64>1 << (bitIndex % 64));
}

export function b2ClearBit(bitSet: b2BitSet, bitIndex: u32): void {
    let blockIndex: u32 = bitIndex / 64;
    if (blockIndex >= bitSet.blockCount) {
        return;
    }
    bitSet.bits[blockIndex] &= ~(<u64>1 << (bitIndex % 64));
}

export function b2GetBit(bitSet: b2BitSet, bitIndex: u32): bool {
    let blockIndex: u32 = bitIndex / 64;
    if (blockIndex >= bitSet.blockCount) {
        return false;
    }
    return (bitSet.bits[blockIndex] & (<u64>1 << (bitIndex % 64))) != 0;
}

export function b2GetBitSetBytes(bitSet: b2BitSet): i32 {
    return <i32>bitSet.blockCapacity * 8;
}
