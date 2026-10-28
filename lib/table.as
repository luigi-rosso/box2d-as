// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/table.c

import { B2_ASSERT, B2_ASSERT_ENABLED } from "./core";
import { b2RoundUpPowerOf2 } from "./ctz";

@inline export function B2_SHAPE_PAIR_KEY(K1: i32, K2: i32): u64 {
	return K1 < K2 ? (<u64>K1 << 32) | <u64>K2 : (<u64>K2 << 32) | <u64>K1;
}

// I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
// todo try:
// https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/

// I compared with CC on https://jacksonallan.github.io/c_cpp_hash_tables_benchmark/ and got slightly better performance
// in the washer benchmark.
// I compared with verstable across 8 benchmarks and the performance was similar.

@inline export function b2KeyHash(key: u64): u64 {
	// Murmur hash
	let h = key;
	h ^= h >> 33;
	h *= 0xff51afd7ed558ccd;
	h ^= h >> 33;
	h *= 0xc4ceb9fe1a85ec53;
	h ^= h >> 33;
	return h;
}

export struct b2SetItem {
	key: u64 = 0;

	// storing lower 32 bits of hash
	// this is wasteful because I just need to know if the item is occupied
	// I could require the key to be non-zero and use 0 to indicate an empty slot
	// Update: looks like I store this to make growing the table faster, however this is wasteful once
	// the table has hit the high water mark
	// uint32_t hash;
}

export class b2HashSet {
	items: StaticArray<b2SetItem> = new StaticArray<b2SetItem>(0);
	capacity: u32 = 0;
	count: u32 = 0;
}

export function b2CreateSet(capacity: i32): b2HashSet {
	let set = new b2HashSet();

	// Capacity must be a power of 2
	if (capacity > 16) {
		set.capacity = <u32>b2RoundUpPowerOf2(capacity);
	} else {
		set.capacity = 16;
	}

	set.count = 0;
	set.items = new StaticArray<b2SetItem>(<i32>set.capacity);

	return set;
}

export function b2DestroySet(set: b2HashSet): void {
	set.items = new StaticArray<b2SetItem>(0);
	set.count = 0;
	set.capacity = 0;
}

export function b2ClearSet(set: b2HashSet): void {
	set.count = 0;
	let items = set.items;
	let capacity = <i32>set.capacity;
	for (let i = 0; i < capacity; ++i) {
		items[i].key = 0;
	}
}

function b2FindSlot(set: b2HashSet, key: u64, hash: u64): i32 {
	let capacity = set.capacity;
	let index = <u32>hash & (capacity - 1);
	let items = set.items;
	// index is masked by capacity - 1 and items holds capacity items
	while (unchecked(items[<i32>index].key) != 0 && unchecked(items[<i32>index].key) != key) {
		index = (index + 1) & (capacity - 1);
	}

	return <i32>index;
}

function b2AddKeyHaveCapacity(set: b2HashSet, key: u64, hash: u64): void {
	let index = b2FindSlot(set, key, hash);
	let items = set.items;

	if (B2_ASSERT_ENABLED) B2_ASSERT(items[index].key == 0);
	items[index].key = key;
	set.count += 1;
}

function b2GrowTable(set: b2HashSet): void {
	let oldCount = set.count;

	let oldCapacity = set.capacity;
	let oldItems = set.items;

	set.count = 0;
	// Capacity must be a power of 2
	set.capacity = 2 * oldCapacity;
	set.items = new StaticArray<b2SetItem>(<i32>set.capacity);

	// Transfer items into new array
	for (let i: u32 = 0; i < oldCapacity; ++i) {
		let itemKey = oldItems[<i32>i].key;
		if (itemKey == 0) {
			// this item was empty
			continue;
		}

		let hash = b2KeyHash(itemKey);
		b2AddKeyHaveCapacity(set, itemKey, hash);
	}

	if (B2_ASSERT_ENABLED) B2_ASSERT(set.count == oldCount);
}

export function b2ContainsKey(set: b2HashSet, key: u64): bool {
	// key of zero is a sentinel
	let hash = b2KeyHash(key);
	let index = b2FindSlot(set, key, hash);
	return set.items[index].key == key;
}

export function b2ContainsHashedKey(set: b2HashSet, key: u64, hash: u64): bool {
	// key of zero is a sentinel
	let index = b2FindSlot(set, key, hash);
	// b2FindSlot returns a masked index
	return unchecked(set.items[index].key) == key;
}

export function b2GetHashSetBytes(set: b2HashSet): i32 {
	return <i32>set.capacity * <i32>sizeof<b2SetItem>();
}

export function b2AddKey(set: b2HashSet, key: u64): bool {
	// key of zero is a sentinel
	if (B2_ASSERT_ENABLED) B2_ASSERT(key != 0);

	let hash = b2KeyHash(key);
	if (B2_ASSERT_ENABLED) B2_ASSERT(hash != 0);

	let index = b2FindSlot(set, key, hash);
	if (set.items[index].key != 0) {
		// Already in set
		if (B2_ASSERT_ENABLED) B2_ASSERT(set.items[index].key == key);
		return true;
	}

	if (2 * set.count >= set.capacity) {
		b2GrowTable(set);
	}

	b2AddKeyHaveCapacity(set, key, hash);
	return false;
}

// See https://en.wikipedia.org/wiki/Open_addressing
export function b2RemoveKey(set: b2HashSet, key: u64): bool {
	let hash = b2KeyHash(key);
	let i = b2FindSlot(set, key, hash);
	let items = set.items;
	if (items[i].key == 0) {
		// Not in set
		return false;
	}

	// Mark item i as unoccupied
	items[i].key = 0;

	if (B2_ASSERT_ENABLED) B2_ASSERT(set.count > 0);
	set.count -= 1;

	// Attempt to fill item i
	let j = i;
	let capacity = set.capacity;
	for (;;) {
		j = (j + 1) & <i32>(capacity - 1);
		if (items[j].key == 0) {
			break;
		}

		// k is the first item for the hash of j
		let hash_j = b2KeyHash(items[j].key);
		let k = <i32>(hash_j & <u64>(capacity - 1));

		// determine if k lies cyclically in (i,j]
		// i <= j: | i..k..j |
		// i > j: |.k..j  i....| or |....j     i..k.|
		if (i <= j) {
			if (i < k && k <= j) {
				continue;
			}
		} else {
			if (i < k || k <= j) {
				continue;
			}
		}

		// Move j into i
		items[i] = items[j];

		// Mark item j as unoccupied
		items[j].key = 0;

		i = j;
	}

	return true;
}

@inline export function b2GetSetCount(set: b2HashSet): i32 {
	return <i32>set.count;
}

@inline export function b2GetSetCapacity(set: b2HashSet): i32 {
	return <i32>set.capacity;
}
