// Ported from box2d src/table.h/.c. Open-addressed hash set with the exact
// murmur key hash and backward-shift deletion; slot layout is deterministic
// so whole-table state can be compared against the C reference.
// b2CountSetBits (which upstream parks here) already lives in bitset.as.

import { b2RoundUpPowerOf2 } from "./core";

export function B2_SHAPE_PAIR_KEY(k1: i32, k2: i32): u64 {
    return k1 < k2 ? (<u64>k1 << 32) | <u64>k2 : (<u64>k2 << 32) | <u64>k1;
}

export class b2HashSet {
    items: Uint64Array = new Uint64Array(0);
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
    set.items = new Uint64Array(<i32>set.capacity);

    return set;
}

export function b2DestroySet(set: b2HashSet): void {
    set.items = new Uint64Array(0);
    set.count = 0;
    set.capacity = 0;
}

export function b2ClearSet(set: b2HashSet): void {
    set.count = 0;
    set.items.fill(0);
}

function b2KeyHash(key: u64): u64 {
    // Murmur hash
    let h: u64 = key;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccd;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53;
    h ^= h >> 33;
    return h;
}

function b2FindSlot(set: b2HashSet, key: u64, hash: u64): i32 {
    let capacity: u32 = set.capacity;
    let index: u32 = <u32>hash & (capacity - 1);
    let items = set.items;
    while (unchecked(items[index]) != 0 && unchecked(items[index]) != key) {
        index = (index + 1) & (capacity - 1);
    }

    return <i32>index;
}

function b2AddKeyHaveCapacity(set: b2HashSet, key: u64, hash: u64): void {
    let index = b2FindSlot(set, key, hash);
    assert(set.items[index] == 0);
    set.items[index] = key;
    set.count += 1;
}

function b2GrowTable(set: b2HashSet): void {
    let oldCount: u32 = set.count;
    let oldCapacity: u32 = set.capacity;
    let oldItems = set.items;

    set.count = 0;
    // Capacity must be a power of 2
    set.capacity = 2 * oldCapacity;
    set.items = new Uint64Array(<i32>set.capacity);

    // Transfer items into new array
    for (let i: u32 = 0; i < oldCapacity; ++i) {
        let key = unchecked(oldItems[i]);
        if (key == 0) {
            // this item was empty
            continue;
        }

        let hash = b2KeyHash(key);
        b2AddKeyHaveCapacity(set, key, hash);
    }

    assert(set.count == oldCount);
}

export function b2ContainsKey(set: b2HashSet, key: u64): bool {
    // key of zero is a sentinel
    assert(key != 0);
    let hash = b2KeyHash(key);
    let index = b2FindSlot(set, key, hash);
    return set.items[index] == key;
}

export function b2GetHashSetBytes(set: b2HashSet): i32 {
    return <i32>set.capacity * 8;
}

// Returns true if key was already in set.
export function b2AddKey(set: b2HashSet, key: u64): bool {
    // key of zero is a sentinel
    assert(key != 0);

    let hash = b2KeyHash(key);
    assert(hash != 0);

    let index = b2FindSlot(set, key, hash);
    if (set.items[index] != 0) {
        // Already in set
        assert(set.items[index] == key);
        return true;
    }

    if (2 * set.count >= set.capacity) {
        b2GrowTable(set);
    }

    b2AddKeyHaveCapacity(set, key, hash);
    return false;
}

// Returns true if the key was found.
// See https://en.wikipedia.org/wiki/Open_addressing
export function b2RemoveKey(set: b2HashSet, key: u64): bool {
    let hash = b2KeyHash(key);
    let i = b2FindSlot(set, key, hash);
    let items = set.items;
    if (unchecked(items[i]) == 0) {
        // Not in set
        return false;
    }

    // Mark item i as unoccupied
    unchecked(items[i] = 0);

    assert(set.count > 0);
    set.count -= 1;

    // Attempt to fill item i
    let j = i;
    let capacity: u32 = set.capacity;
    for (; ;) {
        j = <i32>((<u32>j + 1) & (capacity - 1));
        if (unchecked(items[j]) == 0) {
            break;
        }

        // k is the first item for the hash of j
        let hash_j = b2KeyHash(unchecked(items[j]));
        let k = <i32>(<u32>hash_j & (capacity - 1));

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
        unchecked(items[i] = items[j]);

        // Mark item j as unoccupied
        unchecked(items[j] = 0);

        i = j;
    }

    return true;
}

export function b2GetSetCount(set: b2HashSet): i32 {
    return <i32>set.count;
}

export function b2GetSetCapacity(set: b2HashSet): i32 {
    return <i32>set.capacity;
}
