// Ported from box2d src/arena_allocator.h/.c. Allocations return raw usize
// pointers into an ArrayBuffer block, matching C's char* discipline. The heap
// fallback allocates a dedicated ArrayBuffer per entry and pins it on the
// entry so the pointer stays alive. Offsets keep C's 32-byte-multiple
// rounding; base-pointer 32-byte alignment (a SIMD concern) is not guaranteed
// by the AS runtime and is not needed on the scalar lane.

import { b2Array } from "./array";

export class b2ArenaEntry {
    data: usize = 0;
    name: string = "";
    size: i32 = 0;
    usedMalloc: bool = false;
    // Keeps the heap-fallback allocation alive while the entry lives.
    heapRef: ArrayBuffer | null = null;
}

export class b2ArenaAllocator {
    data: ArrayBuffer;
    capacity: i32;
    index: i32;

    allocation: i32;
    maxAllocation: i32;

    entries: b2Array<b2ArenaEntry>;

    constructor() {
        this.data = new ArrayBuffer(0);
        this.capacity = 0;
        this.index = 0;
        this.allocation = 0;
        this.maxAllocation = 0;
        this.entries = new b2Array<b2ArenaEntry>(0);
    }
}

export function b2CreateArenaAllocator(capacity: i32): b2ArenaAllocator {
    assert(capacity >= 0);
    let allocator = new b2ArenaAllocator();
    allocator.capacity = capacity;
    allocator.data = new ArrayBuffer(capacity);
    allocator.allocation = 0;
    allocator.maxAllocation = 0;
    allocator.index = 0;
    allocator.entries = new b2Array<b2ArenaEntry>(32);
    return allocator;
}

export function b2DestroyArenaAllocator(allocator: b2ArenaAllocator): void {
    allocator.entries.clear();
    allocator.data = new ArrayBuffer(0);
    allocator.capacity = 0;
    allocator.index = 0;
}

export function b2AllocateArenaItem(alloc: b2ArenaAllocator, size: i32, name: string): usize {
    // ensure allocation is 32 byte aligned to support 256-bit SIMD
    let size32 = ((size - 1) | 0x1f) + 1;

    let entry = new b2ArenaEntry();
    entry.size = size32;
    entry.name = name;
    if (alloc.index + size32 > alloc.capacity) {
        // fall back to the heap (undesirable)
        let heap = new ArrayBuffer(size32);
        entry.heapRef = heap;
        entry.data = changetype<usize>(heap);
        entry.usedMalloc = true;
    } else {
        entry.data = changetype<usize>(alloc.data) + alloc.index;
        entry.usedMalloc = false;
        alloc.index += size32;
    }

    alloc.allocation += size32;
    if (alloc.allocation > alloc.maxAllocation) {
        alloc.maxAllocation = alloc.allocation;
    }

    alloc.entries.push(entry);
    return entry.data;
}

export function b2FreeArenaItem(alloc: b2ArenaAllocator, mem: usize): void {
    let entryCount = alloc.entries.count;
    assert(entryCount > 0);
    let entry = alloc.entries.get(entryCount - 1);
    assert(mem == entry.data);
    if (entry.usedMalloc) {
        entry.heapRef = null;
    } else {
        alloc.index -= entry.size;
    }
    alloc.allocation -= entry.size;
    alloc.entries.pop();
}

export function b2GrowArena(alloc: b2ArenaAllocator): void {
    // Stack must not be in use
    assert(alloc.allocation == 0);

    if (alloc.maxAllocation > alloc.capacity) {
        alloc.capacity = alloc.maxAllocation + alloc.maxAllocation / 2;
        alloc.data = new ArrayBuffer(alloc.capacity);
    }
}

export function b2GetArenaCapacity(alloc: b2ArenaAllocator): i32 {
    return alloc.capacity;
}

export function b2GetArenaAllocation(alloc: b2ArenaAllocator): i32 {
    return alloc.allocation;
}

export function b2GetMaxArenaAllocation(alloc: b2ArenaAllocator): i32 {
    return alloc.maxAllocation;
}
