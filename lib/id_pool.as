// Ported from box2d src/id_pool.h/.c.

import { b2IntArray, b2IntArray_Create, b2IntArray_Destroy, b2IntArray_Push, b2IntArray_Pop, b2IntArray_ByteCount } from "./array";

export class b2IdPool {
    freeArray: b2IntArray;
    nextIndex: i32;

    constructor() {
        this.freeArray = new b2IntArray();
        this.nextIndex = 0;
    }
}

export function b2CreateIdPool(): b2IdPool {
    let pool = new b2IdPool();
    pool.freeArray = b2IntArray_Create(32);
    return pool;
}

export function b2DestroyIdPool(pool: b2IdPool): void {
    b2IntArray_Destroy(pool.freeArray);
    pool.freeArray = new b2IntArray();
    pool.nextIndex = 0;
}

export function b2AllocId(pool: b2IdPool): i32 {
    let count = pool.freeArray.count;
    if (count > 0) {
        return b2IntArray_Pop(pool.freeArray);
    }

    let id = pool.nextIndex;
    pool.nextIndex += 1;
    return id;
}

export function b2FreeId(pool: b2IdPool, id: i32): void {
    assert(pool.nextIndex > 0);
    assert(0 <= id && id < pool.nextIndex);
    b2IntArray_Push(pool.freeArray, id);
}

export function b2GetIdCount(pool: b2IdPool): i32 {
    return pool.nextIndex - pool.freeArray.count;
}

export function b2GetIdCapacity(pool: b2IdPool): i32 {
    return pool.nextIndex;
}

export function b2GetIdBytes(pool: b2IdPool): i32 {
    return b2IntArray_ByteCount(pool.freeArray);
}
