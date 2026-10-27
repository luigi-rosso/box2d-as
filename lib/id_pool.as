// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/id_pool.c

import { B2_ASSERT, B2_ASSERT_ENABLED } from "./core";

export class b2IdPool {
	freeArray: Array<i32> = new Array<i32>();
	nextIndex: i32 = 0;
	// C's b2Array capacity, kept so b2GetIdBytes reports what C reports
	freeCapacity: i32 = 0;
}

export function b2CreateIdPool(): b2IdPool {
	let pool = new b2IdPool();
	pool.freeCapacity = 32;
	return pool;
}

export function b2DestroyIdPool(pool: b2IdPool): void {
	pool.freeArray = new Array<i32>();
	pool.nextIndex = 0;
	pool.freeCapacity = 0;
}

export function b2AllocId(pool: b2IdPool): i32 {
	let count = pool.freeArray.length;
	if (count > 0) {
		let id = pool.freeArray.pop();
		return id;
	}

	let id = pool.nextIndex;
	pool.nextIndex += 1;
	return id;
}

export function b2FreeId(pool: b2IdPool, id: i32): void {
	if (B2_ASSERT_ENABLED) B2_ASSERT(pool.nextIndex > 0);
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= id && id < pool.nextIndex);
	if (pool.freeArray.length >= pool.freeCapacity) {
		pool.freeCapacity = pool.freeCapacity == 0 ? 8 : 2 * pool.freeCapacity;
	}
	pool.freeArray.push(id);
}

export function b2ValidateFreeId(pool: b2IdPool, id: i32): void {}

export function b2ValidateUsedId(pool: b2IdPool, id: i32): void {}

@inline export function b2GetIdCount(pool: b2IdPool): i32 {
	return pool.nextIndex - pool.freeArray.length;
}

@inline export function b2GetIdCapacity(pool: b2IdPool): i32 {
	return pool.nextIndex;
}

@inline export function b2GetIdBytes(pool: b2IdPool): i32 {
	return pool.freeCapacity * sizeof<i32>();
}
