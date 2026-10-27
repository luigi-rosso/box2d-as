// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/table.as, mirrors verify/cref/table.c

import { vtag, vi32, vu32, vu64, vbool, vend, vseed, vrand32, vrand64, vrandint } from "./common";
import {
	B2_SHAPE_PAIR_KEY,
	b2HashSet,
	b2KeyHash,
	b2CreateSet,
	b2DestroySet,
	b2ClearSet,
	b2AddKey,
	b2RemoveKey,
	b2ContainsKey,
	b2ContainsHashedKey,
	b2GetHashSetBytes,
	b2GetSetCount,
	b2GetSetCapacity,
} from "../../lib/table";

export { frameBoundary } from "./common";

const keys = new StaticArray<u64>(1 << 16);
let keyCount = 0;

// One line per occupied slot, so slot placement and backward shift deletion are compared exactly.
function emitSet(tag: string, set: b2HashSet): void {
	vtag(tag);
	vu32(set.capacity);
	vu32(set.count);
	vi32(b2GetHashSetBytes(set));
	vi32(b2GetSetCount(set));
	vi32(b2GetSetCapacity(set));
	vend();
	for (let i: u32 = 0; i < set.capacity; ++i) {
		let key = set.items[<i32>i].key;
		if (key != 0) {
			vtag("slot");
			vu32(i);
			vu64(key);
			vend();
		}
	}
}

function makeKey(): u64 {
	let kind = vrand32() % 4;
	if (kind == 3) {
		let key = vrand64();
		return key == 0 ? 1 : key;
	}

	// Pair keys from small increasing shape indices, as the broad phase makes them
	let s1 = vrandint(0, 64 + keyCount / 4);
	let s2 = vrandint(0, 64 + keyCount / 4);
	if (s1 == s2) {
		s2 += 1;
	}
	return B2_SHAPE_PAIR_KEY(s1, s2);
}

export function begin(): void {
	vseed(4);

	let capacities: StaticArray<i32> = [-3, 0, 1, 16, 17, 31, 32, 33, 100, 1000];
	for (let i = 0; i < capacities.length; ++i) {
		let set = b2CreateSet(capacities[i]);
		vtag("cap");
		vi32(capacities[i]);
		vu32(set.capacity);
		vend();
		b2DestroySet(set);
	}

	let pairs: StaticArray<i32> = [0, 1, 1, 0, 5, 5, 4194303, 0, -1, 7, 7, -1, -5, -9, 2147483647, 100];
	for (let i = 0; i < 8; ++i) {
		let key = B2_SHAPE_PAIR_KEY(pairs[2 * i], pairs[2 * i + 1]);
		vtag("pair");
		vu64(key);
		vu64(b2KeyHash(key));
		vend();
	}
}

// The host calls these with a frame boundary between phases, so the set survives collections
let set = new b2HashSet();
let roundIndex = 0;

export function beginRound(round: i32): void {
	roundIndex = round;
	set = b2CreateSet(vrandint(0, 64));
	keyCount = 0;
	emitSet("create", set);
}

export function phase(): void {
	let adds = vrandint(0, 400);
	for (let k = 0; k < adds; ++k) {
		let key = makeKey();
		if (keyCount > 0 && vrand32() % 8 == 0) {
			key = keys[vrandint(0, keyCount)];
		}
		let found = b2AddKey(set, key);
		if (keyCount < 1 << 16) {
			keys[keyCount++] = key;
		}
		vtag("add");
		vu64(key);
		vu64(b2KeyHash(key));
		vbool(found);
		vend();
	}
	emitSet("added", set);

	let removes = keyCount > 0 ? vrandint(0, adds / 2 + 2) : 0;
	for (let k = 0; k < removes; ++k) {
		let key = vrand32() % 10 == 0 ? makeKey() : keys[vrandint(0, keyCount)];
		let found = b2RemoveKey(set, key);
		vtag("remove");
		vu64(key);
		vbool(found);
		vend();
	}
	emitSet("removed", set);

	let queries = vrandint(0, 100);
	for (let k = 0; k < queries; ++k) {
		let key = vrand32() % 3 == 0 || keyCount == 0 ? makeKey() : keys[vrandint(0, keyCount)];
		vtag("has");
		vu64(key);
		vbool(b2ContainsKey(set, key));
		vbool(b2ContainsHashedKey(set, key, b2KeyHash(key)));
		vend();
	}
}

export function endRound(): void {
	if (roundIndex % 2 == 1) {
		b2ClearSet(set);
		emitSet("clear", set);
		for (let k = 0; k < 50; ++k) {
			let key = makeKey();
			vtag("add");
			vu64(key);
			vbool(b2AddKey(set, key));
			vend();
		}
		emitSet("refill", set);
	}

	b2DestroySet(set);
}
