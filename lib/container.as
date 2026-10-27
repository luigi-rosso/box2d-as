// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/container.h

// b2Array(T) is the builtin Array<T>; these helpers are generic over the array type because a
// struct cannot be a generic argument. Raising length keeps stale elements, as C's are uninitialized.

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";

// Add a zeroed element and return its index. C's b2Array_Emplace returns a pointer to it.
@inline export function b2Array_AddIndex<A>(a: A): i32 {
	let index = a.length;
	let zero: valueof<A>;
	a.push(zero);
	return index;
}

export { b2Array_AddIndex as b2Array_Emplace };

@inline export function b2Array_Resize<A>(a: A, n: i32): void {
	a.length = n;
}

export function b2Array_ResizeAndSetZero<A>(a: A, n: i32): void {
	a.length = n;
	let zero: valueof<A>;
	for (let i = 0; i < n; ++i) {
		a[i] = zero;
	}
}

// Append the first n values of src.
export function b2Array_Append<A, S>(a: A, src: S, n: i32): void {
	let count = a.length;
	a.length = count + n;
	for (let i = 0; i < n; ++i) {
		a[count + i] = src[i];
	}
}

// Remove an element by swapping with the last element. If the index is the last element it returns
// B2_NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
export unsafe function b2Array_RemoveSwap<A>(a: A, index: i32): i32 {
	if (B2_ASSERT_ENABLED) B2_ASSERT(0 <= index && index < a.length);

	let count = a.length - 1;
	if (index != count) {
		if (isValueClass<valueof<A>>()) {
			// One copy of the whole element, as C's memcpy; a struct assignment moves it field by field
			let data = load<usize>(changetype<usize>(a) + offsetof<Array<i32>>("dataStart"));
			let size = sizeof<valueof<A>>();
			memory.copy(data + <usize>index * size, data + <usize>count * size, size);
		} else {
			a[index] = a[count];
		}
		a.length = count;
		return count;
	}

	a.length = count;
	return B2_NULL_INDEX;
}

// B2_SWAP on two elements of one array, since rasc refuses a ref to a primitive local.
@inline export function b2SwapElements<A>(a: A, i: i32, j: i32): void {
	let t = a[i];
	a[i] = a[j];
	a[j] = t;
}
