// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box2d/id.h

// A struct cannot be a generic argument, so B2_IS_NULL( id ) is id.index1 == 0 and
// B2_ID_EQUALS( a, b ) is a == b, which compares every field.

/// World id references a world instance. This should be treated as an opaque handle.
export struct b2WorldId {
	index1: u16 = 0;
	generation: u16 = 0;

	constructor(index1: u16, generation: u16) {
		this.index1 = index1;
		this.generation = generation;
	}
}

/// Body id references a body instance. This should be treated as an opaque handle.
export struct b2BodyId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;

	constructor(index1: i32, world0: u16, generation: u16) {
		this.index1 = index1;
		this.world0 = world0;
		this.generation = generation;
	}
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
export struct b2ShapeId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;

	constructor(index1: i32, world0: u16, generation: u16) {
		this.index1 = index1;
		this.world0 = world0;
		this.generation = generation;
	}
}

/// Chain id references a chain instance. This should be treated as an opaque handle.
export struct b2ChainId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;

	constructor(index1: i32, world0: u16, generation: u16) {
		this.index1 = index1;
		this.world0 = world0;
		this.generation = generation;
	}
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
export struct b2JointId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;

	constructor(index1: i32, world0: u16, generation: u16) {
		this.index1 = index1;
		this.world0 = world0;
		this.generation = generation;
	}
}

/// Contact id references a contact instance. This should be treated as an opaque handle.
export struct b2ContactId {
	index1: i32 = 0;
	world0: u16 = 0;
	padding: i16 = 0;
	generation: u32 = 0;
}

/// Use these to make your identifiers null. You may also use zero initialization to get null.
export const b2_nullWorldId: b2WorldId = b2WorldId();
export const b2_nullBodyId: b2BodyId = b2BodyId();
export const b2_nullShapeId: b2ShapeId = b2ShapeId();
export const b2_nullChainId: b2ChainId = b2ChainId();
export const b2_nullJointId: b2JointId = b2JointId();
export const b2_nullContactId: b2ContactId = b2ContactId();

/// Store a world id into a uint32_t.
export function b2StoreWorldId(id: b2WorldId): u32 {
	return (<u32>id.index1 << 16) | <u32>id.generation;
}

/// Load a uint32_t into a world id.
export function b2LoadWorldId(x: u32): b2WorldId {
	let id = b2WorldId(<u16>(x >> 16), <u16>x);
	return id;
}

/// Store a body id into a uint64_t.
export function b2StoreBodyId(id: b2BodyId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a body id.
export function b2LoadBodyId(x: u64): b2BodyId {
	let id = b2BodyId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
	return id;
}

/// Store a shape id into a uint64_t.
export function b2StoreShapeId(id: b2ShapeId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a shape id.
export function b2LoadShapeId(x: u64): b2ShapeId {
	let id = b2ShapeId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
	return id;
}

/// Store a chain id into a uint64_t.
export function b2StoreChainId(id: b2ChainId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a chain id.
export function b2LoadChainId(x: u64): b2ChainId {
	let id = b2ChainId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
	return id;
}

/// Store a joint id into a uint64_t.
export function b2StoreJointId(id: b2JointId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a joint id.
export function b2LoadJointId(x: u64): b2JointId {
	let id = b2JointId(<i32>(x >> 32), <u16>(x >> 16), <u16>x);
	return id;
}

/// Store a contact id.
export function b2StoreContactId(id: b2ContactId, values: ref FixedArray<u32, 3>): void {
	values[0] = <u32>id.index1;
	values[1] = <u32>id.world0;
	values[2] = <u32>id.generation;
}

/// Load a contact id.
export function b2LoadContactId(values: FixedArray<u32, 3>): b2ContactId {
	let id: b2ContactId;
	id.index1 = <i32>values[0];
	id.world0 = <u16>values[1];
	id.padding = 0;
	id.generation = <u32>values[2];
	return id;
}
