// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box2d/base.h, src/core.h and src/core.c

import { clock } from "rive/os";
import { b2IsValidFloat } from "./math_functions";

// <float.h> and <stdint.h> limits used across the port
export const FLT_MAX: f32 = reinterpret<f32>(0x7f7fffff);
export const FLT_MIN: f32 = reinterpret<f32>(0x00800000);
export const FLT_EPSILON: f32 = reinterpret<f32>(0x34000000);
export const INFINITY: f32 = reinterpret<f32>(0x7f800000);
export const INT32_MAX: i32 = 2147483647;
export const INT32_MIN: i32 = -2147483648;
export const UINT8_MAX: i32 = 255;
export const UINT16_MAX: i32 = 65535;
export const UINT32_MAX: u32 = 0xffffffff;
export const UINT64_MAX: u64 = 0xffffffffffffffff;

// base.h

export const B2_ENABLE_VALIDATION: bool = false;

/// Used to indicate an unset or invalid index value.
export const B2_NULL_INDEX: i32 = -1;

/// Prototype for user allocation function. The runtime owns memory in the port.
export type b2AllocCallbackFcn = (size: i32, alignment: i32) => u64;

/// Prototype for user free function
export type b2FreeCallbackFcn = (mem: u64, size: i32) => void;

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
export type b2AssertCallbackFcn = (condition: string, fileName: string, lineNumber: i32) => i32;

/// Prototype for user log callback. Used to log warnings.
export type b2LogCallbackFcn = (message: string) => void;

/// Version numbering scheme.
/// See https://semver.org/
export struct b2Version {
	/// Significant changes
	major: i32 = 0;

	/// Incremental changes
	minor: i32 = 0;

	/// Bug fixes
	revision: i32 = 0;

	constructor(major: i32, minor: i32, revision: i32) {
		this.major = major;
		this.minor = minor;
		this.revision = revision;
	}
}

/// Significant API and behavior changes.
export const B2_VERSION_MAJOR: i32 = 3;

/// API changes and features.
export const B2_VERSION_MINOR: i32 = 2;

/// Bug fixes. No API changes.
export const B2_VERSION_REVISION: i32 = 0;

/// True when built with -D B2_ENABLE_ASSERT. Guard hot asserts with it, since rasc still evaluates
/// the argument of a call whose body folds away.
export const B2_ASSERT_ENABLED: bool = isDefined(B2_ENABLE_ASSERT);

/// Assert that a condition is true. Compiled out unless built with -D B2_ENABLE_ASSERT.
@inline
export function B2_ASSERT(condition: bool): void {
	if (B2_ASSERT_ENABLED) {
		if (!condition) {
			b2InternalAssert("", "", 0);
		}
	}
}

/// Starting hash value.
export const B2_HASH_INIT: u32 = 5381;

// src/timer.c, the parts the port keeps

/// Get the absolute number of system ticks. Nanoseconds, as on Linux and macOS.
export function b2GetTicks(): u64 {
	return <u64>(clock() * 1000000000.0);
}

/// Get the milliseconds passed from an initial tick value.
export function b2GetMilliseconds(ticks: u64): f32 {
	let ticksNow = b2GetTicks();
	return <f32>(<f64>(ticksNow - ticks) / 1000000.0);
}

/// Get the milliseconds passed from an initial tick value. Resets the passed in value to the
/// current tick value.
export function b2GetMillisecondsAndReset(ticks: ref u64): f32 {
	let ticksNow = b2GetTicks();
	let ms = <f32>(<f64>(ticksNow - ticks) / 1000000.0);
	ticks = ticksNow;
	return ms;
}

/// Yield to be used in a busy loop.
export function b2Yield(): void {}

/// Simple djb2 hash function for determinism testing
export function b2Hash(hash: u32, data: Uint8Array, count: i32): u32 {
	let result = hash;
	for (let i = 0; i < count; i++) {
		result = (result << 5) + result + <u32>data[i];
	}

	return result;
}

// core.h

export const B2_DEBUG: i32 = 0;

// Use to validate definitions. Do not take my cookie.
export const B2_SECRET_COOKIE: i32 = 1152023;

export struct b2AtomicInt {
	value: i32 = 0;
}

export struct b2AtomicU32 {
	value: u32 = 0;
}

export struct b2AtomicI64 {
	value: i64 = 0;
}

// Use 64 byte alignment for everything. Needed for tree nodes.
export const B2_ALIGNMENT: i32 = 64;

// core.c

// This allows the user to change the length units at runtime
export let b2_lengthUnitsPerMeter: f32 = 1.0;

/// Box2D bases all length units on meters, but you may need different units for your game.
/// @warning This must be modified before any calls to Box2D
export function b2SetLengthUnitsPerMeter(lengthUnits: f32): void {
	if ((b2IsValidFloat(lengthUnits) && lengthUnits > 0.0) == false) {
		b2Log("invalid input: b2IsValidFloat( lengthUnits ) && lengthUnits > 0.0f in b2SetLengthUnitsPerMeter\n");
		B2_ASSERT(false);
		return;
	}
	b2_lengthUnitsPerMeter = lengthUnits;
}

/// Get the current length units per meter.
export function b2GetLengthUnitsPerMeter(): f32 {
	return b2_lengthUnitsPerMeter;
}

function b2DefaultAssertFcn(_condition: string, _fileName: string, _lineNumber: i32): i32 {
	// return non-zero to break to debugger
	return 1;
}

let b2AssertHandler: b2AssertCallbackFcn = b2DefaultAssertFcn;

/// Override the default assert function
/// @param assertFcn a non-null assert callback
export function b2SetAssertCallback(assertFcn: b2AssertCallbackFcn): void {
	b2AssertHandler = assertFcn;
}

export function b2InternalAssert(condition: string, fileName: string, lineNumber: i32): i32 {
	let result = b2AssertHandler(condition, fileName, lineNumber);
	if (result != 0) {
		unreachable();
	}
	return result;
}

function b2DefaultLogFcn(message: string): void {}

let b2LogHandler: b2LogCallbackFcn = b2DefaultLogFcn;

/// Override the default log function
/// @param logFcn a non-null log callback
export function b2SetLogCallback(logFcn: b2LogCallbackFcn): void {
	b2LogHandler = logFcn;
}

// C formats with vsnprintf; callers pass the finished message.
export function b2Log(message: string): void {
	b2LogHandler(message);
}

/// Get the current version of Box2D
export function b2GetVersion(): b2Version {
	return b2Version(B2_VERSION_MAJOR, B2_VERSION_MINOR, B2_VERSION_REVISION);
}

/// @return true if the library was built with BOX2D_DOUBLE_PRECISION (large world mode)
export function b2IsDoublePrecision(): bool {
	return false;
}

/// This allows the user to override the allocation functions. The runtime owns memory, so the hooks
/// are accepted and ignored.
export function b2SetAllocator(allocFcn: b2AllocCallbackFcn?, freeFcn: b2FreeCallbackFcn?): void {}

/// @return the total bytes allocated by Box2D. The runtime owns memory, so this reads 0.
export function b2GetByteCount(): i64 {
	return 0;
}

/// C float to int casts on the reference target saturate and send NaN to zero, where a wasm cast
/// traps. Use this wherever C could see such a value.
@inline
export function b2FloatToIntSat(a: f32): i32 {
	if (isNaN(a)) {
		return 0;
	}
	if (a >= 2147483648.0) {
		return INT32_MAX;
	}
	if (a <= -2147483648.0) {
		return INT32_MIN;
	}
	return <i32>a;
}

/// Unsigned variant of b2FloatToIntSat.
@inline
export function b2FloatToUIntSat(a: f32): u32 {
	if (isNaN(a) || a <= 0.0) {
		return 0;
	}
	if (a >= 4294967296.0) {
		return UINT32_MAX;
	}
	return <u32>a;
}
