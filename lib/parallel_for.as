// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/parallel_for.c

import { B2_ASSERT } from "./core";
import { B2_MAX_TASKS, B2_MAX_WORKERS } from "./constants";
import { b2World } from "./physics_world";

// Callback invoked by b2ParallelFor to process a range of items. C's void* context is a typed
// argument so the callback can be a plain function, which allocates nothing.
export type b2ParallelForCallback<C> = (startIndex: i32, endIndex: i32, workerIndex: i32, context: C) => void;

// Divide [0, itemCount) into blocks and process them. Tasks run inline, as C's default task
// callback does: the first task claims every block, so all items run on worker 0 in order.
export function b2ParallelFor<C>(
	world: b2World,
	callback: b2ParallelForCallback<C>,
	itemCount: i32,
	minRange: i32,
	context: C,
): void {
	if (itemCount <= 0) {
		return;
	}

	B2_ASSERT(minRange > 0);

	let workerCount = world.workerCount;
	B2_ASSERT(0 < workerCount && workerCount <= B2_MAX_WORKERS);

	// Target multiple blocks per worker to reduce thread stalls.
	// block size grows once items exceed maxBlockCount * minRange
	// so the block count stays bounded and per-block sync overhead stays low.
	let blocksPerWorker = 8;
	let maxBlockCount = blocksPerWorker * workerCount;

	let blockSize: i32;
	let blockCount: i32;
	if (itemCount <= minRange * maxBlockCount) {
		blockSize = minRange;
		blockCount = (itemCount + blockSize - 1) / blockSize;
	} else {
		blockSize = (itemCount + maxBlockCount - 1) / maxBlockCount;
		blockCount = (itemCount + blockSize - 1) / blockSize;
	}

	B2_ASSERT(blockCount >= 1);
	B2_ASSERT(blockSize * blockCount >= itemCount);

	// No point enqueueing more tasks than blocks.
	let taskCount = workerCount < blockCount ? workerCount : blockCount;

	for (let i = 0; i < taskCount; ++i) {
		if (world.taskCount < B2_MAX_TASKS) {
			world.taskCount += 1;
		}
	}

	// b2ParallelForTrampoline for task 0, which drains every block before task 1 starts
	for (let blockIndex = 0; blockIndex < blockCount; ++blockIndex) {
		let start = blockIndex * blockSize;
		let end = start + blockSize;
		if (end > itemCount) {
			end = itemCount;
		}

		callback(start, end, 0, context);
	}
}
