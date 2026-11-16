// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Box2D's threaded step on parallel calls: pair finding, the narrow phase, the solver stages with the tree update and
// finalize each run as the parts of one call over the world's own memory, as C's workers do. notes/threading.md has
// the design and the results.

import { Parallel, Part, SharedList } from "rive/parallel";

import { B2_MAX_WORKERS } from "./constants";
import {
	b2FinalizeBodiesTask,
	b2FinishParallelFinalize,
	b2PrepareParallelFinalize,
	b2SetParallelFinalize,
	b2SetParallelSolve,
	b2SolverWorker,
	b2StepContext,
} from "./solver";
import {
	b2BuildTreeUpdate,
	b2CollectParallelPairs,
	b2CrossContext,
	b2CrossPairsWorldTask,
	b2PrepareParallelPairs,
	b2SelfPairsWorldTask,
	b2SetParallelPairs,
} from "./broad_phase";
import { b2CollideTask, b2GetWorldFromId, b2SetParallelCollide, b2World, b2World_SetWorkerCount } from "./physics_world";
import { b2WorldId } from "./id";

export const b2_parallelSolve: i32 = 1;
export const b2_parallelCollide: i32 = 2;
export const b2_parallelFinalize: i32 = 4;
export const b2_parallelPairs: i32 = 8;
export const b2_parallelAll: i32 = 15;

// Per step totals, for a bench's result line
export class b2ParallelStats {
	solves: i32 = 0;
	collides: i32 = 0;
	finalizes: i32 = 0;
	deferredBodies: i64 = 0;
	pairs: i32 = 0;
	pairOverflows: i32 = 0;
}

export const b2_parallelStats = new b2ParallelStats();

// The most parts a call runs on
let b2_partLimit = B2_MAX_WORKERS;

// Whether world runs task on parallel calls, and on how many parts at most: each needs a task context of its own
@inline function b2RunsParallel(world: b2World, task: i32): bool {
	return (world.parallelTasks & task) != 0;
}

@inline function b2MaxParts(world: b2World): i32 {
	return min(b2_partLimit, world.workerCount);
}

// C's solver task with a part as each worker: part 0 orchestrates the stages and the others claim their blocks. The
// last part first builds a deferred tree update, as C's tree task overlaps the solver, and joins at the stage it finds.
@parallel.inplace export unsafe function b2ParallelSolverTask(part: Part, context: b2StepContext): void {
	if (part.index == part.count - 1) {
		b2BuildTreeUpdate(context.world);
	}
	b2SolverWorker(context, part.index);
}

function b2ParallelSolve(context: b2StepContext): bool {
	let world = context.world;
	if (b2RunsParallel(world, b2_parallelSolve) == false) {
		return false;
	}

	Parallel.runUpTo(b2MaxParts(world), b2ParallelSolverTask, context);
	b2_parallelStats.solves += 1;
	return true;
}

// b2ParallelFor's blocks over the awake contacts, claimed by whichever part asks next
@parallel.inplace export unsafe function b2ParallelCollideTask(part: Part, context: b2StepContext, contactCount: i32, blockSize: i32): void {
	let workerIndex = part.index;
	for (let start = part.claim(contactCount, blockSize); start < contactCount; start = part.claim(contactCount, blockSize)) {
		b2CollideTask(start, min(start + blockSize, contactCount), workerIndex, context);
	}
}

// b2ParallelFor's block size for itemCount items of at least minRange each over workerCount workers
function b2ParallelForBlockSize(itemCount: i32, minRange: i32, workerCount: i32): i32 {
	let maxBlockCount = 8 * workerCount;
	if (itemCount <= minRange * maxBlockCount) {
		return minRange;
	}
	return (itemCount + maxBlockCount - 1) / maxBlockCount;
}

function b2ParallelCollide(context: b2StepContext, contactCount: i32): bool {
	let world = context.world;

	// Parts cannot call the world's callbacks
	let custom = world.preSolveFcn != null || world.frictionCallback != null || world.restitutionCallback != null;
	if (custom || b2RunsParallel(world, b2_parallelCollide) == false) {
		return false;
	}

	Parallel.runUpTo(b2MaxParts(world), b2ParallelCollideTask, context, contactCount, b2ParallelForBlockSize(contactCount, 64, world.workerCount));
	b2_parallelStats.collides += 1;
	return true;
}

@parallel.inplace export unsafe function b2ParallelFinalizeTask(part: Part, context: b2StepContext, bodyCount: i32, blockSize: i32): void {
	let workerIndex = part.index;
	for (let start = part.claim(bodyCount, blockSize); start < bodyCount; start = part.claim(bodyCount, blockSize)) {
		b2FinalizeBodiesTask(start, min(start + blockSize, bodyCount), workerIndex, context);
	}
}

function b2ParallelFinalize(context: b2StepContext, bodyCount: i32): bool {
	let world = context.world;
	if (b2RunsParallel(world, b2_parallelFinalize) == false) {
		return false;
	}

	b2PrepareParallelFinalize(world, context, bodyCount);
	Parallel.runUpTo(b2MaxParts(world), b2ParallelFinalizeTask, context, bodyCount, b2ParallelForBlockSize(bodyCount, 64, world.workerCount));
	b2FinishParallelFinalize(world, context);

	let stats = b2_parallelStats;
	stats.finalizes += 1;
	for (let i = 0; i < world.workerCount; ++i) {
		stats.deferredBodies += world.taskContexts[i].deferredBodyCount;
	}
	return true;
}

// b2UpdateBroadPhasePairs' two parallel fors as one claim loop, so the parts never wait for each other: first the
// cross tree seeds, one each, then the moved siblings of the dynamic tree in blocks of selfBlockSize
@parallel.inplace export function b2ParallelPairsTask(
	part: Part,
	world: b2World,
	crossContext: b2CrossContext,
	crossCount: i32,
	selfCount: i32,
	selfBlockSize: i32,
): void {
	let workerIndex = part.index;
	let taskCount = crossCount + (selfCount + selfBlockSize - 1) / selfBlockSize;
	for (let i = part.claim(taskCount, 1); i < taskCount; i = part.claim(taskCount, 1)) {
		if (i < crossCount) {
			b2CrossPairsWorldTask(i, i + 1, workerIndex, crossContext);
		} else {
			let start = (i - crossCount) * selfBlockSize;
			b2SelfPairsWorldTask(start, min(start + selfBlockSize, selfCount), workerIndex, world);
		}
	}
}

// Where the parts append new pair keys, made larger after a step that filled it
let b2_pairKeys = new SharedList<u64>(1024);

function b2ParallelPairs(world: b2World, crossContext: b2CrossContext, crossCount: i32, selfCount: i32): bool {
	// Parts cannot call a custom filter
	if (world.customFilterFcn != null || b2RunsParallel(world, b2_parallelPairs) == false) {
		return false;
	}

	let keys = b2_pairKeys;
	b2PrepareParallelPairs(world, keys);
	Parallel.runUpTo(b2MaxParts(world), b2ParallelPairsTask, world, crossContext, crossCount, selfCount, b2ParallelForBlockSize(selfCount, 64, world.workerCount));
	b2_parallelStats.pairs += 1;

	// Finding pairs changes nothing else, so parts that ran out of room mean finding them again on the main lane
	if (b2CollectParallelPairs(world, keys) == false) {
		b2_pairKeys = new SharedList<u64>(4 * (keys.capacity + keys.overflow));
		b2_parallelStats.pairOverflows += 1;
		return false;
	}
	return true;
}

// Runs worldId's step on parallel calls: tasks picks among b2_parallelSolve (the solver stages and the tree update),
// b2_parallelCollide (the narrow phase), b2_parallelFinalize (finalize bodies, continuous included) and
// b2_parallelPairs (pair finding),
// or b2_parallelAll; 0 turns it off. The world gets a task context per part, which leaves its results unchanged as
// C's are at any worker count. Call it from init or later: Parallel.size() is 1 in a file's top level code.
export function b2World_EnableParallel(worldId: b2WorldId, tasks: i32): void {
	let world = b2GetWorldFromId(worldId);
	world.parallelTasks = tasks;
	world.deferTreeUpdate = (tasks & b2_parallelSolve) != 0;
	if (tasks == 0) {
		return;
	}

	b2World_SetWorkerCount(worldId, Parallel.size());

	// Every world steps through these; one that has not asked for a task answers false and runs it inline
	b2SetParallelSolve((context: b2StepContext): bool => b2ParallelSolve(context));
	b2SetParallelCollide((context: b2StepContext, contactCount: i32): bool => b2ParallelCollide(context, contactCount));
	b2SetParallelFinalize((context: b2StepContext, bodyCount: i32): bool => b2ParallelFinalize(context, bodyCount));
	b2SetParallelPairs((world: b2World, crossContext: b2CrossContext, crossCount: i32, selfCount: i32): bool =>
		b2ParallelPairs(world, crossContext, crossCount, selfCount),
	);
}

// Runs later calls on at most parts parts, 0 on as many as Parallel.size()
export function b2SetParallelLimit(parts: i32): void {
	b2_partLimit = parts > 0 ? parts : B2_MAX_WORKERS;
}

// Fields for a bench's result line
export function b2ParallelReport(): string {
	return " parts=" + Parallel.size().toString() + " parSolves=" + b2_parallelStats.solves.toString() + " parCollides=" +
		b2_parallelStats.collides.toString() + " parFinalizes=" + b2_parallelStats.finalizes.toString() + " parDeferredBodies=" + b2_parallelStats.deferredBodies.toString() + " parPairs=" + b2_parallelStats.pairs.toString() +
		" parPairOverflows=" + b2_parallelStats.pairOverflows.toString();
}
