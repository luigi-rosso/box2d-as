// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/broad_phase.c, the parts that read the world. The rest of broad_phase.h and
// broad_phase.c lives in lib/broad_phase_core.as, which this re-exports, so it can be checked without a world.

import { B2_NULL_INDEX } from "./core";
import { B2_MAX_TASKS } from "./constants";
import { b2MaxInt } from "./math_functions";
import { b2ShapeId } from "./id";
import { b2DynamicTree_ClearMoved, b2DynamicTree_Rebuild, b2HasTreeMoved, b2NeedsRebuild } from "./dynamic_tree";
import { b2_dynamicBody, b2_kinematicBody, b2_staticBody } from "./types";
import {
	B2_CROSS_SEED_COUNT,
	b2CrossPairsTask,
	b2GatherCrossSeeds,
	b2GatherMovedSiblings,
	b2NodePair,
	b2PairContext,
	b2RadixSortKeys,
	b2SelfPairsTask,
} from "./broad_phase_core";
import { b2ShouldBodiesCollide } from "./body";
import { b2CanCollide, b2CreateContact } from "./contact";
import { b2ParallelFor } from "./parallel_for";
import { b2World } from "./physics_world";
import { b2ShouldShapesCollide } from "./shape";

export * from "./broad_phase_core";

// The world's part of C's b2FlushCandidatePairs: the tests a candidate pair passes before a contact is made
export class b2WorldPairFilter {
	world: b2World;

	constructor(world: b2World) {
		this.world = world;
	}

	shouldCreatePair(shapeIdA: i32, shapeIdB: i32): bool {
		let world = this.world;
		let shapes = world.shapes;

		let bodyIdA = shapes[shapeIdA].bodyId;
		let bodyIdB = shapes[shapeIdB].bodyId;

		// Are the shapes on the same body?
		if (bodyIdA == bodyIdB) {
			return false;
		}

		// Sensors are handled elsewhere
		if (shapes[shapeIdA].sensorIndex != B2_NULL_INDEX || shapes[shapeIdB].sensorIndex != B2_NULL_INDEX) {
			return false;
		}

		if (b2ShouldShapesCollide(shapes[shapeIdA].filter, shapes[shapeIdB].filter) == false) {
			return false;
		}

		if (b2CanCollide(shapes[shapeIdA].type, shapes[shapeIdB].type) == false) {
			// For example, no segment vs segment collision
			return false;
		}

		// Does a joint override collision?
		if (b2ShouldBodiesCollide(world, bodyIdA, bodyIdB) == false) {
			return false;
		}

		// Custom user filter
		if (shapes[shapeIdA].enableCustomFiltering || shapes[shapeIdB].enableCustomFiltering) {
			let customFilterFcn = world.customFilterFcn;
			if (customFilterFcn != null) {
				let idA = b2ShapeId(shapeIdA + 1, world.worldId, shapes[shapeIdA].generation);
				let idB = b2ShapeId(shapeIdB + 1, world.worldId, shapes[shapeIdB].generation);
				let shouldCollide = customFilterFcn(idA, idB);
				if (shouldCollide == false) {
					return false;
				}
			}
		}

		return true;
	}
}

// C's pair contexts live on each task's stack; here one per worker, kept across steps.
const b2_pairContexts = new Array<b2PairContext<b2WorldPairFilter>>();

function b2GetPairContext(world: b2World, workerIndex: i32): b2PairContext<b2WorldPairFilter> {
	while (b2_pairContexts.length <= workerIndex) {
		let index = b2_pairContexts.length;
		b2_pairContexts.push(
			new b2PairContext<b2WorldPairFilter>(new b2WorldPairFilter(world), world.broadPhase.pairSet, world.taskContexts[index].pairKeys),
		);
	}

	let context = b2_pairContexts[workerIndex];
	context.filter.world = world;
	context.pairSet = world.broadPhase.pairSet;
	context.pairKeys = world.taskContexts[workerIndex].pairKeys;
	return context;
}

// b2SelfPairsTask as b2ParallelFor calls it
function b2SelfPairsWorldTask(startIndex: i32, endIndex: i32, workerIndex: i32, world: b2World): void {
	b2SelfPairsTask<b2WorldPairFilter>(startIndex, endIndex, world.broadPhase, b2GetPairContext(world, workerIndex));
}

class b2CrossContext {
	world: b2World;
	seeds: StaticArray<b2NodePair>;
	staticSeedCount: i32 = 0;

	constructor(world: b2World, seeds: StaticArray<b2NodePair>) {
		this.world = world;
		this.seeds = seeds;
	}
}

// b2CrossPairsTask as b2ParallelFor calls it
function b2CrossPairsWorldTask(startIndex: i32, endIndex: i32, workerIndex: i32, crossContext: b2CrossContext): void {
	let world = crossContext.world;
	b2CrossPairsTask<b2WorldPairFilter>(
		startIndex,
		endIndex,
		world.broadPhase,
		crossContext.seeds,
		crossContext.staticSeedCount,
		b2GetPairContext(world, workerIndex),
	);
}

const b2_noSiblings = new StaticArray<i32>(0);

// C's stack buffer for the cross seeds
const b2_crossSeeds = new StaticArray<b2NodePair>(2 * B2_CROSS_SEED_COUNT);
let b2_crossContext: b2CrossContext? = null;

function b2UpdateTreesTask(world: b2World): void {
	b2DynamicTree_Rebuild(world.broadPhase.trees[b2_dynamicBody], false);
	b2DynamicTree_Rebuild(world.broadPhase.trees[b2_kinematicBody], false);
}

// Task that can be done in parallel with the narrow-phase
// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
// Tasks run inline, as with C's default task callback, which returns no task to finish.
function b2EnqueueTreeUpdate(world: b2World): void {
	if (world.taskCount < B2_MAX_TASKS) {
		b2UpdateTreesTask(world);
		world.taskCount += 1;
	} else {
		b2UpdateTreesTask(world);
	}
}

// C's stack allocations grow here and stay for the next step
@inline function b2GrowCapacity(count: i32): i32 {
	return count + count / 4 + 16;
}

export function b2UpdateBroadPhasePairs(world: b2World): void {
	let bp = world.broadPhase;

	let needUpdate = b2HasTreeMoved(bp.trees[b2_staticBody]);
	needUpdate = needUpdate || b2NeedsRebuild(bp.trees[b2_kinematicBody]);
	needUpdate = needUpdate || b2NeedsRebuild(bp.trees[b2_dynamicBody]);

	if (needUpdate == false) {
		return;
	}

	for (let i = 0; i < world.workerCount; ++i) {
		world.taskContexts[i].pairKeys.length = 0;
	}

	// Generate pairs by querying the dynamic body tree against itself and against
	// the kinematic and static trees.
	{
		// Get the sibling pairs of the dynamic body tree that have moved.
		let dynamicTree = bp.trees[b2_dynamicBody];
		let pairCapacity = dynamicTree.nodeEnd / 2;
		if (bp.siblingBuffer.length < pairCapacity) {
			bp.siblingBuffer = new StaticArray<i32>(b2GrowCapacity(pairCapacity));
		}
		bp.movedSiblings = bp.siblingBuffer;
		let dynamicMoveCount = b2GatherMovedSiblings(dynamicTree, bp.movedSiblings);

		// Get seeds for colliding against the static and kinematic trees.
		let crossSeeds = b2_crossSeeds;
		let staticSeedCount = b2GatherCrossSeeds(dynamicTree, bp.trees[b2_staticBody], crossSeeds, 0);
		let kinematicSeedCount = b2GatherCrossSeeds(dynamicTree, bp.trees[b2_kinematicBody], crossSeeds, staticSeedCount);
		let crossMoveCount = staticSeedCount + kinematicSeedCount;

		// Collide the dynamic body tree against the static and kinematic trees.
		let crossContext = b2_crossContext;
		if (crossContext == null) {
			crossContext = new b2CrossContext(world, crossSeeds);
			b2_crossContext = crossContext;
		}
		crossContext.world = world;
		crossContext.staticSeedCount = staticSeedCount;
		b2ParallelFor<b2CrossContext>(world, b2CrossPairsWorldTask, crossMoveCount, 1, crossContext);

		// Collide the dynamic body tree against itself.
		b2ParallelFor<b2World>(world, b2SelfPairsWorldTask, dynamicMoveCount, 64, world);
	}

	b2DynamicTree_ClearMoved(bp.trees[b2_staticBody]);

	// Update stale trees.
	b2EnqueueTreeUpdate(world);

	// Pairs arrive in deterministic order but scrambled relative to body and shape order
	// sorting them here improves solver performance.
	let pairCount = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		pairCount += world.taskContexts[i].pairKeys.length;
	}

	if (bp.pairKeyBuffer.length < pairCount) {
		bp.pairKeyBuffer = new StaticArray<u64>(b2GrowCapacity(b2MaxInt(pairCount, 1)));
	}
	let pairKeys = bp.pairKeyBuffer;
	let keyCount = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		let workerKeys = world.taskContexts[i].pairKeys;
		let workerCount = workerKeys.length;
		for (let k = 0; k < workerCount; ++k) {
			pairKeys[keyCount + k] = workerKeys[k];
		}
		keyCount += workerCount;
	}

	if (pairCount > 1) {
		if (bp.tempKeyBuffer.length < pairCount) {
			bp.tempKeyBuffer = new StaticArray<u64>(b2GrowCapacity(pairCount));
		}
		b2RadixSortKeys(pairKeys, bp.tempKeyBuffer, pairCount);
	}

	for (let i = 0; i < keyCount; ++i) {
		let shapeIdA = <i32>(pairKeys[i] >> 32);
		let shapeIdB = <i32>(pairKeys[i] & 0xffffffff);
		b2CreateContact(world, shapeIdA, shapeIdB);
	}

	bp.movedSiblings = b2_noSiblings;

	// b2ValidateSolverSets is validation, which C builds out
}
