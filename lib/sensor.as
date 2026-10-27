// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/sensor.c

import { B2_ASSERT, B2_NULL_INDEX, FLT_EPSILON } from "./core";
import { b2BitSet, b2InPlaceUnion, b2SetBit, b2SetBitCountAndClear } from "./bitset";
import { b2CTZ64 } from "./ctz";
import { b2Array_RemoveSwap } from "./container";
import { b2InvMulWorldTransforms, b2Transform, b2WorldTransform } from "./math_functions";
import { b2DistanceInput, b2SimplexCache } from "./collision";
import { b2ComputeShapeDistance } from "./distance";
import { b2DynamicTree_OverlapAABB } from "./dynamic_tree";
import { b2ShapeId } from "./id";
import { b2SensorBeginTouchEvent, b2SensorEndTouchEvent } from "./types";
import { b2GetBodyTransform, b2GetBodyTransformQuick } from "./body";
import { b2MakeShapeDistanceProxy, b2ShouldShapesCollide } from "./shape";
import { b2ParallelFor } from "./parallel_for";
import { b2World } from "./physics_world";
import { b2_disabledSet } from "./solver_set";

// Used to track shapes that hit sensors using time of impact
export struct b2SensorHit {
	sensorId: i32 = 0;
	visitorId: i32 = 0;
}

export struct b2Visitor {
	shapeId: i32 = 0;
	generation: u16 = 0;
}

export class b2Sensor {
	// todo find a way to pool these
	hits: Array<b2Visitor> = new Array<b2Visitor>();
	overlaps1: Array<b2Visitor> = new Array<b2Visitor>();
	overlaps2: Array<b2Visitor> = new Array<b2Visitor>();
	shapeId: i32 = 0;
}

export class b2SensorTaskContext {
	eventBits: b2BitSet = new b2BitSet();
}

// functions from sensor.c follow

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect shapes on the same body

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

// C's b2SensorQueryContext plus b2SensorQueryCallback, one module instance reused by every query
class b2SensorQueryContext {
	world: b2World? = null;
	sensor: b2Sensor? = null;
	sensorShapeId: i32 = 0;
	transform: b2WorldTransform = b2Transform();

	visit(proxyId: i32, userData: u64): bool {
		let shapeId = <i32>userData;

		let sensorShapeId = this.sensorShapeId;

		if (shapeId == sensorShapeId) {
			return true;
		}

		let world = this.world!;
		let shapes = world.shapes;

		// Are sensor events enabled on the other shape?
		if (shapes[shapeId].enableSensorEvents == false) {
			return true;
		}

		// Skip shapes on the same body
		if (shapes[shapeId].bodyId == shapes[sensorShapeId].bodyId) {
			return true;
		}

		// Check filter
		if (b2ShouldShapesCollide(shapes[sensorShapeId].filter, shapes[shapeId].filter) == false) {
			return true;
		}

		// Custom user filter
		if (shapes[sensorShapeId].enableCustomFiltering || shapes[shapeId].enableCustomFiltering) {
			let customFilterFcn = world.customFilterFcn;
			if (customFilterFcn != null) {
				let idA = b2ShapeId();
				idA.index1 = sensorShapeId + 1;
				idA.world0 = world.worldId;
				idA.generation = shapes[sensorShapeId].generation;
				let idB = b2ShapeId();
				idB.index1 = shapeId + 1;
				idB.world0 = world.worldId;
				idB.generation = shapes[shapeId].generation;
				let shouldCollide = customFilterFcn(idA, idB);
				if (shouldCollide == false) {
					return true;
				}
			}
		}

		// The relative pose is differenced in double so sensor overlap stays exact far from the origin
		let otherTransform = b2GetBodyTransform(world, shapes[shapeId].bodyId);

		let input = b2DistanceInput();
		input.proxyA = b2MakeShapeDistanceProxy(world, sensorShapeId);
		input.proxyB = b2MakeShapeDistanceProxy(world, shapeId);
		input.transform = b2InvMulWorldTransforms(this.transform, otherTransform);
		input.useRadii = true;
		let cache = b2SimplexCache();
		let output = b2ComputeShapeDistance(input, cache, null, 0);

		let overlaps = output.distance < 10.0 * FLT_EPSILON;
		if (overlaps == false) {
			return true;
		}

		// Record the overlap
		let shapeRef = b2Visitor();
		shapeRef.shapeId = shapeId;
		shapeRef.generation = shapes[shapeId].generation;
		this.sensor!.overlaps2.push(shapeRef);

		return true;
	}
}

const b2_sensorQueryContext = new b2SensorQueryContext();

// C's qsort with b2CompareVisitors, by shape id
function b2SortVisitors(visitors: Array<b2Visitor>): void {
	let count = visitors.length;
	for (let i = 1; i < count; ++i) {
		let item = visitors[i];
		let j = i - 1;
		while (j >= 0 && visitors[j].shapeId > item.shapeId) {
			visitors[j + 1] = visitors[j];
			j -= 1;
		}
		visitors[j + 1] = item;
	}
}

function b2SensorTask(startIndex: i32, endIndex: i32, threadIndex: i32, world: b2World): void {
	let taskContext = world.sensorTaskContexts[threadIndex];

	B2_ASSERT(startIndex < endIndex);

	let trees = world.broadPhase.trees;
	for (let sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex) {
		let sensor = world.sensors[sensorIndex];
		let sensorShapeId = sensor.shapeId;

		// Swap overlap arrays
		let temp = sensor.overlaps1;
		sensor.overlaps1 = sensor.overlaps2;
		sensor.overlaps2 = temp;
		sensor.overlaps2.length = 0;

		// Append sensor hits
		let hitCount = sensor.hits.length;
		for (let i = 0; i < hitCount; ++i) {
			sensor.overlaps2.push(sensor.hits[i]);
		}

		// Clear the hits
		sensor.hits.length = 0;

		let bodyId = world.shapes[sensorShapeId].bodyId;
		if (world.bodies[bodyId].setIndex == b2_disabledSet || world.shapes[sensorShapeId].enableSensorEvents == false) {
			if (sensor.overlaps1.length != 0) {
				// This sensor is dropping all overlaps because it has been disabled.
				b2SetBit(taskContext.eventBits, <u32>sensorIndex);
			}
			continue;
		}

		let transform = b2GetBodyTransformQuick(world, bodyId);

		let queryContext = b2_sensorQueryContext;
		queryContext.world = world;
		queryContext.sensor = sensor;
		queryContext.sensorShapeId = sensorShapeId;
		queryContext.transform = transform;

		B2_ASSERT(world.shapes[sensorShapeId].sensorIndex == sensorIndex);
		let queryBounds = world.shapes[sensorShapeId].aabb;
		let maskBits = world.shapes[sensorShapeId].filter.maskBits;

		// Query all trees
		b2DynamicTree_OverlapAABB(trees[0], queryBounds, maskBits, queryContext);
		b2DynamicTree_OverlapAABB(trees[1], queryBounds, maskBits, queryContext);
		b2DynamicTree_OverlapAABB(trees[2], queryBounds, maskBits, queryContext);

		queryContext.world = null;
		queryContext.sensor = null;

		// Sort the overlaps to enable finding begin and end events.
		b2SortVisitors(sensor.overlaps2);

		// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		let uniqueCount = 0;
		let overlapCount = sensor.overlaps2.length;
		let overlapData = sensor.overlaps2;
		for (let i = 0; i < overlapCount; ++i) {
			if (uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId) {
				overlapData[uniqueCount] = overlapData[i];
				uniqueCount += 1;
			}
		}
		sensor.overlaps2.length = uniqueCount;

		let count1 = sensor.overlaps1.length;
		let count2 = sensor.overlaps2.length;
		if (count1 != count2) {
			// something changed
			b2SetBit(taskContext.eventBits, <u32>sensorIndex);
		} else {
			for (let i = 0; i < count1; ++i) {
				let s1 = sensor.overlaps1[i];
				let s2 = sensor.overlaps2[i];

				if (s1.shapeId != s2.shapeId || s1.generation != s2.generation) {
					// something changed
					b2SetBit(taskContext.eventBits, <u32>sensorIndex);
					break;
				}
			}
		}
	}
}

function b2MakeVisitorShapeId(world: b2World, shapeId: i32, generation: u16): b2ShapeId {
	let id = b2ShapeId();
	id.index1 = shapeId + 1;
	id.world0 = world.worldId;
	id.generation = generation;
	return id;
}

function b2PushSensorEnd(world: b2World, sensorId: b2ShapeId, visitorId: b2ShapeId): void {
	let event = b2SensorEndTouchEvent();
	event.sensorShapeId = sensorId;
	event.visitorShapeId = visitorId;
	world.sensorEndEvents[world.endEventArrayIndex].push(event);
}

function b2PushSensorBegin(world: b2World, sensorId: b2ShapeId, visitorId: b2ShapeId): void {
	let event = b2SensorBeginTouchEvent();
	event.sensorShapeId = sensorId;
	event.visitorShapeId = visitorId;
	world.sensorBeginEvents.push(event);
}

@noinline
export function b2OverlapSensors(world: b2World): void {
	let sensorCount = world.sensors.length;
	if (sensorCount == 0) {
		return;
	}

	B2_ASSERT(world.workerCount > 0);

	for (let i = 0; i < world.workerCount; ++i) {
		b2SetBitCountAndClear(world.sensorTaskContexts[i].eventBits, <u32>sensorCount);
	}

	// Parallel-for sensors overlaps
	let minRange = 16;
	b2ParallelFor<b2World>(world, b2SensorTask, sensorCount, minRange, world);

	let bitSet = world.sensorTaskContexts[0].eventBits;
	for (let i = 1; i < world.workerCount; ++i) {
		b2InPlaceUnion(bitSet, world.sensorTaskContexts[i].eventBits);
	}

	// Iterate sensors bits and publish events
	// Process sensor state changes. Iterate over set bits
	let bits = bitSet.bits;
	let blockCount = bitSet.blockCount;

	for (let k: u32 = 0; k < blockCount; ++k) {
		let word = bits[k];
		while (word != 0) {
			let ctz = b2CTZ64(word);
			let sensorIndex = <i32>(64 * k + ctz);

			let sensor = world.sensors[sensorIndex];
			let sensorId = b2MakeVisitorShapeId(world, sensor.shapeId, world.shapes[sensor.shapeId].generation);

			let count1 = sensor.overlaps1.length;
			let count2 = sensor.overlaps2.length;
			let refs1 = sensor.overlaps1;
			let refs2 = sensor.overlaps2;

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			let index1 = 0;
			let index2 = 0;
			while (index1 < count1 && index2 < count2) {
				let r1 = refs1[index1];
				let r2 = refs2[index2];
				if (r1.shapeId == r2.shapeId) {
					if (r1.generation < r2.generation) {
						// end
						b2PushSensorEnd(world, sensorId, b2MakeVisitorShapeId(world, r1.shapeId, r1.generation));
						index1 += 1;
					} else if (r1.generation > r2.generation) {
						// begin
						b2PushSensorBegin(world, sensorId, b2MakeVisitorShapeId(world, r2.shapeId, r2.generation));
						index2 += 1;
					} else {
						// persisted
						index1 += 1;
						index2 += 1;
					}
				} else if (r1.shapeId < r2.shapeId) {
					// end
					b2PushSensorEnd(world, sensorId, b2MakeVisitorShapeId(world, r1.shapeId, r1.generation));
					index1 += 1;
				} else {
					// begin
					b2PushSensorBegin(world, sensorId, b2MakeVisitorShapeId(world, r2.shapeId, r2.generation));
					index2 += 1;
				}
			}

			while (index1 < count1) {
				// end
				let r1 = refs1[index1];
				b2PushSensorEnd(world, sensorId, b2MakeVisitorShapeId(world, r1.shapeId, r1.generation));
				index1 += 1;
			}

			while (index2 < count2) {
				// begin
				let r2 = refs2[index2];
				b2PushSensorBegin(world, sensorId, b2MakeVisitorShapeId(world, r2.shapeId, r2.generation));
				index2 += 1;
			}

			// Clear the smallest set bit
			word = word & (word - 1);
		}
	}
}

export function b2DestroySensor(world: b2World, shapeId: i32): void {
	let sensorIndex = world.shapes[shapeId].sensorIndex;
	let sensor = world.sensors[sensorIndex];
	let sensorId = b2MakeVisitorShapeId(world, world.shapes[shapeId].id, world.shapes[shapeId].generation);
	for (let i = 0; i < sensor.overlaps2.length; ++i) {
		let visitor = sensor.overlaps2[i];
		b2PushSensorEnd(world, sensorId, b2MakeVisitorShapeId(world, visitor.shapeId, visitor.generation));
	}

	// Destroy sensor
	sensor.hits.length = 0;
	sensor.overlaps1.length = 0;
	sensor.overlaps2.length = 0;

	let movedIndex = b2Array_RemoveSwap(world.sensors, sensorIndex);
	if (movedIndex != B2_NULL_INDEX) {
		// Fixup moved sensor
		let movedSensor = world.sensors[sensorIndex];
		world.shapes[movedSensor.shapeId].sensorIndex = sensorIndex;
	}
}
