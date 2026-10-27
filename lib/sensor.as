// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/sensor.c

import { b2BitSet } from "./bitset";

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
