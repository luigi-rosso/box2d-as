// Ported from box2d src/sensor.h plus the destroy bookkeeping from sensor.c.
// The overlap pass (b2OverlapSensors) is deferred: the Luau subset never
// consumes sensor events, so sensors exist in the broad phase and mass only.

import { B2_NULL_INDEX } from "./core";
import { b2Array } from "./array";
import { b2World, b2SensorEndTouchEvent } from "./physics_world";
import { b2Shape } from "./shape";
import { b2ShapeId } from "./id";

export class b2SensorHit {
    sensorId: i32 = 0;
    visitorId: i32 = 0;
}

export class b2Visitor {
    shapeId: i32 = 0;
    generation: u16 = 0;
}

export class b2Sensor {
    hits: b2Array<b2Visitor> = new b2Array<b2Visitor>();
    overlaps1: b2Array<b2Visitor> = new b2Array<b2Visitor>();
    overlaps2: b2Array<b2Visitor> = new b2Array<b2Visitor>();
    shapeId: i32 = 0;
}

// Deferred: no sensor overlap pass in the AS port.
export function b2OverlapSensors(world: b2World): void {
}

export function b2DestroySensor(world: b2World, sensorShape: b2Shape): void {
    let sensor = world.sensors.get(sensorShape.sensorIndex);
    for (let i = 0; i < sensor.overlaps2.count; ++i) {
        let ref = sensor.overlaps2.get(i);
        let event = new b2SensorEndTouchEvent();
        event.sensorShapeId = new b2ShapeId(sensorShape.id + 1, world.worldId, sensorShape.generation);
        event.visitorShapeId = new b2ShapeId(ref.shapeId + 1, world.worldId, ref.generation);

        world.sensorEndEvents[world.endEventArrayIndex].push(event);
    }

    let movedIndex = world.sensors.removeSwap(sensorShape.sensorIndex);
    if (movedIndex != B2_NULL_INDEX) {
        // Fixup moved sensor
        let movedSensor = world.sensors.get(sensorShape.sensorIndex);
        let otherSensorShape = world.shapes.get(movedSensor.shapeId);
        otherSensorShape.sensorIndex = sensorShape.sensorIndex;
    }
}
