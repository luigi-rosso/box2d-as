// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Runs every example and checks that it shows what it says. examples/check.sh runs this suite with rive --test.

import { Tests } from "rive/test";
import { events } from "./events";
import { helloWorld } from "./hello_world";
import { ANCHOR_HEIGHT, LINK_COUNT, LINK_LENGTH, joints } from "./joints";
import { queries } from "./queries";
import { shapes } from "./shapes";
import { threading } from "./threading";

export class ExampleTests extends Tests {
	override run(): void {
		test("hello world: the box comes to rest on the ground and sleeps", (expect) => {
			let result = helloWorld();
			expect.that(result.height).greaterThan(0.48);
			expect.that(result.height).lessThan(0.52);
			expect.that(result.awake).is(false);
		});

		test("shapes: each body rests on the tray's floor and sleeps", (expect) => {
			let result = shapes();
			expect.that(result.circleHeight).greaterThan(0.48);
			expect.that(result.circleHeight).lessThan(0.52);
			expect.that(result.capsuleHeight).greaterThan(0.23);
			expect.that(result.capsuleHeight).lessThan(0.27);
			expect.that(result.boxHeight).greaterThan(0.48);
			expect.that(result.boxHeight).lessThan(0.52);
			expect.that(result.roundedHeight).greaterThan(0.48);
			expect.that(result.roundedHeight).lessThan(0.52);
			// The hull's bottom edge is 0.4 m under its origin
			expect.that(result.hullHeight).greaterThan(0.38);
			expect.that(result.hullHeight).lessThan(0.42);
			expect.that(result.awakeCount).is(0);
		});

		test("joints: the chain hangs under its anchor and the platform reaches its limit", (expect) => {
			let result = joints();
			let length: f32 = <f32>LINK_COUNT * LINK_LENGTH;
			expect.that(Mathf.abs(result.chainEnd.x)).lessThan(0.5);
			expect.that(result.chainEnd.y).lessThan(ANCHOR_HEIGHT - length + LINK_LENGTH);
			expect.that(result.chainEnd.y).greaterThan(ANCHOR_HEIGHT - length - 0.1);
			expect.that(result.elevatorTranslation).greaterThan(2.95);
			expect.that(result.elevatorTranslation).lessThan(3.05);
		});

		test("queries: rays, a circle cast and overlaps find the boxes", (expect) => {
			let result = queries();
			expect.that(result.closestBox).is(1);
			expect.that(result.closestHeight).greaterThan(0.99);
			expect.that(result.closestHeight).lessThan(1.01);
			expect.that(result.rayHitCount).is(3);
			// The first box's left face is 5.5 m along the 20 m cast, less the circle's radius
			expect.that(result.castBox).is(0);
			expect.that(result.castFraction).greaterThan(0.26);
			expect.that(result.castFraction).lessThan(0.265);
			expect.that(result.aabbOverlaps).is(1);
			expect.that(result.shapeOverlaps).is(2);
		});

		test("events: the sensor sees the box pass, the ground reports its landing", (expect) => {
			let result = events();
			expect.that(result.sensorBegins).is(1);
			expect.that(result.sensorEnds).is(1);
			expect.that(result.contactBegins).is(1);
			expect.that(result.hits).is(1);
			expect.that(result.fastestHit).greaterThan(5.0);
		});

		test("threading: parallel calls end bit for bit where the serial path does", (expect) => {
			let result = threading();
			expect.that(result.differ).is(0);
			// Every step ran its solver stages and narrow phase on parallel calls
			expect.that(result.solves).is(120);
			expect.that(result.collides).is(120);
		});
	}
}
