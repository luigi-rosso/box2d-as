// Step timing for the port, the AnimaScript side of bench/native.
// --define=SCENE=n runs scene n of the verify/scenes mirror (copied in as ./scenes/scenes.as by run.sh), in
// s_sceneNames order: 0 pyramid, 1 shape_rain, 2 chain_car, 3 joints, 4 dynamic_mover, 5 churn, 6 bullets,
// 7 sensors, 8 events, 9 queries, 10 mover, 11 many_bodies, 12 large_pyramid, 13 joint_grid, 14 rain, 15 smash,
// 16 spinner, 17 tumbler. --define=STEPS=n (default the trajectory gate's step count); --define=STEPLOG logs every
// step; --define=DELAY=s spins s seconds before the first step. One Scene_Step per frame; the scene is built in
// init and the result logged after the last step.

import { Layout, Context } from "rive/host";
import { clock } from "rive/os";
import { formatFixed } from "rive/format";
import {
	b2Body_GetAngularVelocity,
	b2Body_GetLinearVelocity,
	b2Body_GetPosition,
	b2Body_GetRotation,
	b2Body_IsValid,
	b2Profile,
	b2World_GetCounters,
	b2World_GetProfile,
} from "./lib/box2d";
// run.sh copies verify/scenes/scenes.as here, the mirror of the native bench's scenes.c
import { Scene, Scene_Create, Scene_GetCount, Scene_GetName, Scene_Step } from "./scenes/scenes";

const STAGE_COUNT: i32 = 23;
const WARMUP_STEPS: i32 = 1;
const SCENE_LARGE_PYRAMID: i32 = 12;

// verify/traj.sh steps_for, in scene order
const DEFAULT_STEPS: StaticArray<i32> = [300, 1000, 720, 500, 900, 800, 400, 600, 600, 300, 900, 100, 100, 100, 360, 120, 300, 200];

const STAGE_NAMES: StaticArray<string> = [
	"step", "pairs", "collide", "solve", "solverSetup", "constraints", "prepareConstraints",
	"integrateVelocities", "warmStart", "solveImpulses", "integratePositions", "relaxImpulses",
	"restitution", "storeImpulses", "splitIslands", "transforms", "sensorHits", "jointEvents",
	"hitEvents", "refit", "bullets", "sleepIslands", "sensors",
];

function hashWord(h: u32, bits: u32): u32 {
	for (let s = 0; s < 4; ++s) {
		h = (h ^ ((bits >> (8 * <u32>s)) & 0xff)) * 16777619;
	}
	return h;
}

// FNV-1a over each body's pose and velocities in the order of a trajectory record, as bench/web's stateHash, so a
// run on any tier or artifact can be checked bit exact; a destroyed body hashes the recording's zero words
function stateHash(scene: Scene): u32 {
	let h: u32 = 2166136261;
	for (let i = 0; i < scene.bodyCount; ++i) {
		let id = scene.bodies[i];
		if (!b2Body_IsValid(id)) {
			for (let k = 0; k < 7; ++k) {
				h = hashWord(h, 0);
			}
			continue;
		}
		let p = b2Body_GetPosition(id);
		let q = b2Body_GetRotation(id);
		let v = b2Body_GetLinearVelocity(id);
		let w: f32 = b2Body_GetAngularVelocity(id);
		h = hashWord(h, reinterpret<u32>(p.x));
		h = hashWord(h, reinterpret<u32>(p.y));
		h = hashWord(h, reinterpret<u32>(q.c));
		h = hashWord(h, reinterpret<u32>(q.s));
		h = hashWord(h, reinterpret<u32>(v.x));
		h = hashWord(h, reinterpret<u32>(v.y));
		h = hashWord(h, reinterpret<u32>(w));
	}
	return h;
}

function sceneIndex(): i32 {
	if (isDefined(SCENE)) {
		return SCENE;
	}
	return SCENE_LARGE_PYRAMID;
}

function stepCount(scene: i32): i32 {
	if (isDefined(STEPS)) {
		return STEPS;
	}
	return DEFAULT_STEPS[scene];
}

function profileToArray(p: b2Profile, out: Float64Array): void {
	out[0] = <f64>p.step;
	out[1] = <f64>p.pairs;
	out[2] = <f64>p.collide;
	out[3] = <f64>p.solve;
	out[4] = <f64>p.solverSetup;
	out[5] = <f64>p.constraints;
	out[6] = <f64>p.prepareConstraints;
	out[7] = <f64>p.integrateVelocities;
	out[8] = <f64>p.warmStart;
	out[9] = <f64>p.solveImpulses;
	out[10] = <f64>p.integratePositions;
	out[11] = <f64>p.relaxImpulses;
	out[12] = <f64>p.restitution;
	out[13] = <f64>p.storeImpulses;
	out[14] = <f64>p.splitIslands;
	out[15] = <f64>p.transforms;
	out[16] = <f64>p.sensorHits;
	out[17] = <f64>p.jointEvents;
	out[18] = <f64>p.hitEvents;
	out[19] = <f64>p.refit;
	out[20] = <f64>p.bullets;
	out[21] = <f64>p.sleepIslands;
	out[22] = <f64>p.sensors;
}

function median(sorted: Float64Array): f64 {
	let n = sorted.length;
	return (n & 1) != 0 ? sorted[n >> 1] : 0.5 * (sorted[(n >> 1) - 1] + sorted[n >> 1]);
}

export class Main extends Layout {
	scene: i32 = 0;
	steps: i32 = 0;
	stepIndex: i32 = 0;
	stepMs: Float64Array = new Float64Array(0);
	stageSum: Float64Array = new Float64Array(STAGE_COUNT);
	stageMin: Float64Array = new Float64Array(STAGE_COUNT);
	stages: Float64Array = new Float64Array(STAGE_COUNT);
	createMs: f64 = 0;
	sceneData: Scene = new Scene();

	override init(context: Context): bool {
		this.scene = sceneIndex();
		if (this.scene < 0 || this.scene >= Scene_GetCount()) {
			console.log("BENCH unknown scene " + this.scene.toString());
			return false;
		}
		this.steps = stepCount(this.scene);
		this.stepMs = new Float64Array(this.steps - WARMUP_STEPS);

		let t0 = clock();
		Scene_Create(this.sceneData, Scene_GetName(this.scene), 1);
		this.createMs = (clock() - t0) * 1000.0;
		console.log("BENCH start scene=" + Scene_GetName(this.scene) + " steps=" + this.steps.toString() +
			" create=" + formatFixed(this.createMs, 2) + "ms");
		return true;
	}

	override advance(seconds: f64): bool {
		if (this.stepIndex >= this.steps) {
			return true;
		}

		if (isDefined(DELAY)) {
			// Lets a profiler attach before the first step; profile.sh passes it
			if (this.stepIndex == 0) {
				let start = clock();
				while (clock() - start < <f64>DELAY) {}
			}
		}

		let t0 = clock();
		Scene_Step(this.sceneData, this.stepIndex);
		let ms = (clock() - t0) * 1000.0;

		if (this.stepIndex >= WARMUP_STEPS) {
			this.stepMs[this.stepIndex - WARMUP_STEPS] = ms;
			profileToArray(b2World_GetProfile(this.sceneData.worldId), this.stages);
			if (isDefined(STEPLOG)) {
				let line = "BENCH STEP " + this.stepIndex.toString() + " ms=" + formatFixed(ms, 3);
				for (let k = 1; k < STAGE_COUNT; ++k) {
					if (this.stages[k] >= 0.5) {
						line += " " + STAGE_NAMES[k] + "=" + formatFixed(this.stages[k], 2);
					}
				}
				console.log(line);
			}
			for (let k = 0; k < STAGE_COUNT; ++k) {
				this.stageSum[k] += this.stages[k];
				this.stageMin[k] = this.stepIndex == WARMUP_STEPS ? this.stages[k] : Math.min(this.stageMin[k], this.stages[k]);
			}
		}

		this.stepIndex += 1;
		if (this.stepIndex == this.steps) {
			this.report();
		}
		return true;
	}

	report(): void {
		let timed = this.stepMs.length;
		let total: f64 = 0;
		for (let i = 0; i < timed; ++i) {
			total += this.stepMs[i];
		}
		let sorted = this.stepMs.slice(0);
		sorted.sort();
		let counters = b2World_GetCounters(this.sceneData.worldId);

		let line = "BENCH RESULT scene=" + Scene_GetName(this.scene) + " steps=" + this.steps.toString() +
			" timed=" + timed.toString() + " bodies=" + counters.bodyCount.toString() +
			" contacts=" + counters.contactCount.toString() + " create=" + formatFixed(this.createMs, 2) +
			" mean=" + formatFixed(total / <f64>timed, 4) + " min=" + formatFixed(sorted[0], 4) +
			" median=" + formatFixed(median(sorted), 4) + " max=" + formatFixed(sorted[timed - 1], 4);
		for (let k = 0; k < STAGE_COUNT; ++k) {
			line += " " + STAGE_NAMES[k] + "=" + formatFixed(this.stageSum[k] / <f64>timed, 4);
		}
		// Per stage minimum over the timed steps, which a loaded machine disturbs least
		for (let k = 0; k < STAGE_COUNT; ++k) {
			line += " min_" + STAGE_NAMES[k] + "=" + formatFixed(this.stageMin[k], 4);
		}
		line += " hash=" + stateHash(this.sceneData).toString();
		console.log(line);
	}
}
