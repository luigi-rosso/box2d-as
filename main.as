// Box2D demo shell: steps one of the 18 scenes of verify/scenes at 60 Hz, serial or on parallel calls, and draws it
// with demo/draw behind the panels of demo/hud. --define=START_SCENE=n opens scene n, counting from 1, and
// --define=PARALLEL starts on parallel calls.

import { Layout, Context } from "rive/host";
import { Renderer } from "rive/renderer";
import { KeyboardEvent, PointerEvent, ScrollEvent } from "rive/events";
import { Parallel } from "rive/parallel";
import { clock } from "rive/os";
import { formatFixed, formatInt } from "rive/format";
import { Camera } from "./demo/camera";
import { WorldDraw } from "./demo/draw";
import { Hud, INK_FAINT, INK_LINE, INK_MUTED, STYLE_BRAND, STYLE_ROW, STYLE_SMALL, STYLE_VALUE } from "./demo/hud";
import { Scene, Scene_Create, Scene_Destroy, Scene_GetCount, Scene_GetName, Scene_Step } from "./verify/scenes/scenes";
import { b2Body_GetPosition, b2World_GetAwakeBodyCount, b2World_GetCounters, b2World_GetProfile } from "./lib/box2d";
import { b2SetParallelLimit, b2World_EnableParallel, b2_parallelAll } from "./lib/parallel_step";

const STEP: f32 = 1.0 / 60.0;
const MAX_STEPS_PER_FRAME: i32 = 3;

// GLFW key codes
const KEY_SPACE: i32 = 32;
const KEY_1: i32 = 49;
const KEY_9: i32 = 57;
const KEY_F: i32 = 70;
const KEY_G: i32 = 71;
const KEY_H: i32 = 72;
const KEY_J: i32 = 74;
const KEY_O: i32 = 79;
const KEY_P: i32 = 80;
const KEY_R: i32 = 82;
const KEY_T: i32 = 84;
const KEY_LEFT_BRACKET: i32 = 91;
const KEY_RIGHT_BRACKET: i32 = 93;
const KEY_DOWN: i32 = 264;
const KEY_UP: i32 = 265;

// Buttons past the scene rows, which answer their scene's index
const ID_PAUSE: i32 = 100;
const ID_STEP: i32 = 101;
const ID_RESTART: i32 = 102;
const ID_SERIAL: i32 = 103;
const ID_PARALLEL: i32 = 104;
const ID_FEWER: i32 = 105;
const ID_MORE: i32 = 106;

const MARGIN: f32 = 28;
const COLUMN: f32 = 196;
const ROW: f32 = 24;
const BUTTON: f32 = 24;
// The background under the panels, so the scene does not run through their text
const PANEL: u32 = 0xD0151412;

// The box each scene's moving bodies cover over its run, as min x, min y, max x, max y, measured by stepping it.
// The scenes that travel keep the camera on their car or mover instead, and take only the box's size.
const HOMES: StaticArray<f32> = [
	-6, -1, 6, 11, // pyramid
	-31, -2, 31, 27, // shape_rain
	-14, -5, 14, 9, // chain_car
	-31, -2, 35, 10, // joints
	-20, -8, 20, 12, // dynamic_mover
	-17, -2, 17, 16, // churn
	-40, -2, 14, 24, // bullets
	-21, -3, 15, 13, // sensors
	-14, -1, 12, 16, // events
	-19, -1, 18, 26, // queries
	-20, -8, 20, 12, // mover
	-56, -2, 56, 111, // many_bodies
	-51, -2, 51, 101, // large_pyramid
	-2, -108, 101, 2, // joint_grid
	-126, 110, 0, 223, // rain
	-26, -20, 79, 20, // smash
	-41, -10, 41, 73, // spinner
	-15, -5, 15, 25, // tumbler
];

const PROFILE_NAMES: StaticArray<string> = ["PAIRS", "COLLIDE", "SOLVER", "FINALIZE", "OTHER"];
const COUNT_NAMES: StaticArray<string> = ["BODIES", "AWAKE", "CONTACTS", "JOINTS", "ISLANDS", "SHAPES DRAWN"];

// large_pyramid reads Large pyramid
function title(name: string): string {
	let spaced = name.replaceAll("_", " ");
	return spaced.charAt(0).toUpperCase() + spaced.substring(1);
}

function ms(value: f64): string {
	return formatFixed(value, value < 0.1 ? 3 : 2) + " ms";
}

export class Main extends Layout {
	width: f32 = 1280;
	height: f32 = 800;
	camera: Camera = new Camera();
	draws: WorldDraw = new WorldDraw();
	hud: Hud = new Hud();
	hudOn: bool = true;

	scene: Scene = new Scene();
	names: Array<string> = new Array<string>();
	current: i32 = -1;
	stepIndex: i32 = 0;
	paused: bool = false;
	stepOnce: bool = false;
	accumulator: f32 = 0;
	// The camera stays on the scene's car or mover, until a drag takes it away
	following: bool = true;

	parallel: bool = false;
	parts: i32 = 1;
	maxParts: i32 = 1;

	// Smoothed over steps, in ms
	stepMs: f64 = 0;
	drawMs: f64 = 0;
	profile: StaticArray<f64> = new StaticArray<f64>(PROFILE_NAMES.length);
	lastProfile: StaticArray<f64> = new StaticArray<f64>(PROFILE_NAMES.length);
	// What the readout shows, refreshed a few times a second so its text is not laid out every frame
	statsAge: f32 = 1;
	shownMs: StaticArray<string> = new StaticArray<string>(2 + PROFILE_NAMES.length);
	shownCounts: StaticArray<string> = new StaticArray<string>(COUNT_NAMES.length);

	dragging: bool = false;
	pointerId: f64 = -1;
	lastX: f32 = 0;
	lastY: f32 = 0;
	point: StaticArray<f32> = new StaticArray<f32>(2);

	override init(context: Context): bool {
		let font = context.font("InterVariable");
		if (font != null) {
			this.hud.setFont(font);
		} else {
			context.log("box2d demo: InterVariable font missing, panels have no text");
		}
		for (let i = 0; i < Scene_GetCount(); i++) {
			this.names.push(title(Scene_GetName(i)));
		}
		this.shownMs.fill("");
		this.shownCounts.fill("");
		this.maxParts = Parallel.size();
		this.parts = this.maxParts;
		if (isDefined(PARALLEL)) {
			this.parallel = this.maxParts > 1;
		}
		let start = 0;
		if (isDefined(START_SCENE)) {
			start = START_SCENE - 1;
		}
		this.openScene(start);
		return true;
	}

	override resize(width: f64, height: f64, scale: f64): void {
		this.width = <f32>width;
		this.height = <f32>height;
		this.layoutCamera();
	}

	private layoutCamera(): void {
		this.camera.setViewport(this.width, this.height, this.hudOn ? MARGIN + COLUMN : 0);
	}

	// ---- simulation -------------------------------------------------------

	private openScene(index: i32): void {
		let count = Scene_GetCount();
		index = ((index % count) + count) % count;
		if (this.current >= 0) {
			Scene_Destroy(this.scene);
		}
		this.current = index;
		this.scene = new Scene();
		Scene_Create(this.scene, Scene_GetName(index), 1);
		this.stepIndex = 0;
		this.accumulator = 0;
		this.stepMs = 0;
		this.statsAge = 1;
		this.draws.reset();
		this.applyThreads();
		this.home(true);
	}

	// Parallel calls give the world a task context per part; serial leaves the step as it was
	private applyThreads(): void {
		b2World_EnableParallel(this.scene.worldId, this.parallel ? b2_parallelAll : 0);
		b2SetParallelLimit(this.parts);
	}

	private setParallel(on: bool): void {
		this.parallel = on && this.maxParts > 1;
		this.applyThreads();
	}

	private setParts(parts: i32): void {
		this.parts = min(max(parts, 1), this.maxParts);
		this.applyThreads();
	}

	private timedStep(): void {
		let t0 = clock();
		Scene_Step(this.scene, this.stepIndex);
		let elapsed = (clock() - t0) * 1000;
		this.stepIndex += 1;
		this.stepMs = this.stepMs == 0 ? elapsed : this.stepMs * 0.9 + elapsed * 0.1;

		let p = b2World_GetProfile(this.scene.worldId);
		let parts = this.lastProfile;
		parts[0] = <f64>p.pairs;
		parts[1] = <f64>p.collide;
		parts[2] = <f64>p.constraints;
		parts[3] = <f64>p.transforms;
		parts[4] = Math.max(0, <f64>p.step - parts[0] - parts[1] - parts[2] - parts[3]);
		for (let i = 0; i < parts.length; i++) {
			this.profile[i] = this.profile[i] * 0.9 + parts[i] * 0.1;
		}
	}

	override advance(seconds: f64): bool {
		let dt = <f32>Math.min(seconds, 0.1);
		if (!this.paused) {
			this.accumulator += dt;
			let n = 0;
			while (this.accumulator + 1e-5 >= STEP && n < MAX_STEPS_PER_FRAME) {
				this.timedStep();
				this.accumulator -= STEP;
				n++;
			}
			if (n == MAX_STEPS_PER_FRAME) {
				this.accumulator = 0;
			}
		} else if (this.stepOnce) {
			this.timedStep();
		}
		this.stepOnce = false;

		let point = this.point;
		if (this.following && this.followPoint(point)) {
			this.camera.targetX = point[0];
			this.camera.targetY = point[1];
		}
		this.camera.advance(dt);

		this.statsAge += dt;
		if (this.statsAge >= 0.25) {
			this.statsAge = 0;
			this.readStats();
		}
		return true;
	}

	// Where the scene's car or mover is, for the scenes that have one
	private followPoint(out: StaticArray<f32>): bool {
		let scene = this.scene;
		let name = Scene_GetName(this.current);
		let p = scene.geometricMover.position;
		if (name == "chain_car") {
			p = b2Body_GetPosition(scene.car.chassisId);
		} else if (name == "dynamic_mover") {
			p = b2Body_GetPosition(scene.dynamicMover.moverId);
		} else if (name != "mover") {
			return false;
		}
		out[0] = p.x;
		out[1] = p.y;
		return true;
	}

	// Frames the scene's home box, around its car or mover when it has one
	private home(snap: bool): void {
		let at = 4 * this.current;
		let x: f32 = 0;
		let y: f32 = 0;
		this.following = true;
		let point = this.point;
		if (this.followPoint(point)) {
			x = point[0] - 0.5 * (HOMES[at] + HOMES[at + 2]);
			y = point[1] - 0.5 * (HOMES[at + 1] + HOMES[at + 3]);
		}
		this.camera.fit(HOMES[at] + x, HOMES[at + 1] + y, HOMES[at + 2] + x, HOMES[at + 3] + y, snap);
	}

	private readStats(): void {
		let worldId = this.scene.worldId;
		let c = b2World_GetCounters(worldId);
		this.shownMs[0] = ms(this.stepMs);
		this.shownMs[1] = ms(this.drawMs);
		for (let i = 0; i < PROFILE_NAMES.length; i++) {
			this.shownMs[2 + i] = ms(this.profile[i]);
		}
		let shown = this.shownCounts;
		shown[0] = formatInt(<f64>c.bodyCount);
		shown[1] = formatInt(<f64>b2World_GetAwakeBodyCount(worldId));
		shown[2] = formatInt(<f64>c.contactCount);
		shown[3] = formatInt(<f64>c.jointCount);
		shown[4] = formatInt(<f64>c.islandCount);
		shown[5] = formatInt(<f64>this.draws.shapeCount);
	}

	// ---- drawing ----------------------------------------------------------

	override draw(renderer: Renderer): void {
		let t0 = clock();
		let scene = this.scene;
		// The geometric mover is a query shape, not a body, so the scene's bodies do not include it
		let mover = scene.geometricMover;
		this.draws.setMover(Scene_GetName(this.current) == "mover", mover.position, mover.capsule);
		this.draws.draw(renderer, this.camera, scene.bodies, scene.bodyCount);
		if (this.hudOn) {
			this.drawHud(renderer);
		}
		let elapsed = (clock() - t0) * 1000;
		this.drawMs = this.drawMs == 0 ? elapsed : this.drawMs * 0.9 + elapsed * 0.1;
	}

	private drawHud(renderer: Renderer): void {
		let hud = this.hud;
		hud.begin();
		let x = MARGIN;
		let y = MARGIN;
		hud.box(renderer, 0, 0, MARGIN + COLUMN + 12, this.height, PANEL, 0);
		hud.box(renderer, this.width - MARGIN - 162, MARGIN - 12, 174, 286, PANEL, 0);
		hud.text(renderer, STYLE_BRAND, "BOX2D", x + 10, y, INK_LINE);
		hud.text(renderer, STYLE_SMALL, "ANIMASCRIPT PORT  /  DEBUG DRAW", x + 10, y + 24, INK_MUTED);
		y += 60;

		// Rows shrink before the controls under them leave the window
		let count = this.names.length;
		let row = Mathf.min(ROW, (this.height - y - 170) / <f32>count);
		for (let i = 0; i < count; i++) {
			let on = i == this.current;
			hud.button(renderer, i, x, y, COLUMN, row - 2, on);
			hud.text(renderer, STYLE_SMALL, formatInt(<f64>(i + 1), 2, true), x + 10, y + 0.5 * row - 4.5, INK_FAINT);
			hud.text(renderer, STYLE_ROW, this.names[i], x + 36, y + 0.5 * row - 6, on ? INK_LINE : INK_MUTED);
			y += row;
		}
		y += 14;

		let third = (COLUMN - 8) / 3;
		hud.labeled(renderer, ID_PAUSE, this.paused ? "PLAY" : "PAUSE", x, y, third, BUTTON, this.paused);
		hud.labeled(renderer, ID_STEP, "STEP", x + third + 4, y, third, BUTTON, false);
		hud.labeled(renderer, ID_RESTART, "RESTART", x + 2 * (third + 4), y, third, BUTTON, false);
		y += BUTTON + 6;

		let half = (COLUMN - 4) / 2;
		hud.labeled(renderer, ID_SERIAL, "SERIAL", x, y, half, BUTTON, !this.parallel);
		hud.labeled(renderer, ID_PARALLEL, "PARALLEL", x + half + 4, y, half, BUTTON, this.parallel);
		y += BUTTON + 6;

		let note = this.maxParts > 1 ? formatInt(<f64>this.parts) + " OF " + formatInt(<f64>this.maxParts) + " PARTS" : "NO THREADS";
		hud.labeled(renderer, ID_FEWER, "-", x, y, BUTTON, BUTTON, false);
		hud.text(renderer, STYLE_SMALL, note, x + 0.5 * COLUMN, y + 0.5 * BUTTON - 3.5, this.parallel ? INK_LINE : INK_FAINT, 0.5);
		hud.labeled(renderer, ID_MORE, "+", x + COLUMN - BUTTON, y, BUTTON, BUTTON, false);
		y += BUTTON + 14;

		hud.text(renderer, STYLE_SMALL, "P PAUSE   O STEP   R RESTART   F HOME", x, y, INK_FAINT);
		hud.text(renderer, STYLE_SMALL, "T THREADS   [ ] PARTS   UP DOWN SCENE", x, y + 14, INK_FAINT);
		hud.text(renderer, STYLE_SMALL, "J JOINTS   G GRID   H PANELS   DRAG PAN", x, y + 28, INK_FAINT);

		let right = this.width - MARGIN;
		let left = right - 150;
		y = MARGIN;
		y = this.readout(renderer, "STEP", this.shownMs[0], left, right, y);
		y = this.readout(renderer, "DRAW", this.shownMs[1], left, right, y) + 10;
		for (let i = 0; i < COUNT_NAMES.length; i++) {
			y = this.readout(renderer, COUNT_NAMES[i], this.shownCounts[i], left, right, y);
		}
		y += 10;
		for (let i = 0; i < PROFILE_NAMES.length; i++) {
			y = this.readout(renderer, PROFILE_NAMES[i], this.shownMs[2 + i], left, right, y);
		}
	}

	// A name on the left and its value on the right; returns the next line's y
	private readout(renderer: Renderer, name: string, value: string, left: f32, right: f32, y: f32): f32 {
		this.hud.text(renderer, STYLE_SMALL, name, left, y + 1.5, INK_MUTED);
		this.hud.text(renderer, STYLE_VALUE, value, right, y, INK_LINE, 1);
		return y + 18;
	}

	// ---- input ------------------------------------------------------------

	private press(id: i32): void {
		if (id < this.names.length) {
			this.openScene(id);
		} else if (id == ID_PAUSE) {
			this.paused = !this.paused;
		} else if (id == ID_STEP) {
			this.paused = true;
			this.stepOnce = true;
		} else if (id == ID_RESTART) {
			this.openScene(this.current);
		} else if (id == ID_SERIAL) {
			this.setParallel(false);
		} else if (id == ID_PARALLEL) {
			this.setParallel(true);
		} else if (id == ID_FEWER) {
			this.setParts(this.parts - 1);
		} else if (id == ID_MORE) {
			this.setParts(this.parts + 1);
		}
	}

	override pointerDown(event: PointerEvent): void {
		let x = event.position.x;
		let y = event.position.y;
		event.hit();
		let id = this.hudOn ? this.hud.hit(x, y) : -1;
		if (id >= 0) {
			this.press(id);
			return;
		}
		this.dragging = true;
		this.following = false;
		this.pointerId = event.id;
		this.lastX = x;
		this.lastY = y;
	}

	override pointerMove(event: PointerEvent): void {
		if (!this.dragging || event.id != this.pointerId) {
			return;
		}
		let x = event.position.x;
		let y = event.position.y;
		this.camera.pan(x - this.lastX, y - this.lastY);
		this.lastX = x;
		this.lastY = y;
	}

	override pointerUp(event: PointerEvent): void {
		if (event.id == this.pointerId) {
			this.dragging = false;
		}
	}

	override pointerExit(event: PointerEvent): void {
		this.pointerUp(event);
	}

	override pointerScroll(event: ScrollEvent): void {
		event.hit();
		let rate: f32 = event.precise ? 0.004 : 0.0025;
		this.camera.zoomAt(Mathf.exp(-event.delta.y * rate), event.position.x, event.position.y);
	}

	override keyboardEvent(event: KeyboardEvent): bool {
		if (event.phase != "down") {
			return false;
		}
		let key = <i32>event.key;
		if (key == KEY_P || key == KEY_SPACE) {
			this.press(ID_PAUSE);
		} else if (key == KEY_O) {
			this.press(ID_STEP);
		} else if (key == KEY_R) {
			this.press(ID_RESTART);
		} else if (key == KEY_T) {
			this.setParallel(!this.parallel);
		} else if (key == KEY_LEFT_BRACKET) {
			this.press(ID_FEWER);
		} else if (key == KEY_RIGHT_BRACKET) {
			this.press(ID_MORE);
		} else if (key == KEY_UP) {
			this.openScene(this.current - 1);
		} else if (key == KEY_DOWN) {
			this.openScene(this.current + 1);
		} else if (key >= KEY_1 && key <= KEY_9) {
			this.openScene(key - KEY_1);
		} else if (key == KEY_F) {
			this.home(false);
		} else if (key == KEY_J) {
			this.draws.showJoints = !this.draws.showJoints;
		} else if (key == KEY_G) {
			this.draws.showGrid = !this.draws.showGrid;
		} else if (key == KEY_H) {
			this.hudOn = !this.hudOn;
			this.layoutCamera();
		} else {
			return false;
		}
		return true;
	}
}
