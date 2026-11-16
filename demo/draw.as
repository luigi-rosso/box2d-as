// Draws a scene's bodies, shapes and joints through the Rive renderer, over the public API. Each body's local
// geometry is read once and kept until its shape count changes, and each kind of body fills one path a frame.

import { Renderer } from "rive/renderer";
import { Paint, PaintStyle } from "rive/paint";
import { Path } from "rive/path";
import { Color } from "rive/color";
import { Camera } from "./camera";
import {
	b2BodyId,
	b2Body_GetJointCount,
	b2Body_GetJoints,
	b2Body_GetShapeCount,
	b2Body_GetShapes,
	b2Body_GetTransform,
	b2Body_GetType,
	b2Body_GetWorldPoint,
	b2Body_IsAwake,
	b2Body_IsBullet,
	b2Body_IsValid,
	b2Capsule,
	b2JointId,
	b2Joint_GetBodyA,
	b2Joint_GetBodyB,
	b2Joint_GetLocalFrameA,
	b2Joint_GetLocalFrameB,
	b2Joint_IsValid,
	b2ShapeId,
	b2Shape_GetCapsule,
	b2Shape_GetChainSegment,
	b2Shape_GetCircle,
	b2Shape_GetPolygon,
	b2Shape_GetSegment,
	b2Shape_GetType,
	b2Shape_IsSensor,
	b2Vec2,
	b2_capsuleShape,
	b2_circleShape,
	b2_dynamicBody,
	b2_polygonShape,
	b2_segmentShape,
	b2_staticBody,
} from "../lib/box2d";

const VERB_MOVE: u8 = 0;
const VERB_LINE: u8 = 1;
const VERB_CUBIC: u8 = 4;
const VERB_CLOSE: u8 = 5;

// A quarter circle as one cubic
const ARC: f32 = 0.5522848;

// Segments in the arc that rounds a polygon's corner
const CORNER_STEPS: i32 = 4;

const KIND_STATIC: i32 = 0;
const KIND_KINEMATIC: i32 = 1;
const KIND_ASLEEP: i32 = 2;
const KIND_AWAKE: i32 = 3;
const KIND_BULLET: i32 = 4;
const KIND_SENSOR: i32 = 5;
const KIND_COUNT: i32 = 6;

const FILLS: StaticArray<Color> = [0xFF34312D, 0xFFD9CBB4, 0xFF8C867E, 0xFFD8D2C8, 0xFFE0875A, 0x30E0875A];
const EDGES: StaticArray<Color> = [0xFF4A4640, 0xFF151412, 0xFF151412, 0xFF151412, 0xFF151412, 0xFFE0875A];

const INK_SEGMENT: Color = 0xFF8C867E;
const INK_JOINT: Color = 0xFFE0875A;
const INK_GRID: Color = 0xFF211F1D;
const INK_AXIS: Color = 0xFF34312D;

const GRID_STEPS: StaticArray<f32> = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000];

// A path's raw verbs and points, filled in script and handed to the renderer in one call. Closed contours wind
// clockwise on screen, which is what the renderer fills.
class Batch {
	verbs: StaticArray<u8> = new StaticArray<u8>(256);
	points: StaticArray<f32> = new StaticArray<f32>(1024);
	verbCount: i32 = 0;
	floatCount: i32 = 0;
	path: Path = new Path();

	clear(): void {
		this.verbCount = 0;
		this.floatCount = 0;
	}

	// Room for one more contour of this many verbs and floats
	reserve(verbs: i32, floats: i32): void {
		if (this.verbCount + verbs > this.verbs.length) {
			let grown = new StaticArray<u8>(2 * (this.verbCount + verbs));
			for (let i = 0; i < this.verbCount; i++) {
				grown[i] = this.verbs[i];
			}
			this.verbs = grown;
		}
		if (this.floatCount + floats > this.points.length) {
			let grown = new StaticArray<f32>(2 * (this.floatCount + floats));
			for (let i = 0; i < this.floatCount; i++) {
				grown[i] = this.points[i];
			}
			this.points = grown;
		}
	}

	// The emitters below write inside what reserve made room for
	@inline private verb(v: u8): void {
		unchecked(this.verbs[this.verbCount] = v);
		this.verbCount += 1;
	}

	@inline private point(x: f32, y: f32): void {
		unchecked(this.points[this.floatCount] = x);
		unchecked(this.points[this.floatCount + 1] = y);
		this.floatCount += 2;
	}

	@inline move(x: f32, y: f32): void {
		this.verb(VERB_MOVE);
		this.point(x, y);
	}

	@inline line(x: f32, y: f32): void {
		this.verb(VERB_LINE);
		this.point(x, y);
	}

	@inline close(): void {
		this.verb(VERB_CLOSE);
	}

	// A quarter turn around (cx, cy) from its point along u to its point along v, both of length r
	@inline private quarter(cx: f32, cy: f32, ux: f32, uy: f32, vx: f32, vy: f32): void {
		this.verb(VERB_CUBIC);
		this.point(cx + ux + ARC * vx, cy + uy + ARC * vy);
		this.point(cx + vx + ARC * ux, cy + vy + ARC * uy);
		this.point(cx + vx, cy + vy);
	}

	segment(x1: f32, y1: f32, x2: f32, y2: f32): void {
		this.reserve(2, 4);
		this.move(x1, y1);
		this.line(x2, y2);
	}

	square(x: f32, y: f32, half: f32): void {
		this.reserve(5, 8);
		this.move(x - half, y - half);
		this.line(x + half, y - half);
		this.line(x + half, y + half);
		this.line(x - half, y + half);
		this.close();
	}

	circle(cx: f32, cy: f32, r: f32): void {
		this.reserve(6, 26);
		this.move(cx + r, cy);
		this.quarter(cx, cy, r, 0, 0, r);
		this.quarter(cx, cy, 0, r, -r, 0);
		this.quarter(cx, cy, -r, 0, 0, -r);
		this.quarter(cx, cy, 0, -r, r, 0);
		this.close();
	}

	capsule(x1: f32, y1: f32, x2: f32, y2: f32, r: f32): void {
		let dx = x2 - x1;
		let dy = y2 - y1;
		let length = Mathf.sqrt(dx * dx + dy * dy);
		if (length < 1e-4) {
			this.circle(x1, y1, r);
			return;
		}
		dx *= r / length;
		dy *= r / length;
		let nx = dy;
		let ny = -dx;
		this.reserve(8, 30);
		this.move(x1 + nx, y1 + ny);
		this.line(x2 + nx, y2 + ny);
		this.quarter(x2, y2, nx, ny, dx, dy);
		this.quarter(x2, y2, dx, dy, -nx, -ny);
		this.line(x1 - nx, y1 - ny);
		this.quarter(x1, y1, -nx, -ny, -dx, -dy);
		this.quarter(x1, y1, -dx, -dy, nx, ny);
		this.close();
	}

	// Local points turned by (a, b), the body's rotation times the zoom, and moved to (tx, ty) on screen
	polygon(x: StaticArray<f32>, y: StaticArray<f32>, tx: f32, ty: f32, a: f32, b: f32): void {
		let n = x.length;
		this.reserve(n + 1, 2 * n);
		for (let i = n - 1; i >= 0; i--) {
			let lx = unchecked(x[i]);
			let ly = unchecked(y[i]);
			let sx = tx + a * lx - b * ly;
			let sy = ty - b * lx - a * ly;
			if (i == n - 1) {
				this.move(sx, sy);
			} else {
				this.line(sx, sy);
			}
		}
		this.close();
	}

	unsafe build(): bool {
		if (this.verbCount == 0) {
			return false;
		}
		this.path.reset();
		this.path.appendRaw(changetype<usize>(this.verbs), <u32>this.verbCount, changetype<usize>(this.points), <u32>this.floatCount);
		return true;
	}
}

class ShapeView {
	type: i32 = 0;
	sensor: bool = false;
	radius: f32 = 0;
	// A polygon's outline, a capsule's or segment's two ends, or a circle's center, in the body's frame
	x: StaticArray<f32> = new StaticArray<f32>(0);
	y: StaticArray<f32> = new StaticArray<f32>(0);
}

class JointView {
	id: b2JointId = b2JointId();
	bodyB: b2BodyId = b2BodyId();
	// The anchors in each body's frame
	ax: f32 = 0;
	ay: f32 = 0;
	bx: f32 = 0;
	by: f32 = 0;
}

class BodyView {
	shapes: Array<ShapeView> = new Array<ShapeView>();
	shapeCount: i32 = -1;
	joints: Array<JointView> = new Array<JointView>();
	jointCount: i32 = -1;
	// How far its shapes reach from its origin
	extent: f32 = 0;
	bullet: bool = false;
}

@inline function sameBody(a: b2BodyId, b: b2BodyId): bool {
	return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

// A polygon's outline, with each corner an arc when it has a radius
function outline(view: ShapeView, shapeId: b2ShapeId): void {
	let polygon = b2Shape_GetPolygon(shapeId);
	let n = polygon.count;
	let r = polygon.radius;
	if (r <= 0) {
		view.x = new StaticArray<f32>(n);
		view.y = new StaticArray<f32>(n);
		for (let i = 0; i < n; i++) {
			view.x[i] = polygon.vertices[i].x;
			view.y[i] = polygon.vertices[i].y;
		}
		return;
	}

	view.x = new StaticArray<f32>(n * (CORNER_STEPS + 1));
	view.y = new StaticArray<f32>(n * (CORNER_STEPS + 1));
	let at = 0;
	for (let i = 0; i < n; i++) {
		let before = polygon.normals[(i + n - 1) % n];
		let after = polygon.normals[i];
		let from = Mathf.atan2(before.y, before.x);
		let turn = Mathf.atan2(before.x * after.y - before.y * after.x, before.x * after.x + before.y * after.y);
		for (let k = 0; k <= CORNER_STEPS; k++) {
			let angle = from + turn * <f32>k / <f32>CORNER_STEPS;
			view.x[at] = polygon.vertices[i].x + r * Mathf.cos(angle);
			view.y[at] = polygon.vertices[i].y + r * Mathf.sin(angle);
			at += 1;
		}
	}
}

function ends(view: ShapeView, p1: b2Vec2, p2: b2Vec2): void {
	view.x = [p1.x, p2.x];
	view.y = [p1.y, p2.y];
}

export class WorldDraw {
	showJoints: bool = true;
	showGrid: bool = true;
	// What the last frame drew
	shapeCount: i32 = 0;

	// A capsule that is no body's shape, in world coordinates
	private moverOn: bool = false;
	private moverX1: f32 = 0;
	private moverY1: f32 = 0;
	private moverX2: f32 = 0;
	private moverY2: f32 = 0;
	private moverRadius: f32 = 0;

	private views: Array<BodyView> = new Array<BodyView>();
	private fills: StaticArray<Batch> = new StaticArray<Batch>(KIND_COUNT);
	private segments: Batch = new Batch();
	private spokes: Batch = new Batch();
	private joints: Batch = new Batch();
	private grid: Batch = new Batch();
	private axes: Batch = new Batch();
	private fill: Paint = new Paint();
	private stroke: Paint = new Paint();
	private shapeIds: StaticArray<b2ShapeId> = new StaticArray<b2ShapeId>(16);
	private jointIds: StaticArray<b2JointId> = new StaticArray<b2JointId>(16);

	constructor() {
		for (let i = 0; i < KIND_COUNT; i++) {
			this.fills[i] = new Batch();
		}
		this.stroke.style = PaintStyle.stroke;
	}

	// The scene made a new world
	reset(): void {
		this.views.length = 0;
	}

	// Draws capsule at position each frame while on
	setMover(on: bool, position: b2Vec2, capsule: b2Capsule): void {
		this.moverOn = on;
		this.moverX1 = position.x + capsule.center1.x;
		this.moverY1 = position.y + capsule.center1.y;
		this.moverX2 = position.x + capsule.center2.x;
		this.moverY2 = position.y + capsule.center2.y;
		this.moverRadius = capsule.radius;
	}

	private view(index: i32, id: b2BodyId): BodyView {
		while (this.views.length <= index) {
			this.views.push(new BodyView());
		}
		let view = this.views[index];
		let shapeCount = b2Body_GetShapeCount(id);
		if (shapeCount != view.shapeCount) {
			this.readShapes(view, id, shapeCount);
		}
		return view;
	}

	private readShapes(view: BodyView, id: b2BodyId, shapeCount: i32): void {
		if (this.shapeIds.length < shapeCount) {
			this.shapeIds = new StaticArray<b2ShapeId>(2 * shapeCount);
		}
		b2Body_GetShapes(id, this.shapeIds, shapeCount);
		view.shapes.length = 0;
		view.shapeCount = shapeCount;
		view.bullet = b2Body_IsBullet(id);
		let extent: f32 = 0;
		for (let i = 0; i < shapeCount; i++) {
			let shapeId = this.shapeIds[i];
			let shape = new ShapeView();
			let type = b2Shape_GetType(shapeId);
			shape.type = type;
			shape.sensor = b2Shape_IsSensor(shapeId);
			if (type == b2_polygonShape) {
				outline(shape, shapeId);
			} else if (type == b2_circleShape) {
				let circle = b2Shape_GetCircle(shapeId);
				shape.x = [circle.center.x];
				shape.y = [circle.center.y];
				shape.radius = circle.radius;
			} else if (type == b2_capsuleShape) {
				let capsule = b2Shape_GetCapsule(shapeId);
				ends(shape, capsule.center1, capsule.center2);
				shape.radius = capsule.radius;
			} else if (type == b2_segmentShape) {
				let segment = b2Shape_GetSegment(shapeId);
				ends(shape, segment.point1, segment.point2);
			} else {
				let segment = b2Shape_GetChainSegment(shapeId).segment;
				ends(shape, segment.point1, segment.point2);
			}
			for (let k = 0; k < shape.x.length; k++) {
				extent = Mathf.max(extent, Mathf.sqrt(shape.x[k] * shape.x[k] + shape.y[k] * shape.y[k]) + shape.radius);
			}
			view.shapes.push(shape);
		}
		view.extent = extent;
	}

	// The joints this body is body A of, so each joint is drawn once
	private unsafe readJoints(view: BodyView, id: b2BodyId, jointCount: i32): void {
		if (this.jointIds.length < jointCount) {
			this.jointIds = new StaticArray<b2JointId>(2 * jointCount);
		}
		b2Body_GetJoints(id, this.jointIds, jointCount);
		view.joints.length = 0;
		view.jointCount = jointCount;
		for (let i = 0; i < jointCount; i++) {
			let jointId = this.jointIds[i];
			if (!sameBody(b2Joint_GetBodyA(jointId), id)) {
				continue;
			}
			let joint = new JointView();
			joint.id = jointId;
			joint.bodyB = b2Joint_GetBodyB(jointId);
			let a = b2Joint_GetLocalFrameA(jointId).p;
			let b = b2Joint_GetLocalFrameB(jointId).p;
			joint.ax = a.x;
			joint.ay = a.y;
			joint.bx = b.x;
			joint.by = b.y;
			view.joints.push(joint);
		}
	}

	private gridLines(camera: Camera): void {
		let step = GRID_STEPS[GRID_STEPS.length - 1];
		for (let i = 0; i < GRID_STEPS.length; i++) {
			if (GRID_STEPS[i] * camera.zoom >= 48) {
				step = GRID_STEPS[i];
				break;
			}
		}
		let left = camera.worldX(0);
		let right = camera.worldX(camera.width);
		let bottom = camera.worldY(camera.height);
		let top = camera.worldY(0);
		for (let k = Mathf.ceil(left / step); k * step <= right; k += 1) {
			let x = camera.originX + k * step * camera.zoom;
			(k == 0 ? this.axes : this.grid).segment(x, 0, x, camera.height);
		}
		for (let k = Mathf.ceil(bottom / step); k * step <= top; k += 1) {
			let y = camera.originY - k * step * camera.zoom;
			(k == 0 ? this.axes : this.grid).segment(0, y, camera.width, y);
		}
	}

	unsafe draw(renderer: Renderer, camera: Camera, bodies: Array<b2BodyId>, count: i32): void {
		for (let i = 0; i < KIND_COUNT; i++) {
			this.fills[i].clear();
		}
		this.segments.clear();
		this.spokes.clear();
		this.joints.clear();
		this.grid.clear();
		this.axes.clear();
		if (this.showGrid) {
			this.gridLines(camera);
		}

		let zoom = camera.zoom;
		let ox = camera.originX;
		let oy = camera.originY;
		let width = camera.width;
		let height = camera.height;
		let drawn = 0;
		if (this.moverOn) {
			this.fills[KIND_BULLET].capsule(ox + this.moverX1 * zoom, oy - this.moverY1 * zoom, ox + this.moverX2 * zoom,
				oy - this.moverY2 * zoom, this.moverRadius * zoom);
		}
		// Anchors are drawn where a joint is large enough on screen to read
		let anchors = zoom >= 16;
		for (let i = 0; i < count; i++) {
			let id = bodies[i];
			if (!b2Body_IsValid(id)) {
				continue;
			}
			let view = this.view(i, id);
			let xf = b2Body_GetTransform(id);
			let tx = ox + xf.p.x * zoom;
			let ty = oy - xf.p.y * zoom;
			let a = xf.q.c * zoom;
			let b = xf.q.s * zoom;

			if (this.showJoints) {
				let jointCount = b2Body_GetJointCount(id);
				if (jointCount != view.jointCount) {
					this.readJoints(view, id, jointCount);
				}
				for (let k = 0; k < view.joints.length; k++) {
					let joint = view.joints[k];
					if (!b2Joint_IsValid(joint.id)) {
						view.jointCount = -1;
						continue;
					}
					let x1 = tx + a * joint.ax - b * joint.ay;
					let y1 = ty - b * joint.ax - a * joint.ay;
					let p = b2Body_GetWorldPoint(joint.bodyB, b2Vec2(joint.bx, joint.by));
					let x2 = ox + p.x * zoom;
					let y2 = oy - p.y * zoom;
					if (Mathf.max(x1, x2) < 0 || Mathf.min(x1, x2) > width || Mathf.max(y1, y2) < 0 || Mathf.min(y1, y2) > height) {
						continue;
					}
					if (Mathf.abs(x2 - x1) + Mathf.abs(y2 - y1) > 1) {
						this.joints.segment(x1, y1, x2, y2);
					}
					if (anchors) {
						this.joints.square(x1, y1, 1.5);
						this.joints.square(x2, y2, 1.5);
					}
				}
			}

			let reach = view.extent * zoom;
			if (reach < 0.5 || tx + reach < 0 || tx - reach > width || ty + reach < 0 || ty - reach > height) {
				continue;
			}

			let kind = KIND_STATIC;
			let type = b2Body_GetType(id);
			if (type == b2_dynamicBody) {
				kind = b2Body_IsAwake(id) ? (view.bullet ? KIND_BULLET : KIND_AWAKE) : KIND_ASLEEP;
			} else if (type != b2_staticBody) {
				kind = KIND_KINEMATIC;
			}

			for (let k = 0; k < view.shapes.length; k++) {
				let shape = view.shapes[k];
				let x = shape.x;
				let y = shape.y;
				let batch = this.fills[shape.sensor ? KIND_SENSOR : kind];
				let r = shape.radius * zoom;
				drawn += 1;
				if (shape.type == b2_polygonShape) {
					batch.polygon(x, y, tx, ty, a, b);
					continue;
				}
				let x1 = tx + a * x[0] - b * y[0];
				let y1 = ty - b * x[0] - a * y[0];
				if (shape.type == b2_circleShape) {
					batch.circle(x1, y1, r);
					if (r > 4) {
						this.spokes.segment(x1, y1, x1 + xf.q.c * r, y1 - xf.q.s * r);
					}
					continue;
				}
				let x2 = tx + a * x[1] - b * y[1];
				let y2 = ty - b * x[1] - a * y[1];
				if (shape.type == b2_capsuleShape) {
					batch.capsule(x1, y1, x2, y2, r);
				} else {
					this.segments.segment(x1, y1, x2, y2);
				}
			}
		}
		this.shapeCount = drawn;

		let stroke = this.stroke;
		stroke.thickness = 1;
		this.strokeBatch(renderer, this.grid, INK_GRID);
		this.strokeBatch(renderer, this.axes, INK_AXIS);
		// Outlines thin out with the zoom, or they would cover small bodies
		stroke.thickness = Mathf.min(Mathf.max(0.04 * zoom, 0.2), 1);
		for (let i = 0; i < KIND_COUNT; i++) {
			let batch = this.fills[i];
			if (batch.build()) {
				this.fill.color = FILLS[i];
				renderer.drawPath(batch.path, this.fill);
				stroke.color = EDGES[i];
				renderer.drawPath(batch.path, stroke);
			}
		}
		this.strokeBatch(renderer, this.spokes, 0xFF151412);
		stroke.thickness = 1.5;
		this.strokeBatch(renderer, this.segments, INK_SEGMENT);
		stroke.thickness = 1;
		this.strokeBatch(renderer, this.joints, INK_JOINT);
	}

	private unsafe strokeBatch(renderer: Renderer, batch: Batch, color: Color): void {
		if (batch.build()) {
			this.stroke.color = color;
			renderer.drawPath(batch.path, this.stroke);
		}
	}
}
