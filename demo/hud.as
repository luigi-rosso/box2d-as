// The demo's panels, drawn straight through the renderer each frame: text laid out again only when its string
// changes, and boxes that remember where they were drawn so a press can find them.

import { Renderer } from "rive/renderer";
import { Paint, PaintStyle } from "rive/paint";
import { Path } from "rive/path";
import { Vector } from "rive/vector";
import { Mat2D } from "rive/mat2d";
import { Color } from "rive/color";
import { Font, Text, TextRunStyle } from "rive/text";

export const INK_BACKGROUND: Color = 0xFF151412;
export const INK_LINE: Color = 0xFFEDE8E1;
export const INK_MUTED: Color = 0xFF8C867E;
export const INK_FAINT: Color = 0xFF5A5650;
export const INK_HAIRLINE: Color = 0xFF3B3833;
export const INK_ACCENT: Color = 0xFFE0875A;

export const STYLE_BRAND: i32 = 0;
export const STYLE_ROW: i32 = 1;
export const STYLE_SMALL: i32 = 2;
export const STYLE_VALUE: i32 = 3;
const STYLE_COUNT: i32 = 4;

class Label {
	text: Text = new Text();
	value: string = "";
	style: i32 = -1;
	width: f32 = 0;
	top: f32 = 0;
}

class Hit {
	id: i32 = 0;
	x: f32 = 0;
	y: f32 = 0;
	w: f32 = 0;
	h: f32 = 0;
}

export function rect(path: Path, x: f32, y: f32, w: f32, h: f32): void {
	path.moveTo(Vector.xy(x, y));
	path.lineTo(Vector.xy(x + w, y));
	path.lineTo(Vector.xy(x + w, y + h));
	path.lineTo(Vector.xy(x, y + h));
	path.close();
}

export class Hud {
	private styles: Array<TextRunStyle> = new Array<TextRunStyle>();
	private labels: Array<Label> = new Array<Label>();
	private labelCount: i32 = 0;
	private hits: Array<Hit> = new Array<Hit>();
	private hitCount: i32 = 0;
	private path: Path = new Path();
	private fill: Paint = new Paint();
	private stroke: Paint = new Paint();

	constructor() {
		this.stroke.style = PaintStyle.stroke;
		this.stroke.thickness = 1;
	}

	// The light weight with tabular figures, in the sizes the panels use
	setFont(font: Font): void {
		let axes = new Map<string, f32>();
		axes.set("wght", 300);
		let features = new Map<string, u32>();
		features.set("tnum", 1);
		let light = font.withOptions(axes, features);
		let sizes: StaticArray<f32> = [15, 13, 9.5, 11.5];
		let spacings: StaticArray<f32> = [4.5, 0.2, 1.2, 0.2];
		for (let i = 0; i < STYLE_COUNT; i++) {
			let style = new TextRunStyle(light);
			style.size = sizes[i];
			style.letterSpacing = spacings[i];
			this.styles.push(style);
		}
	}

	begin(): void {
		this.labelCount = 0;
		this.hitCount = 0;
	}

	// Draws value with its glyphs' top at y and x at its left edge, or at its center or right edge for an align of
	// 0.5 or 1. Returns its width.
	text(renderer: Renderer, style: i32, value: string, x: f32, y: f32, color: Color, align: f32 = 0): f32 {
		if (this.styles.length == 0) {
			return 0;
		}
		if (this.labelCount == this.labels.length) {
			this.labels.push(new Label());
		}
		let l = this.labels[this.labelCount++];
		if (l.value != value || l.style != style) {
			l.value = value;
			l.style = style;
			l.text.clear();
			l.text.append(value, this.styles[style]);
			let b = l.text.bounds();
			l.width = b.v1.x - b.v0.x;
			l.top = b.v0.y;
		}
		renderer.save();
		renderer.transform(Mat2D.withTranslation(x - align * l.width, y - l.top));
		this.fill.color = color;
		l.text.draw(renderer, this.fill);
		renderer.restore();
		return l.width;
	}

	box(renderer: Renderer, x: f32, y: f32, w: f32, h: f32, fill: Color, stroke: Color): void {
		let path = this.path;
		path.reset();
		rect(path, x, y, w, h);
		if (fill != 0) {
			this.fill.color = fill;
			renderer.drawPath(path, this.fill);
		}
		if (stroke != 0) {
			this.stroke.color = stroke;
			renderer.drawPath(path, this.stroke);
		}
	}

	// A box a press on which answers id from hit(), lit when on
	button(renderer: Renderer, id: i32, x: f32, y: f32, w: f32, h: f32, on: bool): void {
		this.box(renderer, x, y, w, h, on ? 0x14EDE8E1 : 0, on ? INK_MUTED : INK_HAIRLINE);
		if (this.hitCount == this.hits.length) {
			this.hits.push(new Hit());
		}
		let hit = this.hits[this.hitCount++];
		hit.id = id;
		hit.x = x;
		hit.y = y;
		hit.w = w;
		hit.h = h;
	}

	// A button with its label centered
	labeled(renderer: Renderer, id: i32, label: string, x: f32, y: f32, w: f32, h: f32, on: bool): void {
		this.button(renderer, id, x, y, w, h, on);
		this.text(renderer, STYLE_SMALL, label, x + 0.5 * w, y + 0.5 * h - 3.5, on ? INK_LINE : INK_MUTED, 0.5);
	}

	// The id of the button drawn last frame under a point, or -1
	hit(x: f32, y: f32): i32 {
		for (let i = this.hitCount - 1; i >= 0; i--) {
			let h = this.hits[i];
			if (x >= h.x && x <= h.x + h.w && y >= h.y && y <= h.y + h.h) {
				return h.id;
			}
		}
		return -1;
	}
}
