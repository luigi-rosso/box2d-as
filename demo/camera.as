// The demo's view of the world: a center in meters and a zoom in points per meter, both easing toward a target,
// inside the part of the window the panels leave clear.

const MIN_ZOOM: f32 = 0.5;
const MAX_ZOOM: f32 = 2000;

export class Camera {
	width: f32 = 1280;
	height: f32 = 800;
	// The clear area's left edge, in points
	inset: f32 = 0;

	x: f32 = 0;
	y: f32 = 0;
	zoom: f32 = 20;
	targetX: f32 = 0;
	targetY: f32 = 0;
	targetZoom: f32 = 20;

	// The screen point of the world origin, valid after update()
	originX: f32 = 0;
	originY: f32 = 0;

	setViewport(width: f32, height: f32, inset: f32): void {
		this.width = width;
		this.height = height;
		this.inset = inset;
		this.update();
	}

	@inline get centerX(): f32 {
		return this.inset + 0.5 * (this.width - this.inset);
	}

	@inline get centerY(): f32 {
		return 0.5 * this.height;
	}

	update(): void {
		this.originX = this.centerX - this.x * this.zoom;
		this.originY = this.centerY + this.y * this.zoom;
	}

	@inline worldX(screenX: f32): f32 {
		return (screenX - this.originX) / this.zoom;
	}

	@inline worldY(screenY: f32): f32 {
		return (this.originY - screenY) / this.zoom;
	}

	// Frames a world box with a margin; snap skips the ease
	fit(minX: f32, minY: f32, maxX: f32, maxY: f32, snap: bool): void {
		let w = Mathf.max(maxX - minX, 1);
		let h = Mathf.max(maxY - minY, 1);
		let zoomX = 0.8 * (this.width - this.inset) / w;
		let zoomY = 0.8 * this.height / h;
		this.targetZoom = Mathf.min(Mathf.max(Mathf.min(zoomX, zoomY), MIN_ZOOM), MAX_ZOOM);
		this.targetX = 0.5 * (minX + maxX);
		this.targetY = 0.5 * (minY + maxY);
		if (snap) {
			this.x = this.targetX;
			this.y = this.targetY;
			this.zoom = this.targetZoom;
			this.update();
		}
	}

	// A drag by a screen delta
	pan(dx: f32, dy: f32): void {
		this.targetX -= dx / this.zoom;
		this.targetY += dy / this.zoom;
		this.x = this.targetX;
		this.y = this.targetY;
		this.update();
	}

	// Zooms by factor, keeping the world point under a screen point in place
	zoomAt(factor: f32, screenX: f32, screenY: f32): void {
		let wx = this.worldX(screenX);
		let wy = this.worldY(screenY);
		let zoom = Mathf.min(Mathf.max(this.targetZoom * factor, MIN_ZOOM), MAX_ZOOM);
		this.targetZoom = zoom;
		this.targetX = wx - (screenX - this.centerX) / zoom;
		this.targetY = wy + (screenY - this.centerY) / zoom;
	}

	advance(dt: f32): void {
		let k = 1 - Mathf.exp(-12 * dt);
		this.x += (this.targetX - this.x) * k;
		this.y += (this.targetY - this.y) * k;
		this.zoom += (this.targetZoom - this.zoom) * k;
		this.update();
	}
}
