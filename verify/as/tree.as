// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/aabb.as, lib/dynamic_tree.as and lib/broad_phase_core.as, mirrors verify/cref/tree.c

import { vtag, vi32, vu32, vu64, vbool, vf32, vend, vseed, vrand64, vrandint, vrandf, vspecialf } from "./common";
import { UINT64_MAX } from "../../lib/core";
import {
	b2AABB,
	b2AABB_Center,
	b2AABB_Extents,
	b2AABB_Union,
	b2Add,
	b2MulSV,
	b2Pos,
	b2Sub,
	b2Vec2,
	b2Vec2_zero,
} from "../../lib/math_functions";
import { b2BoxCastInput, b2DynamicTree, b2RayCastInput, b2TreeStats } from "../../lib/collision";
import { b2Capacity, b2_bodyTypeCount, b2_dynamicBody, b2_kinematicBody, b2_staticBody } from "../../lib/types";
import { B2_SHAPE_PAIR_KEY, b2AddKey, b2RemoveKey } from "../../lib/table";
import { b2AABB_RayCast, b2EnlargeAABB, b2IsValidAABB, b2OffsetAABB, b2Perimeter } from "../../lib/aabb";
import {
	b2CreateDynamicTree,
	b2CreateTreeProxy,
	b2CreateTreeProxyInternal,
	b2DestroyDynamicTree,
	b2DestroyTreeProxy,
	b2DynamicTree_CastBox,
	b2DynamicTree_CastRay,
	b2DynamicTree_ClearMoved,
	b2DynamicTree_EnlargeProxy,
	b2DynamicTree_GatherMovedProxies,
	b2DynamicTree_GetAreaRatio,
	b2DynamicTree_GetByteCount,
	b2DynamicTree_GetCategoryBits,
	b2DynamicTree_GetHeight,
	b2DynamicTree_GetProxyBounds,
	b2DynamicTree_GetProxyCount,
	b2DynamicTree_GetRootBounds,
	b2DynamicTree_GetUserData,
	b2DynamicTree_MarkProxyMoved,
	b2DynamicTree_MarkProxyMovedSerial,
	b2DynamicTree_MoveProxy,
	b2DynamicTree_MoveProxyInternal,
	b2DynamicTree_OverlapAABB,
	b2DynamicTree_OverlapAABBAll,
	b2DynamicTree_Rebuild,
	b2DynamicTree_Refit,
	b2DynamicTree_SetCategoryBits,
	b2HasTreeMoved,
	b2NeedsRebuild,
	B2_LEAF_NODE,
} from "../../lib/dynamic_tree";
import {
	B2_CROSS_SEED_COUNT,
	B2_PROXY_ID,
	B2_PROXY_TYPE,
	b2BroadPhase,
	b2BroadPhase_CreateProxy,
	b2BroadPhase_DestroyProxy,
	b2BroadPhase_MarkProxyMoved,
	b2BroadPhase_MarkProxyMovedSerial,
	b2BroadPhase_MoveProxy,
	b2CreateBroadPhase,
	b2CrossPairsTask,
	b2DestroyBroadPhase,
	b2GatherCrossSeeds,
	b2GatherMovedSiblings,
	b2NodePair,
	b2PairContext,
	b2RadixSortKeys,
	b2SelfPairsTask,
	b2ValidateBroadphase,
} from "../../lib/broad_phase_core";

export { frameBoundary } from "./common";

// ---- shared helpers ----

let g_hash: u64 = 0;

@inline function mix(v: u32): void {
	g_hash ^= <u64>v;
	g_hash *= 0x100000001b3;
}

@inline function mixF(f: f32): void {
	mix(reinterpret<u32>(f));
}

@inline function mix64(v: u64): void {
	mix(<u32>v);
	mix(<u32>(v >> 32));
}

// C reads the height and shape index union raw; here the field the node uses
@inline function unionValue(flagIndex: u32, height: i32, shapeIndex: i32): i32 {
	return (flagIndex & B2_LEAF_NODE) != 0 ? shapeIndex : height;
}

function treeHash(tree: b2DynamicTree): u64 {
	g_hash = 0xcbf29ce484222325;
	mix(<u32>tree.nodeEnd);
	mix(<u32>tree.nodeCapacity);
	mix(<u32>tree.pairFreeList);
	mix(<u32>tree.proxyCount);
	mix(<u32>tree.proxyCapacity);
	mix(<u32>tree.proxyFreeList);
	mix(tree.dfsOrdered ? 1 : 0);
	mix(<u32>tree.rebuildCapacity);
	mix(tree.swapNodes.length != 0 ? 1 : 0);
	let nodes = tree.nodes;
	let parents = tree.parents;
	for (let i = 0; i < tree.nodeEnd; ++i) {
		mixF(nodes[i].aabb.lowerBound.x);
		mixF(nodes[i].aabb.lowerBound.y);
		mixF(nodes[i].aabb.upperBound.x);
		mixF(nodes[i].aabb.upperBound.y);
		mix(nodes[i].flagIndex);
		mix(<u32>unionValue(nodes[i].flagIndex, nodes[i].height, nodes[i].shapeIndex));
		mix(<u32>parents[i]);
	}
	let proxies = tree.proxies;
	for (let i = 0; i < tree.proxyCapacity; ++i) {
		mix64(proxies[i].userData);
		mix64(proxies[i].categoryBits);
		mix(<u32>proxies[i].node);
		mix(<u32>proxies[i].next);
	}
	return g_hash;
}

function vaabb(a: b2AABB): void {
	vf32(a.lowerBound.x);
	vf32(a.lowerBound.y);
	vf32(a.upperBound.x);
	vf32(a.upperBound.y);
}

function dumpTree(tag: string, tree: b2DynamicTree): void {
	vtag(tag);
	vi32(tree.nodeEnd);
	vi32(tree.nodeCapacity);
	vi32(tree.pairFreeList);
	vi32(tree.proxyCount);
	vi32(tree.proxyCapacity);
	vi32(tree.proxyFreeList);
	vbool(tree.dfsOrdered);
	vi32(tree.rebuildCapacity);
	vbool(tree.swapNodes.length != 0);
	vi32(b2DynamicTree_GetHeight(tree));
	vf32(b2DynamicTree_GetAreaRatio(tree));
	vaabb(b2DynamicTree_GetRootBounds(tree));
	vi32(b2DynamicTree_GetProxyCount(tree));
	vi32(b2DynamicTree_GetByteCount(tree));
	vbool(b2HasTreeMoved(tree));
	vbool(b2NeedsRebuild(tree));
	vend();
	let nodes = tree.nodes;
	for (let i = 0; i < tree.nodeEnd; ++i) {
		vtag("n");
		vi32(i);
		vaabb(nodes[i].aabb);
		vu32(nodes[i].flagIndex);
		vi32(unionValue(nodes[i].flagIndex, nodes[i].height, nodes[i].shapeIndex));
		vi32(tree.parents[i]);
		vend();
	}
	let proxies = tree.proxies;
	for (let i = 0; i < tree.proxyCapacity; ++i) {
		vtag("p");
		vi32(i);
		vu64(proxies[i].userData);
		vu64(proxies[i].categoryBits);
		vi32(proxies[i].node);
		vi32(proxies[i].next);
		vend();
	}
}

// ---- aabb ----

function randVec(span: f32): b2Vec2 {
	let v: b2Vec2;
	v.x = vrandf(-span, span);
	v.y = vrandf(-span, span);
	return v;
}

export function aabbChecks(): void {
	vseed(21);
	for (let i = 0; i < 4000; ++i) {
		let a: b2AABB;
		a.lowerBound.x = vspecialf();
		a.lowerBound.y = vspecialf();
		a.upperBound.x = vspecialf();
		a.upperBound.y = vspecialf();
		let b: b2AABB;
		b.lowerBound.x = vspecialf();
		b.lowerBound.y = vspecialf();
		b.upperBound.x = vspecialf();
		b.upperBound.y = vspecialf();
		vtag("aabb");
		vf32(b2Perimeter(a));
		vbool(b2IsValidAABB(a));
		let c: FixedArray<b2AABB, 1>;
		c[0] = a;
		vbool(b2EnlargeAABB(c[0], b));
		vaabb(c[0]);
		let origin: b2Pos;
		origin.x = vspecialf();
		origin.y = vspecialf();
		vaabb(b2OffsetAABB(a, origin));
		vend();
	}

	for (let i = 0; i < 6000; ++i) {
		let a: b2AABB;
		a.lowerBound = randVec(10.0);
		a.upperBound.x = a.lowerBound.x + vrandf(0.0, 5.0);
		a.upperBound.y = a.lowerBound.y + vrandf(0.0, 5.0);
		let p1 = randVec(15.0);
		let p2: b2Vec2;
		let kind = vrandint(0, 4);
		if (kind == 0) {
			p2 = randVec(15.0);
		} else if (kind == 1) {
			p2.x = p1.x;
			p2.y = vrandf(-15.0, 15.0);
		} else if (kind == 2) {
			p2.x = vrandf(-15.0, 15.0);
			p2.y = p1.y;
		} else {
			p2 = p1;
		}
		let out = b2AABB_RayCast(a, p1, p2);
		vtag("ray");
		vf32(out.normal.x);
		vf32(out.normal.y);
		vf32(out.point.x);
		vf32(out.point.y);
		vf32(out.fraction);
		vi32(out.iterations);
		vbool(out.hit);
		vend();
	}
}

// ---- tree ----

let g_tree: b2DynamicTree = new b2DynamicTree();
let g_boxKind = 0;
let g_mode = 0;
let g_hitBudget = 0;
let g_depth = 0;
let g_nestBudget = 0;

function randBox(): b2AABB {
	let box: b2AABB;
	if (g_boxKind == 0) {
		let cx = vrandf(-30.0, 30.0);
		let cy = vrandf(-30.0, 30.0);
		let hx = vrandf(0.05, 2.0);
		let hy = vrandf(0.05, 2.0);
		box.lowerBound.x = cx - hx;
		box.lowerBound.y = cy - hy;
		box.upperBound.x = cx + hx;
		box.upperBound.y = cy + hy;
	} else if (g_boxKind == 1) {
		let cx = <f32>vrandint(-6, 7);
		let cy = <f32>vrandint(-6, 7);
		let h: f32 = 0.5 * <f32>vrandint(1, 4);
		box.lowerBound.x = cx - h;
		box.lowerBound.y = cy - h;
		box.upperBound.x = cx + h;
		box.upperBound.y = cy + h;
	} else if (g_boxKind == 2) {
		let cx = vrandf(-10.0, 10.0);
		let cy = vrandf(-10.0, 10.0);
		let flat = vrandint(0, 3);
		let hx: f32 = flat == 1 ? 0.0 : vrandf(0.0, 1.0);
		let hy: f32 = flat == 2 ? 0.0 : vrandf(0.0, 1.0);
		if (flat == 0) {
			hx = 0.0;
			hy = 0.0;
		}
		box.lowerBound.x = cx - hx;
		box.lowerBound.y = cy - hy;
		box.upperBound.x = cx + hx;
		box.upperBound.y = cy + hy;
	} else if (g_boxKind == 3) {
		let cx: f32 = 100000.0 + vrandf(-60.0, 60.0);
		let cy: f32 = -50000.0 + vrandf(-60.0, 60.0);
		let hx = vrandf(0.01, 3.0);
		let hy = vrandf(0.01, 3.0);
		box.lowerBound.x = cx - hx;
		box.lowerBound.y = cy - hy;
		box.upperBound.x = cx + hx;
		box.upperBound.y = cy + hy;
	} else if (g_boxKind == 4) {
		box.lowerBound.x = -1.0;
		box.lowerBound.y = -1.0;
		box.upperBound.x = 1.0;
		box.upperBound.y = 1.0;
	} else {
		// a line of boxes
		let cx = vrandf(-200.0, 200.0);
		let h = vrandf(0.1, 1.0);
		box.lowerBound.x = cx - h;
		box.lowerBound.y = -h;
		box.upperBound.x = cx + h;
		box.upperBound.y = h;
	}
	return box;
}

function growBox(a: b2AABB): b2AABB {
	let b = a;
	b.lowerBound.x -= vrandf(0.0, 0.5);
	b.lowerBound.y -= vrandf(0.0, 0.5);
	b.upperBound.x += vrandf(0.0, 0.5);
	b.upperBound.y += vrandf(0.0, 0.5);
	return b;
}

function randBits(): u64 {
	let kind = vrandint(0, 4);
	if (kind == 0) {
		return UINT64_MAX;
	}
	if (kind == 1) {
		return (<u64>1) << <u64>vrandint(0, 8);
	}
	if (kind == 2) {
		return vrand64();
	}
	return <u64>vrandint(0, 16);
}

function overlapCallback(proxyId: i32, userData: u64): bool {
	vtag("qh");
	vi32(g_depth);
	vi32(proxyId);
	vu64(userData);
	vend();

	if (g_mode == 2) {
		g_hitBudget -= 1;
		return g_hitBudget > 0;
	}

	if (g_mode == 4 && g_depth < 3 && g_nestBudget > 0 && proxyId % 3 == 0) {
		let s = nestedOverlap(proxyId);
		vtag("qn");
		vi32(s.nodeVisits);
		vi32(s.leafVisits);
		vend();
	}

	return true;
}

function nestedOverlap(proxyId: i32): b2TreeStats {
	g_nestBudget -= 1;
	g_depth += 1;
	let box = b2DynamicTree_GetProxyBounds(g_tree, proxyId);
	let s = b2DynamicTree_OverlapAABB(g_tree, box, UINT64_MAX, overlapCallback);
	g_depth -= 1;
	return s;
}

function rayCallback(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
	let value = input.maxFraction;
	if (g_mode == 2) {
		g_hitBudget -= 1;
		value = g_hitBudget > 0 ? input.maxFraction : 0.0;
	} else if (g_mode == 3 && (proxyId & 1) == 1) {
		value = -1.0;
	} else if (g_mode != 0) {
		if (g_mode == 4 && g_depth < 2 && g_nestBudget > 0) {
			let s = nestedOverlap(proxyId);
			vtag("rn");
			vi32(s.nodeVisits);
			vi32(s.leafVisits);
			vend();
		}

		let box = b2DynamicTree_GetProxyBounds(g_tree, proxyId);
		let p2 = b2Add(input.origin, input.translation);
		let out = b2AABB_RayCast(box, input.origin, p2);
		if (out.hit && out.fraction < input.maxFraction) {
			value = out.fraction;
		}
	}

	vtag("rh");
	vi32(proxyId);
	vu64(userData);
	vf32(input.maxFraction);
	vf32(value);
	vend();
	return value;
}

function boxCallback(input: b2BoxCastInput, proxyId: i32, userData: u64): f32 {
	let value = input.maxFraction;
	if (g_mode == 2) {
		g_hitBudget -= 1;
		value = g_hitBudget > 0 ? input.maxFraction : 0.0;
	} else if (g_mode == 3 && (proxyId & 1) == 1) {
		value = -1.0;
	} else if (g_mode != 0) {
		if (g_mode == 4 && g_depth < 2 && g_nestBudget > 0) {
			let s = nestedOverlap(proxyId);
			vtag("bn");
			vi32(s.nodeVisits);
			vi32(s.leafVisits);
			vend();
		}

		// Minkowski sum of the proxy box and the cast box, swept by the box center
		let box = b2DynamicTree_GetProxyBounds(g_tree, proxyId);
		let e = b2AABB_Extents(input.box);
		box.lowerBound = b2Sub(box.lowerBound, e);
		box.upperBound = b2Add(box.upperBound, e);
		let p1 = b2AABB_Center(input.box);
		let p2 = b2Add(p1, input.translation);
		let out = b2AABB_RayCast(box, p1, p2);
		if (out.hit && out.fraction < input.maxFraction) {
			value = out.fraction;
		}
	}

	vtag("bh");
	vi32(proxyId);
	vu64(userData);
	vf32(input.maxFraction);
	vf32(value);
	vend();
	return value;
}

// The visitor forms of the callbacks, so both dispatch paths of the generic queries run
class QueryVisitor {
	visit(proxyId: i32, userData: u64): bool {
		return overlapCallback(proxyId, userData);
	}
}

class RayVisitor {
	visit(input: b2RayCastInput, proxyId: i32, userData: u64): f32 {
		return rayCallback(input, proxyId, userData);
	}
}

class BoxVisitor {
	visit(input: b2BoxCastInput, proxyId: i32, userData: u64): f32 {
		return boxCallback(input, proxyId, userData);
	}
}

const g_queryVisitor = new QueryVisitor();
const g_rayVisitor = new RayVisitor();
const g_boxVisitor = new BoxVisitor();

const MAX_LIVE: i32 = 700;

const g_live = new StaticArray<i32>(MAX_LIVE);
let g_liveCount = 0;
const g_moved = new StaticArray<i32>(4096);
let g_maxLive = 0;

function vhash(): void {
	vu64(treeHash(g_tree));
	vend();
}

export function treeBegin(): void {
	vseed(7);
}

// Returns the op count of the round
export function treeStart(round: i32): i32 {
	let capacity = 0;
	switch (round % 6) {
		case 0:
			capacity = 0;
			break;
		case 1:
			capacity = 1;
			break;
		case 2:
			capacity = 16;
			break;
		case 3:
			capacity = 40;
			break;
		case 4:
			capacity = 5;
			break;
		default:
			capacity = 100;
			break;
	}
	g_boxKind = round % 6;
	g_tree = b2CreateDynamicTree(capacity);
	g_liveCount = 0;
	vtag("round");
	vi32(round);
	vi32(capacity);
	vhash();

	g_maxLive = round < 6 ? 120 : MAX_LIVE;
	return round < 6 ? 400 : 900;
}

export function treeOp(): void {
	let r = vrandint(0, 100);
	if ((g_liveCount < 6 || r < 30) && g_liveCount < g_maxLive) {
		let box = randBox();
		let bits = randBits();
		let userData = vrand64();
		let which = vrandint(0, 3);
		let proxyId: i32;
		if (which == 0) {
			proxyId = b2CreateTreeProxy(g_tree, box, bits, userData);
		} else {
			proxyId = b2CreateTreeProxyInternal(g_tree, box, bits, userData, which == 2);
		}
		g_live[g_liveCount++] = proxyId;
		vtag("create");
		vi32(which);
		vi32(proxyId);
	} else if (r < 40) {
		let pick = vrandint(0, g_liveCount);
		let proxyId = g_live[pick];
		g_live[pick] = g_live[g_liveCount - 1];
		g_liveCount -= 1;
		b2DestroyTreeProxy(g_tree, proxyId);
		vtag("destroy");
		vi32(proxyId);
	} else if (r < 52) {
		let proxyId = g_live[vrandint(0, g_liveCount)];
		let box = randBox();
		let which = vrandint(0, 3);
		if (which == 0) {
			b2DynamicTree_MoveProxy(g_tree, proxyId, box);
		} else {
			b2DynamicTree_MoveProxyInternal(g_tree, proxyId, box, which == 2);
		}
		vtag("move");
		vi32(which);
		vi32(proxyId);
	} else if (r < 58) {
		let proxyId = g_live[vrandint(0, g_liveCount)];
		let box = growBox(b2DynamicTree_GetProxyBounds(g_tree, proxyId));
		b2DynamicTree_EnlargeProxy(g_tree, proxyId, box);
		vtag("enlarge");
		vi32(proxyId);
	} else if (r < 64) {
		let proxyId = g_live[vrandint(0, g_liveCount)];
		let box = growBox(b2DynamicTree_GetProxyBounds(g_tree, proxyId));
		b2DynamicTree_MarkProxyMoved(g_tree, proxyId, box);
		vtag("mark");
		vi32(proxyId);
	} else if (r < 67) {
		let proxyId = g_live[vrandint(0, g_liveCount)];
		b2DynamicTree_MarkProxyMovedSerial(g_tree, proxyId);
		vtag("marks");
		vi32(proxyId);
	} else if (r < 69) {
		let proxyId = g_live[vrandint(0, g_liveCount)];
		let bits = randBits();
		b2DynamicTree_SetCategoryBits(g_tree, proxyId, bits);
		vtag("cat");
		vi32(proxyId);
		vu64(b2DynamicTree_GetCategoryBits(g_tree, proxyId));
		vu64(b2DynamicTree_GetUserData(g_tree, proxyId));
	} else if (r < 72) {
		b2DynamicTree_Refit(g_tree);
		vtag("refit");
	} else if (r < 78) {
		let full = r >= 76;
		let count = b2DynamicTree_Rebuild(g_tree, full);
		vtag("rebuild");
		vbool(full);
		vi32(count);
	} else if (r < 80) {
		// The solver always refits first; a clear on stale ancestors would leave them unmarked.
		b2DynamicTree_Refit(g_tree);
		b2DynamicTree_ClearMoved(g_tree);
		vtag("clear");
	} else if (r < 82) {
		let count = b2DynamicTree_GatherMovedProxies(g_tree, g_moved);
		vtag("gather");
		vi32(count);
		for (let i = 0; i < count; ++i) {
			vi32(g_moved[i]);
		}
	} else if (r < 90) {
		let box = randBox();
		let mask = randBits();
		g_mode = vrandint(0, 5);
		g_hitBudget = vrandint(1, 6);
		g_nestBudget = 6;
		let visitor = vrandint(0, 2) == 1;
		let all = r >= 88;
		vtag("query");
		vbool(all);
		vi32(g_mode);
		vend();
		let s: b2TreeStats;
		if (all) {
			s = visitor ? b2DynamicTree_OverlapAABBAll(g_tree, box, g_queryVisitor) : b2DynamicTree_OverlapAABBAll(g_tree, box, overlapCallback);
		} else {
			s = visitor
				? b2DynamicTree_OverlapAABB(g_tree, box, mask, g_queryVisitor)
				: b2DynamicTree_OverlapAABB(g_tree, box, mask, overlapCallback);
		}
		vtag("qs");
		vi32(s.nodeVisits);
		vi32(s.leafVisits);
	} else if (r < 95) {
		let input: b2RayCastInput;
		let box = randBox();
		input.origin = b2AABB_Center(box);
		let kind = vrandint(0, 5);
		if (kind == 0) {
			input.translation.x = vrandf(-60.0, 60.0);
			input.translation.y = 0.0;
		} else if (kind == 1) {
			input.translation.x = 0.0;
			input.translation.y = vrandf(-60.0, 60.0);
		} else if (kind == 2) {
			input.translation = b2Vec2_zero;
		} else {
			input.translation.x = vrandf(-60.0, 60.0);
			input.translation.y = vrandf(-60.0, 60.0);
		}
		input.maxFraction = vrandint(0, 2) == 0 ? 1.0 : vrandf(0.2, 1.0);
		let mask = randBits();
		g_mode = vrandint(0, 5);
		g_hitBudget = vrandint(1, 6);
		g_nestBudget = 6;
		let visitor = vrandint(0, 2) == 1;
		vtag("ray");
		vi32(g_mode);
		vend();
		let s = visitor ? b2DynamicTree_CastRay(g_tree, input, mask, g_rayVisitor) : b2DynamicTree_CastRay(g_tree, input, mask, rayCallback);
		vtag("rs");
		vi32(s.nodeVisits);
		vi32(s.leafVisits);
	} else {
		let input: b2BoxCastInput;
		let box = randBox();
		let shrink = vrandf(0.0, 0.9);
		let c = b2AABB_Center(box);
		let e = b2MulSV(1.0 - shrink, b2AABB_Extents(box));
		input.box.lowerBound = b2Sub(c, e);
		input.box.upperBound = b2Add(c, e);
		input.translation.x = vrandf(-40.0, 40.0);
		input.translation.y = vrandf(-40.0, 40.0);
		input.maxFraction = vrandint(0, 2) == 0 ? 1.0 : vrandf(0.2, 1.0);
		let mask = randBits();
		g_mode = vrandint(0, 5);
		g_hitBudget = vrandint(1, 6);
		g_nestBudget = 6;
		let visitor = vrandint(0, 2) == 1;
		vtag("box");
		vi32(g_mode);
		vend();
		let s = visitor ? b2DynamicTree_CastBox(g_tree, input, mask, g_boxVisitor) : b2DynamicTree_CastBox(g_tree, input, mask, boxCallback);
		vtag("bs");
		vi32(s.nodeVisits);
		vi32(s.leafVisits);
	}
	vhash();
}

export function treeEnd(): void {
	dumpTree("tree", g_tree);

	// Solver style finish, then drain
	b2DynamicTree_Refit(g_tree);
	vtag("final");
	vi32(b2DynamicTree_Rebuild(g_tree, false));
	vi32(b2DynamicTree_Rebuild(g_tree, true));
	vhash();
	while (g_liveCount > 0) {
		let proxyId = g_live[--g_liveCount];
		b2DestroyTreeProxy(g_tree, proxyId);
	}
	vtag("drained");
	vi32(b2DynamicTree_Rebuild(g_tree, true));
	vhash();
	dumpTree("empty", g_tree);

	b2DestroyDynamicTree(g_tree);
}

// ---- broad phase ----

// The fake world's filter: shape s lives on body s / 2, and every other test passes
class TestFilter {
	shouldCreatePair(shapeIdA: i32, shapeIdB: i32): bool {
		return shapeIdA >> 1 != shapeIdB >> 1;
	}
}

const BP_MAX: i32 = 600;

let g_bp = new b2BroadPhase();
let g_pairKeys = new Array<u64>();
let g_context = new b2PairContext<TestFilter>(new TestFilter(), g_bp.pairSet, g_pairKeys);
let g_shapeCount = 0;
const g_proxyKeys = new StaticArray<i32>(BP_MAX);
const g_proxyShapes = new StaticArray<i32>(BP_MAX);
let g_proxyCount = 0;
const g_pairs = new StaticArray<u64>(1 << 15);
let g_pairCount = 0;
const g_siblings = new StaticArray<i32>(1 << 13);
const g_seeds = new StaticArray<b2NodePair>(2 * B2_CROSS_SEED_COUNT);
const g_sortKeys = new StaticArray<u64>(1 << 15);
const g_tempKeys = new StaticArray<u64>(1 << 15);
let g_round = 0;

function removePairsOf(shape: i32): void {
	let i = 0;
	while (i < g_pairCount) {
		let key = g_pairs[i];
		let a = <i32>(key >> 32);
		let b = <i32>(key & 0xffffffff);
		if (a == shape || b == shape) {
			b2RemoveKey(g_bp.pairSet, key);
			g_pairs[i] = g_pairs[g_pairCount - 1];
			g_pairCount -= 1;
		} else {
			i += 1;
		}
	}
}

// b2ParallelFor with one worker and every task inline: the blocks run in order
function runCross(itemCount: i32, staticSeedCount: i32): void {
	if (itemCount <= 0) {
		return;
	}
	let minRange = 1;
	let maxBlockCount = 8;
	let blockSize = itemCount <= minRange * maxBlockCount ? minRange : (itemCount + maxBlockCount - 1) / maxBlockCount;
	for (let start = 0; start < itemCount; start += blockSize) {
		let end = start + blockSize < itemCount ? start + blockSize : itemCount;
		b2CrossPairsTask(start, end, g_bp, g_seeds, staticSeedCount, g_context);
	}
}

function runSelf(itemCount: i32): void {
	if (itemCount <= 0) {
		return;
	}
	let minRange = 64;
	let maxBlockCount = 8;
	let blockSize = itemCount <= minRange * maxBlockCount ? minRange : (itemCount + maxBlockCount - 1) / maxBlockCount;
	for (let start = 0; start < itemCount; start += blockSize) {
		let end = start + blockSize < itemCount ? start + blockSize : itemCount;
		b2SelfPairsTask(start, end, g_bp, g_context);
	}
}

function vtrees(): void {
	vtag("trees");
	for (let t = 0; t < b2_bodyTypeCount; ++t) {
		vu64(treeHash(g_bp.trees[t]));
	}
	vu32(g_bp.pairSet.count);
	vend();
}

// b2UpdateBroadPhasePairs up to the contacts, which here only enter the pair set
function updatePairs(): void {
	let bp = g_bp;

	let needUpdate = b2HasTreeMoved(bp.trees[b2_staticBody]);
	needUpdate = needUpdate || b2NeedsRebuild(bp.trees[b2_kinematicBody]);
	needUpdate = needUpdate || b2NeedsRebuild(bp.trees[b2_dynamicBody]);

	vtag("update");
	vbool(needUpdate);
	vend();
	if (needUpdate == false) {
		return;
	}

	g_pairKeys.length = 0;

	let dynamicTree = bp.trees[b2_dynamicBody];
	bp.movedSiblings = g_siblings;
	let dynamicMoveCount = b2GatherMovedSiblings(dynamicTree, bp.movedSiblings);

	let staticSeedCount = b2GatherCrossSeeds(dynamicTree, bp.trees[b2_staticBody], g_seeds, 0);
	let kinematicSeedCount = b2GatherCrossSeeds(dynamicTree, bp.trees[b2_kinematicBody], g_seeds, staticSeedCount);
	let crossMoveCount = staticSeedCount + kinematicSeedCount;

	vtag("siblings");
	vi32(dynamicMoveCount);
	for (let i = 0; i < dynamicMoveCount; ++i) {
		vi32(g_siblings[i]);
	}
	vend();
	vtag("seeds");
	vi32(staticSeedCount);
	vi32(kinematicSeedCount);
	for (let i = 0; i < crossMoveCount; ++i) {
		vu32(g_seeds[i].a.flagIndex);
		vu32(g_seeds[i].b.flagIndex);
	}
	vend();

	runCross(crossMoveCount, staticSeedCount);
	runSelf(dynamicMoveCount);

	b2DynamicTree_ClearMoved(bp.trees[b2_staticBody]);

	let dynamicLeaves = b2DynamicTree_Rebuild(bp.trees[b2_dynamicBody], false);
	let kinematicLeaves = b2DynamicTree_Rebuild(bp.trees[b2_kinematicBody], false);

	let pairCount = g_pairKeys.length;
	vtag("pairs");
	vi32(dynamicLeaves);
	vi32(kinematicLeaves);
	vi32(pairCount);
	for (let i = 0; i < pairCount; ++i) {
		g_sortKeys[i] = g_pairKeys[i];
		vu64(g_pairKeys[i]);
	}
	vend();

	if (pairCount > 1) {
		b2RadixSortKeys(g_sortKeys, g_tempKeys, pairCount);
	}

	vtag("sorted");
	for (let i = 0; i < pairCount; ++i) {
		let key = g_sortKeys[i];
		vu64(key);
		b2AddKey(bp.pairSet, key);
		g_pairs[g_pairCount++] = key;
	}
	vend();

	bp.movedSiblings = new StaticArray<i32>(0);
	vtrees();
}

function bpBox(type: i32): b2AABB {
	let box: b2AABB;
	let span: f32 = type == b2_staticBody ? 40.0 : 25.0;
	let cx = vrandf(-span, span);
	let cy = vrandf(-span, span);
	let hx = vrandf(0.1, type == b2_staticBody ? 6.0 : 2.0);
	let hy = vrandf(0.1, 2.0);
	box.lowerBound.x = cx - hx;
	box.lowerBound.y = cy - hy;
	box.upperBound.x = cx + hx;
	box.upperBound.y = cy + hy;
	return box;
}

function shiftBox(a: b2AABB): b2AABB {
	let dx = vrandf(-1.0, 1.0);
	let dy = vrandf(-1.0, 1.0);
	let b: b2AABB;
	b.lowerBound.x = a.lowerBound.x + dx;
	b.lowerBound.y = a.lowerBound.y + dy;
	b.upperBound.x = a.upperBound.x + dx;
	b.upperBound.y = a.upperBound.y + dy;
	return b;
}

export function broadBegin(): void {
	vseed(11);
}

// Returns the step count of the round
export function broadStart(round: i32): i32 {
	let capacity = b2Capacity();
	capacity.staticShapeCount = vrandint(0, 40);
	capacity.dynamicShapeCount = vrandint(0, 80);
	capacity.contactCount = vrandint(0, 100);

	g_bp = new b2BroadPhase();
	b2CreateBroadPhase(g_bp, capacity);
	g_pairKeys = new Array<u64>();
	g_context = new b2PairContext<TestFilter>(new TestFilter(), g_bp.pairSet, g_pairKeys);
	g_shapeCount = 0;
	g_proxyCount = 0;
	g_pairCount = 0;
	g_round = round;

	vtag("bround");
	vi32(round);
	vend();
	vtrees();

	return round < 2 ? 30 : 60;
}

export function broadStep(): void {
	let maxProxies = g_round < 2 ? 60 : BP_MAX;
	let ops = vrandint(1, g_round < 2 ? 12 : 40);
	for (let op = 0; op < ops; ++op) {
		let r = vrandint(0, 100);
		if ((g_proxyCount < 4 || r < 35) && g_proxyCount < maxProxies) {
			let t = vrandint(0, 10);
			let type = t < 3 ? b2_staticBody : t < 4 ? b2_kinematicBody : b2_dynamicBody;
			let box = bpBox(type);
			let bits = randBits();
			let force = vrandint(0, 4) == 0;
			g_shapeCount += 1;
			let shape = g_shapeCount - 1;
			let key = b2BroadPhase_CreateProxy(g_bp, type, box, bits, shape, force);
			g_proxyKeys[g_proxyCount] = key;
			g_proxyShapes[g_proxyCount] = shape;
			g_proxyCount += 1;
			vtag("bcreate");
			vi32(type);
			vbool(force);
			vi32(key);
			vend();
		} else if (r < 45) {
			let pick = vrandint(0, g_proxyCount);
			let key = g_proxyKeys[pick];
			let shape = g_proxyShapes[pick];
			g_proxyKeys[pick] = g_proxyKeys[g_proxyCount - 1];
			g_proxyShapes[pick] = g_proxyShapes[g_proxyCount - 1];
			g_proxyCount -= 1;
			b2BroadPhase_DestroyProxy(g_bp, key);
			removePairsOf(shape);
			vtag("bdestroy");
			vi32(key);
			vend();
		} else if (r < 60) {
			let key = g_proxyKeys[vrandint(0, g_proxyCount)];
			let box = bpBox(B2_PROXY_TYPE(key));
			b2BroadPhase_MoveProxy(g_bp, key, box);
			vtag("bmove");
			vi32(key);
			vend();
		} else if (r < 85) {
			let key = g_proxyKeys[vrandint(0, g_proxyCount)];
			let type = B2_PROXY_TYPE(key);
			if (type == b2_staticBody) {
				continue;
			}
			let tree = g_bp.trees[type];
			let box = shiftBox(b2DynamicTree_GetProxyBounds(tree, B2_PROXY_ID(key)));
			let which = vrandint(0, 3);
			if (which == 0 || type == b2_kinematicBody) {
				b2BroadPhase_MarkProxyMoved(g_bp, key, box);
			} else if (which == 1) {
				let grown = b2AABB_Union(box, b2DynamicTree_GetProxyBounds(tree, B2_PROXY_ID(key)));
				b2DynamicTree_EnlargeProxy(tree, B2_PROXY_ID(key), grown);
			} else {
				b2BroadPhase_MarkProxyMovedSerial(g_bp, key);
			}
			vtag("bmark");
			vi32(which);
			vi32(key);
			vend();
		} else if (g_pairCount > 0) {
			// a contact ends
			let pick = vrandint(0, g_pairCount);
			let key = g_pairs[pick];
			b2RemoveKey(g_bp.pairSet, key);
			g_pairs[pick] = g_pairs[g_pairCount - 1];
			g_pairCount -= 1;
			vtag("bunpair");
			vu64(key);
			vend();
		}
	}

	b2DynamicTree_Refit(g_bp.trees[b2_kinematicBody]);
	b2DynamicTree_Refit(g_bp.trees[b2_dynamicBody]);
	b2ValidateBroadphase(g_bp);
	updatePairs();
}

export function broadEnd(): void {
	for (let t = 0; t < b2_bodyTypeCount; ++t) {
		dumpTree("btree", g_bp.trees[t]);
	}

	b2DestroyBroadPhase(g_bp);
}

// ---- radix sort ----

const g_keys = new StaticArray<u64>(5000);

export function sortChecks(): void {
	vseed(33);
	for (let c = 0; c < 6; ++c) {
		for (let kind = 0; kind < 4; ++kind) {
			let count = c == 0 ? 2 : c == 1 ? 3 : c == 2 ? 17 : c == 3 ? 256 : c == 4 ? 1000 : 4999;
			for (let i = 0; i < count; ++i) {
				let key: u64;
				if (kind == 0) {
					key = vrand64();
				} else if (kind == 1) {
					let a = vrandint(0, 300);
					let b = vrandint(0, 300);
					key = B2_SHAPE_PAIR_KEY(a, b);
				} else if (kind == 2) {
					key = (<u64>vrandint(0, 4)) << 40;
				} else {
					key = 77;
				}
				g_keys[i] = key;
			}
			b2RadixSortKeys(g_keys, g_tempKeys, count);
			vtag("sort");
			vi32(count);
			vi32(kind);
			for (let i = 0; i < count; ++i) {
				vu64(g_keys[i]);
			}
			vend();
		}
	}
}
