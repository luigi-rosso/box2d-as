// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/contact_solver.as, mirrors verify/cref/contact_solver.c: the overflow and wide kernels over
// synthetic colors, contacts and body states. The wide kernels run on v128 lanes with --enable-simd and one f32
// lane at a time without it; both must print C's scalar bits.

import { vtag, vi32, vbool, vf32, vu64, vend, vseed, vrandint, vrandf } from "./common";
import { b2IntegrateRotation, b2MakeRot, b2MulAdd } from "../../lib/math_functions";
import { b2CreateBitSet, b2SetBitCountAndClear } from "../../lib/bitset";
import { b2BodyState, b2_dynamicFlag } from "../../lib/body";
import { b2ContactSim, b2_simEnableHitEvent } from "../../lib/contact";
import { B2_OVERFLOW_INDEX } from "../../lib/constraint_graph";
import { b2SolverSet } from "../../lib/solver_set";
import { b2TaskContext, b2World } from "../../lib/physics_world";
import { b2ContactPrepareSpan, b2MakeSoft, b2SolverBlock, b2StepContext, b2_graphContactBlock } from "../../lib/solver";
import {
	b2ApplyRestitution_Overflow,
	b2ApplyRestitution_WideW4,
	b2ContactConstraint,
	b2ContactConstraintWide,
	b2PrepareContacts_Overflow,
	b2PrepareContacts_WideW4,
	b2PushContacts_Overflow,
	b2PushContacts_WideW4,
	b2SolveContacts_Overflow,
	b2SolveContacts_WideW4,
	b2StoreImpulses_Overflow,
	b2StoreImpulses_WideW4,
	b2WarmStartContacts_Overflow,
	b2WarmStartContacts_WideW4,
} from "../../lib/contact_solver";

export { frameBoundary } from "./common";

const MAX_STATES: i32 = 24;
const MAX_CONTACTS: i32 = 16;
const COLOR_COUNT: i32 = 2;
const MAX_WIDE: i32 = (COLOR_COUNT * MAX_CONTACTS) / 4;

const colorIds: StaticArray<i32> = [3, 7];

const world = new b2World();
const ctx = new b2StepContext(world);
const taskContext = new b2TaskContext();
const states = new Array<b2BodyState>();
const colorSims = new StaticArray<Array<b2ContactSim>>(COLOR_COUNT);
const overflowSims = new Array<b2ContactSim>();
const wide = new StaticArray<b2ContactConstraintWide>(MAX_WIDE);
const overflowConstraints = new StaticArray<b2ContactConstraint>(MAX_CONTACTS);
const spans = new StaticArray<b2ContactPrepareSpan>(COLOR_COUNT + 1);
const colorCounts = new StaticArray<i32>(COLOR_COUNT);
const wideStarts = new StaticArray<i32>(COLOR_COUNT);
const wideCounts = new StaticArray<i32>(COLOR_COUNT);
let wideTotal = 0;
let overflowCount = 0;

unsafe function words(p: usize, byteCount: usize): void {
	for (let i: usize = 0; i < byteCount / 4; ++i) {
		vf32(load<f32>(p + 4 * i));
	}
}

function genState(): b2BodyState {
	let s = b2BodyState();
	let kind = vrandint(0, 8);
	s.flags = kind == 0 ? 0 : b2_dynamicFlag;
	s.linearVelocity.x = vrandf(-5.0, 5.0);
	s.linearVelocity.y = vrandf(-5.0, 5.0);
	s.angularVelocity = vrandf(-5.0, 5.0);
	s.deltaPosition.x = vrandf(-0.1, 0.1);
	s.deltaPosition.y = vrandf(-0.1, 0.1);
	let angle = vrandf(-0.2, 0.2);
	s.deltaRotation = b2MakeRot(angle);
	return s;
}

function isDynamic(encoded: i32): bool {
	return encoded >= 0 && (states[encoded].flags & b2_dynamicFlag) != 0;
}

// The cursor of C's pickBody, in a FixedArray since rasc refuses a ref to a local
function pickBody(cursor: ref FixedArray<i32, 1>): i32 {
	let kind = vrandint(0, 4);
	if (kind == 0) {
		let staticIndex = vrandint(0, 3);
		return -(staticIndex + 2);
	}

	if (kind == 1) {
		for (let i = 0; i < states.length; ++i) {
			if ((states[i].flags & b2_dynamicFlag) == 0) {
				return i;
			}
		}
		return -2;
	}

	while (cursor[0] < states.length && (states[cursor[0]].flags & b2_dynamicFlag) == 0) {
		cursor[0] += 1;
	}

	if (cursor[0] < states.length) {
		let i = cursor[0];
		cursor[0] += 1;
		return i;
	}

	return -2;
}

function genContact(encodedA: i32, encodedB: i32, contactId: i32): b2ContactSim {
	let c = b2ContactSim();
	c.contactId = contactId;
	c.encodedBodySimA = encodedA;
	c.encodedBodySimB = encodedB;

	let hit = vrandint(0, 2);
	c.simFlags = hit == 0 ? b2_simEnableHitEvent : 0;

	c.invMassA = isDynamic(encodedA) ? vrandf(0.1, 2.0) : <f32>0.0;
	c.invIA = isDynamic(encodedA) ? vrandf(0.1, 2.0) : <f32>0.0;
	c.invMassB = isDynamic(encodedB) ? vrandf(0.1, 2.0) : <f32>0.0;
	c.invIB = isDynamic(encodedB) ? vrandf(0.1, 2.0) : <f32>0.0;

	let angle = vrandf(-3.14, 3.14);
	let r = b2MakeRot(angle);
	c.manifold.normal.x = r.c;
	c.manifold.normal.y = r.s;

	c.friction = vrandf(0.0, 1.0);
	let pick = vrandint(0, 3);
	c.restitution = pick == 0 ? vrandf(0.0, 1.0) : <f32>0.0;
	pick = vrandint(0, 4);
	c.rollingResistance = pick == 0 ? vrandf(0.0, 0.5) : <f32>0.0;
	pick = vrandint(0, 4);
	c.tangentSpeed = pick == 0 ? vrandf(-1.0, 1.0) : <f32>0.0;

	c.manifold.pointCount = vrandint(1, 3);
	c.manifold.rollingImpulse = vrandf(-0.1, 0.1);

	for (let p = 0; p < 2; ++p) {
		ref mp = c.manifold.points[p];
		mp.anchorA.x = vrandf(-1.0, 1.0);
		mp.anchorA.y = vrandf(-1.0, 1.0);
		mp.anchorB.x = vrandf(-1.0, 1.0);
		mp.anchorB.y = vrandf(-1.0, 1.0);
		mp.separation = vrandf(-0.05, 0.05);
		mp.normalImpulse = vrandf(0.0, 2.0);
		mp.tangentImpulse = vrandf(-1.0, 1.0);
		mp.totalNormalImpulse = vrandf(0.0, 1.0);
		mp.normalVelocity = vrandf(-5.0, 5.0);
	}

	return c;
}

function dumpSim(c: b2ContactSim): void {
	vf32(c.manifold.rollingImpulse);
	for (let p = 0; p < 2; ++p) {
		vf32(c.manifold.points[p].normalImpulse);
		vf32(c.manifold.points[p].tangentImpulse);
		vf32(c.manifold.points[p].totalNormalImpulse);
		vf32(c.manifold.points[p].normalVelocity);
	}
}

unsafe function dump(phase: string): void {
	vtag(phase);
	vi32(ctx.anyRestitution.value);
	vbool(taskContext.hasHitEvents);
	vu64(taskContext.hitEventBitSet.bits[0]);
	vend();

	let stateData = load<usize>(changetype<usize>(states) + offsetof<Array<i32>>("dataStart"));
	for (let i = 0; i < states.length; ++i) {
		vtag("state");
		words(stateData + <usize>i * sizeof<b2BodyState>(), sizeof<b2BodyState>());
		vend();
	}

	for (let i = 0; i < wideTotal; ++i) {
		vtag("wide");
		words(changetype<usize>(wide) + <usize>i * sizeof<b2ContactConstraintWide>(), sizeof<b2ContactConstraintWide>());
		vend();
	}

	for (let i = 0; i < overflowCount; ++i) {
		vtag("ovf");
		words(changetype<usize>(overflowConstraints) + <usize>i * sizeof<b2ContactConstraint>(), sizeof<b2ContactConstraint>());
		vend();
	}

	for (let k = 0; k < COLOR_COUNT; ++k) {
		let sims = colorSims[k];
		for (let i = 0; i < colorCounts[k]; ++i) {
			vtag("sim");
			dumpSim(sims[i]);
			vend();
		}
	}

	for (let i = 0; i < overflowCount; ++i) {
		vtag("osim");
		dumpSim(overflowSims[i]);
		vend();
	}
}

function makeBlock(start: i32, count: i32, colorIndex: i32): b2SolverBlock {
	let block = b2SolverBlock();
	block.startIndex = start;
	block.count = <u16>count;
	block.blockType = <u8>b2_graphContactBlock;
	block.colorIndex = <u8>colorIndex;
	return block;
}

export function begin(): void {
	vtag("sizes");
	vi32(<i32>sizeof<b2ContactConstraintWide>());
	vi32(<i32>sizeof<b2ContactConstraint>());
	vi32(<i32>sizeof<b2BodyState>());
	vend();

	for (let i = 0; i < 3; ++i) {
		world.solverSets.push(new b2SolverSet());
	}
	world.solverSets[2].bodyStates = states;
	world.taskContexts.push(taskContext);
	taskContext.hitEventBitSet = b2CreateBitSet(64);

	for (let k = 0; k < COLOR_COUNT; ++k) {
		colorSims[k] = new Array<b2ContactSim>();
	}
	for (let k = 0; k <= COLOR_COUNT; ++k) {
		spans[k] = new b2ContactPrepareSpan();
	}

	ctx.graph = world.constraintGraph;
	ctx.states = states;
	ctx.contactPrepareSpans = spans;
	ctx.wideContactConstraints = wide;

	vseed(0x5eed2d);
}

export unsafe function scenario(): void {
	let stateCount = vrandint(4, MAX_STATES + 1);
	states.length = 0;
	for (let i = 0; i < stateCount; ++i) {
		states.push(genState());
	}

	let zeroWide: b2ContactConstraintWide;
	let zeroConstraint: b2ContactConstraint;
	let w = wide;
	for (let i = 0; i < MAX_WIDE; ++i) {
		w[i] = zeroWide;
	}
	let o = overflowConstraints;
	for (let i = 0; i < MAX_CONTACTS; ++i) {
		o[i] = zeroConstraint;
	}

	let contactId = 0;
	wideTotal = 0;
	for (let k = 0; k < COLOR_COUNT; ++k) {
		let cursor: FixedArray<i32, 1>;
		cursor[0] = 0;
		let count = vrandint(1, MAX_CONTACTS + 1);
		let sims = colorSims[k];
		sims.length = 0;
		for (let i = 0; i < count; ++i) {
			let a = pickBody(cursor);
			let b = pickBody(cursor);
			sims.push(genContact(a, b, contactId));
			contactId += 1;
		}

		colorCounts[k] = count;
		wideStarts[k] = wideTotal;
		wideCounts[k] = ((count - 1) >> 2) + 1;
		wideTotal += wideCounts[k];
	}

	overflowCount = vrandint(0, 7);
	overflowSims.length = 0;
	for (let i = 0; i < overflowCount; ++i) {
		let ca: FixedArray<i32, 1>;
		ca[0] = vrandint(0, stateCount);
		let cb: FixedArray<i32, 1>;
		cb[0] = vrandint(0, stateCount);
		let a = pickBody(ca);
		let b = pickBody(cb);
		overflowSims.push(genContact(a, b, contactId));
		contactId += 1;
	}

	world.hitEventThreshold = vrandf(0.5, 2.0);
	world.restitutionThreshold = vrandf(0.5, 2.0);
	let propagate = vrandint(0, 2);
	world.enableRestitutionPropagation = propagate == 1;
	let warm = vrandint(0, 4);
	world.enableWarmStarting = warm != 0;
	world.contactSpeed = 3.0;

	let dt: f32 = 1.0 / 60.0;
	ctx.dt = dt;
	ctx.inv_dt = 1.0 / dt;
	ctx.subStepCount = 4;
	ctx.h = dt / 4.0;
	ctx.inv_h = 4.0 * ctx.inv_dt;
	ctx.contactSoftness = b2MakeSoft(30.0, 10.0, ctx.h);
	ctx.staticSoftness = b2MakeSoft(60.0, 10.0, ctx.h);
	ctx.anyRestitution.value = 0;

	b2SetBitCountAndClear(taskContext.hitEventBitSet, 64);
	taskContext.hasHitEvents = false;

	for (let k = 0; k < COLOR_COUNT; ++k) {
		let color = world.constraintGraph.colors[colorIds[k]];
		color.contactSims = colorSims[k];
		color.wideConstraintStart = wideStarts[k];
		color.wideConstraintCount = wideCounts[k];

		let span = spans[k];
		span.start = wideStarts[k];
		span.count = colorCounts[k];
		span.contacts = colorSims[k];
	}

	let sentinel = spans[COLOR_COUNT];
	sentinel.start = wideTotal;
	sentinel.count = 0;

	let overflow = world.constraintGraph.colors[B2_OVERFLOW_INDEX];
	overflow.contactSims = overflowSims;
	overflow.overflowConstraints = overflowConstraints;

	let split = vrandint(0, wideTotal + 1);
	vtag("scenario");
	vi32(stateCount);
	vi32(wideTotal);
	vi32(overflowCount);
	vi32(split);
	vend();

	if (split > 0) {
		b2PrepareContacts_WideW4(makeBlock(0, split, 0), ctx);
	}
	if (wideTotal - split > 0) {
		b2PrepareContacts_WideW4(makeBlock(split, wideTotal - split, 0), ctx);
	}
	b2PrepareContacts_Overflow(ctx);
	dump("prepare");

	let h = ctx.h;
	for (let sub = 0; sub < 4; ++sub) {
		b2WarmStartContacts_Overflow(ctx);
		for (let k = 0; k < COLOR_COUNT; ++k) {
			b2WarmStartContacts_WideW4(makeBlock(0, wideCounts[k], colorIds[k]), ctx);
		}

		b2PushContacts_Overflow(ctx);
		for (let k = 0; k < COLOR_COUNT; ++k) {
			b2PushContacts_WideW4(makeBlock(0, wideCounts[k], colorIds[k]), ctx);
		}

		for (let i = 0; i < stateCount; ++i) {
			if ((states[i].flags & b2_dynamicFlag) != 0) {
				states[i].deltaPosition = b2MulAdd(states[i].deltaPosition, h, states[i].linearVelocity);
				states[i].deltaRotation = b2IntegrateRotation(states[i].deltaRotation, h * states[i].angularVelocity);
			}
		}

		b2SolveContacts_Overflow(ctx);
		for (let k = 0; k < COLOR_COUNT; ++k) {
			b2SolveContacts_WideW4(makeBlock(0, wideCounts[k], colorIds[k]), ctx);
		}
	}
	dump("solve");

	for (let j = 0; j < 2; ++j) {
		b2ApplyRestitution_Overflow(ctx);
		for (let k = 0; k < COLOR_COUNT; ++k) {
			b2ApplyRestitution_WideW4(makeBlock(0, wideCounts[k], colorIds[k]), ctx);
		}
	}
	dump("restitution");

	b2StoreImpulses_Overflow(ctx);
	if (split > 0) {
		b2StoreImpulses_WideW4(makeBlock(0, split, 0), ctx, 0);
	}
	if (wideTotal - split > 0) {
		b2StoreImpulses_WideW4(makeBlock(split, wideTotal - split, 0), ctx, 0);
	}
	dump("store");
}
