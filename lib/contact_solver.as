// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver.c and the B2_SIMD_NONE path of src/contact_solver_wide.inl
// at B2_SIMD_WIDTH 4, what contact_solver_w4.c compiles to with BOX2D_DISABLE_SIMD.
// The wide kernels are generic over the lane type of lib/simd.as: v128 under ASC_FEATURE_SIMD, as
// C's SSE2 and NEON builds run them, and f32 one lane at a time without it. They read and write the
// wide constraints in place through their field offsets, as C does through pointers.

import { B2_ASSERT, B2_ASSERT_ENABLED, B2_NULL_INDEX } from "./core";
import { b2Add, b2ClampFloat, b2Cross, b2CrossSV, b2Dot, b2MaxFloat, b2MinFloat, b2MinInt, b2MulAdd, b2MulSV, b2MulSub, b2RightPerp, b2RotateVector, b2Rot, b2Sub, b2Vec2, b2Vec2_zero } from "./math_functions";
import { b2Manifold, b2ManifoldPoint } from "./collision";
import { b2SetBit } from "./bitset";
import { b2ArrayData, b2BodyState, b2DecodeAwakeIndex, b2StateAddress, b2_dynamicFlag, b2_identityStates } from "./body";
import { b2ContactSim, b2_simEnableHitEvent } from "./contact";
import { B2_OVERFLOW_INDEX } from "./constraint_graph";
import { b2_awakeSet } from "./solver_set";
import { b2Softness, b2SolverBlock, b2StepContext } from "./solver";
import {
	b2FloatW,
	b2Vec2W,
	b2RotW,
	b2SplatL,
	b2LoadL,
	b2StoreL,
	b2SetL,
	b2SetLoadL,
	b2LaneL,
	b2AddL,
	b2SubL,
	b2MulL,
	b2DivL,
	b2NegL,
	b2MulAddL,
	b2MulSubL,
	b2MinL,
	b2MaxL,
	b2SymClampL,
	b2GreaterThanL,
	b2LessThanL,
	b2AndL,
	b2BlendL,
	b2DotL,
	b2CrossL,
	b2UnpackLoL,
	b2UnpackHiL,
	b2ColumnsL,
	b2TransposeL,
	b2AllZeroAt,
} from "./simd";

export struct b2ContactConstraintPoint {
	anchorA: b2Vec2 = b2Vec2();
	anchorB: b2Vec2 = b2Vec2();
	baseSeparation: f32 = 0;
	relativeVelocity: f32 = 0;
	normalImpulse: f32 = 0;
	tangentImpulse: f32 = 0;
	totalNormalImpulse: f32 = 0;
	restitutionImpulse: f32 = 0;
	normalMass: f32 = 0;
	tangentMass: f32 = 0;
}

export struct b2ContactConstraint {
	// base-1, 0 for null
	indexA: i32 = 0;
	indexB: i32 = 0;
	points: FixedArray<b2ContactConstraintPoint, 2>;
	normal: b2Vec2 = b2Vec2();
	invMassA: f32 = 0;
	invMassB: f32 = 0;
	invIA: f32 = 0;
	invIB: f32 = 0;
	friction: f32 = 0;
	restitution: f32 = 0;
	tangentSpeed: f32 = 0;
	rollingResistance: f32 = 0;
	rollingMass: f32 = 0;
	rollingImpulse: f32 = 0;
	softness: b2Softness = b2Softness();
	pointCount: i32 = 0;
}

// contact_solver_wide.inl

// Soft contact constraints with sub-stepping support.
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules).
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

export struct b2ContactConstraintWide {
	// Read common
	indexA: FixedArray<i32, 4>;
	indexB: FixedArray<i32, 4>;

	invMassA: b2FloatW = b2FloatW();
	invMassB: b2FloatW = b2FloatW();
	invIA: b2FloatW = b2FloatW();
	invIB: b2FloatW = b2FloatW();
	normal: b2Vec2W = b2Vec2W();
	anchorA1: b2Vec2W = b2Vec2W();
	anchorB1: b2Vec2W = b2Vec2W();
	anchorA2: b2Vec2W = b2Vec2W();
	anchorB2: b2Vec2W = b2Vec2W();
	normalMass1: b2FloatW = b2FloatW();
	normalMass2: b2FloatW = b2FloatW();
	baseSeparation1: b2FloatW = b2FloatW();
	baseSeparation2: b2FloatW = b2FloatW();

	// Write in push and solve
	normalImpulse1: b2FloatW = b2FloatW();
	normalImpulse2: b2FloatW = b2FloatW();
	totalNormalImpulse1: b2FloatW = b2FloatW();
	totalNormalImpulse2: b2FloatW = b2FloatW();

	// Write in solve
	tangentImpulse1: b2FloatW = b2FloatW();
	tangentImpulse2: b2FloatW = b2FloatW();
	rollingImpulse: b2FloatW = b2FloatW();

	// Read in solve
	friction: b2FloatW = b2FloatW();
	tangentSpeed: b2FloatW = b2FloatW();
	rollingResistance: b2FloatW = b2FloatW();
	tangentMass1: b2FloatW = b2FloatW();
	tangentMass2: b2FloatW = b2FloatW();
	restitution: b2FloatW = b2FloatW();
	relativeVelocity1: b2FloatW = b2FloatW();
	relativeVelocity2: b2FloatW = b2FloatW();
	restitutionImpulse1: b2FloatW = b2FloatW();
	restitutionImpulse2: b2FloatW = b2FloatW();
}

// Wide version of b2BodyState. The kernels hold its lanes in locals instead.
export struct b2BodyStateW {
	v: b2Vec2W = b2Vec2W();
	w: b2FloatW = b2FloatW();
	flags: b2FloatW = b2FloatW();
	dp: b2Vec2W = b2Vec2W();
	dq: b2RotW = b2RotW();
}

export function b2GetWideContactConstraintByteCountW4(): i32 {
	return <i32>sizeof<b2ContactConstraintWide>();
}

// Field offsets. C reads wide fields through float pointers, lane i of a b2FloatW at 4 i bytes.

const W_Y = offsetof<b2Vec2W>("Y");
const CW_INDEXA = offsetof<b2ContactConstraintWide>("indexA");
const CW_INDEXB = offsetof<b2ContactConstraintWide>("indexB");
const CW_INVMASSA = offsetof<b2ContactConstraintWide>("invMassA");
const CW_INVMASSB = offsetof<b2ContactConstraintWide>("invMassB");
const CW_INVIA = offsetof<b2ContactConstraintWide>("invIA");
const CW_INVIB = offsetof<b2ContactConstraintWide>("invIB");
const CW_NORMALX = offsetof<b2ContactConstraintWide>("normal") + offsetof<b2Vec2W>("X");
const CW_NORMALY = offsetof<b2ContactConstraintWide>("normal") + offsetof<b2Vec2W>("Y");
const CW_ANCHORA1X = offsetof<b2ContactConstraintWide>("anchorA1") + offsetof<b2Vec2W>("X");
const CW_ANCHORA1Y = offsetof<b2ContactConstraintWide>("anchorA1") + offsetof<b2Vec2W>("Y");
const CW_ANCHORB1X = offsetof<b2ContactConstraintWide>("anchorB1") + offsetof<b2Vec2W>("X");
const CW_ANCHORB1Y = offsetof<b2ContactConstraintWide>("anchorB1") + offsetof<b2Vec2W>("Y");
const CW_ANCHORA2X = offsetof<b2ContactConstraintWide>("anchorA2") + offsetof<b2Vec2W>("X");
const CW_ANCHORA2Y = offsetof<b2ContactConstraintWide>("anchorA2") + offsetof<b2Vec2W>("Y");
const CW_ANCHORB2X = offsetof<b2ContactConstraintWide>("anchorB2") + offsetof<b2Vec2W>("X");
const CW_ANCHORB2Y = offsetof<b2ContactConstraintWide>("anchorB2") + offsetof<b2Vec2W>("Y");
const CW_NORMALMASS1 = offsetof<b2ContactConstraintWide>("normalMass1");
const CW_NORMALMASS2 = offsetof<b2ContactConstraintWide>("normalMass2");
const CW_BASESEPARATION1 = offsetof<b2ContactConstraintWide>("baseSeparation1");
const CW_BASESEPARATION2 = offsetof<b2ContactConstraintWide>("baseSeparation2");
const CW_NORMALIMPULSE1 = offsetof<b2ContactConstraintWide>("normalImpulse1");
const CW_NORMALIMPULSE2 = offsetof<b2ContactConstraintWide>("normalImpulse2");
const CW_TOTALNORMALIMPULSE1 = offsetof<b2ContactConstraintWide>("totalNormalImpulse1");
const CW_TOTALNORMALIMPULSE2 = offsetof<b2ContactConstraintWide>("totalNormalImpulse2");
const CW_TANGENTIMPULSE1 = offsetof<b2ContactConstraintWide>("tangentImpulse1");
const CW_TANGENTIMPULSE2 = offsetof<b2ContactConstraintWide>("tangentImpulse2");
const CW_ROLLINGIMPULSE = offsetof<b2ContactConstraintWide>("rollingImpulse");
const CW_FRICTION = offsetof<b2ContactConstraintWide>("friction");
const CW_TANGENTSPEED = offsetof<b2ContactConstraintWide>("tangentSpeed");
const CW_ROLLINGRESISTANCE = offsetof<b2ContactConstraintWide>("rollingResistance");
const CW_TANGENTMASS1 = offsetof<b2ContactConstraintWide>("tangentMass1");
const CW_TANGENTMASS2 = offsetof<b2ContactConstraintWide>("tangentMass2");
const CW_RESTITUTION = offsetof<b2ContactConstraintWide>("restitution");
const CW_RELATIVEVELOCITY1 = offsetof<b2ContactConstraintWide>("relativeVelocity1");
const CW_RELATIVEVELOCITY2 = offsetof<b2ContactConstraintWide>("relativeVelocity2");
const CW_RESTITUTIONIMPULSE1 = offsetof<b2ContactConstraintWide>("restitutionImpulse1");
const CW_RESTITUTIONIMPULSE2 = offsetof<b2ContactConstraintWide>("restitutionImpulse2");

// The 32 byte body state is two rows of four floats: [vx vy w flags] and [dpx dpy dqc dqs]
const BS_V = offsetof<b2BodyState>("linearVelocity");
const BS_W = offsetof<b2BodyState>("angularVelocity");
const BS_FLAGS = offsetof<b2BodyState>("flags");
const BS_ROW0 = offsetof<b2BodyState>("linearVelocity");
const BS_ROW1 = offsetof<b2BodyState>("deltaPosition");

const CS_CONTACTID = offsetof<b2ContactSim>("contactId");
const CS_ENCODEDBODYSIMA = offsetof<b2ContactSim>("encodedBodySimA");
const CS_ENCODEDBODYSIMB = offsetof<b2ContactSim>("encodedBodySimB");
const CS_SIMFLAGS = offsetof<b2ContactSim>("simFlags");
const CS_INVMASSA = offsetof<b2ContactSim>("invMassA");
const CS_INVIA = offsetof<b2ContactSim>("invIA");
const CS_INVMASSB = offsetof<b2ContactSim>("invMassB");
const CS_INVIB = offsetof<b2ContactSim>("invIB");
const CS_MANIFOLD = offsetof<b2ContactSim>("manifold");
const CS_FRICTION = offsetof<b2ContactSim>("friction");
const CS_RESTITUTION = offsetof<b2ContactSim>("restitution");
const CS_ROLLINGRESISTANCE = offsetof<b2ContactSim>("rollingResistance");
const CS_TANGENTSPEED = offsetof<b2ContactSim>("tangentSpeed");

const M_NORMAL = offsetof<b2Manifold>("normal");
const M_ROLLINGIMPULSE = offsetof<b2Manifold>("rollingImpulse");
const M_POINTS = offsetof<b2Manifold>("points");
const M_POINTCOUNT = offsetof<b2Manifold>("pointCount");

const MP_ANCHORA = offsetof<b2ManifoldPoint>("anchorA");
const MP_ANCHORB = offsetof<b2ManifoldPoint>("anchorB");
const MP_SEPARATION = offsetof<b2ManifoldPoint>("separation");
const MP_NORMALIMPULSE = offsetof<b2ManifoldPoint>("normalImpulse");
const MP_TANGENTIMPULSE = offsetof<b2ManifoldPoint>("tangentImpulse");
const MP_TOTALNORMALIMPULSE = offsetof<b2ManifoldPoint>("totalNormalImpulse");
const MP_NORMALVELOCITY = offsetof<b2ManifoldPoint>("normalVelocity");

// The manifold and point fields of a contact sim at its address
const CS_NORMAL = CS_MANIFOLD + M_NORMAL;
const CS_ROLLINGIMPULSEM = CS_MANIFOLD + M_ROLLINGIMPULSE;
const CS_POINTCOUNT = CS_MANIFOLD + M_POINTCOUNT;
const CS_POINT0 = CS_MANIFOLD + M_POINTS;
const CS_POINT1 = CS_MANIFOLD + M_POINTS + sizeof<b2ManifoldPoint>();

// functions from contact_solver.c follow

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// C's state pointers: b2StateAddress gives b2_identityStates for a static body, which has no dynamic flag and so is
// never written. Indices come from the constraint the overflow prepare wrote, below the awake body count.
@inline function b2StoreStateVelocity(state: usize, v: b2Vec2, w: f32): void {
	if ((load<u32>(state + BS_FLAGS) & b2_dynamicFlag) != 0) {
		store<f32>(state + BS_V, v.x);
		store<f32>(state + BS_V + 4, v.y);
		store<f32>(state + BS_W, w);
	}
}

export function b2PrepareContacts_Overflow(context: b2StepContext): void {
	let world = context.world;
	let graph = context.graph;
	let color = graph.colors[B2_OVERFLOW_INDEX];
	let constraints = color.overflowConstraints;
	let contactCount = color.contactSims.length;
	let contacts = b2ArrayData(color.contactSims);
	let states = context.states;

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	let contactSoftness = context.contactSoftness;
	let staticSoftness = context.staticSoftness;

	let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;

	for (let i = 0; i < contactCount; ++i) {
		// i is below the color's contact count, so the address is a live sim
		let contactSim = contacts + <usize>i * sizeof<b2ContactSim>();

		let pointCount = load<i32>(contactSim + CS_POINTCOUNT);

		if (B2_ASSERT_ENABLED) B2_ASSERT(0 < pointCount && pointCount <= 2);

		let indexA = b2DecodeAwakeIndex(load<i32>(contactSim + CS_ENCODEDBODYSIMA));
		let indexB = b2DecodeAwakeIndex(load<i32>(contactSim + CS_ENCODEDBODYSIMB));

		ref constraint = constraints[i];

		// 0 is null
		constraint.indexA = indexA + 1;
		constraint.indexB = indexB + 1;
		constraint.normal = load<b2Vec2>(contactSim + CS_NORMAL);
		constraint.friction = load<f32>(contactSim + CS_FRICTION);
		constraint.restitution = load<f32>(contactSim + CS_RESTITUTION);
		constraint.rollingResistance = load<f32>(contactSim + CS_ROLLINGRESISTANCE);
		constraint.rollingImpulse = warmStartScale * load<f32>(contactSim + CS_ROLLINGIMPULSEM);
		constraint.tangentSpeed = load<f32>(contactSim + CS_TANGENTSPEED);
		constraint.pointCount = pointCount;

		let haveRestitution = load<f32>(contactSim + CS_RESTITUTION) > 0.0;
		let hitEvents = (load<u32>(contactSim + CS_SIMFLAGS) & b2_simEnableHitEvent) != 0;
		let sampleVelocity = haveRestitution || hitEvents;
		anyRestitution = anyRestitution || haveRestitution;

		let vA = b2Vec2_zero;
		let wA: f32 = 0.0;
		let vB = b2Vec2_zero;
		let wB: f32 = 0.0;

		if (sampleVelocity) {
			if (indexA != B2_NULL_INDEX) {
				vA = states[indexA].linearVelocity;
				wA = states[indexA].angularVelocity;
			}

			if (indexB != B2_NULL_INDEX) {
				vB = states[indexB].linearVelocity;
				wB = states[indexB].angularVelocity;
			}
		}

		let mA = load<f32>(contactSim + CS_INVMASSA);
		let iA = load<f32>(contactSim + CS_INVIA);
		let mB = load<f32>(contactSim + CS_INVMASSB);
		let iB = load<f32>(contactSim + CS_INVIB);

		if (indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX) {
			constraint.softness = staticSoftness;
		} else {
			constraint.softness = contactSoftness;
		}

		// copy mass into constraint to avoid cache misses during sub-stepping
		constraint.invMassA = mA;
		constraint.invIA = iA;
		constraint.invMassB = mB;
		constraint.invIB = iB;

		{
			let k = iA + iB;
			constraint.rollingMass = k > 0.0 ? <f32>1.0 / k : <f32>0.0;
		}

		let normal = constraint.normal;
		let tangent = b2RightPerp(constraint.normal);

		for (let j = 0; j < pointCount; ++j) {
			let mp = contactSim + CS_POINT0 + <usize>j * sizeof<b2ManifoldPoint>();
			ref cp = constraint.points[j];

			cp.normalImpulse = warmStartScale * load<f32>(mp + MP_NORMALIMPULSE);
			cp.tangentImpulse = warmStartScale * load<f32>(mp + MP_TANGENTIMPULSE);
			cp.totalNormalImpulse = 0.0;
			cp.restitutionImpulse = 0.0;

			let rA = load<b2Vec2>(mp + MP_ANCHORA);
			let rB = load<b2Vec2>(mp + MP_ANCHORB);

			cp.anchorA = rA;
			cp.anchorB = rB;
			cp.baseSeparation = load<f32>(mp + MP_SEPARATION) - b2Dot(b2Sub(rB, rA), normal);

			let rnA = b2Cross(rA, normal);
			let rnB = b2Cross(rB, normal);
			let kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			cp.normalMass = kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0;

			let rtA = b2Cross(rA, tangent);
			let rtB = b2Cross(rB, tangent);
			let kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			cp.tangentMass = kTangent > 0.0 ? <f32>1.0 / kTangent : <f32>0.0;

			// Only compute the normal velocity terms if needed.
			if (sampleVelocity) {
				let vrA = b2Add(vA, b2CrossSV(wA, rA));
				let vrB = b2Add(vB, b2CrossSV(wB, rB));
				let vn = b2Dot(normal, b2Sub(vrB, vrA));

				cp.relativeVelocity = vn;
				store<f32>(mp + MP_NORMALVELOCITY, vn);
			} else {
				cp.relativeVelocity = 0.0;
				store<f32>(mp + MP_NORMALVELOCITY, 0.0);
			}
		}
	}

	if (anyRestitution) {
		// Check if set first? Store per task context and OR?
		context.anyRestitution.value = 1;
	}
}

export function b2WarmStartContacts_Overflow(context: b2StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B2_OVERFLOW_INDEX];
	let constraints = color.overflowConstraints;
	let contactCount = color.contactSims.length;
	let world = context.world;
	let awakeSet = world.solverSets[b2_awakeSet];
	let states = awakeSet.bodyStates;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	// b2StateAddress returns it for a null index.

	for (let i = 0; i < contactCount; ++i) {
		ref constraint = constraints[i];

		let indexA = constraint.indexA - 1;
		let indexB = constraint.indexB - 1;

		let stateA = b2StateAddress(states, indexA);
		let stateB = b2StateAddress(states, indexB);

		let vA = load<b2Vec2>(stateA + BS_V);
		let wA = load<f32>(stateA + BS_W);
		let vB = load<b2Vec2>(stateB + BS_V);
		let wB = load<f32>(stateB + BS_W);

		let mA = constraint.invMassA;
		let iA = constraint.invIA;
		let mB = constraint.invMassB;
		let iB = constraint.invIB;

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		let normal = constraint.normal;
		let tangent = b2RightPerp(constraint.normal);
		let pointCount = constraint.pointCount;

		for (let j = 0; j < pointCount; ++j) {
			ref cp = constraint.points[j];

			// fixed anchors
			let rA = cp.anchorA;
			let rB = cp.anchorB;

			let P = b2Add(b2MulSV(cp.normalImpulse, normal), b2MulSV(cp.tangentImpulse, tangent));

			cp.totalNormalImpulse += cp.normalImpulse;

			wA -= iA * b2Cross(rA, P);
			vA = b2MulAdd(vA, -mA, P);
			wB += iB * b2Cross(rB, P);
			vB = b2MulAdd(vB, mB, P);
		}

		wA -= iA * constraint.rollingImpulse;
		wB += iB * constraint.rollingImpulse;

		b2StoreStateVelocity(stateA, vA, wA);
		b2StoreStateVelocity(stateB, vB, wB);
	}
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
export function b2PushContacts_Overflow(context: b2StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B2_OVERFLOW_INDEX];
	let constraints = color.overflowConstraints;
	let contactCount = color.contactSims.length;
	let world = context.world;
	let awakeSet = world.solverSets[b2_awakeSet];
	let states = awakeSet.bodyStates;

	let inv_h = context.inv_h;
	let contactSpeed = context.world.contactSpeed;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	// b2StateAddress returns it for a null index.

	let rnA: FixedArray<f32, 2>;
	let rnB: FixedArray<f32, 2>;
	let impulses: FixedArray<f32, 2>;

	for (let i = 0; i < contactCount; ++i) {
		ref constraint = constraints[i];
		let mA = constraint.invMassA;
		let iA = constraint.invIA;
		let mB = constraint.invMassB;
		let iB = constraint.invIB;

		let indexA = constraint.indexA - 1;
		let indexB = constraint.indexB - 1;

		let stateA = b2StateAddress(states, indexA);
		let vA = load<b2Vec2>(stateA + BS_V);
		let wA = load<f32>(stateA + BS_W);
		let dqA = load<b2Rot>(stateA + offsetof<b2BodyState>("deltaRotation"));

		let stateB = b2StateAddress(states, indexB);
		let vB = load<b2Vec2>(stateB + BS_V);
		let wB = load<f32>(stateB + BS_W);
		let dqB = load<b2Rot>(stateB + offsetof<b2BodyState>("deltaRotation"));

		let dp = b2Sub(load<b2Vec2>(stateB + BS_ROW1), load<b2Vec2>(stateA + BS_ROW1));

		let normal = constraint.normal;
		let softness = constraint.softness;
		let pointCount = constraint.pointCount;

		let vnc = b2Dot(b2Sub(vB, vA), normal);

		impulses[0] = 0.0;
		impulses[1] = 0.0;
		let normalImpulse: f32 = 0.0;
		let angularA: f32 = 0.0;
		let angularB: f32 = 0.0;

		for (let j = 0; j < pointCount; ++j) {
			ref cp = constraint.points[j];

			// fixed anchor points
			let rA = cp.anchorA;
			let rB = cp.anchorB;

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			let ds = b2Add(dp, b2Sub(b2RotateVector(dqB, rB), b2RotateVector(dqA, rA)));
			let s = cp.baseSeparation + b2Dot(ds, normal);

			let velocityBias: f32;
			let massScale: f32;
			let impulseScale: f32;
			if (s > 0.0) {
				// speculative bias is positive
				velocityBias = s * inv_h;
				massScale = 1.0;
				impulseScale = 0.0;
			} else {
				// overlap bias is negative
				velocityBias = b2MaxFloat(softness.massScale * softness.biasRate * s, -contactSpeed);
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			rnA[j] = b2Cross(rA, normal);
			rnB[j] = b2Cross(rB, normal);

			// Relative normal velocity at contact.
			let vn = vnc + wB * rnB[j] - wA * rnA[j];
			for (let k = 0; k < j; ++k) {
				// Point 2 only depends on the impulse from point 1.
				vn += (mA + mB + iA * rnA[k] * rnA[j] + iB * rnB[k] * rnB[j]) * impulses[k];
			}

			let impulse = b2MaxFloat(-cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse, -cp.normalImpulse);
			cp.normalImpulse += impulse;
			cp.totalNormalImpulse += impulse;

			impulses[j] = impulse;

			// These accumulators are not used until the loop is done.
			normalImpulse += impulse;
			angularA += impulse * rnA[j];
			angularB += impulse * rnB[j];
		}

		let P = b2MulSV(normalImpulse, normal);
		vA = b2MulSub(vA, mA, P);
		wA -= iA * angularA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * angularB;

		b2StoreStateVelocity(stateA, vA, wA);
		b2StoreStateVelocity(stateB, vB, wB);
	}
}

// Solve contacts: normal, friction and rolling resistance.
export function b2SolveContacts_Overflow(context: b2StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B2_OVERFLOW_INDEX];
	let constraints = color.overflowConstraints;
	let contactCount = color.contactSims.length;
	let world = context.world;
	let awakeSet = world.solverSets[b2_awakeSet];
	let states = awakeSet.bodyStates;

	let inv_h = context.inv_h;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	// b2StateAddress returns it for a null index.

	let normalImpulses: FixedArray<f32, 2>;
	let tangentImpulses: FixedArray<f32, 2>;

	// Cache cross products
	let rnA: FixedArray<f32, 2>;
	let rnB: FixedArray<f32, 2>;
	let rtA: FixedArray<f32, 2>;
	let rtB: FixedArray<f32, 2>;

	for (let i = 0; i < contactCount; ++i) {
		ref constraint = constraints[i];
		let mA = constraint.invMassA;
		let iA = constraint.invIA;
		let mB = constraint.invMassB;
		let iB = constraint.invIB;

		let indexA = constraint.indexA - 1;
		let indexB = constraint.indexB - 1;

		let stateA = b2StateAddress(states, indexA);
		let vA = load<b2Vec2>(stateA + BS_V);
		let wA = load<f32>(stateA + BS_W);
		let dqA = load<b2Rot>(stateA + offsetof<b2BodyState>("deltaRotation"));

		let stateB = b2StateAddress(states, indexB);
		let vB = load<b2Vec2>(stateB + BS_V);
		let wB = load<f32>(stateB + BS_W);
		let dqB = load<b2Rot>(stateB + offsetof<b2BodyState>("deltaRotation"));

		let dp = b2Sub(load<b2Vec2>(stateB + BS_ROW1), load<b2Vec2>(stateA + BS_ROW1));

		let normal = constraint.normal;
		let tangent = b2RightPerp(normal);
		let friction = constraint.friction;

		let pointCount = constraint.pointCount;
		let totalNormalImpulse: f32 = 0.0;

		let dv = b2Sub(vB, vA);
		let vnc = b2Dot(dv, normal);

		// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
		//    = dot(vrB - vrA, tangent) - (sA + sB)
		let vtc = b2Dot(dv, tangent) - constraint.tangentSpeed;

		normalImpulses[0] = 0.0;
		normalImpulses[1] = 0.0;
		tangentImpulses[0] = 0.0;
		tangentImpulses[1] = 0.0;

		for (let j = 0; j < pointCount; ++j) {
			ref cp = constraint.points[j];
			rnA[j] = b2Cross(cp.anchorA, normal);
			rnB[j] = b2Cross(cp.anchorB, normal);
			rtA[j] = b2Cross(cp.anchorA, tangent);
			rtB[j] = b2Cross(cp.anchorB, tangent);
		}

		for (let j = 0; j < pointCount; ++j) {
			ref cp = constraint.points[j];

			// Fixed anchor points.
			let rA = cp.anchorA;
			let rB = cp.anchorB;

			// Compute current separation.
			// This is subject to round-off error if the anchor is far from the body center of mass.
			let ds = b2Add(dp, b2Sub(b2RotateVector(dqB, rB), b2RotateVector(dqA, rA)));
			let s = cp.baseSeparation + b2Dot(ds, normal);

			// Speculative bias, zero when overlapped.
			let velocityBias: f32 = s > 0.0 ? s * inv_h : <f32>0.0;

			// Relative normal velocity at contact.
			let vn = vnc + wB * rnB[j] - wA * rnA[j];
			for (let k = 0; k < j; ++k) {
				// This allows getting point 2 normal velocity with only one dependency.
				vn += (mA + mB + iA * rnA[k] * rnA[j] + iB * rnB[k] * rnB[j]) * normalImpulses[k];
			}

			let impulse = b2MaxFloat(-cp.normalMass * (vn + velocityBias), -cp.normalImpulse);
			cp.normalImpulse += impulse;
			cp.totalNormalImpulse += impulse;
			normalImpulses[j] = impulse;

			// This is used for rolling resistance
			totalNormalImpulse += cp.normalImpulse;
		}

		// Rolling resistance
		let rollingImpulse: f32;
		{
			let Cdot = wB - wA;
			for (let j = 0; j < pointCount; ++j) {
				Cdot += (iA * rnA[j] + iB * rnB[j]) * normalImpulses[j];
			}

			let lambda = constraint.rollingImpulse;
			let maxLambda = constraint.rollingResistance * totalNormalImpulse;
			constraint.rollingImpulse = b2ClampFloat(lambda - constraint.rollingMass * Cdot, -maxLambda, maxLambda);
			rollingImpulse = constraint.rollingImpulse - lambda;
		}

		// Friction
		for (let j = 0; j < pointCount; ++j) {
			ref cp = constraint.points[j];

			// relative tangent velocity at contact
			let vt = vtc + wB * rtB[j] - wA * rtA[j] + (iA * rtA[j] + iB * rtB[j]) * rollingImpulse;
			for (let k = 0; k < pointCount; ++k) {
				vt += (iA * rtA[j] * rnA[k] + iB * rtB[j] * rnB[k]) * normalImpulses[k];
			}

			for (let k = 0; k < j; ++k) {
				vt += (mA + mB + iA * rtA[k] * rtA[j] + iB * rtB[k] * rtB[j]) * tangentImpulses[k];
			}

			// clamp the accumulated force
			let maxFriction = friction * cp.normalImpulse;
			let impulse = b2ClampFloat(-cp.tangentMass * vt, -maxFriction - cp.tangentImpulse, maxFriction - cp.tangentImpulse);
			cp.tangentImpulse += impulse;
			tangentImpulses[j] = impulse;
		}

		let normalImpulse: f32 = 0.0;
		let tangentImpulse: f32 = 0.0;
		let angularA = rollingImpulse;
		let angularB = rollingImpulse;
		for (let j = 0; j < pointCount; ++j) {
			normalImpulse += normalImpulses[j];
			tangentImpulse += tangentImpulses[j];
			angularA += normalImpulses[j] * rnA[j] + tangentImpulses[j] * rtA[j];
			angularB += normalImpulses[j] * rnB[j] + tangentImpulses[j] * rtB[j];
		}

		let P = b2Add(b2MulSV(normalImpulse, normal), b2MulSV(tangentImpulse, tangent));
		vA = b2MulSub(vA, mA, P);
		wA -= iA * angularA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * angularB;

		b2StoreStateVelocity(stateA, vA, wA);
		b2StoreStateVelocity(stateB, vB, wB);
	}
}

export function b2ApplyRestitution_Overflow(context: b2StepContext): void {
	let graph = context.graph;
	let color = graph.colors[B2_OVERFLOW_INDEX];
	let constraints = color.overflowConstraints;
	let contactCount = color.contactSims.length;
	let world = context.world;
	let awakeSet = world.solverSets[b2_awakeSet];
	let states = awakeSet.bodyStates;

	let threshold = world.restitutionThreshold;
	let inv_h = context.inv_h;
	let propagate = world.enableRestitutionPropagation;

	for (let i = 0; i < contactCount; ++i) {
		ref constraint = constraints[i];
		if (propagate == false && constraint.restitution == 0.0) {
			continue;
		}

		let mA = constraint.invMassA;
		let iA = constraint.invIA;
		let mB = constraint.invMassB;
		let iB = constraint.invIB;

		let indexA = constraint.indexA - 1;
		let indexB = constraint.indexB - 1;

		let stateA = b2StateAddress(states, indexA);
		let vA = load<b2Vec2>(stateA + BS_V);
		let wA = load<f32>(stateA + BS_W);
		let dqA = load<b2Rot>(stateA + offsetof<b2BodyState>("deltaRotation"));

		let stateB = b2StateAddress(states, indexB);
		let vB = load<b2Vec2>(stateB + BS_V);
		let wB = load<f32>(stateB + BS_W);
		let dqB = load<b2Rot>(stateB + offsetof<b2BodyState>("deltaRotation"));

		let dp = b2Sub(load<b2Vec2>(stateB + BS_ROW1), load<b2Vec2>(stateA + BS_ROW1));

		let normal = constraint.normal;
		let restitution = constraint.restitution;
		let pointCount = constraint.pointCount;

		for (let j = 0; j < pointCount; ++j) {
			ref cp = constraint.points[j];

			let rA = cp.anchorA;
			let rB = cp.anchorB;

			// The total normal impulse is 0 for speculative points.
			let compressionImpulse = cp.totalNormalImpulse - cp.restitutionImpulse;
			let armed = restitution > 0.0 && cp.relativeVelocity < -threshold && compressionImpulse > 0.0;

			let velocityBias: f32;
			if (armed) {
				velocityBias = restitution * cp.relativeVelocity;
			} else {
				let ds = b2Add(dp, b2Sub(b2RotateVector(dqB, rB), b2RotateVector(dqA, rA)));
				let s = cp.baseSeparation + b2Dot(ds, normal);

				velocityBias = s > 0.0 ? s * inv_h : <f32>0.0;
			}

			let vrA = b2Add(vA, b2CrossSV(wA, rA));
			let vrB = b2Add(vB, b2CrossSV(wB, rB));
			let vn = b2Dot(b2Sub(vrB, vrA), normal);

			let impulse = -cp.normalMass * (vn + velocityBias);

			let newImpulse = b2MaxFloat(cp.normalImpulse + impulse, 0.0);
			impulse = newImpulse - cp.normalImpulse;

			let approachImpulse = b2MinFloat(b2MaxFloat(-cp.normalMass * vn, 0.0), b2MaxFloat(impulse, 0.0));

			if (armed) {
				// Poisson kinetic restitution guarantees no energy gain.
				let allowance = restitution * (compressionImpulse + approachImpulse) - cp.restitutionImpulse;
				impulse = b2MinFloat(impulse, approachImpulse + b2MaxFloat(allowance, 0.0));
			}

			cp.normalImpulse += impulse;
			cp.restitutionImpulse += impulse - approachImpulse;
			cp.totalNormalImpulse += impulse;

			let P = b2MulSV(impulse, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(rB, P);
		}

		b2StoreStateVelocity(stateA, vA, wA);
		b2StoreStateVelocity(stateB, vB, wB);
	}
}

export function b2StoreImpulses_Overflow(context: b2StepContext): void {
	let world = context.world;
	let graph = context.graph;
	let color = graph.colors[B2_OVERFLOW_INDEX];
	let constraints = color.overflowConstraints;
	let contactSims = b2ArrayData(color.contactSims);
	let taskContext = world.taskContexts[0];
	let hitEventBitSet = taskContext.hitEventBitSet;
	let contactCount = color.contactSims.length;
	let negHitThreshold = -world.hitEventThreshold;
	let hasHitEvents = taskContext.hasHitEvents;

	for (let i = 0; i < contactCount; ++i) {
		ref constraint = constraints[i];

		// i is below the color's contact count
		let contactSim = contactSims + <usize>i * sizeof<b2ContactSim>();
		let pointCount = load<i32>(contactSim + CS_POINTCOUNT);

		for (let j = 0; j < pointCount; ++j) {
			let mp = contactSim + CS_POINT0 + <usize>j * sizeof<b2ManifoldPoint>();
			store<f32>(mp + MP_NORMALIMPULSE, constraint.points[j].normalImpulse);
			store<f32>(mp + MP_TANGENTIMPULSE, constraint.points[j].tangentImpulse);
			store<f32>(mp + MP_TOTALNORMALIMPULSE, constraint.points[j].totalNormalImpulse);
		}

		if ((load<u32>(contactSim + CS_SIMFLAGS) & b2_simEnableHitEvent) != 0) {
			for (let j = 0; j < pointCount; ++j) {
				let mp = contactSim + CS_POINT0 + <usize>j * sizeof<b2ManifoldPoint>();

				// Need to check total impulse because the point may be speculative and not colliding
				if (load<f32>(mp + MP_NORMALVELOCITY) < negHitThreshold && load<f32>(mp + MP_TOTALNORMALIMPULSE) > 0.0) {
					b2SetBit(hitEventBitSet, <u32>load<i32>(contactSim + CS_CONTACTID));
					hasHitEvents = true;
					break;
				}
			}
		}

		store<f32>(contactSim + CS_ROLLINGIMPULSEM, constraint.rollingImpulse);
	}

	taskContext.hasHitEvents = hasHitEvents;
}

export function b2PrepareContacts_Wide(block: b2SolverBlock, context: b2StepContext): void {
	b2PrepareContacts_WideW4(block, context);
}

export function b2WarmStartContacts_Wide(block: b2SolverBlock, context: b2StepContext): void {
	b2WarmStartContacts_WideW4(block, context);
}

export function b2PushContacts_Wide(block: b2SolverBlock, context: b2StepContext): void {
	b2PushContacts_WideW4(block, context);
}

export function b2SolveContacts_Wide(block: b2SolverBlock, context: b2StepContext): void {
	b2SolveContacts_WideW4(block, context);
}

export function b2ApplyRestitution_Wide(block: b2SolverBlock, context: b2StepContext): void {
	b2ApplyRestitution_WideW4(block, context);
}

export function b2StoreImpulses_Wide(block: b2SolverBlock, context: b2StepContext, workerIndex: i32): void {
	b2StoreImpulses_WideW4(block, context, workerIndex);
}

export function b2GetWideContactConstraintByteCount(simdWidth: i32): i32 {
	return b2GetWideContactConstraintByteCountW4();
}

// contact_solver_wide.inl at width 4

// C points empty lanes at b2_zeroContactSim. Nothing writes to it: normal velocities are stored only for lanes
// with points.
@parallel.shared const b2_zeroContactSims = new StaticArray<b2ContactSim>(1);

// Lanes per kernel call: a v128 kernel covers the four, an f32 kernel one
@inline function b2LaneCountL<T>(): i32 {
	return isFloat<T>() ? 4 : 1;
}

// A lane's body state from its base 1 index, b2_identityStates for 0. Indices come from the prepare, which takes
// them from the contacts' encoded awake indices, so they are below the awake body count.
@inline function b2LaneState(states: usize, identity: usize, indices: usize, lane: i32): usize {
	let index = load<i32>(indices + <usize>lane * 4);
	return index == 0 ? identity : states + <usize>(index - 1) * sizeof<b2BodyState>();
}

// Lane k of four values; constant for a v128 kernel, so it folds
@inline function b2PickLane(k: i32, a: usize, b: usize, c: usize, d: usize): usize {
	return k == 0 ? a : k == 1 ? b : k == 2 ? c : d;
}

// One 16 byte row of each lane's state or manifold point transposed into lanes, as C's SSE2 gather. An f32 kernel
// reads its own lane's four floats.
@inline function b2GatherRowL<T>(p1: usize, p2: usize, p3: usize, p4: usize, offset: usize): b2ColumnsL<T> {
	if (isFloat<T>()) {
		let c: b2ColumnsL<T>;
		c.c0 = load<f32>(p1 + offset);
		c.c1 = load<f32>(p1 + offset + 4);
		c.c2 = load<f32>(p1 + offset + 8);
		c.c3 = load<f32>(p1 + offset + 12);
		return c;
	} else {
		return b2TransposeL<T>(v128.load(p1 + offset), v128.load(p2 + offset), v128.load(p3 + offset), v128.load(p4 + offset));
	}
}

// This writes only the velocities back to the solver bodies. indices points at the lane's body index. The v128
// path stores each dynamic lane's first row [vx vy w flags] with the flags it gathered, as C's SSE2 scatter.
@inline function b2ScatterBodiesL<T>(states: usize, indices: usize, vX: T, vY: T, w: T, flags: T): void {
	if (isFloat<T>()) {
		// zero means null
		let index = load<i32>(indices);
		if (index != 0) {
			let state = states + <usize>(index - 1) * sizeof<b2BodyState>();
			if ((load<u32>(state + BS_FLAGS) & b2_dynamicFlag) != 0) {
				store<f32>(state + BS_V, vX);
				store<f32>(state + BS_V + 4, vY);
				store<f32>(state + BS_W, w);
			}
		}
	} else {
		// [vx1 vy1 vx2 vy2]
		let t1 = b2UnpackLoL(vX, vY);
		// [vx3 vy3 vx4 vy4]
		let t2 = b2UnpackHiL(vX, vY);
		// [w1 f1 w2 f2]
		let t3 = b2UnpackLoL(w, flags);
		// [w3 f3 w4 f4]
		let t4 = b2UnpackHiL(w, flags);

		// A null lane gathered the identity state, whose flags have no dynamic bit
		if ((i32x4.extract_lane(flags, 0) & b2_dynamicFlag) != 0) {
			v128.store(states + <usize>(load<i32>(indices) - 1) * sizeof<b2BodyState>(), v128.shuffle<f32>(t1, t3, 0, 1, 4, 5));
		}

		if ((i32x4.extract_lane(flags, 1) & b2_dynamicFlag) != 0) {
			v128.store(states + <usize>(load<i32>(indices + 4) - 1) * sizeof<b2BodyState>(), v128.shuffle<f32>(t1, t3, 2, 3, 6, 7));
		}

		if ((i32x4.extract_lane(flags, 2) & b2_dynamicFlag) != 0) {
			v128.store(states + <usize>(load<i32>(indices + 8) - 1) * sizeof<b2BodyState>(), v128.shuffle<f32>(t2, t4, 0, 1, 4, 5));
		}

		if ((i32x4.extract_lane(flags, 3) & b2_dynamicFlag) != 0) {
			v128.store(states + <usize>(load<i32>(indices + 12) - 1) * sizeof<b2BodyState>(), v128.shuffle<f32>(t2, t4, 2, 3, 6, 7));
		}
	}
}

// b2SoftMaskW over the lane's body indices
@inline function b2SoftMaskL<T>(indexA: usize, indexB: usize): T {
	if (isFloat<T>()) {
		return load<i32>(indexA) == 0 || load<i32>(indexB) == 0 ? <f32>1.0 : <f32>0.0;
	} else {
		return v128.or(i32x4.eq(v128.load(indexA), i32x4.splat(0)), i32x4.eq(v128.load(indexB), i32x4.splat(0)));
	}
}

// The wide constraints of a color: they start at the color's wideConstraintStart in the step's wide array, which
// b2Solve sized to cover every color's wideConstraintCount.
@inline function b2ColorConstraints(context: b2StepContext, colorIndex: i32): usize {
	let color = context.graph.colors[colorIndex];
	return changetype<usize>(context.wideContactConstraints) + <usize>color.wideConstraintStart * sizeof<b2ContactConstraintWide>();
}

// Prepare one wide constraint. contacts holds the four lanes' contact sim addresses, the zero sim for empty lanes,
// and the lanes' body indices are already in place. An f32 kernel reads only its lane: l0 to l3 all name it.
// hitEvents and haveRestitution are C's decisions across the lanes.
@inline function b2PrepareConstraintL<T>(
	cw0: usize,
	cl0: usize,
	cl1: usize,
	cl2: usize,
	cl3: usize,
	hitEvents: bool,
	haveRestitution: bool,
	states: usize,
	identity: usize,
	warmStartScaleValue: f32
): void {
	for (let lane = 0; lane < b2LaneCountL<T>(); ++lane) {
		let cw = cw0 + <usize>lane * 4;
		let l0 = isFloat<T>() ? lane : 0;
		let l1 = isFloat<T>() ? lane : 1;
		let l2 = isFloat<T>() ? lane : 2;
		let l3 = isFloat<T>() ? lane : 3;
		let c0 = b2PickLane(l0, cl0, cl1, cl2, cl3);
		let c1 = b2PickLane(l1, cl0, cl1, cl2, cl3);
		let c2 = b2PickLane(l2, cl0, cl1, cl2, cl3);
		let c3 = b2PickLane(l3, cl0, cl1, cl2, cl3);

		let zeroW = b2SplatL<T>(0.0);
		let oneW = b2SplatL<T>(1.0);
		let warmStartScale = b2SplatL<T>(warmStartScaleValue);

		// B2_GATHER_LANES
		let invMassA = b2SetLoadL<T>(c0 + CS_INVMASSA, c1 + CS_INVMASSA, c2 + CS_INVMASSA, c3 + CS_INVMASSA);
		let invMassB = b2SetLoadL<T>(c0 + CS_INVMASSB, c1 + CS_INVMASSB, c2 + CS_INVMASSB, c3 + CS_INVMASSB);
		let invIA = b2SetLoadL<T>(c0 + CS_INVIA, c1 + CS_INVIA, c2 + CS_INVIA, c3 + CS_INVIA);
		let invIB = b2SetLoadL<T>(c0 + CS_INVIB, c1 + CS_INVIB, c2 + CS_INVIB, c3 + CS_INVIB);
		let normalX = b2SetLoadL<T>(c0 + CS_NORMAL, c1 + CS_NORMAL, c2 + CS_NORMAL, c3 + CS_NORMAL);
		let normalY = b2SetLoadL<T>(c0 + CS_NORMAL + 4, c1 + CS_NORMAL + 4, c2 + CS_NORMAL + 4, c3 + CS_NORMAL + 4);
		b2StoreL<T>(cw + CW_INVMASSA, invMassA);
		b2StoreL<T>(cw + CW_INVMASSB, invMassB);
		b2StoreL<T>(cw + CW_INVIA, invIA);
		b2StoreL<T>(cw + CW_INVIB, invIB);
		b2StoreL<T>(cw + CW_NORMALX, normalX);
		b2StoreL<T>(cw + CW_NORMALY, normalY);
		b2StoreL<T>(cw + CW_FRICTION, b2SetLoadL<T>(c0 + CS_FRICTION, c1 + CS_FRICTION, c2 + CS_FRICTION, c3 + CS_FRICTION));
		b2StoreL<T>(cw + CW_TANGENTSPEED, b2SetLoadL<T>(c0 + CS_TANGENTSPEED, c1 + CS_TANGENTSPEED, c2 + CS_TANGENTSPEED, c3 + CS_TANGENTSPEED));
		b2StoreL<T>(
			cw + CW_ROLLINGRESISTANCE,
			b2SetLoadL<T>(c0 + CS_ROLLINGRESISTANCE, c1 + CS_ROLLINGRESISTANCE, c2 + CS_ROLLINGRESISTANCE, c3 + CS_ROLLINGRESISTANCE)
		);
		let rollingImpulse = b2SetLoadL<T>(c0 + CS_ROLLINGIMPULSEM, c1 + CS_ROLLINGIMPULSEM, c2 + CS_ROLLINGIMPULSEM, c3 + CS_ROLLINGIMPULSEM);
		b2StoreL<T>(cw + CW_ROLLINGIMPULSE, b2MulL(warmStartScale, rollingImpulse));

		b2StoreL<T>(cw + CW_RESTITUTION, b2SetLoadL<T>(c0 + CS_RESTITUTION, c1 + CS_RESTITUTION, c2 + CS_RESTITUTION, c3 + CS_RESTITUTION));

		// b2RightPerpW( normal )
		let tangentX = normalY;
		let tangentY = b2NegL(normalX);

		let pointCounts = b2SetL<T>(
			<f32>load<i32>(c0 + CS_POINTCOUNT),
			<f32>load<i32>(c1 + CS_POINTCOUNT),
			<f32>load<i32>(c2 + CS_POINTCOUNT),
			<f32>load<i32>(c3 + CS_POINTCOUNT)
		);
		let twoPointMask = b2GreaterThanL(pointCounts, oneW);

		{
			// Optimized transpose gather. Similar to b2GatherBodies.
			let a = b2GatherRowL<T>(c0 + CS_POINT0, c1 + CS_POINT0, c2 + CS_POINT0, c3 + CS_POINT0, MP_ANCHORA);
			let b = b2GatherRowL<T>(c0 + CS_POINT0, c1 + CS_POINT0, c2 + CS_POINT0, c3 + CS_POINT0, MP_SEPARATION);
			let anchorAX = a.c0;
			let anchorAY = a.c1;
			let anchorBX = a.c2;
			let anchorBY = a.c3;
			b2StoreL<T>(cw + CW_ANCHORA1X, anchorAX);
			b2StoreL<T>(cw + CW_ANCHORA1Y, anchorAY);
			b2StoreL<T>(cw + CW_ANCHORB1X, anchorBX);
			b2StoreL<T>(cw + CW_ANCHORB1Y, anchorBY);

			// separation1 = cw->baseSeparation1;

			let offset = b2DotL(b2SubL(anchorBX, anchorAX), b2SubL(anchorBY, anchorAY), normalX, normalY);
			b2StoreL<T>(cw + CW_BASESEPARATION1, b2SubL(b.c0, offset));
			b2StoreL<T>(cw + CW_NORMALIMPULSE1, b2MulL(warmStartScale, b.c1));
			b2StoreL<T>(cw + CW_TANGENTIMPULSE1, b2MulL(warmStartScale, b.c2));
			b2StoreL<T>(cw + CW_TOTALNORMALIMPULSE1, zeroW);
			b2StoreL<T>(cw + CW_RELATIVEVELOCITY1, zeroW);
			b2StoreL<T>(cw + CW_RESTITUTIONIMPULSE1, zeroW);

			// Effective mass along normal.
			{
				let rnA = b2CrossL(anchorAX, anchorAY, normalX, normalY);
				let rnB = b2CrossL(anchorBX, anchorBY, normalX, normalY);
				let k = b2AddL(invMassA, invMassB);
				k = b2MulAddL(k, invIA, b2MulL(rnA, rnA));
				k = b2MulAddL(k, invIB, b2MulL(rnB, rnB));
				let test = b2GreaterThanL(k, zeroW);
				b2StoreL<T>(cw + CW_NORMALMASS1, b2BlendL(zeroW, b2DivL(oneW, k), test));
			}

			// Effective mass along tangent.
			{
				let rtA = b2CrossL(anchorAX, anchorAY, tangentX, tangentY);
				let rtB = b2CrossL(anchorBX, anchorBY, tangentX, tangentY);
				let k = b2AddL(invMassA, invMassB);
				k = b2MulAddL(k, invIA, b2MulL(rtA, rtA));
				k = b2MulAddL(k, invIB, b2MulL(rtB, rtB));
				let test = b2GreaterThanL(k, zeroW);
				b2StoreL<T>(cw + CW_TANGENTMASS1, b2BlendL(zeroW, b2DivL(oneW, k), test));
			}
		}

		{
			let a = b2GatherRowL<T>(c0 + CS_POINT1, c1 + CS_POINT1, c2 + CS_POINT1, c3 + CS_POINT1, MP_ANCHORA);
			let b = b2GatherRowL<T>(c0 + CS_POINT1, c1 + CS_POINT1, c2 + CS_POINT1, c3 + CS_POINT1, MP_SEPARATION);
			let anchorAX = a.c0;
			let anchorAY = a.c1;
			let anchorBX = a.c2;
			let anchorBY = a.c3;
			b2StoreL<T>(cw + CW_ANCHORA2X, anchorAX);
			b2StoreL<T>(cw + CW_ANCHORA2Y, anchorAY);
			b2StoreL<T>(cw + CW_ANCHORB2X, anchorBX);
			b2StoreL<T>(cw + CW_ANCHORB2Y, anchorBY);

			// separation2 = cw->baseSeparation2;

			let offset = b2DotL(b2SubL(anchorBX, anchorAX), b2SubL(anchorBY, anchorAY), normalX, normalY);
			b2StoreL<T>(cw + CW_BASESEPARATION2, b2SubL(b.c0, offset));
			b2StoreL<T>(cw + CW_NORMALIMPULSE2, b2MulL(warmStartScale, b.c1));
			b2StoreL<T>(cw + CW_TANGENTIMPULSE2, b2MulL(warmStartScale, b.c2));
			b2StoreL<T>(cw + CW_TOTALNORMALIMPULSE2, zeroW);
			b2StoreL<T>(cw + CW_RELATIVEVELOCITY2, zeroW);
			b2StoreL<T>(cw + CW_RESTITUTIONIMPULSE2, zeroW);

			// Effective mass along normal.
			let normalMass: T;
			{
				let rnA = b2CrossL(anchorAX, anchorAY, normalX, normalY);
				let rnB = b2CrossL(anchorBX, anchorBY, normalX, normalY);
				let k = b2AddL(invMassA, invMassB);
				k = b2MulAddL(k, invIA, b2MulL(rnA, rnA));
				k = b2MulAddL(k, invIB, b2MulL(rnB, rnB));
				let test = b2GreaterThanL(k, zeroW);
				normalMass = b2BlendL(zeroW, b2DivL(oneW, k), test);
			}

			// Effective mass along tangent.
			let tangentMass: T;
			{
				let rtA = b2CrossL(anchorAX, anchorAY, tangentX, tangentY);
				let rtB = b2CrossL(anchorBX, anchorBY, tangentX, tangentY);
				let k = b2AddL(invMassA, invMassB);
				k = b2MulAddL(k, invIA, b2MulL(rtA, rtA));
				k = b2MulAddL(k, invIB, b2MulL(rtB, rtB));
				let test = b2GreaterThanL(k, zeroW);
				tangentMass = b2BlendL(zeroW, b2DivL(oneW, k), test);
			}

			// Zero effective mass on lanes that only have one point.
			b2StoreL<T>(cw + CW_NORMALMASS2, b2BlendL(zeroW, normalMass, twoPointMask));
			b2StoreL<T>(cw + CW_TANGENTMASS2, b2BlendL(zeroW, tangentMass, twoPointMask));
		}

		// Only sample contact point normal velocity if needed.
		if (hitEvents || haveRestitution) {
			let pA1 = b2LaneState(states, identity, cw0 + CW_INDEXA, l0);
			let pA2 = b2LaneState(states, identity, cw0 + CW_INDEXA, l1);
			let pA3 = b2LaneState(states, identity, cw0 + CW_INDEXA, l2);
			let pA4 = b2LaneState(states, identity, cw0 + CW_INDEXA, l3);
			let pB1 = b2LaneState(states, identity, cw0 + CW_INDEXB, l0);
			let pB2 = b2LaneState(states, identity, cw0 + CW_INDEXB, l1);
			let pB3 = b2LaneState(states, identity, cw0 + CW_INDEXB, l2);
			let pB4 = b2LaneState(states, identity, cw0 + CW_INDEXB, l3);
			let bA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW0);
			let bB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW0);

			{
				let rAX = b2LoadL<T>(cw + CW_ANCHORA1X);
				let rAY = b2LoadL<T>(cw + CW_ANCHORA1Y);
				let rBX = b2LoadL<T>(cw + CW_ANCHORB1X);
				let rBY = b2LoadL<T>(cw + CW_ANCHORB1Y);

				// Relative velocity at contact
				let dvx = b2SubL(b2SubL(bB.c0, b2MulL(bB.c2, rBY)), b2SubL(bA.c0, b2MulL(bA.c2, rAY)));
				let dvy = b2SubL(b2AddL(bB.c1, b2MulL(bB.c2, rBX)), b2AddL(bA.c1, b2MulL(bA.c2, rAX)));
				let vn = b2AddL(b2MulL(dvx, normalX), b2MulL(dvy, normalY));
				b2StoreL<T>(cw + CW_RELATIVEVELOCITY1, vn);

				// Check the point count to avoid a race writing to b2_zeroContactSim.
				b2StoreNormalVelocityL<T>(c0, c1, c2, c3, CS_POINT0, 0, vn);
			}

			{
				let rAX = b2LoadL<T>(cw + CW_ANCHORA2X);
				let rAY = b2LoadL<T>(cw + CW_ANCHORA2Y);
				let rBX = b2LoadL<T>(cw + CW_ANCHORB2X);
				let rBY = b2LoadL<T>(cw + CW_ANCHORB2Y);

				// Relative velocity at contact
				let dvx = b2SubL(b2SubL(bB.c0, b2MulL(bB.c2, rBY)), b2SubL(bA.c0, b2MulL(bA.c2, rAY)));
				let dvy = b2SubL(b2AddL(bB.c1, b2MulL(bB.c2, rBX)), b2AddL(bA.c1, b2MulL(bA.c2, rAX)));
				let vn = b2AddL(b2MulL(dvx, normalX), b2MulL(dvy, normalY));
				b2StoreL<T>(cw + CW_RELATIVEVELOCITY2, vn);

				b2StoreNormalVelocityL<T>(c0, c1, c2, c3, CS_POINT1, 1, vn);
			}
		}
	}
}

// Each lane's normal velocity into its contact's manifold point when the manifold has more than minPoints points
@inline function b2StoreNormalVelocityL<T>(c0: usize, c1: usize, c2: usize, c3: usize, point: usize, minPoints: i32, vn: T): void {
	if (load<i32>(c0 + CS_POINTCOUNT) > minPoints) {
		store<f32>(c0 + point + MP_NORMALVELOCITY, b2LaneL(vn, 0));
	}

	if (!isFloat<T>()) {
		if (load<i32>(c1 + CS_POINTCOUNT) > minPoints) {
			store<f32>(c1 + point + MP_NORMALVELOCITY, b2LaneL(vn, 1));
		}

		if (load<i32>(c2 + CS_POINTCOUNT) > minPoints) {
			store<f32>(c2 + point + MP_NORMALVELOCITY, b2LaneL(vn, 2));
		}

		if (load<i32>(c3 + CS_POINTCOUNT) > minPoints) {
			store<f32>(c3 + point + MP_NORMALVELOCITY, b2LaneL(vn, 3));
		}
	}
}

// Prepare wide contact constraints.
// Potential optimization: for SSE2 load 4 manifold points and transpose, like the body gather/scatter.
// I would need to reorder the manifold point so that constraint data comes first.
@noinline
function b2PrepareContactsL<T>(block: b2SolverBlock, context: b2StepContext): void {
	let world = context.world;
	let states = b2ArrayData(context.states);
	let identity = changetype<usize>(b2_identityStates);
	let spans = context.contactPrepareSpans;
	let wideBase = changetype<usize>(context.wideContactConstraints);
	let zeroContact = changetype<usize>(b2_zeroContactSims);

	// The transpose gathers read anchorA, anchorB as one row and separation, normalImpulse, tangentImpulse as the next
	if (B2_ASSERT_ENABLED) {
		B2_ASSERT(MP_ANCHORA == 0 && MP_ANCHORB == 8 && MP_SEPARATION == 16);
		B2_ASSERT(MP_NORMALIMPULSE == 20 && MP_TANGENTIMPULSE == 24 && sizeof<b2ManifoldPoint>() >= 32);
	}

	let warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0;
	let anyRestitution = false;

	let wideIndex = block.startIndex;
	let endWideIndex = block.startIndex + <i32>block.count;

	// Find color for start index. Linear search but fast.
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= wideIndex) {
		colorIndex += 1;
	}

	// Loop over block
	while (wideIndex < endWideIndex) {
		let colorWideStart = spans[colorIndex].start;
		let colorWideEndIndex = b2MinInt(spans[colorIndex + 1].start, endWideIndex);
		let colorContactCount = spans[colorIndex].count;
		let contactSims = b2ArrayData(spans[colorIndex].contacts);

		// Loop over color
		for (; wideIndex < colorWideEndIndex; ++wideIndex) {
			// wideIndex is below the step's wide constraint count
			let cw = wideBase + <usize>wideIndex * sizeof<b2ContactConstraintWide>();
			let localWideIndex = wideIndex - colorWideStart;

			let hitEvents: u32 = 0;
			let restitutionLanes = false;

			// C's contactLanes; contactIndex below colorContactCount is a live sim of the color
			let cl0 = zeroContact;
			let cl1 = zeroContact;
			let cl2 = zeroContact;
			let cl3 = zeroContact;
			for (let laneIndex = 0; laneIndex < 4; ++laneIndex) {
				let contactIndex = 4 * localWideIndex + laneIndex;
				if (contactIndex < colorContactCount) {
					let c = contactSims + <usize>contactIndex * sizeof<b2ContactSim>();
					if (laneIndex == 0) {
						cl0 = c;
					} else if (laneIndex == 1) {
						cl1 = c;
					} else if (laneIndex == 2) {
						cl2 = c;
					} else {
						cl3 = c;
					}

					// index base-1
					store<i32>(cw + CW_INDEXA + <usize>laneIndex * 4, b2DecodeAwakeIndex(load<i32>(c + CS_ENCODEDBODYSIMA)) + 1);
					store<i32>(cw + CW_INDEXB + <usize>laneIndex * 4, b2DecodeAwakeIndex(load<i32>(c + CS_ENCODEDBODYSIMB)) + 1);
					hitEvents |= load<u32>(c + CS_SIMFLAGS) & b2_simEnableHitEvent;

					// b2AnyTrueW( b2GreaterThanW( cw->restitution, zeroW ) ), which the lanes share; an empty lane's is 0
					restitutionLanes = restitutionLanes || load<f32>(c + CS_RESTITUTION) > 0.0;
				}
			}

			let haveRestitution = restitutionLanes;
			anyRestitution = anyRestitution || haveRestitution;

			b2PrepareConstraintL<T>(cw, cl0, cl1, cl2, cl3, (hitEvents & b2_simEnableHitEvent) != 0, haveRestitution, states, identity, warmStartScale);
		}

		// Advance to next color
		colorIndex += 1;
	}

	if (anyRestitution) {
		context.anyRestitution.value = 1;
	}
}

@noinline
function b2WarmStartContactsL<T>(block: b2SolverBlock, context: b2StepContext): void {
	let states = b2ArrayData(context.states);
	let identity = changetype<usize>(b2_identityStates);
	let constraints = b2ColorConstraints(context, block.colorIndex);

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		// i is below the color's wide constraint count
		let c0 = constraints + <usize>i * sizeof<b2ContactConstraintWide>();
		for (let lane = 0; lane < b2LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;

			let pA1 = b2LaneState(states, identity, c + CW_INDEXA, 0);
			let pA2 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 1);
			let pA3 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 2);
			let pA4 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 3);
			let pB1 = b2LaneState(states, identity, c + CW_INDEXB, 0);
			let pB2 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 1);
			let pB3 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 2);
			let pB4 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 3);
			let bA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW0);
			let bB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW0);
			let vAX = bA.c0;
			let vAY = bA.c1;
			let wA = bA.c2;
			let vBX = bB.c0;
			let vBY = bB.c1;
			let wB = bB.c2;

			let normalX = b2LoadL<T>(c + CW_NORMALX);
			let normalY = b2LoadL<T>(c + CW_NORMALY);
			let tangentX = normalY;
			let tangentY = b2SubL(b2SplatL<T>(0.0), normalX);

			let invMassA = b2LoadL<T>(c + CW_INVMASSA);
			let invMassB = b2LoadL<T>(c + CW_INVMASSB);
			let invIA = b2LoadL<T>(c + CW_INVIA);
			let invIB = b2LoadL<T>(c + CW_INVIB);

			{
				// fixed anchors
				let rAX = b2LoadL<T>(c + CW_ANCHORA1X);
				let rAY = b2LoadL<T>(c + CW_ANCHORA1Y);
				let rBX = b2LoadL<T>(c + CW_ANCHORB1X);
				let rBY = b2LoadL<T>(c + CW_ANCHORB1Y);

				let normalImpulse = b2LoadL<T>(c + CW_NORMALIMPULSE1);
				let tangentImpulse = b2LoadL<T>(c + CW_TANGENTIMPULSE1);
				let PX = b2AddL(b2MulL(normalImpulse, normalX), b2MulL(tangentImpulse, tangentX));
				let PY = b2AddL(b2MulL(normalImpulse, normalY), b2MulL(tangentImpulse, tangentY));
				wA = b2MulSubL(wA, invIA, b2CrossL(rAX, rAY, PX, PY));
				vAX = b2MulSubL(vAX, invMassA, PX);
				vAY = b2MulSubL(vAY, invMassA, PY);
				wB = b2MulAddL(wB, invIB, b2CrossL(rBX, rBY, PX, PY));
				vBX = b2MulAddL(vBX, invMassB, PX);
				vBY = b2MulAddL(vBY, invMassB, PY);

				b2StoreL<T>(c + CW_TOTALNORMALIMPULSE1, b2AddL(b2LoadL<T>(c + CW_TOTALNORMALIMPULSE1), normalImpulse));
			}

			{
				// fixed anchors
				let rAX = b2LoadL<T>(c + CW_ANCHORA2X);
				let rAY = b2LoadL<T>(c + CW_ANCHORA2Y);
				let rBX = b2LoadL<T>(c + CW_ANCHORB2X);
				let rBY = b2LoadL<T>(c + CW_ANCHORB2Y);

				let normalImpulse = b2LoadL<T>(c + CW_NORMALIMPULSE2);
				let tangentImpulse = b2LoadL<T>(c + CW_TANGENTIMPULSE2);
				let PX = b2AddL(b2MulL(normalImpulse, normalX), b2MulL(tangentImpulse, tangentX));
				let PY = b2AddL(b2MulL(normalImpulse, normalY), b2MulL(tangentImpulse, tangentY));
				wA = b2MulSubL(wA, invIA, b2CrossL(rAX, rAY, PX, PY));
				vAX = b2MulSubL(vAX, invMassA, PX);
				vAY = b2MulSubL(vAY, invMassA, PY);
				wB = b2MulAddL(wB, invIB, b2CrossL(rBX, rBY, PX, PY));
				vBX = b2MulAddL(vBX, invMassB, PX);
				vBY = b2MulAddL(vBY, invMassB, PY);

				b2StoreL<T>(c + CW_TOTALNORMALIMPULSE2, b2AddL(b2LoadL<T>(c + CW_TOTALNORMALIMPULSE2), normalImpulse));
			}

			let rollingImpulse = b2LoadL<T>(c + CW_ROLLINGIMPULSE);
			wA = b2MulSubL(wA, invIA, rollingImpulse);
			wB = b2MulAddL(wB, invIB, rollingImpulse);

			b2ScatterBodiesL<T>(states, c + CW_INDEXA, vAX, vAY, wA, bA.c3);
			b2ScatterBodiesL<T>(states, c + CW_INDEXB, vBX, vBY, wB, bB.c3);
		}
	}
}

// The separation of a point from its fixed anchors and the bodies' delta rotations, for push, solve and restitution
@inline function b2SeparationL<T>(
	c: usize,
	anchorAX: usize,
	anchorBX: usize,
	baseSeparation: usize,
	dpX: T,
	dpY: T,
	dqAC: T,
	dqAS: T,
	dqBC: T,
	dqBS: T,
	normalX: T,
	normalY: T
): T {
	let rAX = b2LoadL<T>(c + anchorAX);
	let rAY = b2LoadL<T>(c + anchorAX + W_Y);
	let rBX = b2LoadL<T>(c + anchorBX);
	let rBY = b2LoadL<T>(c + anchorBX + W_Y);

	// Moving anchors for current separation, b2RotateVectorW
	let rsAX = b2SubL(b2MulL(dqAC, rAX), b2MulL(dqAS, rAY));
	let rsAY = b2AddL(b2MulL(dqAS, rAX), b2MulL(dqAC, rAY));
	let rsBX = b2SubL(b2MulL(dqBC, rBX), b2MulL(dqBS, rBY));
	let rsBY = b2AddL(b2MulL(dqBS, rBX), b2MulL(dqBC, rBY));

	// compute current separation
	// this is subject to round-off error if the anchor is far from the body center of mass
	let dsX = b2AddL(dpX, b2SubL(rsBX, rsAX));
	let dsY = b2AddL(dpY, b2SubL(rsBY, rsAY));
	return b2AddL(b2DotL(normalX, normalY, dsX, dsY), b2LoadL<T>(c + baseSeparation));
}

@noinline
function b2PushContactsL<T>(block: b2SolverBlock, context: b2StepContext): void {
	let states = b2ArrayData(context.states);
	let identity = changetype<usize>(b2_identityStates);
	let constraints = b2ColorConstraints(context, block.colorIndex);
	let invHValue = context.inv_h;
	let contactSpeedValue = -context.world.contactSpeed;

	let dynamicBiasRateValue = context.contactSoftness.massScale * context.contactSoftness.biasRate;
	let dynamicMassScaleValue = context.contactSoftness.massScale;
	let dynamicImpulseScaleValue = context.contactSoftness.impulseScale;
	let staticBiasRateValue = context.staticSoftness.massScale * context.staticSoftness.biasRate;
	let staticMassScaleValue = context.staticSoftness.massScale;
	let staticImpulseScaleValue = context.staticSoftness.impulseScale;

	let endIndex = block.startIndex + <i32>block.count;
	for (let wideIndex = block.startIndex; wideIndex < endIndex; ++wideIndex) {
		// wideIndex is below the color's wide constraint count
		let c0 = constraints + <usize>wideIndex * sizeof<b2ContactConstraintWide>();
		for (let lane = 0; lane < b2LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;
			let zeroW = b2SplatL<T>(0.0);
			let oneW = b2SplatL<T>(1.0);

			let pA1 = b2LaneState(states, identity, c + CW_INDEXA, 0);
			let pA2 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 1);
			let pA3 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 2);
			let pA4 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 3);
			let pB1 = b2LaneState(states, identity, c + CW_INDEXB, 0);
			let pB2 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 1);
			let pB3 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 2);
			let pB4 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 3);
			let bA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW0);
			let bB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW0);

			let softMask = b2SoftMaskL<T>(c + CW_INDEXA, c + CW_INDEXB);
			let biasRate = b2BlendL(b2SplatL<T>(dynamicBiasRateValue), b2SplatL<T>(staticBiasRateValue), softMask);
			let massScale = b2BlendL(b2SplatL<T>(dynamicMassScaleValue), b2SplatL<T>(staticMassScaleValue), softMask);
			let impulseScale = b2BlendL(b2SplatL<T>(dynamicImpulseScaleValue), b2SplatL<T>(staticImpulseScaleValue), softMask);

			let normalX = b2LoadL<T>(c + CW_NORMALX);
			let normalY = b2LoadL<T>(c + CW_NORMALY);

			let velocityBias1: T;
			let massScale1: T;
			let impulseScale1: T;
			let velocityBias2: T;
			let massScale2: T;
			let impulseScale2: T;
			{
				let dA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW1);
				let dB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW1);
				let dpX = b2SubL(dB.c0, dA.c0);
				let dpY = b2SubL(dB.c1, dA.c1);

				{
					let s = b2SeparationL<T>(c, CW_ANCHORA1X, CW_ANCHORB1X, CW_BASESEPARATION1, dpX, dpY, dA.c2, dA.c3, dB.c2, dB.c3, normalX, normalY);

					// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
					// The contactSpeed is meant to limit stiffness, not increase it.
					let separated = b2GreaterThanL(s, zeroW);

					// Speculative bias - positive
					let specBias = b2MulL(s, b2SplatL<T>(invHValue));

					// Overlap bias - negative when overlapped
					let overlapBias = b2MaxL(b2MulL(biasRate, s), b2SplatL<T>(contactSpeedValue));

					// velocity bias = s > 0 ? spec : overlap
					velocityBias1 = b2BlendL(overlapBias, specBias, separated);

					massScale1 = b2BlendL(massScale, oneW, separated);
					impulseScale1 = b2BlendL(impulseScale, zeroW, separated);
				}

				{
					let s = b2SeparationL<T>(c, CW_ANCHORA2X, CW_ANCHORB2X, CW_BASESEPARATION2, dpX, dpY, dA.c2, dA.c3, dB.c2, dB.c3, normalX, normalY);

					let separated = b2GreaterThanL(s, zeroW);
					let specBias = b2MulL(s, b2SplatL<T>(invHValue));
					let overlapBias = b2MaxL(b2MulL(biasRate, s), b2SplatL<T>(contactSpeedValue));
					velocityBias2 = b2BlendL(overlapBias, specBias, separated);

					massScale2 = b2BlendL(massScale, oneW, separated);
					impulseScale2 = b2BlendL(impulseScale, zeroW, separated);
				}
			}

			let rnA1 = b2CrossL(b2LoadL<T>(c + CW_ANCHORA1X), b2LoadL<T>(c + CW_ANCHORA1Y), normalX, normalY);
			let rnB1 = b2CrossL(b2LoadL<T>(c + CW_ANCHORB1X), b2LoadL<T>(c + CW_ANCHORB1Y), normalX, normalY);
			let rnA2 = b2CrossL(b2LoadL<T>(c + CW_ANCHORA2X), b2LoadL<T>(c + CW_ANCHORA2Y), normalX, normalY);
			let rnB2 = b2CrossL(b2LoadL<T>(c + CW_ANCHORB2X), b2LoadL<T>(c + CW_ANCHORB2Y), normalX, normalY);

			let invMassA = b2LoadL<T>(c + CW_INVMASSA);
			let invMassB = b2LoadL<T>(c + CW_INVMASSB);
			let invIA = b2LoadL<T>(c + CW_INVIA);
			let invIB = b2LoadL<T>(c + CW_INVIB);

			let k21 = b2AddL(b2AddL(invMassA, invMassB), b2AddL(b2MulL(b2MulL(invIA, rnA1), rnA2), b2MulL(b2MulL(invIB, rnB1), rnB2)));

			let dn = b2DotL(b2SubL(bB.c0, bA.c0), b2SubL(bB.c1, bA.c1), normalX, normalY);
			let vn1 = b2AddL(dn, b2SubL(b2MulL(bB.c2, rnB1), b2MulL(bA.c2, rnA1)));
			let vn2 = b2AddL(dn, b2SubL(b2MulL(bB.c2, rnB2), b2MulL(bA.c2, rnA2)));

			let normalImpulse1 = b2LoadL<T>(c + CW_NORMALIMPULSE1);
			let negImpulse1 = b2AddL(
				b2MulL(b2LoadL<T>(c + CW_NORMALMASS1), b2AddL(b2MulL(massScale1, vn1), velocityBias1)),
				b2MulL(impulseScale1, normalImpulse1)
			);
			let impulse1 = b2MaxL(b2NegL(negImpulse1), b2NegL(normalImpulse1));
			b2StoreL<T>(c + CW_NORMALIMPULSE1, b2AddL(normalImpulse1, impulse1));
			b2StoreL<T>(c + CW_TOTALNORMALIMPULSE1, b2AddL(b2LoadL<T>(c + CW_TOTALNORMALIMPULSE1), impulse1));

			vn2 = b2MulAddL(vn2, k21, impulse1);

			let normalImpulse2 = b2LoadL<T>(c + CW_NORMALIMPULSE2);
			let negImpulse2 = b2AddL(
				b2MulL(b2LoadL<T>(c + CW_NORMALMASS2), b2AddL(b2MulL(massScale2, vn2), velocityBias2)),
				b2MulL(impulseScale2, normalImpulse2)
			);
			let impulse2 = b2MaxL(b2NegL(negImpulse2), b2NegL(normalImpulse2));
			b2StoreL<T>(c + CW_NORMALIMPULSE2, b2AddL(normalImpulse2, impulse2));
			b2StoreL<T>(c + CW_TOTALNORMALIMPULSE2, b2AddL(b2LoadL<T>(c + CW_TOTALNORMALIMPULSE2), impulse2));

			let impulse = b2AddL(impulse1, impulse2);
			let Px = b2MulL(impulse, normalX);
			let Py = b2MulL(impulse, normalY);
			let angularA = b2AddL(b2MulL(impulse1, rnA1), b2MulL(impulse2, rnA2));
			let angularB = b2AddL(b2MulL(impulse1, rnB1), b2MulL(impulse2, rnB2));

			let vAX = b2MulSubL(bA.c0, invMassA, Px);
			let vAY = b2MulSubL(bA.c1, invMassA, Py);
			let wA = b2MulSubL(bA.c2, invIA, angularA);

			let vBX = b2MulAddL(bB.c0, invMassB, Px);
			let vBY = b2MulAddL(bB.c1, invMassB, Py);
			let wB = b2MulAddL(bB.c2, invIB, angularB);

			b2ScatterBodiesL<T>(states, c + CW_INDEXA, vAX, vAY, wA, bA.c3);
			b2ScatterBodiesL<T>(states, c + CW_INDEXB, vBX, vBY, wB, bB.c3);
		}
	}
}

// Solve the normal constraint, friction, and rolling resistance.
@noinline
function b2SolveContactsL<T>(block: b2SolverBlock, context: b2StepContext): void {
	let states = b2ArrayData(context.states);
	let identity = changetype<usize>(b2_identityStates);
	let constraints = b2ColorConstraints(context, block.colorIndex);
	let invHValue = context.inv_h;

	let endIndex = block.startIndex + <i32>block.count;
	for (let wideIndex = block.startIndex; wideIndex < endIndex; ++wideIndex) {
		// wideIndex is below the color's wide constraint count
		let c0 = constraints + <usize>wideIndex * sizeof<b2ContactConstraintWide>();

		// b2AllZeroW( c->rollingResistance ), which the lanes share
		let anyRolling = b2AllZeroAt(c0 + CW_ROLLINGRESISTANCE) == false;

		for (let lane = 0; lane < b2LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;
			let zeroW = b2SplatL<T>(0.0);

			let pA1 = b2LaneState(states, identity, c + CW_INDEXA, 0);
			let pA2 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 1);
			let pA3 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 2);
			let pA4 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 3);
			let pB1 = b2LaneState(states, identity, c + CW_INDEXB, 0);
			let pB2 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 1);
			let pB3 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 2);
			let pB4 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 3);
			let bA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW0);
			let bB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW0);

			let normalX = b2LoadL<T>(c + CW_NORMALX);
			let normalY = b2LoadL<T>(c + CW_NORMALY);

			// b2RightPerpW( normal )
			let tangentX = normalY;
			let tangentY = b2NegL(normalX);

			let velocityBias1: T;
			let velocityBias2: T;
			{
				let dA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW1);
				let dB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW1);
				let dpX = b2SubL(dB.c0, dA.c0);
				let dpY = b2SubL(dB.c1, dA.c1);

				{
					let s = b2SeparationL<T>(c, CW_ANCHORA1X, CW_ANCHORB1X, CW_BASESEPARATION1, dpX, dpY, dA.c2, dA.c3, dB.c2, dB.c3, normalX, normalY);

					// Speculative bias, positive if separated and zero if overlapped
					velocityBias1 = b2MaxL(zeroW, b2MulL(s, b2SplatL<T>(invHValue)));
				}

				{
					let s = b2SeparationL<T>(c, CW_ANCHORA2X, CW_ANCHORB2X, CW_BASESEPARATION2, dpX, dpY, dA.c2, dA.c3, dB.c2, dB.c3, normalX, normalY);

					velocityBias2 = b2MaxL(zeroW, b2MulL(s, b2SplatL<T>(invHValue)));
				}
			}

			let anchorA1X = b2LoadL<T>(c + CW_ANCHORA1X);
			let anchorA1Y = b2LoadL<T>(c + CW_ANCHORA1Y);
			let anchorB1X = b2LoadL<T>(c + CW_ANCHORB1X);
			let anchorB1Y = b2LoadL<T>(c + CW_ANCHORB1Y);
			let anchorA2X = b2LoadL<T>(c + CW_ANCHORA2X);
			let anchorA2Y = b2LoadL<T>(c + CW_ANCHORA2Y);
			let anchorB2X = b2LoadL<T>(c + CW_ANCHORB2X);
			let anchorB2Y = b2LoadL<T>(c + CW_ANCHORB2Y);

			let rnA1 = b2CrossL(anchorA1X, anchorA1Y, normalX, normalY);
			let rnB1 = b2CrossL(anchorB1X, anchorB1Y, normalX, normalY);
			let rnA2 = b2CrossL(anchorA2X, anchorA2Y, normalX, normalY);
			let rnB2 = b2CrossL(anchorB2X, anchorB2Y, normalX, normalY);
			let rtA1 = b2CrossL(anchorA1X, anchorA1Y, tangentX, tangentY);
			let rtB1 = b2CrossL(anchorB1X, anchorB1Y, tangentX, tangentY);
			let rtA2 = b2CrossL(anchorA2X, anchorA2Y, tangentX, tangentY);
			let rtB2 = b2CrossL(anchorB2X, anchorB2Y, tangentX, tangentY);

			let invIA = b2LoadL<T>(c + CW_INVIA);
			let invIB = b2LoadL<T>(c + CW_INVIB);
			let wnA1 = b2MulL(invIA, rnA1);
			let wnB1 = b2MulL(invIB, rnB1);
			let wnA2 = b2MulL(invIA, rnA2);
			let wnB2 = b2MulL(invIB, rnB2);
			let wtA1 = b2MulL(invIA, rtA1);
			let wtB1 = b2MulL(invIB, rtB1);

			let invMassA = b2LoadL<T>(c + CW_INVMASSA);
			let invMassB = b2LoadL<T>(c + CW_INVMASSB);
			let invMass = b2AddL(invMassA, invMassB);
			let k21 = b2AddL(invMass, b2AddL(b2MulL(wnA1, rnA2), b2MulL(wnB1, rnB2)));
			let k31 = b2AddL(b2MulL(wnA1, rtA1), b2MulL(wnB1, rtB1));
			let k32 = b2AddL(b2MulL(wnA2, rtA1), b2MulL(wnB2, rtB1));
			let k41 = b2AddL(b2MulL(wnA1, rtA2), b2MulL(wnB1, rtB2));
			let k42 = b2AddL(b2MulL(wnA2, rtA2), b2MulL(wnB2, rtB2));
			let k43 = b2AddL(invMass, b2AddL(b2MulL(wtA1, rtA2), b2MulL(wtB1, rtB2)));

			let dvX = b2SubL(bB.c0, bA.c0);
			let dvY = b2SubL(bB.c1, bA.c1);
			let dn = b2DotL(dvX, dvY, normalX, normalY);
			let dt = b2SubL(b2DotL(dvX, dvY, tangentX, tangentY), b2LoadL<T>(c + CW_TANGENTSPEED));

			let wA = bA.c2;
			let wB = bB.c2;
			let vn1 = b2AddL(dn, b2SubL(b2MulL(wB, rnB1), b2MulL(wA, rnA1)));
			let vn2 = b2AddL(dn, b2SubL(b2MulL(wB, rnB2), b2MulL(wA, rnA2)));
			let vt1 = b2AddL(dt, b2SubL(b2MulL(wB, rtB1), b2MulL(wA, rtA1)));
			let vt2 = b2AddL(dt, b2SubL(b2MulL(wB, rtB2), b2MulL(wA, rtA2)));

			let accumulatedImpulse1 = b2LoadL<T>(c + CW_NORMALIMPULSE1);
			let normalImpulse1 = b2MaxL(b2NegL(b2MulL(b2LoadL<T>(c + CW_NORMALMASS1), b2AddL(vn1, velocityBias1))), b2NegL(accumulatedImpulse1));
			accumulatedImpulse1 = b2AddL(accumulatedImpulse1, normalImpulse1);
			b2StoreL<T>(c + CW_NORMALIMPULSE1, accumulatedImpulse1);
			b2StoreL<T>(c + CW_TOTALNORMALIMPULSE1, b2AddL(b2LoadL<T>(c + CW_TOTALNORMALIMPULSE1), normalImpulse1));

			vn2 = b2MulAddL(vn2, k21, normalImpulse1);

			let accumulatedImpulse2 = b2LoadL<T>(c + CW_NORMALIMPULSE2);
			let normalImpulse2 = b2MaxL(b2NegL(b2MulL(b2LoadL<T>(c + CW_NORMALMASS2), b2AddL(vn2, velocityBias2))), b2NegL(accumulatedImpulse2));
			accumulatedImpulse2 = b2AddL(accumulatedImpulse2, normalImpulse2);
			b2StoreL<T>(c + CW_NORMALIMPULSE2, accumulatedImpulse2);
			b2StoreL<T>(c + CW_TOTALNORMALIMPULSE2, b2AddL(b2LoadL<T>(c + CW_TOTALNORMALIMPULSE2), normalImpulse2));

			vt1 = b2MulAddL(b2MulAddL(vt1, k31, normalImpulse1), k32, normalImpulse2);
			vt2 = b2MulAddL(b2MulAddL(vt2, k41, normalImpulse1), k42, normalImpulse2);

			let angularA = b2AddL(b2MulL(normalImpulse1, rnA1), b2MulL(normalImpulse2, rnA2));
			let angularB = b2AddL(b2MulL(normalImpulse1, rnB1), b2MulL(normalImpulse2, rnB2));

			// Rolling resistance
			if (anyRolling) {
				let relativeW = b2SubL(
					b2SubL(wA, wB),
					b2AddL(b2MulL(b2AddL(wnA1, wnB1), normalImpulse1), b2MulL(b2AddL(wnA2, wnB2), normalImpulse2))
				);
				let k = b2AddL(invIA, invIB);
				let mask = b2GreaterThanL(k, zeroW);
				let deltaLambda = b2BlendL(zeroW, b2DivL(relativeW, k), mask);
				let lambda = b2LoadL<T>(c + CW_ROLLINGIMPULSE);
				let maxLambda = b2MulL(b2LoadL<T>(c + CW_ROLLINGRESISTANCE), b2AddL(accumulatedImpulse1, accumulatedImpulse2));
				let rollingImpulse = b2SymClampL(b2AddL(lambda, deltaLambda), maxLambda);
				b2StoreL<T>(c + CW_ROLLINGIMPULSE, rollingImpulse);
				deltaLambda = b2SubL(rollingImpulse, lambda);

				let wtA2 = b2MulL(invIA, rtA2);
				let wtB2 = b2MulL(invIB, rtB2);
				vt1 = b2MulAddL(vt1, b2AddL(wtA1, wtB1), deltaLambda);
				vt2 = b2MulAddL(vt2, b2AddL(wtA2, wtB2), deltaLambda);

				angularA = b2AddL(angularA, deltaLambda);
				angularB = b2AddL(angularB, deltaLambda);
			}

			let friction = b2LoadL<T>(c + CW_FRICTION);

			let tangentImpulseTotal1 = b2LoadL<T>(c + CW_TANGENTIMPULSE1);
			let maxFriction1 = b2MulL(friction, accumulatedImpulse1);
			let tangentImpulse1 = b2NegL(b2MulL(b2LoadL<T>(c + CW_TANGENTMASS1), vt1));
			tangentImpulse1 = b2MaxL(
				b2SubL(b2NegL(maxFriction1), tangentImpulseTotal1),
				b2MinL(tangentImpulse1, b2SubL(maxFriction1, tangentImpulseTotal1))
			);
			b2StoreL<T>(c + CW_TANGENTIMPULSE1, b2AddL(tangentImpulseTotal1, tangentImpulse1));

			vt2 = b2MulAddL(vt2, k43, tangentImpulse1);

			let tangentImpulseTotal2 = b2LoadL<T>(c + CW_TANGENTIMPULSE2);
			let maxFriction2 = b2MulL(friction, accumulatedImpulse2);
			let tangentImpulse2 = b2NegL(b2MulL(b2LoadL<T>(c + CW_TANGENTMASS2), vt2));
			tangentImpulse2 = b2MaxL(
				b2SubL(b2NegL(maxFriction2), tangentImpulseTotal2),
				b2MinL(tangentImpulse2, b2SubL(maxFriction2, tangentImpulseTotal2))
			);
			b2StoreL<T>(c + CW_TANGENTIMPULSE2, b2AddL(tangentImpulseTotal2, tangentImpulse2));

			angularA = b2AddL(angularA, b2AddL(b2MulL(tangentImpulse1, rtA1), b2MulL(tangentImpulse2, rtA2)));
			angularB = b2AddL(angularB, b2AddL(b2MulL(tangentImpulse1, rtB1), b2MulL(tangentImpulse2, rtB2)));

			let normalImpulse = b2AddL(normalImpulse1, normalImpulse2);
			let tangentImpulse = b2AddL(tangentImpulse1, tangentImpulse2);
			let Px = b2AddL(b2MulL(normalImpulse, normalX), b2MulL(tangentImpulse, tangentX));
			let Py = b2AddL(b2MulL(normalImpulse, normalY), b2MulL(tangentImpulse, tangentY));

			let vAX = b2MulSubL(bA.c0, invMassA, Px);
			let vAY = b2MulSubL(bA.c1, invMassA, Py);
			wA = b2MulSubL(wA, invIA, angularA);

			let vBX = b2MulAddL(bB.c0, invMassB, Px);
			let vBY = b2MulAddL(bB.c1, invMassB, Py);
			wB = b2MulAddL(wB, invIB, angularB);

			b2ScatterBodiesL<T>(states, c + CW_INDEXA, vAX, vAY, wA, bA.c3);
			b2ScatterBodiesL<T>(states, c + CW_INDEXB, vBX, vBY, wB, bB.c3);
		}
	}
}

// One restitution point: updates the point's impulses in the constraint and returns the impulse, for the caller to
// apply. Per component so it inlines.
@inline function b2RestitutionImpulseL<T>(
	c: usize,
	normalImpulseField: usize,
	totalNormalImpulseField: usize,
	restitutionImpulseField: usize,
	relativeVelocityField: usize,
	normalMass: T,
	restitution: T,
	restitutionMask: T,
	negRestitutionThreshold: T,
	s: T,
	vn: T,
	invH: T
): T {
	let zeroW = b2SplatL<T>(0.0);
	let restitutionImpulse = b2LoadL<T>(c + restitutionImpulseField);
	let relativeVelocity = b2LoadL<T>(c + relativeVelocityField);
	let totalNormalImpulse = b2LoadL<T>(c + totalNormalImpulseField);
	let normalImpulse = b2LoadL<T>(c + normalImpulseField);

	let compressionImpulse = b2SubL(totalNormalImpulse, restitutionImpulse);
	let armed = b2AndL(b2AndL(restitutionMask, b2LessThanL(relativeVelocity, negRestitutionThreshold)), b2GreaterThanL(compressionImpulse, zeroW));

	let specBias = b2MaxL(zeroW, b2MulL(s, invH));
	let velocityBias = b2BlendL(specBias, b2MulL(restitution, relativeVelocity), armed);

	let negImpulse = b2MulL(normalMass, b2AddL(vn, velocityBias));

	let newImpulse = b2MaxL(b2SubL(normalImpulse, negImpulse), zeroW);
	let impulse = b2SubL(newImpulse, normalImpulse);

	let approachImpulse = b2MinL(b2MaxL(b2NegL(b2MulL(normalMass, vn)), zeroW), b2MaxL(impulse, zeroW));
	let allowance = b2SubL(b2MulL(restitution, b2AddL(compressionImpulse, approachImpulse)), restitutionImpulse);
	let maxImpulse = b2AddL(approachImpulse, b2MaxL(allowance, zeroW));
	impulse = b2BlendL(impulse, b2MinL(impulse, maxImpulse), armed);

	b2StoreL<T>(c + normalImpulseField, b2AddL(normalImpulse, impulse));
	b2StoreL<T>(c + restitutionImpulseField, b2AddL(restitutionImpulse, b2SubL(impulse, approachImpulse)));
	b2StoreL<T>(c + totalNormalImpulseField, b2AddL(totalNormalImpulse, impulse));
	return impulse;
}

@noinline
function b2ApplyRestitutionL<T>(block: b2SolverBlock, context: b2StepContext): void {
	let states = b2ArrayData(context.states);
	let identity = changetype<usize>(b2_identityStates);
	let constraints = b2ColorConstraints(context, block.colorIndex);
	let invHValue = context.inv_h;
	let negRestitutionThresholdValue = -context.world.restitutionThreshold;
	let propagate = context.world.enableRestitutionPropagation;

	let endIndex = block.startIndex + <i32>block.count;
	for (let wideIndex = block.startIndex; wideIndex < endIndex; ++wideIndex) {
		// wideIndex is below the color's wide constraint count
		let c0 = constraints + <usize>wideIndex * sizeof<b2ContactConstraintWide>();

		// b2AllZeroW( c->restitution ), which the lanes share
		if (propagate == false && b2AllZeroAt(c0 + CW_RESTITUTION)) {
			continue;
		}

		for (let lane = 0; lane < b2LaneCountL<T>(); ++lane) {
			let c = c0 + <usize>lane * 4;
			let zeroW = b2SplatL<T>(0.0);

			let pA1 = b2LaneState(states, identity, c + CW_INDEXA, 0);
			let pA2 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 1);
			let pA3 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 2);
			let pA4 = isFloat<T>() ? pA1 : b2LaneState(states, identity, c + CW_INDEXA, 3);
			let pB1 = b2LaneState(states, identity, c + CW_INDEXB, 0);
			let pB2 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 1);
			let pB3 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 2);
			let pB4 = isFloat<T>() ? pB1 : b2LaneState(states, identity, c + CW_INDEXB, 3);
			let bA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW0);
			let bB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW0);

			let restitution = b2LoadL<T>(c + CW_RESTITUTION);
			let restitutionMask = b2GreaterThanL(restitution, zeroW);
			let normalMass1 = b2LoadL<T>(c + CW_NORMALMASS1);
			let normalMass2 = b2LoadL<T>(c + CW_NORMALMASS2);
			if (propagate == false) {
				normalMass1 = b2BlendL(zeroW, normalMass1, restitutionMask);
				normalMass2 = b2BlendL(zeroW, normalMass2, restitutionMask);
			}

			let normalX = b2LoadL<T>(c + CW_NORMALX);
			let normalY = b2LoadL<T>(c + CW_NORMALY);
			let negRestitutionThreshold = b2SplatL<T>(negRestitutionThresholdValue);
			let invH = b2SplatL<T>(invHValue);

			let s1: T;
			let s2: T;
			{
				let dA = b2GatherRowL<T>(pA1, pA2, pA3, pA4, BS_ROW1);
				let dB = b2GatherRowL<T>(pB1, pB2, pB3, pB4, BS_ROW1);
				let dpX = b2SubL(dB.c0, dA.c0);
				let dpY = b2SubL(dB.c1, dA.c1);
				s1 = b2SeparationL<T>(c, CW_ANCHORA1X, CW_ANCHORB1X, CW_BASESEPARATION1, dpX, dpY, dA.c2, dA.c3, dB.c2, dB.c3, normalX, normalY);
				s2 = b2SeparationL<T>(c, CW_ANCHORA2X, CW_ANCHORB2X, CW_BASESEPARATION2, dpX, dpY, dA.c2, dA.c3, dB.c2, dB.c3, normalX, normalY);
			}

			let vAX = bA.c0;
			let vAY = bA.c1;
			let wA = bA.c2;
			let vBX = bB.c0;
			let vBY = bB.c1;
			let wB = bB.c2;

			let invMassA = b2LoadL<T>(c + CW_INVMASSA);
			let invMassB = b2LoadL<T>(c + CW_INVMASSB);
			let invIA = b2LoadL<T>(c + CW_INVIA);
			let invIB = b2LoadL<T>(c + CW_INVIB);

			{
				let rAX = b2LoadL<T>(c + CW_ANCHORA1X);
				let rAY = b2LoadL<T>(c + CW_ANCHORA1Y);
				let rBX = b2LoadL<T>(c + CW_ANCHORB1X);
				let rBY = b2LoadL<T>(c + CW_ANCHORB1Y);

				let dvx = b2SubL(b2SubL(vBX, b2MulL(wB, rBY)), b2SubL(vAX, b2MulL(wA, rAY)));
				let dvy = b2SubL(b2AddL(vBY, b2MulL(wB, rBX)), b2AddL(vAY, b2MulL(wA, rAX)));
				let vn = b2AddL(b2MulL(dvx, normalX), b2MulL(dvy, normalY));

				let impulse = b2RestitutionImpulseL<T>(
					c,
					CW_NORMALIMPULSE1,
					CW_TOTALNORMALIMPULSE1,
					CW_RESTITUTIONIMPULSE1,
					CW_RELATIVEVELOCITY1,
					normalMass1,
					restitution,
					restitutionMask,
					negRestitutionThreshold,
					s1,
					vn,
					invH
				);

				let Px = b2MulL(impulse, normalX);
				let Py = b2MulL(impulse, normalY);

				vAX = b2MulSubL(vAX, invMassA, Px);
				vAY = b2MulSubL(vAY, invMassA, Py);
				wA = b2MulSubL(wA, invIA, b2SubL(b2MulL(rAX, Py), b2MulL(rAY, Px)));

				vBX = b2MulAddL(vBX, invMassB, Px);
				vBY = b2MulAddL(vBY, invMassB, Py);
				wB = b2MulAddL(wB, invIB, b2SubL(b2MulL(rBX, Py), b2MulL(rBY, Px)));
			}

			{
				let rAX = b2LoadL<T>(c + CW_ANCHORA2X);
				let rAY = b2LoadL<T>(c + CW_ANCHORA2Y);
				let rBX = b2LoadL<T>(c + CW_ANCHORB2X);
				let rBY = b2LoadL<T>(c + CW_ANCHORB2Y);

				let dvx = b2SubL(b2SubL(vBX, b2MulL(wB, rBY)), b2SubL(vAX, b2MulL(wA, rAY)));
				let dvy = b2SubL(b2AddL(vBY, b2MulL(wB, rBX)), b2AddL(vAY, b2MulL(wA, rAX)));
				let vn = b2AddL(b2MulL(dvx, normalX), b2MulL(dvy, normalY));

				let impulse = b2RestitutionImpulseL<T>(
					c,
					CW_NORMALIMPULSE2,
					CW_TOTALNORMALIMPULSE2,
					CW_RESTITUTIONIMPULSE2,
					CW_RELATIVEVELOCITY2,
					normalMass2,
					restitution,
					restitutionMask,
					negRestitutionThreshold,
					s2,
					vn,
					invH
				);

				let Px = b2MulL(impulse, normalX);
				let Py = b2MulL(impulse, normalY);

				vAX = b2MulSubL(vAX, invMassA, Px);
				vAY = b2MulSubL(vAY, invMassA, Py);
				wA = b2MulSubL(wA, invIA, b2SubL(b2MulL(rAX, Py), b2MulL(rAY, Px)));

				vBX = b2MulAddL(vBX, invMassB, Px);
				vBY = b2MulAddL(vBY, invMassB, Py);
				wB = b2MulAddL(wB, invIB, b2SubL(b2MulL(rBX, Py), b2MulL(rBY, Px)));
			}

			b2ScatterBodiesL<T>(states, c + CW_INDEXA, vAX, vAY, wA, bA.c3);
			b2ScatterBodiesL<T>(states, c + CW_INDEXB, vBX, vBY, wB, bB.c3);
		}
	}
}

// I tried adding this to the last relax iterations but it was slower.
//
// Runs as a flat parallel-for over the whole wide constraint range. Per-color
// contact sims are looked up through the prepareSpans cursor rather than the
// block's colorIndex, matching the layout of b2PrepareContacts_Wide.
//
// I could store the manifold pointer in the b2ContactConstraintWide to simplify
// this. But it is important to keep the constraint small.
@noinline
export function b2StoreImpulses_WideW4(block: b2SolverBlock, context: b2StepContext, workerIndex: i32): void {
	let world = context.world;
	let spans = context.contactPrepareSpans;
	let wideBase = changetype<usize>(context.wideContactConstraints);
	let taskContext = world.taskContexts[workerIndex];
	let hitEventBitSet = taskContext.hitEventBitSet;
	let hasHitEvents = taskContext.hasHitEvents;
	let negHitThreshold = -world.hitEventThreshold;

	let wideIndex = block.startIndex;
	let endWideIndex = block.startIndex + <i32>block.count;

	// Find color for start index
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= wideIndex) {
		colorIndex += 1;
	}

	while (wideIndex < endWideIndex) {
		let colorWideEndIndex = b2MinInt(spans[colorIndex + 1].start, endWideIndex);
		let colorWideStart = spans[colorIndex].start;
		let colorContactCount = spans[colorIndex].count;
		let contactSims = b2ArrayData(spans[colorIndex].contacts);

		for (; wideIndex < colorWideEndIndex; ++wideIndex) {
			// wideIndex is below the step's wide constraint count
			let c = wideBase + <usize>wideIndex * sizeof<b2ContactConstraintWide>();

			let localWideIndex = wideIndex - colorWideStart;
			let baseIndex = 4 * localWideIndex;

			for (let laneIndex = 0; laneIndex < 4; ++laneIndex) {
				let contactIndex = baseIndex + laneIndex;
				if (contactIndex >= colorContactCount) {
					break;
				}

				// ( (const float*)&c->field )[laneIndex]; contactIndex is a live sim of the color
				let lane = <usize>laneIndex * 4;
				let contactSim = contactSims + <usize>contactIndex * sizeof<b2ContactSim>();
				store<f32>(contactSim + CS_ROLLINGIMPULSEM, load<f32>(c + CW_ROLLINGIMPULSE + lane));

				store<f32>(contactSim + CS_POINT0 + MP_NORMALIMPULSE, load<f32>(c + CW_NORMALIMPULSE1 + lane));
				store<f32>(contactSim + CS_POINT0 + MP_TANGENTIMPULSE, load<f32>(c + CW_TANGENTIMPULSE1 + lane));
				store<f32>(contactSim + CS_POINT0 + MP_TOTALNORMALIMPULSE, load<f32>(c + CW_TOTALNORMALIMPULSE1 + lane));

				store<f32>(contactSim + CS_POINT1 + MP_NORMALIMPULSE, load<f32>(c + CW_NORMALIMPULSE2 + lane));
				store<f32>(contactSim + CS_POINT1 + MP_TANGENTIMPULSE, load<f32>(c + CW_TANGENTIMPULSE2 + lane));
				store<f32>(contactSim + CS_POINT1 + MP_TOTALNORMALIMPULSE, load<f32>(c + CW_TOTALNORMALIMPULSE2 + lane));

				// Check for hit events to speed up serial processing later in the step
				if ((load<u32>(contactSim + CS_SIMFLAGS) & b2_simEnableHitEvent) != 0) {
					let pointCount = load<i32>(contactSim + CS_POINTCOUNT);
					for (let k = 0; k < pointCount; ++k) {
						let mp = contactSim + CS_POINT0 + <usize>k * sizeof<b2ManifoldPoint>();

						// Need to check total impulse because the point may be speculative and not colliding
						if (load<f32>(mp + MP_NORMALVELOCITY) < negHitThreshold && load<f32>(mp + MP_TOTALNORMALIMPULSE) > 0.0) {
							b2SetBit(hitEventBitSet, <u32>load<i32>(contactSim + CS_CONTACTID));
							hasHitEvents = true;
							break;
						}
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	taskContext.hasHitEvents = hasHitEvents;
}

export function b2PrepareContacts_WideW4(block: b2SolverBlock, context: b2StepContext): void {
	if (ASC_FEATURE_SIMD) {
		b2PrepareContactsL<v128>(block, context);
	} else {
		b2PrepareContactsL<f32>(block, context);
	}
}

export function b2WarmStartContacts_WideW4(block: b2SolverBlock, context: b2StepContext): void {
	if (ASC_FEATURE_SIMD) {
		b2WarmStartContactsL<v128>(block, context);
	} else {
		b2WarmStartContactsL<f32>(block, context);
	}
}

export function b2PushContacts_WideW4(block: b2SolverBlock, context: b2StepContext): void {
	if (ASC_FEATURE_SIMD) {
		b2PushContactsL<v128>(block, context);
	} else {
		b2PushContactsL<f32>(block, context);
	}
}

export function b2SolveContacts_WideW4(block: b2SolverBlock, context: b2StepContext): void {
	if (ASC_FEATURE_SIMD) {
		b2SolveContactsL<v128>(block, context);
	} else {
		b2SolveContactsL<f32>(block, context);
	}
}

export function b2ApplyRestitution_WideW4(block: b2SolverBlock, context: b2StepContext): void {
	if (ASC_FEATURE_SIMD) {
		b2ApplyRestitutionL<v128>(block, context);
	} else {
		b2ApplyRestitutionL<f32>(block, context);
	}
}
