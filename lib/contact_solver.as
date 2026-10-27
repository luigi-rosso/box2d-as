// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/contact_solver.c and the B2_SIMD_NONE path of src/contact_solver_wide.inl

import { b2Vec2 } from "./math_functions";
import { b2Softness } from "./solver";

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

// functions from contact_solver.c follow
