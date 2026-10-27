// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/joint.c

import { b2Mat22, b2Transform, b2Vec2 } from "./math_functions";
import { b2JointType } from "./types";
import { b2Softness } from "./solver";

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export struct b2JointEdge {
	bodyId: i32 = 0;
	prevKey: i32 = 0;
	nextKey: i32 = 0;
}

// Map from b2JointId to b2Joint in the solver sets
export struct b2Joint {
	userData: u64 = 0;

	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	setIndex: i32 = 0;

	// index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
	// B2_NULL_INDEX when slot is free
	colorIndex: i32 = 0;

	// joint index within set or graph color
	// B2_NULL_INDEX when slot is free
	localIndex: i32 = 0;

	edges: FixedArray<b2JointEdge, 2>;

	jointId: i32 = 0;
	islandId: i32 = 0;

	// Index into the island's joints array for O(1) swap-removal.
	// B2_NULL_INDEX when not in an island.
	islandIndex: i32 = 0;

	drawScale: f32 = 0;

	type: b2JointType = 0;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2JointId
	generation: u16 = 0;

	collideConnected: bool = false;
}

// The pogo joint is a hybrid between a contact and a joint
export struct b2PogoJoint {
	// Settings
	normal: b2Vec2 = b2Vec2();
	restLength: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	maxTensionForce: f32 = 0;
	maxCompressionForce: f32 = 0;

	// Runtime
	impulse: f32 = 0;
	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	linearMass: f32 = 0;
	velocity: f32 = 0;
}

export struct b2DistanceJoint {
	length: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	lowerSpringForce: f32 = 0;
	upperSpringForce: f32 = 0;
	minLength: f32 = 0;
	maxLength: f32 = 0;

	maxMotorForce: f32 = 0;
	motorSpeed: f32 = 0;

	impulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	motorImpulse: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	anchorA: b2Vec2 = b2Vec2();
	anchorB: b2Vec2 = b2Vec2();
	deltaCenter: b2Vec2 = b2Vec2();
	distanceSoftness: b2Softness = b2Softness();
	axialMass: f32 = 0;

	enableSpring: bool = false;
	enableLimit: bool = false;
	enableMotor: bool = false;
}

export struct b2MotorJoint {
	linearVelocity: b2Vec2 = b2Vec2();
	maxVelocityForce: f32 = 0;
	angularVelocity: f32 = 0;
	maxVelocityTorque: f32 = 0;
	linearHertz: f32 = 0;
	linearDampingRatio: f32 = 0;
	maxSpringForce: f32 = 0;
	angularHertz: f32 = 0;
	angularDampingRatio: f32 = 0;
	maxSpringTorque: f32 = 0;

	linearVelocityImpulse: b2Vec2 = b2Vec2();
	angularVelocityImpulse: f32 = 0;
	linearSpringImpulse: b2Vec2 = b2Vec2();
	angularSpringImpulse: f32 = 0;

	linearSpring: b2Softness = b2Softness();
	angularSpring: b2Softness = b2Softness();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	linearMass: b2Mat22 = b2Mat22();
	angularMass: f32 = 0;
}

export struct b2MoverJoint {
	linearVelocity: b2Vec2 = b2Vec2();
	maxVelocityForce: b2Vec2 = b2Vec2();

	linearVelocityImpulse: b2Vec2 = b2Vec2();

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	linearMass: f32 = 0;
}

export struct b2PrismaticJoint {
	impulse: b2Vec2 = b2Vec2();
	springImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	targetTranslation: f32 = 0;
	maxMotorForce: f32 = 0;
	motorSpeed: f32 = 0;
	lowerTranslation: f32 = 0;
	upperTranslation: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	springSoftness: b2Softness = b2Softness();

	enableSpring: bool = false;
	enableLimit: bool = false;
	enableMotor: bool = false;
}

export struct b2RevoluteJoint {
	linearImpulse: b2Vec2 = b2Vec2();
	springImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;
	targetAngle: f32 = 0;
	maxMotorTorque: f32 = 0;
	motorSpeed: f32 = 0;
	lowerAngle: f32 = 0;
	upperAngle: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	axialMass: f32 = 0;
	springSoftness: b2Softness = b2Softness();

	enableSpring: bool = false;
	enableMotor: bool = false;
	enableLimit: bool = false;
}

export struct b2WeldJoint {
	linearHertz: f32 = 0;
	linearDampingRatio: f32 = 0;
	angularHertz: f32 = 0;
	angularDampingRatio: f32 = 0;

	linearSpring: b2Softness = b2Softness();
	angularSpring: b2Softness = b2Softness();
	linearImpulse: b2Vec2 = b2Vec2();
	angularImpulse: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	axialMass: f32 = 0;
}

export struct b2WheelJoint {
	perpImpulse: f32 = 0;
	motorImpulse: f32 = 0;
	springImpulse: f32 = 0;
	lowerImpulse: f32 = 0;
	upperImpulse: f32 = 0;
	maxMotorTorque: f32 = 0;
	motorSpeed: f32 = 0;
	lowerTranslation: f32 = 0;
	upperTranslation: f32 = 0;
	hertz: f32 = 0;
	dampingRatio: f32 = 0;

	indexA: i32 = 0;
	indexB: i32 = 0;
	frameA: b2Transform = b2Transform();
	frameB: b2Transform = b2Transform();
	deltaCenter: b2Vec2 = b2Vec2();
	perpMass: f32 = 0;
	motorMass: f32 = 0;
	axialMass: f32 = 0;
	springSoftness: b2Softness = b2Softness();

	enableSpring: bool = false;
	enableMotor: bool = false;
	enableLimit: bool = false;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export struct b2JointSim {
	jointId: i32 = 0;

	bodyIdA: i32 = 0;
	bodyIdB: i32 = 0;

	type: b2JointType = 0;

	localFrameA: b2Transform = b2Transform();
	localFrameB: b2Transform = b2Transform();

	invMassA: f32 = 0;
	invMassB: f32 = 0;
	invIA: f32 = 0;
	invIB: f32 = 0;

	constraintHertz: f32 = 0;
	constraintDampingRatio: f32 = 0;

	constraintSoftness: b2Softness = b2Softness();

	forceThreshold: f32 = 0;
	torqueThreshold: f32 = 0;

	// C's union, one field per member
	distanceJoint: b2DistanceJoint = b2DistanceJoint();
	motorJoint: b2MotorJoint = b2MotorJoint();
	moverJoint: b2MoverJoint = b2MoverJoint();
	pogoJoint: b2PogoJoint = b2PogoJoint();
	revoluteJoint: b2RevoluteJoint = b2RevoluteJoint();
	prismaticJoint: b2PrismaticJoint = b2PrismaticJoint();
	weldJoint: b2WeldJoint = b2WeldJoint();
	wheelJoint: b2WheelJoint = b2WheelJoint();
}

// functions from joint.c follow
