// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Joints: a chain of capsules swings down from a fixed anchor on revolute joints, and a platform rides a prismatic
// joint whose motor lifts it to the joint's upper limit.

import {
	B2_PI,
	b2Body_GetPosition,
	b2Capsule,
	b2CreateBody,
	b2CreateCapsuleShape,
	b2CreatePolygonShape,
	b2CreatePrismaticJoint,
	b2CreateRevoluteJoint,
	b2CreateWorld,
	b2DefaultBodyDef,
	b2DefaultPrismaticJointDef,
	b2DefaultRevoluteJointDef,
	b2DefaultShapeDef,
	b2DefaultWorldDef,
	b2DestroyWorld,
	b2MakeBox,
	b2MakeRot,
	b2PrismaticJoint_GetCurrentTranslation,
	b2Vec2,
	b2World_Step,
	b2_dynamicBody,
} from "../lib/box2d";

export const LINK_COUNT: i32 = 10;
export const LINK_LENGTH: f32 = 1.0;
export const ANCHOR_HEIGHT: f32 = 12.0;

export class JointsResult {
	// The center of the chain's last link, the chain hanging from (0, ANCHOR_HEIGHT)
	chainEnd: b2Vec2 = b2Vec2();
	// The platform's travel along its axis
	elevatorTranslation: f32 = 0;
}

export function joints(): JointsResult {
	let worldId = b2CreateWorld(b2DefaultWorldDef());
	let groundId = b2CreateBody(worldId, b2DefaultBodyDef());

	// The chain starts horizontal along +x. A revolute joint pins the origins of its two frames together, and each
	// link's frames sit at its two ends.
	let half: f32 = 0.5 * LINK_LENGTH;
	let link = b2Capsule(b2Vec2(-half, 0.0), b2Vec2(half, 0.0), 0.1);
	let bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	// Damping lets the swing settle
	bodyDef.linearDamping = 0.5;
	let shapeDef = b2DefaultShapeDef();

	let jointDef = b2DefaultRevoluteJointDef();
	jointDef.base.localFrameA.p = b2Vec2(0.0, ANCHOR_HEIGHT);
	jointDef.base.localFrameB.p = b2Vec2(-half, 0.0);

	let parentId = groundId;
	let lastId = groundId;
	for (let i = 0; i < LINK_COUNT; ++i) {
		bodyDef.position = b2Vec2((<f32>i + 0.5) * LINK_LENGTH, ANCHOR_HEIGHT);
		let linkId = b2CreateBody(worldId, bodyDef);
		b2CreateCapsuleShape(linkId, shapeDef, link);

		jointDef.base.bodyIdA = parentId;
		jointDef.base.bodyIdB = linkId;
		b2CreateRevoluteJoint(worldId, jointDef);

		jointDef.base.localFrameA.p = b2Vec2(half, 0.0);
		parentId = linkId;
		lastId = linkId;
	}

	// The platform, clear of the chain's swing. A prismatic joint slides along the x axis of frame A, turned here
	// to point up.
	let up = b2MakeRot(0.5 * B2_PI);
	bodyDef.position = b2Vec2(-6.0, 1.0);
	bodyDef.linearDamping = 0.0;
	let platformId = b2CreateBody(worldId, bodyDef);
	b2CreatePolygonShape(platformId, shapeDef, b2MakeBox(1.0, 0.1));

	let prismaticDef = b2DefaultPrismaticJointDef();
	prismaticDef.base.bodyIdA = groundId;
	prismaticDef.base.bodyIdB = platformId;
	prismaticDef.base.localFrameA.p = b2Vec2(-6.0, 1.0);
	prismaticDef.base.localFrameA.q = up;
	prismaticDef.base.localFrameB.q = up;
	prismaticDef.enableLimit = true;
	prismaticDef.lowerTranslation = 0.0;
	prismaticDef.upperTranslation = 3.0;
	prismaticDef.enableMotor = true;
	prismaticDef.motorSpeed = 1.0;
	// Newtons, more than the weight of the 0.2 kg platform
	prismaticDef.maxMotorForce = 100.0;
	let elevatorId = b2CreatePrismaticJoint(worldId, prismaticDef);

	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 600; ++i) {
		b2World_Step(worldId, timeStep, 4);
	}

	let result = new JointsResult();
	result.chainEnd = b2Body_GetPosition(lastId);
	result.elevatorTranslation = b2PrismaticJoint_GetCurrentTranslation(elevatorId);
	b2DestroyWorld(worldId);
	return result;
}
