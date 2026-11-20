// Mirror of verify/cref/joints.c.
import { vbool, vend, vf32, vi32, vrandf, vseed, vtag } from "./common";
import { b2IntegrateRotation, b2MakeRot, b2MulAdd, b2Normalize, b2Transform, b2Vec2 } from "../../lib/math_functions";
import { b2BodyId, b2JointId, b2WorldId } from "../../lib/id";
import {
	b2DefaultBodyDef,
	b2DefaultDistanceJointDef,
	b2DefaultFilterJointDef,
	b2DefaultMotorJointDef,
	b2DefaultMoverJointDef,
	b2DefaultPogoJointDef,
	b2DefaultPrismaticJointDef,
	b2DefaultRevoluteJointDef,
	b2DefaultShapeDef,
	b2DefaultWeldJointDef,
	b2DefaultWheelJointDef,
	b2DefaultWorldDef,
	b2_distanceJoint,
	b2_dynamicBody,
	b2_motorJoint,
	b2_moverJoint,
	b2_pogoJoint,
	b2_prismaticJoint,
	b2_revoluteJoint,
	b2_staticBody,
	b2_weldJoint,
	b2_wheelJoint,
} from "../../lib/types";
import { b2SetLogCallback } from "../../lib/core";
import { B2_GRAPH_COLOR_COUNT } from "../../lib/constants";
import { b2GetBit, b2SetBitCountAndClear } from "../../lib/bitset";
import { b2GetIdCount } from "../../lib/id_pool";
import { b2MakeBox, b2MakeOffsetBox } from "../../lib/geometry";
import { b2CreateWorld, b2GetWorldFromId } from "../../lib/physics_world";
import { b2Body_SetType, b2CreateBody } from "../../lib/body";
import { b2CreatePolygonShape } from "../../lib/shape";
import { b2_awakeSet } from "../../lib/solver_set";
import { B2_OVERFLOW_INDEX } from "../../lib/constraint_graph";
import { b2JointPrepareSpan, b2SolverBlock, b2StepContext, b2_emptyJointSims } from "../../lib/solver";
import {
	b2CreateDistanceJoint,
	b2CreateFilterJoint,
	b2CreateMotorJoint,
	b2CreateMoverJoint,
	b2CreatePogoJoint,
	b2CreatePrismaticJoint,
	b2CreateRevoluteJoint,
	b2CreateWeldJoint,
	b2CreateWheelJoint,
	b2DestroyJoint,
	b2GetJointReaction,
	b2JointSimAddress,
	b2JointSimView,
	b2PrepareJointsTask,
	b2PrepareJoints_Overflow,
	b2SolveJointsTask,
	b2SolveJoints_Overflow,
	b2WarmStartJointsTask,
	b2WarmStartJoints_Overflow,
} from "../../lib/joint";
import { b2DistanceJoint_EnableLimit, b2DistanceJoint_EnableMotor, b2DistanceJoint_EnableSpring, b2DistanceJoint_GetCurrentLength, b2DistanceJoint_GetCurrentMotorForce, b2DistanceJoint_GetLowerLimit, b2DistanceJoint_GetMaxMotorForce, b2DistanceJoint_GetRestLength, b2DistanceJoint_GetSpringDampingRatio, b2DistanceJoint_GetSpringForce, b2DistanceJoint_GetSpringForceRange, b2DistanceJoint_GetSpringHertz, b2DistanceJoint_GetTargetMotorSpeed, b2DistanceJoint_GetUpperLimit, b2DistanceJoint_IsLimitEnabled, b2DistanceJoint_IsMotorEnabled, b2DistanceJoint_IsSpringEnabled, b2DistanceJoint_SetLimits, b2DistanceJoint_SetMaxMotorForce, b2DistanceJoint_SetRestLength, b2DistanceJoint_SetSpringDampingRatio, b2DistanceJoint_SetSpringForceRange, b2DistanceJoint_SetSpringHertz, b2DistanceJoint_SetTargetMotorSpeed } from "../../lib/distance_joint";
import { b2Joint_GetAngularSeparation, b2Joint_GetBodyA, b2Joint_GetBodyB, b2Joint_GetCollideConnected, b2Joint_GetConstraintForce, b2Joint_GetConstraintTorque, b2Joint_GetConstraintTuning, b2Joint_GetForceThreshold, b2Joint_GetLinearSeparation, b2Joint_GetLocalFrameA, b2Joint_GetLocalFrameB, b2Joint_GetTorqueThreshold, b2Joint_GetType, b2Joint_GetUserData, b2Joint_SetCollideConnected, b2Joint_SetConstraintTuning, b2Joint_SetForceThreshold, b2Joint_SetLocalFrameA, b2Joint_SetLocalFrameB, b2Joint_SetTorqueThreshold, b2Joint_SetUserData } from "../../lib/joint";
import { b2MotorJoint_GetAngularDampingRatio, b2MotorJoint_GetAngularHertz, b2MotorJoint_GetAngularVelocity, b2MotorJoint_GetLinearDampingRatio, b2MotorJoint_GetLinearHertz, b2MotorJoint_GetLinearVelocity, b2MotorJoint_GetMaxSpringForce, b2MotorJoint_GetMaxSpringTorque, b2MotorJoint_GetMaxVelocityForce, b2MotorJoint_GetMaxVelocityTorque, b2MotorJoint_SetAngularDampingRatio, b2MotorJoint_SetAngularHertz, b2MotorJoint_SetAngularVelocity, b2MotorJoint_SetLinearDampingRatio, b2MotorJoint_SetLinearHertz, b2MotorJoint_SetLinearVelocity, b2MotorJoint_SetMaxSpringForce, b2MotorJoint_SetMaxSpringTorque, b2MotorJoint_SetMaxVelocityForce, b2MotorJoint_SetMaxVelocityTorque } from "../../lib/motor_joint";
import { b2MoverJoint_GetLinearVelocity, b2MoverJoint_GetMaxVelocityForce, b2MoverJoint_SetLinearVelocity, b2MoverJoint_SetMaxVelocityForce } from "../../lib/mover_joint";
import { b2PogoJoint_GetCurrentLength, b2PogoJoint_GetCurrentSpeed, b2PogoJoint_GetImpulse, b2PogoJoint_GetRestLength, b2PogoJoint_GetSpringDampingRatio, b2PogoJoint_GetSpringHertz, b2PogoJoint_SetRestLength, b2PogoJoint_SetSpringDampingRatio, b2PogoJoint_SetSpringHertz } from "../../lib/pogo_joint";
import { b2PrismaticJoint_EnableLimit, b2PrismaticJoint_EnableMotor, b2PrismaticJoint_EnableSpring, b2PrismaticJoint_GetCurrentMotorForce, b2PrismaticJoint_GetCurrentSpeed, b2PrismaticJoint_GetCurrentTranslation, b2PrismaticJoint_GetLowerLimit, b2PrismaticJoint_GetMaxMotorForce, b2PrismaticJoint_GetSpringDampingRatio, b2PrismaticJoint_GetSpringHertz, b2PrismaticJoint_GetTargetMotorSpeed, b2PrismaticJoint_GetTargetTranslation, b2PrismaticJoint_GetUpperLimit, b2PrismaticJoint_IsLimitEnabled, b2PrismaticJoint_IsMotorEnabled, b2PrismaticJoint_IsSpringEnabled, b2PrismaticJoint_SetLimits, b2PrismaticJoint_SetMaxMotorForce, b2PrismaticJoint_SetSpringDampingRatio, b2PrismaticJoint_SetSpringHertz, b2PrismaticJoint_SetTargetMotorSpeed, b2PrismaticJoint_SetTargetTranslation } from "../../lib/prismatic_joint";
import { b2RevoluteJoint_EnableLimit, b2RevoluteJoint_EnableMotor, b2RevoluteJoint_EnableSpring, b2RevoluteJoint_GetCurrentAngle, b2RevoluteJoint_GetCurrentMotorTorque, b2RevoluteJoint_GetCurrentSpeed, b2RevoluteJoint_GetLowerLimit, b2RevoluteJoint_GetMaxMotorTorque, b2RevoluteJoint_GetSpringDampingRatio, b2RevoluteJoint_GetSpringHertz, b2RevoluteJoint_GetTargetAngle, b2RevoluteJoint_GetTargetMotorSpeed, b2RevoluteJoint_GetUpperLimit, b2RevoluteJoint_IsLimitEnabled, b2RevoluteJoint_IsMotorEnabled, b2RevoluteJoint_IsSpringEnabled, b2RevoluteJoint_SetLimits, b2RevoluteJoint_SetMaxMotorTorque, b2RevoluteJoint_SetSpringDampingRatio, b2RevoluteJoint_SetSpringHertz, b2RevoluteJoint_SetTargetAngle, b2RevoluteJoint_SetTargetMotorSpeed } from "../../lib/revolute_joint";
import { b2WeldJoint_GetAngularDampingRatio, b2WeldJoint_GetAngularHertz, b2WeldJoint_GetLinearDampingRatio, b2WeldJoint_GetLinearHertz, b2WeldJoint_SetAngularDampingRatio, b2WeldJoint_SetAngularHertz, b2WeldJoint_SetLinearDampingRatio, b2WeldJoint_SetLinearHertz } from "../../lib/weld_joint";
import { b2WheelJoint_EnableLimit, b2WheelJoint_EnableMotor, b2WheelJoint_EnableSpring, b2WheelJoint_GetCurrentMotorTorque, b2WheelJoint_GetLowerLimit, b2WheelJoint_GetMaxMotorTorque, b2WheelJoint_GetSpringDampingRatio, b2WheelJoint_GetSpringHertz, b2WheelJoint_GetTargetMotorSpeed, b2WheelJoint_GetUpperLimit, b2WheelJoint_IsLimitEnabled, b2WheelJoint_IsMotorEnabled, b2WheelJoint_IsSpringEnabled, b2WheelJoint_SetLimits, b2WheelJoint_SetMaxMotorTorque, b2WheelJoint_SetSpringDampingRatio, b2WheelJoint_SetSpringHertz, b2WheelJoint_SetTargetMotorSpeed } from "../../lib/wheel_joint";

export { frameBoundary } from "./common";

let worldId = b2WorldId();
const bodies = new StaticArray<b2BodyId>(9);
const joints = new StaticArray<b2JointId>(16);
let jointCount = 0;

function vvec(v: b2Vec2): void {
	vf32(v.x);
	vf32(v.y);
}

function emitStates(tag: string): void {
	let world = b2GetWorldFromId(worldId);
	let states = world.solverSets[b2_awakeSet].bodyStates;
	for (let i = 0; i < states.length; ++i) {
		vtag(tag);
		vi32(i);
		vvec(states[i].linearVelocity);
		vf32(states[i].angularVelocity);
		vend();
	}
}

function emitSims(tag: string): void {
	let world = b2GetWorldFromId(worldId);
	for (let c = 0; c < B2_GRAPH_COLOR_COUNT; ++c) {
		let sims = world.constraintGraph.colors[c].jointSims;
		let view = b2JointSimView(sims);
		for (let i = 0; i < sims.length; ++i) {
			ref sim = view[i];
			let r = b2Vec2();
			b2GetJointReaction(b2JointSimAddress(view, i), 60.0, r.x, r.y);
			vtag(tag);
			vi32(c);
			vi32(sim.jointId);
			vf32(r.x);
			vf32(r.y);
			vf32(sim.invMassA);
			vf32(sim.invIB);
			vf32(sim.constraintSoftness.biasRate);
			vf32(sim.constraintSoftness.massScale);
			vend();
		}
	}
}

function addJoint(id: b2JointId): void {
	joints[jointCount++] = id;
}

function build(): void {
	let wd = b2DefaultWorldDef();
	worldId = b2CreateWorld(wd);

	let bd = b2DefaultBodyDef();
	bodies[0] = b2CreateBody(worldId, bd);
	let sd = b2DefaultShapeDef();
	let ground = b2MakeBox(10.0, 0.5);
	b2CreatePolygonShape(bodies[0], sd, ground);

	for (let i = 1; i < 9; ++i) {
		bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = b2Vec2(-4.0 + 1.1 * <f32>i, 2.0 + 0.3 * <f32>i);
		bd.rotation = b2MakeRot(0.2 * <f32>i);
		bodies[i] = b2CreateBody(worldId, bd);
		let box = b2MakeOffsetBox(0.4, 0.2 + 0.05 * <f32>i, b2Vec2(0.1, -0.05), b2MakeRot(0.1));
		b2CreatePolygonShape(bodies[i], sd, box);
	}
	{
		let d = b2DefaultDistanceJointDef();
		d.base.bodyIdA = bodies[0];
		d.base.bodyIdB = bodies[1];
		d.base.localFrameA.p = b2Vec2(-3.0, 4.0);
		d.base.localFrameB.p = b2Vec2(0.2, 0.1);
		d.length = 1.5;
		addJoint(b2CreateDistanceJoint(worldId, d));

		d = b2DefaultDistanceJointDef();
		d.base.bodyIdA = bodies[1];
		d.base.bodyIdB = bodies[2];
		d.base.localFrameB.p = b2Vec2(-0.3, 0.1);
		d.length = 1.2;
		d.enableSpring = true;
		d.hertz = 3.0;
		d.dampingRatio = 0.4;
		d.lowerSpringForce = -50.0;
		d.upperSpringForce = 40.0;
		d.enableLimit = true;
		d.minLength = 0.9;
		d.maxLength = 1.1;
		d.enableMotor = true;
		d.maxMotorForce = 5.0;
		d.motorSpeed = 0.5;
		d.base.forceThreshold = 0.0;
		addJoint(b2CreateDistanceJoint(worldId, d));
	}

	{
		let m = b2DefaultMotorJointDef();
		m.base.bodyIdA = bodies[0];
		m.base.bodyIdB = bodies[3];
		m.base.localFrameA.p = b2Vec2(-0.5, 3.0);
		m.base.localFrameA.q = b2MakeRot(0.3);
		m.linearVelocity = b2Vec2(0.5, -0.2);
		m.maxVelocityForce = 20.0;
		m.angularVelocity = 0.7;
		m.maxVelocityTorque = 4.0;
		m.linearHertz = 2.0;
		m.linearDampingRatio = 0.6;
		m.maxSpringForce = 30.0;
		m.angularHertz = 1.5;
		m.angularDampingRatio = 0.5;
		m.maxSpringTorque = 6.0;
		addJoint(b2CreateMotorJoint(worldId, m));
	}

	{
		let m = b2DefaultMoverJointDef();
		m.base.bodyIdA = bodies[0];
		m.base.bodyIdB = bodies[4];
		m.linearVelocity = b2Vec2(1.0, 0.3);
		m.maxVelocityForce = b2Vec2(10.0, 2.0);
		addJoint(b2CreateMoverJoint(worldId, m));
	}

	{
		let p = b2DefaultPogoJointDef();
		p.base.bodyIdA = bodies[0];
		p.base.bodyIdB = bodies[5];
		p.base.localFrameA.p = b2Vec2(1.5, 0.5);
		p.normal = b2Normalize(b2Vec2(0.2, 1.0));
		p.restLength = 3.0;
		p.hertz = 4.0;
		p.dampingRatio = 0.8;
		p.maxTensionForce = 15.0;
		p.maxCompressionForce = 60.0;
		p.impulse = 0.05;
		p.velocity = -0.1;
		addJoint(b2CreatePogoJoint(worldId, p));
	}

	{
		let p = b2DefaultPrismaticJointDef();
		p.base.bodyIdA = bodies[2];
		p.base.bodyIdB = bodies[3];
		p.base.localFrameA.q = b2MakeRot(0.6);
		p.base.localFrameA.p = b2Vec2(0.3, 0.2);
		p.base.localFrameB.p = b2Vec2(-0.2, 0.0);
		p.enableSpring = true;
		p.hertz = 2.5;
		p.dampingRatio = 0.3;
		p.targetTranslation = 0.4;
		p.enableLimit = true;
		p.lowerTranslation = -0.2;
		p.upperTranslation = 0.3;
		p.enableMotor = true;
		p.maxMotorForce = 8.0;
		p.motorSpeed = -0.6;
		p.base.torqueThreshold = 0.5;
		addJoint(b2CreatePrismaticJoint(worldId, p));
	}

	{
		let r = b2DefaultRevoluteJointDef();
		r.base.bodyIdA = bodies[3];
		r.base.bodyIdB = bodies[4];
		r.base.localFrameA.p = b2Vec2(0.5, 0.1);
		r.base.localFrameB.p = b2Vec2(-0.5, 0.0);
		r.base.localFrameB.q = b2MakeRot(-0.4);
		r.targetAngle = 0.3;
		r.enableSpring = true;
		r.hertz = 1.0;
		r.dampingRatio = 0.2;
		r.enableLimit = true;
		r.lowerAngle = -0.25;
		r.upperAngle = 0.1;
		r.enableMotor = true;
		r.maxMotorTorque = 3.0;
		r.motorSpeed = 1.2;
		addJoint(b2CreateRevoluteJoint(worldId, r));
	}

	{
		let w = b2DefaultWeldJointDef();
		w.base.bodyIdA = bodies[5];
		w.base.bodyIdB = bodies[6];
		w.base.localFrameA.p = b2Vec2(0.5, 0.0);
		w.base.localFrameB.p = b2Vec2(-0.5, 0.1);
		addJoint(b2CreateWeldJoint(worldId, w));

		w = b2DefaultWeldJointDef();
		w.base.bodyIdA = bodies[6];
		w.base.bodyIdB = bodies[7];
		w.base.localFrameA.p = b2Vec2(0.4, 0.0);
		w.base.localFrameB.p = b2Vec2(-0.4, 0.0);
		w.linearHertz = 5.0;
		w.linearDampingRatio = 0.7;
		w.angularHertz = 3.0;
		w.angularDampingRatio = 0.4;
		addJoint(b2CreateWeldJoint(worldId, w));
	}

	{
		let w = b2DefaultWheelJointDef();
		w.base.bodyIdA = bodies[7];
		w.base.bodyIdB = bodies[8];
		w.base.localFrameA.q = b2MakeRot(1.3);
		w.base.localFrameA.p = b2Vec2(0.0, -0.3);
		w.enableLimit = true;
		w.lowerTranslation = -0.1;
		w.upperTranslation = 0.15;
		w.enableMotor = true;
		w.maxMotorTorque = 2.0;
		w.motorSpeed = -3.0;
		addJoint(b2CreateWheelJoint(worldId, w));
	}

	{
		let f = b2DefaultFilterJointDef();
		f.base.bodyIdA = bodies[1];
		f.base.bodyIdB = bodies[8];
		addJoint(b2CreateFilterJoint(worldId, f));
	}

	let world = b2GetWorldFromId(worldId);
	let states = world.solverSets[b2_awakeSet].bodyStates;
	vseed(7);
	for (let i = 0; i < states.length; ++i) {
		let vx = vrandf(-2.0, 2.0);
		let vy = vrandf(-2.0, 2.0);
		let w = vrandf(-3.0, 3.0);
		states[i].linearVelocity = b2Vec2(vx, vy);
		states[i].angularVelocity = w;
	}
}

function block(startIndex: i32, count: i32, colorIndex: i32): b2SolverBlock {
	let b = b2SolverBlock();
	b.startIndex = startIndex;
	b.count = <u16>count;
	b.colorIndex = <u8>colorIndex;
	return b;
}

function stepJoints(warmStarting: bool): void {
	let world = b2GetWorldFromId(worldId);
	let states = world.solverSets[b2_awakeSet].bodyStates;

	let ctx = new b2StepContext(world);
	ctx.dt = 1.0 / <f32>60.0;
	ctx.inv_dt = 60.0;
	ctx.subStepCount = 4;
	ctx.h = ctx.dt / 4.0;
	ctx.inv_h = 4.0 * 60.0;
	ctx.states = states;
	ctx.enableWarmStarting = warmStarting;
	world.inv_h = ctx.inv_h;

	let spans = new StaticArray<b2JointPrepareSpan>(B2_GRAPH_COLOR_COUNT + 1);
	let spanCount = 0;
	let total = 0;
	for (let c = 0; c < B2_OVERFLOW_INDEX; ++c) {
		let sims = world.constraintGraph.colors[c].jointSims;
		if (sims.length == 0) {
			continue;
		}
		let span = new b2JointPrepareSpan();
		span.start = total;
		span.count = sims.length;
		span.joints = sims;
		spans[spanCount] = span;
		total += sims.length;
		spanCount += 1;
	}
	let sentinel = new b2JointPrepareSpan();
	sentinel.start = total;
	sentinel.count = 0;
	sentinel.joints = b2_emptyJointSims;
	spans[spanCount] = sentinel;
	ctx.jointPrepareSpans = spans;
	ctx.jointCount = total;

	let bits = world.taskContexts[0].jointStateBitSet;
	b2SetBitCountAndClear(bits, <u32>world.joints.length);

	let half = total / 2;
	b2PrepareJointsTask(block(0, half, 0), ctx);
	b2PrepareJointsTask(block(half, total - half, 0), ctx);
	b2PrepareJoints_Overflow(ctx);
	emitSims("p");

	for (let sub = 0; sub < 4; ++sub) {
		for (let c = 0; c < B2_OVERFLOW_INDEX; ++c) {
			let n = world.constraintGraph.colors[c].jointSims.length;
			if (n > 0) {
				b2WarmStartJointsTask(block(0, n, c), ctx);
			}
		}
		b2WarmStartJoints_Overflow(ctx);
		emitStates("w");

		for (let c = 0; c < B2_OVERFLOW_INDEX; ++c) {
			let n = world.constraintGraph.colors[c].jointSims.length;
			if (n > 0) {
				b2SolveJointsTask(block(0, n, c), ctx, true, 0);
			}
		}
		b2SolveJoints_Overflow(ctx, true);
		emitStates("s");

		for (let i = 0; i < states.length; ++i) {
			states[i].deltaPosition = b2MulAdd(states[i].deltaPosition, ctx.h, states[i].linearVelocity);
			states[i].deltaRotation = b2IntegrateRotation(states[i].deltaRotation, ctx.h * states[i].angularVelocity);
		}

		for (let c = 0; c < B2_OVERFLOW_INDEX; ++c) {
			let n = world.constraintGraph.colors[c].jointSims.length;
			if (n > 0) {
				b2SolveJointsTask(block(0, n, c), ctx, false, 0);
			}
		}
		b2SolveJoints_Overflow(ctx, false);
		emitStates("r");
		emitSims("j");
	}

	vtag("bits");
	for (let i = 0; i < jointCount; ++i) {
		vbool(b2GetBit(bits, <u32>(joints[i].index1 - 1)));
	}
	vend();
}

function emitApi(): void {
	for (let i = 0; i < jointCount; ++i)
	{
		let id = joints[i];
		vtag("a");
		vi32(b2Joint_GetType(id));
		vi32(b2Joint_GetBodyA(id).index1);
		vi32(b2Joint_GetBodyB(id).index1);
		vbool(b2Joint_GetCollideConnected(id));
		vvec(b2Joint_GetConstraintForce(id));
		let type = b2Joint_GetType(id);
		// C asserts on these for the mover and pogo joints
		if (type != b2_moverJoint && type != b2_pogoJoint)
		{
			vf32(b2Joint_GetLinearSeparation(id));
			vf32(b2Joint_GetAngularSeparation(id));
			vf32(b2Joint_GetConstraintTorque(id));
		}
		let tuning = b2Vec2();
		b2Joint_GetConstraintTuning(id, tuning.x, tuning.y);
		vf32(tuning.x);
		vf32(tuning.y);
		vf32(b2Joint_GetForceThreshold(id));
		vf32(b2Joint_GetTorqueThreshold(id));
		vvec(b2Joint_GetLocalFrameA(id).p);
		vvec(b2Joint_GetLocalFrameB(id).p);
		vf32(b2Joint_GetLocalFrameB(id).q.s);
		switch (type)
		{
			case b2_distanceJoint: {
				let r = b2Vec2();
				b2DistanceJoint_GetSpringForceRange(id, r.x, r.y);
				vf32(b2DistanceJoint_GetRestLength(id));
				vf32(b2DistanceJoint_GetLowerLimit(id));
				vf32(b2DistanceJoint_GetUpperLimit(id));
				vf32(b2DistanceJoint_GetCurrentLength(id));
				vbool(b2DistanceJoint_IsSpringEnabled(id));
				vbool(b2DistanceJoint_IsLimitEnabled(id));
				vbool(b2DistanceJoint_IsMotorEnabled(id));
				vf32(r.x);
				vf32(r.y);
				vf32(b2DistanceJoint_GetSpringHertz(id));
				vf32(b2DistanceJoint_GetSpringDampingRatio(id));
				vf32(b2DistanceJoint_GetSpringForce(id));
				vf32(b2DistanceJoint_GetTargetMotorSpeed(id));
				vf32(b2DistanceJoint_GetCurrentMotorForce(id));
				vf32(b2DistanceJoint_GetMaxMotorForce(id));
				break;
			}
			case b2_motorJoint:
				vvec(b2MotorJoint_GetLinearVelocity(id));
				vf32(b2MotorJoint_GetAngularVelocity(id));
				vf32(b2MotorJoint_GetMaxVelocityForce(id));
				vf32(b2MotorJoint_GetMaxVelocityTorque(id));
				vf32(b2MotorJoint_GetLinearHertz(id));
				vf32(b2MotorJoint_GetLinearDampingRatio(id));
				vf32(b2MotorJoint_GetAngularHertz(id));
				vf32(b2MotorJoint_GetAngularDampingRatio(id));
				vf32(b2MotorJoint_GetMaxSpringForce(id));
				vf32(b2MotorJoint_GetMaxSpringTorque(id));
				break;
			case b2_moverJoint:
				vvec(b2MoverJoint_GetLinearVelocity(id));
				vvec(b2MoverJoint_GetMaxVelocityForce(id));
				break;
			case b2_pogoJoint:
				vf32(b2PogoJoint_GetRestLength(id));
				vf32(b2PogoJoint_GetSpringHertz(id));
				vf32(b2PogoJoint_GetSpringDampingRatio(id));
				vf32(b2PogoJoint_GetCurrentLength(id));
				vf32(b2PogoJoint_GetImpulse(id));
				vf32(b2PogoJoint_GetCurrentSpeed(id));
				break;
			case b2_prismaticJoint:
				vbool(b2PrismaticJoint_IsSpringEnabled(id));
				vf32(b2PrismaticJoint_GetSpringHertz(id));
				vf32(b2PrismaticJoint_GetSpringDampingRatio(id));
				vf32(b2PrismaticJoint_GetTargetTranslation(id));
				vbool(b2PrismaticJoint_IsLimitEnabled(id));
				vf32(b2PrismaticJoint_GetLowerLimit(id));
				vf32(b2PrismaticJoint_GetUpperLimit(id));
				vbool(b2PrismaticJoint_IsMotorEnabled(id));
				vf32(b2PrismaticJoint_GetTargetMotorSpeed(id));
				vf32(b2PrismaticJoint_GetMaxMotorForce(id));
				vf32(b2PrismaticJoint_GetCurrentMotorForce(id));
				vf32(b2PrismaticJoint_GetCurrentTranslation(id));
				vf32(b2PrismaticJoint_GetCurrentSpeed(id));
				break;
			case b2_revoluteJoint:
				vbool(b2RevoluteJoint_IsSpringEnabled(id));
				vf32(b2RevoluteJoint_GetSpringHertz(id));
				vf32(b2RevoluteJoint_GetSpringDampingRatio(id));
				vf32(b2RevoluteJoint_GetTargetAngle(id));
				vbool(b2RevoluteJoint_IsLimitEnabled(id));
				vf32(b2RevoluteJoint_GetLowerLimit(id));
				vf32(b2RevoluteJoint_GetUpperLimit(id));
				vbool(b2RevoluteJoint_IsMotorEnabled(id));
				vf32(b2RevoluteJoint_GetTargetMotorSpeed(id));
				vf32(b2RevoluteJoint_GetMaxMotorTorque(id));
				vf32(b2RevoluteJoint_GetCurrentMotorTorque(id));
				vf32(b2RevoluteJoint_GetCurrentAngle(id));
				vf32(b2RevoluteJoint_GetCurrentSpeed(id));
				break;
			case b2_weldJoint:
				vf32(b2WeldJoint_GetLinearHertz(id));
				vf32(b2WeldJoint_GetLinearDampingRatio(id));
				vf32(b2WeldJoint_GetAngularHertz(id));
				vf32(b2WeldJoint_GetAngularDampingRatio(id));
				break;
			case b2_wheelJoint:
				vbool(b2WheelJoint_IsSpringEnabled(id));
				vf32(b2WheelJoint_GetSpringHertz(id));
				vf32(b2WheelJoint_GetSpringDampingRatio(id));
				vbool(b2WheelJoint_IsLimitEnabled(id));
				vf32(b2WheelJoint_GetLowerLimit(id));
				vf32(b2WheelJoint_GetUpperLimit(id));
				vbool(b2WheelJoint_IsMotorEnabled(id));
				vf32(b2WheelJoint_GetTargetMotorSpeed(id));
				vf32(b2WheelJoint_GetMaxMotorTorque(id));
				vf32(b2WheelJoint_GetCurrentMotorTorque(id));
				break;
			default:
				break;
		}
		vend();
	}}

function setters(): void {
	b2DistanceJoint_SetRestLength(joints[0], 1.7);
	b2DistanceJoint_SetLimits(joints[1], 1.3, 0.8);
	b2DistanceJoint_SetSpringForceRange(joints[1], -30.0, 20.0);
	b2DistanceJoint_SetSpringHertz(joints[1], 4.0);
	b2DistanceJoint_SetSpringDampingRatio(joints[1], 0.5);
	b2DistanceJoint_EnableMotor(joints[1], false);
	b2DistanceJoint_SetTargetMotorSpeed(joints[1], 0.2);
	b2DistanceJoint_SetMaxMotorForce(joints[1], 3.0);
	b2DistanceJoint_EnableLimit(joints[0], true);
	b2DistanceJoint_EnableSpring(joints[0], true);
	b2MotorJoint_SetLinearVelocity(joints[2], b2Vec2(-0.3, 0.1));
	b2MotorJoint_SetAngularVelocity(joints[2], -0.4);
	b2MotorJoint_SetMaxVelocityForce(joints[2], 25.0);
	b2MotorJoint_SetMaxVelocityTorque(joints[2], 5.0);
	b2MotorJoint_SetLinearHertz(joints[2], 3.0);
	b2MotorJoint_SetLinearDampingRatio(joints[2], 0.9);
	b2MotorJoint_SetAngularHertz(joints[2], 2.0);
	b2MotorJoint_SetAngularDampingRatio(joints[2], 0.3);
	b2MotorJoint_SetMaxSpringForce(joints[2], -1.0);
	b2MotorJoint_SetMaxSpringTorque(joints[2], 7.0);
	b2MoverJoint_SetLinearVelocity(joints[3], b2Vec2(-1.0, 0.0));
	b2MoverJoint_SetMaxVelocityForce(joints[3], b2Vec2(4.0, 4.0));
	b2PogoJoint_SetRestLength(joints[4], 2.5);
	b2PogoJoint_SetSpringHertz(joints[4], 6.0);
	b2PogoJoint_SetSpringDampingRatio(joints[4], 0.9);
	b2PrismaticJoint_EnableSpring(joints[5], false);
	b2PrismaticJoint_SetSpringHertz(joints[5], 1.0);
	b2PrismaticJoint_SetSpringDampingRatio(joints[5], 0.1);
	b2PrismaticJoint_SetTargetTranslation(joints[5], -0.1);
	b2PrismaticJoint_EnableLimit(joints[5], false);
	b2PrismaticJoint_SetLimits(joints[5], -0.5, 0.5);
	b2PrismaticJoint_EnableMotor(joints[5], false);
	b2PrismaticJoint_SetTargetMotorSpeed(joints[5], 0.9);
	b2PrismaticJoint_SetMaxMotorForce(joints[5], 2.0);
	b2RevoluteJoint_EnableSpring(joints[6], false);
	b2RevoluteJoint_SetSpringHertz(joints[6], 2.0);
	b2RevoluteJoint_SetSpringDampingRatio(joints[6], 0.6);
	b2RevoluteJoint_SetTargetAngle(joints[6], -0.2);
	b2RevoluteJoint_EnableLimit(joints[6], false);
	b2RevoluteJoint_SetLimits(joints[6], -4.0, 3.5);
	b2RevoluteJoint_EnableMotor(joints[6], false);
	b2RevoluteJoint_SetTargetMotorSpeed(joints[6], -0.8);
	b2RevoluteJoint_SetMaxMotorTorque(joints[6], 1.0);
	b2WeldJoint_SetLinearHertz(joints[7], 2.0);
	b2WeldJoint_SetLinearDampingRatio(joints[7], 0.2);
	b2WeldJoint_SetAngularHertz(joints[7], 1.0);
	b2WeldJoint_SetAngularDampingRatio(joints[7], 0.3);
	b2WheelJoint_EnableSpring(joints[9], false);
	b2WheelJoint_SetSpringHertz(joints[9], 3.0);
	b2WheelJoint_SetSpringDampingRatio(joints[9], 0.2);
	b2WheelJoint_EnableLimit(joints[9], false);
	b2WheelJoint_SetLimits(joints[9], -0.2, 0.2);
	b2WheelJoint_EnableMotor(joints[9], false);
	b2WheelJoint_SetTargetMotorSpeed(joints[9], 2.0);
	b2WheelJoint_SetMaxMotorTorque(joints[9], 4.0);
	b2Joint_SetConstraintTuning(joints[6], 30.0, 1.0);
	b2Joint_SetForceThreshold(joints[5], 3.0);
	b2Joint_SetTorqueThreshold(joints[5], 2.0);
	let frame = b2Transform(b2Vec2(0.1, 0.2), b2MakeRot(0.5));
	b2Joint_SetLocalFrameA(joints[8], frame);
	b2Joint_SetLocalFrameB(joints[8], frame);
	b2Joint_SetCollideConnected(joints[10], true);
	b2Joint_SetUserData(joints[10], 42);
	vtag("ud");
	vi32(<i32>b2Joint_GetUserData(joints[10]));
	vend();}

function silentLog(message: string): void {}

export function run(): void {
	b2SetLogCallback(silentLog);
	build();
	emitApi();
	stepJoints(true);
	emitApi();
	setters();
	emitApi();
	stepJoints(true);
	emitApi();
	stepJoints(false);
	emitApi();

	b2Joint_SetCollideConnected(joints[10], false);
	b2DestroyJoint(joints[1]);
	b2DestroyJoint(joints[10]);
	jointCount = 0;
	for (let i = 0; i < 11; ++i) {
		if (i != 1 && i != 10) {
			joints[jointCount++] = joints[i];
		}
	}
	b2Body_SetType(bodies[8], b2_staticBody);
	stepJoints(true);
	emitApi();

	let world = b2GetWorldFromId(worldId);
	vtag("end");
	vi32(world.joints.length);
	vi32(b2GetIdCount(world.jointIdPool));
	vend();
}
