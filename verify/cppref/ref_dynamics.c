// Phase C reference generator: builds dynamics scenes with the public box2d
// API and steps them, dumping per-step body states (hex f32) plus FNV-1a
// hashes of contact/island/solver-set/graph-color bookkeeping and event
// buffers. The AS driver (_verifyC.as) replays identical scenes; every line
// compares bit-exactly.
//
// Build with the pinned determinism flags (see run.sh):
//   clang -O2 -ffp-contract=off -DNDEBUG -DBOX2D_DISABLE_SIMD

#include "box2d/box2d.h"

// internals for bookkeeping hashes
#include "body.h"
#include "contact.h"
#include "constraint_graph.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"

#include <stdio.h>
#include <stdint.h>

static uint32_t F(float x)
{
	union
	{
		float f;
		uint32_t u;
	} c;
	c.f = x;
	return c.u;
}

// FNV-1a 64 over u32 words
static uint64_t hashInit(void)
{
	return 14695981039346656037ull;
}

static uint64_t hashU32(uint64_t h, uint32_t w)
{
	for (int i = 0; i < 4; ++i)
	{
		h ^= (w >> (8 * i)) & 0xff;
		h *= 1099511628211ull;
	}
	return h;
}

static uint64_t hashI32(uint64_t h, int w)
{
	return hashU32(h, (uint32_t)w);
}

static uint64_t hashF32(uint64_t h, float x)
{
	return hashU32(h, F(x));
}

// ---------------------------------------------------------------------------
// per-step dump
// ---------------------------------------------------------------------------

static void dumpStep(b2WorldId worldId, int step)
{
	b2World* world = b2GetWorldFromId(worldId);

	// count live bodies
	int liveBodies = 0;
	for (int i = 0; i < world->bodies.count; ++i)
	{
		if (world->bodies.data[i].id != B2_NULL_INDEX)
		{
			liveBodies += 1;
		}
	}

	printf("S %d %d\n", step, liveBodies);

	// body states in id order
	uint64_t bookHash = hashInit();
	for (int i = 0; i < world->bodies.count; ++i)
	{
		b2Body* body = world->bodies.data + i;
		if (body->id == B2_NULL_INDEX)
		{
			continue;
		}

		b2SolverSet* set = b2SolverSetArray_Get(&world->solverSets, body->setIndex);
		b2BodySim* sim = b2BodySimArray_Get(&set->bodySims, body->localIndex);

		float vx = 0.0f, vy = 0.0f, w = 0.0f;
		if (body->setIndex == b2_awakeSet)
		{
			b2BodyState* state = b2BodyStateArray_Get(&set->bodyStates, body->localIndex);
			vx = state->linearVelocity.x;
			vy = state->linearVelocity.y;
			w = state->angularVelocity;
		}

		printf("B %d %d %08x %08x %08x %08x %08x %08x %08x\n", i, body->setIndex, F(sim->transform.p.x),
			   F(sim->transform.p.y), F(sim->transform.q.c), F(sim->transform.q.s), F(vx), F(vy), F(w));

		bookHash = hashI32(bookHash, body->id);
		bookHash = hashI32(bookHash, body->setIndex);
		bookHash = hashI32(bookHash, body->localIndex);
		bookHash = hashI32(bookHash, body->islandId);
		bookHash = hashI32(bookHash, body->islandPrev);
		bookHash = hashI32(bookHash, body->islandNext);
		bookHash = hashF32(bookHash, body->sleepTime);
		bookHash = hashF32(bookHash, body->mass);
		bookHash = hashF32(bookHash, body->inertia);
		bookHash = hashF32(bookHash, sim->center.x);
		bookHash = hashF32(bookHash, sim->center.y);
	}

	// contacts by id order
	int liveContacts = 0;
	int touching = 0;
	uint64_t contactHash = hashInit();
	for (int i = 0; i < world->contacts.count; ++i)
	{
		b2Contact* contact = world->contacts.data + i;
		if (contact->contactId == B2_NULL_INDEX)
		{
			continue;
		}

		liveContacts += 1;
		if (contact->flags & b2_contactTouchingFlag)
		{
			touching += 1;
		}

		contactHash = hashI32(contactHash, contact->contactId);
		contactHash = hashI32(contactHash, contact->setIndex);
		contactHash = hashI32(contactHash, contact->colorIndex);
		contactHash = hashI32(contactHash, contact->shapeIdA);
		contactHash = hashI32(contactHash, contact->shapeIdB);
		contactHash = hashU32(contactHash, contact->flags & (b2_contactTouchingFlag | b2_contactEnableContactEvents));
		contactHash = hashI32(contactHash, contact->islandId);

		b2ContactSim* sim = b2GetContactSim(world, contact);
		b2Manifold* m = &sim->manifold;
		contactHash = hashI32(contactHash, m->pointCount);
		if (m->pointCount > 0)
		{
			contactHash = hashF32(contactHash, m->normal.x);
			contactHash = hashF32(contactHash, m->normal.y);
			contactHash = hashF32(contactHash, m->rollingImpulse);
			for (int k = 0; k < m->pointCount; ++k)
			{
				b2ManifoldPoint* mp = m->points + k;
				contactHash = hashU32(contactHash, mp->id);
				contactHash = hashF32(contactHash, mp->anchorA.x);
				contactHash = hashF32(contactHash, mp->anchorA.y);
				contactHash = hashF32(contactHash, mp->anchorB.x);
				contactHash = hashF32(contactHash, mp->anchorB.y);
				contactHash = hashF32(contactHash, mp->separation);
				contactHash = hashF32(contactHash, mp->normalImpulse);
				contactHash = hashF32(contactHash, mp->tangentImpulse);
				contactHash = hashF32(contactHash, mp->totalNormalImpulse);
				contactHash = hashF32(contactHash, mp->normalVelocity);
			}
		}
	}

	printf("C %d %d %016llx\n", liveContacts, touching, (unsigned long long)contactHash);

	// islands by id order
	uint64_t islandHash = hashInit();
	for (int i = 0; i < world->islands.count; ++i)
	{
		b2Island* island = world->islands.data + i;
		if (island->islandId == B2_NULL_INDEX || island->setIndex == B2_NULL_INDEX)
		{
			continue;
		}

		islandHash = hashI32(islandHash, island->islandId);
		islandHash = hashI32(islandHash, island->setIndex);
		islandHash = hashI32(islandHash, island->localIndex);
		islandHash = hashI32(islandHash, island->headBody);
		islandHash = hashI32(islandHash, island->tailBody);
		islandHash = hashI32(islandHash, island->bodyCount);
		islandHash = hashI32(islandHash, island->headContact);
		islandHash = hashI32(islandHash, island->tailContact);
		islandHash = hashI32(islandHash, island->contactCount);
		islandHash = hashI32(islandHash, island->headJoint);
		islandHash = hashI32(islandHash, island->tailJoint);
		islandHash = hashI32(islandHash, island->jointCount);
		islandHash = hashI32(islandHash, island->constraintRemoveCount);
	}

	// solver sets
	uint64_t setHash = hashInit();
	for (int i = 0; i < world->solverSets.count; ++i)
	{
		b2SolverSet* set = world->solverSets.data + i;
		if (set->setIndex == B2_NULL_INDEX)
		{
			continue;
		}

		setHash = hashI32(setHash, set->setIndex);
		setHash = hashI32(setHash, set->bodySims.count);
		setHash = hashI32(setHash, set->contactSims.count);
		setHash = hashI32(setHash, set->jointSims.count);
		setHash = hashI32(setHash, set->islandSims.count);
		for (int k = 0; k < set->bodySims.count; ++k)
		{
			setHash = hashI32(setHash, set->bodySims.data[k].bodyId);
		}
		for (int k = 0; k < set->contactSims.count; ++k)
		{
			setHash = hashI32(setHash, set->contactSims.data[k].contactId);
		}
		for (int k = 0; k < set->jointSims.count; ++k)
		{
			setHash = hashI32(setHash, set->jointSims.data[k].jointId);
		}
		for (int k = 0; k < set->islandSims.count; ++k)
		{
			setHash = hashI32(setHash, set->islandSims.data[k].islandId);
		}
	}

	// graph colors
	uint64_t colorHash = hashInit();
	for (int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i)
	{
		b2GraphColor* color = world->constraintGraph.colors + i;
		colorHash = hashI32(colorHash, color->contactSims.count);
		colorHash = hashI32(colorHash, color->jointSims.count);
		for (int k = 0; k < color->contactSims.count; ++k)
		{
			colorHash = hashI32(colorHash, color->contactSims.data[k].contactId);
		}
		for (int k = 0; k < color->jointSims.count; ++k)
		{
			colorHash = hashI32(colorHash, color->jointSims.data[k].jointId);
		}
	}

	printf("I %016llx %016llx %016llx %016llx\n", (unsigned long long)islandHash, (unsigned long long)setHash,
		   (unsigned long long)colorHash, (unsigned long long)bookHash);

	// events
	b2BodyEvents bodyEvents = b2World_GetBodyEvents(worldId);
	b2ContactEvents contactEvents = b2World_GetContactEvents(worldId);
	b2JointEvents jointEvents = b2World_GetJointEvents(worldId);

	uint64_t eventHash = hashInit();
	for (int i = 0; i < bodyEvents.moveCount; ++i)
	{
		b2BodyMoveEvent* ev = bodyEvents.moveEvents + i;
		eventHash = hashI32(eventHash, ev->bodyId.index1);
		eventHash = hashF32(eventHash, ev->transform.p.x);
		eventHash = hashF32(eventHash, ev->transform.p.y);
		eventHash = hashF32(eventHash, ev->transform.q.c);
		eventHash = hashF32(eventHash, ev->transform.q.s);
		eventHash = hashI32(eventHash, ev->fellAsleep ? 1 : 0);
	}
	for (int i = 0; i < contactEvents.beginCount; ++i)
	{
		b2ContactBeginTouchEvent* ev = contactEvents.beginEvents + i;
		eventHash = hashI32(eventHash, ev->shapeIdA.index1);
		eventHash = hashI32(eventHash, ev->shapeIdB.index1);
		eventHash = hashI32(eventHash, ev->contactId.index1);
	}
	for (int i = 0; i < contactEvents.endCount; ++i)
	{
		b2ContactEndTouchEvent* ev = contactEvents.endEvents + i;
		eventHash = hashI32(eventHash, ev->shapeIdA.index1);
		eventHash = hashI32(eventHash, ev->shapeIdB.index1);
		eventHash = hashI32(eventHash, ev->contactId.index1);
	}
	for (int i = 0; i < contactEvents.hitCount; ++i)
	{
		b2ContactHitEvent* ev = contactEvents.hitEvents + i;
		eventHash = hashI32(eventHash, ev->shapeIdA.index1);
		eventHash = hashI32(eventHash, ev->shapeIdB.index1);
		eventHash = hashF32(eventHash, ev->point.x);
		eventHash = hashF32(eventHash, ev->point.y);
		eventHash = hashF32(eventHash, ev->normal.x);
		eventHash = hashF32(eventHash, ev->normal.y);
		eventHash = hashF32(eventHash, ev->approachSpeed);
	}
	for (int i = 0; i < jointEvents.count; ++i)
	{
		eventHash = hashI32(eventHash, jointEvents.jointEvents[i].jointId.index1);
	}

	printf("E %d %d %d %d %d %016llx\n", bodyEvents.moveCount, contactEvents.beginCount, contactEvents.endCount,
		   contactEvents.hitCount, jointEvents.count, (unsigned long long)eventHash);
}

// ---------------------------------------------------------------------------
// scene helpers (mirrored 1:1 in _verifyC.as)
// ---------------------------------------------------------------------------

static uint32_t g_rng = 42;

static uint32_t nextRand(void)
{
	uint32_t x = g_rng;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g_rng = x;
	return x;
}

// [0,1) with 24-bit mantissa, exact in f32
static float randFloat(void)
{
	return (float)(nextRand() >> 8) * (1.0f / 16777216.0f);
}

static b2BodyId makeGround(b2WorldId worldId, float halfWidth)
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = (b2Vec2){ 0.0f, -1.0f };
	b2BodyId groundId = b2CreateBody(worldId, &bodyDef);

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox(halfWidth, 1.0f);
	b2CreatePolygonShape(groundId, &shapeDef, &box);
	return groundId;
}

static void buildPyramid(b2WorldId worldId, int rows)
{
	makeGround(worldId, 40.0f);

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = 0.6f;

	float h = 0.5f;
	b2Polygon box = b2MakeBox(h, h);

	float shift = 1.0f * h;

	for (int i = 0; i < rows; ++i)
	{
		float y = (2.0f * i + 1.0f) * shift;

		for (int j = i; j < rows; ++j)
		{
			float x = (i + 1.0f) * shift + 2.0f * (j - i) * shift - h * rows;

			bodyDef.position = (b2Vec2){ x, y };
			b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);
			b2CreatePolygonShape(bodyId, &shapeDef, &box);
		}
	}
}

static void buildRain(b2WorldId worldId, int count)
{
	// container: floor plus two walls
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = (b2Vec2){ 0.0f, -1.0f };
	b2BodyId groundId = b2CreateBody(worldId, &bodyDef);
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon floor = b2MakeBox(10.0f, 1.0f);
	b2CreatePolygonShape(groundId, &shapeDef, &floor);

	bodyDef.position = (b2Vec2){ -10.0f, 5.0f };
	b2BodyId leftId = b2CreateBody(worldId, &bodyDef);
	b2Polygon wall = b2MakeBox(1.0f, 6.0f);
	b2CreatePolygonShape(leftId, &shapeDef, &wall);

	bodyDef.position = (b2Vec2){ 10.0f, 5.0f };
	b2BodyId rightId = b2CreateBody(worldId, &bodyDef);
	b2CreatePolygonShape(rightId, &shapeDef, &wall);

	// falling shapes
	b2BodyDef dynDef = b2DefaultBodyDef();
	dynDef.type = b2_dynamicBody;

	for (int i = 0; i < count; ++i)
	{
		float x = -8.0f + 16.0f * randFloat();
		float y = 2.0f + 20.0f * randFloat();
		dynDef.position = (b2Vec2){ x, y };
		dynDef.angularVelocity = 2.0f * randFloat() - 1.0f;
		dynDef.linearVelocity = (b2Vec2){ 2.0f * randFloat() - 1.0f, 0.0f };
		b2BodyId bodyId = b2CreateBody(worldId, &dynDef);

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.material.friction = 0.3f;
		sd.material.restitution = 0.1f;

		int kind = i % 3;
		if (kind == 0)
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.35f };
			b2CreateCircleShape(bodyId, &sd, &circle);
		}
		else if (kind == 1)
		{
			b2Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.25f };
			b2CreateCapsuleShape(bodyId, &sd, &capsule);
		}
		else
		{
			b2Polygon box = b2MakeBox(0.35f, 0.35f);
			b2CreatePolygonShape(bodyId, &sd, &box);
		}
	}
}

static b2BodyId makeDynamicBox(b2WorldId worldId, float x, float y, float hx, float hy)
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Vec2){ x, y };
	b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox(hx, hy);
	b2CreatePolygonShape(bodyId, &shapeDef, &box);
	return bodyId;
}

static b2BodyId g_bodies[512];
static int g_bodyCount;
static b2JointId g_joints[64];
static int g_jointCount;

static void rememberBody(b2BodyId id)
{
	g_bodies[g_bodyCount++] = id;
}

static void rememberJoint(b2JointId id)
{
	g_joints[g_jointCount++] = id;
}

static void buildDistanceScene(b2WorldId worldId)
{
	b2BodyId groundId = makeGround(worldId, 20.0f);

	b2BodyId prev = groundId;
	for (int i = 0; i < 3; ++i)
	{
		b2BodyId bodyId = makeDynamicBox(worldId, 1.0f * i + 1.0f, 6.0f - 1.0f * i, 0.25f, 0.25f);
		rememberBody(bodyId);

		b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
		jointDef.base.bodyIdA = prev;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ 1.0f * i, i == 0 ? 8.0f : 0.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.25f };
		jointDef.length = 1.5f;

		if (i == 0)
		{
			jointDef.enableSpring = true;
			jointDef.hertz = 3.0f;
			jointDef.dampingRatio = 0.5f;
		}
		else if (i == 1)
		{
			jointDef.enableLimit = true;
			jointDef.minLength = 1.0f;
			jointDef.maxLength = 2.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 2.0f;
			jointDef.dampingRatio = 0.3f;
		}
		else
		{
			jointDef.enableMotor = true;
			jointDef.maxMotorForce = 50.0f;
			jointDef.motorSpeed = 0.5f;
		}

		rememberJoint(b2CreateDistanceJoint(worldId, &jointDef));
		prev = bodyId;
	}
}

static void buildRevoluteScene(b2WorldId worldId)
{
	b2BodyId groundId = makeGround(worldId, 20.0f);

	b2BodyId prev = groundId;
	for (int i = 0; i < 3; ++i)
	{
		b2BodyId bodyId = makeDynamicBox(worldId, 1.0f * (i + 1), 6.0f, 0.5f, 0.125f);
		rememberBody(bodyId);

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = prev;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = i == 0 ? (b2Vec2){ 0.5f, 7.0f } : (b2Vec2){ 0.5f, 0.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };

		if (i == 0)
		{
			jointDef.enableLimit = true;
			jointDef.lowerAngle = -0.5f;
			jointDef.upperAngle = 0.5f;
		}
		else if (i == 1)
		{
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 20.0f;
			jointDef.motorSpeed = 1.0f;
		}
		else
		{
			jointDef.enableSpring = true;
			jointDef.hertz = 2.0f;
			jointDef.dampingRatio = 0.2f;
			jointDef.targetAngle = 0.25f;
		}

		rememberJoint(b2CreateRevoluteJoint(worldId, &jointDef));
		prev = bodyId;
	}
}

static void buildPrismaticScene(b2WorldId worldId)
{
	b2BodyId groundId = makeGround(worldId, 20.0f);

	// slider with limit
	{
		b2BodyId bodyId = makeDynamicBox(worldId, 0.0f, 2.0f, 0.5f, 0.5f);
		rememberBody(bodyId);

		b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ 0.0f, 3.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.0f };
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -2.0f;
		jointDef.upperTranslation = 2.0f;
		jointDef.enableMotor = true;
		jointDef.maxMotorForce = 30.0f;
		jointDef.motorSpeed = 1.5f;
		rememberJoint(b2CreatePrismaticJoint(worldId, &jointDef));
	}

	// vertical spring slider
	{
		b2BodyId bodyId = makeDynamicBox(worldId, 5.0f, 4.0f, 0.5f, 0.5f);
		rememberBody(bodyId);

		b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ 5.0f, 3.0f };
		jointDef.base.localFrameA.q = b2MakeRot(0.5f * 3.14159265359f);
		jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.0f };
		jointDef.base.localFrameB.q = b2MakeRot(0.5f * 3.14159265359f);
		jointDef.enableSpring = true;
		jointDef.hertz = 2.0f;
		jointDef.dampingRatio = 0.3f;
		jointDef.targetTranslation = 1.0f;
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -3.0f;
		jointDef.upperTranslation = 3.0f;
		rememberJoint(b2CreatePrismaticJoint(worldId, &jointDef));
	}
}

static void buildWeldScene(b2WorldId worldId)
{
	b2BodyId groundId = makeGround(worldId, 20.0f);

	// cantilever of 4, mixed soft and rigid welds
	b2BodyId prev = groundId;
	for (int i = 0; i < 4; ++i)
	{
		b2BodyId bodyId = makeDynamicBox(worldId, 1.0f * (i + 1), 6.0f, 0.5f, 0.125f);
		rememberBody(bodyId);

		b2WeldJointDef jointDef = b2DefaultWeldJointDef();
		jointDef.base.bodyIdA = prev;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = i == 0 ? (b2Vec2){ 0.5f, 7.0f } : (b2Vec2){ 0.5f, 0.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };

		if (i % 2 == 0)
		{
			jointDef.linearHertz = 5.0f;
			jointDef.linearDampingRatio = 0.7f;
			jointDef.angularHertz = 1.0f;
			jointDef.angularDampingRatio = 0.5f;
		}

		rememberJoint(b2CreateWeldJoint(worldId, &jointDef));
		prev = bodyId;
	}
}

static void buildMotorScene(b2WorldId worldId)
{
	makeGround(worldId, 20.0f);

	b2BodyId bodyId = makeDynamicBox(worldId, 0.0f, 4.0f, 0.5f, 0.5f);
	rememberBody(bodyId);

	b2MotorJointDef jointDef = b2DefaultMotorJointDef();
	b2BodyDef anchorDef = b2DefaultBodyDef();
	anchorDef.position = (b2Vec2){ 0.0f, 4.0f };
	b2BodyId anchorId = b2CreateBody(worldId, &anchorDef);

	jointDef.base.bodyIdA = anchorId;
	jointDef.base.bodyIdB = bodyId;
	jointDef.linearVelocity = (b2Vec2){ 1.0f, 0.0f };
	jointDef.maxVelocityForce = 20.0f;
	jointDef.angularVelocity = 2.0f;
	jointDef.maxVelocityTorque = 5.0f;
	jointDef.linearHertz = 1.0f;
	jointDef.linearDampingRatio = 0.5f;
	jointDef.maxSpringForce = 50.0f;
	jointDef.angularHertz = 1.0f;
	jointDef.angularDampingRatio = 0.5f;
	jointDef.maxSpringTorque = 20.0f;
	rememberJoint(b2CreateMotorJoint(worldId, &jointDef));
}

static void buildWheelScene(b2WorldId worldId)
{
	b2BodyId groundId = makeGround(worldId, 40.0f);

	// bump
	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon bump = b2MakeOffsetBox(1.0f, 0.25f, (b2Vec2){ 4.0f, 1.0f }, b2MakeRot(0.0f));
		b2CreatePolygonShape(groundId, &shapeDef, &bump);
	}

	// chassis
	b2BodyId chassisId = makeDynamicBox(worldId, 0.0f, 1.0f, 1.0f, 0.25f);
	rememberBody(chassisId);

	for (int i = 0; i < 2; ++i)
	{
		float x = i == 0 ? -0.75f : 0.75f;

		b2BodyDef wheelDef = b2DefaultBodyDef();
		wheelDef.type = b2_dynamicBody;
		wheelDef.position = (b2Vec2){ x, 0.5f };
		wheelDef.allowFastRotation = true;
		b2BodyId wheelId = b2CreateBody(worldId, &wheelDef);
		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.material.friction = 0.8f;
		b2Circle circle = { { 0.0f, 0.0f }, 0.3f };
		b2CreateCircleShape(wheelId, &sd, &circle);
		rememberBody(wheelId);

		b2WheelJointDef jointDef = b2DefaultWheelJointDef();
		jointDef.base.bodyIdA = chassisId;
		jointDef.base.bodyIdB = wheelId;
		jointDef.base.localFrameA.p = (b2Vec2){ x, -0.5f };
		jointDef.base.localFrameA.q = b2MakeRot(0.5f * 3.14159265359f);
		jointDef.base.localFrameB.q = b2MakeRot(0.5f * 3.14159265359f);
		jointDef.enableSpring = true;
		jointDef.hertz = 4.0f;
		jointDef.dampingRatio = 0.7f;
		jointDef.enableLimit = true;
		jointDef.lowerTranslation = -0.25f;
		jointDef.upperTranslation = 0.25f;

		if (i == 1)
		{
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = -10.0f;
		}

		rememberJoint(b2CreateWheelJoint(worldId, &jointDef));
	}
}

static void buildChurnScene(b2WorldId worldId)
{
	buildPyramid(worldId, 6);

	// remember dynamic bodies for churn (bodies are ids 1..21 in creation order)
	// pyramid of 6 rows = 21 boxes, body id 0 is the ground
}

static void buildBulletScene(b2WorldId worldId)
{
	makeGround(worldId, 40.0f);

	// thin wall
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Vec2){ 6.0f, 3.0f };
		b2BodyId wallId = b2CreateBody(worldId, &bodyDef);
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon wall = b2MakeBox(0.05f, 3.0f);
		b2CreatePolygonShape(wallId, &shapeDef, &wall);
	}

	// bullet circle
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Vec2){ -5.0f, 3.0f };
		bodyDef.isBullet = true;
		bodyDef.linearVelocity = (b2Vec2){ 80.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;
		b2BodyId bulletId = b2CreateBody(worldId, &bodyDef);
		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 5.0f;
		b2Circle circle = { { 0.0f, 0.0f }, 0.1f };
		b2CreateCircleShape(bulletId, &sd, &circle);
	}

	// fast non-bullet box
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Vec2){ -5.0f, 1.0f };
		bodyDef.linearVelocity = (b2Vec2){ 60.0f, 0.0f };
		b2BodyId fastId = b2CreateBody(worldId, &bodyDef);
		b2ShapeDef sd = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox(0.25f, 0.25f);
		b2CreatePolygonShape(fastId, &sd, &box);
	}
}

// churn actions between steps, mirrored in AS
static void churnAction(b2WorldId worldId, int step)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_UNUSED(world);

	if (step == 60)
	{
		// destroy a mid-pyramid body (id 8, generation matches first alloc)
		b2BodyId id = { 9, (uint16_t)(worldId.index1 - 1), 1 };
		if (b2Body_IsValid(id))
		{
			b2DestroyBody(id);
		}
	}
	else if (step == 90)
	{
		// joint two pyramid boxes
		b2BodyId a = { 3, (uint16_t)(worldId.index1 - 1), 1 };
		b2BodyId c = { 5, (uint16_t)(worldId.index1 - 1), 1 };
		if (b2Body_IsValid(a) && b2Body_IsValid(c))
		{
			b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
			jointDef.base.bodyIdA = a;
			jointDef.base.bodyIdB = c;
			jointDef.length = 2.0f;
			g_joints[g_jointCount++] = b2CreateDistanceJoint(worldId, &jointDef);
		}
	}
	else if (step == 120)
	{
		// destroy a base body
		b2BodyId id = { 2, (uint16_t)(worldId.index1 - 1), 1 };
		if (b2Body_IsValid(id))
		{
			b2DestroyBody(id);
		}
	}
	else if (step == 150)
	{
		// drop a new box on top
		makeDynamicBox(worldId, 0.0f, 9.0f, 0.5f, 0.5f);
	}
	else if (step == 180 && g_jointCount > 0)
	{
		b2JointId jid = g_joints[g_jointCount - 1];
		if (b2Joint_IsValid(jid))
		{
			b2DestroyJoint(jid, true);
			g_jointCount -= 1;
		}
	}
	else if (step == 210)
	{
		// force sleep on body 4's island if awake
		b2BodyId id = { 5, (uint16_t)(worldId.index1 - 1), 1 };
		if (b2Body_IsValid(id) && b2Body_IsAwake(id))
		{
			b2Body_SetAwake(id, false);
		}
	}
	else if (step == 240)
	{
		b2BodyId id = { 5, (uint16_t)(worldId.index1 - 1), 1 };
		if (b2Body_IsValid(id))
		{
			b2Body_SetAwake(id, true);
		}
	}
}

static void runScene(int sceneId, int steps)
{
	g_rng = 42;
	g_bodyCount = 0;
	g_jointCount = 0;

	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld(&worldDef);

	switch (sceneId)
	{
		case 0:
			buildPyramid(worldId, 5);
			break;
		case 1:
			buildPyramid(worldId, 10);
			break;
		case 2:
			buildPyramid(worldId, 15);
			break;
		case 3:
			buildRain(worldId, 30);
			break;
		case 4:
			buildRain(worldId, 80);
			break;
		case 5:
			buildDistanceScene(worldId);
			break;
		case 6:
			buildRevoluteScene(worldId);
			break;
		case 7:
			buildPrismaticScene(worldId);
			break;
		case 8:
			buildWeldScene(worldId);
			break;
		case 9:
			buildMotorScene(worldId);
			break;
		case 10:
			buildWheelScene(worldId);
			break;
		case 11:
			buildChurnScene(worldId);
			break;
		case 12:
			buildBulletScene(worldId);
			break;
		default:
			break;
	}

	printf("SCENE %d %d\n", sceneId, steps);

	float dt = 1.0f / 60.0f;
	for (int step = 0; step < steps; ++step)
	{
		if (sceneId == 11)
		{
			churnAction(worldId, step);
		}

		b2World_Step(worldId, dt, 4);
		dumpStep(worldId, step);
	}

	b2DestroyWorld(worldId);
}

int main(void)
{
	runScene(0, 300);
	runScene(1, 300);
	runScene(2, 320);
	runScene(3, 240);
	runScene(4, 300);
	runScene(5, 240);
	runScene(6, 240);
	runScene(7, 240);
	runScene(8, 240);
	runScene(9, 240);
	runScene(10, 300);
	runScene(11, 300);
	runScene(12, 120);

	// determinism double-run: same scene twice in one process
	runScene(0, 300);

	return 0;
}
