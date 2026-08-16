// Pyramid bench reference: the exact driver.luau scene, printing the f64
// checksum the AS bench must reproduce bit-exactly.
#include "box2d/box2d.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){0.0f, -10.0f};
	worldDef.enableSleep = false;
	b2WorldId worldId = b2CreateWorld(&worldDef);

	b2BodyDef groundDef = b2DefaultBodyDef();
	groundDef.position = (b2Vec2){0.0f, -1.0f};
	b2BodyId groundId = b2CreateBody(worldId, &groundDef);
	b2Polygon groundBox = b2MakeBox(50.0f, 1.0f);
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);

	enum
	{
		kRows = 5,
		kBodies = 15,
		kSteps = 240
	};
	b2BodyId bodies[kBodies];
	int count = 0;
	b2Polygon box = b2MakeBox(0.5f, 0.5f);
	for (int row = 1; row <= kRows; row++)
	{
		int cols = kRows - row + 1;
		for (int col = 1; col <= cols; col++)
		{
			b2BodyDef def = b2DefaultBodyDef();
			def.type = b2_dynamicBody;
			def.position =
				(b2Vec2){(float)(col - 1) * 1.1f - (float)(cols - 1) * 0.55f,
						 (float)row * 1.1f + 0.5f};
			b2BodyId id = b2CreateBody(worldId, &def);
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape(id, &shapeDef, &box);
			bodies[count++] = id;
		}
	}

	for (int i = 0; i < kSteps; i++)
	{
		b2World_Step(worldId, 1.0f / 60.0f, 4);
	}

	double sum = 0.0;
	for (int i = 0; i < count; i++)
	{
		b2Vec2 p = b2Body_GetPosition(bodies[i]);
		sum += (double)p.x + (double)p.y * 3.0;
	}
	b2DestroyWorld(worldId);

	uint64_t bits;
	memcpy(&bits, &sum, sizeof bits);
	printf("BOX2D-BENCH-REF: pyramid%d | %d bodies | %.17g | bits %llx\n",
		   kSteps,
		   count,
		   sum,
		   (unsigned long long)bits);
	return 0;
}
