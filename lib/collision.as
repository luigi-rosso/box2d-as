// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of the types of include/box2d/collision.h, plus the tree types of
// src/dynamic_tree.h. Function bodies live in the modules that define them in C.

import { b2AABB, b2Plane, b2Rot, b2Transform, b2Vec2 } from "./math_functions";

/**
 * @defgroup geometry Geometry
 * @brief Geometry types and algorithms
 */

/// The maximum number of vertices on a convex polygon. Changing this affects performance even if
/// you don't use more vertices.
// rasc takes only a literal FixedArray length, so the arrays below spell it 8.
export const B2_MAX_POLYGON_VERTICES: i32 = 8;

/// Low level ray cast input data
export struct b2RayCastInput {
	/// Start point of the ray cast
	origin: b2Vec2 = b2Vec2();

	/// Translation of the ray cast
	translation: b2Vec2 = b2Vec2();

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction: f32 = 0;

	constructor(origin: b2Vec2, translation: b2Vec2, maxFraction: f32) {
		this.origin = origin;
		this.translation = translation;
		this.maxFraction = maxFraction;
	}
}

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape. You can provide
/// between 1 and B2_MAX_POLYGON_VERTICES points and a radius.
export struct b2ShapeProxy {
	/// The point cloud
	points: FixedArray<b2Vec2, 8>;

	/// The number of points. Must be greater than 0.
	count: i32 = 0;

	/// The external radius of the point cloud. May be zero.
	radius: f32 = 0;
}

/// Low level shape cast input in generic form. This allows casting an arbitrary point cloud wrapped
/// with a radius. For example, a circle is a single point with a non-zero radius. A capsule is two
/// points with a non-zero radius. A box is four points with a zero radius.
export struct b2ShapeCastInput {
	/// A generic shape
	proxy: b2ShapeProxy = b2ShapeProxy();

	/// The translation of the shape cast
	translation: b2Vec2 = b2Vec2();

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction: f32 = 0;

	/// Allow shape cast to encroach when initially touching. This only works if the radius is
	/// greater than zero.
	canEncroach: bool = false;
}

/// Low level ray cast or shape cast output data. The hit point is in the local or relative frame of
/// the input. Returns a zero fraction and normal in the case of initial overlap.
export struct b2CastOutput {
	/// The surface normal at the hit point
	normal: b2Vec2 = b2Vec2();

	/// The surface hit point
	point: b2Vec2 = b2Vec2();

	/// The fraction of the input translation at collision
	fraction: f32 = 0;

	/// The number of iterations used
	iterations: i32 = 0;

	/// Did the cast hit?
	hit: bool = false;
}

/// This is an alias in single precision.
export type b2WorldCastOutput = b2CastOutput;

/// This holds the mass data computed for a shape.
export struct b2MassData {
	/// The mass of the shape, usually in kilograms.
	mass: f32 = 0;

	/// The position of the shape's centroid relative to the shape's origin.
	center: b2Vec2 = b2Vec2();

	/// The rotational inertia of the shape about the shape center.
	rotationalInertia: f32 = 0;
}

/// A solid circle
export struct b2Circle {
	/// The local center
	center: b2Vec2 = b2Vec2();

	/// The radius
	radius: f32 = 0;

	constructor(center: b2Vec2, radius: f32) {
		this.center = center;
		this.radius = radius;
	}
}

/// A solid capsule can be viewed as two semicircles connected by a rectangle.
export struct b2Capsule {
	/// Local center of the first semicircle
	center1: b2Vec2 = b2Vec2();

	/// Local center of the second semicircle
	center2: b2Vec2 = b2Vec2();

	/// The radius of the semicircles
	radius: f32 = 0;

	constructor(center1: b2Vec2, center2: b2Vec2, radius: f32) {
		this.center1 = center1;
		this.center2 = center2;
		this.radius = radius;
	}
}

/// A solid convex polygon. It is assumed that the interior of the polygon is to the left of each
/// edge. Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES. In most cases
/// you should not need many vertices for a convex polygon.
/// @warning DO NOT fill this out manually, instead use a helper function like b2MakePolygon or
/// b2MakeBox.
export struct b2Polygon {
	/// The polygon vertices
	vertices: FixedArray<b2Vec2, 8>;

	/// The outward normal vectors of the polygon sides
	normals: FixedArray<b2Vec2, 8>;

	/// The centroid of the polygon
	centroid: b2Vec2 = b2Vec2();

	/// The external radius for rounded polygons
	radius: f32 = 0;

	/// The number of polygon vertices
	count: i32 = 0;
}

/// A line segment with two-sided collision.
export struct b2Segment {
	/// The first point
	point1: b2Vec2 = b2Vec2();

	/// The second point
	point2: b2Vec2 = b2Vec2();

	constructor(point1: b2Vec2, point2: b2Vec2) {
		this.point1 = point1;
		this.point2 = point2;
	}
}

/// A line segment with one-sided collision. Only collides on the right side. Several of these are
/// generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
export struct b2ChainSegment {
	/// The tail ghost vertex
	ghost1: b2Vec2 = b2Vec2();

	/// The line segment
	segment: b2Segment = b2Segment();

	/// The head ghost vertex
	ghost2: b2Vec2 = b2Vec2();

	/// The owning chain shape index (internal usage only)
	chainId: i32 = 0;
}

/// A convex hull. Used to create convex polygons.
/// @warning Do not modify these values directly, instead use b2ComputeHull()
export struct b2Hull {
	/// The final points of the hull
	points: FixedArray<b2Vec2, 8>;

	/// The number of points
	count: i32 = 0;
}

/**
 * @defgroup distance Distance
 */

/// Result of computing the distance between two line segments
export struct b2SegmentDistanceResult {
	/// The closest point on the first segment
	closest1: b2Vec2 = b2Vec2();

	/// The closest point on the second segment
	closest2: b2Vec2 = b2Vec2();

	/// The barycentric coordinate on the first segment
	fraction1: f32 = 0;

	/// The barycentric coordinate on the second segment
	fraction2: f32 = 0;

	/// The squared distance between the closest points
	distanceSquared: f32 = 0;
}

/// Used to warm start the GJK simplex. If you call b2ComputeShapeDistance multiple times with
/// nearby transforms this might improve performance. The distance cache must be initialized to zero
/// on the first call.
export struct b2SimplexCache {
	/// The number of stored simplex points
	count: u16 = 0;

	/// The cached simplex indices on shape A
	indexA: FixedArray<u8, 3>;

	/// The cached simplex indices on shape B
	indexB: FixedArray<u8, 3>;
}

export const b2_emptySimplexCache: b2SimplexCache = b2SimplexCache();

/// Input for b2ComputeShapeDistance
export struct b2DistanceInput {
	/// The proxy for shape A
	proxyA: b2ShapeProxy = b2ShapeProxy();

	/// The proxy for shape B
	proxyB: b2ShapeProxy = b2ShapeProxy();

	/// Transform of shape B in shape A's frame, the relative pose B in A (b2InvMulTransforms(
	/// worldA, worldB )). The query is origin independent and runs in frame A.
	transform: b2Transform = b2Transform();

	/// Should the proxy radius be considered?
	useRadii: bool = false;
}

/// Output for b2ComputeShapeDistance
export struct b2DistanceOutput {
	pointA: b2Vec2 = b2Vec2(); ///< Closest point on shapeA, in shape A's frame
	pointB: b2Vec2 = b2Vec2(); ///< Closest point on shapeB, in shape A's frame
	normal: b2Vec2 = b2Vec2(); ///< A to B normal in shape A's frame. Invalid if distance is zero.
	distance: f32 = 0; ///< The final distance, zero if overlapped
	iterations: i32 = 0; ///< Number of GJK iterations used
	simplexCount: i32 = 0; ///< The number of simplexes stored in the simplex array
}

/// Simplex vertex for debugging the GJK algorithm
export struct b2SimplexVertex {
	wA: b2Vec2 = b2Vec2(); ///< support point in proxyA
	wB: b2Vec2 = b2Vec2(); ///< support point in proxyB
	w: b2Vec2 = b2Vec2(); ///< wB - wA
	a: f32 = 0; ///< barycentric coordinate for closest point
	indexA: i32 = 0; ///< wA index
	indexB: i32 = 0; ///< wB index
}

/// Simplex from the GJK algorithm
export struct b2Simplex {
	v1: b2SimplexVertex = b2SimplexVertex(); ///< vertices
	v2: b2SimplexVertex = b2SimplexVertex();
	v3: b2SimplexVertex = b2SimplexVertex();
	count: i32 = 0; ///< number of valid vertices
}

/// Input parameters for b2CastShape
export struct b2ShapeCastPairInput {
	proxyA: b2ShapeProxy = b2ShapeProxy(); ///< The proxy for shape A
	proxyB: b2ShapeProxy = b2ShapeProxy(); ///< The proxy for shape B
	transform: b2Transform = b2Transform(); ///< Transform of shape B in shape A's frame, the relative pose B in A
	translationB: b2Vec2 = b2Vec2(); ///< The translation of shape B, in A's frame
	maxFraction: f32 = 0; ///< The fraction of the translation to consider, typically 1
	canEncroach: bool = false; ///< Allows shapes with a radius to move slightly closer if already touching
}

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect
/// to the body origin, which may not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
export struct b2Sweep {
	localCenter: b2Vec2 = b2Vec2(); ///< Local center of mass position
	c1: b2Vec2 = b2Vec2(); ///< Starting center of mass world position
	c2: b2Vec2 = b2Vec2(); ///< Ending center of mass world position
	q1: b2Rot = b2Rot(); ///< Starting world rotation
	q2: b2Rot = b2Rot(); ///< Ending world rotation
}

/// Time of impact input
export struct b2TOIInput {
	proxyA: b2ShapeProxy = b2ShapeProxy(); ///< The proxy for shape A
	proxyB: b2ShapeProxy = b2ShapeProxy(); ///< The proxy for shape B
	sweepA: b2Sweep = b2Sweep(); ///< The movement of shape A
	sweepB: b2Sweep = b2Sweep(); ///< The movement of shape B
	maxFraction: f32 = 0; ///< Defines the sweep interval [0, maxFraction]
}

/// Describes the TOI output
export type b2TOIState = i32;
export const b2_toiStateUnknown: b2TOIState = 0;
export const b2_toiStateFailed: b2TOIState = 1;
export const b2_toiStateOverlapped: b2TOIState = 2;
export const b2_toiStateHit: b2TOIState = 3;
export const b2_toiStateSeparated: b2TOIState = 4;

/// Time of impact output
export struct b2TOIOutput {
	/// The type of result
	state: b2TOIState = 0;

	/// The hit point
	point: b2Vec2 = b2Vec2();

	/// The hit normal
	normal: b2Vec2 = b2Vec2();

	/// The sweep time of the collision
	fraction: f32 = 0;
}

/**
 * @defgroup collision Collision
 */

/// A manifold point is a contact point belonging to a contact manifold. It holds details related to
/// the geometry and dynamics of the contact points. Box2D uses speculative collision so some
/// contact points may be separated. You may use the totalNormalImpulse to determine if there was an
/// interaction during the time step.
export struct b2ManifoldPoint {
	/// Location of the contact point relative to bodyA's center of mass in world space. This can be
	/// converted to a world point using:
	/// b2Pos worldPointA = b2OffsetPos(b2Body_GetWorldCenter(myBodyIdA), anchorA);
	anchorA: b2Vec2 = b2Vec2();

	/// Similar for body B.
	anchorB: b2Vec2 = b2Vec2();

	/// The separation of the contact point, negative if penetrating
	separation: f32 = 0;

	/// The impulse along the manifold normal vector.
	normalImpulse: f32 = 0;

	/// The friction impulse
	tangentImpulse: f32 = 0;

	/// The total normal impulse applied across sub-stepping and restitution. This is important to
	/// identify speculative contact points that had an interaction in the time step. This includes
	/// the warm starting impulse, the sub-step delta impulse, and the restitution impulse.
	totalNormalImpulse: f32 = 0;

	/// Relative normal velocity pre-solve. Negative when approaching. This is only computed if hit
	/// events are enabled. It may be computed otherwise as a side effect of SIMD bundling, but that
	/// is not guaranteed.
	normalVelocity: f32 = 0;

	/// Cached separation used for contact recycling
	baseSeparation: f32 = 0;

	/// Uniquely identifies a contact point between two shapes
	id: u16 = 0;

	/// Did this contact point exist the previous step?
	persisted: bool = false;
}

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box2D uses speculative collision so some contact points may be separated.
export struct b2Manifold {
	/// The unit normal vector in world space, points from shape A to shape B.
	normal: b2Vec2 = b2Vec2();

	/// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	rollingImpulse: f32 = 0;

	/// The manifold points, up to two are possible in 2D.
	points: FixedArray<b2ManifoldPoint, 2>;

	/// The number of contact points, will be 0, 1, or 2.
	pointCount: i32 = 0;
}

/// Contact manifold point in local coordinates (frame A).
export struct b2LocalManifoldPoint {
	/// Contact point in frame A
	point: b2Vec2 = b2Vec2();

	/// The separation of the contact point, negative if penetrating. May be positive or negative.
	separation: f32 = 0;

	/// Uniquely identifies a contact point between two shapes
	id: u16 = 0;
}

/// Contact manifold in local coordinates (frame A).
export struct b2LocalManifold {
	/// The unit normal vector in frame A, points from shape A to shape B
	normal: b2Vec2 = b2Vec2();

	/// The manifold points, up to two are possible in 2D
	points: FixedArray<b2LocalManifoldPoint, 2>;

	/// The number of contact points, will be 0, 1, or 2
	pointCount: i32 = 0;
}

/**
 * @defgroup tree Dynamic Tree
 */

/// These are performance results returned by dynamic tree queries.
export struct b2TreeStats {
	/// Number of internal nodes visited during the query
	nodeVisits: i32 = 0;

	/// Number of leaf nodes visited during the query
	leafVisits: i32 = 0;
}

// Callbacks drop C's void* context; the closure carries it.

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
export type b2TreeQueryCallbackFcn = (proxyId: i32, userData: u64) => bool;

/// This function receives clipped ray cast input for a proxy. The function returns the new ray
/// fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
export type b2TreeRayCastCallbackFcn = (input: b2RayCastInput, proxyId: i32, userData: u64) => f32;

/// Input for casting an AABB through a dynamic tree
export struct b2BoxCastInput {
	/// The AABB to cast, in the tree's frame.
	box: b2AABB = b2AABB();

	/// The sweep translation.
	translation: b2Vec2 = b2Vec2();

	/// The maximum fraction of the translation to consider, typically 1.
	maxFraction: f32 = 0;

	constructor(box: b2AABB, translation: b2Vec2, maxFraction: f32) {
		this.box = box;
		this.translation = translation;
		this.maxFraction = maxFraction;
	}
}

/// This function receives clipped AABB cast input for a proxy. The function returns the new cast
/// fraction.
/// - return a value of 0 to terminate the cast
/// - return a value less than input->maxFraction to clip the cast
/// - return a value of input->maxFraction to continue the cast without clipping
export type b2TreeBoxCastCallbackFcn = (input: b2BoxCastInput, proxyId: i32, userData: u64) => f32;

// src/dynamic_tree.h types. The constants and inline helpers stay in lib/dynamic_tree.as.

/// A node in the dynamic tree. Siblings sit together at an even index so two nodes fit in a 64 byte
/// cache line. The root is at index zero and index one is always empty.
export struct b2TreeNode {
	/// The node bounding box
	aabb: b2AABB = b2AABB();

	// C's union of height and shapeIndex is two fields here; shapeIndex takes half of the padding so a node stays
	// 32 bytes as in C, and a sibling pair one cache line

	/// The shape index for a leaf. Truncated from proxy user data.
	shapeIndex: i32 = 0;

	/// In 3D this space is used by the AABB z components.
	padding: u32 = 0;

	/// - bit 31 : 1 for leaf node
	/// - bit 30 : 1 for moved flag
	/// - bits 0-29 : index of the sibling pair node or the proxy id for a leaf
	flagIndex: u32 = 0;

	/// The height of an internal node. A leaf has zero height.
	height: i32 = 0;
}

/// Separate storage for tree leaves.
export struct b2TreeProxy {
	/// User data is an index instead of void* because it is used internally as a shape index.
	userData: u64 = 0;

	/// Category bits for collision filtering.
	categoryBits: u64 = 0;

	/// The leaf node. B2_NULL_INDEX for a free proxy.
	node: i32 = 0;

	/// Next free proxy.
	next: i32 = 0;
}

/// Opaque dynamic tree. C pointers become arrays; growth replaces an array with a larger copy.
export class b2DynamicTree {
	/// Array of nodes. The root is at index zero and index 1 is empty. Otherwise siblings are
	/// paired at even indices. Has holes for free node pairs.
	nodes: StaticArray<b2TreeNode> = new StaticArray<b2TreeNode>(0);

	/// Parent index per node. The free list is interwoven.
	parents: StaticArray<i32> = new StaticArray<i32>(0);

	/// Proxy data split from node array as cold data.
	proxies: StaticArray<b2TreeProxy> = new StaticArray<b2TreeProxy>(0);

	/// Every allocated node has a lower index than this.
	nodeEnd: i32 = 0;

	/// The allocated node space
	nodeCapacity: i32 = 0;

	/// Free pairs below nodeEnd
	pairFreeList: i32 = 0;

	/// Number of proxies created
	proxyCount: i32 = 0;

	/// The allocated proxy space
	proxyCapacity: i32 = 0;

	/// Proxy free list
	proxyFreeList: i32 = 0;

	/// Array of nodes for rebuild.
	swapNodes: StaticArray<b2TreeNode> = new StaticArray<b2TreeNode>(0);

	/// Leaf indices for rebuild
	leafIndices: StaticArray<i32> = new StaticArray<i32>(0);

	/// Leaves for the rebuild. May represent a proxy or a retained subtree.
	leafNodes: StaticArray<b2TreeNode> = new StaticArray<b2TreeNode>(0);

	/// Leaf bounding boxes for rebuild
	leafBoxes: StaticArray<b2AABB> = new StaticArray<b2AABB>(0);

	/// Leaf bounding box centers for rebuild
	leafCenters: StaticArray<b2Vec2> = new StaticArray<b2Vec2>(0);

	/// Bins for sorting during rebuild
	binIndices: StaticArray<i32> = new StaticArray<i32>(0);

	/// Allocated space for rebuilding
	rebuildCapacity: i32 = 0;

	/// Rebuild orders the nodes so the children follow parents. Cache friendly for queries and
	/// refitting. The order can be disrupted by proxy creation.
	dfsOrdered: bool = false;

	/// A rebuild is built in swapNodes and b2DynamicTree_FinishRebuild has yet to swap it in
	rebuildPending: bool = false;
}

/**
 * @defgroup character Character mover
 */

/// These are the collision planes returned from b2World_CollideMover. The plane and point are
/// relative to the query origin, matching the mover capsule.
export struct b2PlaneResult {
	/// The collision plane between the mover and a convex shape
	plane: b2Plane = b2Plane();

	/// The collision point on the shape.
	point: b2Vec2 = b2Vec2();

	/// Did the collision register a hit? If not this plane should be ignored.
	hit: bool = false;
}

/// These are collision planes that can be fed to b2SolvePlanes. Normally this is assembled by the
/// user from plane results in b2PlaneResult.
export struct b2CollisionPlane {
	/// The collision plane between the mover and some shape.
	plane: b2Plane = b2Plane();

	/// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can make the
	/// plane collision soft. Usually in meters.
	pushLimit: f32 = 0;

	/// The push on the mover determined by b2SolvePlanes. Usually in meters.
	push: f32 = 0;

	/// Indicates if b2ClipVector should clip against this plane. Should be false for soft
	/// collision.
	clipVelocity: bool = false;
}

/// Result returned by b2SolvePlanes.
export struct b2PlaneSolverResult {
	/// The final relative translation.
	delta: b2Vec2 = b2Vec2();

	/// The number of iterations used by the plane solver. For diagnostics.
	iterationCount: i32 = 0;
}
