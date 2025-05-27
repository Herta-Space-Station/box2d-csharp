using System.Runtime.CompilerServices;
using static Box2d.Box2d;

#pragma warning disable CS0169

// Resharper disable ALL

namespace Box2d
{
    // @defgroup geometry Geometry
    // @brief Geometry types and algorithms
    //
    // Definitions of circles, capsules, segments, and polygons. Various algorithms to compute hulls, mass properties, and so on.

    public static unsafe partial class Box2d
    {
        /// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
        /// don't use more vertices.
        public const int B2_MAX_POLYGON_VERTICES = 8;
    }

    /// Low level ray cast input data
    public struct b2RayCastInput
    {
        /// Start point of the ray cast
        public b2Vec2 origin;

        /// Translation of the ray cast
        public b2Vec2 translation;

        /// The maximum fraction of the translation to consider, typically 1
        public float maxFraction;
    }

    /// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
    /// You can provide between 1 and B2_MAX_POLYGON_VERTICES and a radius.
    public unsafe struct b2ShapeProxy
    {
        private b2ShapeProxy_b2Vec2_t points_t;

        /// The point cloud
        [FixedArray(B2_MAX_POLYGON_VERTICES)]
        public b2Vec2* points => (b2Vec2*)Unsafe.AsPointer(ref points_t);

        /// The number of points. Must be greater than 0.
        public int count;

        /// The external radius of the point cloud. May be zero.
        public float radius;

        private struct b2ShapeProxy_b2Vec2_t
        {
            private b2Vec2 element0, element1, element2, element3, element4, element5, element6, element7;
        }
    }

    /// Low level shape cast input in generic form. This allows casting an arbitrary point
    /// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
    /// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
    public struct b2ShapeCastInput
    {
        /// A generic shape
        public b2ShapeProxy proxy;

        /// The translation of the shape cast
        public b2Vec2 translation;

        /// The maximum fraction of the translation to consider, typically 1
        public float maxFraction;

        /// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
        public bool canEncroach;
    }

    /// Low level ray cast or shape-cast output data
    public struct b2CastOutput
    {
        /// The surface normal at the hit point
        public b2Vec2 normal;

        /// The surface hit point
        public b2Vec2 point;

        /// The fraction of the input translation at collision
        public float fraction;

        /// The number of iterations used
        public int iterations;

        /// Did the cast hit?
        public bool hit;
    }

    /// This holds the mass data computed for a shape.
    public struct b2MassData
    {
        /// The mass of the shape, usually in kilograms.
        public float mass;

        /// The position of the shape's centroid relative to the shape's origin.
        public b2Vec2 center;

        /// The rotational inertia of the shape about the local origin.
        public float rotationalInertia;
    }

    /// A solid circle
    public struct b2Circle
    {
        /// The local center
        public b2Vec2 center;

        /// The radius
        public float radius;
    }

    /// A solid capsule can be viewed as two semicircles connected
    /// by a rectangle.
    public struct b2Capsule
    {
        /// Local center of the first semicircle
        public b2Vec2 center1;

        /// Local center of the second semicircle
        public b2Vec2 center2;

        /// The radius of the semicircles
        public float radius;
    }

    /// A solid convex polygon. It is assumed that the interior of the polygon is to
    /// the left of each edge.
    /// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
    /// In most cases you should not need many vertices for a convex polygon.
    /// @warning DO NOT fill this out manually, instead use a helper function like
    /// b2MakePolygon or b2MakeBox.
    public unsafe struct b2Polygon
    {
        private b2Polygon_Vec2_t vertices_t;

        /// The polygon vertices
        [FixedArray(B2_MAX_POLYGON_VERTICES)]
        public b2Vec2* vertices => (b2Vec2*)Unsafe.AsPointer(ref vertices_t);

        private b2Polygon_Vec2_t normals_t;

        /// The outward normal vectors of the polygon sides
        [FixedArray(B2_MAX_POLYGON_VERTICES)]
        public b2Vec2* normals => (b2Vec2*)Unsafe.AsPointer(ref normals_t);

        /// The centroid of the polygon
        public b2Vec2 centroid;

        /// The external radius for rounded polygons
        public float radius;

        /// The number of polygon vertices
        public int count;

        private struct b2Polygon_Vec2_t
        {
            private b2Vec2 element0, element1, element2, element3, element4, element5, element6, element7;
        }
    }

    /// A line segment with two-sided collision.
    public struct b2Segment
    {
        /// The first point
        public b2Vec2 point1;

        /// The second point
        public b2Vec2 point2;
    }

    /// A line segment with one-sided collision. Only collides on the right side.
    /// Several of these are generated for a chain shape.
    /// ghost1 -> point1 -> point2 -> ghost2
    public struct b2ChainSegment
    {
        /// The tail ghost vertex
        public b2Vec2 ghost1;

        /// The line segment
        public b2Segment segment;

        /// The head ghost vertex
        public b2Vec2 ghost2;

        /// The owning chain shape index (internal usage only)
        public int chainId;
    }

    public static unsafe partial class Box2d
    {
        /// Validate ray cast input data (NaN, etc)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2IsValidRay(b2RayCastInput* input);

        /// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
        /// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakePolygon(b2Hull* hull, float radius);

        /// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
        /// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeOffsetPolygon(b2Hull* hull, b2Vec2 position, b2Rot rotation);

        /// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
        /// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeOffsetRoundedPolygon(b2Hull* hull, b2Vec2 position, b2Rot rotation, float radius);

        /// Make a square polygon, bypassing the need for a convex hull.
        /// @param halfWidth the half-width
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeSquare(float halfWidth);

        /// Make a box (rectangle) polygon, bypassing the need for a convex hull.
        /// @param halfWidth the half-width (x-axis)
        /// @param halfHeight the half-height (y-axis)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeBox(float halfWidth, float halfHeight);

        /// Make a rounded box, bypassing the need for a convex hull.
        /// @param halfWidth the half-width (x-axis)
        /// @param halfHeight the half-height (y-axis)
        /// @param radius the radius of the rounded extension
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeRoundedBox(float halfWidth, float halfHeight, float radius);

        /// Make an offset box, bypassing the need for a convex hull.
        /// @param halfWidth the half-width (x-axis)
        /// @param halfHeight the half-height (y-axis)
        /// @param center the local center of the box
        /// @param rotation the local rotation of the box
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeOffsetBox(float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation);

        /// Make an offset rounded box, bypassing the need for a convex hull.
        /// @param halfWidth the half-width (x-axis)
        /// @param halfHeight the half-height (y-axis)
        /// @param center the local center of the box
        /// @param rotation the local rotation of the box
        /// @param radius the radius of the rounded extension
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2MakeOffsetRoundedBox(float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation, float radius);

        /// Transform a polygon. This is useful for transferring a shape from one body to another.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Polygon b2TransformPolygon(b2Transform transform, b2Polygon* polygon);

        /// Compute mass properties of a circle
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2MassData b2ComputeCircleMass(b2Circle* shape, float density);

        /// Compute mass properties of a capsule
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2MassData b2ComputeCapsuleMass(b2Capsule* shape, float density);

        /// Compute mass properties of a polygon
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2MassData b2ComputePolygonMass(b2Polygon* shape, float density);

        /// Compute the bounding box of a transformed circle
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2AABB b2ComputeCircleAABB(b2Circle* shape, b2Transform transform);

        /// Compute the bounding box of a transformed capsule
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2AABB b2ComputeCapsuleAABB(b2Capsule* shape, b2Transform transform);

        /// Compute the bounding box of a transformed polygon
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2AABB b2ComputePolygonAABB(b2Polygon* shape, b2Transform transform);

        /// Compute the bounding box of a transformed line segment
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2AABB b2ComputeSegmentAABB(b2Segment* shape, b2Transform transform);

        /// Test a point for overlap with a circle in local space
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2PointInCircle(b2Vec2 point, b2Circle* shape);

        /// Test a point for overlap with a capsule in local space
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2PointInCapsule(b2Vec2 point, b2Capsule* shape);

        /// Test a point for overlap with a convex polygon in local space
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2PointInPolygon(b2Vec2 point, b2Polygon* shape);

        /// Ray cast versus circle shape in local space. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2RayCastCircle(b2RayCastInput* input, b2Circle* shape);

        /// Ray cast versus capsule shape in local space. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2RayCastCapsule(b2RayCastInput* input, b2Capsule* shape);

        /// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
        /// the left side being treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2RayCastSegment(b2RayCastInput* input, b2Segment* shape, bool oneSided);

        /// Ray cast versus polygon shape in local space. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2RayCastPolygon(b2RayCastInput* input, b2Polygon* shape);

        /// Shape cast versus a circle. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2ShapeCastCircle(b2ShapeCastInput* input, b2Circle* shape);

        /// Shape cast versus a capsule. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2ShapeCastCapsule(b2ShapeCastInput* input, b2Capsule* shape);

        /// Shape cast versus a line segment. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2ShapeCastSegment(b2ShapeCastInput* input, b2Segment* shape);

        /// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2ShapeCastPolygon(b2ShapeCastInput* input, b2Polygon* shape);
    }

    /// A convex hull. Used to create convex polygons.
    /// @warning Do not modify these values directly, instead use b2ComputeHull()
    public unsafe struct b2Hull
    {
        private b2Hull_b2Vec2_t points_t;

        /// The final points of the hull
        [FixedArray(B2_MAX_POLYGON_VERTICES)]
        public b2Vec2* points => (b2Vec2*)Unsafe.AsPointer(ref points_t);;

        /// The number of points
        public int count;

        private struct b2Hull_b2Vec2_t
        {
            private b2Vec2 element0, element1, element2, element3, element4, element5, element6, element7;
        }
    }

    public unsafe partial class Box2d
    {
        /// Compute the convex hull of a set of points. Returns an empty hull if it fails.
        /// Some failure cases:
        /// - all points very close together
        /// - all points on a line
        /// - less than 3 points
        /// - more than B2_MAX_POLYGON_VERTICES points
        /// This welds close points and removes collinear points.
        /// @warning Do not modify a hull once it has been computed
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Hull b2ComputeHull(b2Vec2* points, int count);

        /// This determines if a hull is valid. Checks for:
        /// - convexity
        /// - collinear points
        /// This is expensive and should not be called at runtime.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial bool b2ValidateHull(b2Hull* hull);
    }

    // @defgroup distance Distance
    // Functions for computing the distance between shapes.
    //
    // These are advanced functions you can use to perform distance calculations. There
    // are functions for computing the closest points between shapes, doing linear shape casts,
    // and doing rotational shape casts. The latter is called time of impact (TOI).

    /// Result of computing the distance between two line segments
    public struct b2SegmentDistanceResult
    {
        /// The closest point on the first segment
        public b2Vec2 closest1;

        /// The closest point on the second segment
        public b2Vec2 closest2;

        /// The barycentric coordinate on the first segment
        public float fraction1;

        /// The barycentric coordinate on the second segment
        public float fraction2;

        /// The squared distance between the closest points
        public float distanceSquared;
    }

    public unsafe partial class Box2d
    {
        /// Compute the distance between two line segments, clamping at the end points if needed.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2);
    }

    /// Used to warm start the GJK simplex. If you call this function multiple times with nearby
    /// transforms this might improve performance. Otherwise you can zero initialize this.
    /// The distance cache must be initialized to zero on the first call.
    /// Users should generally just zero initialize this structure for each call.
    public unsafe struct b2SimplexCache
    {
        /// The number of stored simplex points
        public ushort count;

        /// The cached simplex indices on shape A
        public fixed byte indexA[3];

        /// The cached simplex indices on shape B
        public fixed byte indexB[3];
    }

    public unsafe partial class Box2d
    {
        public static readonly b2SimplexCache b2_emptySimplexCache = new b2SimplexCache();
    }

    /// Input for b2ShapeDistance
    public struct b2DistanceInput
    {
        /// The proxy for shape A
        public b2ShapeProxy proxyA;

        /// The proxy for shape B
        public b2ShapeProxy proxyB;

        /// The world transform for shape A
        public b2Transform transformA;

        /// The world transform for shape B
        public b2Transform transformB;

        /// Should the proxy radius be considered?
        public bool useRadii;
    }

    /// Output for b2ShapeDistance
    public struct b2DistanceOutput
    {
        /// Closest point on shapeA
        public b2Vec2 pointA;

        /// Closest point on shapeB
        public b2Vec2 pointB;

        /// Normal vector that points from A to B
        public b2Vec2 normal;

        /// The final distance, zero if overlapped
        public float distance;

        /// Number of GJK iterations used
        public int iterations;

        /// The number of simplexes stored in the simplex array
        public int simplexCount;
    }

    /// Simplex vertex for debugging the GJK algorithm
    public struct b2SimplexVertex
    {
        /// support point in proxyA
        public b2Vec2 wA;

        /// support point in proxyB
        public b2Vec2 wB;

        /// wB - wA
        public b2Vec2 w;

        /// barycentric coordinate for closest point
        public float a;

        /// wA index
        public int indexA;

        /// wB index
        public int indexB;
    }

    /// Simplex from the GJK algorithm
    public struct b2Simplex
    {
        /// vertices
        public b2SimplexVertex v1, v2, v3;

        /// number of valid vertices
        public int count;
    }

    public unsafe partial class Box2d
    {
        /// Compute the closest points between two shapes represented as point clouds.
        /// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
        /// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2DistanceOutput b2ShapeDistance(b2DistanceInput* input, b2SimplexCache* cache, b2Simplex* simplexes, int simplexCapacity);
    }

    /// Input parameters for b2ShapeCast
    public struct b2ShapeCastPairInput
    {
        /// The proxy for shape A
        public b2ShapeProxy proxyA;

        /// The proxy for shape B
        public b2ShapeProxy proxyB;

        /// The world transform for shape A
        public b2Transform transformA;

        /// The world transform for shape B
        public b2Transform transformB;

        /// The translation of shape B
        public b2Vec2 translationB;

        /// The fraction of the translation to consider, typically 1
        public float maxFraction;

        /// Allows shapes with a radius to move slightly closer if already touching
        public bool canEncroach;
    }

    /// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
    /// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
    /// position.
    public struct b2Sweep
    {
        /// Local center of mass position
        public b2Vec2 localCenter;

        /// Starting center of mass world position
        public b2Vec2 c1;

        /// Ending center of mass world position
        public b2Vec2 c2;

        /// Starting world rotation
        public b2Rot q1;

        /// Ending world rotation
        public b2Rot q2;
    }

    /// Input parameters for b2TimeOfImpact
    public struct b2TOIInput
    {
        /// The proxy for shape A
        public b2ShapeProxy proxyA;

        /// The proxy for shape B
        public b2ShapeProxy proxyB;

        /// The movement of shape A
        public b2Sweep sweepA;

        /// The movement of shape B
        public b2Sweep sweepB;

        /// Defines the sweep interval [0, maxFraction]
        public float maxFraction;
    }

    /// Describes the TOI output
    public enum b2TOIState
    {
        b2_toiStateUnknown,
        b2_toiStateFailed,
        b2_toiStateOverlapped,
        b2_toiStateHit,
        b2_toiStateSeparated
    }

    /// Output parameters for b2TimeOfImpact.
    public struct b2TOIOutput
    {
        /// The type of result
        public b2TOIState state;

        /// The sweep time of the collision
        public float fraction;
    }

    public unsafe partial class Box2d
    {
        /// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
        /// You may optionally supply an array to hold debug data.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2CastOutput b2ShapeCast(b2ShapeCastPairInput* input);

        /// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2ShapeProxy b2MakeProxy(b2Vec2* points, int count, float radius);

        /// Make a proxy with a transform. This is a deep copy of the points.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2ShapeProxy b2MakeOffsetProxy(b2Vec2* points, int count, float radius, b2Vec2 position, b2Rot rotation);

        /// Evaluate the transform sweep at a specific time.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Transform b2GetSweepTransform(b2Sweep* sweep, float time);

        /// Compute the upper bound on time before two shapes penetrate. Time is represented as
        /// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
        /// non-tunneling collisions. If you change the time interval, you should call this function
        /// again.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2TOIOutput b2TimeOfImpact(b2TOIInput* input);
    }

    // @defgroup collision Collision
    // @brief Functions for colliding pairs of shapes

    /// A manifold point is a contact point belonging to a contact manifold.
    /// It holds details related to the geometry and dynamics of the contact points.
    /// Box2D uses speculative collision so some contact points may be separated.
    /// You may use the totalNormalImpulse to determine if there was an interaction during
    /// the time step.
    public unsafe struct b2ManifoldPoint
    {
        /// Location of the contact point in world space. Subject to precision loss at large coordinates.
        /// @note Should only be used for debugging.
        public b2Vec2 point;

        /// Location of the contact point relative to shapeA's origin in world space
        /// @note When used internally to the Box2D solver, this is relative to the body center of mass.
        public b2Vec2 anchorA;

        /// Location of the contact point relative to shapeB's origin in world space
        /// @note When used internally to the Box2D solver, this is relative to the body center of mass.
        public b2Vec2 anchorB;

        /// The separation of the contact point, negative if penetrating
        public float separation;

        /// The impulse along the manifold normal vector.
        public float normalImpulse;

        /// The friction impulse
        public float tangentImpulse;

        /// The total normal impulse applied across sub-stepping and restitution. This is important
        /// to identify speculative contact points that had an interaction in the time step.
        public float totalNormalImpulse;

        /// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
        /// zero then there was no hit. Negative means shapes are approaching.
        public float normalVelocity;

        /// Uniquely identifies a contact point between two shapes
        public ushort id;

        /// Did this contact point exist the previous step?
        public bool persisted;
    }

    /// A contact manifold describes the contact points between colliding shapes.
    /// @note Box2D uses speculative collision so some contact points may be separated.
    public unsafe struct b2Manifold
    {
        /// The unit normal vector in world space, points from shape A to bodyB
        public b2Vec2 normal;

        /// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
        public float rollingImpulse;

        private b2Manifold_b2ManifoldPoint_t points_t;

        /// The manifold points, up to two are possible in 2D
        [FixedArray(2)]
        public b2ManifoldPoint* points => (b2ManifoldPoint*)Unsafe.AsPointer(ref points_t);

        /// The number of contacts points, will be 0, 1, or 2
        public int pointCount;

        private struct b2Manifold_b2ManifoldPoint_t
        {
            private b2ManifoldPoint element0, element1;
        }
    }

    public unsafe partial class Box2d
    {
        /// Compute the contact manifold between two circles.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideCircles(b2Circle* circleA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

        /// Compute the contact manifold between a capsule and circle.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideCapsuleAndCircle(b2Capsule* capsuleA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

        /// Compute the contact manifold between a segment and a circle.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideSegmentAndCircle(b2Segment* segmentA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

        /// Compute the contact manifold between a polygon and a circle.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollidePolygonAndCircle(b2Polygon* polygonA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

        /// Compute the contact manifold between two capsules.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideCapsules(b2Capsule* capsuleA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB);

        /// Compute the contact manifold between a segment and a capsule.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideSegmentAndCapsule(b2Segment* segmentA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB);

        /// Compute the contact manifold between a polygon and a capsule.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollidePolygonAndCapsule(b2Polygon* polygonA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB);

        /// Compute the contact manifold between two polygons.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollidePolygons(b2Polygon* polygonA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB);

        /// Compute the contact manifold between a segment and a polygon.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideSegmentAndPolygon(b2Segment* segmentA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB);

        /// Compute the contact manifold between a chain segment and a circle.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideChainSegmentAndCircle(b2ChainSegment* segmentA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

        /// Compute the contact manifold between a chain segment and a capsule.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideChainSegmentAndCapsule(b2ChainSegment* segmentA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB, b2SimplexCache* cache);

        /// Compute the contact manifold between a chain segment and a polygon.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Manifold b2CollideChainSegmentAndPolygon(b2ChainSegment* segmentA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB, b2SimplexCache* cache);
    }

    // @defgroup tree Dynamic Tree
    // The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
    //
    // Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
    // This data structure may have uses in games for organizing other geometry data and may be used independently
    // of Box2D rigid body simulation.
    //
    // A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
    // A dynamic tree arranges data in a binary tree to accelerate
    // queries such as AABB queries and ray casts. Leaf nodes are proxies
    // with an AABB. These are used to hold a user collision object.
    // Nodes are pooled and relocatable, so I use node indices rather than pointers.
    // The dynamic tree is made available for advanced users that would like to use it to organize
    // spatial game data besides rigid bodies.

    /// The dynamic tree structure. This should be considered private data.
    /// It is placed here for performance reasons.
    public unsafe struct b2DynamicTree
    {
        /// The tree nodes
        public b2TreeNode* nodes;

        /// The root index
        public int root;

        /// The number of nodes
        public int nodeCount;

        /// The allocated node space
        public int nodeCapacity;

        /// Node free list
        public int freeList;

        /// Number of proxies created
        public int proxyCount;

        /// Leaf indices for rebuild
        public int* leafIndices;

        /// Leaf bounding boxes for rebuild
        public b2AABB* leafBoxes;

        /// Leaf bounding box centers for rebuild
        public b2Vec2* leafCenters;

        /// Bins for sorting during rebuild
        public int* binIndices;

        /// Allocated space for rebuilding
        public int rebuildCapacity;
    }

    /// These are performance results returned by dynamic tree queries.
    public struct b2TreeStats
    {
        /// Number of internal nodes visited during the query
        public int nodeVisits;

        /// Number of leaf nodes visited during the query
        public int leafVisits;
    }

    public unsafe partial class Box2d
    {
        /// Constructing the tree initializes the node pool.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2DynamicTree b2DynamicTree_Create();

        /// Destroy the tree, freeing the node pool.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_Destroy(b2DynamicTree* tree);

        /// Create a proxy. Provide an AABB and a userData value.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, ulong categoryBits, ulong userData);

        /// Destroy a proxy. This asserts if the id is invalid.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int proxyId);

        /// Move a proxy to a new AABB by removing and reinserting into the tree.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_MoveProxy(b2DynamicTree* tree, int proxyId, b2AABB aabb);

        /// Enlarge a proxy and enlarge ancestors as necessary.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_EnlargeProxy(b2DynamicTree* tree, int proxyId, b2AABB aabb);

        /// Modify the category bits on a proxy. This is an expensive operation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_SetCategoryBits(b2DynamicTree* tree, int proxyId, ulong categoryBits);

        /// Get the category bits on a proxy.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial ulong b2DynamicTree_GetCategoryBits(b2DynamicTree* tree, int proxyId);
    }

    /// This function receives proxies found in the AABB query.
    /// @return true if the query should continue
    public unsafe delegate bool b2TreeQueryCallbackFcn(int proxyId, ulong userData, void* context);

    public unsafe partial class Box2d
    {
        /// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
        /// @return performance data
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2TreeStats b2DynamicTree_Query(b2DynamicTree* tree, b2AABB aabb, ulong maskBits, [NativeType(typeof(b2TreeQueryCallbackFcn))] delegate* managed<int, ulong, void*, bool> callback, void* context);
    }

    /// This function receives clipped ray cast input for a proxy. The function
    /// returns the new ray fraction.
    /// - return a value of 0 to terminate the ray cast
    /// - return a value less than input->maxFraction to clip the ray
    /// - return a value of input->maxFraction to continue the ray cast without clipping
    public unsafe delegate float b2TreeRayCastCallbackFcn(b2RayCastInput* input, int proxyId, ulong userData, void* context);

    public unsafe partial class Box2d
    {
        /// Ray cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
        /// However, this filtering may be approximate, so the user should still apply filtering to results.
        /// @param tree the dynamic tree to ray cast
        /// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
        /// @param maskBits mask bit hint: `bool accept = (maskBits & node->categoryBits) != 0;`
        /// @param callback a callback class that is called for each proxy that is hit by the ray
        /// @param context user context that is passed to the callback
        /// @return performance data
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2TreeStats b2DynamicTree_RayCast(b2DynamicTree* tree, b2RayCastInput* input, ulong maskBits, [NativeType(typeof(b2TreeRayCastCallbackFcn))] delegate* managed<b2RayCastInput*, int, ulong, void*, float> callback, void* context);
    }


    /// This function receives clipped ray cast input for a proxy. The function
    /// returns the new ray fraction.
    /// - return a value of 0 to terminate the ray cast
    /// - return a value less than input->maxFraction to clip the ray
    /// - return a value of input->maxFraction to continue the ray cast without clipping
    public unsafe delegate float b2TreeShapeCastCallbackFcn(b2ShapeCastInput* input, int proxyId, ulong userData, void* context);

    public unsafe partial class Box2d
    {
        /// Ray cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// @param tree the dynamic tree to ray cast
        /// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// @param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
        /// @param callback a callback class that is called for each proxy that is hit by the shape
        /// @param context user context that is passed to the callback
        /// @return performance data
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2TreeStats b2DynamicTree_ShapeCast(b2DynamicTree* tree, b2ShapeCastInput* input, ulong maskBits, [NativeType(typeof(b2TreeShapeCastCallbackFcn))] delegate* managed<b2ShapeCastInput*, int, ulong, void*, float> callback, void* context);

        /// Get the height of the binary tree.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2DynamicTree_GetHeight(b2DynamicTree* tree);

        /// Get the ratio of the sum of the node areas to the root area.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial float b2DynamicTree_GetAreaRatio(b2DynamicTree* tree);

        /// Get the bounding box that contains the entire tree
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2AABB b2DynamicTree_GetRootBounds(b2DynamicTree* tree);

        /// Get the number of proxies created
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2DynamicTree_GetProxyCount(b2DynamicTree* tree);

        /// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2DynamicTree_Rebuild(b2DynamicTree* tree, bool fullBuild);

        /// Get the number of bytes used by this tree
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial int b2DynamicTree_GetByteCount(b2DynamicTree* tree);

        /// Get proxy user data
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial ulong b2DynamicTree_GetUserData(b2DynamicTree* tree, int proxyId);

        /// Get the AABB of a proxy
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2AABB b2DynamicTree_GetAABB(b2DynamicTree* tree, int proxyId);

        /// Validate this tree. For testing.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_Validate(b2DynamicTree* tree);

        /// Validate this tree has no enlarged AABBs. For testing.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial void b2DynamicTree_ValidateNoEnlarged(b2DynamicTree* tree);
    }

    // @defgroup character Character mover
    // Character movement solver

    /// These are the collision planes returned from b2World_CollideMover
    public struct b2PlaneResult
    {
        /// The collision plane between the mover and a convex shape
        b2Plane plane;

        /// Did the collision register a hit? If not this plane should be ignored.
        bool hit;
    }

    /// These are collision planes that can be fed to b2SolvePlanes. Normally
    /// this is assembled by the user from plane results in b2PlaneResult
    public struct b2CollisionPlane
    {
        /// The collision plane between the mover and some shape
        public b2Plane plane;

        /// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
        /// make the plane collision soft. Usually in meters.
        public float pushLimit;

        /// The push on the mover determined by b2SolvePlanes. Usually in meters.
        public float push;

        /// Indicates if b2ClipVector should clip against this plane. Should be false for soft collision.
        bool clipVelocity;
    }

    /// Result returned by b2SolvePlanes
    public struct b2PlaneSolverResult
    {
        /// The final position of the mover
        public b2Vec2 position;

        /// The number of iterations used by the plane solver. For diagnostics.
        public int iterationCount;
    }

    public unsafe partial class Box2d
    {
        /// Solves the position of a mover that satisfies the given collision planes.
        /// @param position this must be the position used to generate the collision planes
        /// @param planes the collision planes
        /// @param count the number of collision planes
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2PlaneSolverResult b2SolvePlanes(b2Vec2 position, b2CollisionPlane* planes, int count);

        /// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
        /// set to false are skipped.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static partial b2Vec2 b2ClipVector(b2Vec2 vector, b2CollisionPlane* planes, int count);
    }
}