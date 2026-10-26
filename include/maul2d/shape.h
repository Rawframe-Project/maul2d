// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: geometry, materials, collision filters, chains, convex
// hulls, outline decomposition and the query functions.

#ifndef MAUL2D_SHAPE_H
#define MAUL2D_SHAPE_H

#include "maul2d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M2_MAX_POLYGON_VERTICES 8

    /// A shape of a world: a 1-based slot and a generation.
    typedef struct m2ShapeId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2ShapeId;

    /// A circle. All geometry is authored in body-local space, in 32-bit
    /// floats, with every coordinate and radius within M2_HUGE.
    typedef struct m2Circle
    {
        m2Vec2 center;
        float radius;
    } m2Circle;

    /// A segment swept by a radius.
    typedef struct m2Capsule
    {
        m2Vec2 point1;
        m2Vec2 point2;
        float radius;
    } m2Capsule;

    /// A convex, counter-clockwise polygon of at most 8 vertices; a radius
    /// above zero rounds it. Build one with m2MakePolygon, m2ComputeHull or
    /// m2MakeBox, which compute the normals and check it. A world keeps
    /// only the vertices, count and radius of a polygon it is given: it
    /// computes the normals itself and zeroes the unused slots.
    typedef struct m2Polygon
    {
        m2Vec2 vertices[M2_MAX_POLYGON_VERTICES];
        m2Vec2 normals[M2_MAX_POLYGON_VERTICES];
        int32_t count;
        float radius;
    } m2Polygon;

    /// A two-sided line segment.
    typedef struct m2Segment
    {
        m2Vec2 point1;
        m2Vec2 point2;
    } m2Segment;

    /// One link of a chain: the segment and the ghost points of its
    /// neighbors, which keep bodies from snagging on the seams. Built by
    /// m2CreateChain. It collides on one side only: solid on the right when
    /// walking from point1 to point2.
    typedef struct m2ChainSegment
    {
        m2Segment segment;
        m2Vec2 ghost1;
        m2Vec2 ghost2;
    } m2ChainSegment;

    typedef enum m2ShapeType
    {
        m2_circleShape = 0,
        m2_capsuleShape = 1,
        m2_polygonShape = 2,
        m2_segmentShape = 3,
        m2_chainSegmentShape = 4,
    } m2ShapeType;

    /// The def of a shape. Initialize it with m2DefaultShapeDef.
    typedef struct m2ShapeDef
    {
        float density;      // kg/m^2, 0 to M2_MAX_DENSITY; a dynamic body gets a minimum mass
        float friction;     // Coulomb, 0 to M2_MAX_COEFFICIENT; pairs mix by geometric mean
        float tangentSpeed; // conveyor along the contact tangent, m/s, within M2_MAX_SPEED
        float restitution;  // bounce in [0, 1]; pairs mix by maximum
        /// Two shapes collide when each one's category intersects the
        /// other's mask. Defaults: category 1, mask all. Queries take
        /// their own m2QueryFilter.
        M2_ALIGN8 uint64_t categoryBits;
        M2_ALIGN8 uint64_t maskBits;
        /// The same nonzero group on both shapes overrides the mask rule:
        /// positive always collides, negative never does. Zero defers to
        /// categories and masks. Queries ignore groups.
        int32_t groupIndex;
        /// A sensor detects overlaps and never pushes back: no solver
        /// response, no bullet blocking, no effect on sleep or islands.
        /// Its overlaps arrive through m2World_GetSensorEvents. Two sensors
        /// never detect each other.
        bool isSensor;
        M2_ALIGN8 uint64_t userData;
        int32_t internalValue; // the cookie m2DefaultShapeDef sets
    } m2ShapeDef;

    /// A chain of one-sided segments: a 1-based slot and a generation.
    typedef struct m2ChainId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2ChainId;

    /// The def of a chain. Initialize it with m2DefaultChainDef. An open
    /// chain needs at least 4 points: the first and last are ghosts, and
    /// the chain collides along points[1..count-2]. A loop needs at least
    /// 3 and wraps. Every point lies within M2_HUGE. The points are copied,
    /// so the array may be temporary.
    typedef struct m2ChainDef
    {
        const m2Vec2* points;
        int32_t count;
        bool isLoop;
        float friction;    // 0 to M2_MAX_COEFFICIENT
        float restitution; // 0 to 1
        M2_ALIGN8 uint64_t categoryBits;
        M2_ALIGN8 uint64_t maskBits;
        int32_t groupIndex;
        M2_ALIGN8 uint64_t userData;
        int32_t internalValue; // the cookie m2DefaultChainDef sets
    } m2ChainDef;

    /// Returns a chain def with the default material and filter.
    ///
    /// @return The default def, with no points.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2ChainDef m2DefaultChainDef(void);

    /// Creates a chain's segment shapes on a body. A shape pool that runs
    /// out midway leaves a shorter chain, which m2Chain_GetSegmentCount
    /// reports. The id names the whole chain; destroying the body retires
    /// it too. Journaled as one operation.
    ///
    /// @param world     The world.
    /// @param bodyId    The body the segments attach to.
    /// @param def       The chain, from m2DefaultChainDef.
    /// @param chainOut  Receives the chain; the null id on failure.
    /// @return m2_success; m2_errorStale for a stale body; m2_errorInvalid
    ///         for a bad def or a null chainOut; m2_errorCapacity when the
    ///         chain or shape pool has no room at all; m2_errorState from
    ///         host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateChain(m2World* world, m2BodyId bodyId,
                                               const m2ChainDef* def, m2ChainId* chainOut);

    /// Destroys a chain and every segment of it at once: their contacts end
    /// and whatever rested on them wakes. Journaled.
    ///
    /// @param world    The world.
    /// @param chainId  The chain.
    /// @return m2_success; m2_errorStale when the chain is already gone;
    ///         m2_errorState from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2DestroyChain(m2World* world, m2ChainId chainId);

    /// Tells whether a chain id names a live chain. Counts no misuse.
    ///
    /// @param world    The world.
    /// @param chainId  The chain.
    /// @return True for a live chain.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Chain_IsValid(const m2World* world, m2ChainId chainId);

    /// Counts the segments a chain has, which is fewer than its def asked
    /// for when the shape pool ran out.
    ///
    /// @param world    The world.
    /// @param chainId  The chain; a stale id counts one misuse.
    /// @return The number of segments; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2Chain_GetSegmentCount(const m2World* world, m2ChainId chainId);

    static const m2ChainId m2_nullChainId = {0, 0};

    /// Returns a shape def: density 1, friction 0.6, no bounce, category
    /// 1, mask all, not a sensor.
    ///
    /// @return The default def.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2ShapeDef m2DefaultShapeDef(void);

    /// Builds a polygon from convex, counter-clockwise points, with
    /// thresholds relative to its size.
    ///
    /// @param points      The vertices.
    /// @param count       The number of points, 3 to 8.
    /// @param radius      The rounding radius, at least 0.
    /// @param polygonOut  Receives the polygon; left as it was on failure.
    /// @return m2_success; m2_errorInvalid when the points do not make a
    ///         valid polygon or an argument is null.
    /// @par Thread safety
    /// Safe from any thread.
    M2_NODISCARD M2_API m2Result m2MakePolygon(const m2Vec2* points, int32_t count, float radius,
                                               m2Polygon* polygonOut);

    /// Builds the convex hull of a loose point cloud: near points are
    /// welded, collinear ones merged, the rest wrapped in a deterministic
    /// order. The way from a sprite's points to a collision shape.
    ///
    /// @param points      The cloud.
    /// @param count       The number of points.
    /// @param radius      The rounding radius, at least 0.
    /// @param polygonOut  Receives the hull; left as it was on failure.
    /// @return m2_success; m2_errorInvalid when no hull remains (too few
    ///         points, all welded together, all collinear) or an argument
    ///         is null.
    /// @par Thread safety
    /// Safe from any thread.
    M2_NODISCARD M2_API m2Result m2ComputeHull(const m2Vec2* points, int32_t count, float radius,
                                               m2Polygon* polygonOut);

    /// Splits a simple counter-clockwise outline, without holes, into
    /// convex pieces of at most 8 vertices: the way from a sprite outline to
    /// the shapes of a destructible body. Pieces of near-zero area are
    /// dropped by validation; a clean outline loses nothing, and one that
    /// would lose every piece is refused.
    ///
    /// @param points    The outline, up to 64 points.
    /// @param count     The number of points.
    /// @param pieces    Receives up to capacity pieces; may be null to
    ///                  measure.
    /// @param capacity  The length of pieces.
    /// @param countOut  Receives the number of pieces, which may exceed
    ///                  capacity; left as it was on failure.
    /// @return m2_success; m2_errorInvalid for too few or too many points,
    ///         clockwise winding, an outline too thin for any piece to pass
    ///         validation, an outline that is not simple (an edge
    ///         crossing another, a vertex within 0.005 m of an edge it
    ///         does not end, a repeated vertex, an edge folding back over
    ///         its neighbor), a point beyond M2_HUGE or a null argument. It takes no world, so
    ///         a refusal counts nothing.
    /// @par Thread safety
    /// Safe from any thread.
    M2_NODISCARD M2_API m2Result m2DecomposeOutline(const m2Vec2* points, int32_t count,
                                                    m2Polygon* pieces, int32_t capacity,
                                                    int32_t* countOut);

    /// Breaks a dynamic body into one new dynamic body per piece, all at
    /// the parent's pose. Each takes the parent's velocity at its own
    /// center of mass and the parent's spin, and the material and filter of
    /// the parent's first shape. The parent is destroyed like any body: its
    /// joints go with it and touching sleepers wake. All or nothing: when
    /// the pools cannot seat every piece, nothing changes. Journaled as one
    /// operation.
    ///
    /// @param world       The world.
    /// @param bodyId      The dynamic body to break.
    /// @param pieces      The pieces, body-local polygons (pair with
    ///                    m2DecomposeOutline).
    /// @param pieceCount  The number of pieces, 1 to 64.
    /// @param bodiesOut   Receives up to capacity new bodies, in piece
    ///                    order; null ids on failure. May be null with
    ///                    capacity 0.
    /// @param capacity    The length of bodiesOut, not below 0.
    /// @param countOut    Receives the number of bodies made, which may
    ///                    exceed capacity; 0 on failure.
    /// @return m2_success; m2_errorStale for a stale body; m2_errorInvalid
    ///         for a body that is not dynamic, an invalid piece or piece
    ///         count, or a bad output argument; m2_errorCapacity when the
    ///         body or shape pool cannot seat every piece, which counts no
    ///         misuse; m2_errorState from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2World_ShatterBody(m2World* world, m2BodyId bodyId,
                                                     const m2Polygon* pieces, int32_t pieceCount,
                                                     m2BodyId* bodiesOut, int32_t capacity,
                                                     int32_t* countOut);

    /// Builds a box centered on the body origin.
    ///
    /// @param halfWidth   Half the width, above 0.
    /// @param halfHeight  Half the height, above 0.
    /// @return The box; a polygon with count 0 when the extents are not
    ///         positive and finite, which every create refuses.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2Polygon m2MakeBox(float halfWidth, float halfHeight);

    /// Attaches a circle to a body. A dynamic body recomputes its mass from
    /// all its shapes. Nothing is left half made on failure. Journaled.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param def       The shape, from m2DefaultShapeDef.
    /// @param circle    The geometry, in the body's frame.
    /// @param shapeOut  Receives the shape; the null id on failure.
    /// @return m2_success; m2_errorStale for a stale body; m2_errorInvalid
    ///         for a bad def or geometry or a null argument;
    ///         m2_errorCapacity when the shape pool is full; m2_errorState
    ///         from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateCircleShape(m2World* world, m2BodyId bodyId,
                                                     const m2ShapeDef* def, const m2Circle* circle,
                                                     m2ShapeId* shapeOut);

    /// Attaches a capsule to a body, as m2CreateCircleShape does.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param def       The shape, from m2DefaultShapeDef.
    /// @param capsule   The geometry, in the body's frame.
    /// @param shapeOut  Receives the shape; the null id on failure.
    /// @return As m2CreateCircleShape.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateCapsuleShape(m2World* world, m2BodyId bodyId,
                                                      const m2ShapeDef* def,
                                                      const m2Capsule* capsule,
                                                      m2ShapeId* shapeOut);

    /// Attaches a polygon to a body, as m2CreateCircleShape does.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param def       The shape, from m2DefaultShapeDef.
    /// @param polygon   The geometry, in the body's frame; its normals are
    ///                  recomputed from the vertices, not taken as given.
    /// @param shapeOut  Receives the shape; the null id on failure.
    /// @return As m2CreateCircleShape.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreatePolygonShape(m2World* world, m2BodyId bodyId,
                                                      const m2ShapeDef* def,
                                                      const m2Polygon* polygon,
                                                      m2ShapeId* shapeOut);

    /// Attaches a two-sided segment to a body, as m2CreateCircleShape does.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param def       The shape, from m2DefaultShapeDef.
    /// @param segment   The geometry, in the body's frame.
    /// @param shapeOut  Receives the shape; the null id on failure.
    /// @return As m2CreateCircleShape.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateSegmentShape(m2World* world, m2BodyId bodyId,
                                                      const m2ShapeDef* def,
                                                      const m2Segment* segment,
                                                      m2ShapeId* shapeOut);

    /// Tells whether a shape id names a live shape. Counts no misuse.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @return True for a live shape.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Shape_IsValid(const m2World* world, m2ShapeId shapeId);

    /// Destroys one shape: its touching contacts end (their end events
    /// arrive with the next step's), its pairs go, and its body's mass and
    /// center of mass are recomputed. Journaled.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @return m2_success; m2_errorStale when the shape is already gone;
    ///         m2_errorState from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2DestroyShape(m2World* world, m2ShapeId shapeId);

    /// Sets a shape's friction. It applies the next time its contacts are
    /// prepared. A stale id or an invalid value changes nothing and counts
    /// one misuse. Journaled.
    ///
    /// @param world     The world.
    /// @param shapeId   The shape.
    /// @param friction  The Coulomb friction, 0 to M2_MAX_COEFFICIENT.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetFriction(m2World* world, m2ShapeId shapeId, float friction);

    /// Sets a shape's restitution, as m2Shape_SetFriction does.
    ///
    /// @param world        The world.
    /// @param shapeId      The shape.
    /// @param restitution  The bounce, in [0, 1].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetRestitution(m2World* world, m2ShapeId shapeId, float restitution);

    /// Sets a shape's conveyor speed along the contact tangent. A pair
    /// slides at the sum of both shapes' speeds; positive drives a rider
    /// toward +x on an upward-facing floor. Riders wake. A stale id or a
    /// speed beyond M2_MAX_SPEED changes nothing and counts one misuse.
    /// Journaled.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @param speed    The surface speed, m/s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetTangentSpeed(m2World* world, m2ShapeId shapeId, float speed);

    /// Reads a shape's conveyor speed.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The surface speed; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Shape_GetTangentSpeed(const m2World* world, m2ShapeId shapeId);

    /// Sets a shape's collision filter. Its pairs are rebuilt at once, the
    /// ended ones with end events, and whatever touched it wakes. A stale
    /// id changes nothing and counts one misuse. Journaled.
    ///
    /// @param world         The world.
    /// @param shapeId       The shape.
    /// @param categoryBits  The categories the shape belongs to.
    /// @param maskBits      The categories it collides with.
    /// @param groupIndex    The group; see m2ShapeDef.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetFilter(m2World* world, m2ShapeId shapeId, uint64_t categoryBits,
                                  uint64_t maskBits, int32_t groupIndex);

    /// Reads a shape's friction.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The friction; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Shape_GetFriction(const m2World* world, m2ShapeId shapeId);

    /// Reads a shape's restitution.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The restitution; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Shape_GetRestitution(const m2World* world, m2ShapeId shapeId);

    /// Reads a shape's geometry type.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The type; m2_circleShape for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2ShapeType m2Shape_GetType(const m2World* world, m2ShapeId shapeId);

    /// Tells whether a shape is a sensor.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return True for a sensor; false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Shape_IsSensor(const m2World* world, m2ShapeId shapeId);

    /// Reads a shape's collision filter.
    ///
    /// @param world            The world.
    /// @param shapeId          The shape; a stale id counts one misuse and
    ///                         writes zeros.
    /// @param categoryBitsOut  Receives the categories; may be null.
    /// @param maskBitsOut      Receives the mask; may be null.
    /// @param groupIndexOut    Receives the group; may be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API void m2Shape_GetFilter(const m2World* world, m2ShapeId shapeId,
                                  uint64_t* categoryBitsOut, uint64_t* maskBitsOut,
                                  int32_t* groupIndexOut);

    /// Reads a circle shape's geometry, the exact stored bits in the body's
    /// frame.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id or a shape of another type
    ///                 counts one misuse.
    /// @return The circle; zeroed on a refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Circle m2Shape_GetCircle(const m2World* world, m2ShapeId shapeId);

    /// Reads a capsule shape's geometry, as m2Shape_GetCircle does.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id or a shape of another type
    ///                 counts one misuse.
    /// @return The capsule; zeroed on a refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Capsule m2Shape_GetCapsule(const m2World* world, m2ShapeId shapeId);

    /// Reads a polygon shape's geometry, as m2Shape_GetCircle does.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id or a shape of another type
    ///                 counts one misuse.
    /// @return The polygon; zeroed on a refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Polygon m2Shape_GetPolygon(const m2World* world, m2ShapeId shapeId);

    /// Reads a segment shape's geometry, as m2Shape_GetCircle does.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id or a shape of another type
    ///                 counts one misuse.
    /// @return The segment; zeroed on a refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Segment m2Shape_GetSegment(const m2World* world, m2ShapeId shapeId);

    /// Reads a chain link's geometry and ghosts, as m2Shape_GetCircle does.
    ///
    /// @param world    The world.
    /// @param shapeId  The link; a stale id or a shape of another type
    ///                 counts one misuse.
    /// @return The link; zeroed on a refusal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2ChainSegment m2Shape_GetChainSegment(const m2World* world, m2ShapeId shapeId);

    /// Reads a shape's density.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The density; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Shape_GetDensity(const m2World* world, m2ShapeId shapeId);

    /// Replaces a shape's geometry with a circle, changing its type if it
    /// has another. The body's mass recomputes, touching partners wake, and
    /// the broadphase refreshes. A stale id, invalid geometry or a chain
    /// link changes nothing and counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @param circle   The new geometry, in the body's frame.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetCircle(m2World* world, m2ShapeId shapeId, const m2Circle* circle);

    /// Replaces a shape's geometry with a capsule, as m2Shape_SetCircle
    /// does.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @param capsule  The new geometry, in the body's frame.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetCapsule(m2World* world, m2ShapeId shapeId, const m2Capsule* capsule);

    /// Replaces a shape's geometry with a polygon, as m2Shape_SetCircle
    /// does.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @param polygon  The new geometry, in the body's frame; its normals
    ///                 are recomputed from the vertices, not taken as given.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetPolygon(m2World* world, m2ShapeId shapeId, const m2Polygon* polygon);

    /// Replaces a shape's geometry with a segment, as m2Shape_SetCircle
    /// does.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @param segment  The new geometry, in the body's frame.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetSegment(m2World* world, m2ShapeId shapeId, const m2Segment* segment);

    /// Lists a body's shapes in ascending slot order.
    ///
    /// @param world     The world.
    /// @param bodyId    The body; a stale id counts one misuse.
    /// @param ids       Receives up to capacity shapes; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of shapes, which may exceed capacity; 0 for a
    ///         stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2Body_GetShapes(const m2World* world, m2BodyId bodyId, m2ShapeId* ids,
                                    int32_t capacity);

    /// Lists the world's chains in ascending slot order.
    ///
    /// @param world     The world.
    /// @param ids       Receives up to capacity chains; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of chains, which may exceed capacity.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetChains(const m2World* world, m2ChainId* ids, int32_t capacity);

    /// Lists a chain's links in ascending slot order.
    ///
    /// @param world     The world.
    /// @param chainId   The chain; a stale id counts one misuse.
    /// @param ids       Receives up to capacity shapes; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of links, which may exceed capacity; 0 for a
    ///         stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2Chain_GetShapes(const m2World* world, m2ChainId chainId, m2ShapeId* ids,
                                     int32_t capacity);

    /// Sets the friction of every link of a chain, as one journaled
    /// operation. It applies the next time the contacts are prepared. A
    /// stale id or an invalid value changes nothing and counts one misuse.
    ///
    /// @param world     The world.
    /// @param chainId   The chain.
    /// @param friction  The Coulomb friction, 0 to M2_MAX_COEFFICIENT.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Chain_SetFriction(m2World* world, m2ChainId chainId, float friction);

    /// Sets the restitution of every link of a chain, as
    /// m2Chain_SetFriction does.
    ///
    /// @param world        The world.
    /// @param chainId      The chain.
    /// @param restitution  The bounce, in [0, 1].
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Chain_SetRestitution(m2World* world, m2ChainId chainId, float restitution);

    /// Reads the body a shape is attached to.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The body; the null id for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2BodyId m2Shape_GetBody(const m2World* world, m2ShapeId shapeId);

    /// Reads the chain a link belongs to.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The chain; the null id for a shape outside any chain or a
    ///         stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2ChainId m2Shape_GetParentChain(const m2World* world, m2ShapeId shapeId);

    /// Computes a shape's tight world-space box, without the broadphase
    /// margin.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The box; zeroed for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2AabbResult m2Shape_GetAabb(const m2World* world, m2ShapeId shapeId);

    /// Tells whether a world point is inside a shape. A point within the
    /// engine's slop skin of the surface counts as inside, as overlaps do.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @param point    The world point.
    /// @return True when the point is inside; false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Shape_TestPoint(const m2World* world, m2ShapeId shapeId, m2Pos2 point);

    /// Finds the point of a shape's surface nearest a world point, radius
    /// included.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @param point    The world point.
    /// @return The nearest surface point; the origin for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2Shape_GetClosestPoint(const m2World* world, m2ShapeId shapeId, m2Pos2 point);

    /// Sets a shape's density; a dynamic body recomputes its mass. A stale
    /// id or a density outside 0 to M2_MAX_DENSITY changes nothing and
    /// counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape.
    /// @param density  The density, kg/m^2.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetDensity(m2World* world, m2ShapeId shapeId, float density);

    /// Sets a shape's opaque game data, kept through snapshots and journals.
    /// A stale id changes nothing and counts one misuse. Journaled.
    ///
    /// @param world     The world.
    /// @param shapeId   The shape.
    /// @param userData  Any value.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Shape_SetUserData(m2World* world, m2ShapeId shapeId, uint64_t userData);

    /// Reads a shape's game data.
    ///
    /// @param world    The world.
    /// @param shapeId  The shape; a stale id counts one misuse.
    /// @return The value; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2Shape_GetUserData(const m2World* world, m2ShapeId shapeId);

    /// Reads a body's mass, derived from its shapes' densities.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The mass; 0 for a body that is not dynamic or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetMass(const m2World* world, m2BodyId bodyId);

    static const m2ShapeId m2_nullShapeId = {0, 0};

    /// The filter of a query. A shape answers a query when each side's
    /// category intersects the other's mask, as contacts do.
    typedef struct m2QueryFilter
    {
        M2_ALIGN8 uint64_t categoryBits;
        M2_ALIGN8 uint64_t maskBits;
    } m2QueryFilter;

    /// Returns the default query filter, category 1 and mask all, which
    /// sees every shape whose mask includes category 1.
    ///
    /// @return The default filter.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2QueryFilter m2DefaultQueryFilter(void);

    /// The closest hit of a ray or sweep. Queries never change the world,
    /// and their results are canonical: on a tie the lower shape index
    /// wins.
    typedef struct m2RayCastResult
    {
        m2ShapeId shapeId; // the null id when hit is false
        m2Pos2 point;      // the world hit point; the origin on initial overlap
        m2Vec2 normal;     // the world surface normal; (0, 0) on initial overlap
        float fraction;    // the hit is origin + fraction * translation
        bool hit;
    } m2RayCastResult;

    /// Casts a ray and returns its closest hit. A chain link is hit only
    /// from its solid side.
    ///
    /// @param world        The world.
    /// @param origin       The start, in world space.
    /// @param translation  The ray, origin + t * translation for t in
    ///                     [0, 1].
    /// @param filter       Which shapes the ray sees.
    /// @return The closest hit; no hit for a non-finite argument, which
    ///         counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2RayCastResult m2World_CastRayClosest(const m2World* world, m2Pos2 origin,
                                                  m2Vec2 translation, m2QueryFilter filter);

    /// Casts a ray against one shape, with the conventions of
    /// m2World_CastRayClosest.
    ///
    /// @param world        The world.
    /// @param shapeId      The shape; a stale id counts one misuse.
    /// @param origin       The start, in world space.
    /// @param translation  The ray.
    /// @return The hit; no hit for a stale id or a non-finite argument.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2RayCastResult m2Shape_CastRay(const m2World* world, m2ShapeId shapeId, m2Pos2 origin,
                                           m2Vec2 translation);

    /// One hit of a ray or sweep that lists every hit.
    typedef struct m2RayHit
    {
        m2ShapeId shapeId;
        m2Pos2 point;
        m2Vec2 normal; // (0, 0) on initial overlap
        float fraction;
    } m2RayHit;

    /// Casts a ray and lists every hit in ascending fraction, ties to the
    /// lower shape index. When there are more hits than capacity, the
    /// closest are kept.
    ///
    /// @param world        The world.
    /// @param origin       The start, in world space.
    /// @param translation  The ray.
    /// @param hits         Receives up to capacity hits; may be null to
    ///                     measure.
    /// @param capacity     The length of hits.
    /// @param filter       Which shapes the ray sees.
    /// @return The number of hits, which may exceed capacity; 0 for a
    ///         non-finite argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_CastRayAll(const m2World* world, m2Pos2 origin, m2Vec2 translation,
                                      m2RayHit* hits, int32_t capacity, m2QueryFilter filter);

    /// Sweeps a circle and lists every hit, as m2World_CastRayAll does.
    ///
    /// @param world        The world.
    /// @param circle       The circle, in its own frame.
    /// @param origin       The circle's pose at the start.
    /// @param translation  The sweep.
    /// @param hits         Receives up to capacity hits; may be null to
    ///                     measure.
    /// @param capacity     The length of hits.
    /// @param filter       Which shapes the sweep sees.
    /// @return The number of hits, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_CastCircleAll(const m2World* world, const m2Circle* circle,
                                         m2Transform origin, m2Vec2 translation, m2RayHit* hits,
                                         int32_t capacity, m2QueryFilter filter);

    /// Sweeps a capsule and lists every hit, as m2World_CastRayAll does.
    ///
    /// @param world        The world.
    /// @param capsule      The capsule, in its own frame.
    /// @param origin       The capsule's pose at the start.
    /// @param translation  The sweep.
    /// @param hits         Receives up to capacity hits; may be null to
    ///                     measure.
    /// @param capacity     The length of hits.
    /// @param filter       Which shapes the sweep sees.
    /// @return The number of hits, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_CastCapsuleAll(const m2World* world, const m2Capsule* capsule,
                                          m2Transform origin, m2Vec2 translation, m2RayHit* hits,
                                          int32_t capacity, m2QueryFilter filter);

    /// Sweeps a polygon and lists every hit, as m2World_CastRayAll does.
    ///
    /// @param world        The world.
    /// @param polygon      The polygon, in its own frame.
    /// @param origin       The polygon's pose at the start.
    /// @param translation  The sweep.
    /// @param hits         Receives up to capacity hits; may be null to
    ///                     measure.
    /// @param capacity     The length of hits.
    /// @param filter       Which shapes the sweep sees.
    /// @return The number of hits, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_CastPolygonAll(const m2World* world, const m2Polygon* polygon,
                                          m2Transform origin, m2Vec2 translation, m2RayHit* hits,
                                          int32_t capacity, m2QueryFilter filter);

    /// A plane touching a character mover, from m2World_CollideMover.
    typedef struct m2MoverPlane
    {
        m2ShapeId shapeId;
        m2Vec2 normal;    // from the shape toward the mover
        float separation; // the gap along the normal; negative is overlap
        m2Pos2 point;     // the closest point of the shape's surface
    } m2MoverPlane;

/// Planes past this count take no part in m2SolveMover.
#define M2_MOVER_PLANES 16

    /// The result of m2SolveMover.
    typedef struct m2MoverMove
    {
        m2Vec2 translation;
        uint32_t pressed; // bit i: plane i stops part of the wish
    } m2MoverMove;

    /// Gathers the planes touching a posed capsule, one per nearby shape,
    /// in ascending shape order. A collar of four linear slops lets a
    /// controller see a wall before it reaches it. A chain link counts only
    /// from its solid side. Use m2World_CastCapsuleClosest for the sweep.
    ///
    /// @param world     The world.
    /// @param mover     The capsule, in its own frame.
    /// @param origin    The capsule's pose.
    /// @param planes    Receives up to capacity planes; may be null to
    ///                  measure.
    /// @param capacity  The length of planes.
    /// @param filter    Which shapes the mover sees.
    /// @return The number of planes, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_CollideMover(const m2World* world, const m2Capsule* mover,
                                        m2Transform origin, m2MoverPlane* planes, int32_t capacity,
                                        m2QueryFilter filter);

    /// Turns a wished translation into the closest one no plane blocks, and
    /// reports which planes it rests on.
    ///
    /// @param wish    The translation the character wants.
    /// @param planes  The planes from m2World_CollideMover.
    /// @param count   The number of planes; only the first M2_MOVER_PLANES
    ///                take part.
    /// @return The translation to apply and the planes that stopped part of
    ///         the wish.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2MoverMove m2SolveMover(m2Vec2 wish, const m2MoverPlane* planes, int32_t count);

    /// Removes the part of a velocity that points into the planes a move
    /// rests on.
    ///
    /// @param velocity  The character's velocity.
    /// @param planes    The planes given to m2SolveMover.
    /// @param count     Their number.
    /// @param pressed   The pressed bits m2SolveMover returned.
    /// @return The clipped velocity.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2Vec2 m2ClipMoverVelocity(m2Vec2 velocity, const m2MoverPlane* planes, int32_t count,
                                      uint32_t pressed);

    /// Sweeps a circle and returns its closest hit, ties to the lower shape
    /// index. A sweep that starts on a chain link's ghost side passes
    /// through it. An initial overlap reports fraction 0 and a zero normal,
    /// as rays do.
    ///
    /// @param world        The world.
    /// @param circle       The circle, in its own frame.
    /// @param origin       The circle's pose at the start.
    /// @param translation  The sweep.
    /// @param filter       Which shapes the sweep sees.
    /// @return The closest hit; no hit for an invalid argument, which counts
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2RayCastResult m2World_CastCircleClosest(const m2World* world, const m2Circle* circle,
                                                     m2Transform origin, m2Vec2 translation,
                                                     m2QueryFilter filter);

    /// Sweeps a capsule and returns its closest hit, as
    /// m2World_CastCircleClosest does. A character controller's sweep.
    ///
    /// @param world        The world.
    /// @param capsule      The capsule, in its own frame.
    /// @param origin       The capsule's pose at the start.
    /// @param translation  The sweep.
    /// @param filter       Which shapes the sweep sees.
    /// @return The closest hit; no hit for an invalid argument, which counts
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2RayCastResult m2World_CastCapsuleClosest(const m2World* world,
                                                      const m2Capsule* capsule, m2Transform origin,
                                                      m2Vec2 translation, m2QueryFilter filter);

    /// Sweeps a polygon and returns its closest hit, as
    /// m2World_CastCircleClosest does.
    ///
    /// @param world        The world.
    /// @param polygon      The polygon, in its own frame.
    /// @param origin       The polygon's pose at the start.
    /// @param translation  The sweep.
    /// @param filter       Which shapes the sweep sees.
    /// @return The closest hit; no hit for an invalid argument, which counts
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2RayCastResult m2World_CastPolygonClosest(const m2World* world,
                                                      const m2Polygon* polygon, m2Transform origin,
                                                      m2Vec2 translation, m2QueryFilter filter);

    /// Lists the live shapes touching a posed circle, in ascending slot
    /// order. Touching within the engine's slop skin counts. A chain link
    /// counts only from its solid side.
    ///
    /// @param world     The world.
    /// @param circle    The circle, in its own frame.
    /// @param origin    The circle's pose.
    /// @param ids       Receives up to capacity shapes; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @param filter    Which shapes the query sees.
    /// @return The number of shapes, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_OverlapCircle(const m2World* world, const m2Circle* circle,
                                         m2Transform origin, m2ShapeId* ids, int32_t capacity,
                                         m2QueryFilter filter);

    /// Lists the live shapes touching a posed capsule, as
    /// m2World_OverlapCircle does.
    ///
    /// @param world     The world.
    /// @param capsule   The capsule, in its own frame.
    /// @param origin    The capsule's pose.
    /// @param ids       Receives up to capacity shapes; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @param filter    Which shapes the query sees.
    /// @return The number of shapes, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_OverlapCapsule(const m2World* world, const m2Capsule* capsule,
                                          m2Transform origin, m2ShapeId* ids, int32_t capacity,
                                          m2QueryFilter filter);

    /// Lists the live shapes touching a posed polygon, as
    /// m2World_OverlapCircle does.
    ///
    /// @param world     The world.
    /// @param polygon   The polygon, in its own frame.
    /// @param origin    The polygon's pose.
    /// @param ids       Receives up to capacity shapes; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @param filter    Which shapes the query sees.
    /// @return The number of shapes, which may exceed capacity; 0 for an
    ///         invalid argument, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_OverlapPolygon(const m2World* world, const m2Polygon* polygon,
                                          m2Transform origin, m2ShapeId* ids, int32_t capacity,
                                          m2QueryFilter filter);

    /// Lists the live shapes whose tight box overlaps a world box, in
    /// ascending slot order.
    ///
    /// @param world     The world.
    /// @param lower     The box's lower corner.
    /// @param upper     The box's upper corner.
    /// @param results   Receives up to capacity shapes; may be null to
    ///                  measure.
    /// @param capacity  The length of results.
    /// @param filter    Which shapes the query sees.
    /// @return The number of shapes, which may exceed capacity; 0 for a
    ///         non-finite or inverted box, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_OverlapAabb(const m2World* world, m2Pos2 lower, m2Pos2 upper,
                                       m2ShapeId* results, int32_t capacity, m2QueryFilter filter);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_SHAPE_H
