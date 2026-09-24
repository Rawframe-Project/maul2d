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

    typedef struct m2ShapeId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m2ShapeId;

    /// All geometry is authored in body-local space, in 32-bit floats.
    typedef struct m2Circle
    {
        m2Vec2 center;
        float radius;
    } m2Circle;

    typedef struct m2Capsule
    {
        m2Vec2 point1;
        m2Vec2 point2;
        float radius;
    } m2Capsule;

    /// Convex, counter-clockwise, at most 8 vertices. radius > 0 makes a
    /// rounded polygon. Build via m2MakePolygon
    /// or m2MakeBox so normals and validity are computed for you.
    typedef struct m2Polygon
    {
        m2Vec2 vertices[M2_MAX_POLYGON_VERTICES];
        m2Vec2 normals[M2_MAX_POLYGON_VERTICES];
        int32_t count;
        float radius;
    } m2Polygon;

    typedef struct m2Segment
    {
        m2Vec2 point1;
        m2Vec2 point2;
    } m2Segment;

    /// One link of a chain: the collidable segment plus the neighbor
    /// ghost points that keep bodies from snagging on internal seams.
    /// Built by m2CreateChain; collision is one-sided (solid on the
    /// right when walking point1 -> point2).
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

    typedef struct m2ShapeDef
    {
        float density;      // kg/m^2; dynamic bodies get a minimum-mass floor
        float friction;     // Coulomb; pairs mix by geometric mean
        float tangentSpeed; // conveyor: surface slides along the contact tangent, m/s
        float restitution;  // bounce in [0,1]; pairs mix by maximum
        /// Collision filtering: shapes collide when each one's category
        /// intersects the other's mask. Defaults: category 1, mask all.
        /// Queries ignore filters for now (a query filter parameter is
        /// a recorded pending).
        uint32_t categoryBits;
        uint32_t maskBits;
        /// Same non-zero group on both shapes overrides the mask rule:
        /// positive always collides, negative never does. Zero defers
        /// to categories and masks. Queries ignore groups.
        int32_t groupIndex;
        /// Sensors detect overlap without ever pushing back: no solver
        /// response, no bullet blocking, no effect on sleep or islands.
        /// Overlaps arrive through m2World_GetSensorEvents. Two sensors
        /// never detect each other.
        bool isSensor;
        uint64_t userData;
        int32_t internalValue;
    } m2ShapeDef;

    /// A chain of one-sided segments with seam-smoothing ghosts.
    /// Open chains need count >= 4: the first and last points are the
    /// ghosts and the chain collides along points[1..count-2]. Loops
    /// need count >= 3 and wrap. Cloned; the array may be temporary.
    typedef struct m2ChainId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world0;
        uint16_t generation;
    } m2ChainId;

    typedef struct m2ChainDef
    {
        const m2Vec2* points;
        int32_t count;
        bool isLoop;
        float friction;
        float restitution;
        uint32_t categoryBits;
        uint32_t maskBits;
        int32_t groupIndex;
        uint64_t userData;
        int32_t internalValue;
    } m2ChainDef;

    M2_API m2ChainDef m2DefaultChainDef(void);

    /// Creates the chain's segment shapes on the body and returns the
    /// chain's id (null on failure). The id names the whole group:
    /// m2DestroyChain removes every segment at once, ends their
    /// contacts, and wakes whoever was resting on them. Destroying the
    /// body also retires the chain id. Journaled. Thread class: writer.
    M2_API m2ChainId m2CreateChain(m2BodyId bodyId, const m2ChainDef* def);
    M2_API void m2DestroyChain(m2ChainId chainId);
    M2_API bool m2Chain_IsValid(m2ChainId chainId);
    M2_API int32_t m2Chain_GetSegmentCount(m2ChainId chainId);

    static const m2ChainId m2_nullChainId = {0, 0, 0};

    M2_API m2ShapeDef m2DefaultShapeDef(void);

    /// Validated constructors. Thresholds are relative to the shape's
    /// size; a returned polygon with count == 0 means the input was
    /// refused.
    M2_API m2Polygon m2MakePolygon(const m2Vec2* points, int32_t count, float radius);

    /// Convex hull of a loose point cloud (welding, collinear
    /// merging, deterministic quickhull): the doorway from sprite
    /// outlines to collision shapes. Degenerate input returns a
    /// polygon with count == 0, the loud-invalid convention.
    M2_API m2Polygon m2ComputeHull(const m2Vec2* points, int32_t count, float radius);

    /// Split a simple counter-clockwise outline (up to 64 points, no
    /// self-intersections, no holes) into convex pieces of at most 8
    /// vertices each: the road from a sprite outline to destructible
    /// bodies. Fills up to capacity pieces and returns the truthful
    /// total (the enumeration contract). Refuses invalid input (too
    /// few or too many points, clockwise winding, self-intersection,
    /// non-finite coordinates) by returning 0. Near-zero-area
    /// sliver pieces are welded away by validation and skipped; clean
    /// outlines lose nothing. Pure math, no world required.
    /// Thread class: reader (pure).
    M2_API int32_t m2DecomposeOutline(const m2Vec2* points, int32_t count, m2Polygon* pieces,
                                      int32_t capacity);

    /// Break a dynamic body into one new dynamic body per piece, all
    /// at the parent's pose, each inheriting the parent's rigid
    /// velocity field evaluated at its own center of mass (v + w x r)
    /// plus the parent's spin, and the material and filter of the
    /// parent's first shape. Pieces are body-local polygons (pair
    /// with m2DecomposeOutline). The parent is destroyed: its joints
    /// die with it and touching sleepers wake, the ordinary destroy
    /// road. All-or-nothing: if body or shape capacity cannot seat
    /// every piece the call returns 0 and changes nothing (a full
    /// pool is a runtime fact). One journal op replays the whole
    /// shatter. Fills outBodies up to capacity, returns the piece
    /// count. Thread class: writer.
    M2_API int32_t m2World_ShatterBody(m2BodyId bodyId, const m2Polygon* pieces, int32_t pieceCount,
                                       m2BodyId* outBodies, int32_t capacity);
    M2_API m2Polygon m2MakeBox(float halfWidth, float halfHeight);

    /// Attach a shape to a body. Validation failure or exhausted capacity
    /// returns the null id; nothing is half-constructed. Dynamic bodies
    /// recompute mass from all attached shapes (explicit override comes
    /// with the solver slice). Thread class: writer.
    M2_API m2ShapeId m2CreateCircleShape(m2BodyId bodyId, const m2ShapeDef* def,
                                         const m2Circle* circle);
    M2_API m2ShapeId m2CreateCapsuleShape(m2BodyId bodyId, const m2ShapeDef* def,
                                          const m2Capsule* capsule);
    M2_API m2ShapeId m2CreatePolygonShape(m2BodyId bodyId, const m2ShapeDef* def,
                                          const m2Polygon* polygon);
    M2_API m2ShapeId m2CreateSegmentShape(m2BodyId bodyId, const m2ShapeDef* def,
                                          const m2Segment* segment);

    M2_API bool m2Shape_IsValid(m2ShapeId shapeId);

    /// Destroys one shape: touching contacts end (bookended into the
    /// next step's events), pairs are pruned, and the owning body's
    /// mass and center of mass are recomputed. Thread class: writer.
    M2_API void m2DestroyShape(m2ShapeId shapeId);

    /// Runtime material and filter tuning, journaled. Material changes
    /// apply the next time the contact is prepared; filter changes
    /// rebuild the shape's pairs immediately (ends are bookended) and
    /// wake whoever was touching it. Thread class: writer / reader.
    M2_API void m2Shape_SetFriction(m2ShapeId shapeId, float friction);
    M2_API void m2Shape_SetRestitution(m2ShapeId shapeId, float restitution);
    /// Conveyor surface speed along the contact tangent; the pair
    /// value is the SUM of both shapes. Journaled.
    M2_API void m2Shape_SetTangentSpeed(m2ShapeId shapeId, float speed);
    M2_API float m2Shape_GetTangentSpeed(m2ShapeId shapeId);
    M2_API void m2Shape_SetFilter(m2ShapeId shapeId, uint32_t categoryBits, uint32_t maskBits,
                                  int32_t groupIndex);
    M2_API float m2Shape_GetFriction(m2ShapeId shapeId);
    M2_API float m2Shape_GetRestitution(m2ShapeId shapeId);
    M2_API m2ShapeType m2Shape_GetType(m2ShapeId shapeId);
    M2_API bool m2Shape_IsSensor(m2ShapeId shapeId);
    /// Reads the collision filter; any out pointer may be NULL.
    M2_API void m2Shape_GetFilter(m2ShapeId shapeId, uint32_t* categoryBits, uint32_t* maskBits,
                                  int32_t* groupIndex);

    /// Geometry readback for editors and gizmos: the getter must
    /// match the shape's type (checked loudly). Returned structs are
    /// the exact stored bits, in the shape's body-local frame.
    M2_API m2Circle m2Shape_GetCircle(m2ShapeId shapeId);
    M2_API m2Capsule m2Shape_GetCapsule(m2ShapeId shapeId);
    M2_API m2Polygon m2Shape_GetPolygon(m2ShapeId shapeId);
    M2_API m2Segment m2Shape_GetSegment(m2ShapeId shapeId);
    M2_API m2ChainSegment m2Shape_GetChainSegment(m2ShapeId shapeId);
    M2_API float m2Shape_GetDensity(m2ShapeId shapeId);

    /// Runtime geometry: replace a shape's geometry in place, type
    /// changes included. The owner's mass recomputes, touching
    /// partners wake (a floor shrinking under a sleeper is a
    /// teleport-class change), and the broadphase refreshes.
    /// Journaled. Thread class: writer.
    M2_API void m2Shape_SetCircle(m2ShapeId shapeId, const m2Circle* circle);
    M2_API void m2Shape_SetCapsule(m2ShapeId shapeId, const m2Capsule* capsule);
    M2_API void m2Shape_SetPolygon(m2ShapeId shapeId, const m2Polygon* polygon);
    M2_API void m2Shape_SetSegment(m2ShapeId shapeId, const m2Segment* segment);

    /// Enumeration walks, ascending slot order, truthful totals
    /// (same contract as m2World_OverlapAabb). Thread class: reader.
    M2_API int32_t m2Body_GetShapes(m2BodyId bodyId, m2ShapeId* ids, int32_t capacity);
    M2_API int32_t m2World_GetChains(m2WorldId worldId, m2ChainId* ids, int32_t capacity);
    M2_API int32_t m2Chain_GetShapes(m2ChainId chainId, m2ShapeId* ids, int32_t capacity);

    /// Runtime chain materials: applied to every link at once, one
    /// journal op each; takes effect at the next contact prepare,
    /// like the per-shape material setters. Thread class: writer.
    M2_API m2WorldId m2Chain_GetWorld(m2ChainId chainId);
    M2_API void m2Chain_SetFriction(m2ChainId chainId, float friction);
    M2_API void m2Chain_SetRestitution(m2ChainId chainId, float restitution);
    M2_API m2BodyId m2Shape_GetBody(m2ShapeId shapeId);
    M2_API m2WorldId m2Shape_GetWorld(m2ShapeId shapeId);
    M2_API m2ChainId m2Shape_GetParentChain(m2ShapeId shapeId); // null if free-standing
    M2_API m2AabbResult m2Shape_GetAabb(m2ShapeId shapeId);     // tight, world space

    /// Point and ray queries against ONE shape. TestPoint counts
    /// touching within the engine's slop skin (the overlap law);
    /// GetClosestPoint returns the surface point nearest to the
    /// query, radius included; RayCast follows the world ray
    /// conventions including the one-sided chain law.
    M2_API bool m2Shape_TestPoint(m2ShapeId shapeId, m2Pos2 point);
    M2_API m2Pos2 m2Shape_GetClosestPoint(m2ShapeId shapeId, m2Pos2 point);
    M2_API void m2Shape_SetDensity(m2ShapeId shapeId, float density); // journaled, mass recomputes
    M2_API void m2Shape_SetUserData(m2ShapeId shapeId, uint64_t userData); // journaled
    M2_API uint64_t m2Shape_GetUserData(m2ShapeId shapeId);

    /// Body mass derived from attached shape densities (0 for non-dynamic).
    M2_API float m2Body_GetMass(m2BodyId bodyId);

    static const m2ShapeId m2_nullShapeId = {0, 0, 0};

    /// Query filtering mirrors contact filtering: a shape answers a
    /// query when each side's category intersects the other's mask.
    /// The default filter (category 1, mask all) sees every shape
    /// whose mask includes category 1.
    typedef struct m2QueryFilter
    {
        uint32_t categoryBits;
        uint32_t maskBits;
    } m2QueryFilter;

    M2_API m2QueryFilter m2DefaultQueryFilter(void);

    /// Queries are read-only: they never touch simulation state, and
    /// their results are canonical (closest hit with lowest-shape-index
    /// tie break; overlap lists in ascending creation order).
    typedef struct m2RayCastResult
    {
        m2ShapeId shapeId; // null when hit is false
        m2Pos2 point;      // world hit point (ray origin on initial overlap)
        m2Vec2 normal;     // world surface normal ((0,0) on initial overlap)
        float fraction;    // hit = origin + fraction * translation
        bool hit;
    } m2RayCastResult;

    /// Closest hit along origin + t * translation, t in [0, 1].
    /// Thread class: reader.
    M2_API m2RayCastResult m2World_CastRayClosest(m2WorldId worldId, m2Pos2 origin,
                                                  m2Vec2 translation, m2QueryFilter filter);
    M2_API m2RayCastResult m2Shape_CastRay(m2ShapeId shapeId, m2Pos2 origin, m2Vec2 translation);

    /// Every hit along a ray or sweep, not just the first: results
    /// arrive in ascending fraction order (ties break to the lower
    /// shape index) and the return value is the TRUE total even when
    /// it exceeds capacity; when it does, the closest hits are the
    /// ones kept. Chain segments stay one-sided. Thread class: reader.
    typedef struct m2RayHit
    {
        m2ShapeId shapeId;
        m2Pos2 point;
        m2Vec2 normal; // (0,0) on initial overlap
        float fraction;
    } m2RayHit;

    M2_API int32_t m2World_CastRayAll(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation,
                                      m2RayHit* hits, int32_t capacity, m2QueryFilter filter);
    M2_API int32_t m2World_CastCircleAll(m2WorldId worldId, const m2Circle* circle,
                                         m2Transform origin, m2Vec2 translation, m2RayHit* hits,
                                         int32_t capacity, m2QueryFilter filter);
    M2_API int32_t m2World_CastCapsuleAll(m2WorldId worldId, const m2Capsule* capsule,
                                          m2Transform origin, m2Vec2 translation, m2RayHit* hits,
                                          int32_t capacity, m2QueryFilter filter);
    M2_API int32_t m2World_CastPolygonAll(m2WorldId worldId, const m2Polygon* polygon,
                                          m2Transform origin, m2Vec2 translation, m2RayHit* hits,
                                          int32_t capacity, m2QueryFilter filter);

    /// The character mover kit. m2World_CollideMover gathers the
    /// planes touching a posed capsule: one per nearby shape, the normal
    /// from the shape toward the mover, the separation along it
    /// (negative means overlap), in ascending shape order, with the
    /// one-sided chain rule and a truthful total. m2SolveMover turns a
    /// wished translation into the closest one that no plane blocks and
    /// reports which planes it rests on; m2ClipMoverVelocity strips the
    /// velocity that points into those planes. For the sweep itself,
    /// m2World_CastCapsuleClosest is the mover cast. Thread class:
    /// reader (the solver and the clip are pure math).
    typedef struct m2MoverPlane
    {
        m2ShapeId shapeId;
        m2Vec2 normal;    // from the shape toward the mover
        float separation; // gap along the normal; negative = overlap
        m2Pos2 point;     // closest point on the shape's surface
    } m2MoverPlane;

/// Planes past this count take no part in the solve.
#define M2_MOVER_PLANES 16

    typedef struct m2MoverMove
    {
        m2Vec2 translation;
        uint32_t pressed; // bit i: plane i stops part of the wish
    } m2MoverMove;

    M2_API int32_t m2World_CollideMover(m2WorldId worldId, const m2Capsule* mover,
                                        m2Transform origin, m2MoverPlane* planes, int32_t capacity,
                                        m2QueryFilter filter);
    M2_API m2MoverMove m2SolveMover(m2Vec2 wish, const m2MoverPlane* planes, int32_t count);
    M2_API m2Vec2 m2ClipMoverVelocity(m2Vec2 velocity, const m2MoverPlane* planes, int32_t count,
                                      uint32_t pressed);

    /// Convex sweeps: the given shape (in its own local frame, posed
    /// by origin) slides along translation; the closest hit wins and
    /// ties break to the lower shape index. Chain segments stay
    /// one-sided: sweeps starting on the ghost side pass through.
    /// Initial overlap reports fraction 0 with a zero normal, like
    /// rays. Thread class: reader.
    M2_API m2RayCastResult m2World_CastCircleClosest(m2WorldId worldId, const m2Circle* circle,
                                                     m2Transform origin, m2Vec2 translation,
                                                     m2QueryFilter filter);
    M2_API m2RayCastResult m2World_CastCapsuleClosest(m2WorldId worldId, const m2Capsule* capsule,
                                                      m2Transform origin, m2Vec2 translation,
                                                      m2QueryFilter filter);
    M2_API m2RayCastResult m2World_CastPolygonClosest(m2WorldId worldId, const m2Polygon* polygon,
                                                      m2Transform origin, m2Vec2 translation,
                                                      m2QueryFilter filter);

    /// Convex overlaps: live shapes touching the posed shape, in
    /// ascending slot order with a truthful total (the OverlapAabb
    /// contract). Chain segments are one-sided here too. Thread
    /// class: reader.
    M2_API int32_t m2World_OverlapCircle(m2WorldId worldId, const m2Circle* circle,
                                         m2Transform origin, m2ShapeId* ids, int32_t capacity,
                                         m2QueryFilter filter);
    M2_API int32_t m2World_OverlapCapsule(m2WorldId worldId, const m2Capsule* capsule,
                                          m2Transform origin, m2ShapeId* ids, int32_t capacity,
                                          m2QueryFilter filter);
    M2_API int32_t m2World_OverlapPolygon(m2WorldId worldId, const m2Polygon* polygon,
                                          m2Transform origin, m2ShapeId* ids, int32_t capacity,
                                          m2QueryFilter filter);

    /// Fills results with up to capacity alive shapes whose tight AABB
    /// overlaps [lower, upper], ascending shape order. Returns the total
    /// number of overlapping shapes even when it exceeds capacity.
    /// Thread class: reader.
    M2_API int32_t m2World_OverlapAabb(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper,
                                       m2ShapeId* results, int32_t capacity, m2QueryFilter filter);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_SHAPE_H
