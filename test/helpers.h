// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shorthands for the suites: each wraps one public call that returns a
// status and hands back what it made, the null id, 0 or false on any
// failure. A test that checks why a call refuses calls the public
// function and reads its status instead.

#ifndef MAUL2D_TEST_HELPERS_H
#define MAUL2D_TEST_HELPERS_H

#include "maul2d/maul2d.h"

static inline m2World* NewWorld(const m2WorldDef* def)
{
    m2World* world = NULL;
    m2Result result = m2CreateWorld(def, &world);
    (void)result;
    return world;
}

static inline m2BodyId NewBody(m2World* world, const m2BodyDef* def)
{
    m2BodyId id = m2_nullBodyId;
    m2Result result = m2CreateBody(world, def, &id);
    (void)result;
    return id;
}

static inline m2ShapeId NewCircleShape(m2World* world, m2BodyId body, const m2ShapeDef* def,
                                       const m2Circle* circle)
{
    m2ShapeId id = m2_nullShapeId;
    m2Result result = m2CreateCircleShape(world, body, def, circle, &id);
    (void)result;
    return id;
}

static inline m2ShapeId NewCapsuleShape(m2World* world, m2BodyId body, const m2ShapeDef* def,
                                        const m2Capsule* capsule)
{
    m2ShapeId id = m2_nullShapeId;
    m2Result result = m2CreateCapsuleShape(world, body, def, capsule, &id);
    (void)result;
    return id;
}

static inline m2ShapeId NewPolygonShape(m2World* world, m2BodyId body, const m2ShapeDef* def,
                                        const m2Polygon* polygon)
{
    m2ShapeId id = m2_nullShapeId;
    m2Result result = m2CreatePolygonShape(world, body, def, polygon, &id);
    (void)result;
    return id;
}

static inline m2ShapeId NewSegmentShape(m2World* world, m2BodyId body, const m2ShapeDef* def,
                                        const m2Segment* segment)
{
    m2ShapeId id = m2_nullShapeId;
    m2Result result = m2CreateSegmentShape(world, body, def, segment, &id);
    (void)result;
    return id;
}

static inline m2ChainId NewChain(m2World* world, m2BodyId body, const m2ChainDef* def)
{
    m2ChainId id = m2_nullChainId;
    m2Result result = m2CreateChain(world, body, def, &id);
    (void)result;
    return id;
}

#define M2_TEST_NEW_JOINT(kind)                                                                    \
    static inline m2JointId New##kind##Joint(m2World* world, const m2##kind##JointDef* def)        \
    {                                                                                              \
        m2JointId id = m2_nullJointId;                                                             \
        m2Result result = m2Create##kind##Joint(world, def, &id);                                  \
        (void)result;                                                                              \
        return id;                                                                                 \
    }

M2_TEST_NEW_JOINT(Distance)
M2_TEST_NEW_JOINT(Revolute)
M2_TEST_NEW_JOINT(Prismatic)
M2_TEST_NEW_JOINT(Weld)
M2_TEST_NEW_JOINT(Wheel)
M2_TEST_NEW_JOINT(Filter)
M2_TEST_NEW_JOINT(Gear)
M2_TEST_NEW_JOINT(Pulley)
M2_TEST_NEW_JOINT(Ratchet)
M2_TEST_NEW_JOINT(Motor)
M2_TEST_NEW_JOINT(Mouse)

#undef M2_TEST_NEW_JOINT

static inline m2FluidVolumeId NewFluidVolume(m2World* world, const m2FluidVolumeDef* def)
{
    m2FluidVolumeId id = m2_nullFluidVolumeId;
    m2Result result = m2CreateFluidVolume(world, def, &id);
    (void)result;
    return id;
}

static inline m2ParticleId EmitParticle(m2World* world, m2Pos2 position, m2Vec2 velocity,
                                        uint32_t flags)
{
    m2ParticleId id = m2_nullParticleId;
    m2Result result = m2World_EmitParticle(world, position, velocity, flags, &id);
    (void)result;
    return id;
}

// The destroys return their status for the tests that compare it; the
// others may drop it.
static inline m2Result DestroyBody(m2World* world, m2BodyId id)
{
    return m2DestroyBody(world, id);
}

static inline m2Result DestroyShape(m2World* world, m2ShapeId id)
{
    return m2DestroyShape(world, id);
}

static inline m2Result DestroyChain(m2World* world, m2ChainId id)
{
    return m2DestroyChain(world, id);
}

static inline m2Result DestroyJoint(m2World* world, m2JointId id)
{
    return m2DestroyJoint(world, id);
}

static inline m2Result DestroyParticle(m2World* world, m2ParticleId id)
{
    return m2DestroyParticle(world, id);
}

static inline m2Result DestroyFluidVolume(m2World* world, m2FluidVolumeId id)
{
    return m2DestroyFluidVolume(world, id);
}

// A polygon with count 0 when the points do not make one.
static inline m2Polygon MakePolygon(const m2Vec2* points, int32_t count, float radius)
{
    m2Polygon polygon = {0};
    m2Result result = m2MakePolygon(points, count, radius, &polygon);
    (void)result;
    return polygon;
}

static inline m2Polygon ComputeHull(const m2Vec2* points, int32_t count, float radius)
{
    m2Polygon polygon = {0};
    m2Result result = m2ComputeHull(points, count, radius, &polygon);
    (void)result;
    return polygon;
}

// The piece total, or 0 for an outline the call refuses.
static inline int32_t DecomposeOutline(const m2Vec2* points, int32_t count, m2Polygon* pieces,
                                       int32_t capacity)
{
    int32_t total = 0;
    m2Result result = m2DecomposeOutline(points, count, pieces, capacity, &total);
    (void)result;
    return total;
}

// The bytes written, or 0 on any failure.
static inline int32_t Snapshot(const m2World* world, void* buffer, int32_t capacity)
{
    int32_t bytes = 0;
    return m2World_Snapshot(world, buffer, capacity, &bytes) == m2_success ? bytes : 0;
}

static inline bool Restore(m2World* world, const void* buffer, int32_t size)
{
    return m2World_Restore(world, buffer, size) == m2_success;
}

static inline bool StartJournal(m2World* world, void* buffer, int32_t capacity)
{
    return m2World_StartJournal(world, buffer, capacity) == m2_success;
}

// The tape's size, or 0 when it overflowed or nothing was recording.
static inline int32_t StopJournal(m2World* world)
{
    int32_t bytes = 0;
    m2Result result = m2World_StopJournal(world, &bytes);
    (void)result;
    return bytes;
}

static inline int32_t ShatterBody(m2World* world, m2BodyId bodyId, const m2Polygon* pieces,
                                  int32_t pieceCount, m2BodyId* bodiesOut, int32_t capacity)
{
    int32_t made = 0;
    m2Result result =
        m2World_ShatterBody(world, bodyId, pieces, pieceCount, bodiesOut, capacity, &made);
    (void)result;
    return made;
}

static inline bool ReplayJournal(m2World* world, const void* data, int32_t size)
{
    return m2World_ReplayJournal(world, data, size) == m2_success;
}

#endif // MAUL2D_TEST_HELPERS_H
