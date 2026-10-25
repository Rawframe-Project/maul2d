// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The per-element rules of an incoming snapshot (state_checks.h). The
// range rules call the validators the defs, the setters and replay use
// (core.h), so a value one path accepts every path accepts.

#include "state_checks.h"

#include "core.h"
#include "dynamic_tree.h"
#include "geometry.h"

#include "maul2d/base.h"
#include "maul2d/joint.h"

#include <string.h>

static int32_t ReadInt32(const uint8_t* bytes)
{
    int32_t value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

static bool InRange(int64_t value, int64_t lo, int64_t hi)
{
    return value >= lo && value <= hi;
}

static bool CheckIndices(const uint8_t* data, size_t count, int64_t lo, int64_t range)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (!InRange(ReadInt32(data + 4 * i), lo, range - 1))
        {
            return false;
        }
    }
    return true;
}

static bool CheckBytes(const uint8_t* data, size_t count, uint8_t max)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (data[i] > max)
        {
            return false;
        }
    }
    return true;
}

static bool CheckGeometry(const uint8_t* data, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2ShapeGeometry g;
        memcpy(&g, data + i * sizeof(g), sizeof(g));
        if (!InRange(g.type, m2_circleShape, m2_chainSegmentShape) ||
            (g.type == m2_polygonShape && !InRange(g.polygon.count, 3, M2_MAX_POLYGON_VERTICES)))
        {
            return false;
        }
    }
    return true;
}

// A point count that fits, finite geometry, and the accumulated impulses
// within M2_IMPULSE_LIMIT, where the solver holds them.
static bool CheckManifolds(const uint8_t* data, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2Manifold m;
        memcpy(&m, data + i * sizeof(m), sizeof(m));
        if (!InRange(m.pointCount, 0, 2) || !m2FiniteVec2(m.normal))
        {
            return false;
        }
        for (int32_t k = 0; k < 2; ++k)
        {
            const m2ManifoldPoint* p = &m.points[k];
            if (!m2FiniteVec2(p->anchorA) || !m2FiniteVec2(p->anchorB) ||
                !m2FiniteF(p->separation) || !m2ValidImpulse(p->normalImpulse) ||
                !m2ValidImpulse(p->tangentImpulse))
            {
                return false;
            }
        }
    }
    return true;
}

static bool CheckPairKeys(const uint8_t* data, size_t count, int64_t shapes)
{
    for (size_t i = 0; i < count; ++i)
    {
        uint64_t key;
        memcpy(&key, data + i * sizeof(key), sizeof(key));
        if ((int64_t)(key >> 32) >= shapes || (int64_t)(key & 0xFFFFFFFFu) >= shapes)
        {
            return false;
        }
    }
    return true;
}

static bool CheckTrees(const uint8_t* data, size_t bytes, int64_t nodes)
{
    for (size_t at = 0; at + sizeof(m2DynamicTree) <= bytes; at += sizeof(m2DynamicTree))
    {
        m2DynamicTree tree;
        memcpy(&tree, data + at, sizeof(tree));
        if (!InRange(tree.root, -1, nodes - 1) || !InRange(tree.freeList, -1, nodes - 1) ||
            !InRange(tree.nodeCount, 0, nodes) || tree.nodeCapacity != nodes)
        {
            return false;
        }
    }
    return true;
}

static bool CheckTreeNodes(const uint8_t* data, size_t count, int64_t nodes, int64_t shapes)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2TreeNode node;
        memcpy(&node, data + i * sizeof(node), sizeof(node));
        if (!InRange(node.child1, -1, nodes - 1) || !InRange(node.child2, -1, nodes - 1) ||
            !InRange(node.parent, -1, nodes - 1) || !InRange(node.userData, -1, shapes - 1))
        {
            return false;
        }
    }
    return true;
}

static bool CheckFinite(const uint8_t* data, size_t bytes, size_t width)
{
    for (size_t at = 0; at + width <= bytes; at += width)
    {
        bool finite;
        if (width == sizeof(float))
        {
            float f;
            memcpy(&f, data + at, sizeof(f));
            finite = m2FiniteF(f);
        }
        else
        {
            double d;
            memcpy(&d, data + at, sizeof(d));
            finite = m2FiniteD(d);
        }
        if (!finite)
        {
            return false;
        }
    }
    return true;
}

// Every float of the block passes valid.
static bool CheckFloats(const uint8_t* data, size_t bytes, bool (*valid)(float))
{
    for (size_t at = 0; at + sizeof(float) <= bytes; at += sizeof(float))
    {
        float f;
        memcpy(&f, data + at, sizeof(f));
        if (!valid(f))
        {
            return false;
        }
    }
    return true;
}

// One axis of an acceleration, through the vector's own validator.
static bool AccelerationAxisValid(float x)
{
    return m2ValidAcceleration((m2Vec2){x, 0.0f});
}

// The last step's 1 / substep: 0 before the first step, else
// substepCount / dt for a step m2World_Step accepts. Dividing by a power
// of two is exact, so the largest a step stores rounds to exactly the
// bound.
static bool InverseStepValid(float x)
{
    return x == 0.0f ||
           m2WithinF(x, 1.0f / M2_MAX_TIME_STEP, (float)M2_MAX_SUBSTEPS / M2_MIN_TIME_STEP);
}

static bool CheckRange(const uint8_t* data, size_t bytes, m2Check check)
{
    switch (check)
    {
    case m2_check_speed:
        return CheckFloats(data, bytes, m2ValidSpeed);
    case m2_check_acceleration:
        return CheckFloats(data, bytes, AccelerationAxisValid);
    case m2_check_gravityScale:
        return CheckFloats(data, bytes, m2ValidGravityScale);
    case m2_check_rate:
        return CheckFloats(data, bytes, m2ValidRate);
    case m2_check_coefficient:
        return CheckFloats(data, bytes, m2ValidCoefficient);
    case m2_check_restitution:
        return CheckFloats(data, bytes, m2ValidRestitution);
    case m2_check_density:
        return CheckFloats(data, bytes, m2ValidDensity);
    case m2_check_force:
        return CheckFloats(data, bytes, m2ValidForce);
    case m2_check_maxForce:
        return CheckFloats(data, bytes, m2ValidMaxForce);
    case m2_check_impulse:
        return CheckFloats(data, bytes, m2ValidImpulse);
    case m2_check_breakLimit:
        return CheckFloats(data, bytes, m2ValidBreakLimit);
    case m2_check_inverseStep:
        return CheckFloats(data, bytes, InverseStepValid);
    default:
        return false;
    }
}

static bool CheckLocal(const uint8_t* data, size_t bytes)
{
    for (size_t at = 0; at + sizeof(float) <= bytes; at += sizeof(float))
    {
        float f;
        memcpy(&f, data + at, sizeof(f));
        if (!(f >= -M2_HUGE && f <= M2_HUGE))
        {
            return false;
        }
    }
    return true;
}

static bool CheckTransforms(const uint8_t* data, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2Transform xf;
        memcpy(&xf, data + i * sizeof(xf), sizeof(xf));
        if (!m2FinitePos2(xf.p) || !m2UnitRot(xf.q))
        {
            return false;
        }
    }
    return true;
}

bool m2StateCheckElements(const uint8_t* data, size_t count, size_t elementSize, m2Check check,
                          m2StateBounds bounds)
{
    int64_t range = bounds.range;
    switch (check)
    {
    case m2_check_none:
        return true;
    case m2_check_index:
        return CheckIndices(data, count, 0, range);
    case m2_check_indexOrNone:
        return CheckIndices(data, count, -1, range);
    case m2_check_count:
        return InRange(ReadInt32(data), 0, range);
    case m2_check_cursor:
        return InRange(ReadInt32(data), 0, range > 0 ? range - 1 : 0);
    case m2_check_bool:
        return CheckBytes(data, count, 1);
    case m2_check_bodyType:
        return CheckBytes(data, count, (uint8_t)m2_dynamicBody);
    case m2_check_jointType:
        return CheckBytes(data, count, (uint8_t)m2_ratchetJoint);
    case m2_check_geometry:
        return CheckGeometry(data, count);
    case m2_check_manifold:
        return CheckManifolds(data, count);
    case m2_check_pairKey:
        return CheckPairKeys(data, count, range);
    case m2_check_tree:
        return CheckTrees(data, elementSize, range);
    case m2_check_treeNode:
        return CheckTreeNodes(data, count, range, bounds.shapes);
    case m2_check_finite32:
        return CheckFinite(data, count * elementSize, sizeof(float));
    case m2_check_finite64:
        return CheckFinite(data, count * elementSize, sizeof(double));
    case m2_check_local:
        return CheckLocal(data, count * elementSize);
    case m2_check_transform:
        return CheckTransforms(data, count);
    default:
        return CheckRange(data, count * elementSize, check);
    }
}
