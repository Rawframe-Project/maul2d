// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle-free water: buoyancy volumes. A box gates which awake
// dynamic bodies are in the water, by their origin; a horizontal
// surface line decides how much of each shape lies below it.
//
// The water pushes up with the weight it displaces (Archimedes), at the
// centroid of the submerged area, and drags on that centroid's motion
// relative to the flow; spin is braked by a torque proportional to the
// submerged area times the body's squared radius of gyration. The
// forces go into the body accumulators, so they integrate beside
// gravity and die with the step, and a body that settles at the
// waterline and sleeps simply floats.
//
// Circles use the exact circular segment. Polygons are clipped by the
// surface. Rounded polygons and capsules become polygons first: each
// rounded corner turns into a fan whose inner vertices sit just outside
// the circle, far enough that the fan has exactly the area of its
// circular sector, so the outline keeps the shape's true area. Segments
// have no area and do not float.

#include "buoyancy.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>

// acos(x) = atan2(sqrt(1 - x^2), x), on the engine's own atan2 (libm's
// acos is not bit-identical across platforms).
static float Acos(float x)
{
    float s = 1.0f - x * x;
    s = s > 0.0f ? sqrtf(s) : 0.0f;
    return m2Atan2(s, x);
}

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

// What of a shape lies below the surface: area and world centroid.
typedef struct Immersion
{
    float area;
    m2Vec2 centroid;
} Immersion;

static const Immersion s_dry = {0.0f, {0.0f, 0.0f}};

// A circle against the surface. With the center at depth d below the
// line, the dry cap above it has area r^2 acos(d/r) - d sqrt(r^2 - d^2)
// and its centroid sits (2/3) (r^2 - d^2)^(3/2) / area above the
// center; the wet part balances it.
static Immersion ImmerseCircle(m2Vec2 center, float r, double surface)
{
    float d = (float)(surface - (double)center.y);
    if (d <= -r)
    {
        return s_dry;
    }
    float disc = (float)M2_PI * r * r;
    if (d >= r)
    {
        return (Immersion){disc, center};
    }
    float half = sqrtf(r * r - d * d);
    float dryArea = r * r * Acos(d / r) - d * half;
    float wetArea = disc - dryArea;
    if (!(wetArea > 0.0f))
    {
        return s_dry;
    }
    float dryRise = dryArea > 1.0e-9f ? (2.0f / 3.0f) * half * half * half / dryArea : 0.0f;
    return (Immersion){wetArea, {center.x, center.y - dryRise * dryArea / wetArea}};
}

// Area and centroid of a simple polygon, from the signed triangle fan
// about the origin.
static Immersion PolygonMoments(const m2Vec2* v, int32_t n)
{
    float twice = 0.0f;
    float sx = 0.0f;
    float sy = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        m2Vec2 a = v[i];
        m2Vec2 b = v[i + 1 < n ? i + 1 : 0];
        float cross = a.x * b.y - b.x * a.y;
        twice += cross;
        sx += (a.x + b.x) * cross;
        sy += (a.y + b.y) * cross;
    }
    if (!(twice > 2.0e-9f || twice < -2.0e-9f))
    {
        return s_dry;
    }
    float area = 0.5f * twice;
    return (Immersion){area < 0.0f ? -area : area, {sx / (3.0f * twice), sy / (3.0f * twice)}};
}

// The part of a convex outline below the surface (Sutherland and
// Hodgman against one half-plane).
#define M2_OUTLINE_MAX 64

static Immersion ImmerseOutline(const m2Vec2* v, int32_t n, double surface)
{
    m2Vec2 wet[M2_OUTLINE_MAX + 2];
    int32_t count = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        m2Vec2 a = v[i];
        m2Vec2 b = v[i + 1 < n ? i + 1 : 0];
        bool aWet = (double)a.y <= surface;
        bool bWet = (double)b.y <= surface;
        if (aWet)
        {
            wet[count++] = a;
        }
        if (aWet != bWet)
        {
            float t = (float)((surface - (double)a.y) / ((double)b.y - (double)a.y));
            wet[count++] = (m2Vec2){a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
        }
    }
    return count >= 3 ? PolygonMoments(wet, count) : s_dry;
}

// The radius for the inner vertices of a corner fan. The fan runs m
// steps of angle a from one edge normal to the next, its two end
// vertices on the circle (radius r, so the straight edges stay exact)
// and the m - 1 inner ones at radius R. Its area, r R sin a for the two
// end triangles plus (m - 2) R^2 sin(a) / 2 for the rest, equals the
// sector's m a r^2 / 2 when
//   (m - 2) sin(a) R^2 + 2 r sin(a) R - m a r^2 = 0.
static float FanRadius(float r, int32_t m, float a, float sinA)
{
    if (m == 2)
    {
        return r * a / sinA;
    }
    float k = (float)(m - 2) * sinA;
    return r * (sqrtf(sinA * sinA + k * (float)m * a) - sinA) / k;
}

// The outline of a rounded convex polygon in the world: each corner fans
// from the previous edge's normal to the next one's in at least two
// steps of at most a sixteenth of a turn, keeping the shape's area.
static int32_t RoundedOutline(const m2Vec2* verts, const m2Vec2* normals, int32_t count,
                              float radius, m2Transform xf, m2Vec2* out)
{
    int32_t n = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 from = normals[i > 0 ? i - 1 : count - 1];
        m2Vec2 to = normals[i];
        float turn = m2Atan2(from.x * to.y - from.y * to.x, from.x * to.x + from.y * to.y);
        turn = turn < 0.0f ? turn + 2.0f * (float)M2_PI : turn; // a half turn may read as -pi
        int32_t steps = (int32_t)ceilf(turn * (8.0f / (float)M2_PI));
        steps = steps < 2 ? 2 : steps;
        m2Rot rot = m2MakeRot(turn / (float)steps);
        float inner = FanRadius(radius, steps, turn / (float)steps, rot.s);
        m2Vec2 dir = from;
        for (int32_t k = 0; k <= steps && n < M2_OUTLINE_MAX; ++k)
        {
            float reach = k == 0 || k == steps ? radius : inner;
            m2Vec2 local = {verts[i].x + reach * dir.x, verts[i].y + reach * dir.y};
            m2Vec2 world = Rotate(xf.q, local);
            out[n++] = (m2Vec2){world.x + (float)xf.p.x, world.y + (float)xf.p.y};
            dir = k + 1 == steps ? to : Rotate(rot, dir);
        }
    }
    return n;
}

static Immersion ImmerseShape(const m2World* world, int32_t shape, m2Transform xf, double surface)
{
    const m2ShapeGeometry* g = &world->shapes.shapeGeometry[shape];
    m2Vec2 outline[M2_OUTLINE_MAX];
    int32_t n = 0;
    if (g->type == m2_circleShape)
    {
        m2Vec2 c = Rotate(xf.q, g->circle.center);
        return ImmerseCircle((m2Vec2){c.x + (float)xf.p.x, c.y + (float)xf.p.y}, g->circle.radius,
                             surface);
    }
    if (g->type == m2_capsuleShape)
    {
        m2Polygon core =
            m2MakeSegmentProxy(g->capsule.point1, g->capsule.point2, g->capsule.radius);
        n = RoundedOutline(core.vertices, core.normals, 2, core.radius, xf, outline);
    }
    else if (g->type == m2_polygonShape && g->polygon.radius > 0.0f)
    {
        n = RoundedOutline(g->polygon.vertices, g->polygon.normals, g->polygon.count,
                           g->polygon.radius, xf, outline);
    }
    else if (g->type == m2_polygonShape)
    {
        for (int32_t i = 0; i < g->polygon.count; ++i)
        {
            m2Vec2 w = Rotate(xf.q, g->polygon.vertices[i]);
            outline[n++] = (m2Vec2){w.x + (float)xf.p.x, w.y + (float)xf.p.y};
        }
    }
    return n >= 3 ? ImmerseOutline(outline, n, surface) : s_dry;
}

m2FluidVolumeDef m2DefaultFluidVolumeDef(void)
{
    m2FluidVolumeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 2.0f;
    def.linearDrag = 1.0f;
    def.angularDrag = 0.5f;
    def.internalValue = M2_FVOLUME_COOKIE;
    return def;
}

m2FluidVolumeId m2World_CreateFluidVolume(m2WorldId worldId, const m2FluidVolumeDef* def)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_FVOLUME_COOKIE ||
        world->volumes.fvCapacity == 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullFluidVolumeId;
    }
    if (!(def->density >= 0.0f) || !(def->linearDrag >= 0.0f) || !(def->angularDrag >= 0.0f) ||
        !m2FiniteD(def->surface))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullFluidVolumeId;
    }
    if (world->volumes.fvFreeCount == 0)
    {
        return m2_nullFluidVolumeId; // pool full: a runtime fact
    }
    int32_t index = world->volumes.fvFreeQueue[world->volumes.fvFreeHead];
    world->volumes.fvFreeHead = (world->volumes.fvFreeHead + 1) % world->volumes.fvCapacity;
    world->volumes.fvFreeCount -= 1;
    if (index + 1 > world->volumes.maxFvIndex)
    {
        world->volumes.maxFvIndex = index + 1;
    }
    world->volumes.fvLower[index] = def->regionLower;
    world->volumes.fvUpper[index] = def->regionUpper;
    world->volumes.fvSurface[index] = def->surface;
    world->volumes.fvDensity[index] = def->density;
    world->volumes.fvLinearDrag[index] = def->linearDrag;
    world->volumes.fvAngularDrag[index] = def->angularDrag;
    world->volumes.fvFlow[index] = def->flow;
    world->volumes.fvUserData[index] = def->userData;
    world->volumes.fvAlive[index] = 1;
    m2FluidVolumeId id = {index + 1, world->idWorld, world->volumes.fvGenerations[index]};
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateFluidVolume record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;
        m2JournalRecord(world, m2_opCreateFluidVolume, &record, (int32_t)sizeof(record));
    }
    return id;
}

static int32_t FvSlot(const m2World* world, m2FluidVolumeId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || index < 0 || index >= world->volumes.fvCapacity ||
        world->volumes.fvAlive[index] == 0 || world->volumes.fvGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

void m2World_DestroyFluidVolume(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromTag(volumeId.world);
    int32_t index = FvSlot(world, volumeId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2JournalRecord(world, m2_opDestroyFluidVolume, &volumeId, (int32_t)sizeof(volumeId));
    }
    world->volumes.fvAlive[index] = 0;
    world->volumes.fvGenerations[index] += 1;
    world->volumes.fvFreeQueue[(world->volumes.fvFreeHead + world->volumes.fvFreeCount) %
                               world->volumes.fvCapacity] = index;
    world->volumes.fvFreeCount += 1;
}

bool m2FluidVolume_IsValid(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromTag(volumeId.world);
    return FvSlot(world, volumeId) >= 0;
}

void m2FluidVolume_SetSurface(m2FluidVolumeId volumeId, double surface)
{
    m2World* world = m2WorldFromTag(volumeId.world);
    int32_t index = FvSlot(world, volumeId);
    if (index < 0 || !m2FiniteD(surface))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpFluidSurface record;
        memset(&record, 0, sizeof(record));
        record.id = volumeId;
        record.surface = surface;
        m2JournalRecord(world, m2_opSetFluidSurface, &record, (int32_t)sizeof(record));
    }
    world->volumes.fvSurface[index] = surface;
}

double m2FluidVolume_GetSurface(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromTag(volumeId.world);
    int32_t index = FvSlot(world, volumeId);
    return index >= 0 ? world->volumes.fvSurface[index] : 0.0;
}

uint64_t m2FluidVolume_GetUserData(m2FluidVolumeId volumeId)
{
    m2World* world = m2WorldFromTag(volumeId.world);
    int32_t index = FvSlot(world, volumeId);
    return index >= 0 ? world->volumes.fvUserData[index] : 0;
}

static bool InWater(const m2World* world, int32_t b, int32_t v)
{
    if (world->bodies.alive[b] == 0 || world->bodies.types[b] != (uint8_t)m2_dynamicBody ||
        world->bodies.asleep[b] != 0 || world->bodies.disabled[b] != 0)
    {
        return false;
    }
    m2Pos2 p = world->bodies.transforms[b].p;
    m2Pos2 lo = world->volumes.fvLower[v];
    m2Pos2 hi = world->volumes.fvUpper[v];
    return p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y;
}

// Lift, drag and spin drag of one volume on one body.
static void Soak(m2World* world, int32_t b, int32_t vol)
{
    m2Transform xf = world->bodies.transforms[b];
    Immersion wet = s_dry;
    for (int32_t s = world->bodies.bodyShapeHead[b]; s != -1; s = world->shapes.shapeNext[s])
    {
        Immersion part = ImmerseShape(world, s, xf, world->volumes.fvSurface[vol]);
        wet.area += part.area;
        wet.centroid.x += part.area * part.centroid.x;
        wet.centroid.y += part.area * part.centroid.y;
    }
    if (wet.area <= 1.19209290e-7f)
    {
        return;
    }
    m2Vec2 center = Rotate(xf.q, world->bodies.localCenters[b]);
    m2Vec2 arm = {wet.centroid.x / wet.area - ((float)xf.p.x + center.x),
                  wet.centroid.y / wet.area - ((float)xf.p.y + center.y)};
    float w = world->bodies.angularVelocities[b];
    m2Vec2 v = world->bodies.linearVelocities[b];
    m2Vec2 flow = world->volumes.fvFlow[vol];
    float lift = -world->volumes.fvDensity[vol] * wet.area;
    float drag = -world->volumes.fvLinearDrag[vol] * wet.area;
    float fx = lift * world->gravity.x + drag * (v.x - w * arm.y - flow.x);
    float fy = lift * world->gravity.y + drag * (v.y + w * arm.x - flow.y);
    world->bodies.forces[b].x += fx;
    world->bodies.forces[b].y += fy;
    world->bodies.torques[b] += arm.x * fy - arm.y * fx;
    float invM = world->bodies.invMass[b];
    float invI = world->bodies.invInertia[b];
    if (invM > 0.0f && invI > 0.0f)
    {
        // I / m is the squared radius of gyration.
        world->bodies.torques[b] -=
            (invM / invI) * wet.area * w * world->volumes.fvAngularDrag[vol];
    }
}

// The per-step pass, volume by volume, bodies in slot order.
void m2ApplyFluidVolumes(m2World* world, float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }
    for (int32_t v = 0; v < world->volumes.maxFvIndex; ++v)
    {
        for (int32_t b = 0; world->volumes.fvAlive[v] != 0 && b < world->bodies.maxBodyIndex; ++b)
        {
            if (InWater(world, b, v))
            {
                Soak(world, b, v);
            }
        }
    }
}

// Global wind: an area-weighted linear drag toward the world wind
// velocity, into the same force accumulators as gravity and buoyancy,
// in canonical body order. Runs on the FULL shape area (wind acts in
// air, not on a submerged part) and only on dynamic awake enabled
// bodies; frozen bodies skip it exactly as they skip gravity. Uniform
// wind is applied at the center of mass, so it adds no spurious torque.
// The world gates on a positive drag before calling this (opt-in).
void m2ApplyWind(m2World* world, float dt)
{
    if (dt <= 0.0f || !(world->windLinearDrag > 0.0f))
    {
        return;
    }
    float drag = world->windLinearDrag;
    m2Vec2 wind = world->windVelocity;
    for (int32_t b = 0; b < world->bodies.maxBodyIndex; ++b)
    {
        if (world->bodies.alive[b] == 0 || world->bodies.types[b] != (uint8_t)m2_dynamicBody ||
            world->bodies.asleep[b] != 0 || world->bodies.disabled[b] != 0)
        {
            continue;
        }
        // Full shape area is rotation invariant; mass at unit density
        // reuses the tested area math (circles, rounded polygons, caps).
        float area = 0.0f;
        for (int32_t s = world->bodies.bodyShapeHead[b]; s != -1; s = world->shapes.shapeNext[s])
        {
            area += m2ComputeShapeMass(&world->shapes.shapeGeometry[s], 1.0f).mass;
        }
        if (!(area > 0.0f))
        {
            continue;
        }
        m2Vec2 v = world->bodies.linearVelocities[b];
        world->bodies.forces[b].x += -drag * area * (v.x - wind.x);
        world->bodies.forces[b].y += -drag * area * (v.y - wind.y);
    }
}
