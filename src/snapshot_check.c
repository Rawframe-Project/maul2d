// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The structural check of an incoming snapshot, run on its bytes in place
// before any of them land. m2StateValidate has already checked each array
// alone (every index within its pool, every flag a flag); this checks how
// the arrays fit together, so that no later call or step can index past
// a pool, loop forever or overflow a fixed stack:
//   - each free queue holds every dead slot that is not retired, once;
//   - each body's shape list holds exactly its live shapes, with no cycle;
//   - shapes, chains and joints point at live owners, and particle
//     springs and triads at live particles;
//   - each live joint's length, limits and reference hold what its kind
//     keeps there, in the ranges its def and setters give them;
//   - each body's mass data is what its shapes give it, bit for bit, or
//     an override m2Body_SetMassData would have accepted;
//   - each broadphase tree is a balanced tree whose leaves are exactly the
//     live shapes of enabled bodies of its type, with boxes that contain
//     their children, and its free list holds every other node;
//   - the moved list and its flags agree, and the pairs are ordered and
//     join live shapes in the broadphase;
//   - each shape of a static or sleeping body lies inside its fat box at
//     the body's pose: the step refits no such box before it, or a host,
//     queries it.
// The bytes may be unaligned, so every read goes through memcpy. Every
// index read is bounded here too, though m2StateValidate already put each
// one within its pool, so the check stands on its own. The only memory
// written is the world's restoreMarks scratch.

#include "snapshot_check.h"

#include "core.h"
#include "dynamic_tree.h"
#include "geometry.h"
#include "joint.h"
#include "mass.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <string.h>

// --- Reading the incoming bytes ---------------------------------------------

static bool InPool(int32_t index, int32_t capacity)
{
    return index >= 0 && index < capacity;
}

static int32_t ReadI32(const uint8_t* array, int32_t index)
{
    int32_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static float ReadF32(const uint8_t* array, int32_t index)
{
    float value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static uint32_t ReadU32(const uint8_t* array, int32_t index)
{
    uint32_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static uint64_t ReadU64(const uint8_t* array, int32_t index)
{
    uint64_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static m2TreeNode ReadNode(const uint8_t* nodes, int32_t index)
{
    m2TreeNode node;
    memcpy(&node, nodes + (size_t)index * sizeof(node), sizeof(node));
    return node;
}

// The incoming copy of a field; every field this file reads is snapshot
// state in every world that has its pool.
#define M2_IN(field) m2StateIncoming(world, in, (const void*)&world->field)

// --- Free queues ------------------------------------------------------------

// One pool's slots and its ring of free slots, as the incoming bytes have
// them. A slot whose generation is exhausted retires: it stays out of the
// ring and is counted.
typedef struct Pool
{
    const uint8_t* alive;       // uint8 per slot
    const uint8_t* generations; // uint32 per slot
    const uint8_t* queue;       // int32 per slot, a ring
    int32_t capacity;
    int32_t head;
    int32_t count;
    bool hasTail; // whether the pool keeps a tail cursor
    int32_t tail; // read only when hasTail
    int32_t retired;
    int32_t highWater;
} Pool;

// Every live slot lies below the high-water mark, and the ring holds each
// dead, unretired slot exactly once. Returns the live count, or -1.
static int32_t CheckPool(uint8_t* marks, const Pool* pool)
{
    if (pool->highWater < 0 || pool->highWater > pool->capacity)
    {
        return -1;
    }
    int32_t live = 0;
    int32_t exhausted = 0;
    for (int32_t i = 0; i < pool->capacity; ++i)
    {
        if (pool->alive[i] != 0)
        {
            live += 1;
            if (i >= pool->highWater)
            {
                return -1;
            }
        }
        else if (ReadU32(pool->generations, i) == M2_LAST_GENERATION)
        {
            exhausted += 1;
        }
    }
    int32_t cursors = pool->capacity > 0 ? pool->capacity : 1;
    if (!InPool(pool->head, cursors) || pool->count < 0 || exhausted != pool->retired ||
        live + pool->count + pool->retired != pool->capacity ||
        (pool->hasTail &&
         (!InPool(pool->tail, cursors) || pool->tail != (pool->head + pool->count) % cursors)))
    {
        return -1;
    }
    memset(marks, 0, (size_t)pool->capacity);
    for (int32_t k = 0; k < pool->count; ++k)
    {
        int32_t slot = ReadI32(pool->queue, (pool->head + k) % pool->capacity);
        if (!InPool(slot, pool->capacity))
        {
            return -1;
        }
        bool retiredSlot = ReadU32(pool->generations, slot) == M2_LAST_GENERATION;
        if (pool->alive[slot] != 0 || retiredSlot || marks[slot] != 0)
        {
            return -1;
        }
        marks[slot] = 1;
    }
    return live;
}

static int32_t InlineI32(const m2World* world, const uint8_t* in, const void* field)
{
    return ReadI32(m2StateIncoming(world, in, field), 0);
}

// Each end of `count` links, `ends` arrays of particle slots, is a live
// particle.
static bool LinksJoinLive(const m2World* world, const uint8_t* alive, const uint8_t* const* ends,
                          int32_t endCount, int32_t count, int32_t capacity)
{
    if (count < 0 || count > capacity)
    {
        return false;
    }
    for (int32_t k = 0; k < count; ++k)
    {
        for (int32_t e = 0; e < endCount; ++e)
        {
            int32_t p = ReadI32(ends[e], k);
            if (!InPool(p, world->particles.particleCapacity) || alive[p] == 0)
            {
                return false;
            }
        }
    }
    return true;
}

// Springs and triads join live particles only, as a particle's death
// leaves them: one with a dead end would push that slot's velocity and
// latch onto the next particle emitted there.
static bool CheckParticleLinks(const m2World* world, const uint8_t* in)
{
    const uint8_t* alive = M2_IN(particles.particleAlive);
    const uint8_t* springs[2] = {M2_IN(particles.particleSpringA),
                                 M2_IN(particles.particleSpringB)};
    const uint8_t* triads[3] = {M2_IN(particles.particleTriadA), M2_IN(particles.particleTriadB),
                                M2_IN(particles.particleTriadC)};
    return LinksJoinLive(world, alive, springs, 2,
                         InlineI32(world, in, &world->particles.particleSpringCount),
                         world->particles.particleSpringCapacity) &&
           LinksJoinLive(world, alive, triads, 3,
                         InlineI32(world, in, &world->particles.particleTriadCount),
                         world->particles.particleTriadCapacity);
}

// The particle and fluid volume pools, which exist only in worlds that
// have them.
static bool CheckFluidPools(m2World* world, const uint8_t* in)
{
    uint8_t* marks = world->restoreMarks;
    if (world->particles.particleCapacity > 0)
    {
        const Pool particles = {M2_IN(particles.particleAlive),
                                M2_IN(particles.particleGenerations),
                                M2_IN(particles.particleFreeQueue),
                                world->particles.particleCapacity,
                                InlineI32(world, in, &world->particles.particleFreeHead),
                                InlineI32(world, in, &world->particles.particleFreeCount),
                                false,
                                0,
                                InlineI32(world, in, &world->particles.particleRetiredCount),
                                InlineI32(world, in, &world->particles.maxParticleIndex)};
        int32_t live = CheckPool(marks, &particles);
        if (live < 0 || live != InlineI32(world, in, &world->particles.particleCount) ||
            !CheckParticleLinks(world, in))
        {
            return false;
        }
    }
    if (world->volumes.fvCapacity > 0)
    {
        const Pool volumes = {M2_IN(volumes.fvAlive),
                              M2_IN(volumes.fvGenerations),
                              M2_IN(volumes.fvFreeQueue),
                              world->volumes.fvCapacity,
                              InlineI32(world, in, &world->volumes.fvFreeHead),
                              InlineI32(world, in, &world->volumes.fvFreeCount),
                              false,
                              0,
                              InlineI32(world, in, &world->volumes.fvRetiredCount),
                              InlineI32(world, in, &world->volumes.maxFvIndex)};
        if (CheckPool(marks, &volumes) < 0)
        {
            return false;
        }
    }
    return true;
}

static bool CheckPools(m2World* world, const uint8_t* in)
{
    uint8_t* marks = world->restoreMarks;
    const Pool bodies = {M2_IN(bodies.alive),
                         M2_IN(bodies.generations),
                         M2_IN(bodies.freeQueue),
                         world->bodies.bodyCapacity,
                         InlineI32(world, in, &world->bodies.freeHead),
                         InlineI32(world, in, &world->bodies.freeCount),
                         true,
                         InlineI32(world, in, &world->bodies.freeTail),
                         InlineI32(world, in, &world->bodies.retiredCount),
                         InlineI32(world, in, &world->bodies.maxBodyIndex)};
    const Pool shapes = {M2_IN(shapes.shapeAlive),
                         M2_IN(shapes.shapeGenerations),
                         M2_IN(shapes.shapeFreeQueue),
                         world->shapes.shapeCapacity,
                         InlineI32(world, in, &world->shapes.shapeFreeHead),
                         InlineI32(world, in, &world->shapes.shapeFreeCount),
                         true,
                         InlineI32(world, in, &world->shapes.shapeFreeTail),
                         InlineI32(world, in, &world->shapes.shapeRetiredCount),
                         InlineI32(world, in, &world->shapes.maxShapeIndex)};
    const Pool joints = {M2_IN(joints.jointAlive),
                         M2_IN(joints.jointGenerations),
                         M2_IN(joints.jointFreeQueue),
                         world->joints.jointCapacity,
                         InlineI32(world, in, &world->joints.jointFreeHead),
                         InlineI32(world, in, &world->joints.jointFreeCount),
                         true,
                         InlineI32(world, in, &world->joints.jointFreeTail),
                         InlineI32(world, in, &world->joints.jointRetiredCount),
                         InlineI32(world, in, &world->joints.maxJointIndex)};
    const Pool chains = {M2_IN(chains.chainAlive),
                         M2_IN(chains.chainGenerations),
                         M2_IN(chains.chainFreeQueue),
                         world->shapes.shapeCapacity,
                         InlineI32(world, in, &world->chains.chainFreeHead),
                         InlineI32(world, in, &world->chains.chainFreeCount),
                         true,
                         InlineI32(world, in, &world->chains.chainFreeTail),
                         InlineI32(world, in, &world->chains.chainRetiredCount),
                         InlineI32(world, in, &world->chains.maxChainIndex)};
    if (CheckPool(marks, &bodies) < 0 || CheckPool(marks, &shapes) < 0 ||
        CheckPool(marks, &joints) < 0 || CheckPool(marks, &chains) < 0)
    {
        return false;
    }
    return CheckFluidPools(world, in);
}

// --- Owners and lists -------------------------------------------------------

// Each live body's shape list holds only its own live shapes and ends
// within the pool; together the lists hold every live shape, so no shape
// is on two lists and none is missing. A dead body owns no list.
static bool CheckShapeLists(const m2World* world, const uint8_t* in)
{
    const uint8_t* bodyAlive = M2_IN(bodies.alive);
    const uint8_t* heads = M2_IN(bodies.bodyShapeHead);
    const uint8_t* shapeAlive = M2_IN(shapes.shapeAlive);
    const uint8_t* shapeBody = M2_IN(shapes.shapeBody);
    const uint8_t* shapeNext = M2_IN(shapes.shapeNext);
    int32_t shapeCapacity = world->shapes.shapeCapacity;
    int32_t listed = 0;
    for (int32_t b = 0; b < world->bodies.bodyCapacity; ++b)
    {
        int32_t s = ReadI32(heads, b);
        if (bodyAlive[b] == 0 && s != -1)
        {
            return false;
        }
        for (int32_t steps = 0; s != -1; ++steps)
        {
            if (steps >= shapeCapacity || !InPool(s, shapeCapacity) || shapeAlive[s] == 0 ||
                ReadI32(shapeBody, s) != b)
            {
                return false;
            }
            listed += 1;
            s = ReadI32(shapeNext, s);
        }
    }
    int32_t live = 0;
    for (int32_t s = 0; s < shapeCapacity; ++s)
    {
        live += shapeAlive[s] != 0 ? 1 : 0;
    }
    return listed == live;
}

// A live chain belongs to a live body, and a live shape of a chain shares
// the chain's body.
static bool CheckChains(const m2World* world, const uint8_t* in)
{
    const uint8_t* bodyAlive = M2_IN(bodies.alive);
    const uint8_t* chainAlive = M2_IN(chains.chainAlive);
    const uint8_t* chainBody = M2_IN(chains.chainBody);
    const uint8_t* shapeAlive = M2_IN(shapes.shapeAlive);
    const uint8_t* shapeBody = M2_IN(shapes.shapeBody);
    const uint8_t* shapeChain = M2_IN(shapes.shapeChain);
    for (int32_t c = 0; c < world->shapes.shapeCapacity; ++c)
    {
        int32_t body = ReadI32(chainBody, c);
        if (chainAlive[c] != 0 &&
            (!InPool(body, world->bodies.bodyCapacity) || bodyAlive[body] == 0))
        {
            return false;
        }
    }
    for (int32_t s = 0; s < world->shapes.shapeCapacity; ++s)
    {
        int32_t chain = ReadI32(shapeChain, s);
        if (shapeAlive[s] != 0 && chain != -1 &&
            (!InPool(chain, world->shapes.shapeCapacity) || chainAlive[chain] == 0 ||
             ReadI32(chainBody, chain) != ReadI32(shapeBody, s)))
        {
            return false;
        }
    }
    return true;
}

// A live joint joins two live bodies.
// The fields a joint's kind gives its own meaning, as one joint holds
// them. The state table has already put every one of them in the float
// range.
typedef struct JointFields
{
    uint8_t type;
    float length;
    float lower;
    float upper;
    float reference;
    float correction; // jointDamping
} JointFields;

// The motor and limit kinds keep angle limits (revolute) or translation
// limits (prismatic, wheel) in lower and upper.
static bool LimitsValid(const JointFields* f)
{
    if (f->type == (uint8_t)m2_revoluteJoint)
    {
        return m2ValidAngle(f->lower) && m2ValidAngle(f->upper);
    }
    return m2ValidReach(f->lower) && m2ValidReach(f->upper);
}

// What each kind keeps in the fields it overloads, by the validators its
// def and setters use, or by what the step itself writes there. A field a
// kind never reads needs only the state table's finite check. Ranges
// only, not the order of a pair: a range out of order stays bounded.
// Accumulated state the step grows without a bound (a gear's phase in
// reference, a ratchet's angle and tooth in upper and lower) stays any
// finite value, so every snapshot a world writes restores.
static bool JointKindValid(const JointFields* f)
{
    switch ((m2JointType)f->type)
    {
    case m2_distanceJoint:
        // A rest length set or taken from the anchors; a range from 0, open
        // above when it has no maximum.
        return m2ValidSpan(f->length) && m2ValidReach(f->lower) && f->lower >= 0.0f &&
               ((m2ValidReach(f->upper) && f->upper >= 0.0f) || f->upper == M2_OPEN_LENGTH);
    case m2_revoluteJoint:
    case m2_prismaticJoint:
        return LimitsValid(f) && m2ValidAngle(f->reference);
    case m2_wheelJoint:
        return LimitsValid(f);
    case m2_weldJoint:
        return m2ValidAngle(f->reference);
    case m2_motorJoint:
        // The force budget in length, the angular offset in reference.
        return m2ValidMaxForce(f->length) && m2ValidAngle(f->reference) &&
               m2ValidFraction(f->correction);
    case m2_mouseJoint:
        return m2ValidMaxForce(f->length);
    case m2_gearJoint:
        return m2ValidRatio(f->length);
    case m2_pulleyJoint:
        // The ratio in length, the rope total (one side plus ratio times
        // the other) in reference.
        return m2ValidRatio(f->length) && f->length > 0.0f &&
               m2WithinF(f->reference, 0.0f, (1.0f + f->length) * 2.0f * (float)M2_JOINT_REACH);
    case m2_ratchetJoint:
        // The tooth pitch in length, its phase in reference.
        return m2ValidRatchetTooth(f->length) && m2ValidAngle(f->reference);
    case m2_filterJoint:
        return true;
    }
    return false;
}

static bool CheckJoints(const m2World* world, const uint8_t* in)
{
    const uint8_t* bodyAlive = M2_IN(bodies.alive);
    const uint8_t* jointAlive = M2_IN(joints.jointAlive);
    const uint8_t* bodyA = M2_IN(joints.jointBodyA);
    const uint8_t* bodyB = M2_IN(joints.jointBodyB);
    const uint8_t* types = M2_IN(joints.jointType);
    const uint8_t* lengths = M2_IN(joints.jointLength);
    const uint8_t* lowers = M2_IN(joints.jointLower);
    const uint8_t* uppers = M2_IN(joints.jointUpper);
    const uint8_t* references = M2_IN(joints.jointRefAngle);
    const uint8_t* dampings = M2_IN(joints.jointDamping);
    for (int32_t j = 0; j < world->joints.jointCapacity; ++j)
    {
        if (jointAlive[j] == 0)
        {
            continue;
        }
        int32_t a = ReadI32(bodyA, j);
        int32_t b = ReadI32(bodyB, j);
        int32_t bodies = world->bodies.bodyCapacity;
        if (!InPool(a, bodies) || !InPool(b, bodies) || bodyAlive[a] == 0 || bodyAlive[b] == 0)
        {
            return false;
        }
        JointFields fields = {types[j],           ReadF32(lengths, j),    ReadF32(lowers, j),
                              ReadF32(uppers, j), ReadF32(references, j), ReadF32(dampings, j)};
        if (!JointKindValid(&fields))
        {
            return false;
        }
    }
    return true;
}

// Every live shape's geometry is one the library would keep: valid for its
// type, and a polygon exactly as the library builds it from its vertices,
// normals included.
static bool CheckGeometry(const m2World* world, const uint8_t* in)
{
    const uint8_t* shapeAlive = M2_IN(shapes.shapeAlive);
    const uint8_t* geometries = M2_IN(shapes.shapeGeometry);
    for (int32_t s = 0; s < world->shapes.shapeCapacity; ++s)
    {
        if (shapeAlive[s] == 0)
        {
            continue;
        }
        m2ShapeGeometry geometry;
        memcpy(&geometry, geometries + (size_t)s * sizeof(geometry), sizeof(geometry));
        if (!m2GeometryIsCanonical(&geometry))
        {
            return false;
        }
    }
    return true;
}

// --- Mass data ---------------------------------------------------------------

// A body's shapes in the incoming bytes, for the mass sum.
typedef struct IncomingShapes
{
    const uint8_t* next;
    const uint8_t* geometries;
    const uint8_t* densities;
} IncomingShapes;

static int32_t IncomingNext(const void* context, int32_t shape)
{
    return ReadI32(((const IncomingShapes*)context)->next, shape);
}

static m2MassData IncomingShapeMass(const void* context, int32_t shape)
{
    const IncomingShapes* shapes = context;
    m2ShapeGeometry geometry;
    memcpy(&geometry, shapes->geometries + (size_t)shape * sizeof(geometry), sizeof(geometry));
    float density;
    memcpy(&density, shapes->densities + (size_t)shape * sizeof(density), sizeof(density));
    return m2ComputeShapeMass(&geometry, density);
}

// Mass data set by m2Body_SetMassData: a dynamic body's, with inverses a
// float can invert back, as the setter's own rules leave them.
static bool OverrideValid(float invMass, float invInertia, bool dynamic, bool fixedRotation)
{
    return dynamic && m2MassInversesValid(invMass, invInertia) &&
           (!fixedRotation || invInertia == 0.0f);
}

// Every live body's mass data is what its shapes give it, bit for bit, or
// a valid override: derived data a world would never hold cannot land.
static bool CheckMass(const m2World* world, const uint8_t* in)
{
    const uint8_t* alive = M2_IN(bodies.alive);
    const uint8_t* types = M2_IN(bodies.types);
    const uint8_t* fixed = M2_IN(bodies.fixedRotations);
    const uint8_t* overrides = M2_IN(bodies.massOverrides);
    const uint8_t* heads = M2_IN(bodies.bodyShapeHead);
    const uint8_t* invMass = M2_IN(bodies.invMass);
    const uint8_t* invInertia = M2_IN(bodies.invInertia);
    const uint8_t* centers = M2_IN(bodies.localCenters);
    IncomingShapes source = {M2_IN(shapes.shapeNext), M2_IN(shapes.shapeGeometry),
                             M2_IN(shapes.shapeDensity)};
    for (int32_t b = 0; b < world->bodies.bodyCapacity; ++b)
    {
        if (alive[b] == 0)
        {
            continue;
        }
        m2BodyMass stored = {ReadF32(invMass, b), ReadF32(invInertia, b), {0.0f, 0.0f}};
        memcpy(&stored.center, centers + (size_t)b * sizeof(m2Vec2), sizeof(m2Vec2));
        if (overrides[b] != 0)
        {
            if (!OverrideValid(stored.invMass, stored.invInertia,
                               types[b] == (uint8_t)m2_dynamicBody, fixed[b] != 0))
            {
                return false;
            }
            continue;
        }
        m2MassShapes shapes = {&source, ReadI32(heads, b), IncomingNext, IncomingShapeMass};
        m2BodyMass derived = m2BodyMassFromShapes(types[b], fixed[b] != 0, &shapes);
        // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison): bit-exact compare
        if (memcmp(&derived, &stored, sizeof(derived)) != 0)
        {
            return false;
        }
    }
    return true;
}

// --- Broadphase trees -------------------------------------------------------

// What a tree's leaves must agree with: the shapes and the bodies they
// belong to.
typedef struct Leaves
{
    const uint8_t* shapeAlive;
    const uint8_t* shapeBody;
    const uint8_t* proxyIds;
    const uint8_t* bodyTypes;
    const uint8_t* bodyDisabled;
    int32_t shapeCapacity;
} Leaves;

// A node as reached from its parent: its own rules, and its children's
// links back to it.
static bool CheckNode(const uint8_t* nodes, int32_t capacity, int32_t index, int32_t tree,
                      const Leaves* leaves)
{
    m2TreeNode node = ReadNode(nodes, index);
    if (node.height == 0)
    {
        int32_t s = node.userData;
        if (node.child1 != M2_NULL_NODE || node.child2 != M2_NULL_NODE || s < 0 ||
            s >= leaves->shapeCapacity || leaves->shapeAlive[s] == 0 ||
            ReadI32(leaves->proxyIds, s) != index)
        {
            return false;
        }
        // A live shape's body is live and in range: CheckShapeLists ran first.
        int32_t body = ReadI32(leaves->shapeBody, s);
        return leaves->bodyTypes[body] == (uint8_t)tree && leaves->bodyDisabled[body] == 0;
    }
    int32_t a = node.child1;
    int32_t b = node.child2;
    if (node.height < 1 || node.userData != -1 || a < 0 || a >= capacity || b < 0 ||
        b >= capacity || a == b)
    {
        return false;
    }
    m2TreeNode childA = ReadNode(nodes, a);
    m2TreeNode childB = ReadNode(nodes, b);
    int32_t taller = childA.height > childB.height ? childA.height : childB.height;
    return childA.parent == index && childB.parent == index && childA.height >= 0 &&
           childB.height >= 0 && node.height == 1 + taller && childA.height - childB.height <= 1 &&
           childB.height - childA.height <= 1 && m2Aabb_Contains(node.aabb, childA.aabb) &&
           m2Aabb_Contains(node.aabb, childB.aabb);
}

// Walks the tree from its root without a stack, down a child only after
// the child names the node as its parent, so the walk visits each node
// once and cannot cycle. Returns the leaves visited, or -1.
static int32_t WalkTree(const uint8_t* nodes, int32_t capacity, int32_t root, int32_t tree,
                        const Leaves* leaves, int32_t* visitedOut)
{
    int32_t visited = 0;
    int32_t leafCount = 0;
    int32_t previous = M2_NULL_NODE;
    int32_t current = root;
    if (root != M2_NULL_NODE &&
        (root < 0 || root >= capacity || ReadNode(nodes, root).parent != M2_NULL_NODE))
    {
        return -1;
    }
    for (int64_t steps = 0; current != M2_NULL_NODE; ++steps)
    {
        if (steps > 3 * (int64_t)capacity || current < 0 || current >= capacity)
        {
            return -1;
        }
        m2TreeNode node = ReadNode(nodes, current);
        int32_t next;
        if (previous == node.parent)
        {
            // Arrived from above: the node's first visit.
            visited += 1;
            if (visited > capacity || !CheckNode(nodes, capacity, current, tree, leaves))
            {
                return -1;
            }
            leafCount += node.height == 0 ? 1 : 0;
            next = node.height == 0 ? node.parent : node.child1;
        }
        else
        {
            next = previous == node.child1 ? node.child2 : node.parent;
        }
        previous = current;
        current = next;
    }
    *visitedOut = visited;
    return leafCount;
}

// The free list runs through parent links and holds free nodes only.
// Returns its length, or -1.
static int32_t WalkFreeList(const uint8_t* nodes, int32_t capacity, int32_t head)
{
    int32_t length = 0;
    for (int32_t n = head; n != M2_NULL_NODE;)
    {
        if (length >= capacity || n < 0 || n >= capacity)
        {
            return -1;
        }
        m2TreeNode node = ReadNode(nodes, n);
        if (node.height != -1)
        {
            return -1;
        }
        length += 1;
        n = node.parent;
    }
    return length;
}

// Every node is in the tree or on the free list: the tree's nodes are
// exactly the ones the walk visits, and the free list holds the rest. Each
// live shape of an enabled body has a leaf in its body type's tree, and no
// other shape has a proxy.
static bool CheckTrees(const m2World* world, const uint8_t* in)
{
    const Leaves leaves = {M2_IN(shapes.shapeAlive),   M2_IN(shapes.shapeBody),
                           M2_IN(broadphase.proxyIds), M2_IN(bodies.types),
                           M2_IN(bodies.disabled),     world->shapes.shapeCapacity};
    const uint8_t* headers = M2_IN(broadphase.trees);
    const uint8_t* nodeArrays[M2_TREE_COUNT] = {M2_IN(broadphase.treeNodes[0]),
                                                M2_IN(broadphase.treeNodes[1]),
                                                M2_IN(broadphase.treeNodes[2])};
    int32_t capacity = world->broadphase.treeNodeCapacity;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2DynamicTree tree;
        memcpy(&tree, headers + (size_t)t * sizeof(tree), sizeof(tree));
        int32_t visited = 0;
        int32_t leafCount = WalkTree(nodeArrays[t], capacity, tree.root, t, &leaves, &visited);
        int32_t free = WalkFreeList(nodeArrays[t], capacity, tree.freeList);
        if (leafCount < 0 || free < 0 || visited + free != capacity || tree.nodeCount != visited ||
            (visited > 0 && visited != 2 * leafCount - 1))
        {
            return false;
        }
    }
    for (int32_t s = 0; s < leaves.shapeCapacity; ++s)
    {
        int32_t proxy = ReadI32(leaves.proxyIds, s);
        bool wanted =
            leaves.shapeAlive[s] != 0 && leaves.bodyDisabled[ReadI32(leaves.shapeBody, s)] == 0;
        if (!wanted)
        {
            if (proxy != M2_NULL_NODE)
            {
                return false;
            }
            continue;
        }
        int32_t tree = leaves.bodyTypes[ReadI32(leaves.shapeBody, s)];
        if (!InPool(tree, M2_TREE_COUNT) || !InPool(proxy, capacity))
        {
            return false;
        }
        m2TreeNode leaf = ReadNode(nodeArrays[tree], proxy);
        if (leaf.height != 0 || leaf.userData != s)
        {
            return false;
        }
    }
    return true;
}

// --- Moved shapes and pairs -------------------------------------------------

// The moved list holds distinct shapes, and the flags mark exactly them.
static bool CheckMoved(m2World* world, const uint8_t* in)
{
    const uint8_t* moved = M2_IN(broadphase.moved);
    const uint8_t* inMoved = M2_IN(broadphase.inMoved);
    int32_t count = InlineI32(world, in, &world->broadphase.movedCount);
    if (count < 0 || count > world->shapes.shapeCapacity)
    {
        return false;
    }
    uint8_t* marks = world->restoreMarks;
    memset(marks, 0, (size_t)world->shapes.shapeCapacity);
    for (int32_t k = 0; k < count; ++k)
    {
        int32_t s = ReadI32(moved, k);
        if (!InPool(s, world->shapes.shapeCapacity) || marks[s] != 0 || inMoved[s] == 0)
        {
            return false;
        }
        marks[s] = 1;
    }
    int32_t flagged = 0;
    for (int32_t s = 0; s < world->shapes.shapeCapacity; ++s)
    {
        flagged += inMoved[s] != 0 ? 1 : 0;
    }
    return flagged == count;
}

// Pairs are in strictly increasing key order, and each joins two live
// shapes of different bodies, both in the broadphase.
static bool CheckPairs(const m2World* world, const uint8_t* in)
{
    const uint8_t* keys = M2_IN(contacts.pairKeys);
    const uint8_t* touching = M2_IN(contacts.pairTouching);
    const uint8_t* shapeAlive = M2_IN(shapes.shapeAlive);
    const uint8_t* shapeBody = M2_IN(shapes.shapeBody);
    const uint8_t* proxyIds = M2_IN(broadphase.proxyIds);
    int32_t count = InlineI32(world, in, &world->contacts.pairCount);
    if (count < 0 || count > world->contacts.pairCapacity)
    {
        return false;
    }
    uint64_t previous = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        uint64_t key = ReadU64(keys, i);
        int32_t a = (int32_t)(key >> 32);
        int32_t b = (int32_t)(key & 0xFFFFFFFFu);
        if (a < 0 || b >= world->shapes.shapeCapacity || (i > 0 && key <= previous) || a >= b ||
            touching[i] > 1 || shapeAlive[a] == 0 || shapeAlive[b] == 0 ||
            ReadI32(proxyIds, a) == M2_NULL_NODE || ReadI32(proxyIds, b) == M2_NULL_NODE ||
            ReadI32(shapeBody, a) == ReadI32(shapeBody, b))
        {
            return false;
        }
        previous = key;
    }
    return true;
}

// The boxes the step will not refit before it queries them: those of
// static bodies, which never move in a step, and of sleeping dynamic
// bodies, which a world refits as they fall asleep. Each must hold its
// shape's tight box at the body's pose, by the step's own computation and
// test. Awake bodies' boxes lag their poses by the last step in every
// snapshot, and the next step refits them first.
static bool CheckRestingBoxes(const m2World* world, const uint8_t* in)
{
    const uint8_t* alive = M2_IN(shapes.shapeAlive);
    const uint8_t* owners = M2_IN(shapes.shapeBody);
    const uint8_t* geometries = M2_IN(shapes.shapeGeometry);
    const uint8_t* proxies = M2_IN(broadphase.proxyIds);
    const uint8_t* types = M2_IN(bodies.types);
    const uint8_t* asleep = M2_IN(bodies.asleep);
    const uint8_t* transforms = M2_IN(bodies.transforms);
    const uint8_t* nodes[3] = {M2_IN(broadphase.treeNodes[0]), M2_IN(broadphase.treeNodes[1]),
                               M2_IN(broadphase.treeNodes[2])};
    for (int32_t s = 0; s < world->shapes.shapeCapacity; ++s)
    {
        int32_t proxy = ReadI32(proxies, s);
        if (alive[s] == 0 || proxy < 0)
        {
            continue;
        }
        int32_t body = ReadI32(owners, s);
        uint8_t type = types[body];
        bool resting = type == (uint8_t)m2_staticBody ||
                       (type == (uint8_t)m2_dynamicBody && asleep[body] != 0);
        if (!resting)
        {
            continue;
        }
        m2ShapeGeometry geometry;
        memcpy(&geometry, geometries + (size_t)s * sizeof(geometry), sizeof(geometry));
        m2Transform xf;
        memcpy(&xf, transforms + (size_t)body * sizeof(xf), sizeof(xf));
        if (!m2Aabb_Contains(ReadNode(nodes[type], proxy).aabb, m2ComputeShapeAabb(&geometry, xf)))
        {
            return false;
        }
    }
    return true;
}

#undef M2_IN

bool m2SnapshotCheck(m2World* world, const uint8_t* in)
{
    return CheckPools(world, in) && CheckShapeLists(world, in) && CheckChains(world, in) &&
           CheckJoints(world, in) && CheckGeometry(world, in) && CheckMass(world, in) &&
           CheckTrees(world, in) && CheckMoved(world, in) && CheckPairs(world, in) &&
           CheckRestingBoxes(world, in);
}
