// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshots and hashing: the block walk that sizes, writes and restores
// the world state, and the world hashes built on the same walk.

#include "snapshot.h"

#include "fpenv.h"
#include "joint.h"
#include "joint_edges.h"
#include "journal.h"
#include "snapshot_check.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define M2_SNAPSHOT_MAGIC M2_STATE_MAGIC('M', '2', 'S', 'N')

#define M2_SNAPSHOT_VERSION 7u

// --- Snapshot -------------------------------------------------------------------

// The header says which release, which build and which world shape wrote
// the bytes; everything else, pool cursors included, is the state table's.
// The world shape is the capacities and the particle radius, which sets
// the particle state a world can make (spring rest lengths, triad spawn
// corners) and is the one def setting the restore check measures by. It
// is held widened to a double, which keeps the header free of padding.
typedef struct m2SnapshotHeader
{
    m2StatePrefix prefix;
    uint64_t configHash;
    int32_t capacities[6];
    double particleRadius;
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 56, "snapshot header must be padding-free");

static void WorldCapacities(const m2World* world, int32_t capacities[6])
{
    capacities[0] = world->bodies.bodyCapacity;
    capacities[1] = world->shapes.shapeCapacity;
    capacities[2] = world->joints.jointCapacity;
    capacities[3] = world->particles.particleCapacity;
    capacities[4] = world->volumes.fvCapacity;
    capacities[5] = world->contacts.pairCapacity;
}

static uint64_t ConfigHash(void)
{
    // Everything that changes what the serialized bytes MEAN, and
    // nothing that does not (SIMD backend and worker count are
    // deliberately absent: the format is portable across them). The
    // release that wrote the bytes is the prefix's, not part of this.
    uint64_t h = 0xCBF29CE484222325ull;
    int32_t version = (int32_t)M2_SNAPSHOT_VERSION;
    int32_t realSize = (int32_t)sizeof(float);
    int32_t posSize = (int32_t)sizeof(double);
    const char* fpPolicy = "contract-off;no-fast-math;explicit-fma";
    h = m2Hash64(h, &version, 4);
    h = m2Hash64(h, &realSize, 4);
    h = m2Hash64(h, &posSize, 4);
    h = m2Hash64(h, fpPolicy, (int32_t)strlen(fpPolicy));
    return h;
}

// The snapshot's bytes, its header and the state table's block, from the
// world's capacities; INT64_MAX when they do not fit a size_t. The block
// is the walk's own size: m2StateSnapshotBytes sums the per-array sizes
// m2StateWalk copies. A world m2CreateWorld accepted has a size below
// INT64_MAX.
static int64_t SnapshotBytes(const m2World* world)
{
    size_t block = 0;
    if (!m2StateSnapshotBytes(world, &block) ||
        block > (size_t)INT64_MAX - sizeof(m2SnapshotHeader))
    {
        return INT64_MAX;
    }
    return (int64_t)(sizeof(m2SnapshotHeader) + block);
}

bool m2SnapshotFits(const m2World* world)
{
    return SnapshotBytes(world) < INT64_MAX;
}

static M2_NOINLINE int64_t WorldSnapshotSizeWorker(const m2World* world)
{
    return world != NULL ? SnapshotBytes(world) : 0;
}

int64_t m2World_SnapshotSize(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    int64_t result = WorldSnapshotSizeWorker(world);
    m2FloatEnvLeave(env);
    return result;
}

// The block walk is shared by Snapshot and Restore so the layouts can
// never drift apart (direction 0 = write, 1 = read).

static M2_NOINLINE m2Result WorldSnapshotWorker(const m2World* world, void* buffer,
                                                int64_t capacity, int64_t* bytesOut)
{
    if (bytesOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *bytesOut = 0;
    if (world == NULL || buffer == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int64_t size = SnapshotBytes(world);
    if (capacity < size)
    {
        *bytesOut = size; // the size a buffer must have
        return m2_errorCapacity;
    }

    m2SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.prefix =
        m2MakeStatePrefix(M2_SNAPSHOT_MAGIC, M2_SNAPSHOT_VERSION, (int32_t)sizeof(header));
    header.configHash = ConfigHash();
    WorldCapacities(world, header.capacities);
    header.particleRadius = (double)world->particles.particleRadius;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    size_t cursor = sizeof(header) + m2StateWalk((m2World*)world, out + sizeof(header), NULL, 0);
    M2_ASSERT(cursor == (size_t)size);
    (void)cursor;
    *bytesOut = size; // at most capacity
    return m2_success;
}

m2Result m2World_Snapshot(const m2World* world, void* buffer, int64_t capacity, int64_t* bytesOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = WorldSnapshotWorker(world, buffer, capacity, bytesOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Result WorldRestoreWorker(m2World* world, const void* buffer, int64_t size)
{
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (buffer == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    // From here on the bytes are untrusted input: refusing them is no
    // misuse of the API, so nothing is counted.
    // Only the release that wrote a snapshot reads it.
    m2Result prefix = m2CheckStatePrefix(buffer, size, M2_SNAPSHOT_MAGIC, M2_SNAPSHOT_VERSION,
                                         (int32_t)sizeof(m2SnapshotHeader));
    if (prefix != m2_success)
    {
        return prefix;
    }
    m2SnapshotHeader header;
    memcpy(&header, buffer, sizeof(header));
    if (header.configHash != ConfigHash())
    {
        return m2_errorVersion; // a build whose bytes mean something else
    }
    int32_t capacities[6];
    WorldCapacities(world, capacities);
    if (memcmp(header.capacities, capacities, sizeof(capacities)) != 0 ||
        !(header.particleRadius == (double)world->particles.particleRadius))
    {
        return m2_errorFormat; // another world shape
    }
    // Hostile bytes are checked before any of them land: every row the
    // table describes, cursors included, then how the rows fit together.
    const uint8_t* in = buffer;
    if (size != SnapshotBytes(world) || !m2StateValidate(world, in + sizeof(header)) ||
        !m2SnapshotCheck(world, in + sizeof(header)))
    {
        return m2_errorFormat;
    }

    size_t cursor = sizeof(header) + m2StateWalk(world, NULL, in + sizeof(header), 1);
    M2_ASSERT(cursor == (size_t)size);
    (void)cursor;
    m2RebuildJointEdges(world);

    // Restores are first-class journal citizens: the tape carries the
    // snapshot itself, so rollback-heavy sessions replay bit-exactly.
    // The price is tape size, and that is the caller's tradeoff.
    m2JournalRecordRestore(world, buffer, size);

    // Events are an observer stream from an abandoned timeline: cleared
    // on restore, re-emitted by re-simulation.
    world->events.beginEventCount = 0;
    world->events.endEventCount = 0;
    world->events.pendingEndCount = 0;
    world->events.sensorBeginCount = 0;
    world->events.sensorEndCount = 0;
    world->events.pendingSensorEndCount = 0;
    world->events.jointBreakEventCount = 0;
    return m2_success;
}

m2Result m2World_Restore(m2World* world, const void* buffer, int64_t size)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = WorldRestoreWorker(world, buffer, size);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE uint64_t WorldHashWorker(const m2World* world)
{
    return world != NULL ? m2StateHash(world, m2_hashAll) : 0;
}

uint64_t m2World_Hash(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    uint64_t result = WorldHashWorker(world);
    m2FloatEnvLeave(env);
    return result;
}

// Subsystem hashes: independent seeds on purpose (the total is not
// a function of the parts).
static M2_NOINLINE m2WorldHashParts WorldGetHashPartsWorker(const m2World* world)
{
    m2WorldHashParts parts;
    memset(&parts, 0, sizeof(parts));
    if (world == NULL)
    {
        return parts;
    }
    parts.world = m2StateHash(world, m2_hashWorld);
    parts.bodies = m2StateHash(world, m2_hashBodies);
    parts.contacts = m2StateHash(world, m2_hashContacts);
    parts.joints = m2StateHash(world, m2_hashJoints);
    parts.particles = m2StateHash(world, m2_hashParticles);
    parts.volumes = m2StateHash(world, m2_hashVolumes);
    return parts;
}

m2WorldHashParts m2World_GetHashParts(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    m2WorldHashParts result = WorldGetHashPartsWorker(world);
    m2FloatEnvLeave(env);
    return result;
}
