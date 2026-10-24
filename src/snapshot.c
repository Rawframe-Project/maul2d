// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshots and hashing: the block walk that sizes, writes and restores
// the world state, and the world hashes built on the same walk.

#include "snapshot.h"

#include "joint.h"
#include "joint_edges.h"
#include "journal.h"
#include "snapshot_check.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_SNAPSHOT_MAGIC 0x4D32534Eu // 'M2SN'

#define M2_SNAPSHOT_VERSION 2u

// --- Snapshot -------------------------------------------------------------------

// The header says which build and which world shape wrote the bytes;
// everything else, pool cursors included, is the state table's.
typedef struct m2SnapshotHeader
{
    uint32_t magic;
    uint32_t formatVersion;
    uint64_t configHash;
    int32_t capacities[6];
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 40, "snapshot header must be padding-free");

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
    // deliberately absent: the format is portable across them).
    // The format's own number, never the library version: a release that
    // keeps the format reads the snapshots of the one before.
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

// Single source of truth: the size IS the walk (measure mode). The
// duplicated byte formula died here after its third drift (assert-caught
// every time; root cause now removed).
static int32_t BlockBytes(const m2World* world)
{
    return m2StateWalk((m2World*)world, NULL, NULL, 2);
}

int32_t m2World_SnapshotSize(const m2World* world)
{
    if (world == NULL)
    {
        return 0;
    }
    return (int32_t)sizeof(m2SnapshotHeader) + BlockBytes(world);
}

// The block walk is shared by Snapshot and Restore so the layouts can
// never drift apart (direction 0 = write, 1 = read).

m2Result m2World_Snapshot(const m2World* world, void* buffer, int32_t capacity, int32_t* bytesOut)
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
    int32_t size = m2World_SnapshotSize(world);
    if (capacity < size)
    {
        *bytesOut = size; // the size a buffer must have
        return m2_errorCapacity;
    }

    m2SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M2_SNAPSHOT_MAGIC;
    header.formatVersion = M2_SNAPSHOT_VERSION;
    header.configHash = ConfigHash();
    WorldCapacities(world, header.capacities);

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    int32_t cursor =
        (int32_t)sizeof(header) + m2StateWalk((m2World*)world, out + sizeof(header), NULL, 0);
    M2_ASSERT(cursor == size);
    *bytesOut = cursor;
    return m2_success;
}

m2Result m2World_Restore(m2World* world, const void* buffer, int32_t size)
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
    if (size < (int32_t)sizeof(m2SnapshotHeader))
    {
        return m2_errorInvalid;
    }
    m2SnapshotHeader header;
    memcpy(&header, buffer, sizeof(header));
    int32_t capacities[6];
    WorldCapacities(world, capacities);
    if (header.magic != M2_SNAPSHOT_MAGIC || header.formatVersion != M2_SNAPSHOT_VERSION ||
        header.configHash != ConfigHash() ||
        memcmp(header.capacities, capacities, sizeof(capacities)) != 0)
    {
        return m2_errorInvalid; // another build or another world shape
    }
    // Hostile bytes are checked before any of them land: every row the
    // table describes, cursors included, then how the rows fit together.
    const uint8_t* in = buffer;
    if (size != (int32_t)sizeof(header) + BlockBytes(world) ||
        !m2StateValidate(world, in + sizeof(header)) ||
        !m2SnapshotCheck(world, in + sizeof(header)))
    {
        return m2_errorInvalid;
    }

    int32_t cursor = (int32_t)sizeof(header) + m2StateWalk(world, NULL, in + sizeof(header), 1);
    M2_ASSERT(cursor == size);
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

uint64_t m2World_Hash(const m2World* world)
{
    return world != NULL ? m2StateHash(world, m2_hashAll) : 0;
}

// Subsystem hashes: independent seeds on purpose (the total is not
// a function of the parts).
m2WorldHashParts m2World_GetHashParts(const m2World* world)
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
