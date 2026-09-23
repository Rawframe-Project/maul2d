// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Command journal. Wire format, all little-endian raw structs (floats
// as IEEE-754 bit patterns): a header { magic, version, world echo,
// snapshotSize }, the embedded snapshot, then op records, each one op
// byte followed by that op's payload (journal.h). Replay restores the
// snapshot and re-applies every op through the same public entry
// points; every recreated id must match the recording.

#include "journal.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_JOURNAL_MAGIC   0x4D324A4Eu // 'M2JN'
#define M2_JOURNAL_VERSION 1u

typedef struct m2JournalHeader
{
    uint32_t magic;
    uint32_t version;
    m2Vec2 gravity;
    int32_t bodyCapacity;
    int32_t shapeCapacity;
    int32_t jointCapacity;
    int32_t snapshotSize;
} m2JournalHeader;

_Static_assert(sizeof(m2JournalHeader) == 32, "journal header must be padding-free");

// Reserves one record of 1 + bytes and writes its op byte. Returns where
// the payload goes, or NULL when nothing is recording or the buffer is
// full (the overflow is reported by m2World_StopJournal, never silent).
static uint8_t* ReserveRecord(m2World* world, uint8_t op, int32_t bytes)
{
    if (world->journalActive == 0 || world->journalOverflow != 0)
    {
        return NULL;
    }
    if (world->journalCursor + 1 + bytes > world->journalCapacity)
    {
        world->journalOverflow = 1;
        return NULL;
    }
    uint8_t* record = world->journal + world->journalCursor;
    world->journalCursor += 1 + bytes;
    record[0] = op;
    return record + 1;
}

void m2JournalRecord(m2World* world, uint8_t op, const void* payload, int32_t bytes)
{
    uint8_t* out = ReserveRecord(world, op, bytes);
    if (out != NULL)
    {
        memcpy(out, payload, (size_t)bytes);
    }
}

void m2JournalRecordRestore(m2World* world, const void* snapshot, int32_t size)
{
    uint8_t* out = ReserveRecord(world, m2_opRestore, (int32_t)sizeof(int32_t) + size);
    if (out != NULL)
    {
        memcpy(out, &size, sizeof(int32_t));
        memcpy(out + sizeof(int32_t), snapshot, (size_t)size);
    }
}

void m2JournalRecordShatter(m2World* world, m2BodyId bodyId, const m2Polygon* pieces,
                            int32_t pieceCount, int32_t expectedFirst)
{
    int32_t pieceBytes = pieceCount * (int32_t)sizeof(m2Polygon);
    uint8_t* out =
        ReserveRecord(world, m2_opShatterBody, (int32_t)sizeof(m2OpShatterHeader) + pieceBytes);
    if (out == NULL)
    {
        return;
    }
    m2OpShatterHeader header;
    memset(&header, 0, sizeof(header));
    header.body = bodyId;
    header.pieceCount = pieceCount;
    header.expectedFirst = expectedFirst;
    memcpy(out, &header, sizeof(header));
    memcpy(out + sizeof(header), pieces, (size_t)pieceBytes);
}

void m2JournalRecordChain(m2World* world, m2BodyId bodyId, const m2ChainDef* def,
                          int32_t createdCount)
{
    int32_t pointBytes = def->count * (int32_t)sizeof(m2Vec2);
    uint8_t* out =
        ReserveRecord(world, m2_opCreateChain, (int32_t)sizeof(m2OpChainHeader) + pointBytes);
    if (out == NULL)
    {
        return;
    }
    m2OpChainHeader header;
    memset(&header, 0, sizeof(header));
    header.body = bodyId;
    header.count = def->count;
    header.createdCount = createdCount;
    header.friction = def->friction;
    header.restitution = def->restitution;
    header.categoryBits = def->categoryBits;
    header.maskBits = def->maskBits;
    header.groupIndex = def->groupIndex;
    header.userData = def->userData;
    header.isLoop = def->isLoop ? 1 : 0;
    memcpy(out, &header, sizeof(header));
    memcpy(out + sizeof(header), def->points, (size_t)pointBytes);
}

int32_t m2World_JournalBaseSize(m2WorldId worldId)
{
    int32_t snapshot = m2World_SnapshotSize(worldId);
    if (snapshot <= 0)
    {
        return 0;
    }
    return (int32_t)sizeof(m2JournalHeader) + snapshot;
}

bool m2World_StartJournal(m2WorldId worldId, void* buffer, int32_t capacity)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || buffer == NULL || world->journalActive != 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    int32_t snapshotSize = m2World_SnapshotSize(worldId);
    if (capacity < (int32_t)sizeof(m2JournalHeader) + snapshotSize)
    {
        m2Refuse(world, m2_errorCapacity);
        return false;
    }

    m2JournalHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M2_JOURNAL_MAGIC;
    header.version = M2_JOURNAL_VERSION;
    header.gravity = world->gravity;
    header.bodyCapacity = world->bodyCapacity;
    header.shapeCapacity = world->shapeCapacity;
    header.jointCapacity = world->jointCapacity;
    header.snapshotSize = snapshotSize;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    if (m2World_Snapshot(worldId, out + sizeof(header), snapshotSize) != snapshotSize)
    {
        m2Refuse(world, m2_errorCapacity);
        return false;
    }

    world->journal = out;
    world->journalCapacity = capacity;
    world->journalCursor = (int32_t)sizeof(header) + snapshotSize;
    world->journalActive = 1;
    world->journalOverflow = 0;
    return true;
}

int32_t m2World_StopJournal(m2WorldId worldId)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || world->journalActive == 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t size = world->journalCursor;
    if (world->journalOverflow != 0)
    {
        size = 0;
        m2Refuse(world, m2_errorCapacity);
    }
    world->journalActive = 0;
    world->journal = NULL;
    world->journalCapacity = 0;
    world->journalCursor = 0;
    world->journalOverflow = 0;
    return size;
}

// Applies a tape to a world: header, snapshot, then one command per op.
// Stops at the first op that is malformed, unknown, or recreates an
// object under a different id than the recording saw; the caller rolls
// the world back.
static m2Result ReplayOps(m2WorldId worldId, m2World* world, const uint8_t* data, int32_t size)
{
    m2JournalHeader header;
    memcpy(&header, data, sizeof(header));
    if (header.magic != M2_JOURNAL_MAGIC || header.version != M2_JOURNAL_VERSION ||
        header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity ||
        header.jointCapacity != world->jointCapacity)
    {
        return m2_errorConfig; // a tape from another build or another world shape
    }
    if (header.snapshotSize < 0 || size - (int32_t)sizeof(header) < header.snapshotSize ||
        !m2World_Restore(worldId, data + sizeof(header), header.snapshotSize))
    {
        return m2_errorInvalid;
    }

    // Recorded ids carry the recording world's registry index; every
    // apply rebinds them to the target world, so replay can never reach
    // across the registry into the original.
    m2ReplayCursor cursor;
    cursor.worldId = worldId;
    cursor.world = world;
    cursor.here = worldId.index1;
    cursor.data = data;
    cursor.size = size;
    cursor.offset = (int32_t)sizeof(header) + header.snapshotSize;

    // Payloads sit unaligned in the stream; each is copied out first, so
    // an apply function always reads an aligned struct.
    m2OpPayload payload;
    while (cursor.offset < size)
    {
        const m2JournalCommand* command = m2JournalCommandFor(data[cursor.offset]);
        cursor.offset += 1;
        if (command == NULL || size - cursor.offset < command->payloadSize)
        {
            return m2_errorInvalid; // unknown op or truncated payload
        }
        memcpy(&payload, data + cursor.offset, (size_t)command->payloadSize);
        cursor.offset += command->payloadSize;
        if (!command->apply(&cursor, &payload))
        {
            return m2_errorInvalid;
        }
    }
    return m2_success;
}

bool m2World_ReplayJournal(m2WorldId worldId, const void* data, int32_t size)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || data == NULL || size < (int32_t)sizeof(m2JournalHeader) ||
        world->journalActive != 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    // Atomic: the world takes the whole tape or none of it. A snapshot
    // taken first backs out a replay that fails part way, so a corrupt
    // or truncated tape never leaves a half-built world behind.
    int32_t backupSize = m2World_SnapshotSize(worldId);
    void* backup = m2AllocZeroed((size_t)backupSize);
    if (backup == NULL)
    {
        m2Refuse(world, m2_errorCapacity);
        return false; // no memory for the guarantee means no replay
    }
    m2Result result = m2World_Snapshot(worldId, backup, backupSize) == backupSize
                          ? ReplayOps(worldId, world, data, size)
                          : m2_errorCapacity;
    if (result != m2_success)
    {
        bool restored = m2World_Restore(worldId, backup, backupSize);
        M2_ASSERT(restored); // our own snapshot must restore: invariant
        (void)restored;
        m2Refuse(world, result);
    }
    m2Free(backup);
    return result == m2_success;
}
