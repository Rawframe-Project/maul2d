// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Command journal. Wire format, all little-endian raw structs (floats
// as IEEE-754 bit patterns): a header { the state prefix, world echo,
// snapshotSize }, the embedded snapshot, then op records, each one op
// byte followed by that op's payload (journal.h). Replay restores the
// snapshot and re-applies every op through the same public entry
// points; every recreated id must match the recording.

#include "journal.h"

#include "fpenv.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <stdint.h>
#include <string.h>

#define M2_JOURNAL_MAGIC   M2_STATE_MAGIC('M', '2', 'J', 'N')
#define M2_JOURNAL_VERSION 7u

typedef struct m2JournalHeader
{
    m2StatePrefix prefix;
    m2Vec2 gravity;
    int32_t bodyCapacity;
    int32_t shapeCapacity;
    int32_t jointCapacity;
    int32_t snapshotSize;
} m2JournalHeader;

_Static_assert(sizeof(m2JournalHeader) == 40, "journal header must be padding-free");
// A tape of any world holds its header, the snapshot it starts from and
// the record of one restore of a snapshot of that size.
_Static_assert(2 * (int64_t)M2_MAX_SNAPSHOT_BYTES + (int64_t)sizeof(m2JournalHeader) + 1 +
                       (int64_t)sizeof(int32_t) <=
                   INT32_MAX,
               "a journal of the largest snapshot and one restore fits an int32_t");

bool m2JournalRecordFits(int32_t cursor, int32_t capacity, int64_t bytes)
{
    return bytes >= 0 && bytes <= (int64_t)capacity - (int64_t)cursor - 1;
}

// Reserves one record of 1 + bytes and writes its op byte. Returns where
// the payload goes, or NULL when nothing is recording or the buffer is
// full (the overflow is reported by m2World_StopJournal, never silent).
static uint8_t* ReserveRecord(m2World* world, uint8_t op, int64_t bytes)
{
    if (world->recorder.journalActive == 0 || world->recorder.journalOverflow != 0)
    {
        return NULL;
    }
    if (!m2JournalRecordFits(world->recorder.journalCursor, world->recorder.journalCapacity, bytes))
    {
        world->recorder.journalOverflow = 1;
        return NULL;
    }
    uint8_t* record = world->recorder.journal + world->recorder.journalCursor;
    world->recorder.journalCursor += 1 + (int32_t)bytes; // within the capacity
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
    uint8_t* out = ReserveRecord(world, m2_opRestore, (int64_t)sizeof(int32_t) + size);
    if (out != NULL)
    {
        memcpy(out, &size, sizeof(int32_t));
        memcpy(out + sizeof(int32_t), snapshot, (size_t)size);
    }
}

void m2JournalRecordShatter(m2World* world, m2BodyId bodyId, const m2Polygon* pieces,
                            int32_t pieceCount, int32_t expectedFirst)
{
    int64_t pieceBytes = (int64_t)pieceCount * (int64_t)sizeof(m2Polygon);
    uint8_t* out =
        ReserveRecord(world, m2_opShatterBody, (int64_t)sizeof(m2OpShatterHeader) + pieceBytes);
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
    int64_t pointBytes = (int64_t)def->count * (int64_t)sizeof(m2Vec2);
    uint8_t* out =
        ReserveRecord(world, m2_opCreateChain, (int64_t)sizeof(m2OpChainHeader) + pointBytes);
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

static M2_NOINLINE int32_t WorldGetJournalBaseSizeWorker(const m2World* world)
{
    int32_t snapshot = m2World_SnapshotSize(world);
    if (snapshot <= 0)
    {
        return 0;
    }
    return (int32_t)sizeof(m2JournalHeader) + snapshot;
}

int32_t m2World_GetJournalBaseSize(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    int32_t result = WorldGetJournalBaseSizeWorker(world);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Result WorldStartJournalWorker(m2World* world, void* buffer, int32_t capacity)
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
    if (world->recorder.journalActive != 0)
    {
        return m2Refuse(world, m2_errorState);
    }
    int32_t snapshotSize = m2World_SnapshotSize(world);
    if ((int64_t)capacity < (int64_t)sizeof(m2JournalHeader) + snapshotSize)
    {
        return m2Refuse(world, m2_errorCapacity);
    }

    m2JournalHeader header;
    memset(&header, 0, sizeof(header));
    header.prefix =
        m2MakeStatePrefix(M2_JOURNAL_MAGIC, M2_JOURNAL_VERSION, (int32_t)sizeof(header));
    header.gravity = world->gravity;
    header.bodyCapacity = world->bodies.bodyCapacity;
    header.shapeCapacity = world->shapes.shapeCapacity;
    header.jointCapacity = world->joints.jointCapacity;
    header.snapshotSize = snapshotSize;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    int32_t written = 0;
    m2Result snapshot = m2World_Snapshot(world, out + sizeof(header), snapshotSize, &written);
    M2_ASSERT(snapshot == m2_success && written == snapshotSize); // sized just above
    (void)snapshot;

    world->recorder.journal = out;
    world->recorder.journalCapacity = capacity;
    world->recorder.journalCursor = (int32_t)sizeof(header) + snapshotSize;
    world->recorder.journalActive = 1;
    world->recorder.journalOverflow = 0;
    return m2_success;
}

m2Result m2World_StartJournal(m2World* world, void* buffer, int32_t capacity)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = WorldStartJournalWorker(world, buffer, capacity);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Result WorldStopJournalWorker(m2World* world, int32_t* bytesOut)
{
    if (bytesOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *bytesOut = 0;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (world->recorder.journalActive == 0)
    {
        return m2Refuse(world, m2_errorState);
    }
    int32_t size = world->recorder.journalCursor;
    bool overflowed = world->recorder.journalOverflow != 0;
    world->recorder.journalActive = 0;
    world->recorder.journal = NULL;
    world->recorder.journalCapacity = 0;
    world->recorder.journalCursor = 0;
    world->recorder.journalOverflow = 0;
    if (overflowed)
    {
        return m2Refuse(world, m2_errorCapacity);
    }
    *bytesOut = size;
    return m2_success;
}

m2Result m2World_StopJournal(m2World* world, int32_t* bytesOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = WorldStopJournalWorker(world, bytesOut);
    m2FloatEnvLeave(env);
    return result;
}

// Applies a tape to a world: header, snapshot, then one command per op.
// Stops at the first op that is malformed, unknown, or recreates an
// object under a different id than the recording saw; the caller rolls
// the world back.
static m2Result ReplayOps(m2World* world, const uint8_t* data, int32_t size)
{
    m2JournalHeader header;
    memcpy(&header, data, sizeof(header));
    if (header.bodyCapacity != world->bodies.bodyCapacity ||
        header.shapeCapacity != world->shapes.shapeCapacity ||
        header.jointCapacity != world->joints.jointCapacity)
    {
        return m2_errorInvalid; // a tape from another world shape
    }
    if (header.snapshotSize < 0 || size - (int32_t)sizeof(header) < header.snapshotSize)
    {
        return m2_errorInvalid;
    }
    // The embedded snapshot's refusal is the tape's: one from another
    // release or build stays m2_errorVersion.
    m2Result restored = m2World_Restore(world, data + sizeof(header), header.snapshotSize);
    if (restored != m2_success)
    {
        return restored;
    }

    // Ids carry no world: the recorded ids name slots of whichever world
    // replays the tape.
    m2ReplayCursor cursor;
    cursor.world = world;
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
        for (int32_t k = 0; k < command->boolCount; ++k)
        {
            if (data[cursor.offset + command->bools[k]] > 1)
            {
                return m2_errorInvalid; // a bool that is neither false nor true
            }
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

static M2_NOINLINE m2Result WorldReplayJournalWorker(m2World* world, const void* data, int32_t size)
{
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (data == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    if (world->recorder.journalActive != 0)
    {
        return m2Refuse(world, m2_errorState);
    }
    // From here on the tape is untrusted input: refusing it is no misuse
    // of the API, so nothing is counted. Only the release that wrote a
    // tape replays it.
    m2Result prefix = m2CheckStatePrefix(data, size, M2_JOURNAL_MAGIC, M2_JOURNAL_VERSION,
                                         (int32_t)sizeof(m2JournalHeader));
    if (prefix != m2_success)
    {
        return prefix;
    }
    // Atomic: the world takes the whole tape or none of it. A snapshot
    // taken first backs out a replay that fails part way, so a corrupt
    // or truncated tape never leaves a half-built world behind.
    int32_t backupSize = m2World_SnapshotSize(world);
    void* backup = m2WorldAlloc(world, (size_t)backupSize);
    if (backup == NULL)
    {
        return m2Refuse(world, m2_errorCapacity); // no memory for the guarantee means no replay
    }
    int32_t written = 0;
    m2Result snapshot = m2World_Snapshot(world, backup, backupSize, &written);
    M2_ASSERT(snapshot == m2_success && written == backupSize); // sized just above
    (void)snapshot;
    // The ops re-enter the public calls with the tape's values; a value
    // they refuse is the tape's fault, not the caller's, and counts nothing
    // (m2Refuse). Host code the replay runs is the caller's and still
    // counts, whatever it gets wrong.
    world->replaying = 1;
    m2Result result = ReplayOps(world, data, size);
    if (result != m2_success)
    {
        m2Result restored = m2World_Restore(world, backup, backupSize);
        M2_ASSERT(restored == m2_success); // our own snapshot must restore: invariant
        (void)restored;
    }
    world->replaying = 0;
    m2WorldFree(world, backup, (size_t)backupSize);
    return result;
}

m2Result m2World_ReplayJournal(m2World* world, const void* data, int32_t size)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = WorldReplayJournalWorker(world, data, size);
    m2FloatEnvLeave(env);
    return result;
}
