// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes m2World_ReplayJournal with hostile tapes. The first input byte
// picks the form: even, the rest of the input is the tape; odd, the rest
// is a list of (offset, byte) edits to a valid tape that builds the fuzz
// scene, then rolls part of it back, which reaches past the header
// without seeds that a format bump would make stale. The promises:
//   - a refused tape changes nothing (replay is atomic) and counts no
//     misuse, since the bytes are untrusted;
//   - an accepted one leaves a world that validates and steps.

#include "fuzz_scene.h"

#include "maul2d/maul2d.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum
{
    FUZZ_TAPE_ROOM = 1 << 16
};

static m2World* s_world;
static uint8_t* s_empty; // a snapshot of s_world before any replay
static int64_t s_emptySize;
static uint8_t* s_tape;
static uint8_t* s_bytes;
static int64_t s_tapeSize;

// Records the scene, a snapshot, a few more steps and a restore, so the
// tape holds creates, steps, a setter and a restore record.
static void RecordTape(void)
{
    m2WorldDef def = FuzzSceneDef();
    m2World* recorder = NewWorld(&def);
    if (recorder == NULL)
    {
        FuzzFail("the recording world builds");
    }
    int64_t capacity = m2World_GetJournalBaseSize(recorder) * 2 + FUZZ_TAPE_ROOM;
    s_tape = (uint8_t*)malloc((size_t)capacity);
    int64_t snapshotSize = m2World_SnapshotSize(recorder);
    uint8_t* snapshot = (uint8_t*)malloc((size_t)snapshotSize);
    if (s_tape == NULL || snapshot == NULL || !StartJournal(recorder, s_tape, capacity) ||
        !BuildFuzzScene(recorder) || Snapshot(recorder, snapshot, snapshotSize) != snapshotSize)
    {
        FuzzFail("the fuzz scene records");
    }
    m2World_SetGravity(recorder, (m2Vec2){1.0f, -9.0f});
    for (int32_t i = 0; i < 5; ++i)
    {
        m2World_Step(recorder, 1.0f / 60.0f, 4);
    }
    if (!Restore(recorder, snapshot, snapshotSize))
    {
        FuzzFail("the fuzz scene rolls back");
    }
    s_tapeSize = StopJournal(recorder);
    s_bytes = (uint8_t*)malloc((size_t)s_tapeSize);
    if (s_tapeSize <= 0 || s_bytes == NULL)
    {
        FuzzFail("the fuzz tape stops");
    }
    free(snapshot);
    m2DestroyWorld(recorder);
}

static void Setup(void)
{
    RecordTape();
    m2WorldDef def = FuzzSceneDef();
    s_world = NewWorld(&def);
    if (s_world == NULL)
    {
        FuzzFail("the replay world builds");
    }
    s_emptySize = m2World_SnapshotSize(s_world);
    s_empty = (uint8_t*)malloc((size_t)s_emptySize);
    if (s_empty == NULL || Snapshot(s_world, s_empty, s_emptySize) != s_emptySize)
    {
        FuzzFail("the replay world snapshots");
    }
    if (!ReplayJournal(s_world, s_tape, s_tapeSize) || !m2World_Validate(s_world))
    {
        FuzzFail("the valid tape replays");
    }
    if (!Restore(s_world, s_empty, s_emptySize))
    {
        FuzzFail("the replay world resets");
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_world == NULL)
    {
        Setup();
    }
    if (size == 0)
    {
        return 0;
    }
    const uint8_t* bytes = data + 1;
    int64_t count = (int64_t)(size - 1);
    if ((data[0] & 1u) != 0)
    {
        memcpy(s_bytes, s_tape, (size_t)s_tapeSize);
        for (size_t i = 1; i + 2 < size; i += 3)
        {
            uint32_t offset =
                ((uint32_t)data[i] | ((uint32_t)data[i + 1] << 8)) % (uint32_t)s_tapeSize;
            s_bytes[offset] = data[i + 2];
        }
        bytes = s_bytes;
        count = s_tapeSize;
    }

    uint64_t hash = m2World_Hash(s_world);
    uint64_t misuse = m2World_GetCounters(s_world).misuse;
    m2Result result = m2World_ReplayJournal(s_world, bytes, count);
    if (m2World_GetCounters(s_world).misuse != misuse)
    {
        FuzzFail("untrusted tape bytes count no misuse");
    }
    if (result != m2_success)
    {
        if (m2World_Hash(s_world) != hash)
        {
            FuzzFail("a refused tape changes nothing");
        }
        return 0;
    }
    if (!m2World_Validate(s_world))
    {
        FuzzFail("an accepted tape leaves a valid world");
    }
    for (int32_t i = 0; i < 4; ++i)
    {
        m2World_Step(s_world, 1.0f / 60.0f, 4);
    }
    if (!m2World_Validate(s_world))
    {
        FuzzFail("a world from an accepted tape steps into a valid world");
    }
    if (!Restore(s_world, s_empty, s_emptySize))
    {
        FuzzFail("the replay world resets");
    }
    return 0;
}
