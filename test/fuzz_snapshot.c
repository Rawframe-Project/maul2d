// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes m2World_Restore with hostile snapshot bytes. The first input byte
// picks the form: even, the rest of the input is the snapshot; odd, the
// rest is a list of (offset, byte) edits to a valid snapshot of the fuzz
// scene, which reaches past the header without seeds that a format bump
// would make stale. The promises:
//   - a refused snapshot changes nothing (the hash stays) and counts no
//     misuse, since the bytes are untrusted;
//   - an accepted one leaves a world that validates, snapshots back to the
//     same bytes, and steps.

#include "fuzz_scene.h"

#include "maul2d/maul2d.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static m2World* s_world;
static uint8_t* s_valid;
static uint8_t* s_bytes;
static uint8_t* s_again;
static int64_t s_size;

static void Setup(void)
{
    m2WorldDef def = FuzzSceneDef();
    s_world = NewWorld(&def);
    if (s_world == NULL || !BuildFuzzScene(s_world))
    {
        FuzzFail("the fuzz scene builds");
    }
    s_size = m2World_SnapshotSize(s_world);
    s_valid = (uint8_t*)malloc((size_t)s_size);
    s_bytes = (uint8_t*)malloc((size_t)s_size);
    s_again = (uint8_t*)malloc((size_t)s_size);
    if (s_valid == NULL || s_bytes == NULL || s_again == NULL ||
        Snapshot(s_world, s_valid, s_size) != s_size)
    {
        FuzzFail("the fuzz scene snapshots");
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
        memcpy(s_bytes, s_valid, (size_t)s_size);
        for (size_t i = 1; i + 2 < size; i += 3)
        {
            uint32_t offset = ((uint32_t)data[i] | ((uint32_t)data[i + 1] << 8)) % (uint32_t)s_size;
            s_bytes[offset] = data[i + 2];
        }
        bytes = s_bytes;
        count = s_size;
    }

    uint64_t hash = m2World_Hash(s_world);
    uint64_t misuse = m2World_GetCounters(s_world).misuse;
    m2Result result = m2World_Restore(s_world, bytes, count);
    if (m2World_GetCounters(s_world).misuse != misuse)
    {
        FuzzFail("untrusted snapshot bytes count no misuse");
    }
    if (result != m2_success)
    {
        if (m2World_Hash(s_world) != hash)
        {
            FuzzFail("a refused snapshot changes nothing");
        }
        return 0;
    }
    if (!m2World_Validate(s_world))
    {
        FuzzFail("an accepted snapshot leaves a valid world");
    }
    if (Snapshot(s_world, s_again, s_size) != count || memcmp(s_again, bytes, (size_t)count) != 0)
    {
        FuzzFail("an accepted snapshot snapshots back to its bytes");
    }
    for (int32_t i = 0; i < 4; ++i)
    {
        m2World_Step(s_world, 1.0f / 60.0f, 4);
    }
    if (!m2World_Validate(s_world))
    {
        FuzzFail("a world from an accepted snapshot steps into a valid world");
    }
    if (!Restore(s_world, s_valid, s_size))
    {
        FuzzFail("the valid snapshot restores");
    }
    return 0;
}
