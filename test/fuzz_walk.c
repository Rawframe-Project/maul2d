// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Drives the seeded API walk (walk.h) from fuzzer input: every draw the
// walk makes takes the next eight input bytes, so coverage steers which
// calls it makes, with which arguments. Once the bytes run out, the walk
// draws from a seed hashed from the whole input, so no two inputs share
// the rest of their walk. Each input runs every promise of
// the walk: a journaled run that replays onto its bits, a rollback that
// returns them and snapshots back byte for byte, and an unjournaled and a
// threaded twin that reach the same hash. A broken promise crashes, so
// libFuzzer keeps the input.

#include "walk.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    static uint8_t* s_journal;
    if (s_journal == NULL)
    {
        s_journal = (uint8_t*)malloc(WALK_JOURNAL_CAPACITY);
        if (s_journal == NULL)
        {
            abort();
        }
    }
    s_walkBytes = data;
    s_walkByteCount = size;
    uint64_t seed = 0x5EED5EEDull;
    if (size > 0)
    {
        int32_t count = size < (size_t)INT32_MAX ? (int32_t)size : INT32_MAX;
        seed = m2Hash64(M2_HASH_INIT, data, count);
        seed = seed != 0 ? seed : 0x5EED5EEDull; // xorshift stays at 0 from 0
    }
    (void)WalkSeed(seed, s_journal);
    s_walkBytes = NULL;
    s_walkByteCount = 0;
    if (s_failures > 0)
    {
        fflush(stdout); // CHECK printed the promise that broke
        abort();
    }
    return 0;
}
