// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Drives the seeded API walk (walk.h) from fuzzer input: every draw the
// walk makes takes the next eight input bytes, so coverage steers which
// calls it makes, with which arguments. Each input runs every promise of
// the walk: a journaled run that replays onto its bits, a rollback that
// returns them and snapshots back byte for byte, and an unjournaled and a
// threaded twin that reach the same hash. A broken promise crashes, so
// libFuzzer keeps the input.

#include "walk.h"

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
    (void)WalkSeed(0x5EED5EEDull, s_journal);
    s_walkBytes = NULL;
    s_walkByteCount = 0;
    if (s_failures > 0)
    {
        fflush(stdout); // CHECK printed the promise that broke
        abort();
    }
    return 0;
}
