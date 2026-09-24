// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The 64-bit hash every determinism gate is built on. The same function
// in both engines of the family.

#include "maul2d/base.h"

#include <string.h>

uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount)
{
    // Eight bytes a round: each word, read in the host's byte order like
    // every value the hash covers, is xored into the state, the state is
    // multiplied by an odd constant and its high half folded into the
    // low half, so every input bit reaches every output bit within two
    // rounds. Leftover bytes go in one at a time (FNV-1a). The constants
    // are frozen: the hash feeds the determinism gates.
    const uint8_t* bytes = (const uint8_t*)data;
    uint64_t hash = seed;
    int32_t i = 0;
    for (; i + 8 <= byteCount; i += 8)
    {
        uint64_t word;
        memcpy(&word, bytes + i, sizeof(word));
        hash = (hash ^ word) * 0x9E3779B97F4A7C15ull;
        hash ^= hash >> 32;
    }
    for (; i < byteCount; ++i)
    {
        hash = (hash ^ bytes[i]) * 0x100000001B3ull;
    }
    return hash;
}
