// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for chain.c.

#ifndef MAUL2D_SRC_CHAIN_H
#define MAUL2D_SRC_CHAIN_H

#include "world_internal.h"

void m2RetireChainSlot(m2World* world, int32_t chainIndex);

// A def is valid only when its internalValue matches its cookie.
#define M2_CHAIN_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2ChainDef) << 8) ^ 9)

#endif // MAUL2D_SRC_CHAIN_H
