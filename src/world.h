// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for world.c.

#ifndef MAUL2D_SRC_WORLD_H
#define MAUL2D_SRC_WORLD_H

#include "world_internal.h"

// A def is valid only when its internalValue matches its cookie.
#define M2_WORLD_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)

#endif // MAUL2D_SRC_WORLD_H
