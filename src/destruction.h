// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for destruction.c.

#ifndef MAUL2D_SRC_DESTRUCTION_H
#define MAUL2D_SRC_DESTRUCTION_H

#include "world_internal.h"

// A def is valid only when its internalValue matches its cookie.
#define M2_EXPLODE_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2ExplosionDef) << 8) ^ 13)

#endif // MAUL2D_SRC_DESTRUCTION_H
