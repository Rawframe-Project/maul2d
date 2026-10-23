// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The task dispatch every parallel phase goes through.

#ifndef MAUL2D_SRC_PARALLEL_H
#define MAUL2D_SRC_PARALLEL_H

#include "world_internal.h"

// Runs fn over [0, itemCount) in ranges of at least minRange: through the
// host's task hooks when it installed them, serial in the caller
// otherwise. Results never depend on the split.
void m2RunParallel(m2World* world, m2TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange);

#endif // MAUL2D_SRC_PARALLEL_H
