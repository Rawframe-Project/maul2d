// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fluid volumes and wind: forces from water regions and moving air.

#ifndef MAUL2D_SRC_BUOYANCY_H
#define MAUL2D_SRC_BUOYANCY_H

#include "world_internal.h"

// A def is valid only when its internalValue matches its cookie.
#define M2_FVOLUME_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2FluidVolumeDef) << 8) ^ 17)

// Per-step forces on bodies: buoyancy and drag in the fluid volumes,
// then the ambient wind.
void m2ApplyFluidVolumes(m2World* world, float dt);
void m2ApplyWind(m2World* world, float dt);

#endif // MAUL2D_SRC_BUOYANCY_H
