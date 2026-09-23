// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for particle.c.

#ifndef MAUL2D_SRC_PARTICLE_H
#define MAUL2D_SRC_PARTICLE_H

#include "world_internal.h"

// The particle step: neighbor pairs from positions, then the solve.
void m2UpdateParticlePairs(m2World* world);
void m2SolveParticles(m2World* world, float dt);

#endif // MAUL2D_SRC_PARTICLE_H
