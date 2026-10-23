// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particles: the step's neighbor pairs, body contacts and solve, and
// the destroy a lifetime expiry uses.

#ifndef MAUL2D_SRC_PARTICLE_H
#define MAUL2D_SRC_PARTICLE_H

#include "world_internal.h"

// The particle step: neighbor pairs and body contacts from positions,
// then the solve.
void m2UpdateParticlePairs(m2World* world);
void m2UpdateParticleContacts(m2World* world);
void m2SolveParticles(m2World* world, float dt);

// Destroys a live particle slot with its springs and triads, without
// journaling: a lifetime expiry inside the step comes through here.
void m2DestroyParticleInternal(m2World* world, int32_t index);

#endif // MAUL2D_SRC_PARTICLE_H
