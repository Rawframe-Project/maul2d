// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle fluids: a fixed-capacity particle system that lives inside
// the world when its def asks for one. Particles are stored in stable
// slots with generation-checked ids and are never reordered, so
// snapshots, the journal and rollback cover them like everything else.

#ifndef MAUL2D_PARTICLE_H
#define MAUL2D_PARTICLE_H

#include "maul2d/core_math.h"
#include "maul2d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct m2ParticleId
    {
        int32_t index1; // 1-based, 0 = null
        uint16_t world;
        uint16_t generation;
    } m2ParticleId;

    static const m2ParticleId m2_nullParticleId = {0, 0, 0};

    /// Per-particle behavior flags, set at emit. Plain water is 0.
    /// Tensile particles attract their tensile neighbors (surface
    /// tension): droplets bead up and cling instead of dispersing.
    /// Viscous particles drag their neighbors (honey, syrup);
    /// powder grains repel when packed tighter than the rest stride
    /// and never cohere (sand, rubble, dust).
    typedef enum m2ParticleFlags
    {
        m2_waterParticle = 0,
        m2_tensileParticle = 1u << 0,
        m2_viscousParticle = 1u << 1,
        m2_powderParticle = 1u << 2,
        m2_springParticle = 1u << 3,  // pairwise springs captured at fill
        m2_elasticParticle = 1u << 4, // shape-restoring triads captured at fill
    } m2ParticleFlags;

    /// Emit one particle at a world position. Refuses with the null id
    /// when the world has no particle system (invalid) or when the
    /// system is full (capacity; also counted in
    /// m2Counters.particlePoolFull, so pace emitters off
    /// m2World_GetParticleCount). Journaled. Thread class: writer.
    M2_API m2ParticleId m2World_EmitParticle(m2WorldId worldId, m2Pos2 position, m2Vec2 velocity,
                                             uint32_t flags);

    /// Destroy one particle; its slot recycles FIFO under a fresh
    /// generation. Journaled. Thread class: writer.
    M2_API void m2DestroyParticle(m2ParticleId particleId);

    /// Generation-checked liveness. Thread class: reader.
    M2_API bool m2Particle_IsValid(m2ParticleId particleId);

    M2_API m2Pos2 m2Particle_GetPosition(m2ParticleId particleId);
    M2_API uint32_t m2Particle_GetFlags(m2ParticleId particleId);

    /// Give a particle a finite lifetime in seconds: it counts down by
    /// the step's dt and auto-destroys at the end of the step it
    /// reaches zero, in ascending slot order, deterministically and
    /// without a journal op (the countdown is state, so it replays and
    /// rolls back by itself). Zero, the default, means immortal.
    /// Journaled. Thread class: writer.
    M2_API void m2Particle_SetLifetime(m2ParticleId particleId, float seconds);
    M2_API float m2Particle_GetLifetime(m2ParticleId particleId);

    /// Opaque per-particle game data, copied unchanged through
    /// snapshots and journals. Journaled. Thread class: writer/reader.
    M2_API void m2Particle_SetUserData(m2ParticleId particleId, uint64_t userData);
    M2_API uint64_t m2Particle_GetUserData(m2ParticleId particleId);
    M2_API m2Vec2 m2Particle_GetVelocity(m2ParticleId particleId);

    /// Journaled. Thread class: writer.
    M2_API void m2Particle_SetVelocity(m2ParticleId particleId, m2Vec2 velocity);

    /// Live particle count. Thread class: reader.
    M2_API int32_t m2World_GetParticleCount(m2WorldId worldId);

    /// Fill a convex polygon (given in world space at position) with
    /// particles on the rest stride (0.75 diameters), row-major
    /// bottom-up, left to right: deterministic by construction. Stops
    /// quietly when the pool fills; returns the number emitted.
    /// Spring and elastic flags make the batch a body: springs
    /// remember their spawn lengths, elastic triads remember their
    /// spawn shape, both captured here, journaled as one op, and
    /// carried by every snapshot. Thread class: writer.
    M2_API int32_t m2World_FillPolygonWithParticles(m2WorldId worldId, const m2Polygon* polygon,
                                                    m2Pos2 position, m2Vec2 velocity,
                                                    uint32_t flags);

    /// Live particles whose centers lie inside the box: ascending
    /// slot order, truthful total, NULL ids with zero capacity is a
    /// count query (the enumeration contract). Circular regions are
    /// one distance filter away on the caller's side.
    /// Thread class: reader.
    M2_API int32_t m2World_OverlapParticlesAabb(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper,
                                                m2ParticleId* ids, int32_t capacity);

    /// Fill ids with live particles in ascending slot order; returns
    /// the truthful total even beyond capacity (the enumeration
    /// contract). NULL ids with zero capacity is a count query.
    /// Thread class: reader.
    M2_API int32_t m2World_GetParticles(m2WorldId worldId, m2ParticleId* ids, int32_t capacity);

#ifdef __cplusplus
}
#endif

#endif
