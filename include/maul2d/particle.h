// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle fluids: a fixed-capacity particle system that lives inside
// the world when its def asks for one. Particles are stored in stable
// slots with generation-checked ids and are never reordered, so
// snapshots, the journal and rollback cover them like everything else.
//
// A particle can die on its own, when its lifetime runs out, and no
// event reports it. A program may therefore hold a dead particle id, and
// a stale particle id counts no misuse in any call: a getter returns
// zero, a setter changes nothing, m2DestroyParticle returns
// m2_errorStale.

#ifndef MAUL2D_PARTICLE_H
#define MAUL2D_PARTICLE_H

#include "maul2d/core_math.h"
#include "maul2d/shape.h"
#include "maul2d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// A particle of a world: a 1-based slot and a generation.
    typedef struct m2ParticleId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2ParticleId;

    static const m2ParticleId m2_nullParticleId = {0, 0};

    /// Per-particle behavior, set at emit; plain water is 0. Tensile
    /// particles pull their tensile neighbors together (surface tension):
    /// droplets bead up and cling instead of spreading. Viscous particles
    /// drag their neighbors (honey, syrup). Powder grains repel when packed
    /// tighter than the rest stride and never cohere (sand, rubble, dust).
    typedef enum m2ParticleFlags
    {
        m2_waterParticle = 0,
        m2_tensileParticle = 1u << 0,
        m2_viscousParticle = 1u << 1,
        m2_powderParticle = 1u << 2,
        m2_springParticle = 1u << 3,  // pairwise springs captured by a fill
        m2_elasticParticle = 1u << 4, // shape-restoring triads captured by a fill
    } m2ParticleFlags;

    /// Emits one particle. A full pool is also counted in
    /// m2Counters.particlePoolFull; pace emitters off
    /// m2World_GetParticleCount. Journaled.
    ///
    /// @param world        The world.
    /// @param position     Where, in world space.
    /// @param velocity     Its velocity, m/s, within M2_MAX_SPEED per axis.
    /// @param flags        m2ParticleFlags values, ored together.
    /// @param particleOut  Receives the particle; the null id on failure.
    /// @return m2_success; m2_errorInvalid when the world has no particle
    ///         system, an argument is not finite or particleOut is null;
    ///         m2_errorCapacity when the pool is full, which counts no
    ///         misuse; m2_errorState from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2World_EmitParticle(m2World* world, m2Pos2 position,
                                                      m2Vec2 velocity, uint32_t flags,
                                                      m2ParticleId* particleOut);

    /// Destroys one particle, with its springs and triads. Its slot is
    /// reused in FIFO order under a new generation, and retires rather than
    /// let the generation wrap. Journaled.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return m2_success; m2_errorStale when it is already gone, which
    ///         counts no misuse; m2_errorState from host code the world is
    ///         running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2DestroyParticle(m2World* world, m2ParticleId particleId);

    /// Tells whether a particle id names a live particle. A particle can
    /// expire during any step.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return True for a live particle.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Particle_IsValid(const m2World* world, m2ParticleId particleId);

    /// Reads a particle's position.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return The position; the origin for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2Particle_GetPosition(const m2World* world, m2ParticleId particleId);

    /// Reads a particle's behavior flags.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return The m2ParticleFlags it was emitted with; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint32_t m2Particle_GetFlags(const m2World* world, m2ParticleId particleId);

    /// Gives a particle a lifetime. It counts down by each step's length,
    /// and the particle is destroyed at the end of the step it reaches
    /// zero, in ascending slot order. The countdown is world state, so it
    /// replays and rolls back by itself. A negative or non-finite value
    /// counts one misuse. Journaled.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @param seconds     The time left; 0, the default, means immortal.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Particle_SetLifetime(m2World* world, m2ParticleId particleId, float seconds);

    /// Reads a particle's remaining lifetime.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return The seconds left; 0 for an immortal particle or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Particle_GetLifetime(const m2World* world, m2ParticleId particleId);

    /// Sets a particle's opaque game data, kept through snapshots and
    /// journals. Journaled.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @param userData    Any value.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Particle_SetUserData(m2World* world, m2ParticleId particleId, uint64_t userData);

    /// Reads a particle's game data.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return The value; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2Particle_GetUserData(const m2World* world, m2ParticleId particleId);

    /// Reads a particle's velocity.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return The velocity, m/s; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Particle_GetVelocity(const m2World* world, m2ParticleId particleId);

    /// Sets a particle's velocity. A velocity beyond M2_MAX_SPEED on either
    /// axis counts one misuse.
    /// Journaled.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @param velocity    The velocity, m/s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Particle_SetVelocity(m2World* world, m2ParticleId particleId, m2Vec2 velocity);

    /// Counts the live particles.
    ///
    /// @param world  The world.
    /// @return The count; 0 for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetParticleCount(const m2World* world);

    /// Fills a convex polygon with particles on the rest stride (0.75
    /// diameters), row by row from the bottom, left to right. It stops
    /// quietly when the pool is full. With the spring or elastic flag the
    /// batch becomes a body: springs keep their spawn lengths and triads
    /// their spawn shape. Journaled as one operation.
    ///
    /// @param world     The world.
    /// @param polygon   The region, in a frame placed at position.
    /// @param position  Where the polygon's frame sits, in world space.
    /// @param velocity  The velocity of every particle, m/s, within
    ///                  M2_MAX_SPEED per axis.
    /// @param flags     m2ParticleFlags values, ored together.
    /// @return The number of particles emitted; 0 and one misuse counted
    ///         when the world has no particle system, an argument is not
    ///         finite or the polygon is wider than 256 strides; 0 from host
    ///         code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API int32_t m2World_FillPolygonWithParticles(m2World* world, const m2Polygon* polygon,
                                                    m2Pos2 position, m2Vec2 velocity,
                                                    uint32_t flags);

    /// Lists the live particles whose centers lie inside a box, in
    /// ascending slot order. A circular region is one distance check away
    /// on the caller's side.
    ///
    /// @param world     The world.
    /// @param lower     The box's lower corner.
    /// @param upper     The box's upper corner.
    /// @param ids       Receives up to capacity particles; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of particles inside, which may exceed capacity;
    ///         0 for a non-finite corner, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_OverlapParticlesAabb(const m2World* world, m2Pos2 lower, m2Pos2 upper,
                                                m2ParticleId* ids, int32_t capacity);

    /// Lists the live particles in ascending slot order.
    ///
    /// @param world     The world.
    /// @param ids       Receives up to capacity particles; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of particles, which may exceed capacity.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetParticles(const m2World* world, m2ParticleId* ids, int32_t capacity);

#ifdef __cplusplus
}
#endif

#endif
