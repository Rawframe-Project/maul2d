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

// The most rest strides a particle fill spans across: a polygon wider
// than this many strides is refused.
#define M2_MAX_FILL_COLUMNS 256

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

    /// Per-particle behavior, set at creation; plain water is 0. Tensile
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

    /// The def of a particle, or of every particle of a polygon fill.
    /// Initialize it with m2DefaultParticleDef.
    typedef struct m2ParticleDef
    {
        m2Pos2 position;       // world space; for a fill, where the polygon's frame sits
        m2Vec2 velocity;       // m/s, within M2_SPEED_LIMIT per axis
        uint32_t flags;        // m2ParticleFlags values, ored together
        int32_t internalValue; // the cookie m2DefaultParticleDef sets
    } m2ParticleDef;

    /// A particle def with its defaults: a water particle at rest at the
    /// origin.
    ///
    /// @return The def.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2ParticleDef m2DefaultParticleDef(void);

    /// Creates one particle. A full pool is also counted in
    /// m2Counters.particlePoolFull; pace emitters off
    /// m2World_GetParticleCount. Journaled.
    ///
    /// @param world        The world.
    /// @param def          The particle, from m2DefaultParticleDef.
    /// @param particleOut  Receives the particle; the null id on failure.
    /// @return m2_success; m2_errorInvalid when def or particleOut is
    ///         null, def was not initialized by m2DefaultParticleDef or a
    ///         value of it is NaN or not finite where it has no bound;
    ///         m2_errorLimit for a velocity past M2_SPEED_LIMIT;
    ///         m2_errorCapacity when the pool is full, a pool of 0 included,
    ///         which counts no misuse; m2_errorBusy from host code the world
    ///         is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateParticle(m2World* world, const m2ParticleDef* def,
                                                  m2ParticleId* particleOut);

    /// Destroys one particle, with its springs and triads. Its slot is
    /// reused in FIFO order under a new generation, and retires rather than
    /// let the generation wrap. Journaled.
    ///
    /// @param world       The world.
    /// @param particleId  The particle.
    /// @return m2_success; m2_errorStale when it is already gone, which
    ///         counts no misuse; m2_errorBusy from host code the world is
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
    /// @return The m2ParticleFlags it was created with; 0 for a stale id.
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

    /// Sets a particle's velocity. A velocity beyond M2_SPEED_LIMIT on either
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
    /// diameters), row by row from the bottom, left to right. A full pool
    /// ends the fill early: the particles created so far stay, and the
    /// fill returns m2_errorCapacity with their count, which may be 0. With
    /// the spring or elastic flag the batch becomes a body: springs keep
    /// their spawn lengths and triads their spawn shape. Journaled as one
    /// operation, a fill cut short included.
    ///
    /// @param world     The world.
    /// @param def       Every particle's velocity and flags, and where the
    ///                  polygon's frame sits, from m2DefaultParticleDef.
    /// @param polygon   The region, in the frame at def->position: a
    ///                  polygon a shape would take, within M2_LOCAL_LIMIT, convex
    ///                  and not a sliver. Its normals are recomputed from
    ///                  its vertices.
    /// @param countOut  Receives the number of particles created; 0 when
    ///                  the fill is refused.
    /// @return m2_success; m2_errorCapacity when the pool ran out before
    ///         the polygon was full, a pool of 0 included, which counts no
    ///         misuse and counts once in m2Counters.particlePoolFull;
    ///         m2_errorInvalid when an argument is null, def was not
    ///         initialized by m2DefaultParticleDef, a value of it is NaN or
    ///         the polygon is not one a shape would take;
    ///         m2_errorLimit for a velocity or a polygon past its published
    ///         bound, or a polygon wider than M2_MAX_FILL_COLUMNS strides;
    ///         m2_errorBusy from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateParticlesInPolygon(m2World* world,
                                                            const m2ParticleDef* def,
                                                            const m2Polygon* polygon,
                                                            int32_t* countOut);

    /// Lists the live particles whose centers lie inside a box, in
    /// ascending slot order. A circular region is one distance check away
    /// on the caller's side.
    ///
    /// @param world     The world.
    /// @param box       The box, in world space.
    /// @param ids       Receives up to capacity particles; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of particles inside, which may exceed capacity;
    ///         0 for a non-finite corner, which counts one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_OverlapParticlesAabb(const m2World* world, m2WorldAabb box,
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
