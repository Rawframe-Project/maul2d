// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: creation, stepping, tuning, snapshots, the command
// journal, counters and diagnostics.

#ifndef MAUL2D_WORLD_H
#define MAUL2D_WORLD_H

#include "maul2d/core_math.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// Def cookie base. Default functions set internalValue to
/// M2_COOKIE ^ (sizeof(def) << 8) ^ defTypeTag; create calls reject
/// anything else, zero-initialized defs included. The high byte is
/// always nonzero, so no legal cookie can collide with small garbage.
#define M2_COOKIE 0x6D32C0DE

    /// A world: the bodies, shapes, joints, particles and volumes it owns,
    /// and everything a step needs. Worlds share nothing, so separate worlds
    /// may be created, stepped and destroyed on separate threads.
    typedef struct m2World m2World;

    /// A range job: run items [startIndex, endIndex).
    typedef void m2TaskFn(int32_t startIndex, int32_t endIndex, void* taskContext);
    /// Splits [0, itemCount) into subranges of at least minRange and runs
    /// task over them on the host's scheduler; the return value is handed
    /// to finishTask, which waits until all of them are done.
    typedef void* m2EnqueueTaskFn(m2TaskFn* task, int32_t itemCount, int32_t minRange,
                                  void* taskContext, void* userContext);
    typedef void m2FinishTaskFn(void* userTask, void* userContext);

    /// The def of a world. Initialize it with m2DefaultWorldDef.
    typedef struct m2WorldDef
    {
        // The cookie and the ABI version stay the first two fields in every
        // version, so that m2CreateWorld can refuse a def of another layout
        // before it reads anything else.
        int32_t internalValue;
        // M2_ABI_VERSION of the headers the program was built with.
        uint32_t version;
        m2Vec2 gravity;       // m/s^2, within M2_MAX_ACCELERATION per axis
        int32_t bodyCapacity; // fixed pool sizes: a world never grows
        int32_t shapeCapacity;
        int32_t jointCapacity;
        bool enableSleeping; // islands may sleep
        /// particleCapacity above 0 builds the particle system into the
        /// world for its whole life, so every snapshot has the same shape
        /// and rollback stays safe at any point in history. Its parameters
        /// are fixed here like every other physics setting.
        int32_t particleCapacity;    // 0: no fluids
        int32_t fluidVolumeCapacity; // 0: no buoyancy volumes
        float particleRadius;        // meters, 0.02 to M2_HUGE
        float particleDensity;       // kg/m^2; a particle weighs M2_MIN_MASS or more
        float particleGravityScale;  // within M2_MAX_GRAVITY_SCALE
        /// The particle model's strengths: the pressure above rest density,
        /// the damping of approach speed, shear viscosity for viscous
        /// particles, the spacing push of powder grains, the pull of
        /// spring nets and elastic triads, and for tensile particles the
        /// cohesion below rest density and the near pressure that keeps a
        /// droplet from collapsing. Each is 0 to M2_MAX_COEFFICIENT.
        float particlePressureStrength;
        float particleDampingStrength;
        float particleViscousStrength;
        float particlePowderStrength;
        float particleSpringStrength;
        float particleElasticStrength;
        float particleCohesionStrength;
        float particleNearPressureStrength;
        /// The host's task system. The step hands enqueueTask a range job;
        /// the host runs it on its own scheduler, splitting [0, itemCount)
        /// into subranges of at least minRange, and finishTask returns only
        /// when every subrange is done. Both null, the default, runs the
        /// work serially on the calling thread. Any executor and any split
        /// give the same bits.
        m2EnqueueTaskFn* enqueueTask;
        m2FinishTaskFn* finishTask;
        void* userTaskContext;
        /// The allocator the world takes all its memory from, kept for its
        /// lifetime. Zeroed, it is the C library's. Set both functions or
        /// neither.
        m2Allocator allocator;
    } m2WorldDef;

    /// THREADS. Readers (queries, getters, diagnostics, drawing) take a
    /// const world and may run at the same time as each other, never with
    /// m2World_Step or another writer. Writers (creates and destroys,
    /// setters, impulses, the step) need the world to themselves. Separate
    /// worlds share nothing. The host code a world runs (the task hooks,
    /// the debug draw callbacks, the allocator) must not change that world:
    /// its writers refuse with m2_errorState, or count one misuse where
    /// they return void. Readers stay open to it.
    ///
    /// FLOATING POINT. A call that computes runs in the library's own
    /// floating-point setting, whatever the calling thread's is: it rounds
    /// to nearest, keeps subnormals and masks every exception, and gives the
    /// thread back its rounding mode and flush-to-zero and denormals-are-zero
    /// bits as they were. A host may run its threads in any setting and
    /// still gets the same bits. The host code a world runs
    /// (the task hooks, the debug draw callbacks, the allocator) runs in the
    /// thread's own setting, and a range job sets the library's on whatever
    /// thread the host runs it.
    ///
    /// IDS. An id names an object of the world that issued it: a 1-based
    /// slot and a generation. A correct program never uses an id after
    /// destroying its object, so a stale id counts one misuse: a getter
    /// returns the zeroed value, a setter changes nothing, and a call that
    /// returns a status returns m2_errorStale. Particles are the exception:
    /// the world ends a particle on its own when its lifetime runs out, so
    /// a program may hold a dead particle id, and a stale particle id counts
    /// nothing. The validity queries (m2Body_IsValid and the others) count
    /// nothing either. An id carries no world: one passed to another world
    /// names whatever that world holds in the same slot and generation, and
    /// the library cannot tell. A slot takes a new generation each time it
    /// is reused, and after 2^32 objects it retires instead of starting
    /// over, so no id ever names two objects. A pool of capacity n
    /// therefore makes n * 2^32 objects over a world's life, snapshots
    /// included; then its creates return m2_errorCapacity.
    ///
    /// MISUSE. m2Counters.misuse counts the bugs of the calling code: an
    /// argument a correct program never passes, a stale id, a writer called
    /// from host code the world is running. Untrusted input is not misuse:
    /// a snapshot or journal tape that fails its checks, and a value one of
    /// a tape's ops refuses while it replays, return m2_errorInvalid and
    /// count nothing, so bytes from outside cannot inflate the counter.
    /// Host code a replay runs (a task hook, the allocator) is the calling
    /// code's: what it gets wrong counts, replay or not. A full pool counts
    /// nothing either.

    /// Returns the library's default world def with the version left zero.
    /// Call m2DefaultWorldDef instead, which stamps the version of the
    /// program's headers.
    ///
    /// @return The library's defaults: gravity (0, -10), 1024 bodies, 2048
    ///         shapes, 256 joints, sleeping on, no particles or volumes.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2WorldDef m2DefaultWorldDefValues(void);

    /// Returns a world def with the library's defaults, a valid cookie and
    /// the ABI version of the headers the program is built with. It is
    /// built in the program, so m2CreateWorld can refuse a def made for
    /// another release's binary interface.
    ///
    /// @return The default def.
    /// @par Thread safety
    /// Safe from any thread.
    static inline m2WorldDef m2DefaultWorldDef(void)
    {
        m2WorldDef def = m2DefaultWorldDefValues();
        def.version = M2_ABI_VERSION;
        return def;
    }

    /// Creates a world. Every byte the world keeps is allocated here, from
    /// the def's allocator, and pools never grow; afterwards only a journal
    /// replay takes scratch memory from it, and gives it back.
    ///
    /// @param def       The world, from m2DefaultWorldDef.
    /// @param worldOut  Receives the world; null on any failure.
    /// @return m2_success, or the first failed check in this order:
    ///         m2_errorInvalid for a null argument; m2_errorUnsupported
    ///         when this CPU cannot run the library's SIMD backend;
    ///         m2_errorInvalid for a bad cookie; m2_errorVersion for a def
    ///         built for another binary interface; m2_errorInvalid for the
    ///         rest of the def (a capacity below 1, an allocator with one
    ///         function set, a gravity beyond M2_MAX_ACCELERATION, a
    ///         particle parameter out of its range);
    ///         m2_errorCapacity when the allocator refuses or a size does
    ///         not fit.
    /// @par Thread safety
    /// Safe from any thread.
    M2_NODISCARD M2_API m2Result m2CreateWorld(const m2WorldDef* def, m2World** worldOut);

    /// Destroys a world and everything in it, and returns its memory to its
    /// allocator. Ids into it become meaningless.
    ///
    /// @param world  The world; null does nothing.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2DestroyWorld(m2World* world);

    /// Walks the whole world and checks its invariants: every live
    /// position, velocity and particle finite, every body rotation of unit
    /// length, every joint and jelly net
    /// attached to live objects, the counts consistent, the pairs in key
    /// order. It costs a full walk, so call it from debug paths; building
    /// with -DMAUL2D_VALIDATE=ON asserts it after every step.
    ///
    /// @param world  The world.
    /// @return True when the world is sound; false at the first broken
    ///         invariant and for a null world. It never stops the program.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2World_Validate(const m2World* world);

    /// Sets the gravity and wakes every sleeping dynamic body: a stack must
    /// not float in a world that turned upside down. A gravity beyond
    /// M2_MAX_ACCELERATION on either axis counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param gravity  The acceleration, m/s^2.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2World_SetGravity(m2World* world, m2Vec2 gravity);

    /// Reads the gravity.
    ///
    /// @param world  The world.
    /// @return The acceleration; zero for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2World_GetGravity(const m2World* world);

    /// Sets the global wind. Each step every awake dynamic body feels
    /// -linearDrag * shapeArea * (bodyVelocity - windVelocity), added like
    /// gravity, so larger bodies catch more wind and everything drifts
    /// toward the wind's speed. Unlike gravity it wakes no sleeper, since
    /// wind is meant to gust. Regional wind is a fluid volume of density 0.
    /// A velocity beyond M2_MAX_SPEED on either axis or a drag outside 0 to
    /// M2_MAX_RATE counts one misuse. Journaled.
    ///
    /// @param world       The world.
    /// @param velocity    The air's velocity, m/s.
    /// @param linearDrag  The drag coefficient; 0, the default, is off and
    ///                    costs nothing.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2World_SetWind(m2World* world, m2Vec2 velocity, float linearDrag);

    /// Reads the global wind.
    ///
    /// @param world          The world; null writes zeros.
    /// @param velocityOut    Receives the air's velocity; may be null.
    /// @param linearDragOut  Receives the drag coefficient; may be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API void m2World_GetWind(const m2World* world, m2Vec2* velocityOut, float* linearDragOut);

    /// One blast. Every dynamic, non-sensor shape whose category passes
    /// maskBits and whose closest point lies within radius + falloff gets
    /// an outward impulse at that point: full strength inside radius,
    /// fading to zero across the falloff band.
    typedef struct m2ExplosionDef
    {
        m2Pos2 position;
        float radius;  // 0 to M2_HUGE
        float falloff; // above 0, to M2_HUGE
        float impulse; // N s at full strength, within M2_IMPULSE_LIMIT
        M2_ALIGN8 uint64_t maskBits;
        int32_t internalValue; // the cookie m2DefaultExplosionDef sets
    } m2ExplosionDef;

    /// Returns an explosion def: radius 1, falloff 0.5, impulse 1, every
    /// category.
    ///
    /// @return The default def; set the position.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2ExplosionDef m2DefaultExplosionDef(void);

    /// Sets off an explosion. Each body's velocity saturates at M2_MAX_SPEED
    /// per axis. A bad def counts one misuse. Journaled as one
    /// operation.
    ///
    /// @param world  The world.
    /// @param def    The blast, from m2DefaultExplosionDef.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2World_Explode(m2World* world, const m2ExplosionDef* def);

    /// Allows or forbids sleep in the whole world. Forbidding wakes every
    /// sleeping dynamic body. Journaled.
    ///
    /// @param world  The world.
    /// @param flag   True to allow sleep.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2World_EnableSleeping(m2World* world, bool flag);

    /// Tells whether the world allows sleep.
    ///
    /// @param world  The world.
    /// @return True when allowed; false for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2World_IsSleepingEnabled(const m2World* world);

    /// Debug drawing: the world walks its state and calls back, and the
    /// host renders. Polygon vertices arrive body-local with the body's
    /// f64 origin and rotation, so a renderer can subtract its camera in
    /// doubles and stay steady far from the origin. A null callback skips
    /// that primitive. Colors are 0xRRGGBB hints.
    typedef struct m2DebugDraw
    {
        void (*drawPolygon)(const m2Vec2* localVertices, int32_t count, m2Pos2 origin,
                            m2Rot rotation, uint32_t color, void* context);
        void (*drawCircle)(m2Pos2 center, float radius, m2Rot rotation, uint32_t color,
                           void* context);
        void (*drawCapsule)(m2Pos2 p1, m2Pos2 p2, float radius, uint32_t color, void* context);
        void (*drawSegment)(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context);
        void (*drawPoint)(m2Pos2 p, float size, uint32_t color, void* context);
        bool drawShapes;
        bool drawJoints;
        bool drawContacts;
        bool drawContactForces; // per-point normal and friction impulse arrows
        bool drawAabbs;
        /// World units per unit of impulse for the force arrows; 0 or less
        /// draws them at the raw impulse.
        float forceScale;
        void* context;
    } m2DebugDraw;

    /// Draws the world through the callbacks. The callbacks are host code
    /// the world runs: they may read the world, never change it.
    ///
    /// @param world  The world; null draws nothing.
    /// @param draw   The callbacks and options; null draws nothing.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API void m2World_Draw(const m2World* world, const m2DebugDraw* draw);

    /// Wall-clock times of the last step. They are not deterministic and
    /// never enter a snapshot or a hash.
    typedef struct m2Profile
    {
        float stepMs;     // the whole m2World_Step
        float pairsMs;    // broadphase and pair update
        float contactsMs; // narrowphase manifolds
        float solveMs;    // solver, continuous collision included
        float sleepMs;    // islands and sleep accounting
    } m2Profile;

    /// Counts derived from world state, so they are deterministic.
    typedef struct m2Counters
    {
        int32_t bodies;
        int32_t awakeBodies;
        int32_t shapes;
        int32_t joints;
        int32_t pairs;
        int32_t touchingPairs;
        int32_t constraints;         // solved last step
        int32_t graphColors;         // colors used last step
        int32_t overflowConstraints; // in the serial bucket last step
        M2_ALIGN8 uint64_t stepCount;
        // Counted in every build. Poll them once per frame and alarm on
        // growth: there is no per-call callback, so healthy calls pay
        // nothing for the diagnosis.
        int32_t pairOverflow;                // body pairs dropped last step (pair table full)
        int32_t particlePairOverflow;        // particle pairs dropped last step (budget)
        int32_t particleBodyOverflow;        // particle body contacts dropped last step
        M2_ALIGN8 uint64_t particlePoolFull; // emits refused by a full pool, cumulative
        M2_ALIGN8 uint64_t misuse;           // calls refused as misuse, cumulative
    } m2Counters;

    /// Sums the kinetic energy of the awake dynamic bodies, in f64 and in
    /// body order, so two worlds given the same calls report the same
    /// bits. Sleeping bodies add nothing.
    ///
    /// @param world  The world.
    /// @return The energy, joules; 0 for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API double m2World_GetKineticEnergy(const m2World* world);

    /// Reads the times of the last step.
    ///
    /// @param world  The world.
    /// @return The times; zeroed for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Profile m2World_GetProfile(const m2World* world);

    /// Reads the counters.
    ///
    /// @param world  The world.
    /// @return The counters; zeroed for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Counters m2World_GetCounters(const m2World* world);

    /// Advances the simulation. Identical worlds given identical calls
    /// reach identical bits on every supported platform. Feed the same dt
    /// every step: 1/60 with 4 substeps is the tuned default. A dt outside
    /// M2_MIN_TIME_STEP to M2_MAX_TIME_STEP, or a substep count below 1 or
    /// above M2_MAX_SUBSTEPS, changes nothing and counts one misuse.
    /// Journaled.
    ///
    /// @param world         The world.
    /// @param dt            The step, seconds, from M2_MIN_TIME_STEP (1e-6)
    ///                      to M2_MAX_TIME_STEP (10).
    /// @param substepCount  The solver substeps, 1 to M2_MAX_SUBSTEPS.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2World_Step(m2World* world, float dt, int32_t substepCount);

    /// Counts the steps taken since creation; a restore brings back the
    /// snapshot's count.
    ///
    /// @param world  The world.
    /// @return The count; 0 for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2World_GetStepCount(const m2World* world);

    /// Returns the size of this world's snapshot. It is fixed for the
    /// world's life.
    ///
    /// @param world  The world.
    /// @return The size in bytes; 0 for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_SnapshotSize(const m2World* world);

    /// Writes the whole simulation state into caller memory.
    ///
    /// @param world     The world.
    /// @param buffer    Receives the snapshot.
    /// @param capacity  The length of buffer, in bytes.
    /// @param bytesOut  Receives the bytes written; with m2_errorCapacity,
    ///                  the size needed; 0 on any other failure.
    /// @return m2_success; m2_errorInvalid for a null argument;
    ///         m2_errorCapacity when capacity is below
    ///         m2World_SnapshotSize, with nothing written.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_NODISCARD M2_API m2Result m2World_Snapshot(const m2World* world, void* buffer,
                                                  int32_t capacity, int32_t* bytesOut);

    /// Restores a snapshot of a world with the same def shape. Everything,
    /// bodies, id pools and the step count included, returns to the
    /// snapshot's instant, bit for bit, and the event streams clear. Only
    /// the release that wrote a snapshot reads it (M2_ABI_VERSION). The
    /// buffer is checked before any byte lands: a header from another
    /// release, build or world shape, an index or count out of range,
    /// flags that are not flags and non-finite state are refused, and the
    /// world stays as it was. The checks make every index safe; they do not
    /// rebuild every structural invariant (tree shape, list links), so
    /// restore only snapshots this library wrote. A restore while recording
    /// is recorded.
    ///
    /// @param world   The world.
    /// @param buffer  The snapshot.
    /// @param size    Its length, in bytes.
    /// @return m2_success; m2_errorVersion for a snapshot another release
    ///         or build wrote, and m2_errorInvalid for other bytes that fail
    ///         the checks, neither of which counts misuse; m2_errorInvalid
    ///         for a null buffer, which counts one; m2_errorState from host
    ///         code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2World_Restore(m2World* world, const void* buffer, int32_t size);

    /// Hashes every piece of state the step reads and a call can change:
    /// the world's gravity, wind, sleeping switch and step count; every
    /// live body, shape, chain, joint, particle and fluid volume, each
    /// field keyed by its slot (pose, velocities, forces, mass data,
    /// damping, gravity scale, locks, dominance, sleep and enable state;
    /// geometry, materials, filter and sensor flag; joint anchors,
    /// lengths, ranges, motor, spring and break settings, targets and
    /// impulses; particle and volume settings); the contact pairs, their
    /// touching flags and manifolds; each shape's fat box and moved flag in
    /// the broadphase, from which the next pairs are found; the particle
    /// springs and triads; and each id pool's free order and generations,
    /// which decide where the next object lands and the id it gets. Two
    /// worlds with the same hash step on to the same bits. Not hashed: user
    /// data, events, and the broadphase tree's shape, which decides only
    /// the order a query meets the boxes in. The value changes between
    /// library versions. Present in every build: it is how a divergence is
    /// found.
    ///
    /// @param world  The world.
    /// @return The hash; 0 for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2World_Hash(const m2World* world);

    /// A world's memory footprint.
    typedef struct m2MemoryUsage
    {
        M2_ALIGN8 int64_t
            persistentBytes; // everything creation allocated, the world struct included
    } m2MemoryUsage;

    /// Reports a world's memory footprint. It is fixed for the world's
    /// life: pools never grow, step scratch lives in them, and a journal's
    /// buffer is the host's.
    ///
    /// @param world  The world.
    /// @return The footprint; zeroed for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2MemoryUsage m2World_GetMemoryUsage(const m2World* world);

    /// Hashes of the world's parts, for hunting a divergence.
    typedef struct m2WorldHashParts
    {
        M2_ALIGN8 uint64_t world;     // step count, gravity, wind and sleeping
        M2_ALIGN8 uint64_t bodies;    // bodies, their shapes and chains, and their id pools
        M2_ALIGN8 uint64_t contacts;  // pairs, their touching flags and manifolds
        M2_ALIGN8 uint64_t joints;    // joint settings and impulses, and the joint id pool
        M2_ALIGN8 uint64_t particles; // particles and the jelly nets, and the particle id pool
        M2_ALIGN8 uint64_t volumes;   // fluid volumes and their id pool
    } m2WorldHashParts;

    /// Hashes each part of the world. Compare two runs part by part each
    /// step: the step where a part first differs names the moment, and the
    /// part names the subsystem. The values are diagnostics: they change
    /// between library versions, and m2World_Hash is not built from them.
    ///
    /// @param world  The world.
    /// @return The hashes; zeroed for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2WorldHashParts m2World_GetHashParts(const m2World* world);

    /// A fluid volume of a world: a 1-based slot and a generation.
    typedef struct m2FluidVolumeId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2FluidVolumeId;

    static const m2FluidVolumeId m2_nullFluidVolumeId = {0, 0};

    /// Water without particles: an axis-aligned region and a horizontal
    /// surface. Every awake dynamic body whose origin lies in the region
    /// feels buoyancy on the part of each shape below the surface (exact
    /// for circles and polygons, a box fraction for capsules and
    /// segments), linear and angular drag, and the flow. Far cheaper than a
    /// particle pool where one would be overkill.
    typedef struct m2FluidVolumeDef
    {
        m2Pos2 regionLower; // the region, world space
        m2Pos2 regionUpper;
        M2_ALIGN8 double surface; // the waterline, world y
        float density;            // buoyant force per submerged area; 0 to M2_MAX_DENSITY
        float linearDrag;         // velocity damping per submerged area; 0 to M2_MAX_RATE
        float angularDrag;        // spin damping; 0 to M2_MAX_RATE
        m2Vec2 flow;              // the current, m/s, within M2_MAX_SPEED per axis
        M2_ALIGN8 uint64_t userData;
        int32_t internalValue; // the cookie m2DefaultFluidVolumeDef sets
    } m2FluidVolumeDef;

    /// Returns a fluid volume def: density 2, linear drag 1, angular drag
    /// 0.5, no flow, an empty region at the origin.
    ///
    /// @return The default def; set the region and the surface.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2FluidVolumeDef m2DefaultFluidVolumeDef(void);

    /// Creates a fluid volume. Journaled.
    ///
    /// @param world      The world.
    /// @param def        The volume, from m2DefaultFluidVolumeDef.
    /// @param volumeOut  Receives the volume; the null id on failure.
    /// @return m2_success; m2_errorInvalid for a bad def, a null argument
    ///         or a world without volumes; m2_errorCapacity when the
    ///         volume pool is full; m2_errorState from host code the world
    ///         is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateFluidVolume(m2World* world, const m2FluidVolumeDef* def,
                                                     m2FluidVolumeId* volumeOut);

    /// Destroys a fluid volume. Journaled.
    ///
    /// @param world     The world.
    /// @param volumeId  The volume.
    /// @return m2_success; m2_errorStale when it is already gone;
    ///         m2_errorState from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2DestroyFluidVolume(m2World* world, m2FluidVolumeId volumeId);

    /// Tells whether a volume id names a live volume. Counts no misuse.
    ///
    /// @param world     The world.
    /// @param volumeId  The volume.
    /// @return True for a live volume.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2FluidVolume_IsValid(const m2World* world, m2FluidVolumeId volumeId);

    /// Moves a volume's waterline: a rising tide, a draining tank. A stale
    /// id or a non-finite surface counts one misuse. Journaled.
    ///
    /// @param world     The world.
    /// @param volumeId  The volume.
    /// @param surface   The new waterline, world y.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2FluidVolume_SetSurface(m2World* world, m2FluidVolumeId volumeId, double surface);

    /// Reads a volume's waterline.
    ///
    /// @param world     The world.
    /// @param volumeId  The volume; a stale id counts one misuse.
    /// @return The waterline; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API double m2FluidVolume_GetSurface(const m2World* world, m2FluidVolumeId volumeId);

    /// Reads a volume's game data.
    ///
    /// @param world     The world.
    /// @param volumeId  The volume; a stale id counts one misuse.
    /// @return The value; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2FluidVolume_GetUserData(const m2World* world, m2FluidVolumeId volumeId);

    /// Returns a journal tape's fixed cost: its header and the snapshot it
    /// embeds. Size a tape as this plus room for the operations.
    ///
    /// @param world  The world.
    /// @return The size in bytes; 0 for a null world.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetJournalBaseSize(const m2World* world);

    /// Starts recording a journal into caller memory: a snapshot first,
    /// then every call that changes the world, the steps and the restores
    /// included, with floats as their exact bits. The buffer must stay
    /// valid until m2World_StopJournal.
    ///
    /// @param world     The world.
    /// @param buffer    Receives the tape.
    /// @param capacity  The length of buffer, in bytes.
    /// @return m2_success; m2_errorInvalid for a null buffer;
    ///         m2_errorCapacity when capacity is below
    ///         m2World_GetJournalBaseSize; m2_errorState when the world is
    ///         already recording or the call comes from host code the
    ///         world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2World_StartJournal(m2World* world, void* buffer,
                                                      int32_t capacity);

    /// Stops recording. Operations that did not fit make the tape unusable:
    /// it is never cut short silently.
    ///
    /// @param world     The world.
    /// @param bytesOut  Receives the tape's size; 0 on failure.
    /// @return m2_success; m2_errorInvalid for a null bytesOut;
    ///         m2_errorCapacity when the operations overflowed the buffer;
    ///         m2_errorState when the world was not recording or the call
    ///         comes from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2World_StopJournal(m2World* world, int32_t* bytesOut);

    /// Replays a tape: restores the snapshot it embeds and applies its
    /// operations through the same calls that recorded them. It is atomic:
    /// a tape that is malformed or truncated, or that recreates an object
    /// under another id than the recording saw, is refused and the world
    /// stays as it was. Only the release that wrote a tape replays it
    /// (M2_ABI_VERSION); a tape from another release, build or world shape
    /// is refused too.
    ///
    /// @param world  The world.
    /// @param data   The tape.
    /// @param size   Its length, in bytes.
    /// @return m2_success; m2_errorVersion for a tape, or its embedded
    ///         snapshot, another release or build wrote, and m2_errorInvalid
    ///         for another tape that fails its checks, neither of which
    ///         counts misuse; m2_errorInvalid for a null data pointer, which
    ///         counts one; m2_errorState while the world is recording
    ///         or from host code the world is running; m2_errorCapacity
    ///         when the allocator cannot give the replay its scratch memory.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2World_ReplayJournal(m2World* world, const void* data,
                                                       int32_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_WORLD_H
