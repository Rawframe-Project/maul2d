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
/// anything else - including zero-initialized defs. The high byte is
/// always nonzero, so no legal cookie can collide with small garbage.
#define M2_COOKIE 0x6D32C0DE

    /// A world: the bodies, shapes, joints, particles and volumes it owns,
    /// and everything a step needs. Worlds share nothing, so separate worlds
    /// may be created, stepped and destroyed on separate threads.
    typedef struct m2World m2World;

    /// A range job: run items [startIndex, endIndex).
    typedef void m2TaskFn(int32_t startIndex, int32_t endIndex, void* taskContext);
    /// Split [0, itemCount) into subranges of at least minRange and
    /// run task over them on the host's scheduler; the return value
    /// is handed back to finishTask, which blocks until all done.
    typedef void* m2EnqueueTaskFn(m2TaskFn* task, int32_t itemCount, int32_t minRange,
                                  void* taskContext, void* userContext);
    typedef void m2FinishTaskFn(void* userTask, void* userContext);

    typedef struct m2WorldDef
    {
        // The cookie and the ABI version stay the first two fields in every
        // version, so that m2CreateWorld can refuse a def of another layout
        // before it reads anything else.
        int32_t internalValue;
        // M2_ABI_VERSION of the headers the program was built with.
        uint32_t version;
        m2Vec2 gravity;       // meters/s^2, applied to dynamic bodies
        int32_t bodyCapacity; // fixed capacities for now (no growth yet)
        int32_t shapeCapacity;
        int32_t jointCapacity;
        bool enableSleeping; // islands may sleep (default true)
        /// Fluids: particleCapacity > 0 builds the particle system
        /// into the world for its whole lifetime (constant snapshot
        /// shape keeps rollback safe across every point in history).
        /// Parameters are pinned here like every other physics knob.
        int32_t particleCapacity;    // 0 = no fluids
        int32_t fluidVolumeCapacity; // 0 = no buoyancy volumes
        float particleRadius;        // meters, floor 4x linear slop
        float particleDensity;       // mass per area
        float particleGravityScale;
        /// The particle model's strengths (see particle_solver.c): the
        /// pressure above rest density, the damping of approach
        /// speed, shear viscosity for viscous-flagged particles, the
        /// spacing push of powder grains, the pull of spring nets and
        /// elastic triads, and for tensile particles the cohesion below
        /// rest density and the near pressure that keeps a droplet
        /// from collapsing.
        float particlePressureStrength;
        float particleDampingStrength;
        float particleViscousStrength;
        float particlePowderStrength;
        float particleSpringStrength;
        float particleElasticStrength;
        float particleCohesionStrength;
        float particleNearPressureStrength;
        /// The host task executor, the same
        /// contract Maul3D carries: the engine calls
        /// enqueueTask with a range job, the host runs it on its own
        /// scheduler (splitting [0, itemCount) into subranges of at
        /// least minRange), and finishTask must not return before
        /// every subrange completed. Both NULL (the default) means
        /// serial in the caller's thread. NON-SEMANTIC by law: any
        /// executor and any split produce identical bits.
        m2EnqueueTaskFn* enqueueTask;
        m2FinishTaskFn* finishTask;
        void* userTaskContext;
        /// The allocator the world takes all its memory from, kept for its
        /// lifetime. Zeroed, it is the C library's. Set both functions or
        /// neither.
        m2Allocator allocator;
    } m2WorldDef;

    /// THREAD CLASSES. Every call is tagged reader or writer. Readers
    /// (queries, getters, diagnostics) take a const world and may run
    /// concurrently with each other, but never during m2World_Step or any
    /// writer. Writers (create and destroy, setters, impulses, Step itself)
    /// need exclusive access to their world. Distinct worlds share nothing,
    /// so each may be used on its own thread. The host code a world calls
    /// (the task hooks, the debug draw callbacks, the allocator) must not
    /// call back into that world: its writers refuse with m2_errorState,
    /// or count one misuse where they return void.
    ///
    /// IDS. An id names an object of the world that issued it: a 1-based
    /// slot and a generation. A correct program never uses an id after
    /// destroying its object, so a stale id counts one misuse: a getter
    /// returns the zeroed value, a setter changes nothing, and a call that
    /// returns a status returns m2_errorStale. Particles are the exception:
    /// the world ends a particle on its own when its lifetime runs out, so
    /// a program may hold a dead particle id, and a stale particle id counts
    /// nothing. The validity queries (m2Body_IsValid and the others) count
    /// nothing either. An id carries no
    /// world: one passed to another world names whatever that world holds
    /// in the same slot and generation, and the library cannot tell.
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

    /// The library's default world def, with the version left zero. Use
    /// m2DefaultWorldDef, which stamps the version of the program's
    /// headers. Thread class: reader (pure).
    M2_API m2WorldDef m2DefaultWorldDefValues(void);

    /// Returns a def with the library's defaults, a valid cookie and the
    /// ABI version of the headers the program is built with. It is built
    /// in the program, so m2CreateWorld can refuse a def made for another
    /// release's binary interface. Thread class: reader (pure).
    static inline m2WorldDef m2DefaultWorldDef(void)
    {
        m2WorldDef def = m2DefaultWorldDefValues();
        def.version = M2_ABI_VERSION;
        return def;
    }

    /// Creates a world. Checks, in order: that this CPU runs the SIMD
    /// backend (m2_errorUnsupported), the def's cookie (m2_errorInvalid),
    /// its version (m2_errorVersion), then the rest of the def
    /// (m2_errorInvalid: a nonpositive capacity, an allocator with one
    /// function set, a fluid parameter out of range). All the world's
    /// memory comes from the def's allocator here; m2_errorCapacity when it
    /// refuses or a size does not fit. On any failure *worldOut is null.
    /// Thread class: writer.
    M2_NODISCARD M2_API m2Result m2CreateWorld(const m2WorldDef* def, m2World** worldOut);

    /// Destroys a world and everything in it, and returns its memory to its
    /// allocator. Ids into it become meaningless. Null is a no-op.
    /// Thread class: writer.
    M2_API void m2DestroyWorld(m2World* world);

    /// Walk the whole world and check its invariants: finiteness of
    /// every live transform, velocity and particle, endpoint liveness
    /// of joints and jelly nets, registry count consistency, pair-key
    /// ordering. Returns false on the first violation and never stops the
    /// program; true means the world is sound. Pure reader; costs a full
    /// walk, so call it from debug paths. Building with
    /// -DMAUL2D_VALIDATE=ON asserts it after every step.
    M2_API bool m2World_Validate(const m2World* world);

    /// Changing gravity wakes every sleeping dynamic body: a stack
    /// must not float against a world that turned upside down. The
    /// change is journaled. Thread class: writer / reader.
    M2_API void m2World_SetGravity(m2World* world, m2Vec2 gravity);
    M2_API m2Vec2 m2World_GetGravity(const m2World* world);

    /// Global wind: an ambient air velocity and a linear drag coefficient
    /// (>= 0). Each step every dynamic awake body feels a force equal to
    /// -linearDrag * shapeArea * (bodyVelocity - windVelocity), into the
    /// same accumulators as gravity, so bigger bodies catch more wind and
    /// everything drifts toward the wind. linearDrag 0 (the default) is
    /// off and costs nothing. Unlike gravity this does NOT wake sleepers:
    /// wind is meant to gust, and waking every sleeper each change would
    /// defeat sleeping. Regional wind is a density-0 fluid volume; this is
    /// the global case. Journaled and snapshot state. Thread class:
    /// writer / reader. Any out pointer may be NULL.
    M2_API void m2World_SetWind(m2World* world, m2Vec2 velocity, float linearDrag);
    M2_API void m2World_GetWind(const m2World* world, m2Vec2* velocityOut, float* linearDragOut);

    /// World-wide sleep master switch, journaled. Disabling wakes
    /// every sleeping dynamic body (a sleeper must not outlive the
    /// rule that let it sleep). Thread class: writer / reader.
    /// One deterministic blast: every dynamic shape whose category
    /// passes maskBits and whose closest point lies inside
    /// radius+falloff gets an outward impulse at that point, full
    /// strength inside radius, fading linearly to zero across the
    /// falloff band. One journal op replays the whole thing.
    typedef struct m2ExplosionDef
    {
        m2Pos2 position;
        float radius;
        float falloff;
        float impulse; // newton seconds at full strength
        uint64_t maskBits;
        int32_t internalValue;
    } m2ExplosionDef;

    M2_API m2ExplosionDef m2DefaultExplosionDef(void);
    M2_API void m2World_Explode(m2World* world, const m2ExplosionDef* def);

    M2_API void m2World_EnableSleeping(m2World* world, bool flag);
    M2_API bool m2World_IsSleepingEnabled(const m2World* world);

    /// Debug drawing: the engine walks its state and calls back; you
    /// render. Polygon vertices arrive BODY-LOCAL with the body's f64
    /// origin and rotation so your renderer can compose camera-relative
    /// doubles and stay steady far from the origin. Leave any callback
    /// NULL to skip that primitive. Colors are 0xRRGGBB hints.
    /// Read-only. Thread class: reader.
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
        /// Length in world units per unit of impulse for the force arrows;
        /// <= 0 draws them at the raw impulse magnitude (scale 1).
        float forceScale;
        void* context;
    } m2DebugDraw;

    M2_API void m2World_Draw(const m2World* world, const m2DebugDraw* draw);

    /// Diagnostics. Counters are derived from simulation state and are
    /// therefore deterministic; profile times are wall-clock and are
    /// NOT - neither is ever snapshot state or hash input.
    typedef struct m2Profile
    {
        float stepMs;     // whole m2World_Step
        float pairsMs;    // broadphase + pair update
        float contactsMs; // narrowphase manifolds
        float solveMs;    // solver incl. CCD
        float sleepMs;    // islands + sleep accounting
    } m2Profile;

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
        int32_t overflowConstraints; // serial bucket last step
        uint64_t stepCount;
        // Diagnostics, counted in every build. Poll them once per
        // frame and alarm on growth; there is no per-call callback,
        // so healthy calls pay nothing for the diagnosis.
        int32_t pairOverflow;         // body pairs dropped last step (pair table full)
        int32_t particlePairOverflow; // pairs dropped last step (budget)
        int32_t particleBodyOverflow; // body contacts dropped last step
        uint64_t particlePoolFull;    // emits refused by a full pool, cumulative
        uint64_t misuse;              // stale-id or wrong-type API rejections, cumulative
    } m2Counters;

    /// Snapshot of the last completed Step. Thread class: reader.
    /// Total kinetic energy of awake dynamic bodies, in joules.
    /// State-derived and deterministic (f64 accumulation in canonical
    /// body order) - twin worlds report identical bits. Sleeping
    /// bodies contribute zero by construction. Thread class: reader.
    M2_API double m2World_GetKineticEnergy(const m2World* world);

    M2_API m2Profile m2World_GetProfile(const m2World* world);
    M2_API m2Counters m2World_GetCounters(const m2World* world);

    /// Advance the simulation. dt in seconds, substepCount >= 1.
    /// Deterministic: identical worlds and inputs produce bit-identical
    /// state on every supported platform. Thread class: writer.
    M2_API void m2World_Step(m2World* world, float dt, int32_t substepCount);

    /// Steps taken since creation (restored by m2World_Restore).
    /// Thread class: reader.
    M2_API uint64_t m2World_GetStepCount(const m2World* world);

    /// Snapshot size in bytes for this world. Thread class: reader.
    M2_API int32_t m2World_SnapshotSize(const m2World* world);

    /// Writes a full snapshot into caller memory. *bytesOut is the number
    /// of bytes written; with m2_errorCapacity, when capacity is smaller
    /// than m2World_SnapshotSize, it is the size needed and nothing is
    /// written. On any other failure it is 0. Thread class: reader.
    M2_NODISCARD M2_API m2Result m2World_Snapshot(const m2World* world, void* buffer,
                                                  int32_t capacity, int32_t* bytesOut);

    /// Restore a snapshot taken from a world with the same def shape. All
    /// sim state - bodies, id pools, step counter - returns to the
    /// snapshot instant, bit-exactly. Before any byte lands, the buffer
    /// is checked: a header from another build or world shape,
    /// out-of-range indices and counts, flags that are not flags, and
    /// non-finite state refuse with m2_errorInvalid, and the world is left
    /// as it was. The checks make every index safe to use; they do not
    /// re-derive every structural invariant (tree shape, list links), so
    /// restore only snapshots this engine wrote. Refused bytes count no
    /// misuse; a null buffer does. Thread class: writer.
    M2_NODISCARD M2_API m2Result m2World_Restore(m2World* world, const void* buffer, int32_t size);

    /// Deterministic state hash (alive bodies in index order + globals).
    /// Present in all builds: this is the desync-forensics primitive.
    /// Thread class: reader.
    M2_API uint64_t m2World_Hash(const m2World* world);

    /// Per-world memory footprint. persistentBytes is everything create
    /// allocated for this world, including the world struct itself; it
    /// is fixed for the world's lifetime (pools never grow, step
    /// scratch lives in them, and the journal buffer is the host's).
    /// Thread class: reader.
    typedef struct m2MemoryUsage
    {
        int64_t persistentBytes;
    } m2MemoryUsage;
    M2_API m2MemoryUsage m2World_GetMemoryUsage(const m2World* world);

    /// Per-subsystem hashes for hunting a divergence: run your twin
    /// simulations, compare parts each step, and the first field
    /// that splits names the subsystem while the step count names
    /// the moment. Diagnostic values: they are not portable across
    /// engine versions and the total hash is not a function of the
    /// parts. Thread class: reader.
    typedef struct m2WorldHashParts
    {
        uint64_t world;     // step counter and gravity
        uint64_t bodies;    // transforms, velocities, mass and sleep state
        uint64_t contacts;  // pair keys and manifolds
        uint64_t joints;    // constraint accumulator memory
        uint64_t particles; // fluid state including the jelly nets
    } m2WorldHashParts;
    M2_API m2WorldHashParts m2World_GetHashParts(const m2World* world);

    typedef struct m2FluidVolumeId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2FluidVolumeId;

    static const m2FluidVolumeId m2_nullFluidVolumeId = {0, 0};

    /// A particle-free body of water: an axis-aligned activation
    /// region and a horizontal surface line. Every awake dynamic body
    /// whose bounds touch the region feels buoyancy on the part of
    /// each shape below the surface (Archimedes, exact for circles and
    /// polygons, a bounding-box approximation for capsules and
    /// segments), plus linear and angular drag, plus an optional flow
    /// current. Cheap where a full particle pool would be overkill.
    typedef struct m2FluidVolumeDef
    {
        m2Pos2 regionLower; // activation box, world space
        m2Pos2 regionUpper;
        double surface;    // waterline, world y
        float density;     // fluid density (buoyant force per submerged area)
        float linearDrag;  // velocity damping inside, per submerged area
        float angularDrag; // spin damping inside
        m2Vec2 flow;       // current the water drags bodies toward, m/s
        uint64_t userData;
        int32_t internalValue;
    } m2FluidVolumeDef;

    M2_API m2FluidVolumeDef m2DefaultFluidVolumeDef(void);
    /// Creates a fluid volume: m2_errorInvalid for a bad def,
    /// m2_errorCapacity when the world's volume pool is full. On failure
    /// *volumeOut is the null id. Thread class: writer. Journaled.
    M2_NODISCARD M2_API m2Result m2CreateFluidVolume(m2World* world, const m2FluidVolumeDef* def,
                                                     m2FluidVolumeId* volumeOut);
    /// Destroys a fluid volume; m2_errorStale when it is already gone.
    /// Thread class: writer. Journaled.
    M2_NODISCARD M2_API m2Result m2DestroyFluidVolume(m2World* world, m2FluidVolumeId volumeId);
    M2_API bool m2FluidVolume_IsValid(const m2World* world, m2FluidVolumeId volumeId);
    /// Move the waterline at runtime (a rising tide, a draining tank).
    /// Journaled. Thread class: writer.
    M2_API void m2FluidVolume_SetSurface(m2World* world, m2FluidVolumeId volumeId, double surface);
    M2_API double m2FluidVolume_GetSurface(const m2World* world, m2FluidVolumeId volumeId);
    M2_API uint64_t m2FluidVolume_GetUserData(const m2World* world, m2FluidVolumeId volumeId);

    /// Command journal (the replay primitive). StartJournal embeds a
    /// full snapshot into the caller's buffer, then records every
    /// mutating call and step marker with raw IEEE-754 bit encoding.
    /// StopJournal ends the recording and reports the tape's size.
    /// ReplayJournal restores the embedded snapshot and re-applies the
    /// stream. It is atomic: a tape that is malformed, truncated or
    /// recreates an object under a different id than the recording saw is
    /// refused and the world is left as it was. A restore during recording
    /// is recorded too.
    /// The journal's fixed cost: header plus the embedded snapshot.
    /// Size tapes as this plus room for your ops. Thread class: reader.
    M2_API int32_t m2World_GetJournalBaseSize(const m2World* world);

    /// Starts recording into buffer: m2_errorCapacity when capacity cannot
    /// hold m2World_GetJournalBaseSize, m2_errorState when the world is
    /// already recording. Thread class: writer.
    M2_NODISCARD M2_API m2Result m2World_StartJournal(m2World* world, void* buffer,
                                                      int32_t capacity);

    /// Stops recording. *bytesOut is the tape's size; m2_errorCapacity when
    /// the ops overflowed the buffer (the tape is unusable, never cut
    /// short) and m2_errorState when the world was not recording, both
    /// with *bytesOut 0. Thread class: writer.
    M2_NODISCARD M2_API m2Result m2World_StopJournal(m2World* world, int32_t* bytesOut);

    /// Replays a tape: m2_errorInvalid for a tape that fails its checks,
    /// which counts no misuse, m2_errorState while the world is recording,
    /// m2_errorCapacity when the allocator cannot give replay its scratch
    /// memory. On failure the world is as it was. Thread class: writer.
    M2_NODISCARD M2_API m2Result m2World_ReplayJournal(m2World* world, const void* data,
                                                       int32_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_WORLD_H
