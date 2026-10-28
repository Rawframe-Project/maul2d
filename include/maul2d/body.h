// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Rigid bodies: creation and destruction, motion, forces and
// impulses, mass, sleep, and state readback.

#ifndef MAUL2D_BODY_H
#define MAUL2D_BODY_H

#include "maul2d/world.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// A box in world space, its corners 64-bit positions: what a body or
    /// shape covers, and the region a box query looks in.
    typedef struct m2WorldAabb
    {
        m2Pos2 lowerBound;
        m2Pos2 upperBound;
    } m2WorldAabb;

    typedef enum m2BodyType
    {
        m2_staticBody = 0,
        m2_kinematicBody = 1,
        m2_dynamicBody = 2,
    } m2BodyType;

    /// A body of a world: a 1-based slot and a generation.
    typedef struct m2BodyId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2BodyId;

    /// Per-axis motion locks. A locked axis holds still: its velocity is
    /// zeroed each substep, so contacts, joints, gravity and impulses
    /// cannot move the body along it. linearX and linearY lock world-frame
    /// translation; angularZ locks rotation, which also makes the
    /// rotational inertia infinite. All default false.
    typedef struct m2MotionLocks
    {
        bool linearX;
        bool linearY;
        bool angularZ;
    } m2MotionLocks;

    /// The def of a body. Initialize it with m2DefaultBodyDef.
    typedef struct m2BodyDef
    {
        m2BodyType type;
        m2Pos2 position; // world space, f64
        m2Rot rotation;
        m2Vec2 linearVelocity;
        float angularVelocity; // radians/s
        float gravityScale;
        float linearDamping; // v / (1 + h d) per substep, implicit
        float angularDamping;
        m2MotionLocks motionLocks;   // angularZ: never rotates, infinite inertia
        bool enableSleep;            // false: this body never sleeps
        bool isEnabled;              // false: created dormant, outside the simulation
        int8_t dominance;            // the higher side of a contact cannot be pushed
        bool isBullet;               // continuous collision against non-bullets
        M2_ALIGN8 uint64_t userData; // opaque, kept through snapshots and journals
        int32_t internalValue;       // the cookie m2DefaultBodyDef sets
    } m2BodyDef;

    /// Returns a body def: a static, enabled body at the origin with
    /// identity rotation, gravity scale 1, sleep allowed, everything else
    /// zero.
    ///
    /// @return The default def.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2BodyDef m2DefaultBodyDef(void);

    /// Creates a body. Journaled.
    ///
    /// @param world    The world.
    /// @param def      The body, from m2DefaultBodyDef. Its type must be
    ///                 one of m2BodyType; its position and rotation
    ///                 finite; its velocities within M2_SPEED_LIMIT, and
    ///                 zero for a static body; its rotation not zero, and
    ///                 normalized to its direction; its gravity scale
    ///                 within M2_GRAVITY_SCALE_LIMIT; its dampings from 0
    ///                 to M2_RATE_LIMIT.
    /// @param bodyOut  Receives the body; the null id on failure.
    /// @return m2_success; m2_errorInvalid for a null argument, a def not
    ///         from m2DefaultBodyDef, a NaN or a field outside its domain,
    ///         and m2_errorLimit for a field past its published bound, each
    ///         of which counts one misuse; m2_errorCapacity when the body
    ///         pool is full; m2_errorBusy from host code the world is
    ///         running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateBody(m2World* world, const m2BodyDef* def,
                                              m2BodyId* bodyOut);

    /// Destroys a body with its shapes, chains and joints; their contacts
    /// end and touching sleepers wake. The slot is reused in FIFO order
    /// under a new generation, and retires rather than let the generation
    /// wrap. Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @return m2_success; m2_errorStale when the body is already gone;
    ///         m2_errorBusy from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2DestroyBody(m2World* world, m2BodyId bodyId);

    /// Tells whether a body id names a live body. Counts no misuse.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @return True for a live body.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Body_IsValid(const m2World* world, m2BodyId bodyId);

    /// Reads a body's pose.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The pose; the identity at the origin for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Transform m2Body_GetTransform(const m2World* world, m2BodyId bodyId);

    /// Reads a body's origin in world space.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The position; the origin for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2Body_GetPosition(const m2World* world, m2BodyId bodyId);

    /// Reads a body's rotation.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The rotation; the identity for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Rot m2Body_GetRotation(const m2World* world, m2BodyId bodyId);

    /// Reads the velocity of a body's center of mass.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The velocity, m/s; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetLinearVelocity(const m2World* world, m2BodyId bodyId);

    /// Reads a body's angular velocity.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The angular velocity, radians/s; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetAngularVelocity(const m2World* world, m2BodyId bodyId);

    /// Reads a body's game data.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The value; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2Body_GetUserData(const m2World* world, m2BodyId bodyId);

    /// Tells whether a body is awake. Setters and new contacts wake a body;
    /// waking spreads through its island at the next step.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return True when awake, and always for a static or kinematic body;
    ///         false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Body_IsAwake(const m2World* world, m2BodyId bodyId);

    /// Wakes a body, or puts it to sleep at once with its velocities
    /// zeroed. Only an enabled dynamic body sleeps; for any other body the
    /// call changes nothing. A stale id counts one misuse. Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param awake   True to wake, false to sleep.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetAwake(m2World* world, m2BodyId bodyId, bool awake);

    /// Makes a body a bullet, which sweeps continuously against the shapes
    /// of non-bullets, or a plain body again. A stale id counts one misuse.
    /// Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param flag    True for a bullet.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetBullet(m2World* world, m2BodyId bodyId, bool flag);

    /// Sets a body's opaque game data, kept through snapshots and journals.
    /// A stale id counts one misuse. Journaled.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param userData  Any value.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetUserData(m2World* world, m2BodyId bodyId, uint64_t userData);

    /// Sets a body's contact dominance. In a pair, the higher side acts as
    /// unmovable toward the lower one, and statics outrank everything.
    /// Joints are unaffected. A stale id counts one misuse. Journaled.
    ///
    /// @param world      The world.
    /// @param bodyId     The body.
    /// @param dominance  The dominance; 0 is the default.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetDominance(m2World* world, m2BodyId bodyId, int8_t dominance);

    /// Reads a body's contact dominance.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The dominance; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int8_t m2Body_GetDominance(const m2World* world, m2BodyId bodyId);

    /// Sets the velocities that carry a body to a target pose in one step
    /// of the given length: the way to drive a kinematic body. It goes
    /// through the journaled velocity setters, so a replay repeats it. A
    /// stale id, a static body, an invalid argument or a pose too far to
    /// reach within M2_SPEED_LIMIT changes nothing and counts one misuse.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param position  The target origin.
    /// @param rotation  The target rotation, unit length.
    /// @param dt        The step the move takes, from M2_MIN_STEP to
    ///                  M2_STEP_LIMIT.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetTargetTransform(m2World* world, m2BodyId bodyId, m2Pos2 position,
                                          m2Rot rotation, float dt);

    /// Reads a body's type.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The type; m2_staticBody for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2BodyType m2Body_GetType(const m2World* world, m2BodyId bodyId);

    /// A body's mass properties.
    typedef struct m2MassData
    {
        float mass;           // kg
        m2Vec2 center;        // the body-local centroid
        float centralInertia; // rotational inertia about the center of mass
    } m2MassData;

    /// Overrides the mass a dynamic body's shapes give it. The override
    /// lasts until a shape change, a fixed-rotation change or
    /// m2Body_ApplyMassFromShapes recomputes the mass. The body wakes. A
    /// stale id, a body that is not dynamic, a mass outside M2_MIN_MASS to
    /// M2_MASS_LIMIT, a center beyond M2_LOCAL_LIMIT, or a central inertia that is
    /// neither 0 nor within M2_MIN_INERTIA to M2_INERTIA_LIMIT, changes
    /// nothing and counts one misuse. Every override this accepts survives
    /// a snapshot. A central inertia of 0 leaves the body unable to turn.
    /// Mass computed from shapes is clamped into the same ranges.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param massData  The mass, centroid and inertia about the centroid.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetMassData(m2World* world, m2BodyId bodyId, m2MassData massData);

    /// Reads a body's mass properties.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The mass, centroid and inertia about the centroid; zeroed
    ///         for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2MassData m2Body_GetMassData(const m2World* world, m2BodyId bodyId);

    /// Recomputes a body's mass from its shapes, dropping an override. A
    /// dynamic body wakes. A stale id counts one misuse. Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyMassFromShapes(m2World* world, m2BodyId bodyId);

    /// Takes a body out of the simulation without destroying it, or puts
    /// it back where it was. A disabled body's shapes leave the broadphase
    /// (contacts end, riders wake), its joints stay attached but inert, and
    /// queries stop seeing it. Setting the state a body already has changes
    /// nothing and records nothing. A stale id counts one misuse.
    /// Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param flag    True to enable it, false to disable it.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetEnabled(m2World* world, m2BodyId bodyId, bool flag);

    /// Tells whether a body takes part in the simulation.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return True when enabled; false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Body_IsEnabled(const m2World* world, m2BodyId bodyId);

    /// Reads a body's center of mass in its own frame.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The body-local center; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetLocalCenter(const m2World* world, m2BodyId bodyId);

    /// Tells whether a body is a bullet.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return True for a bullet; false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Body_IsBullet(const m2World* world, m2BodyId bodyId);

    /// Reads a body's gravity scale.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The scale; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetGravityScale(const m2World* world, m2BodyId bodyId);

    /// Lists the world's bodies in ascending slot order.
    ///
    /// @param world     The world.
    /// @param ids       Receives up to capacity bodies; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of bodies, which may exceed capacity.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetBodies(const m2World* world, m2BodyId* ids, int32_t capacity);

    /// Maps a point from a body's frame to world space.
    ///
    /// @param world       The world.
    /// @param bodyId      The body; a stale id counts one misuse.
    /// @param localPoint  The point in the body's frame.
    /// @return The world point; the origin for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2Body_GetWorldPoint(const m2World* world, m2BodyId bodyId, m2Vec2 localPoint);

    /// Maps a world point to a body's frame.
    ///
    /// @param world       The world.
    /// @param bodyId      The body; a stale id counts one misuse.
    /// @param worldPoint  The world point.
    /// @return The point in the body's frame; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetLocalPoint(const m2World* world, m2BodyId bodyId, m2Pos2 worldPoint);

    /// Rotates a vector from a body's frame to the world frame.
    ///
    /// @param world        The world.
    /// @param bodyId       The body; a stale id counts one misuse.
    /// @param localVector  The vector in the body's frame.
    /// @return The world vector; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetWorldVector(const m2World* world, m2BodyId bodyId, m2Vec2 localVector);

    /// Rotates a world vector into a body's frame.
    ///
    /// @param world        The world.
    /// @param bodyId       The body; a stale id counts one misuse.
    /// @param worldVector  The world vector.
    /// @return The vector in the body's frame; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetLocalVector(const m2World* world, m2BodyId bodyId, m2Vec2 worldVector);

    /// Reads the world velocity of the body point at a world position.
    ///
    /// @param world       The world.
    /// @param bodyId      The body; a stale id counts one misuse.
    /// @param worldPoint  The point, in world space.
    /// @return Its velocity; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetWorldPointVelocity(const m2World* world, m2BodyId bodyId,
                                               m2Pos2 worldPoint);

    /// Reads the world velocity of the body point at a body-local position.
    ///
    /// @param world       The world.
    /// @param bodyId      The body; a stale id counts one misuse.
    /// @param localPoint  The point, in the body's frame.
    /// @return Its velocity; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Body_GetLocalPointVelocity(const m2World* world, m2BodyId bodyId,
                                               m2Vec2 localPoint);

    /// Reads a body's center of mass in world space.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The world center; the origin for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2Body_GetWorldCenterOfMass(const m2World* world, m2BodyId bodyId);

    /// Reads a body's mass: m2MassData.mass.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The mass, kg; 0 for a body that is not dynamic or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetMass(const m2World* world, m2BodyId bodyId);

    /// Reads a body's rotational inertia about its center of mass:
    /// m2MassData.centralInertia.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The inertia, kg m^2; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetCentralInertia(const m2World* world, m2BodyId bodyId);

    /// Computes the tight box around every shape of a body, without the
    /// broadphase margins.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The box; a point at the body origin for a body without
    ///         shapes; zeroed for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2WorldAabb m2Body_ComputeAabb(const m2World* world, m2BodyId bodyId);

    /// Sets the velocity of a body's center of mass and wakes it. A stale
    /// id, a static body or a velocity beyond M2_SPEED_LIMIT on either axis
    /// changes nothing and counts one misuse: a static body never moves.
    /// Journaled.
    ///
    /// @param world     The world.
    /// @param bodyId    The kinematic or dynamic body.
    /// @param velocity  The velocity, m/s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetLinearVelocity(m2World* world, m2BodyId bodyId, m2Vec2 velocity);

    /// Sets a body's angular velocity and wakes it, as
    /// m2Body_SetLinearVelocity does.
    ///
    /// @param world     The world.
    /// @param bodyId    The kinematic or dynamic body.
    /// @param velocity  The angular velocity, radians/s, within M2_SPEED_LIMIT.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetAngularVelocity(m2World* world, m2BodyId bodyId, float velocity);

    /// Teleports a body: position and rotation snap, velocities stay, the
    /// broadphase refreshes at once, and the body and everything it touched
    /// wake. A teleport is not a sweep: nothing keeps it from tunneling. A
    /// stale id, a non-finite position or a rotation that is not unit
    /// length changes nothing and counts one misuse. Journaled.
    ///
    /// @param world     The world.
    /// @param bodyId    The body.
    /// @param position  The new origin.
    /// @param rotation  The new rotation.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetTransform(m2World* world, m2BodyId bodyId, m2Pos2 position,
                                    m2Rot rotation);

    /// Changes a body's type in place. Becoming static zeroes the
    /// velocities; becoming dynamic recomputes the mass from the shapes. The
    /// body and everything it touches wake, its shapes move to the tree of
    /// the new type, and pairs that no longer apply end with end events. A
    /// stale id or an unknown type counts one misuse. Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param type    The new type.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetType(m2World* world, m2BodyId bodyId, m2BodyType type);

    /// Applies an impulse at a world point, changing the velocity at once;
    /// the lever arm is measured from the center of mass, each axis clamped
    /// to M2_REACH_LIMIT. The body wakes, and its velocity saturates at
    /// M2_SPEED_LIMIT per axis. A stale id, a body that is not dynamic, a
    /// non-finite point or an impulse beyond M2_IMPULSE_LIMIT on either
    /// axis changes nothing and counts one misuse. Journaled.
    ///
    /// @param world       The world.
    /// @param bodyId      The dynamic body.
    /// @param impulse     The impulse, N s.
    /// @param worldPoint  Where it acts.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyLinearImpulseAtPoint(m2World* world, m2BodyId bodyId, m2Vec2 impulse,
                                                 m2Pos2 worldPoint);

    /// Applies an impulse at the center of mass, as
    /// m2Body_ApplyLinearImpulseAtPoint does.
    ///
    /// @param world    The world.
    /// @param bodyId   The dynamic body.
    /// @param impulse  The impulse, N s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyLinearImpulse(m2World* world, m2BodyId bodyId, m2Vec2 impulse);

    /// Adds a force at a world point for the next step. Forces accumulate
    /// across calls, saturating at M2_FORCE_LIMIT per axis (and torque), act
    /// during the step and clear when it ends. The lever arm is clamped as
    /// m2Body_ApplyLinearImpulseAtPoint clamps it. The body wakes. A stale
    /// id, a body that is not dynamic, a non-finite point or a force beyond
    /// M2_FORCE_LIMIT on either axis changes nothing and counts one misuse.
    /// Journaled.
    ///
    /// @param world       The world.
    /// @param bodyId      The dynamic body.
    /// @param force       The force, N.
    /// @param worldPoint  Where it acts.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyForceAtPoint(m2World* world, m2BodyId bodyId, m2Vec2 force,
                                         m2Pos2 worldPoint);

    /// Adds a force at the center of mass for the next step, as
    /// m2Body_ApplyForceAtPoint does.
    ///
    /// @param world   The world.
    /// @param bodyId  The dynamic body.
    /// @param force   The force, N.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyForce(m2World* world, m2BodyId bodyId, m2Vec2 force);

    /// Adds a torque for the next step, as m2Body_ApplyForceAtPoint does.
    ///
    /// @param world   The world.
    /// @param bodyId  The dynamic body.
    /// @param torque  The torque, N m, within M2_FORCE_LIMIT.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyTorque(m2World* world, m2BodyId bodyId, float torque);

    /// Sets a body's linear damping. A stale id or a value outside 0 to
    /// M2_RATE_LIMIT changes nothing and counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param bodyId   The body.
    /// @param damping  The damping, 1/s; velocity decays as v / (1 + h d)
    ///                 per substep.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetLinearDamping(m2World* world, m2BodyId bodyId, float damping);

    /// Reads a body's linear damping.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The damping; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetLinearDamping(const m2World* world, m2BodyId bodyId);

    /// Sets a body's angular damping, as m2Body_SetLinearDamping does.
    ///
    /// @param world    The world.
    /// @param bodyId   The body.
    /// @param damping  The damping, 1/s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetAngularDamping(m2World* world, m2BodyId bodyId, float damping);

    /// Reads a body's angular damping.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The damping; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Body_GetAngularDamping(const m2World* world, m2BodyId bodyId);

    /// Sets how strongly gravity pulls a body. A stale id or a scale beyond
    /// M2_GRAVITY_SCALE_LIMIT changes nothing and counts one misuse.
    /// Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param scale   The scale; 1 is normal gravity, 0 none.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetGravityScale(m2World* world, m2BodyId bodyId, float scale);

    /// Sets a body's motion locks. Locking an axis zeroes its velocity, and
    /// changing angularZ recomputes the inertia. The body wakes. A stale id
    /// counts one misuse. Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param locks   The locks.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_SetMotionLocks(m2World* world, m2BodyId bodyId, m2MotionLocks locks);

    /// Reads a body's motion locks.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return The locks; all false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2MotionLocks m2Body_GetMotionLocks(const m2World* world, m2BodyId bodyId);

    /// Allows or forbids a body to sleep; forbidding wakes it. A stale id
    /// counts one misuse. Journaled.
    ///
    /// @param world   The world.
    /// @param bodyId  The body.
    /// @param flag    True to allow sleep.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_EnableSleep(m2World* world, m2BodyId bodyId, bool flag);

    /// Tells whether a body may sleep.
    ///
    /// @param world   The world.
    /// @param bodyId  The body; a stale id counts one misuse.
    /// @return True when it may; false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Body_IsSleepEnabled(const m2World* world, m2BodyId bodyId);

    /// Applies an angular impulse, changing the angular velocity at once.
    /// The body wakes, and its spin saturates at M2_SPEED_LIMIT. A stale id, a
    /// body that is not dynamic or an impulse beyond M2_IMPULSE_LIMIT
    /// changes nothing and counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param bodyId   The dynamic body.
    /// @param impulse  The impulse, N m s.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Body_ApplyAngularImpulse(m2World* world, m2BodyId bodyId, float impulse);

    static const m2BodyId m2_nullBodyId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BODY_H
