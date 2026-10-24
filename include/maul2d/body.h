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

    /// A world-space box in f64, for editor and camera math.
    typedef struct m2AabbResult
    {
        m2Pos2 lowerBound;
        m2Pos2 upperBound;
    } m2AabbResult;

    typedef enum m2BodyType
    {
        m2_staticBody = 0,
        m2_kinematicBody = 1,
        m2_dynamicBody = 2,
    } m2BodyType;

    typedef struct m2BodyId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2BodyId;

    /// Per-axis motion locks. A locked axis holds still: its velocity is
    /// zeroed each substep, so contacts, joints, gravity and impulses
    /// cannot move the body along it. linearX and linearY lock world-frame
    /// translation; angularZ locks rotation and is the same lock as
    /// fixedRotation (setting either locks the spin). All default false.
    typedef struct m2MotionLocks
    {
        bool linearX;
        bool linearY;
        bool angularZ;
    } m2MotionLocks;

    typedef struct m2BodyDef
    {
        m2BodyType type;
        m2Pos2 position; // world space, f64 (hybrid precision)
        m2Rot rotation;
        m2Vec2 linearVelocity;
        float angularVelocity; // radians/s
        float gravityScale;
        float linearDamping; // v / (1 + h d) per substep, implicit
        float angularDamping;
        m2MotionLocks motionLocks; // per-axis motion locks (angularZ aliases fixedRotation)
        bool fixedRotation;        // never rotates: infinite rotational inertia
        bool enableSleep;          // false = this body never sleeps
        bool isEnabled;            // false = created dormant, outside simulation
        int8_t dominance;          // higher wins contacts: it cannot be pushed by lower
        bool isBullet;             // continuous collision vs non-bullets
        uint64_t userData;         // opaque, copied unchanged through snapshots
        int32_t internalValue;
    } m2BodyDef;

    /// Returns a def with pinned defaults and a valid cookie.
    M2_API m2BodyDef m2DefaultBodyDef(void);

    /// Creates a body: m2_errorInvalid for a bad def, m2_errorCapacity when
    /// the world's body pool is full. On failure *bodyOut is the null id.
    /// Thread class: writer.
    M2_NODISCARD M2_API m2Result m2CreateBody(m2World* world, const m2BodyDef* def,
                                              m2BodyId* bodyOut);

    /// Destroys a body with its shapes, chains and joints. The id becomes
    /// stale; the slot is recycled FIFO with a generation bump, and retires
    /// instead of wrapping. m2_errorStale when the body is already gone.
    /// Thread class: writer.
    M2_NODISCARD M2_API m2Result m2DestroyBody(m2World* world, m2BodyId bodyId);

    /// Generation-checked liveness. Thread class: reader.
    M2_API bool m2Body_IsValid(const m2World* world, m2BodyId bodyId);

    M2_API m2Transform m2Body_GetTransform(const m2World* world, m2BodyId bodyId);
    M2_API m2Pos2 m2Body_GetPosition(const m2World* world, m2BodyId bodyId);
    M2_API m2Rot m2Body_GetRotation(const m2World* world, m2BodyId bodyId);
    M2_API m2Vec2 m2Body_GetLinearVelocity(const m2World* world, m2BodyId bodyId);
    M2_API float m2Body_GetAngularVelocity(const m2World* world, m2BodyId bodyId);
    M2_API uint64_t m2Body_GetUserData(const m2World* world, m2BodyId bodyId);

    /// Sleep state. Setters and new contacts wake bodies;
    /// waking is island-transitive at the next step.
    M2_API bool m2Body_IsAwake(const m2World* world, m2BodyId bodyId);

    /// Manual sleep control: false forces the body to sleep NOW
    /// (velocities zero), true wakes it.
    /// Journaled. Thread class: writer.
    M2_API void m2Body_SetAwake(m2World* world, m2BodyId bodyId, bool awake);
    M2_API void m2Body_SetBullet(m2World* world, m2BodyId bodyId, bool flag);
    M2_API void m2Body_SetUserData(m2World* world, m2BodyId bodyId, uint64_t userData);

    /// Contact dominance: in a pair,
    /// the higher-dominance body acts as unmovable toward the lower
    /// one. Statics outrank everything. Enemies stop pushing the
    /// player. Contacts only; joints are unaffected. Journaled.
    M2_API void m2Body_SetDominance(m2World* world, m2BodyId bodyId, int8_t dominance);
    M2_API int8_t m2Body_GetDominance(const m2World* world, m2BodyId bodyId);

    /// Kinematic follow: sets the velocities that carry the body to
    /// the target pose over one step of the given dt. Applies via
    /// the journaled velocity setters, so replays are free.
    M2_API void m2Body_SetTargetTransform(m2World* world, m2BodyId bodyId, m2Pos2 position,
                                          m2Rot rotation, float dt);
    M2_API m2BodyType m2Body_GetType(const m2World* world, m2BodyId bodyId);

    /// Mass properties: mass in kg, body-local centroid, rotational
    /// inertia about the body ORIGIN. SetMassData overrides what the
    /// shapes derived; any later shape change or fixed-rotation flip
    /// recomputes from shapes again (documented lifetime). Journaled.
    typedef struct m2MassData
    {
        float mass;
        m2Vec2 center;           // body-local centroid
        float rotationalInertia; // about the body origin
    } m2MassData;

    M2_API void m2Body_SetMassData(m2World* world, m2BodyId bodyId, m2MassData massData);
    M2_API m2MassData m2Body_GetMassData(const m2World* world, m2BodyId bodyId);
    M2_API void m2Body_ApplyMassFromShapes(m2World* world, m2BodyId bodyId);

    /// Disable removes the body from simulation without destroying it:
    /// shapes leave the broadphase (contacts end, riders wake), joints
    /// stay attached but inert, queries no longer see it. Enable puts
    /// it back where it is. Both journaled. Thread class: writer.
    M2_API void m2Body_Disable(m2World* world, m2BodyId bodyId);
    M2_API void m2Body_Enable(m2World* world, m2BodyId bodyId);
    M2_API bool m2Body_IsEnabled(const m2World* world, m2BodyId bodyId);
    M2_API m2Vec2 m2Body_GetLocalCenter(const m2World* world,
                                        m2BodyId bodyId); // body-frame center of mass
    M2_API bool m2Body_IsBullet(const m2World* world, m2BodyId bodyId);
    M2_API float m2Body_GetGravityScale(const m2World* world, m2BodyId bodyId);

    /// Editor and integration walk: fills ids with up to capacity
    /// live body handles in ascending slot order and returns the
    /// TRUE total, even when it exceeds capacity. Thread class:
    /// reader.
    M2_API int32_t m2World_GetBodies(const m2World* world, m2BodyId* ids, int32_t capacity);

    /// Frame helpers (pure math on the body's pose) and the joint
    /// walk (ascending slot order, truthful total).
    M2_API m2Pos2 m2Body_GetWorldPoint(const m2World* world, m2BodyId bodyId, m2Vec2 localPoint);
    M2_API m2Vec2 m2Body_GetLocalPoint(const m2World* world, m2BodyId bodyId, m2Pos2 worldPoint);
    M2_API m2Vec2 m2Body_GetWorldVector(const m2World* world, m2BodyId bodyId, m2Vec2 localVector);
    M2_API m2Vec2 m2Body_GetLocalVector(const m2World* world, m2BodyId bodyId, m2Vec2 worldVector);
    M2_API m2Vec2 m2Body_GetWorldPointVelocity(const m2World* world, m2BodyId bodyId,
                                               m2Pos2 worldPoint);
    M2_API m2Vec2 m2Body_GetLocalPointVelocity(const m2World* world, m2BodyId bodyId,
                                               m2Vec2 localPoint);
    M2_API m2Pos2 m2Body_GetWorldCenterOfMass(const m2World* world, m2BodyId bodyId);
    M2_API float m2Body_GetRotationalInertia(const m2World* world,
                                             m2BodyId bodyId); // about the center of mass

    /// The tight AABB enclosing every shape on the body (fat tree
    /// margins excluded); a shapeless body returns a point at its
    /// origin. Thread class: reader.
    M2_API m2AabbResult m2Body_ComputeAabb(const m2World* world, m2BodyId bodyId);

    /// Setters wake nothing yet (no sleep system in this slice) but are
    /// already journal-shaped: every mutation is a discrete command.
    /// Thread class: writer.
    M2_API void m2Body_SetLinearVelocity(m2World* world, m2BodyId bodyId, m2Vec2 velocity);
    M2_API void m2Body_SetAngularVelocity(m2World* world, m2BodyId bodyId, float velocity);

    /// Impulses act instantly on the velocity; the world point's arm
    /// is measured from the center of mass. Dynamic bodies only; the
    /// body wakes. Thread class: writer.
    /// Teleports the body: position and rotation snap, velocities are
    /// untouched, the broadphase refreshes immediately, and the body
    /// plus everything it was touching wake up (a sleeping stack must
    /// notice its support vanishing). Teleporting is not a sweep - no
    /// tunneling protection applies. Journaled. Thread class: writer.
    M2_API void m2Body_SetTransform(m2World* world, m2BodyId bodyId, m2Pos2 position,
                                    m2Rot rotation);

    /// Converts the body's type in place. Becoming static zeroes the
    /// velocities; becoming dynamic recomputes mass from the shapes.
    /// The body and everything it touches wake, proxies migrate to the
    /// right tree, and stale pairs end with proper events. Journaled.
    /// Thread class: writer.
    M2_API void m2Body_SetType(m2World* world, m2BodyId bodyId, m2BodyType type);

    M2_API void m2Body_ApplyLinearImpulseAtPoint(m2World* world, m2BodyId bodyId, m2Vec2 impulse,
                                                 m2Pos2 worldPoint);
    M2_API void m2Body_ApplyLinearImpulse(m2World* world, m2BodyId bodyId, m2Vec2 impulse);

    /// Continuous forces: accumulated across calls, applied during the
    /// step, cleared when it ends. Waking is implied. Journaled.
    /// Thread class: writer.
    M2_API void m2Body_ApplyForceAtPoint(m2World* world, m2BodyId bodyId, m2Vec2 force,
                                         m2Pos2 worldPoint);
    M2_API void m2Body_ApplyForce(m2World* world, m2BodyId bodyId, m2Vec2 force);
    M2_API void m2Body_ApplyTorque(m2World* world, m2BodyId bodyId, float torque);

    /// Runtime body dynamics tuning, journaled. Fixed rotation zeroes
    /// the angular velocity and recomputes inertia; disabling sleep
    /// wakes the body so it cannot stay asleep illegally.
    M2_API void m2Body_SetLinearDamping(m2World* world, m2BodyId bodyId, float damping);
    M2_API float m2Body_GetLinearDamping(const m2World* world, m2BodyId bodyId);
    M2_API void m2Body_SetAngularDamping(m2World* world, m2BodyId bodyId, float damping);
    M2_API float m2Body_GetAngularDamping(const m2World* world, m2BodyId bodyId);
    M2_API void m2Body_SetGravityScale(m2World* world, m2BodyId bodyId, float scale);
    M2_API void m2Body_SetFixedRotation(m2World* world, m2BodyId bodyId, bool flag);
    M2_API bool m2Body_IsFixedRotation(const m2World* world, m2BodyId bodyId);

    /// Set or read the per-axis motion locks. angularZ is the same lock as
    /// fixedRotation, so setting it here also fixes the rotation (and
    /// m2Body_IsFixedRotation reflects it). Changing a lock wakes the body
    /// and, for angularZ, recomputes the inertia. Journaled and snapshot
    /// state. Thread class: writer / reader.
    M2_API void m2Body_SetMotionLocks(m2World* world, m2BodyId bodyId, m2MotionLocks locks);
    M2_API m2MotionLocks m2Body_GetMotionLocks(const m2World* world, m2BodyId bodyId);
    M2_API void m2Body_EnableSleep(m2World* world, m2BodyId bodyId, bool flag);
    M2_API bool m2Body_IsSleepEnabled(const m2World* world, m2BodyId bodyId);
    M2_API void m2Body_ApplyAngularImpulse(m2World* world, m2BodyId bodyId, float impulse);

    static const m2BodyId m2_nullBodyId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BODY_H
