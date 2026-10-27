// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: the eleven joint types, their defs, motors, limits,
// springs, runtime tuning and breaking.
//
// Every joint call on a stale id changes nothing, returns zero and
// counts one misuse. A call made for one joint type (a motor joint's
// offsets, a distance joint's length) does the same on a joint of
// another type. Motors and limits belong to the revolute, prismatic and
// wheel joints; m2Joint_SetMaxMotor also sets a motor joint's torque
// budget. Every setter is journaled, and one that changes how a joint
// pulls wakes the bodies it moves.

#ifndef MAUL2D_JOINT_H
#define MAUL2D_JOINT_H

#include "maul2d/body.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum m2JointType
    {
        m2_distanceJoint = 0,
        m2_revoluteJoint = 1,
        m2_prismaticJoint = 2,
        m2_weldJoint = 3,
        m2_wheelJoint = 4,
        m2_filterJoint = 5,
        m2_motorJoint = 6,
        m2_mouseJoint = 7,
        m2_gearJoint = 8,
        m2_pulleyJoint = 9,
        m2_ratchetJoint = 10,
    } m2JointType;

    /// A joint of a world: a 1-based slot and a generation.
    typedef struct m2JointId
    {
        uint32_t index1; // 1-based slot, 0 = null
        uint32_t generation;
    } m2JointId;

    // Every joint def and setter takes its numbers within the ranges of
    // base.h (docs/user/limits.md): hertz from 0 to M2_RATE_LIMIT, damping
    // ratios from 0 to M2_COEFFICIENT_LIMIT, motor speeds within
    // M2_SPEED_LIMIT, budgets from 0 to M2_FORCE_LIMIT, lengths and
    // translations within M2_REACH_LIMIT, angles within M2_ANGLE_LIMIT and
    // ratios from M2_MIN_RATIO to M2_RATIO_LIMIT by magnitude. A def past
    // them is refused.

    /// Keeps two local anchor points at a distance: a rod, or with
    /// enableSpring a spring, a rope or a strut.
    typedef struct m2DistanceJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA; // body-local
        m2Vec2 localAnchorB;
        float length; // meters; 0 or less takes the distance at creation
        /// A hard length range, off by default. A bound that is set clamps
        /// whatever the spring does.
        float minLength;
        float maxLength;   // 0 or less: unbounded
        float springHertz; // 0 with enableSpring: no spring force, only the range
        float springDampingRatio;
        /// Off, a rod: rigid at length, with the range clamping. On, a
        /// spring toward length with springHertz above 0; with springHertz
        /// 0 and a maxLength, a slack rope (free until taut), and with a
        /// minLength, a strut (free until squeezed).
        bool enableSpring;
        M2_ALIGN8 uint64_t userData;   // opaque, kept through snapshots and journals
        bool enableConnectedCollision; // false: the two bodies pass through each other
        int32_t internalValue;         // the cookie m2DefaultDistanceJointDef sets
    } m2DistanceJointDef;

    /// Pins two local anchor points together: a hinge. Limit angles, the
    /// motor and the angular spring are measured from the relative angle at
    /// creation.
    typedef struct m2RevoluteJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        float constraintHertz; // the pin's softness; 0: the stiff default
        float constraintDampingRatio;
        float springHertz; // the angular spring toward the creation angle
        float springDampingRatio;
        bool enableSpring; // off by default; springHertz 0 is no spring force
        bool enableMotor;
        float motorSpeed;     // rad/s, positive: B turns counter-clockwise against A
        float maxMotorTorque; // N m
        bool enableLimit;
        float lowerAngle; // radians from the creation angle
        float upperAngle;
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultRevoluteJointDef sets
    } m2RevoluteJointDef;

    /// A slider: body B moves along an axis fixed in body A's frame, and
    /// the relative rotation stays at its creation value. The motor drives
    /// the speed along the axis; the limits bound the travel from the
    /// separation at creation.
    typedef struct m2PrismaticJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        m2Vec2 localAxisA;     // in body A's frame; normalized at creation
        float constraintHertz; // the slide's softness; 0: the stiff default
        float constraintDampingRatio;
        bool enableMotor;
        float motorSpeed;    // m/s along the axis
        float maxMotorForce; // N
        bool enableLimit;
        float lowerTranslation; // meters from the separation at creation
        float upperTranslation;
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultPrismaticJointDef sets
    } m2PrismaticJointDef;

    /// Welds two bodies into one rigid piece: relative position and
    /// rotation keep their creation values. The linear and angular rows
    /// soften apart: a spring hertz of 0 is rigid, above 0 the row is a
    /// spring.
    typedef struct m2WeldJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        float linearSpringHertz; // 0: rigid
        float linearSpringDampingRatio;
        float angularSpringHertz; // 0: rigid
        float angularSpringDampingRatio;
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultWeldJointDef sets
    } m2WeldJointDef;

    /// A suspension wheel: body B slides on an axis of body A on a spring
    /// and spins freely; the motor drives the relative rotation.
    typedef struct m2WheelJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 localAnchorA;
        m2Vec2 localAnchorB;
        m2Vec2 localAxisA; // in body A's frame; normalized at creation
        bool enableSpring; // the suspension spring
        float springHertz;
        float springDampingRatio;
        bool enableMotor;
        float motorSpeed;     // rad/s of B against A
        float maxMotorTorque; // N m
        bool enableLimit;
        float lowerTranslation; // meters from the separation at creation
        float upperTranslation;
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultWheelJointDef sets
    } m2WheelJointDef;

    /// Drives body B's pose toward an offset from body A within a force
    /// and a torque budget: moving platforms, magnetic grips, soft follow.
    typedef struct m2MotorJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Vec2 linearOffset;    // B's target minus A, in A's frame
        float angularOffset;    // B's target angle minus A's, radians
        float maxForce;         // N
        float maxTorque;        // N m
        float correctionFactor; // the share of the position error removed per step, [0, 1]
        /// springHertz 0 keeps the hard servo at correctionFactor per step.
        /// Above 0 the drive is a soft spring with this frequency and
        /// damping ratio (correctionFactor is then ignored), still within
        /// maxForce and maxTorque. One stiffness serves both rows.
        float springHertz;
        float springDampingRatio;
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultMotorJointDef sets
    } m2MotorJointDef;

    /// A soft spring pulling a point of body B toward a world target: the
    /// way to drag a body. Body A only anchors the bookkeeping. The grabbed
    /// point is where the target lies at creation, in B's frame, so each
    /// of its coordinates there starts within M2_LOCAL_LIMIT. The spring has B's
    /// own mass: it pulls equally hard in every direction, with stiffness
    /// mass (2 pi springHertz)^2, held within maxForce.
    typedef struct m2MouseJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Pos2 target; // world space
        float springHertz;
        float springDampingRatio;
        float maxForce; // N
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultMouseJointDef sets
    } m2MouseJointDef;

    /// Couples two bodies' spins at a ratio: cogs, belts, linked platforms.
    /// ratio * angleA + angleB stays at its value at creation, tracked
    /// across any number of turns. Pin the bodies with their own joints;
    /// the gear couples rotation only.
    typedef struct m2GearJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        /// M2_MIN_RATIO to M2_RATIO_LIMIT by magnitude; meshed cogs take a
        /// positive ratio and spin opposite.
        float ratio;
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultGearJointDef sets
    } m2GearJointDef;

    /// A rope over two fixed world points: lengthA + ratio * lengthB stays
    /// at the total measured at creation, so there is no length to set. The
    /// B side feels ratio times the A-side tension. A side that unspools
    /// fully goes limp near its ground anchor instead of pushing.
    typedef struct m2PulleyJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        m2Pos2 groundAnchorA; // the world point the A rope hangs from
        m2Pos2 groundAnchorB;
        m2Vec2 localAnchorA; // where the rope holds body A, in its frame
        m2Vec2 localAnchorB;
        float ratio; // M2_MIN_RATIO to M2_RATIO_LIMIT
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultPulleyJointDef sets
    } m2PulleyJointDef;

    /// A ratchet: relative rotation runs free in one direction, clicking
    /// tooth by tooth, and never runs back past the last engaged tooth
    /// (socket wrenches, winches, turnstiles). Pin the bodies with their
    /// own joints; the ratchet owns rotation only, across any number of
    /// turns.
    typedef struct m2RatchetJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        /// The tooth angle, M2_MIN_RATCHET_TOOTH to M2_ANGLE_LIMIT by
        /// magnitude; its sign is the free direction.
        float ratchet;
        float phase; // offsets the teeth, radians
        M2_ALIGN8 uint64_t userData;
        bool enableConnectedCollision;
        int32_t internalValue; // the cookie m2DefaultRatchetJointDef sets
    } m2RatchetJointDef;

    /// A joint with no constraint: it only switches collision off between
    /// its two bodies for as long as it lives.
    typedef struct m2FilterJointDef
    {
        m2BodyId bodyIdA;
        m2BodyId bodyIdB;
        M2_ALIGN8 uint64_t userData;
        int32_t internalValue; // the cookie m2DefaultFilterJointDef sets
    } m2FilterJointDef;

    /// Returns a distance joint def: zero anchors, the length taken at
    /// creation, no range, the stiff default, no collision between the
    /// bodies.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2DistanceJointDef m2DefaultDistanceJointDef(void);

    /// Returns a revolute joint def: zero anchors, no spring, motor or
    /// limit.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2RevoluteJointDef m2DefaultRevoluteJointDef(void);

    /// Returns a prismatic joint def: zero anchors, the x axis, no motor or
    /// limit.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2PrismaticJointDef m2DefaultPrismaticJointDef(void);

    /// Returns a weld joint def: zero anchors, both rows rigid.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2WeldJointDef m2DefaultWeldJointDef(void);

    /// Returns a wheel joint def: zero anchors, the y axis, a 2 Hz
    /// suspension at damping ratio 0.7, no motor or limit.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2WheelJointDef m2DefaultWheelJointDef(void);

    /// Returns a filter joint def.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2FilterJointDef m2DefaultFilterJointDef(void);

    /// Returns a gear joint def with ratio 1.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2GearJointDef m2DefaultGearJointDef(void);

    /// Returns a pulley joint def with ratio 1 and every anchor at the
    /// origin.
    ///
    /// @return The default def; set both bodies and both ground anchors.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2PulleyJointDef m2DefaultPulleyJointDef(void);

    /// Returns a ratchet joint def with a 0.5 radian tooth.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2RatchetJointDef m2DefaultRatchetJointDef(void);

    /// Returns a motor joint def: zero offsets, a 1 N force and 1 N m
    /// torque budget, correction factor 0.3, the hard servo.
    ///
    /// @return The default def; set both bodies.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2MotorJointDef m2DefaultMotorJointDef(void);

    /// Returns a mouse joint def: a 4 Hz spring at damping ratio 1 with a
    /// 35 N budget, the target at the origin.
    ///
    /// @return The default def; set both bodies and the target.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2MouseJointDef m2DefaultMouseJointDef(void);

    /// Creates a distance joint. A joint joins its bodies' sleep island,
    /// so they sleep and wake together, and both wake now. Destroying
    /// either body destroys the joint. Journaled. Every joint def's local
    /// anchors, axis and linear offset lie within M2_LOCAL_LIMIT.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultDistanceJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return m2_success; m2_errorStale when either body is stale;
    ///         m2_errorInvalid for a bad def, the same body twice or a null
    ///         argument; m2_errorLimit for a value of the def past its
    ///         published bound; m2_errorCapacity when the joint pool is
    ///         full; m2_errorBusy from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateDistanceJoint(m2World* world,
                                                       const m2DistanceJointDef* def,
                                                       m2JointId* jointOut);

    /// Creates a revolute joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultRevoluteJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateRevoluteJoint(m2World* world,
                                                       const m2RevoluteJointDef* def,
                                                       m2JointId* jointOut);

    /// Creates a prismatic joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultPrismaticJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreatePrismaticJoint(m2World* world,
                                                        const m2PrismaticJointDef* def,
                                                        m2JointId* jointOut);

    /// Creates a weld joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultWeldJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateWeldJoint(m2World* world, const m2WeldJointDef* def,
                                                   m2JointId* jointOut);

    /// Creates a wheel joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultWheelJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateWheelJoint(m2World* world, const m2WheelJointDef* def,
                                                    m2JointId* jointOut);

    /// Creates a filter joint, as m2CreateDistanceJoint does. Destroying it
    /// switches collision back on, with the usual events.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultFilterJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateFilterJoint(m2World* world, const m2FilterJointDef* def,
                                                     m2JointId* jointOut);

    /// Creates a gear joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultGearJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateGearJoint(m2World* world, const m2GearJointDef* def,
                                                   m2JointId* jointOut);

    /// Creates a pulley joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultPulleyJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreatePulleyJoint(m2World* world, const m2PulleyJointDef* def,
                                                     m2JointId* jointOut);

    /// Creates a ratchet joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultRatchetJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateRatchetJoint(m2World* world, const m2RatchetJointDef* def,
                                                      m2JointId* jointOut);

    /// Creates a motor joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultMotorJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateMotorJoint(m2World* world, const m2MotorJointDef* def,
                                                    m2JointId* jointOut);

    /// Creates a mouse joint, as m2CreateDistanceJoint does.
    ///
    /// @param world     The world.
    /// @param def       The joint, from m2DefaultMouseJointDef.
    /// @param jointOut  Receives the joint; the null id on failure.
    /// @return As m2CreateDistanceJoint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2CreateMouseJoint(m2World* world, const m2MouseJointDef* def,
                                                    m2JointId* jointOut);

    /// Destroys a joint and wakes its bodies. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return m2_success; m2_errorStale when it is already gone;
    ///         m2_errorBusy from host code the world is running.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_NODISCARD M2_API m2Result m2DestroyJoint(m2World* world, m2JointId jointId);

    /// Sets a motor's speed: rad/s on revolute and wheel joints, m/s on a
    /// prismatic joint. A speed beyond M2_SPEED_LIMIT or another joint type
    /// counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @param speed    The target speed.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetMotorSpeed(m2World* world, m2JointId jointId, float speed);

    /// Sets a motor's budget: a torque on revolute and wheel joints and on
    /// a motor joint, a force on a prismatic joint. A budget outside 0 to
    /// M2_FORCE_LIMIT or another joint type counts one misuse.
    ///
    /// @param world             The world.
    /// @param jointId           A revolute, prismatic, wheel or motor joint.
    /// @param maxTorqueOrForce  The budget, N m or N.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetMaxMotor(m2World* world, m2JointId jointId, float maxTorqueOrForce);

    /// Turns a motor on or off. Another joint type counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @param enable   True to drive.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_EnableMotor(m2World* world, m2JointId jointId, bool enable);

    /// Turns the limits on or off. Another joint type counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @param enable   True to limit.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_EnableLimit(m2World* world, m2JointId jointId, bool enable);

    /// Sets the limits: angles in radians on a revolute joint, travel in
    /// meters on prismatic and wheel joints, both from the creation pose.
    /// An inverted range, an angle beyond M2_ANGLE_LIMIT, a travel beyond
    /// M2_REACH_LIMIT or another joint type changes nothing, on either
    /// bound, and counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @param lower    The lower limit.
    /// @param upper    The upper limit, at least lower.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetLimits(m2World* world, m2JointId jointId, float lower, float upper);

    /// Sets the loads past which a joint breaks during a step. A broken
    /// joint is destroyed and reported by m2World_GetJointEvents. A
    /// negative or NaN threshold changes nothing and counts one misuse.
    ///
    /// @param world      The world.
    /// @param jointId    The joint.
    /// @param maxForce   The reaction force, N; 0, the default, never breaks.
    /// @param maxTorque  The reaction torque, N m; 0, the default, never
    ///                   breaks.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetBreakLimits(m2World* world, m2JointId jointId, float maxForce,
                                       float maxTorque);

    /// Sets the softness of the rows that hold a joint together: a revolute
    /// joint's pin, a prismatic joint's slide, the tie of a gear, pulley or
    /// ratchet joint. Another joint type, or a value out of its range,
    /// counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic, gear, pulley or ratchet joint.
    /// @param hertz    The frequency; 0 is the stiff default.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetConstraintHertz(m2World* world, m2JointId jointId, float hertz);

    /// Sets the damping ratio of the rows m2Joint_SetConstraintHertz softens.
    /// Another joint type, or a value out of its range, counts one misuse.
    /// Journaled.
    ///
    /// @param world         The world.
    /// @param jointId       A revolute, prismatic, gear, pulley or ratchet joint.
    /// @param dampingRatio  The ratio, at least 0.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetConstraintDampingRatio(m2World* world, m2JointId jointId,
                                                  float dampingRatio);

    /// Sets a joint's spring frequency: a distance joint's spring, a wheel's
    /// suspension, a revolute joint's angular spring, a motor joint's drive, a
    /// mouse joint's drag. A distance, wheel or revolute spring acts only while
    /// it is enabled (m2Joint_EnableSpring). Another joint type, or a value out
    /// of its range, counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  A distance, wheel, revolute, motor or mouse joint.
    /// @param hertz    The frequency; 0 is no spring force, or the hard servo
    ///                 on a motor joint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetSpringHertz(m2World* world, m2JointId jointId, float hertz);

    /// Sets the damping ratio of the spring m2Joint_SetSpringHertz tunes.
    /// Another joint type, or a value out of its range, counts one misuse.
    /// Journaled.
    ///
    /// @param world         The world.
    /// @param jointId       A distance, wheel, revolute, motor or mouse joint.
    /// @param dampingRatio  The ratio, at least 0.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetSpringDampingRatio(m2World* world, m2JointId jointId,
                                              float dampingRatio);

    /// Softens a weld joint's linear rows into a spring. Another joint type, or
    /// a value out of its range, counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  A weld joint.
    /// @param hertz    The frequency; 0 is rigid.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2WeldJoint_SetLinearSpringHertz(m2World* world, m2JointId jointId, float hertz);

    /// Sets the damping ratio of a weld joint's linear spring. Another joint
    /// type, or a value out of its range, counts one misuse. Journaled.
    ///
    /// @param world         The world.
    /// @param jointId       A weld joint.
    /// @param dampingRatio  The ratio, at least 0.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2WeldJoint_SetLinearSpringDampingRatio(m2World* world, m2JointId jointId,
                                                        float dampingRatio);

    /// Softens a weld joint's angular row into a spring. Another joint type, or
    /// a value out of its range, counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  A weld joint.
    /// @param hertz    The frequency; 0 is rigid.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2WeldJoint_SetAngularSpringHertz(m2World* world, m2JointId jointId, float hertz);

    /// Sets the damping ratio of a weld joint's angular spring. Another joint
    /// type, or a value out of its range, counts one misuse. Journaled.
    ///
    /// @param world         The world.
    /// @param jointId       A weld joint.
    /// @param dampingRatio  The ratio, at least 0.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2WeldJoint_SetAngularSpringDampingRatio(m2World* world, m2JointId jointId,
                                                         float dampingRatio);

    /// Sets a distance joint's rest length and drops its stored impulses.
    /// Another joint type, or a length that is not positive or is beyond
    /// M2_REACH_LIMIT, counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A distance joint.
    /// @param length   The length, meters.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2DistanceJoint_SetLength(m2World* world, m2JointId jointId, float length);

    /// Sets a distance joint's hard length range. Each bound is raised to
    /// the linear slop (0.005 m) and the two are put in order. Another
    /// joint type or a bound beyond M2_REACH_LIMIT counts one misuse. Read the range
    /// with m2DistanceJoint_GetLengthRange.
    ///
    /// @param world      The world.
    /// @param jointId    A distance joint.
    /// @param minLength  The shortest the joint may get, meters.
    /// @param maxLength  The longest, meters.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2DistanceJoint_SetLengthRange(m2World* world, m2JointId jointId, float minLength,
                                               float maxLength);

    /// Reads a distance joint's rest length.
    ///
    /// @param world    The world.
    /// @param jointId  A distance joint; another type or a stale id counts
    ///                 one misuse.
    /// @return The length, meters; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2DistanceJoint_GetLength(const m2World* world, m2JointId jointId);

    /// Reads the length range m2DistanceJoint_SetLengthRange sets.
    ///
    /// @param world         The world.
    /// @param jointId       A distance joint; another type or a stale id
    ///                      writes zeros and counts one misuse.
    /// @param minLengthOut  Receives the shortest length, meters; may be null.
    /// @param maxLengthOut  Receives the longest length, meters; may be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API void m2DistanceJoint_GetLengthRange(const m2World* world, m2JointId jointId,
                                               float* minLengthOut, float* maxLengthOut);

    /// Reads the force a joint carried on the last step: its stored
    /// impulses over that step's substep length. The break pass compares
    /// the same number with the break limits, bit for bit, so a threshold
    /// tuned against this reading behaves exactly as read.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The force, N; 0 before the first step and for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetReactionForce(const m2World* world, m2JointId jointId);

    /// Reads the torque a joint carried on the last step, as
    /// m2Joint_GetReactionForce does.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The torque, N m; 0 before the first step and for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetReactionTorque(const m2World* world, m2JointId jointId);

    /// Tells whether a joint id names a live joint. Counts no misuse.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return True for a live joint.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Joint_IsValid(const m2World* world, m2JointId jointId);

    /// Reads a joint's type.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The type; m2_distanceJoint for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2JointType m2Joint_GetType(const m2World* world, m2JointId jointId);

    /// Reads a joint's anchor on body A, in A's frame.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The anchor; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Joint_GetLocalAnchorA(const m2World* world, m2JointId jointId);

    /// Reads a joint's anchor on body B, in B's frame.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The anchor; zero for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Joint_GetLocalAnchorB(const m2World* world, m2JointId jointId);

    /// Reads the axis of a prismatic or wheel joint, in body A's frame.
    ///
    /// @param world    The world.
    /// @param jointId  A prismatic or wheel joint.
    /// @return The unit axis; zero for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2Joint_GetLocalAxisA(const m2World* world, m2JointId jointId);

    /// Reads the frequency m2Joint_SetConstraintHertz sets.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic, gear, pulley or ratchet joint.
    /// @return  The frequency; 0 for another joint type or a stale id, which
    ///         count one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetConstraintHertz(const m2World* world, m2JointId jointId);

    /// Reads the damping ratio m2Joint_SetConstraintDampingRatio sets.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic, gear, pulley or ratchet joint.
    /// @return  The ratio; 0 for another joint type or a stale id, which count
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetConstraintDampingRatio(const m2World* world, m2JointId jointId);

    /// Reads the frequency m2Joint_SetSpringHertz sets.
    ///
    /// @param world    The world.
    /// @param jointId  A distance, wheel, revolute, motor or mouse joint.
    /// @return  The frequency; 0 for another joint type or a stale id, which
    ///         count one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetSpringHertz(const m2World* world, m2JointId jointId);

    /// Reads the damping ratio m2Joint_SetSpringDampingRatio sets.
    ///
    /// @param world    The world.
    /// @param jointId  A distance, wheel, revolute, motor or mouse joint.
    /// @return  The ratio; 0 for another joint type or a stale id, which count
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetSpringDampingRatio(const m2World* world, m2JointId jointId);

    /// Reads a weld joint's linear spring frequency.
    ///
    /// @param world    The world.
    /// @param jointId  A weld joint.
    /// @return  The frequency; 0 for another joint type or a stale id, which
    ///         count one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2WeldJoint_GetLinearSpringHertz(const m2World* world, m2JointId jointId);

    /// Reads a weld joint's linear spring damping ratio.
    ///
    /// @param world    The world.
    /// @param jointId  A weld joint.
    /// @return  The ratio; 0 for another joint type or a stale id, which count
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2WeldJoint_GetLinearSpringDampingRatio(const m2World* world, m2JointId jointId);

    /// Reads a weld joint's angular spring frequency.
    ///
    /// @param world    The world.
    /// @param jointId  A weld joint.
    /// @return  The frequency; 0 for another joint type or a stale id, which
    ///         count one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2WeldJoint_GetAngularSpringHertz(const m2World* world, m2JointId jointId);

    /// Reads a weld joint's angular spring damping ratio.
    ///
    /// @param world    The world.
    /// @param jointId  A weld joint.
    /// @return  The ratio; 0 for another joint type or a stale id, which count
    ///         one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2WeldJoint_GetAngularSpringDampingRatio(const m2World* world, m2JointId jointId);

    /// Reads a motor's target speed.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @return The speed; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetMotorSpeed(const m2World* world, m2JointId jointId);

    /// Reads a motor's budget.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic, wheel or motor joint.
    /// @return The torque or force; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetMaxMotor(const m2World* world, m2JointId jointId);

    /// Tells whether a motor drives.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @return True when on; false for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Joint_IsMotorEnabled(const m2World* world, m2JointId jointId);

    /// Tells whether the limits hold.
    ///
    /// @param world    The world.
    /// @param jointId  A revolute, prismatic or wheel joint.
    /// @return True when on; false for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Joint_IsLimitEnabled(const m2World* world, m2JointId jointId);

    /// Turns a joint's spring on or off: a distance joint's, which makes a
    /// rod a spring (or, at spring hertz 0, a slack rope or a strut), a
    /// wheel's suspension, or a revolute joint's angular spring. A wheel's
    /// or revolute joint's spring row drops its accumulated impulse; a
    /// distance joint's rod and spring share the rest row, which keeps it.
    /// Another joint type counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  A distance, wheel or revolute joint.
    /// @param enable   True to turn the spring on.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_EnableSpring(m2World* world, m2JointId jointId, bool enable);

    /// Tells whether a distance, wheel or revolute joint's spring is on.
    ///
    /// @param world    The world.
    /// @param jointId  A distance, wheel or revolute joint.
    /// @return True when its spring is on; false for another joint type or
    ///         a stale id, which count one misuse.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Joint_IsSpringEnabled(const m2World* world, m2JointId jointId);

    /// Reads the limits m2Joint_SetLimits sets.
    ///
    /// @param world     The world.
    /// @param jointId   A revolute, prismatic or wheel joint; another type
    ///                  or a stale id writes zeros.
    /// @param lowerOut  Receives the lower limit; may be null.
    /// @param upperOut  Receives the upper limit; may be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API void m2Joint_GetLimits(const m2World* world, m2JointId jointId, float* lowerOut,
                                  float* upperOut);

    /// Reads the break limits m2Joint_SetBreakLimits sets.
    ///
    /// @param world         The world.
    /// @param jointId       The joint; a stale id writes zeros.
    /// @param maxForceOut   Receives the force threshold; may be null.
    /// @param maxTorqueOut  Receives the torque threshold; may be null.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API void m2Joint_GetBreakLimits(const m2World* world, m2JointId jointId, float* maxForceOut,
                                       float* maxTorqueOut);

    /// Reads a joint's body A.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The body; the null id for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2BodyId m2Joint_GetBodyA(const m2World* world, m2JointId jointId);

    /// Reads a joint's body B.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The body; the null id for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2BodyId m2Joint_GetBodyB(const m2World* world, m2JointId jointId);

    /// Lets a joint's two bodies collide with each other, or keeps them
    /// apart, as the def's enableConnectedCollision does at creation. Both
    /// bodies wake and their pairs are found again on the next step. A
    /// filter joint, which exists to keep its bodies apart, or a stale id
    /// counts one misuse. Journaled.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @param enable   True to let the bodies collide.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_EnableConnectedCollision(m2World* world, m2JointId jointId, bool enable);

    /// Tells whether a joint's two bodies collide with each other.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return True when they collide; false for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API bool m2Joint_IsConnectedCollisionEnabled(const m2World* world, m2JointId jointId);

    /// Reads a joint's game data.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The value; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API uint64_t m2Joint_GetUserData(const m2World* world, m2JointId jointId);

    /// Sets a joint's opaque game data, kept through snapshots and journals.
    ///
    /// @param world     The world.
    /// @param jointId   The joint.
    /// @param userData  Any value.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2Joint_SetUserData(m2World* world, m2JointId jointId, uint64_t userData);

    /// Measures how far a joint is now from what it holds: the anchor gap
    /// of point constraints, a distance joint's length error, a slider's
    /// gap off its axis. Bodies more than M2_REACH_LIMIT apart along an axis
    /// are taken as that far apart, as the solver takes them.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The distance, meters; 0 for a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetLinearSeparation(const m2World* world, m2JointId jointId);

    /// Measures a joint's angle error now, unwound, where it holds an
    /// angle.
    ///
    /// @param world    The world.
    /// @param jointId  The joint.
    /// @return The error, radians; 0 where no angle is held and for a stale
    ///         id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2Joint_GetAngularSeparation(const m2World* world, m2JointId jointId);

    /// Moves a motor joint's target, as a moving platform does every frame.
    /// The torque budget is set with m2Joint_SetMaxMotor. Another joint
    /// type, an angular offset beyond M2_ANGLE_LIMIT or a linear offset beyond
    /// M2_LOCAL_LIMIT counts one misuse.
    ///
    /// @param world          The world.
    /// @param jointId        A motor joint.
    /// @param linearOffset   B's target minus A, in A's frame.
    /// @param angularOffset  B's target angle minus A's, radians.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2MotorJoint_SetOffsets(m2World* world, m2JointId jointId, m2Vec2 linearOffset,
                                        float angularOffset);

    /// Reads a motor joint's linear offset.
    ///
    /// @param world    The world.
    /// @param jointId  A motor joint.
    /// @return The offset; zero for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Vec2 m2MotorJoint_GetLinearOffset(const m2World* world, m2JointId jointId);

    /// Reads a motor joint's angular offset.
    ///
    /// @param world    The world.
    /// @param jointId  A motor joint.
    /// @return The offset, radians; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2MotorJoint_GetAngularOffset(const m2World* world, m2JointId jointId);

    /// Reads a motor joint's force budget.
    ///
    /// @param world    The world.
    /// @param jointId  A motor joint.
    /// @return The force, N; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2MotorJoint_GetMaxForce(const m2World* world, m2JointId jointId);

    /// Reads a motor joint's correction factor.
    ///
    /// @param world    The world.
    /// @param jointId  A motor joint.
    /// @return The factor; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2MotorJoint_GetCorrectionFactor(const m2World* world, m2JointId jointId);

    /// Sets a gear joint's ratio. Another joint type, or a ratio whose
    /// magnitude is outside M2_MIN_RATIO to M2_RATIO_LIMIT, counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A gear joint.
    /// @param ratio    The new ratio.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2GearJoint_SetRatio(m2World* world, m2JointId jointId, float ratio);

    /// Reads a gear joint's ratio.
    ///
    /// @param world    The world.
    /// @param jointId  A gear joint.
    /// @return The ratio; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2GearJoint_GetRatio(const m2World* world, m2JointId jointId);

    /// Sets a pulley joint's ratio. The rope total is measured again from
    /// the current pose, so the machine does not snap, and the stored
    /// impulse is dropped. Another joint type, or a ratio outside
    /// M2_MIN_RATIO to M2_RATIO_LIMIT, counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A pulley joint.
    /// @param ratio    The new ratio.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2PulleyJoint_SetRatio(m2World* world, m2JointId jointId, float ratio);

    /// Reads a pulley joint's ratio.
    ///
    /// @param world    The world.
    /// @param jointId  A pulley joint.
    /// @return The ratio; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2PulleyJoint_GetRatio(const m2World* world, m2JointId jointId);

    /// Measures the A side of a pulley's rope now.
    ///
    /// @param world    The world.
    /// @param jointId  A pulley joint.
    /// @return The length, meters; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2PulleyJoint_GetLengthA(const m2World* world, m2JointId jointId);

    /// Measures the B side of a pulley's rope now.
    ///
    /// @param world    The world.
    /// @param jointId  A pulley joint.
    /// @return The length, meters; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2PulleyJoint_GetLengthB(const m2World* world, m2JointId jointId);

    /// Reads the world point a pulley's A rope hangs from.
    ///
    /// @param world    The world.
    /// @param jointId  A pulley joint.
    /// @return The point; the origin for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2PulleyJoint_GetGroundAnchorA(const m2World* world, m2JointId jointId);

    /// Reads the world point a pulley's B rope hangs from.
    ///
    /// @param world    The world.
    /// @param jointId  A pulley joint.
    /// @return The point; the origin for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2PulleyJoint_GetGroundAnchorB(const m2World* world, m2JointId jointId);

    /// Reads a ratchet's tooth angle.
    ///
    /// @param world    The world.
    /// @param jointId  A ratchet joint.
    /// @return The angle, radians, signed by the free direction; 0 for
    ///         another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2RatchetJoint_GetRatchet(const m2World* world, m2JointId jointId);

    /// Reads a ratchet's tooth phase.
    ///
    /// @param world    The world.
    /// @param jointId  A ratchet joint.
    /// @return The phase, radians; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2RatchetJoint_GetPhase(const m2World* world, m2JointId jointId);

    /// Moves a mouse joint's target. Another joint type or a non-finite
    /// target counts one misuse.
    ///
    /// @param world    The world.
    /// @param jointId  A mouse joint.
    /// @param target   The new target, in world space.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time.
    M2_API void m2MouseJoint_SetTarget(m2World* world, m2JointId jointId, m2Pos2 target);

    /// Reads a mouse joint's target.
    ///
    /// @param world    The world.
    /// @param jointId  A mouse joint.
    /// @return The target; the origin for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API m2Pos2 m2MouseJoint_GetTarget(const m2World* world, m2JointId jointId);

    /// Reads a mouse joint's force budget.
    ///
    /// @param world    The world.
    /// @param jointId  A mouse joint.
    /// @return The force, N; 0 for another joint type or a stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API float m2MouseJoint_GetMaxForce(const m2World* world, m2JointId jointId);

    /// Lists the world's joints in ascending slot order.
    ///
    /// @param world     The world.
    /// @param ids       Receives up to capacity joints; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of joints, which may exceed capacity.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2World_GetJoints(const m2World* world, m2JointId* ids, int32_t capacity);

    /// Lists the joints attached to a body, in ascending slot order.
    ///
    /// @param world     The world.
    /// @param bodyId    The body; a stale id counts one misuse.
    /// @param ids       Receives up to capacity joints; may be null to
    ///                  measure.
    /// @param capacity  The length of ids.
    /// @return The number of joints, which may exceed capacity; 0 for a
    ///         stale id.
    /// @par Thread safety
    /// Safe from any thread; a world is used by one thread at a time, or by
    /// several readers at once.
    M2_API int32_t m2Body_GetJoints(const m2World* world, m2BodyId bodyId, m2JointId* ids,
                                    int32_t capacity);

    static const m2JointId m2_nullJointId = {0, 0};

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_JOINT_H
