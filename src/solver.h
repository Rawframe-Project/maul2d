// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Types, tuning constants and small helpers shared by the solver
// modules, and the step entry point.

#ifndef MAUL2D_SRC_SOLVER_H
#define MAUL2D_SRC_SOLVER_H

#include "world_internal.h"

#define M2_CONTACT_HERTZ          30.0f
#define M2_CONTACT_DAMPING_RATIO  10.0f
#define M2_CONTACT_PUSH_MAX_SPEED 3.0f

// Reference (b2DefaultWorldDef) default maximum linear speed, 400 m/s. It
// is a SAFETY bound, not a gameplay knob: an over-constrained or
// near-degenerate configuration is bounded here instead of pumping speed
// exponentially to infinity and then a NaN. Hardcoded like the angular cap
// (M2_PI quarter-turn) so the guard stays off the determinism-sensitive
// worldDef surface; 400 m/s leaves ample headroom over any real 2D motion.
#define M2_MAX_LINEAR_SPEED      400.0f
#define M2_RESTITUTION_THRESHOLD 1.0f

typedef struct m2Softness
{
    float biasRate;
    float massScale;
    float impulseScale;
} m2Softness;

// Reference formula (b2MakeSoft): bias = w/(2z+hw),
// massScale = hw(2z+hw)/(1+hw(2z+hw)), impulseScale = 1/(1+hw(2z+hw)).
static inline m2Softness m2MakeSoft(float hertz, float zeta, float h)
{
    if (hertz == 0.0f)
    {
        return (m2Softness){0.0f, 0.0f, 0.0f};
    }
    float omega = 2.0f * M2_PI * hertz;
    float a1 = 2.0f * zeta + h * omega;
    float a2 = h * omega * a1;
    float a3 = 1.0f / (1.0f + a2);
    return (m2Softness){omega / a1, a2 * a3, a3};
}

static inline m2Vec2 m2RotateVec2(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

static inline float m2Cross2(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

typedef struct m2ConstraintPoint
{
    m2Vec2 rA; // anchor relative to body origin, world-rotated at prepare
    m2Vec2 rB;
    float baseSeparation;
    float relativeVelocity; // normal speed at prepare (restitution input)
    float normalMass;
    float tangentMass;
    float normalImpulse;
    float tangentImpulse;
    uint16_t id;
    uint16_t persisted;
} m2ConstraintPoint;

typedef struct m2JointConstraint
{
    int32_t jointIndex;
    int32_t bodyA;
    int32_t bodyB;
    uint8_t type;  // m2JointType, the index into the kind table
    uint8_t flags; // M2_JOINT_* (joint.h)
    m2Vec2 rA;     // world-rotated anchors at prepare
    m2Vec2 rB;
    m2Vec2 axis;         // distance/prismatic: unit axis at prepare
    m2Vec2 perp;         // prismatic: left-perp of axis
    float baseC;         // distance: C0; prismatic: translation0
    m2Vec2 baseCVec;     // revolute: C0; prismatic: (perpC0, unused)
    float baseAngle;     // relative angle at prepare minus reference
    float a1, a2;        // prismatic axial torque arms
    float s1, s2;        // prismatic perpendicular torque arms
    float axialMass;     // distance/prismatic axial; revolute 1/(iA+iB)
    float k11, k12, k22; // revolute/prismatic 2x2 effective mass
    float motorSpeed;
    float maxMotorImpulse; // h * maxMotorTorque(Force), clamp budget
    float lower;
    float upper;
    m2Softness softness;       // constraint rows (stiff for wheel); weld linear
    m2Softness springSoftness; // wheel suspension (real spring)
    m2Softness softness2;      // weld angular row
    bool linearSpring;         // weld: nonzero hertz = biased even in relax
    bool angularSpring;
    m2Vec2 impulse;     // point/axial; prismatic (perp, angle); wheel (perp, spring)
    float motorImpulse; // motor accumulator; weld reuses it for the angle lock
    float lowerImpulse;
    float upperImpulse;
    float springImpulse; // revolute angular spring
} m2JointConstraint;

typedef struct m2ContactConstraint
{
    int32_t pairIndex;
    int32_t bodyA;
    int32_t bodyB;
    // Pair-effective masses: usually the bodies' own, but dominance
    // zeroes one side so the winner cannot be pushed in this pair.
    float invMassA;
    float invIA;
    float invMassB;
    float invIB;
    m2Vec2 normal; // world frame
    float friction;
    float restitution;
    float tangentSpeed; // conveyor: sum of both shapes (reference mixing)
    m2Softness softness;
    int32_t pointCount;
    m2ConstraintPoint points[2];
} m2ContactConstraint;

// --- Graph coloring: constraints in one color share no
// dynamic body, so a color solves in parallel with bit-identical
// results at ANY worker count. The color assignment itself is greedy
// over canonical constraint order - fully deterministic. The colored
// order is used even when serial, so worker count can never change
// the arithmetic sequence.

#define M2_GRAPH_COLORS 24 // colors 0..23; 24 = overflow, solved serially

typedef enum m2ContactStage
{
    m2_stageWarmStart,
    m2_stageSolve,
    m2_stageRestitution,
    m2_stageStore,
} m2ContactStage;

typedef struct m2ContactStageCtx
{
    m2World* world;
    m2ContactConstraint* constraints;
    const int32_t* order;
    m2ContactStage stage;
    float invH;
    float minBiasVel;
    bool useBias;
} m2ContactStageCtx;

void m2StoreBodyVelocities(m2World* world, const m2ContactConstraint* c, m2Vec2 vA, float wA,
                           m2Vec2 vB, float wB);

#endif // MAUL2D_SRC_SOLVER_H
