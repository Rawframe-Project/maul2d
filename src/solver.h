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

// Joint rows without a user softness: stiff, and damped well past
// critical so they settle without ringing.
#define M2_JOINT_HERTZ         60.0f
#define M2_JOINT_DAMPING_RATIO 2.0f

// A safety bound on linear speed, not a gameplay knob: a degenerate or
// over-constrained setup is held here instead of pumping speed without
// limit into a NaN. It stays off the world def, like the quarter-turn
// angular cap, so tuning cannot touch it; 400 m/s is far above any real
// 2D motion.
#define M2_MAX_LINEAR_SPEED      400.0f
#define M2_RESTITUTION_THRESHOLD 1.0f

// A soft row: the constraint behaves as a spring of frequency hertz and
// damping ratio zeta on the row's effective mass m.
typedef struct m2Softness
{
    float biasRate;     // fraction of the position error removed per second
    float massScale;    // the row's mass is scaled down by this
    float impulseScale; // this fraction of the accumulated impulse leaks away
} m2Softness;

// With w = 2 pi hertz, stiffness k = m w^2 and damping c = 2 m zeta w, an
// implicit Euler step of length h turns the spring into a row that solves
//   J v + (k / (h k + c)) C + (1 / (h (h k + c))) lambda = 0
// for the total impulse lambda. Let a = h w (2 zeta + h w). Then
//   biasRate     = k / (h k + c) = w / (2 zeta + h w)
//   massScale    = a / (1 + a)   (m against m + the lambda term)
//   impulseScale = 1 / (1 + a)   (the lambda term on the accumulated part)
// and a solve adds -m massScale (J v + biasRate C) - impulseScale
// accumulated. Zero hertz is a rigid row with no position feedback.
static inline m2Softness m2MakeSoft(float hertz, float zeta, float h)
{
    if (hertz == 0.0f)
    {
        return (m2Softness){0.0f, 0.0f, 0.0f};
    }
    float omega = 2.0f * M2_PI * hertz;
    float a = h * omega * (2.0f * zeta + h * omega);
    float leak = 1.0f / (1.0f + a);
    return (m2Softness){omega / (2.0f * zeta + h * omega), a * leak, leak};
}

static inline m2Vec2 m2RotateVec2(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

static inline float m2Cross2(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

// Contacts in one graph color share no dynamic body, so a color solves
// in parallel. Colors are assigned greedily in canonical pair order and
// the colored order is used even on one thread, so the worker count
// never changes the arithmetic.
#define M2_GRAPH_COLORS 24 // one more range holds the overflow, solved serially

// The soft-step solve for one step.
void m2SolveStep(m2World* world, float dt, int32_t substepCount);

#endif // MAUL2D_SRC_SOLVER_H
