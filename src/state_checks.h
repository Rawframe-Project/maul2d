// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The rules a restore checks each array of an incoming snapshot against,
// one per kind of element, before any byte lands (m2StateValidate walks
// the state table and names each array's rule). They read the bytes in
// place, unaligned, and write nothing.

#ifndef MAUL2D_SRC_STATE_CHECKS_H
#define MAUL2D_SRC_STATE_CHECKS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// What a restore checks in an incoming block before any of it lands, so
// hostile snapshot bytes can never index out of bounds or feed the step a
// number past its range.
typedef enum m2Check
{
    m2_check_none,
    m2_check_index,       // int32 elements in [0, range)
    m2_check_indexOrNone, // int32 elements in [-1, range)
    m2_check_count,       // an inline int32 in [0, range]
    m2_check_cursor,      // an inline int32 ring cursor in [0, max(range - 1, 0)]
    m2_check_bool,        // uint8 elements 0 or 1
    m2_check_bodyType,    // uint8 elements below the body type count
    m2_check_jointType,   // uint8 elements below the joint type count
    m2_check_geometry,    // shape geometry: a known type, a polygon count that fits
    m2_check_manifold,    // manifolds: point counts that fit, impulses in range
    m2_check_pairKey,     // both shape indices of every pair key in range
    m2_check_tree,        // the inline tree headers
    m2_check_treeNode,    // tree node links and leaf payloads
    m2_check_finite32,    // every float of every element finite
    m2_check_finite64,    // every double of every element finite
    m2_check_local,       // every float of every element within M2_LOCAL_LIMIT
    m2_check_transform,   // a finite double position and a unit float rotation
    // The ranges of base.h: every float of every element passes the
    // validator the defs, the setters and replay use for its quantity.
    m2_check_speed,        // m2ValidSpeed: velocities per axis, spins, speeds
    m2_check_acceleration, // m2ValidAcceleration, per axis
    m2_check_gravityScale, // m2ValidGravityScale
    m2_check_rate,         // m2ValidRate: damping, drag, a spring's hertz
    m2_check_coefficient,  // m2ValidCoefficient: friction, a damping ratio
    m2_check_restitution,  // m2ValidRestitution
    m2_check_density,      // m2ValidDensity
    m2_check_force,        // m2ValidForce: forces per axis, torques
    m2_check_maxForce,     // m2ValidMaxForce: a motor's budget
    m2_check_impulse,      // m2ValidImpulse: accumulated joint impulses
    m2_check_breakLimit,   // m2ValidBreakLimit
    m2_check_inverseStep,  // the last step's 1 / substep (InverseStepValid)
} m2Check;

// The pools an array's values may index into.
typedef struct m2StateBounds
{
    int64_t range;  // the pool the checked values index into, or count up to
    int64_t shapes; // the shape pool, which tree leaves name
} m2StateBounds;

// Checks count elements of elementSize bytes (an inline field: one element
// of the field's size) against one rule.
bool m2StateCheckElements(const uint8_t* data, size_t count, size_t elementSize, m2Check check,
                          m2StateBounds bounds);

#endif // MAUL2D_SRC_STATE_CHECKS_H
