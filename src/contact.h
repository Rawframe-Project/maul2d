// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for contact.c.

#ifndef MAUL2D_SRC_CONTACT_H
#define MAUL2D_SRC_CONTACT_H

#include "world_internal.h"

void m2EmitEnd(m2World* world, int32_t shapeA, int32_t shapeB);
void m2EmitBegin(m2World* world, int32_t shapeA, int32_t shapeB, int32_t pairIndex);
void m2EmitSensorEnd(m2World* world, int32_t shapeA, int32_t shapeB);
void m2EmitSensorBegin(m2World* world, int32_t shapeA, int32_t shapeB);
void m2StashContacts(m2World* world);

// Relative pose of body B in body A's frame: the single f64 -> f32
// crossing for the contact stage.
m2RelativePose m2MakeRelativePose(m2Transform xfA, m2Transform xfB);

// The manifold of geometry ga, in a pair's lower slot, and gb, posed by
// `pose` in ga's body frame: what the step computes for the pair before
// the impulses carry over.
m2Manifold m2ComputeManifold(const m2ShapeGeometry* ga, const m2ShapeGeometry* gb,
                             m2RelativePose pose);
void m2UpdateContacts(m2World* world);

#endif // MAUL2D_SRC_CONTACT_H
