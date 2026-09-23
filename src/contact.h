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
void m2EmitSensorBegin(m2World* world, int32_t shapeA, int32_t shapeB, int32_t pairIndex);
void m2StashContacts(m2World* world);
void m2UpdateContacts(m2World* world);

#endif // MAUL2D_SRC_CONTACT_H
