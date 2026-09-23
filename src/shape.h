// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for shape.c.

#ifndef MAUL2D_SRC_SHAPE_H
#define MAUL2D_SRC_SHAPE_H

#include "world_internal.h"

m2ShapeId m2MakeShapeId(const m2World* world, int32_t shapeIndex);
void m2RetireShapeFromBroadphase(m2World* world, int32_t shapeIndex);
void m2DestroyShapeInternal(m2World* world, int32_t shapeIndex);
m2ShapeId m2CreateShape(m2BodyId bodyId, const m2ShapeDef* def, const m2ShapeGeometry* geometry);

// A def is valid only when its internalValue matches its cookie.
#define M2_SHAPE_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2ShapeDef) << 8) ^ 3)

// The journaled parameter channel (codes in journal.h).
bool m2SetShapeParamInternal(m2World* world, m2ShapeId shapeId, uint8_t param, float value);

#endif // MAUL2D_SRC_SHAPE_H
