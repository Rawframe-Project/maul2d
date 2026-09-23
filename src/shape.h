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

#endif // MAUL2D_SRC_SHAPE_H
