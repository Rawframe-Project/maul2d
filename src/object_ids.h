// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The translation between public body and shape ids and their slots.

#ifndef MAUL2D_SRC_OBJECT_IDS_H
#define MAUL2D_SRC_OBJECT_IDS_H

#include "world_internal.h"

// The slot of a live body, or -1 for a stale or foreign id.
int32_t m2BodySlot(const m2World* world, m2BodyId id);

// The slot of a live shape, or -1 for a stale or foreign id.
int32_t m2ShapeSlot(const m2World* world, m2ShapeId id);

// The public id of a shape slot.
m2ShapeId m2MakeShapeId(const m2World* world, int32_t shapeIndex);

// The public ids of a sensor pair's shapes: the sensor and the shape
// inside it. Two sensors never pair, so exactly one of the two is a sensor.
void m2SensorPairIds(const m2World* world, int32_t shapeA, int32_t shapeB, m2ShapeId* sensorOut,
                     m2ShapeId* visitorOut);

#endif // MAUL2D_SRC_OBJECT_IDS_H
