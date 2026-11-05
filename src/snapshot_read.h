// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Reads of an incoming snapshot block for its structural check. The bytes
// may be unaligned, so every read goes through memcpy.

#ifndef MAUL2D_SRC_SNAPSHOT_READ_H
#define MAUL2D_SRC_SNAPSHOT_READ_H

#include "dynamic_tree.h"
#include "world_internal.h"
#include "world_state.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

M2_NODISCARD static inline bool InPool(int32_t index, int32_t capacity)
{
    return index >= 0 && index < capacity;
}

static inline int32_t ReadI32(const uint8_t* array, int32_t index)
{
    int32_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static inline float ReadF32(const uint8_t* array, int32_t index)
{
    float value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static inline uint32_t ReadU32(const uint8_t* array, int32_t index)
{
    uint32_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static inline uint64_t ReadU64(const uint8_t* array, int32_t index)
{
    uint64_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static inline m2TreeNode ReadNode(const uint8_t* nodes, int32_t index)
{
    m2TreeNode node;
    memcpy(&node, nodes + (size_t)index * sizeof(node), sizeof(node));
    return node;
}

static inline int32_t InlineI32(const m2World* world, const uint8_t* in, const void* field)
{
    return ReadI32(m2StateIncoming(world, in, field), 0);
}

// The incoming copy of a field, in a function that names the world
// `world` and the block `in`; every field the checks read is snapshot
// state in every world that has its pool. Each file that uses it undefines
// it at its end.
#define M2_IN(field) m2StateIncoming(world, in, (const void*)&world->field)

#endif // MAUL2D_SRC_SNAPSHOT_READ_H
