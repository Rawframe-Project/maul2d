// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase tree over fat leaf boxes: an index-based node pool,
// AVL balanced. Every field is POD and snapshot state: the tree shape
// depends on the operation history, so rollback restores these arrays
// byte for byte instead of rebuilding them.

#ifndef MAUL2D_SRC_DYNAMIC_TREE_H
#define MAUL2D_SRC_DYNAMIC_TREE_H

#include "maul2d/core_math.h"

#include <stdbool.h>

#define M2_NULL_NODE (-1)

// World-space AABB in f64.
typedef struct m2AABB
{
    m2Pos2 lowerBound;
    m2Pos2 upperBound;
} m2AABB;

typedef struct m2TreeNode
{
    m2AABB aabb; // fat for leaves, union for interior nodes
    int32_t child1;
    int32_t child2;
    int32_t parent;   // the next free node while the node is free
    int32_t height;   // leaf = 0, free = -1
    int32_t userData; // leaf payload: the shape index; -1 on interior nodes
    int32_t pad;      // zero; keeps sizeof equal to the sum of the members
} m2TreeNode;

_Static_assert(sizeof(m2AABB) == 32, "m2AABB must be padding-free");
_Static_assert(sizeof(m2TreeNode) == 56, "m2TreeNode must be padding-free");

// POD scalars + caller-owned node array: the world snapshots this struct
// and the node block byte for byte.
typedef struct m2DynamicTree
{
    int32_t root;
    int32_t nodeCount;
    int32_t nodeCapacity;
    int32_t freeList;
} m2DynamicTree;

// Nodes are allocated by the world (snapshot block); the tree functions
// take (tree, nodes) explicitly so the struct stays flat POD.
void m2TreeInit(m2DynamicTree* tree, m2TreeNode* nodes, int32_t nodeCapacity);

// Returns the new proxy (node) index, or M2_NULL_NODE when the pool is
// exhausted. The caller owns fattening: aabb passed here is stored as-is.
int32_t m2TreeInsert(m2DynamicTree* tree, m2TreeNode* nodes, m2AABB aabb, int32_t userData);

void m2TreeRemove(m2DynamicTree* tree, m2TreeNode* nodes, int32_t proxy);

// Moves a leaf to a new box (the caller decided the tight box escaped
// the fat one). The proxy index stays the same.
void m2TreeMove(m2DynamicTree* tree, m2TreeNode* nodes, int32_t proxy, m2AABB aabb);

// Collect userData of every leaf overlapping the query AABB, in an order
// determined solely by tree shape (deterministic given identical state).
// Returns the hit count; writes at most resultCapacity entries.
int32_t m2TreeQuery(const m2DynamicTree* tree, const m2TreeNode* nodes, m2AABB aabb,
                    int32_t* results, int32_t resultCapacity);

// Incremental form of m2TreeQuery for callers that must not share a
// result buffer: the cursor lives on the caller's stack and yields the
// same leaves in the same order as m2TreeQuery.
#define M2_TREE_STACK_CAPACITY 256
typedef struct m2TreeCursor
{
    const m2TreeNode* nodes;
    m2AABB aabb;
    int32_t top;
    int32_t stack[M2_TREE_STACK_CAPACITY];
} m2TreeCursor;

void m2TreeBeginQuery(m2TreeCursor* cursor, const m2DynamicTree* tree, const m2TreeNode* nodes,
                      m2AABB aabb);
// Writes the next overlapping leaf's userData and returns true, or
// returns false when the query is exhausted.
bool m2TreeNextQuery(m2TreeCursor* cursor, int32_t* userData);

// Test oracle: links both ways, heights, AVL balance, box containment
// and the node count. Returns false on any breach.
bool m2TreeValidate(const m2DynamicTree* tree, const m2TreeNode* nodes);

static inline bool m2AABB_Overlaps(m2AABB a, m2AABB b)
{
    return a.lowerBound.x <= b.upperBound.x && a.lowerBound.y <= b.upperBound.y &&
           b.lowerBound.x <= a.upperBound.x && b.lowerBound.y <= a.upperBound.y;
}

static inline bool m2AABB_Contains(m2AABB outer, m2AABB inner)
{
    return outer.lowerBound.x <= inner.lowerBound.x && outer.lowerBound.y <= inner.lowerBound.y &&
           inner.upperBound.x <= outer.upperBound.x && inner.upperBound.y <= outer.upperBound.y;
}

#endif // MAUL2D_SRC_DYNAMIC_TREE_H
