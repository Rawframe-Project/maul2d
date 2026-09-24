// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase tree: a bounding volume hierarchy over fat leaf boxes,
// kept height balanced as an AVL tree so a query stack of a fixed size
// always suffices.
//
// Insertion descends from the root toward the child whose box grows
// least by the new leaf, pairs the leaf with the node on that path that
// adds the least total box area to the tree, then walks back up refitting
// boxes and restoring the AVL height rule with single and double
// rotations.
// A move detaches a leaf and reattaches it through the same descent,
// reusing its node and its old parent node, so a proxy id never changes
// while the shape lives.
//
// Every decision is a comparison of f64 sums and products, so the tree
// shape is a pure function of the operation history.

#include "dynamic_tree.h"

#include "core.h"

#include "maul2d/base.h"

static m2Aabb Union(m2Aabb a, m2Aabb b)
{
    m2Aabb c;
    c.lowerBound.x = a.lowerBound.x < b.lowerBound.x ? a.lowerBound.x : b.lowerBound.x;
    c.lowerBound.y = a.lowerBound.y < b.lowerBound.y ? a.lowerBound.y : b.lowerBound.y;
    c.upperBound.x = a.upperBound.x > b.upperBound.x ? a.upperBound.x : b.upperBound.x;
    c.upperBound.y = a.upperBound.y > b.upperBound.y ? a.upperBound.y : b.upperBound.y;
    return c;
}

// Half the perimeter: in 2D the surface area heuristic weighs a box by
// its perimeter, and the factor two never changes a comparison.
static double HalfPerimeter(m2Aabb box)
{
    return (box.upperBound.x - box.lowerBound.x) + (box.upperBound.y - box.lowerBound.y);
}

static int32_t MaxHeight(const m2TreeNode* nodes, int32_t a, int32_t b)
{
    return nodes[a].height > nodes[b].height ? nodes[a].height : nodes[b].height;
}

void m2TreeInit(m2DynamicTree* tree, m2TreeNode* nodes, int32_t nodeCapacity)
{
    tree->root = M2_NULL_NODE;
    tree->nodeCount = 0;
    tree->nodeCapacity = nodeCapacity;
    tree->freeList = nodeCapacity > 0 ? 0 : M2_NULL_NODE;
    for (int32_t i = 0; i < nodeCapacity; ++i)
    {
        nodes[i] = (m2TreeNode){0};
        nodes[i].parent = i + 1 < nodeCapacity ? i + 1 : M2_NULL_NODE;
        nodes[i].height = -1;
    }
}

static int32_t TakeNode(m2DynamicTree* tree, m2TreeNode* nodes)
{
    int32_t node = tree->freeList;
    if (node == M2_NULL_NODE)
    {
        return M2_NULL_NODE;
    }
    tree->freeList = nodes[node].parent;
    tree->nodeCount += 1;
    nodes[node] = (m2TreeNode){0};
    nodes[node].parent = M2_NULL_NODE;
    nodes[node].child1 = M2_NULL_NODE;
    nodes[node].child2 = M2_NULL_NODE;
    nodes[node].userData = -1;
    return node;
}

static void GiveNode(m2DynamicTree* tree, m2TreeNode* nodes, int32_t node)
{
    nodes[node] = (m2TreeNode){0};
    nodes[node].parent = tree->freeList;
    nodes[node].height = -1;
    tree->freeList = node;
    tree->nodeCount -= 1;
}

// Points the parent of old (or the root) at replacement.
static void Relink(m2DynamicTree* tree, m2TreeNode* nodes, int32_t parent, int32_t old,
                   int32_t replacement)
{
    nodes[replacement].parent = parent;
    if (parent == M2_NULL_NODE)
    {
        tree->root = replacement;
    }
    else if (nodes[parent].child1 == old)
    {
        nodes[parent].child1 = replacement;
    }
    else
    {
        nodes[parent].child2 = replacement;
    }
}

static void Refit(m2TreeNode* nodes, int32_t node)
{
    int32_t a = nodes[node].child1;
    int32_t b = nodes[node].child2;
    nodes[node].aabb = Union(nodes[a].aabb, nodes[b].aabb);
    nodes[node].height = 1 + MaxHeight(nodes, a, b);
}

// Lifts the child on the given side (1 or 2) of top into its place. The
// lifted node keeps its own outer child and adopts top on the vacated
// side; top adopts the lifted node's inner child. Returns the new root
// of the subtree.
static int32_t Rotate(m2DynamicTree* tree, m2TreeNode* nodes, int32_t top, int32_t side)
{
    int32_t up = side == 1 ? nodes[top].child1 : nodes[top].child2;
    int32_t inner = side == 1 ? nodes[up].child2 : nodes[up].child1;
    Relink(tree, nodes, nodes[top].parent, top, up);
    if (side == 1)
    {
        nodes[top].child1 = inner;
        nodes[up].child2 = top;
    }
    else
    {
        nodes[top].child2 = inner;
        nodes[up].child1 = top;
    }
    nodes[inner].parent = top;
    nodes[top].parent = up;
    Refit(nodes, top);
    Refit(nodes, up);
    return up;
}

// Restores the AVL rule at node, whose two children are balanced subtrees
// of any heights. The taller child rises; when its inner grandchild is
// the taller one, that grandchild rises first so the rotation cannot
// leave the imbalance on the other side. When the heights were more than
// two apart the demoted node can still lean, so it is balanced in turn
// and the new top checked again: in effect an AVL join, which lets an
// insertion pair a leaf with a subtree of any height. Returns the root
// of the balanced subtree.
static int32_t Rebalance(m2DynamicTree* tree, m2TreeNode* nodes, int32_t node)
{
    for (;;)
    {
        int32_t a = nodes[node].child1;
        int32_t b = nodes[node].child2;
        int32_t skew = nodes[b].height - nodes[a].height;
        if (skew >= -1 && skew <= 1)
        {
            return node;
        }
        int32_t side = skew > 0 ? 2 : 1;
        int32_t tall = side == 1 ? a : b;
        int32_t outer = side == 1 ? nodes[tall].child1 : nodes[tall].child2;
        int32_t inner = side == 1 ? nodes[tall].child2 : nodes[tall].child1;
        if (nodes[inner].height > nodes[outer].height)
        {
            Rotate(tree, nodes, tall, side == 1 ? 2 : 1);
        }
        int32_t up = Rotate(tree, nodes, node, side);
        Rebalance(tree, nodes, node);
        Refit(nodes, up);
        node = up;
    }
}

// Refits boxes and heights from node up to the root, rebalancing on the
// way.
static void RepairUpward(m2DynamicTree* tree, m2TreeNode* nodes, int32_t node)
{
    while (node != M2_NULL_NODE)
    {
        Refit(nodes, node);
        node = Rebalance(tree, nodes, node);
        node = nodes[node].parent;
    }
}

// The node a new box becomes the sibling of. Pairing box with node X adds
// a junction of area |X u box| and grows every ancestor A of X by
// |A u box| - |A|, so the added area of the whole tree is
//
//     cost(X) = |X u box| + (growth of the ancestors of X).
//
// The descent follows the child that grows least (ties to the smaller
// union, then to the first child) and keeps the cheapest node it meets.
// It stops once the growth inherited so far plus |box|, the least any
// deeper junction can cost, reaches the best cost found.
static int32_t PickSibling(const m2DynamicTree* tree, const m2TreeNode* nodes, m2Aabb box)
{
    double boxArea = HalfPerimeter(box);
    int32_t node = tree->root;
    int32_t best = node;
    double bestCost = HalfPerimeter(Union(nodes[node].aabb, box));
    double inherited = 0.0;
    while (nodes[node].height > 0)
    {
        inherited += HalfPerimeter(Union(nodes[node].aabb, box)) - HalfPerimeter(nodes[node].aabb);
        if (inherited + boxArea >= bestCost)
        {
            break;
        }
        int32_t a = nodes[node].child1;
        int32_t b = nodes[node].child2;
        double areaA = HalfPerimeter(Union(nodes[a].aabb, box));
        double areaB = HalfPerimeter(Union(nodes[b].aabb, box));
        double growA = areaA - HalfPerimeter(nodes[a].aabb);
        double growB = areaB - HalfPerimeter(nodes[b].aabb);
        bool takeB = growB < growA || (growB == growA && areaB < areaA);
        node = takeB ? b : a;
        double cost = inherited + (takeB ? areaB : areaA);
        if (cost < bestCost)
        {
            best = node;
            bestCost = cost;
        }
    }
    return best;
}

// Hangs leaf in the tree under junction, a free interior node.
static void Attach(m2DynamicTree* tree, m2TreeNode* nodes, int32_t leaf, int32_t junction)
{
    if (tree->root == M2_NULL_NODE)
    {
        tree->root = leaf;
        nodes[leaf].parent = M2_NULL_NODE;
        return;
    }
    int32_t sibling = PickSibling(tree, nodes, nodes[leaf].aabb);
    Relink(tree, nodes, nodes[sibling].parent, sibling, junction);
    nodes[junction].child1 = sibling;
    nodes[junction].child2 = leaf;
    nodes[sibling].parent = junction;
    nodes[leaf].parent = junction;
    RepairUpward(tree, nodes, junction);
}

// Takes leaf out of the tree and returns its old parent node, which is
// no longer linked (M2_NULL_NODE when the leaf was the root).
static int32_t Detach(m2DynamicTree* tree, m2TreeNode* nodes, int32_t leaf)
{
    int32_t parent = nodes[leaf].parent;
    nodes[leaf].parent = M2_NULL_NODE;
    if (parent == M2_NULL_NODE)
    {
        tree->root = M2_NULL_NODE;
        return M2_NULL_NODE;
    }
    int32_t sibling = nodes[parent].child1 == leaf ? nodes[parent].child2 : nodes[parent].child1;
    int32_t grandParent = nodes[parent].parent;
    Relink(tree, nodes, grandParent, parent, sibling);
    RepairUpward(tree, nodes, grandParent);
    return parent;
}

int32_t m2TreeInsert(m2DynamicTree* tree, m2TreeNode* nodes, m2Aabb aabb, int32_t userData)
{
    // A non-empty tree needs a junction node besides the leaf; both are
    // taken up front so a full pool refuses with the tree untouched.
    bool empty = tree->root == M2_NULL_NODE;
    if (tree->nodeCount + (empty ? 1 : 2) > tree->nodeCapacity)
    {
        return M2_NULL_NODE;
    }
    int32_t leaf = TakeNode(tree, nodes);
    nodes[leaf].aabb = aabb;
    nodes[leaf].userData = userData;
    nodes[leaf].height = 0;
    int32_t junction = empty ? M2_NULL_NODE : TakeNode(tree, nodes);
    Attach(tree, nodes, leaf, junction);
    return leaf;
}

void m2TreeRemove(m2DynamicTree* tree, m2TreeNode* nodes, int32_t proxy)
{
    M2_ASSERT(proxy >= 0 && proxy < tree->nodeCapacity && nodes[proxy].height == 0);
    int32_t junction = Detach(tree, nodes, proxy);
    if (junction != M2_NULL_NODE)
    {
        GiveNode(tree, nodes, junction);
    }
    GiveNode(tree, nodes, proxy);
}

void m2TreeMove(m2DynamicTree* tree, m2TreeNode* nodes, int32_t proxy, m2Aabb aabb)
{
    int32_t junction = Detach(tree, nodes, proxy);
    nodes[proxy].aabb = aabb;
    Attach(tree, nodes, proxy, junction);
}

int32_t m2TreeQuery(const m2DynamicTree* tree, const m2TreeNode* nodes, m2Aabb aabb,
                    int32_t* results, int32_t resultCapacity)
{
    m2TreeCursor cursor;
    m2TreeBeginQuery(&cursor, tree, nodes, aabb);
    int32_t count = 0;
    int32_t userData;
    while (m2TreeNextQuery(&cursor, &userData))
    {
        if (count < resultCapacity)
        {
            results[count] = userData;
        }
        count += 1;
    }
    return count;
}

void m2TreeBeginQuery(m2TreeCursor* cursor, const m2DynamicTree* tree, const m2TreeNode* nodes,
                      m2Aabb aabb)
{
    cursor->nodes = nodes;
    cursor->aabb = aabb;
    cursor->top = 0;
    if (tree->root != M2_NULL_NODE)
    {
        cursor->stack[cursor->top++] = tree->root;
    }
}

// Depth first, first child first. The stack holds at most one pending
// sibling per level plus the node in hand, and an AVL tree over any
// int32 node count is under 64 levels deep.
bool m2TreeNextQuery(m2TreeCursor* cursor, int32_t* userData)
{
    while (cursor->top > 0)
    {
        const m2TreeNode* node = cursor->nodes + cursor->stack[--cursor->top];
        if (!m2Aabb_Overlaps(node->aabb, cursor->aabb))
        {
            continue;
        }
        if (node->height == 0)
        {
            *userData = node->userData;
            return true;
        }
        M2_ASSERT(cursor->top + 2 <= M2_TREE_STACK_CAPACITY);
        cursor->stack[cursor->top++] = node->child2;
        cursor->stack[cursor->top++] = node->child1;
    }
    return false;
}

// Checks one subtree and returns its leaf count, or -1 on any breach:
// links both ways, the height rule, AVL balance and box containment.
static int32_t CheckSubtree(const m2DynamicTree* tree, const m2TreeNode* nodes, int32_t node)
{
    const m2TreeNode* n = nodes + node;
    if (n->height == 0)
    {
        return n->child1 == M2_NULL_NODE && n->child2 == M2_NULL_NODE ? 1 : -1;
    }
    int32_t a = n->child1;
    int32_t b = n->child2;
    if (a < 0 || a >= tree->nodeCapacity || b < 0 || b >= tree->nodeCapacity ||
        nodes[a].parent != node || nodes[b].parent != node ||
        n->height != 1 + MaxHeight(nodes, a, b) || nodes[a].height - nodes[b].height > 1 ||
        nodes[b].height - nodes[a].height > 1 || !m2Aabb_Contains(n->aabb, nodes[a].aabb) ||
        !m2Aabb_Contains(n->aabb, nodes[b].aabb))
    {
        return -1;
    }
    int32_t leavesA = CheckSubtree(tree, nodes, a);
    int32_t leavesB = CheckSubtree(tree, nodes, b);
    return leavesA < 0 || leavesB < 0 ? -1 : leavesA + leavesB;
}

bool m2TreeValidate(const m2DynamicTree* tree, const m2TreeNode* nodes)
{
    if (tree->root == M2_NULL_NODE)
    {
        return tree->nodeCount == 0;
    }
    if (nodes[tree->root].parent != M2_NULL_NODE)
    {
        return false;
    }
    int32_t leaves = CheckSubtree(tree, nodes, tree->root);
    return leaves > 0 && tree->nodeCount == 2 * leaves - 1;
}
