// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hostile snapshots that pass every per-array check but break how the
// arrays fit together: a free queue that hands a slot out twice, a shape
// list that loops, owners that are dead, broadphase trees with broken
// links, a moved list out of step with its flags, pairs out of order. Each
// edit is made at the field's place in a real snapshot, found through the
// state table, and each restore must refuse it, change nothing and count
// no misuse. fuzz_snapshot found these as heap overflows, stack overflows
// and invalid worlds.

#include "dynamic_tree.h"
#include "geometry.h"
#include "helpers.h"
#include "snapshot_check.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul2d/maul2d.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct Scene
{
    m2World* world;
    uint8_t* valid;
    int32_t size;
    int32_t headerBytes;
    m2BodyId ground;
    m2BodyId boxA;
    m2BodyId boxB;
    m2BodyId lift; // alone in the kinematic tree
} Scene;

// Three bodies, shapes on each (a chain on the ground), a joint and a few
// particles, stepped so pairs and trees hold state.
static Scene MakeScene(void)
{
    Scene scene;
    memset(&scene, 0, sizeof(scene));
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    def.jointCapacity = 4;
    def.particleCapacity = 8;
    scene.world = NewWorld(&def);
    m2BodyDef groundDef = m2DefaultBodyDef();
    scene.ground = NewBody(scene.world, &groundDef);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(scene.world, scene.ground, &shapeDef, &slab);
    m2Vec2 ridge[4] = {{-6.0f, 2.0f}, {-5.0f, 0.5f}, {-4.0f, 0.5f}, {-3.0f, 2.0f}};
    m2ChainDef chainDef = m2DefaultChainDef();
    chainDef.points = ridge;
    chainDef.count = 4;
    NewChain(scene.world, scene.ground, &chainDef);
    m2BodyDef boxDef = m2DefaultBodyDef();
    boxDef.type = m2_dynamicBody;
    boxDef.position = (m2Pos2){0.0, 1.0};
    scene.boxA = NewBody(scene.world, &boxDef);
    boxDef.position = (m2Pos2){0.5, 2.0};
    scene.boxB = NewBody(scene.world, &boxDef);
    m2Polygon box = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(scene.world, scene.boxA, &shapeDef, &box);
    NewPolygonShape(scene.world, scene.boxB, &shapeDef, &box);
    boxDef.type = m2_kinematicBody;
    boxDef.position = (m2Pos2){8.0, 1.0};
    scene.lift = NewBody(scene.world, &boxDef);
    NewPolygonShape(scene.world, scene.lift, &shapeDef, &box);
    m2RevoluteJointDef pin = m2DefaultRevoluteJointDef();
    pin.bodyIdA = scene.boxA;
    pin.bodyIdB = scene.boxB;
    NewRevoluteJoint(scene.world, &pin);
    for (int32_t i = 0; i < 3; ++i)
    {
        EmitParticle(scene.world, (m2Pos2){3.0 + 0.1 * (double)i, 0.6}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(scene.world, 1.0f / 60.0f, 4);
    }
    scene.size = m2World_SnapshotSize(scene.world);
    scene.valid = (uint8_t*)malloc((size_t)scene.size);
    CHECK(scene.valid != NULL && Snapshot(scene.world, scene.valid, scene.size) == scene.size,
          "the scene snapshots");
    scene.headerBytes = scene.size - m2StateWalk(scene.world, NULL, NULL, 2);
    return scene;
}

static void FreeScene(Scene* scene)
{
    free(scene->valid);
    m2DestroyWorld(scene->world);
}

// --- Reading and writing fields in a snapshot block -------------------------

static uint8_t* At(const Scene* scene, uint8_t* block, const void* field)
{
    return (uint8_t*)(uintptr_t)m2StateIncoming(scene->world, block, field);
}

static int32_t GetI32(const uint8_t* array, int32_t index)
{
    int32_t value;
    memcpy(&value, array + (size_t)index * sizeof(value), sizeof(value));
    return value;
}

static void PutI32(uint8_t* array, int32_t index, int32_t value)
{
    memcpy(array + (size_t)index * sizeof(value), &value, sizeof(value));
}

static m2TreeNode GetNode(const uint8_t* nodes, int32_t index)
{
    m2TreeNode node;
    memcpy(&node, nodes + (size_t)index * sizeof(node), sizeof(node));
    return node;
}

static void PutNode(uint8_t* nodes, int32_t index, m2TreeNode node)
{
    memcpy(nodes + (size_t)index * sizeof(node), &node, sizeof(node));
}

typedef void (*Edit)(const Scene* scene, uint8_t* block);

static void ExpectRefused(const Scene* scene, Edit edit, const char* what)
{
    uint8_t* bytes = (uint8_t*)malloc((size_t)scene->size);
    CHECK(bytes != NULL, "a copy of the snapshot");
    if (bytes == NULL)
    {
        return;
    }
    memcpy(bytes, scene->valid, (size_t)scene->size);
    edit(scene, bytes + scene->headerBytes);
    uint64_t hash = m2World_Hash(scene->world);
    uint64_t misuse = m2World_GetCounters(scene->world).misuse;
    m2Result result = m2World_Restore(scene->world, bytes, scene->size);
    CHECK(result == m2_errorInvalid, what);
    CHECK(m2World_Hash(scene->world) == hash && m2World_GetCounters(scene->world).misuse == misuse,
          "a refused snapshot changes nothing and counts nothing");
    free(bytes);
}

// --- The edits ----------------------------------------------------------------

// The body queue hands its head slot out twice: [3, 3, ...].
static void QueueRepeatsASlot(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    uint8_t* queue = At(scene, block, &w->bodies.freeQueue);
    int32_t head = GetI32(At(scene, block, &w->bodies.freeHead), 0);
    PutI32(queue, (head + 1) % w->bodies.bodyCapacity, GetI32(queue, head));
}

// The body queue hands out a live body.
static void QueueHoldsALiveSlot(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    int32_t head = GetI32(At(scene, block, &w->bodies.freeHead), 0);
    PutI32(At(scene, block, &w->bodies.freeQueue), head, (int32_t)scene->boxA.index1 - 1);
}

// The particle queue hands its head slot out twice.
static void ParticleQueueRepeatsASlot(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    uint8_t* queue = At(scene, block, &w->particles.particleFreeQueue);
    int32_t head = GetI32(At(scene, block, &w->particles.particleFreeHead), 0);
    PutI32(queue, (head + 1) % w->particles.particleCapacity, GetI32(queue, head));
}

// A body's shape list loops back onto its first shape.
static void ShapeListLoops(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    int32_t first = GetI32(At(scene, block, &w->bodies.bodyShapeHead), 0);
    uint8_t* next = At(scene, block, &w->shapes.shapeNext);
    int32_t last = first;
    while (GetI32(next, last) != -1)
    {
        last = GetI32(next, last);
    }
    PutI32(next, last, first);
}

// A listed shape names another body as its owner.
static void ShapeNamesAnotherBody(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    int32_t first = GetI32(At(scene, block, &w->bodies.bodyShapeHead), 0);
    PutI32(At(scene, block, &w->shapes.shapeBody), first, (int32_t)scene->boxA.index1 - 1);
}

// The chain belongs to a body slot no body holds.
static void ChainOnADeadBody(const Scene* scene, uint8_t* block)
{
    PutI32(At(scene, block, &scene->world->chains.chainBody), 0, 7);
}

// The joint's second body is a slot no body holds.
static void JointToADeadBody(const Scene* scene, uint8_t* block)
{
    PutI32(At(scene, block, &scene->world->joints.jointBodyB), 0, 7);
}

// A body's center of mass moves far past M2_HUGE.
static void BodyCenterTooFar(const Scene* scene, uint8_t* block)
{
    m2Vec2 center = {1e30f, 0.0f};
    uint8_t* centers = At(scene, block, &scene->world->bodies.localCenters);
    memcpy(centers + (size_t)(scene->boxA.index1 - 1) * sizeof(center), &center, sizeof(center));
}

// The pin's anchor on its first body moves past M2_HUGE.
static void JointAnchorTooFar(const Scene* scene, uint8_t* block)
{
    m2Vec2 anchor = {2.0f * M2_HUGE, 0.0f};
    memcpy(At(scene, block, &scene->world->joints.jointLocalAnchorA), &anchor, sizeof(anchor));
}

// The dynamic tree's root, an interior node, loses its first child.
static uint8_t* DynamicTreeNodes(const Scene* scene, uint8_t* block, int32_t* root)
{
    const m2World* w = scene->world;
    m2DynamicTree tree;
    memcpy(&tree, At(scene, block, &w->broadphase.trees) + sizeof(tree) * m2_dynamicBody,
           sizeof(tree));
    *root = tree.root;
    return At(scene, block, &w->broadphase.treeNodes[m2_dynamicBody]);
}

static void TreeChildDisownsParent(const Scene* scene, uint8_t* block)
{
    int32_t root = 0;
    uint8_t* nodes = DynamicTreeNodes(scene, block, &root);
    m2TreeNode top = GetNode(nodes, root);
    m2TreeNode child = GetNode(nodes, top.child1);
    child.parent = top.child2;
    PutNode(nodes, top.child1, child);
}

// The root's second child points back at the root: a cycle.
static void TreeLoopsToRoot(const Scene* scene, uint8_t* block)
{
    int32_t root = 0;
    uint8_t* nodes = DynamicTreeNodes(scene, block, &root);
    m2TreeNode top = GetNode(nodes, root);
    top.child2 = root;
    PutNode(nodes, root, top);
}

// The root claims a height its children do not give it.
static void TreeHeightLies(const Scene* scene, uint8_t* block)
{
    int32_t root = 0;
    uint8_t* nodes = DynamicTreeNodes(scene, block, &root);
    m2TreeNode top = GetNode(nodes, root);
    top.height += 5;
    PutNode(nodes, root, top);
}

// The root's box no longer contains its first child's.
static void TreeBoxShrinks(const Scene* scene, uint8_t* block)
{
    int32_t root = 0;
    uint8_t* nodes = DynamicTreeNodes(scene, block, &root);
    m2TreeNode top = GetNode(nodes, root);
    top.aabb.upperBound = top.aabb.lowerBound;
    PutNode(nodes, root, top);
}

// The free list is dropped: its nodes belong nowhere.
static void TreeLosesFreeNodes(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    uint8_t* headers = At(scene, block, &w->broadphase.trees);
    m2DynamicTree tree;
    memcpy(&tree, headers + sizeof(tree) * m2_dynamicBody, sizeof(tree));
    tree.freeList = M2_NULL_NODE;
    memcpy(headers + sizeof(tree) * m2_dynamicBody, &tree, sizeof(tree));
}

// Two shapes swap proxies: each leaf names the other shape.
static void ProxiesSwap(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    uint8_t* proxies = At(scene, block, &w->broadphase.proxyIds);
    uint8_t* heads = At(scene, block, &w->bodies.bodyShapeHead);
    int32_t a = GetI32(heads, (int32_t)scene->boxA.index1 - 1);
    int32_t b = GetI32(heads, (int32_t)scene->boxB.index1 - 1);
    int32_t proxyA = GetI32(proxies, a);
    PutI32(proxies, a, GetI32(proxies, b));
    PutI32(proxies, b, proxyA);
}

// The moved list holds one shape twice.
static void MovedRepeatsAShape(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    PutI32(At(scene, block, &w->broadphase.movedCount), 0, 2);
    PutI32(At(scene, block, &w->broadphase.moved), 0, 0);
    PutI32(At(scene, block, &w->broadphase.moved), 1, 0);
    At(scene, block, &w->broadphase.inMoved)[0] = 1;
}

// The moved list holds a shape its flag says is not there.
static void MovedFlagMissing(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    PutI32(At(scene, block, &w->broadphase.movedCount), 0, 1);
    PutI32(At(scene, block, &w->broadphase.moved), 0, 0);
    At(scene, block, &w->broadphase.inMoved)[0] = 0;
}

// The first pair names a shape slot no shape holds.
static void PairNamesADeadShape(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    uint64_t key = ((uint64_t)0 << 32) | (uint64_t)(w->shapes.shapeCapacity - 1);
    PutI32(At(scene, block, &w->contacts.pairCount), 0, 1);
    memcpy(At(scene, block, &w->contacts.pairKeys), &key, sizeof(key));
}

// A geometry edit on the first shape of a body, through the stored union.
static void EditGeometry(const Scene* scene, uint8_t* block, m2BodyId owner,
                         void (*change)(m2ShapeGeometry* geometry))
{
    const m2World* w = scene->world;
    int32_t s = GetI32(At(scene, block, &w->bodies.bodyShapeHead), (int32_t)owner.index1 - 1);
    uint8_t* geometries = At(scene, block, &w->shapes.shapeGeometry);
    m2ShapeGeometry geometry;
    memcpy(&geometry, geometries + (size_t)s * sizeof(geometry), sizeof(geometry));
    change(&geometry);
    memcpy(geometries + (size_t)s * sizeof(geometry), &geometry, sizeof(geometry));
}

static void TurnFirstNormal(m2ShapeGeometry* geometry)
{
    geometry->polygon.normals[0] = (m2Vec2){0.0f, 1.0f};
}

static void PolygonNormalLies(const Scene* scene, uint8_t* block)
{
    EditGeometry(scene, block, scene->boxA, TurnFirstNormal);
}

static void UnfinishedGhost(m2ShapeGeometry* geometry)
{
    if (geometry->type == m2_chainSegmentShape)
    {
        geometry->chainSegment.ghost1.x = NAN;
    }
    else if (geometry->type == m2_polygonShape)
    {
        geometry->polygon.radius = -1.0f; // the ground's slab, when it heads the list
    }
}

static void GroundGeometryBroken(const Scene* scene, uint8_t* block)
{
    EditGeometry(scene, block, scene->ground, UnfinishedGhost);
}

// The structural check stands on its own: these edits are refused by it
// alone, without the per-array checks that also catch them.
static void BodyTailNegative(const Scene* scene, uint8_t* block)
{
    PutI32(At(scene, block, &scene->world->bodies.freeTail), 0, -1);
}

static void BodyHighWaterPastCapacity(const Scene* scene, uint8_t* block)
{
    PutI32(At(scene, block, &scene->world->bodies.maxBodyIndex), 0,
           scene->world->bodies.bodyCapacity + 5);
}

static void BodyTypeUnknown(const Scene* scene, uint8_t* block)
{
    At(scene, block, &scene->world->bodies.types)[scene->boxA.index1 - 1] = 7;
}

// The lift's leaf leaves the kinematic tree for its free list, so no walk
// meets the lift's shape, and the lift's type becomes 7: only the bound on
// the tree a shape's type picks can refuse it.
static void UntreedShapeOfUnknownType(const Scene* scene, uint8_t* block)
{
    const m2World* w = scene->world;
    uint8_t* headers = At(scene, block, &w->broadphase.trees);
    uint8_t* nodes = At(scene, block, &w->broadphase.treeNodes[m2_kinematicBody]);
    m2DynamicTree tree;
    memcpy(&tree, headers + sizeof(tree) * m2_kinematicBody, sizeof(tree));
    m2TreeNode leaf = GetNode(nodes, tree.root);
    leaf.height = -1;
    leaf.parent = tree.freeList;
    PutNode(nodes, tree.root, leaf);
    tree.freeList = tree.root;
    tree.root = M2_NULL_NODE;
    tree.nodeCount = 0;
    memcpy(headers + sizeof(tree) * m2_kinematicBody, &tree, sizeof(tree));
    At(scene, block, &w->bodies.types)[scene->lift.index1 - 1] = 7;
}

static void ExpectCheckRefuses(const Scene* scene, Edit edit, const char* what)
{
    uint8_t* bytes = (uint8_t*)malloc((size_t)scene->size);
    CHECK(bytes != NULL, "a copy of the snapshot");
    if (bytes == NULL)
    {
        return;
    }
    memcpy(bytes, scene->valid, (size_t)scene->size);
    edit(scene, bytes + scene->headerBytes);
    CHECK(!m2SnapshotCheck(scene->world, bytes + scene->headerBytes), what);
    free(bytes);
}

static void TestCheckStandsAlone(void)
{
    Scene scene = MakeScene();
    CHECK(m2SnapshotCheck(scene.world, scene.valid + scene.headerBytes),
          "the structural check accepts the valid snapshot");
    ExpectCheckRefuses(&scene, BodyTailNegative, "a negative tail is refused by the check alone");
    ExpectCheckRefuses(&scene, BodyHighWaterPastCapacity,
                       "a high-water mark past the pool is refused by the check alone");
    ExpectCheckRefuses(&scene, BodyTypeUnknown,
                       "an unknown body type is refused by the check alone");
    ExpectCheckRefuses(&scene, UntreedShapeOfUnknownType,
                       "an untreed shape of an unknown type is refused by the bound on its tree");
    FreeScene(&scene);
}

static void TestHostileStructures(void)
{
    Scene scene = MakeScene();
    int32_t root = 0;
    m2DynamicTree tree;
    memcpy(&tree, &scene.world->broadphase.trees[m2_dynamicBody], sizeof(tree));
    root = tree.root;
    CHECK(root != M2_NULL_NODE &&
              scene.world->broadphase.treeNodes[m2_dynamicBody][root].height > 0,
          "the dynamic tree has an interior root");
    CHECK(Restore(scene.world, scene.valid, scene.size), "the valid snapshot restores");

    ExpectRefused(&scene, QueueRepeatsASlot, "a body queue that repeats a slot is refused");
    ExpectRefused(&scene, QueueHoldsALiveSlot, "a body queue that holds a live body is refused");
    ExpectRefused(&scene, ParticleQueueRepeatsASlot,
                  "a particle queue that repeats a slot is refused");
    ExpectRefused(&scene, ShapeListLoops, "a shape list that loops is refused");
    ExpectRefused(&scene, ShapeNamesAnotherBody, "a shape on another body's list is refused");
    ExpectRefused(&scene, ChainOnADeadBody, "a chain on a dead body is refused");
    ExpectRefused(&scene, JointToADeadBody, "a joint to a dead body is refused");
    ExpectRefused(&scene, JointAnchorTooFar, "a joint anchor beyond M2_HUGE is refused");
    ExpectRefused(&scene, BodyCenterTooFar, "a center of mass beyond M2_HUGE is refused");
    ExpectRefused(&scene, TreeChildDisownsParent,
                  "a tree child that disowns its parent is refused");
    ExpectRefused(&scene, TreeLoopsToRoot, "a tree that loops is refused");
    ExpectRefused(&scene, TreeHeightLies, "a tree height that lies is refused");
    ExpectRefused(&scene, TreeBoxShrinks, "a tree box that misses a child is refused");
    ExpectRefused(&scene, TreeLosesFreeNodes, "a tree that loses its free nodes is refused");
    ExpectRefused(&scene, ProxiesSwap, "swapped proxies are refused");
    ExpectRefused(&scene, MovedRepeatsAShape, "a moved list that repeats a shape is refused");
    ExpectRefused(&scene, MovedFlagMissing, "a moved shape without its flag is refused");
    ExpectRefused(&scene, PairNamesADeadShape, "a pair with a dead shape is refused");
    ExpectRefused(&scene, PolygonNormalLies, "a polygon whose normal lies is refused");
    ExpectRefused(&scene, GroundGeometryBroken, "a broken ground geometry is refused");

    CHECK(Restore(scene.world, scene.valid, scene.size) && m2World_Validate(scene.world),
          "the valid snapshot still restores");
    FreeScene(&scene);
}

int main(void)
{
    TestHostileStructures();
    TestCheckStandsAlone();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
