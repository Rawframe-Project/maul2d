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
#include "joint.h"
#include "joint_row.h"
#include "snapshot_check.h"
#include "test_harness.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul2d/maul2d.h"

#include <math.h>
#include <stddef.h>
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
// particles, stepped the given number of times so trees, and after the
// boxes land the pairs, hold state.
static Scene MakeSceneAfter(int32_t steps)
{
    Scene scene;
    memset(&scene, 0, sizeof(scene));
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    def.jointCapacity = 4;
    def.particleCapacity = 8;
    def.fluidVolumeCapacity = 1; // none made, so its slot keeps its zeros
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
    for (int32_t i = 0; i < steps; ++i)
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

static Scene MakeScene(void)
{
    return MakeSceneAfter(10);
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

// One float of a body's mass data, as stored.
static void PutBodyFloat(const Scene* scene, uint8_t* block, const void* field, m2BodyId body,
                         float value)
{
    uint8_t* bytes = At(scene, block, field);
    memcpy(bytes + (size_t)(body.index1 - 1) * sizeof(float), &value, sizeof(value));
}

// A dynamic box's inverse mass doubles: no longer what its shape gives it.
static void DerivedMassLies(const Scene* scene, uint8_t* block)
{
    float invMass = scene->world->bodies.invMass[scene->boxA.index1 - 1];
    PutBodyFloat(scene, block, &scene->world->bodies.invMass, scene->boxA, 2.0f * invMass);
}

// A dynamic box's center of mass moves a millimeter off its shape's.
static void DerivedCenterMoves(const Scene* scene, uint8_t* block)
{
    m2Vec2 center = scene->world->bodies.localCenters[scene->boxA.index1 - 1];
    m2Vec2 moved = {center.x + 0.001f, center.y};
    uint8_t* centers = At(scene, block, &scene->world->bodies.localCenters);
    memcpy(centers + (size_t)(scene->boxA.index1 - 1) * sizeof(moved), &moved, sizeof(moved));
}

// The static ground claims an override, which only a dynamic body takes.
static void StaticOverride(const Scene* scene, uint8_t* block)
{
    At(scene, block, &scene->world->bodies.massOverrides)[scene->ground.index1 - 1] = 1;
}

// A dynamic box claims an override with a negative inverse mass.
static void OverrideNegativeMass(const Scene* scene, uint8_t* block)
{
    At(scene, block, &scene->world->bodies.massOverrides)[scene->boxA.index1 - 1] = 1;
    PutBodyFloat(scene, block, &scene->world->bodies.invMass, scene->boxA, -1.0f);
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

// --- The fuzz finds ---------------------------------------------------------
//
// fuzz_snapshot found these as worlds that turned invalid a step after a
// restore. Each edit carries one mechanism:
//   crash-6834a002: a gravity scale times a gravity makes an infinite
//     velocity;
//   crash-b4ff047a: a wind drag on a moving body makes an infinite force;
//   crash-1844ca69, crash-1b23f4ae, crash-352f027c, crash-d2972878: a
//     spring impulse, warm started, overflows a body's spin; contact
//     impulses add to it;
//   crash-2eef07f3, crash-6becfced: a joint's point impulse, warm started;
//   crash-4c1b1e36: a joint's upper limit impulse, warm started.
// Past the range a restore holds each value to, the snapshot is refused.
// At the edge of the range the solver saturates what the values feed it,
// so the world steps on with every speed within M2_MAX_SPEED.

static void PutBodyVec(const Scene* scene, uint8_t* block, const void* field, m2BodyId body,
                       m2Vec2 value)
{
    memcpy(At(scene, block, field) + (size_t)(body.index1 - 1) * sizeof(value), &value,
           sizeof(value));
}

static void PutWorld(const Scene* scene, uint8_t* block, const void* field, const void* value,
                     size_t bytes)
{
    memcpy(At(scene, block, field), value, bytes);
}

static void Gravity(const Scene* scene, uint8_t* block, float scale, float gravity)
{
    PutBodyFloat(scene, block, &scene->world->bodies.gravityScales, scene->boxA, scale);
    m2Vec2 g = {0.0f, gravity};
    PutWorld(scene, block, &scene->world->gravity, &g, sizeof(g));
}

static void GravityPast(const Scene* scene, uint8_t* block)
{
    Gravity(scene, block, -1.0e30f, 1.0e30f);
}

static void GravityAtTheEdge(const Scene* scene, uint8_t* block)
{
    Gravity(scene, block, -M2_MAX_GRAVITY_SCALE, M2_MAX_ACCELERATION);
}

static void WindDrag(const Scene* scene, uint8_t* block, float drag)
{
    PutWorld(scene, block, &scene->world->windLinearDrag, &drag, sizeof(drag));
    PutBodyVec(scene, block, &scene->world->bodies.linearVelocities, scene->boxA,
               (m2Vec2){10.0f, 0.0f});
}

static void WindDragPast(const Scene* scene, uint8_t* block)
{
    WindDrag(scene, block, 1.0e38f);
}

static void WindDragAtTheEdge(const Scene* scene, uint8_t* block)
{
    WindDrag(scene, block, M2_MAX_RATE);
}

// Every contact point of the scene carries the same impulse.
static void ContactImpulses(const Scene* scene, uint8_t* block, float impulse)
{
    int32_t pairs = GetI32(At(scene, block, &scene->world->contacts.pairCount), 0);
    uint8_t* manifolds = At(scene, block, &scene->world->contacts.manifolds);
    for (int32_t i = 0; i < pairs; ++i)
    {
        m2Manifold m;
        memcpy(&m, manifolds + (size_t)i * sizeof(m), sizeof(m));
        for (int32_t k = 0; k < m.pointCount; ++k)
        {
            m.points[k].normalImpulse = impulse;
            m.points[k].tangentImpulse = -impulse;
        }
        memcpy(manifolds + (size_t)i * sizeof(m), &m, sizeof(m));
    }
}

// The pin is the scene's only joint, in slot 0.
static void PutJointFloat(const Scene* scene, uint8_t* block, const void* field, float value)
{
    memcpy(At(scene, block, field), &value, sizeof(value));
}

static void SpringImpulsePast(const Scene* scene, uint8_t* block)
{
    PutJointFloat(scene, block, &scene->world->joints.jointSpringImpulse, 1.0e38f);
}

static void ContactImpulsesPast(const Scene* scene, uint8_t* block)
{
    ContactImpulses(scene, block, 1.0e37f);
}

static void WarmStartAtTheEdge(const Scene* scene, uint8_t* block)
{
    PutJointFloat(scene, block, &scene->world->joints.jointSpringImpulse, M2_IMPULSE_LIMIT);
    ContactImpulses(scene, block, M2_IMPULSE_LIMIT);
}

static void PointImpulse(const Scene* scene, uint8_t* block, float impulse)
{
    m2Vec2 j = {impulse, -impulse};
    PutWorld(scene, block, &scene->world->joints.jointImpulse, &j, sizeof(j));
}

static void PointImpulsePast(const Scene* scene, uint8_t* block)
{
    PointImpulse(scene, block, -1.7e38f);
}

static void PointImpulseAtTheEdge(const Scene* scene, uint8_t* block)
{
    PointImpulse(scene, block, M2_IMPULSE_LIMIT);
}

static void UpperImpulsePast(const Scene* scene, uint8_t* block)
{
    PutJointFloat(scene, block, &scene->world->joints.jointUpperImpulse, -1.7e38f);
}

static void UpperImpulseAtTheEdge(const Scene* scene, uint8_t* block)
{
    PutJointFloat(scene, block, &scene->world->joints.jointUpperImpulse, -M2_IMPULSE_LIMIT);
}

static bool SpeedsInRange(const m2World* world, m2BodyId body)
{
    m2Vec2 v = m2Body_GetLinearVelocity(world, body);
    float w = m2Body_GetAngularVelocity(world, body);
    return fabsf(v.x) <= M2_MAX_SPEED && fabsf(v.y) <= M2_MAX_SPEED && fabsf(w) <= M2_MAX_SPEED;
}

static void ExpectSteps(const Scene* scene, Edit edit, const char* what)
{
    uint8_t* bytes = (uint8_t*)malloc((size_t)scene->size);
    CHECK(bytes != NULL, "a copy of the snapshot");
    if (bytes == NULL)
    {
        return;
    }
    memcpy(bytes, scene->valid, (size_t)scene->size);
    edit(scene, bytes + scene->headerBytes);
    bool valid = Restore(scene->world, bytes, scene->size);
    for (int32_t step = 0; step < 10 && valid; ++step)
    {
        m2World_Step(scene->world, 1.0f / 60.0f, 4);
        valid = m2World_Validate(scene->world) && SpeedsInRange(scene->world, scene->boxA) &&
                SpeedsInRange(scene->world, scene->boxB);
    }
    CHECK(valid, what);
    CHECK(Restore(scene->world, scene->valid, scene->size), "the valid snapshot restores again");
    free(bytes);
}

// Whether the scene holds a contact point for the contact edits to reach.
static bool HasContactPoints(const m2World* world)
{
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.manifolds[i].pointCount > 0)
        {
            return true;
        }
    }
    return false;
}

static void TestFuzzFinds(void)
{
    Scene scene = MakeSceneAfter(60); // the boxes have landed
    CHECK(HasContactPoints(scene.world), "the boxes rest on the ground");
    ExpectRefused(&scene, GravityPast, "a gravity and a scale past their ranges are refused");
    ExpectRefused(&scene, WindDragPast, "a wind drag past its range is refused");
    ExpectRefused(&scene, SpringImpulsePast, "a spring impulse past the limit is refused");
    ExpectRefused(&scene, ContactImpulsesPast, "contact impulses past the limit are refused");
    ExpectRefused(&scene, PointImpulsePast, "a joint impulse past the limit is refused");
    ExpectRefused(&scene, UpperImpulsePast, "a limit impulse past the limit is refused");
    ExpectSteps(&scene, GravityAtTheEdge, "the strongest gravity steps into a valid world");
    ExpectSteps(&scene, WindDragAtTheEdge, "the strongest drag steps into a valid world");
    ExpectSteps(&scene, WarmStartAtTheEdge, "the largest warm start steps into a valid world");
    ExpectSteps(&scene, PointImpulseAtTheEdge,
                "the largest joint impulse steps into a valid world");
    ExpectSteps(&scene, UpperImpulseAtTheEdge,
                "the largest limit impulse steps into a valid world");
    FreeScene(&scene);
}

// --- Ranges at their edges ---------------------------------------------------
//
// A restore holds every value to the range its def, its setter and its
// replay take: the edge of each range restores, the next float past it is
// refused and changes nothing.

// The snapshot with one float written at a byte offset into a field's
// incoming bytes, restored; the scene is put back after.
static m2Result RestoreWithFloat(const Scene* scene, const void* field, size_t at, float value)
{
    uint8_t* bytes = (uint8_t*)malloc((size_t)scene->size);
    CHECK(bytes != NULL, "a copy of the snapshot");
    if (bytes == NULL)
    {
        return m2_errorCapacity;
    }
    memcpy(bytes, scene->valid, (size_t)scene->size);
    memcpy(At(scene, bytes + scene->headerBytes, field) + at, &value, sizeof(value));
    uint64_t misuse = m2World_GetCounters(scene->world).misuse;
    m2Result result = m2World_Restore(scene->world, bytes, scene->size);
    CHECK(m2World_GetCounters(scene->world).misuse == misuse, "a restore counts no misuse");
    CHECK(Restore(scene->world, scene->valid, scene->size), "the valid snapshot restores again");
    free(bytes);
    return result;
}

typedef struct Bound
{
    const char* name;
    const void* field;
    size_t at;  // byte offset of the float in the field's bytes
    float edge; // in the range
} Bound;

// The edge restores, and the next float away from the range is refused.
static void ExpectBound(const Scene* scene, Bound bound, float outward)
{
    float past = nextafterf(bound.edge, outward);
    bool edge = RestoreWithFloat(scene, bound.field, bound.at, bound.edge) == m2_success;
    bool refused = RestoreWithFloat(scene, bound.field, bound.at, past) == m2_errorInvalid;
    if (!edge || !refused)
    {
        printf("FAIL: %s: the edge %g %s, the next float %g %s\n", bound.name, (double)bound.edge,
               edge ? "restores" : "is refused", (double)past, refused ? "is refused" : "restores");
        s_failures += 1;
    }
}

static void ExpectUpper(const Scene* scene, Bound bound)
{
    ExpectBound(scene, bound, INFINITY);
}

static void ExpectLower(const Scene* scene, Bound bound)
{
    ExpectBound(scene, bound, -INFINITY);
}

static void TestWorldAndBodyBounds(const Scene* scene)
{
    const m2World* w = scene->world;
    size_t body = (size_t)(scene->boxA.index1 - 1) * sizeof(float);
    size_t bodyVec = (size_t)(scene->boxA.index1 - 1) * sizeof(m2Vec2);
    float maxInvH = (float)M2_MAX_SUBSTEPS / M2_MIN_TIME_STEP;
    ExpectUpper(scene, (Bound){"gravity", &w->gravity, 4, M2_MAX_ACCELERATION});
    ExpectUpper(scene, (Bound){"wind velocity", &w->windVelocity, 0, M2_MAX_SPEED});
    ExpectUpper(scene, (Bound){"wind drag", &w->windLinearDrag, 0, M2_MAX_RATE});
    ExpectLower(scene, (Bound){"wind drag", &w->windLinearDrag, 0, 0.0f});
    ExpectUpper(scene, (Bound){"1 / substep", &w->lastInvH, 0, maxInvH});
    ExpectLower(scene, (Bound){"1 / substep", &w->lastInvH, 0, 1.0f / M2_MAX_TIME_STEP});
    ExpectUpper(scene, (Bound){"velocity", &w->bodies.linearVelocities, bodyVec + 4, M2_MAX_SPEED});
    ExpectLower(scene, (Bound){"spin", &w->bodies.angularVelocities, body, -M2_MAX_SPEED});
    ExpectLower(scene,
                (Bound){"gravity scale", &w->bodies.gravityScales, body, -M2_MAX_GRAVITY_SCALE});
    ExpectUpper(scene, (Bound){"linear damping", &w->bodies.linearDampings, body, M2_MAX_RATE});
    ExpectLower(scene, (Bound){"angular damping", &w->bodies.angularDampings, body, 0.0f});
    ExpectUpper(scene, (Bound){"force", &w->bodies.forces, bodyVec, M2_MAX_FORCE});
    ExpectLower(scene, (Bound){"torque", &w->bodies.torques, body, -M2_MAX_FORCE});
}

// Shape materials in a slot no shape uses, whose mass no body takes.
static void TestShapeAndParticleBounds(const Scene* scene)
{
    const m2World* w = scene->world;
    size_t shape = (size_t)(w->shapes.shapeCapacity - 1) * sizeof(float);
    ExpectUpper(scene, (Bound){"density", &w->shapes.shapeDensity, shape, M2_MAX_DENSITY});
    ExpectLower(scene, (Bound){"density", &w->shapes.shapeDensity, shape, 0.0f});
    ExpectUpper(scene, (Bound){"friction", &w->shapes.shapeFriction, shape, M2_MAX_COEFFICIENT});
    ExpectUpper(scene, (Bound){"restitution", &w->shapes.shapeRestitution, shape, 1.0f});
    ExpectLower(scene, (Bound){"restitution", &w->shapes.shapeRestitution, shape, 0.0f});
    ExpectUpper(scene, (Bound){"tangent speed", &w->shapes.shapeTangentSpeed, shape, M2_MAX_SPEED});
    ExpectLower(scene,
                (Bound){"particle velocity", &w->particles.particleVelocities, 0, -M2_MAX_SPEED});
    ExpectUpper(scene, (Bound){"volume density", &w->volumes.fvDensity, 0, M2_MAX_DENSITY});
    ExpectUpper(scene, (Bound){"volume drag", &w->volumes.fvLinearDrag, 0, M2_MAX_RATE});
    ExpectLower(scene, (Bound){"volume spin drag", &w->volumes.fvAngularDrag, 0, 0.0f});
    ExpectUpper(scene, (Bound){"volume flow", &w->volumes.fvFlow, 4, M2_MAX_SPEED});
}

// The joint channels every kind keeps the same way, on the pin in slot 0,
// and the impulses of a contact point.
static void TestImpulseAndChannelBounds(const Scene* scene)
{
    const m2World* w = scene->world;
    ExpectUpper(scene, (Bound){"hertz", &w->joints.jointHertz, 0, M2_MAX_RATE});
    ExpectLower(scene, (Bound){"hertz", &w->joints.jointHertz, 0, 0.0f});
    ExpectUpper(scene, (Bound){"damping ratio", &w->joints.jointDamping, 0, M2_MAX_COEFFICIENT});
    ExpectUpper(scene, (Bound){"spring hertz", &w->joints.jointHertz2, 0, M2_MAX_RATE});
    ExpectLower(scene, (Bound){"spring damping", &w->joints.jointDamping2, 0, 0.0f});
    ExpectLower(scene, (Bound){"motor speed", &w->joints.jointMotorSpeed, 0, -M2_MAX_SPEED});
    ExpectUpper(scene, (Bound){"max motor", &w->joints.jointMaxMotor, 0, M2_MAX_FORCE});
    ExpectLower(scene, (Bound){"max motor", &w->joints.jointMaxMotor, 0, 0.0f});
    ExpectUpper(scene, (Bound){"joint impulse", &w->joints.jointImpulse, 4, M2_IMPULSE_LIMIT});
    ExpectLower(scene,
                (Bound){"motor impulse", &w->joints.jointMotorImpulse, 0, -M2_IMPULSE_LIMIT});
    ExpectUpper(scene, (Bound){"lower impulse", &w->joints.jointLowerImpulse, 0, M2_IMPULSE_LIMIT});
    ExpectLower(scene,
                (Bound){"upper impulse", &w->joints.jointUpperImpulse, 0, -M2_IMPULSE_LIMIT});
    ExpectUpper(scene,
                (Bound){"spring impulse", &w->joints.jointSpringImpulse, 0, M2_IMPULSE_LIMIT});
    ExpectLower(scene, (Bound){"break force", &w->joints.jointBreakForce, 0, 0.0f});
    ExpectLower(scene, (Bound){"break torque", &w->joints.jointBreakTorque, 0, 0.0f});
    CHECK(RestoreWithFloat(scene, &w->joints.jointBreakForce, 0, INFINITY) == m2_success,
          "a break force of infinity, which never breaks, restores");
    size_t point = offsetof(m2Manifold, points);
    ExpectUpper(scene, (Bound){"normal impulse", &w->contacts.manifolds,
                               point + offsetof(m2ManifoldPoint, normalImpulse), M2_IMPULSE_LIMIT});
    ExpectLower(scene,
                (Bound){"tangent impulse", &w->contacts.manifolds,
                        point + offsetof(m2ManifoldPoint, tangentImpulse), -M2_IMPULSE_LIMIT});
}

static void TestRangeBounds(void)
{
    Scene scene = MakeSceneAfter(60);
    TestWorldAndBodyBounds(&scene);
    TestShapeAndParticleBounds(&scene);
    TestImpulseAndChannelBounds(&scene);
    FreeScene(&scene);
}

// A static ground and one dynamic box per joint kind, each box held to the
// ground by a joint of its kind, in the slot its kind's number names.
static Scene MakeJointScene(void)
{
    Scene scene;
    memset(&scene, 0, sizeof(scene));
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 16;
    scene.world = NewWorld(&def);
    m2World* w = scene.world;
    m2BodyDef groundDef = m2DefaultBodyDef();
    scene.ground = NewBody(w, &groundDef);
    m2BodyId boxes[11];
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.2f, 0.2f);
    for (int32_t k = 0; k < 11; ++k)
    {
        m2BodyDef boxDef = m2DefaultBodyDef();
        boxDef.type = m2_dynamicBody;
        boxDef.position = (m2Pos2){(double)k, 2.0};
        boxes[k] = NewBody(w, &boxDef);
        NewPolygonShape(w, boxes[k], &shapeDef, &box);
    }
    m2BodyId g = scene.ground;
    m2DistanceJointDef distance = m2DefaultDistanceJointDef();
    distance.bodyIdA = g;
    distance.bodyIdB = boxes[m2_distanceJoint];
    NewDistanceJoint(w, &distance);
    m2RevoluteJointDef revolute = m2DefaultRevoluteJointDef();
    revolute.bodyIdA = g;
    revolute.bodyIdB = boxes[m2_revoluteJoint];
    NewRevoluteJoint(w, &revolute);
    m2PrismaticJointDef prismatic = m2DefaultPrismaticJointDef();
    prismatic.bodyIdA = g;
    prismatic.bodyIdB = boxes[m2_prismaticJoint];
    NewPrismaticJoint(w, &prismatic);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = g;
    weld.bodyIdB = boxes[m2_weldJoint];
    NewWeldJoint(w, &weld);
    m2WheelJointDef wheel = m2DefaultWheelJointDef();
    wheel.bodyIdA = g;
    wheel.bodyIdB = boxes[m2_wheelJoint];
    NewWheelJoint(w, &wheel);
    m2FilterJointDef filter = m2DefaultFilterJointDef();
    filter.bodyIdA = g;
    filter.bodyIdB = boxes[m2_filterJoint];
    NewFilterJoint(w, &filter);
    m2GearJointDef gear = m2DefaultGearJointDef();
    gear.bodyIdA = g;
    gear.bodyIdB = boxes[m2_gearJoint];
    NewGearJoint(w, &gear);
    m2PulleyJointDef pulley = m2DefaultPulleyJointDef();
    pulley.bodyIdA = boxes[m2_pulleyJoint];
    pulley.bodyIdB = boxes[m2_distanceJoint];
    pulley.groundAnchorA = (m2Pos2){(double)m2_pulleyJoint, 5.0};
    pulley.groundAnchorB = (m2Pos2){0.0, 5.0};
    NewPulleyJoint(w, &pulley);
    m2RatchetJointDef ratchet = m2DefaultRatchetJointDef();
    ratchet.bodyIdA = g;
    ratchet.bodyIdB = boxes[m2_ratchetJoint];
    NewRatchetJoint(w, &ratchet);
    m2MotorJointDef motor = m2DefaultMotorJointDef();
    motor.bodyIdA = g;
    motor.bodyIdB = boxes[m2_motorJoint];
    NewMotorJoint(w, &motor);
    m2MouseJointDef mouse = m2DefaultMouseJointDef();
    mouse.bodyIdA = g;
    mouse.bodyIdB = boxes[m2_mouseJoint];
    mouse.target = (m2Pos2){(double)m2_mouseJoint, 2.0};
    NewMouseJoint(w, &mouse);
    for (int32_t i = 0; i < 5; ++i)
    {
        m2World_Step(w, 1.0f / 60.0f, 4);
    }
    scene.size = m2World_SnapshotSize(w);
    scene.valid = (uint8_t*)malloc((size_t)scene.size);
    CHECK(scene.valid != NULL && Snapshot(w, scene.valid, scene.size) == scene.size,
          "the joint scene snapshots");
    scene.headerBytes = scene.size - m2StateWalk(w, NULL, NULL, 2);
    return scene;
}

// The byte offset of a joint kind's float in a joint array: the scene puts
// each kind in its own slot, created in the order of the kinds.
static size_t Slot(const Scene* scene, m2JointType type)
{
    for (int32_t j = 0; j < scene->world->joints.jointCapacity; ++j)
    {
        if (scene->world->joints.jointAlive[j] != 0 && scene->world->joints.jointType[j] == type)
        {
            return (size_t)j * sizeof(float);
        }
    }
    CHECK(false, "the joint scene holds every kind");
    return 0;
}

static void TestJointKindBounds(void)
{
    Scene scene = MakeJointScene();
    const m2Joints* j = &scene.world->joints;
    float reach = (float)M2_JOINT_REACH;
    size_t at = Slot(&scene, m2_distanceJoint);
    ExpectUpper(&scene, (Bound){"distance length", &j->jointLength, at, 2.0f * reach});
    ExpectLower(&scene, (Bound){"distance length", &j->jointLength, at, 0.0f});
    ExpectLower(&scene, (Bound){"distance minimum", &j->jointLower, at, 0.0f});
    ExpectUpper(&scene, (Bound){"distance maximum", &j->jointUpper, at, reach});
    CHECK(RestoreWithFloat(&scene, &j->jointUpper, at, M2_OPEN_LENGTH) == m2_success,
          "a distance joint without a maximum restores");
    at = Slot(&scene, m2_revoluteJoint);
    ExpectLower(&scene, (Bound){"revolute lower", &j->jointLower, at, -M2_MAX_ANGLE});
    ExpectUpper(&scene, (Bound){"revolute upper", &j->jointUpper, at, M2_MAX_ANGLE});
    ExpectUpper(&scene, (Bound){"revolute reference", &j->jointRefAngle, at, M2_MAX_ANGLE});
    at = Slot(&scene, m2_prismaticJoint);
    ExpectLower(&scene, (Bound){"prismatic lower", &j->jointLower, at, -reach});
    ExpectLower(&scene, (Bound){"prismatic reference", &j->jointRefAngle, at, -M2_MAX_ANGLE});
    at = Slot(&scene, m2_wheelJoint);
    ExpectUpper(&scene, (Bound){"wheel upper", &j->jointUpper, at, reach});
    at = Slot(&scene, m2_weldJoint);
    ExpectUpper(&scene, (Bound){"weld reference", &j->jointRefAngle, at, M2_MAX_ANGLE});
    at = Slot(&scene, m2_motorJoint);
    ExpectUpper(&scene, (Bound){"motor force", &j->jointLength, at, M2_MAX_FORCE});
    ExpectLower(&scene, (Bound){"motor force", &j->jointLength, at, 0.0f});
    ExpectLower(&scene, (Bound){"motor offset", &j->jointRefAngle, at, -M2_MAX_ANGLE});
    ExpectUpper(&scene, (Bound){"motor correction", &j->jointDamping, at, 1.0f});
    at = Slot(&scene, m2_mouseJoint);
    ExpectUpper(&scene, (Bound){"mouse force", &j->jointLength, at, M2_MAX_FORCE});
    at = Slot(&scene, m2_gearJoint);
    ExpectUpper(&scene, (Bound){"gear ratio", &j->jointLength, at, M2_MAX_RATIO});
    ExpectUpper(&scene, (Bound){"gear ratio", &j->jointLength, at, -M2_MIN_RATIO});
    ExpectLower(&scene, (Bound){"gear ratio", &j->jointLength, at, M2_MIN_RATIO});
    CHECK(RestoreWithFloat(&scene, &j->jointRefAngle, at, 1.0e30f) == m2_success,
          "a gear's phase, which the step accumulates, restores at any finite value");
    at = Slot(&scene, m2_pulleyJoint);
    float ratio = j->jointLength[at / sizeof(float)];
    ExpectUpper(&scene, (Bound){"pulley ratio", &j->jointLength, at, M2_MAX_RATIO});
    ExpectLower(&scene, (Bound){"pulley ratio", &j->jointLength, at, M2_MIN_RATIO});
    ExpectUpper(&scene,
                (Bound){"pulley rope", &j->jointRefAngle, at, (1.0f + ratio) * 2.0f * reach});
    ExpectLower(&scene, (Bound){"pulley rope", &j->jointRefAngle, at, 0.0f});
    at = Slot(&scene, m2_ratchetJoint);
    ExpectUpper(&scene, (Bound){"ratchet tooth", &j->jointLength, at, M2_MAX_ANGLE});
    ExpectLower(&scene, (Bound){"ratchet tooth", &j->jointLength, at, M2_MIN_RATCHET_TOOTH});
    ExpectUpper(&scene, (Bound){"ratchet phase", &j->jointRefAngle, at, M2_MAX_ANGLE});
    CHECK(RestoreWithFloat(&scene, &j->jointUpper, at, 1.0e30f) == m2_success &&
              RestoreWithFloat(&scene, &j->jointLower, at, -1.0e30f) == m2_success,
          "a ratchet's angle and tooth, which the step accumulates, restore at any finite value");
    FreeScene(&scene);
}

// --- What the step stores stays in range ------------------------------------
//
// A solve that scales a pair's impulse to its limit, or stores the old
// impulse plus a clamped difference, can round a float past the limit; a
// restore would then refuse the world's own snapshot. Each case below is
// an input that rounds past it before the stored value is held.

static void TestStoredImpulsesStayInRange(void)
{
    m2Vec2 zero = {0.0f, 0.0f};
    m2RowDrive still = {0.0f, 0.0f, 0.0f};
    // A point pair whose impulse, (21 2^46, 0), is scaled to 1e15: the
    // scaled component rounds to the float above 1e15.
    float mass = 0x1p40f;
    m2JointBodies heavy = {zero, 0.0f, zero, 0.0f, 1.0f / mass, 0.0f, 1.0f / mass, 0.0f};
    m2PointMass pointMass = {mass, 0.0f, mass};
    m2Vec2 pair = zero;
    m2SolvePointPair(zero, zero, pointMass, &heavy, (m2Vec2){-21.0f * 64.0f, 0.0f}, still, &pair,
                     M2_MAX_FORCE);
    CHECK(m2ValidImpulseVec(pair) && pair.x == M2_IMPULSE_LIMIT,
          "a point pair scaled to the limit stores the limit");
    // Two rows whose first impulse turns from just inside -1e15 to past
    // +1e15 in one solve: the old impulse plus the clamped difference
    // rounds to the float above 1e15.
    m2JointRow rows[2] = {{zero, 0.0f, {1.0f, 0.0f}, 0.0f}, {zero, 0.0f, {0.0f, 1.0f}, 0.0f}};
    m2JointBodies light = {zero, 0.0f, zero, 0.0f, 0.0f, 0.0f, 1.0f / mass, 0.0f};
    m2Vec2 rowPair = {-0x1.c6bf4cp+49f, 0.0f};
    m2SolveRowPair(rows, &light, (m2Vec2){-4096.0f, 0.0f}, still, &rowPair);
    CHECK(m2ValidImpulseVec(rowPair) && rowPair.x == M2_IMPULSE_LIMIT,
          "a row pair clamped to the limit stores the limit");
}

// --- Boxes of bodies the step does not refit -----------------------------------
//
// The step refits a body's boxes only while it is awake and not static.
// A snapshot that puts a static or sleeping body far from its boxes would
// leave them stale until something wakes or moves it, so the step and a
// host's queries would find the body where it is not (crash-8e3b702a
// moved the static ground to about 1e40 m, and a bullet near the origin
// found it there); a restore refuses it. Every world keeps those boxes
// right, and still restores its own snapshot.

static void PutTransform(const Scene* scene, uint8_t* block, m2BodyId body, m2Pos2 position)
{
    uint8_t* transforms = At(scene, block, &scene->world->bodies.transforms);
    m2Transform xf;
    memcpy(&xf, transforms + (size_t)(body.index1 - 1) * sizeof(xf), sizeof(xf));
    xf.p = position;
    memcpy(transforms + (size_t)(body.index1 - 1) * sizeof(xf), &xf, sizeof(xf));
}

static void StaticGroundFarAway(const Scene* scene, uint8_t* block)
{
    PutTransform(scene, block, scene->ground, (m2Pos2){1.0e30, 0.0});
}

static void SleepingBoxFarAway(const Scene* scene, uint8_t* block)
{
    PutTransform(scene, block, scene->boxA, (m2Pos2){0.0, 1.0e30});
}

static void AwakeBoxMoved(const Scene* scene, uint8_t* block)
{
    PutTransform(scene, block, scene->boxA, (m2Pos2){50.0, 50.0});
}

// The world's own snapshot restores onto the same hash.
static bool RoundTripsAlone(m2World* world)
{
    int32_t size = m2World_SnapshotSize(world);
    void* bytes = malloc((size_t)size);
    if (bytes == NULL)
    {
        return false;
    }
    uint64_t hash = m2World_Hash(world);
    bool restores = Snapshot(world, bytes, size) == size && Restore(world, bytes, size) &&
                    m2World_Hash(world) == hash;
    free(bytes);
    return restores;
}

// A world without gravity holding one box, made with the given velocity.
static m2World* CrawlWorld(m2Vec2 velocity, m2BodyId* box)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    bodyDef.linearVelocity = velocity;
    *box = NewBody(world, &bodyDef);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon square = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, *box, &shapeDef, &square);
    return world;
}

static void TestRestingBoxes(void)
{
    Scene scene = MakeScene();
    ExpectRefused(&scene, StaticGroundFarAway, "a static body far from its boxes is refused");
    CHECK(m2Body_IsAwake(scene.world, scene.boxA), "the box is awake after 10 steps");
    uint8_t* bytes = (uint8_t*)malloc((size_t)scene.size);
    CHECK(bytes != NULL, "a copy of the snapshot");
    if (bytes != NULL)
    {
        memcpy(bytes, scene.valid, (size_t)scene.size);
        AwakeBoxMoved(&scene, bytes + scene.headerBytes);
        CHECK(Restore(scene.world, bytes, scene.size),
              "an awake body away from its boxes restores: the next step refits them");
        free(bytes);
    }
    FreeScene(&scene);

    // Under the sleep tolerance, a 10 s step carries the box 0.4 m, past
    // the box margin, and it falls asleep at the step's end.
    m2BodyId box;
    m2World* world = CrawlWorld((m2Vec2){0.04f, 0.0f}, &box);
    m2World_Step(world, M2_MAX_TIME_STEP, 1);
    CHECK(!m2Body_IsAwake(world, box), "the crawling box falls asleep in one long step");
    Scene sleeping = {world, NULL, m2World_SnapshotSize(world), 0, {0}, box, {0}, {0}};
    sleeping.valid = (uint8_t*)malloc((size_t)sleeping.size);
    CHECK(sleeping.valid != NULL && Snapshot(world, sleeping.valid, sleeping.size) == sleeping.size,
          "the sleeping world snapshots");
    sleeping.headerBytes = sleeping.size - m2StateWalk(world, NULL, NULL, 2);
    uint64_t hash = m2World_Hash(world);
    CHECK(Restore(world, sleeping.valid, sleeping.size) && m2World_Hash(world) == hash,
          "a body that fell asleep after moving past its margin restores its own snapshot");
    ExpectRefused(&sleeping, SleepingBoxFarAway, "a sleeping body far from its boxes is refused");
    FreeScene(&sleeping);

    // Put to sleep by hand right after a step that carried it 1.7 m.
    world = CrawlWorld((m2Vec2){100.0f, 0.0f}, &box);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Body_SetAwake(world, box, false);
    CHECK(RoundTripsAlone(world), "a body put to sleep by hand restores its own snapshot");
    m2DestroyWorld(world);
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
    ExpectRefused(&scene, DerivedMassLies, "mass data its shapes do not give is refused");
    ExpectRefused(&scene, DerivedCenterMoves, "a center its shapes do not give is refused");
    ExpectRefused(&scene, StaticOverride, "an override on a static body is refused");
    ExpectRefused(&scene, OverrideNegativeMass, "an override no setter would take is refused");
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
    TestFuzzFinds();
    TestRangeBounds();
    TestJointKindBounds();
    TestStoredImpulsesStayInRange();
    TestRestingBoxes();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
