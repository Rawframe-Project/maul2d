// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: creation, destruction, the per-body joint lists, parameters
// and readback for every joint type.

#include "joint.h"

#include "body.h"
#include "broadphase.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

static int32_t JointSlotChecked(const m2World* world, m2JointId jointId);

static int32_t TypedJointSlot(m2World* world, m2JointId jointId, uint8_t type);

static float PulleyLiveLength(m2World* world, int32_t index, int32_t side);

// Inserts edge (2 * joint + side) into body's list, keeping the list in
// ascending joint order so walks visit joints in slot order.
static void LinkJointEdge(m2World* world, int32_t body, int32_t edge)
{
    int32_t joint = edge >> 1;
    int32_t* link = &world->bodyJointHead[body];
    while (*link != -1 && (*link >> 1) < joint)
    {
        link = &world->jointEdgeNext[*link];
    }
    world->jointEdgeNext[edge] = *link;
    *link = edge;
}

static void UnlinkJointEdge(m2World* world, int32_t body, int32_t edge)
{
    int32_t* link = &world->bodyJointHead[body];
    while (*link != -1 && *link != edge)
    {
        link = &world->jointEdgeNext[*link];
    }
    M2_ASSERT(*link == edge);
    if (*link == edge)
    {
        *link = world->jointEdgeNext[edge];
        world->jointEdgeNext[edge] = -1;
    }
}

static void LinkJoint(m2World* world, int32_t joint)
{
    LinkJointEdge(world, world->jointBodyA[joint], 2 * joint);
    LinkJointEdge(world, world->jointBodyB[joint], 2 * joint + 1);
}

void m2UnlinkJoint(m2World* world, int32_t joint)
{
    UnlinkJointEdge(world, world->jointBodyA[joint], 2 * joint);
    UnlinkJointEdge(world, world->jointBodyB[joint], 2 * joint + 1);
}

// Rebuilds every adjacency list from the joint arrays, after a restore
// has replaced them wholesale.
void m2RebuildJointEdges(m2World* world)
{
    for (int32_t b = 0; b < world->bodyCapacity; ++b)
    {
        world->bodyJointHead[b] = -1;
    }
    for (int32_t e = 0; e < 2 * world->jointCapacity; ++e)
    {
        world->jointEdgeNext[e] = -1;
    }
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] != 0)
        {
            LinkJoint(world, j);
        }
    }
}

// Jointed bodies do not collide unless the joint allows it. Walks only
// body A's joints, so the cost is its joint count, not the world's.
bool m2JointsForbidPair(const m2World* world, int32_t bodyA, int32_t bodyB)
{
    for (int32_t e = world->bodyJointHead[bodyA]; e != -1; e = world->jointEdgeNext[e])
    {
        int32_t j = e >> 1;
        int32_t other = (e & 1) != 0 ? world->jointBodyA[j] : world->jointBodyB[j];
        if (other == bodyB && world->jointCollide[j] == 0)
        {
            return true;
        }
    }
    return false;
}

uint64_t m2Joint_GetUserData(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->jointUserData[index];
}

void m2Joint_SetUserData(m2JointId jointId, uint64_t userData)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m2OpJointUserData record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.userData = userData;
        m2JournalRecord(world, m2_opJointUserData, &record, (int32_t)sizeof(record));
    }
    world->jointUserData[index] = userData;
}

m2WorldId m2Joint_GetWorld(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return id;
    }
    id.index1 = world->worldIndex0;
    id.generation = world->worldGeneration;
    return id;
}

float m2Joint_GetLinearSeparation(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t j = jointId.index1 - 1;
    if (world == NULL || j < 0 || j >= world->jointCapacity || world->jointAlive[j] == 0 ||
        world->jointGenerations[j] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    int32_t bodyA = world->jointBodyA[j];
    int32_t bodyB = world->jointBodyB[j];
    m2Transform xfA = world->transforms[bodyA];
    m2Transform xfB = world->transforms[bodyB];
    m2Vec2 aA = world->jointLocalAnchorA[j];
    m2Vec2 aB = world->jointLocalAnchorB[j];
    m2Vec2 wA = {xfA.q.c * aA.x - xfA.q.s * aA.y, xfA.q.s * aA.x + xfA.q.c * aA.y};
    m2Vec2 wB = {xfB.q.c * aB.x - xfB.q.s * aB.y, xfB.q.s * aB.x + xfB.q.c * aB.y};
    float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
    float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
    switch (world->jointType[j])
    {
    case 0: // distance: length error along the rod
        return m2AbsF(sqrtf(dx * dx + dy * dy) - world->jointLength[j]);
    case 2: // prismatic: the off-axis gap
    case 4: // wheel: same slider geometry
    {
        m2Vec2 axis = world->jointLocalAxisA[j];
        m2Vec2 worldAxis = {xfA.q.c * axis.x - xfA.q.s * axis.y,
                            xfA.q.s * axis.x + xfA.q.c * axis.y};
        float perp = dx * -worldAxis.y + dy * worldAxis.x;
        return m2AbsF(perp);
    }
    case 5: // filter: pins nothing
        return 0.0f;
    case 6: // motor: distance from the commanded offset
    {
        m2Vec2 off = world->jointLocalAxisA[j];
        m2Vec2 worldOff = {xfA.q.c * off.x - xfA.q.s * off.y, xfA.q.s * off.x + xfA.q.c * off.y};
        float ex = dx - worldOff.x;
        float ey = dy - worldOff.y;
        return sqrtf(ex * ex + ey * ey);
    }
    case 7: // mouse: gap between grab point and target
    {
        m2Pos2 grab = m2Body_GetWorldPoint(
            (m2BodyId){bodyB + 1, jointId.world0, world->generations[bodyB]}, aB);
        float gx = (float)(grab.x - world->jointTargets[j].x);
        float gy = (float)(grab.y - world->jointTargets[j].y);
        return sqrtf(gx * gx + gy * gy);
    }
    default: // revolute, weld: the pinned point's gap
        return sqrtf(dx * dx + dy * dy);
    }
}

float m2Joint_GetAngularSeparation(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t j = jointId.index1 - 1;
    if (world == NULL || j < 0 || j >= world->jointCapacity || world->jointAlive[j] == 0 ||
        world->jointGenerations[j] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    uint8_t type = world->jointType[j];
    if (type != 3 && type != 6 && type != 2)
    {
        return 0.0f; // no angle is pinned
    }
    m2Rot qA = world->transforms[world->jointBodyA[j]].q;
    m2Rot qB = world->transforms[world->jointBodyB[j]].q;
    return m2AbsF(m2UnwindAngle(m2RelativeJointAngle(qA, qB) - world->jointRefAngle[j]));
}

static int32_t AllocateJoint(m2World* world)
{
    if (world->jointFreeCount == 0)
    {
        return -1;
    }
    int32_t index = world->jointFreeQueue[world->jointFreeHead];
    world->jointFreeHead = (world->jointFreeHead + 1) % world->jointCapacity;
    world->jointFreeCount -= 1;
    if (index + 1 > world->maxJointIndex)
    {
        world->maxJointIndex = index + 1;
    }
    return index;
}

// Relative angle of B vs A from their rotations (own trig: ADR-0010).
float m2RelativeJointAngle(m2Rot qA, m2Rot qB)
{
    float sin = qA.c * qB.s - qA.s * qB.c;
    float cos = qA.c * qB.c + qA.s * qB.s;
    return m2Atan2(sin, cos);
}

static m2JointId FinishJoint(m2World* world, m2WorldId worldId, int32_t index, uint8_t type,
                             int32_t bodyA, int32_t bodyB, m2Vec2 anchorA, m2Vec2 anchorB,
                             float length, float hertz, float damping)
{
    world->jointType[index] = type;
    world->jointBodyA[index] = bodyA;
    world->jointBodyB[index] = bodyB;
    world->jointLocalAnchorA[index] = anchorA;
    world->jointLocalAnchorB[index] = anchorB;
    world->jointLength[index] = length;
    world->jointHertz[index] = hertz;
    world->jointDamping[index] = damping;
    world->jointHertz2[index] = 0.0f;
    world->jointDamping2[index] = 0.0f;
    world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
    world->jointFlags[index] = 0;
    world->jointMotorSpeed[index] = 0.0f;
    world->jointMaxMotor[index] = 0.0f;
    world->jointLower[index] = 0.0f;
    world->jointUpper[index] = 0.0f;
    world->jointLocalAxisA[index] = (m2Vec2){1.0f, 0.0f};
    world->jointRefAngle[index] = 0.0f;
    world->jointMotorImpulse[index] = 0.0f;
    world->jointLowerImpulse[index] = 0.0f;
    world->jointUpperImpulse[index] = 0.0f;
    world->jointSpringImpulse[index] = 0.0f;
    world->jointBreakForce[index] = 0.0f;
    world->jointBreakTorque[index] = 0.0f;
    world->jointCollide[index] = 1;
    world->jointTargets[index] = (m2Pos2){0.0, 0.0};
    world->jointTargetsB[index] = (m2Pos2){0.0, 0.0};
    world->jointUserData[index] = 0;
    world->jointAlive[index] = 1;
    LinkJoint(world, index);
    // A new constraint wakes both ends.
    world->asleep[bodyA] = 0;
    world->sleepTimes[bodyA] = 0.0f;
    world->asleep[bodyB] = 0;
    world->sleepTimes[bodyB] = 0.0f;
    m2JointId id = {index + 1, worldId.index1, world->jointGenerations[index]};
    return id;
}

m2DistanceJointDef m2DefaultDistanceJointDef(void)
{
    m2DistanceJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_DJOINT_COOKIE;
    return def;
}

m2RevoluteJointDef m2DefaultRevoluteJointDef(void)
{
    m2RevoluteJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_RJOINT_COOKIE;
    return def;
}

m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_DJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    float length = def->length;
    if (!(length > 0.0f))
    {
        // Derive from spawn poses: the single f64 crossing.
        m2Transform xfA = world->transforms[bodyA];
        m2Transform xfB = world->transforms[bodyB];
        m2Vec2 wA = {xfA.q.c * def->localAnchorA.x - xfA.q.s * def->localAnchorA.y,
                     xfA.q.s * def->localAnchorA.x + xfA.q.c * def->localAnchorA.y};
        m2Vec2 wB = {xfB.q.c * def->localAnchorB.x - xfB.q.s * def->localAnchorB.y,
                     xfB.q.s * def->localAnchorB.x + xfB.q.c * def->localAnchorB.y};
        float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
        float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
        length = sqrtf(dx * dx + dy * dy);
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 0, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, length, def->hertz, def->dampingRatio);
    // The hard range: off by default (0 .. huge); a def maxLength <= 0
    // means unbounded, mirroring "length <= 0 derives".
    world->jointLower[index] = def->minLength > 0.0f ? def->minLength : 0.0f;
    world->jointUpper[index] = def->maxLength > 0.0f ? def->maxLength : 3.4e38f;
    if (def->enableSpring)
    {
        world->jointFlags[index] |= 16u; // rope/rod: gate the rest-length row
    }
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateDistanceJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateDistanceJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 1, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] = (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerAngle;
    world->jointUpper[index] = def->upperAngle;
    world->jointRefAngle[index] =
        m2RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    world->jointHertz2[index] = def->springHertz;
    world->jointDamping2[index] = def->springDampingRatio;
    if (world->journalActive != 0)
    {
        m2OpCreateRevoluteJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRevoluteJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2PrismaticJointDef m2DefaultPrismaticJointDef(void)
{
    m2PrismaticJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){1.0f, 0.0f};
    def.internalValue = M2_PJOINT_COOKIE;
    return def;
}

m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 2, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] = (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorForce;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    world->jointRefAngle[index] =
        m2RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        m2OpCreatePrismaticJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePrismaticJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2WeldJointDef m2DefaultWeldJointDef(void)
{
    m2WeldJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_WJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2JointId jointId =
        FinishJoint(world, worldId, index, 3, bodyA, bodyB, def->localAnchorA, def->localAnchorB,
                    0.0f, def->linearHertz, def->linearDampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointHertz2[index] = def->angularHertz;
    world->jointDamping2[index] = def->angularDampingRatio;
    world->jointRefAngle[index] =
        m2RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        m2OpCreateWeldJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWeldJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2WheelJointDef m2DefaultWheelJointDef(void)
{
    m2WheelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){0.0f, 1.0f};
    def.enableSpring = true;
    def.hertz = 2.0f;
    def.dampingRatio = 0.7f;
    def.internalValue = M2_WHJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WHJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 4, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] =
        (def->enableMotor ? 1u : 0u) | (def->enableLimit ? 2u : 0u) | (def->enableSpring ? 4u : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    if (world->journalActive != 0)
    {
        m2OpCreateWheelJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWheelJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// One journaled channel for every runtime joint parameter: replay
// re-drives the same setters through the same records.
// The contract of each joint parameter channel: the joint kinds that
// carry the parameter and the values it takes. Live setters and replay
// both pass through it, so a tape can never write what the API refuses.
static bool JointParamValid(uint8_t type, uint8_t param, float value)
{
    switch (param)
    {
    case m2_jointParamEnableMotor:
    case m2_jointParamEnableLimit:
        return value == 0.0f || value == 1.0f;
    case m2_jointParamMotorSpeed:
    case m2_jointParamLower:
    case m2_jointParamUpper:
        return m2FiniteF(value);
    case m2_jointParamMaxMotor:
        return m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamBreakForce:
    case m2_jointParamBreakTorque:
        return value >= 0.0f; // zero turns breaking off; infinity never breaks
    case m2_jointParamHertz:
    case m2_jointParamDamping:
        return type != (uint8_t)m2_filterJoint && type != (uint8_t)m2_motorJoint &&
               m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamAngularHertz:
    case m2_jointParamAngularDamping:
        return (type == (uint8_t)m2_weldJoint || type == (uint8_t)m2_revoluteJoint) &&
               m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamLength:
        return type == (uint8_t)m2_distanceJoint && m2FiniteF(value) && value > 0.0f;
    case m2_jointParamMinLength:
    case m2_jointParamMaxLength:
        return type == (uint8_t)m2_distanceJoint && m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamGearRatio:
        return type == (uint8_t)m2_gearJoint && m2FiniteF(value) && value != 0.0f;
    case m2_jointParamPulleyRatio:
        return type == (uint8_t)m2_pulleyJoint && m2FiniteF(value) && value > 0.0f;
    default:
        return false;
    }
}

// One journaled channel for every joint parameter. Refuses a stale id
// (world may be NULL) or a parameter outside the channel's contract.
bool m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value)
{
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0 || !JointParamValid(world->jointType[index], param, value))
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    if (world->journalActive != 0)
    {
        m2OpJointParam record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opSetJointParam, &record, (int32_t)sizeof(record));
    }
    switch (param)
    {
    case m2_jointParamMotorSpeed:
        world->jointMotorSpeed[index] = value;
        break;
    case m2_jointParamMaxMotor:
        world->jointMaxMotor[index] = value;
        break;
    case m2_jointParamEnableMotor:
        world->jointFlags[index] =
            value != 0.0f ? (world->jointFlags[index] | 1u) : (world->jointFlags[index] & ~1u);
        break;
    case m2_jointParamEnableLimit:
        world->jointFlags[index] =
            value != 0.0f ? (world->jointFlags[index] | 2u) : (world->jointFlags[index] & ~2u);
        break;
    case m2_jointParamLower:
        world->jointLower[index] = value;
        break;
    case m2_jointParamBreakForce:
        world->jointBreakForce[index] = value;
        break;
    case m2_jointParamBreakTorque:
        world->jointBreakTorque[index] = value;
        break;
    case m2_jointParamHertz:
        world->jointHertz[index] = value;
        break;
    case m2_jointParamDamping:
        world->jointDamping[index] = value;
        break;
    case m2_jointParamAngularHertz:
        world->jointHertz2[index] = value;
        if (value == 0.0f)
        {
            world->jointSpringImpulse[index] = 0.0f; // disable drops memory
        }
        break;
    case m2_jointParamAngularDamping:
        world->jointDamping2[index] = value;
        break;
    case m2_jointParamLength:
        // Reference semantics: retargeting the rod drops its memory.
        world->jointLength[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        world->jointLowerImpulse[index] = 0.0f;
        world->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamMinLength:
        world->jointLower[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        world->jointLowerImpulse[index] = 0.0f;
        world->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamMaxLength:
        world->jointUpper[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        world->jointLowerImpulse[index] = 0.0f;
        world->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamGearRatio:
        world->jointLength[index] = value; // phase carries over on purpose
        break;
    case m2_jointParamPulleyRatio:
        // Recapture the rope total from current geometry so the
        // machine does not snap; drop memory like a distance retarget.
        world->jointRefAngle[index] =
            PulleyLiveLength(world, index, 0) + value * PulleyLiveLength(world, index, 1);
        world->jointLength[index] = value;
        world->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        break;
    case m2_jointParamUpper:
        world->jointUpper[index] = value;
        break;
    default:
        M2_ASSERT(false); // JointParamValid admits no other channel
        break;
    }
    // Any parameter change wakes both ends.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
    return true;
}

void m2Joint_SetMotorSpeed(m2JointId jointId, float speed)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamMotorSpeed,
                            speed);
}

void m2Joint_SetMaxMotor(m2JointId jointId, float maxTorqueOrForce)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamMaxMotor,
                            maxTorqueOrForce);
}

void m2Joint_EnableMotor(m2JointId jointId, bool enable)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamEnableMotor,
                            enable ? 1.0f : 0.0f);
}

void m2Joint_EnableLimit(m2JointId jointId, bool enable)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamEnableLimit,
                            enable ? 1.0f : 0.0f);
}

void m2Joint_SetLimits(m2JointId jointId, float lower, float upper)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    if (!(lower <= upper) || !m2FiniteF(lower) || !m2FiniteF(upper))
    {
        m2Refuse(world, m2_errorInvalid); // both or neither: never half a range
        return;
    }
    if (m2SetJointParamInternal(world, jointId, m2_jointParamLower, lower))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamUpper, upper);
    }
}

void m2Joint_SetSpringHertz(m2JointId jointId, float hertz)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamHertz, hertz);
}

void m2Joint_SetSpringDampingRatio(m2JointId jointId, float dampingRatio)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamDamping,
                            dampingRatio);
}

void m2Joint_SetAngularSpringHertz(m2JointId jointId, float hertz)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamAngularHertz,
                            hertz);
}

void m2Joint_SetAngularSpringDampingRatio(m2JointId jointId, float dampingRatio)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamAngularDamping,
                            dampingRatio);
}

void m2DistanceJoint_SetLength(m2JointId jointId, float length)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamLength, length);
}

void m2DistanceJoint_SetLengthRange(m2JointId jointId, float minLength, float maxLength)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    if (!m2FiniteF(minLength) || !m2FiniteF(maxLength))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    // Reference clamps: slop floor, ordered pair.
    float lo = minLength > 0.005f ? minLength : 0.005f;
    float hi = maxLength > 0.005f ? maxLength : 0.005f;
    float lower = lo < hi ? lo : hi;
    float upper = lo < hi ? hi : lo;
    if (m2SetJointParamInternal(world, jointId, m2_jointParamMinLength, lower))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMaxLength, upper);
    }
}

void m2Joint_SetBreakLimits(m2JointId jointId, float maxForce, float maxTorque)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    if (!(maxForce >= 0.0f) || !(maxTorque >= 0.0f))
    {
        m2Refuse(world, m2_errorInvalid); // both or neither
        return;
    }
    if (m2SetJointParamInternal(world, jointId, m2_jointParamBreakForce, maxForce))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamBreakTorque, maxTorque);
    }
}

void m2DestroyJoint(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2JournalRecord(world, m2_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    m2DestroyJointInternal(world, index);
}

// The guts, shared with the solver's break pass (which must not
// journal: breaking is derived from state and replays by itself).
void m2DestroyJointInternal(m2World* world, int32_t index)
{
    // Both ends wake: a constraint vanished.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
    world->jointAlive[index] = 0;
    m2UnlinkJoint(world, index);
    if (world->jointCollide[index] == 0)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB); // pairs may return
    }
    if (world->jointGenerations[index] == UINT16_MAX)
    {
        world->jointRetiredCount += 1;
        return;
    }
    world->jointGenerations[index] += 1;
    world->jointFreeQueue[world->jointFreeTail] = index;
    world->jointFreeTail = (world->jointFreeTail + 1) % world->jointCapacity;
    world->jointFreeCount += 1;
}

m2FilterJointDef m2DefaultFilterJointDef(void)
{
    m2FilterJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_FJOINT_COOKIE;
    return def;
}

m2JointId m2CreateFilterJoint(m2WorldId worldId, const m2FilterJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_FJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 5, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = 0; // its entire purpose
    m2RefilterJointedBodies(world, bodyA, bodyB);
    if (world->journalActive != 0)
    {
        m2OpCreateFilterJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateFilterJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2GearJointDef m2DefaultGearJointDef(void)
{
    m2GearJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_GJOINT_COOKIE;
    return def;
}

// Gear registry mapping: ratio rides jointLength; the two previous
// body rotations ride the anchor slots as (c, s) pairs so the phase
// accumulator in prepare survives any number of full turns; the
// accumulated phase itself rides jointRefAngle. All snapshot state.
m2JointId m2CreateGearJoint(m2WorldId worldId, const m2GearJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_GJOINT_COOKIE ||
        !(def->ratio != 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 8, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLength[index] = def->ratio;
    m2Rot qA = world->transforms[bodyA].q;
    m2Rot qB = world->transforms[bodyB].q;
    world->jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->jointRefAngle[index] = 0.0f; // in phase by definition at birth
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateGearJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateGearJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2GearJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamGearRatio,
                            ratio);
}

float m2GearJoint_GetRatio(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_gearJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

m2PulleyJointDef m2DefaultPulleyJointDef(void)
{
    m2PulleyJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_PLJOINT_COOKIE;
    return def;
}

// Live rope length for one pulley side: attach point (f64 body origin
// plus rotated local anchor) against the f64 ground anchor, a single
// f64 crossing like every other narrowphase entry.
static float PulleyLiveLength(m2World* world, int32_t index, int32_t side)
{
    int32_t body = side == 0 ? world->jointBodyA[index] : world->jointBodyB[index];
    m2Vec2 la = side == 0 ? world->jointLocalAnchorA[index] : world->jointLocalAnchorB[index];
    m2Pos2 g = side == 0 ? world->jointTargets[index] : world->jointTargetsB[index];
    m2Rot q = world->transforms[body].q;
    m2Vec2 arm = {q.c * la.x - q.s * la.y, q.s * la.x + q.c * la.y};
    float dx = (float)(world->transforms[body].p.x - g.x) + arm.x;
    float dy = (float)(world->transforms[body].p.y - g.y) + arm.y;
    return sqrtf(dx * dx + dy * dy);
}

// Pulley registry mapping: ratio rides jointLength, the rope total
// (constant) rides jointRefAngle, ground anchors ride jointTargets
// (A side, shared with mouse) and jointTargetsB. The total is
// CAPTURED from spawn geometry, the reference-angle convention: defs
// carry no length knobs. All snapshot state.
m2JointId m2CreatePulleyJoint(m2WorldId worldId, const m2PulleyJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PLJOINT_COOKIE ||
        !(def->ratio > 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2JointId jointId = FinishJoint(world, worldId, index, 9, bodyA, bodyB, def->localAnchorA,
                                    def->localAnchorB, def->ratio, 0.0f, 0.0f);
    world->jointTargets[index] = def->groundAnchorA;
    world->jointTargetsB[index] = def->groundAnchorB;
    float lengthA = PulleyLiveLength(world, index, 0);
    float lengthB = PulleyLiveLength(world, index, 1);
    world->jointRefAngle[index] = lengthA + def->ratio * lengthB;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreatePulleyJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePulleyJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2PulleyJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamPulleyRatio,
                            ratio);
}

float m2PulleyJoint_GetRatio(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2PulleyJoint_GetLengthA(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? PulleyLiveLength(world, index, 0) : 0.0f;
}

float m2PulleyJoint_GetLengthB(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? PulleyLiveLength(world, index, 1) : 0.0f;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorA(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->jointTargets[index] : zero;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorB(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->jointTargetsB[index] : zero;
}

m2RatchetJointDef m2DefaultRatchetJointDef(void)
{
    m2RatchetJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratchet = 0.5f;
    def.internalValue = M2_RTJOINT_COOKIE;
    return def;
}

// Ratchet registry mapping: tooth angle rides jointLength, phase
// rides jointRefAngle, the accumulated relative angle rides
// jointUpper (multi-turn exact via the gear trick: previous body
// rotations live in the anchor slots as (c, s) pairs), and the
// engaged tooth rides jointLower. All snapshot state.
m2JointId m2CreateRatchetJoint(m2WorldId worldId, const m2RatchetJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RTJOINT_COOKIE ||
        !(def->ratchet != 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 10, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLength[index] = def->ratchet;
    world->jointRefAngle[index] = def->phase;
    m2Rot qA = world->transforms[bodyA].q;
    m2Rot qB = world->transforms[bodyB].q;
    world->jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->jointUpper[index] = 0.0f; // accumulated relative angle
    // Engage the tooth at or behind the spawn angle (reference click).
    world->jointLower[index] =
        floorf((0.0f - def->phase) / def->ratchet) * def->ratchet + def->phase;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateRatchetJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRatchetJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

float m2RatchetJoint_GetRatchet(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_ratchetJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2RatchetJoint_GetPhase(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_ratchetJoint);
    return index >= 0 ? world->jointRefAngle[index] : 0.0f;
}

m2MotorJointDef m2DefaultMotorJointDef(void)
{
    m2MotorJointDef def;
    memset(&def, 0, sizeof(def));
    def.maxForce = 1.0f;
    def.maxTorque = 1.0f;
    def.correctionFactor = 0.3f;
    def.internalValue = M2_MOJOINT_COOKIE;
    return def;
}

m2MouseJointDef m2DefaultMouseJointDef(void)
{
    m2MouseJointDef def;
    memset(&def, 0, sizeof(def));
    def.hertz = 4.0f;
    def.dampingRatio = 1.0f;
    def.maxForce = 35.0f;
    def.internalValue = M2_MSJOINT_COOKIE;
    return def;
}

// Registry mapping for the utility joints (documented deviations from
// the slot names): motor keeps linearOffset in jointLocalAxisA,
// angularOffset in jointRefAngle, maxForce in jointLength and
// correctionFactor in jointDamping; mouse keeps maxForce in
// jointLength and its world target in jointTargets.
m2JointId m2CreateMotorJoint(m2WorldId worldId, const m2MotorJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MOJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId =
        FinishJoint(world, worldId, index, 6, bodyA, bodyB, zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLocalAxisA[index] = def->linearOffset;
    world->jointRefAngle[index] = def->angularOffset;
    world->jointMaxMotor[index] = def->maxTorque;
    world->jointLength[index] = def->maxForce;
    world->jointDamping[index] = def->correctionFactor;
    // Spring drive rides the otherwise-idle secondary spring slots
    // (jointHertz2/jointDamping2 are only read for the angular spring of
    // the revolute and weld, which type 6 is not).
    world->jointHertz2[index] = def->hertz;
    world->jointDamping2[index] = def->dampingRatio;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateMotorJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMotorJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

m2JointId m2CreateMouseJoint(m2WorldId worldId, const m2MouseJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MSJOINT_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    // The grab point is where the target sits at creation, in B's
    // local frame (the single f64 crossing).
    m2Transform xfB = world->transforms[bodyB];
    m2Vec2 rel = {(float)(def->target.x - xfB.p.x), (float)(def->target.y - xfB.p.y)};
    m2Vec2 grab = {xfB.q.c * rel.x + xfB.q.s * rel.y, -xfB.q.s * rel.x + xfB.q.c * rel.y};
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = FinishJoint(world, worldId, index, 7, bodyA, bodyB, zero, grab, 0.0f,
                                    def->hertz, def->dampingRatio);
    world->jointLength[index] = def->maxForce;
    world->jointTargets[index] = def->target;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateMouseJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMouseJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

static int32_t TypedJointSlot(m2World* world, m2JointId jointId, uint8_t type)
{
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation ||
        world->jointType[index] != type)
    {
        m2Refuse(world, m2_errorInvalid); // stale id or wrong type on a typed path
        return -1;
    }
    return index;
}

void m2MotorJoint_SetOffsets(m2JointId jointId, m2Vec2 linearOffset, float angularOffset)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    if (index < 0)
    {
        return; // TypedJointSlot refused
    }
    if (!m2FiniteVec2(linearOffset) || !m2FiniteF(angularOffset))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m2OpMotorOffsets record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.linear = linearOffset;
        record.angular = angularOffset;
        m2JournalRecord(world, m2_opMotorOffsets, &record, (int32_t)sizeof(record));
    }
    world->jointLocalAxisA[index] = linearOffset;
    world->jointRefAngle[index] = angularOffset;
    // Retargeting wakes both ends: the platform starts moving.
    int32_t bodyA = world->jointBodyA[index];
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
}

m2Vec2 m2MotorJoint_GetLinearOffset(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->jointLocalAxisA[index] : zero;
}

float m2MotorJoint_GetAngularOffset(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->jointRefAngle[index] : 0.0f;
}

float m2MotorJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2MotorJoint_GetCorrectionFactor(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->jointDamping[index] : 0.0f;
}

void m2MouseJoint_SetTarget(m2JointId jointId, m2Pos2 target)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    if (index < 0)
    {
        return; // TypedJointSlot refused
    }
    if (!m2FinitePos2(target))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        m2OpMouseTarget record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.target = target;
        m2JournalRecord(world, m2_opMouseTarget, &record, (int32_t)sizeof(record));
    }
    world->jointTargets[index] = target;
    int32_t bodyB = world->jointBodyB[index];
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
}

m2Pos2 m2MouseJoint_GetTarget(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    m2Pos2 zero = {0.0, 0.0};
    return index >= 0 ? world->jointTargets[index] : zero;
}

float m2MouseJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

bool m2Joint_GetCollideConnected(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->jointCollide[index] != 0;
}

float m2Joint_GetReactionForce(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        return 0.0f;
    }
    float force = 0.0f;
    float torque = 0.0f;
    m2JointReactionMagnitudes(world, index, world->lastInvH, &force, &torque);
    return force;
}

float m2Joint_GetReactionTorque(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->jointCapacity ||
        world->jointAlive[index] == 0 || world->jointGenerations[index] != jointId.generation)
    {
        return 0.0f;
    }
    float force = 0.0f;
    float torque = 0.0f;
    m2JointReactionMagnitudes(world, index, world->lastInvH, &force, &torque);
    return torque;
}

bool m2Joint_IsValid(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    if (world == NULL)
    {
        return false;
    }
    int32_t index = jointId.index1 - 1;
    return index >= 0 && index < world->jointCapacity && world->jointAlive[index] != 0 &&
           world->jointGenerations[index] == jointId.generation;
}

// Introspection and enumeration: pure readers for the
// editor and engine-integration walk. Every list is ascending slot
// order (the canonical order everywhere else in Maul) and returns the
// TRUE total even when it exceeds capacity, so callers can size and
// retry instead of silently missing objects.

static int32_t JointSlotChecked(const m2World* world, m2JointId jointId)
{
    int32_t index = jointId.index1 - 1;
    if (index < 0 || index >= world->jointCapacity || world->jointAlive[index] == 0 ||
        world->jointGenerations[index] != jointId.generation)
    {
        return -1;
    }
    return index;
}

m2JointType m2Joint_GetType(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_distanceJoint;
    }
    return (m2JointType)world->jointType[index];
}

m2BodyId m2Joint_GetBodyA(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t b = world->jointBodyA[index];
    m2BodyId id = {b + 1, jointId.world0, world->generations[b]};
    return id;
}

m2BodyId m2Joint_GetBodyB(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t b = world->jointBodyB[index];
    m2BodyId id = {b + 1, jointId.world0, world->generations[b]};
    return id;
}

// Joint parameter readback: with these, a world is
// reconstructible from public getters alone; the mirror test in
// test_world.c holds that promise to hash equality.

static int32_t JointSlotRefusing(m2JointId jointId, m2World** outWorld)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = world != NULL ? JointSlotChecked(world, jointId) : -1;
    *outWorld = world;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
    }
    return index;
}

// Like JointSlotRefusing, but the joint must also be one of the
// kinds in the mask (bit n = m2JointType n). Refuses exactly once.
static int32_t JointSlotOfKind(m2JointId jointId, m2World** outWorld, uint32_t kindMask)
{
    int32_t index = JointSlotRefusing(jointId, outWorld);
    if (index >= 0 && (kindMask & (1u << (*outWorld)->jointType[index])) == 0)
    {
        m2Refuse(*outWorld, m2_errorInvalid);
        return -1;
    }
    return index;
}

m2Vec2 m2Joint_GetLocalAnchorA(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->jointLocalAnchorA[index] : zero;
}

m2Vec2 m2Joint_GetLocalAnchorB(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->jointLocalAnchorB[index] : zero;
}

m2Vec2 m2Joint_GetLocalAxisA(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index =
        JointSlotOfKind(jointId, &world, (1u << m2_prismaticJoint) | (1u << m2_wheelJoint));
    m2Vec2 zero = {0.0f, 0.0f};
    if (index < 0)
    {
        return zero;
    }
    return world->jointLocalAxisA[index];
}

float m2Joint_GetLength(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotOfKind(jointId, &world, 1u << m2_distanceJoint);
    if (index < 0)
    {
        return 0.0f;
    }
    return world->jointLength[index];
}

float m2Joint_GetHertz(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 ? world->jointHertz[index] : 0.0f;
}

float m2Joint_GetDampingRatio(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 ? world->jointDamping[index] : 0.0f;
}

float m2Joint_GetAngularHertz(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index =
        JointSlotOfKind(jointId, &world, (1u << m2_weldJoint) | (1u << m2_revoluteJoint));
    if (index < 0)
    {
        return 0.0f;
    }
    return world->jointHertz2[index];
}

float m2Joint_GetAngularDampingRatio(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index =
        JointSlotOfKind(jointId, &world, (1u << m2_weldJoint) | (1u << m2_revoluteJoint));
    if (index < 0)
    {
        return 0.0f;
    }
    return world->jointDamping2[index];
}

float m2Joint_GetMotorSpeed(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 ? world->jointMotorSpeed[index] : 0.0f;
}

float m2Joint_GetMaxMotor(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 ? world->jointMaxMotor[index] : 0.0f;
}

bool m2Joint_IsMotorEnabled(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 && (world->jointFlags[index] & 1u) != 0;
}

bool m2Joint_IsLimitEnabled(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 && (world->jointFlags[index] & 2u) != 0;
}

bool m2Joint_IsSpringEnabled(m2JointId jointId)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    return index >= 0 && (world->jointFlags[index] & 4u) != 0;
}

void m2Joint_GetLimits(m2JointId jointId, float* lower, float* upper)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    if (lower != NULL)
    {
        *lower = index >= 0 ? world->jointLower[index] : 0.0f;
    }
    if (upper != NULL)
    {
        *upper = index >= 0 ? world->jointUpper[index] : 0.0f;
    }
}

void m2Joint_GetBreakLimits(m2JointId jointId, float* maxForce, float* maxTorque)
{
    m2World* world = NULL;
    int32_t index = JointSlotRefusing(jointId, &world);
    if (maxForce != NULL)
    {
        *maxForce = index >= 0 ? world->jointBreakForce[index] : 0.0f;
    }
    if (maxTorque != NULL)
    {
        *maxTorque = index >= 0 ? world->jointBreakTorque[index] : 0.0f;
    }
}

// Spring-named getter aliases: the setters say Spring, the readers
// now can too. Same slots, same validation.
float m2Joint_GetSpringHertz(m2JointId jointId)
{
    return m2Joint_GetHertz(jointId);
}

float m2Joint_GetSpringDampingRatio(m2JointId jointId)
{
    return m2Joint_GetDampingRatio(jointId);
}

float m2Joint_GetAngularSpringHertz(m2JointId jointId)
{
    return m2Joint_GetAngularHertz(jointId);
}

float m2Joint_GetAngularSpringDampingRatio(m2JointId jointId)
{
    return m2Joint_GetAngularDampingRatio(jointId);
}
