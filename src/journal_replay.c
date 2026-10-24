// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay: one apply function per op and the command table that
// maps op codes to them. Every apply re-enters through the same public
// call the recording came from, so the setter's validation is replay's
// validation. Creates must land on the id the recording saw, and a
// recorded destroy must succeed again; anything else means the tape does
// not belong to this world's history.

#include "body.h"
#include "joint.h"
#include "journal.h"
#include "shape.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <stddef.h>
#include <string.h>

// Ids are compared by slot and generation.
#define M2_SAME_ID(a, b) ((a).index1 == (b).index1 && (a).generation == (b).generation)

static bool ApplyStep(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_Step(r->world, p->step.dt, p->step.substepCount);
    return true;
}

static bool ApplyCreateBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2BodyId id = m2_nullBodyId;
    return m2CreateBody(r->world, &p->createBody.def, &id) == m2_success &&
           M2_SAME_ID(id, p->createBody.expected);
}

static bool ApplyDestroyBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2DestroyBody(r->world, p->bodyId) == m2_success;
}

static bool ApplyDisableBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_Disable(r->world, p->bodyId);
    return true;
}

static bool ApplyEnableBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_Enable(r->world, p->bodyId);
    return true;
}

static bool ApplyMassFromShapes(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyMassFromShapes(r->world, p->bodyId);
    return true;
}

static bool ApplySetLinearVelocity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetLinearVelocity(r->world, p->bodyVec.body, p->bodyVec.value);
    return true;
}

static bool ApplyApplyForceCenter(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyForce(r->world, p->bodyVec.body, p->bodyVec.value);
    return true;
}

static bool ApplyImpulseCenter(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyLinearImpulse(r->world, p->bodyVec.body, p->bodyVec.value);
    return true;
}

static bool ApplySetAngularVelocity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetAngularVelocity(r->world, p->bodyFloat.body, p->bodyFloat.value);
    return true;
}

static bool ApplyApplyTorque(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyTorque(r->world, p->bodyFloat.body, p->bodyFloat.value);
    return true;
}

static bool ApplyAngularImpulse(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyAngularImpulse(r->world, p->bodyFloat.body, p->bodyFloat.value);
    return true;
}

static bool ApplyLinearImpulse(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyLinearImpulseAtPoint(r->world, p->bodyPoint.body, p->bodyPoint.value,
                                     p->bodyPoint.point);
    return true;
}

static bool ApplyApplyForce(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyForceAtPoint(r->world, p->bodyPoint.body, p->bodyPoint.value, p->bodyPoint.point);
    return true;
}

static bool ApplyBodyParam(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2SetBodyParamInternal(r->world, p->bodyParam.body, p->bodyParam.param,
                                  p->bodyParam.value);
}

static bool ApplySetType(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetType(r->world, p->bodyByte.body, (m2BodyType)p->bodyByte.value);
    return true;
}

static bool ApplySetAwake(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetAwake(r->world, p->bodyByte.body, p->bodyByte.value != 0);
    return true;
}

static bool ApplySetBullet(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetBullet(r->world, p->bodyByte.body, p->bodyByte.value != 0);
    return true;
}

static bool ApplySetDominance(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetDominance(r->world, p->bodyByte.body, (int8_t)p->bodyByte.value);
    return true;
}

static bool ApplyBodyUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetUserData(r->world, p->bodyUserData.body, p->bodyUserData.userData);
    return true;
}

static bool ApplySetTransform(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetTransform(r->world, p->setTransform.body, p->setTransform.position,
                        p->setTransform.rotation);
    return true;
}

static bool ApplySetMassData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetMassData(r->world, p->setMassData.body, p->setMassData.data);
    return true;
}

// Shapes.

static bool ApplyCreateShape(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpCreateShape* op = &p->createShape;
    m2BodyId body = op->body;
    m2ShapeId id = m2_nullShapeId;
    m2Result made = m2_errorInvalid;
    switch (op->geometry.type)
    {
    case m2_circleShape:
        made = m2CreateCircleShape(r->world, body, &op->def, &op->geometry.circle, &id);
        break;
    case m2_capsuleShape:
        made = m2CreateCapsuleShape(r->world, body, &op->def, &op->geometry.capsule, &id);
        break;
    case m2_polygonShape:
        made = m2CreatePolygonShape(r->world, body, &op->def, &op->geometry.polygon, &id);
        break;
    case m2_segmentShape:
        made = m2CreateSegmentShape(r->world, body, &op->def, &op->geometry.segment, &id);
        break;
    default:
        return false; // no recorder writes another shape type
    }
    return made == m2_success && M2_SAME_ID(id, op->expected);
}

static bool ApplyDestroyShape(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2DestroyShape(r->world, p->shapeId) == m2_success;
}

static bool ApplyShapeParam(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2SetShapeParamInternal(r->world, p->shapeParam.shape, p->shapeParam.param,
                                   p->shapeParam.value);
}

static bool ApplySetDensity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Shape_SetDensity(r->world, p->shapeFloat.shape, p->shapeFloat.value);
    return true;
}

static bool ApplySetFilter(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpSetFilter* op = &p->setFilter;
    m2Shape_SetFilter(r->world, op->shape, op->categoryBits, op->maskBits, op->groupIndex);
    return true;
}

static bool ApplySetGeometry(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpSetGeometry* op = &p->setGeometry;
    m2ShapeId shape = op->shape;
    switch (op->geometry.type)
    {
    case m2_circleShape:
        m2Shape_SetCircle(r->world, shape, &op->geometry.circle);
        return true;
    case m2_capsuleShape:
        m2Shape_SetCapsule(r->world, shape, &op->geometry.capsule);
        return true;
    case m2_polygonShape:
        m2Shape_SetPolygon(r->world, shape, &op->geometry.polygon);
        return true;
    case m2_segmentShape:
        m2Shape_SetSegment(r->world, shape, &op->geometry.segment);
        return true;
    default:
        return false;
    }
}

static bool ApplyShapeUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Shape_SetUserData(r->world, p->shapeUserData.shape, p->shapeUserData.userData);
    return true;
}

// Chains.

static bool ApplyCreateChain(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpChainHeader* op = &p->chainHeader;
    if (op->count < 3 || op->count > (r->size - r->offset) / (int32_t)sizeof(m2Vec2))
    {
        return false;
    }
    int32_t pointBytes = op->count * (int32_t)sizeof(m2Vec2);
    // The stream is unaligned; the points are copied out before use.
    m2Vec2* points = m2WorldAlloc(r->world, (size_t)pointBytes);
    if (points == NULL)
    {
        return false;
    }
    memcpy(points, r->data + r->offset, (size_t)pointBytes);
    r->offset += pointBytes;
    m2ChainDef def = m2DefaultChainDef();
    def.points = points;
    def.count = op->count;
    def.isLoop = op->isLoop != 0;
    def.friction = op->friction;
    def.restitution = op->restitution;
    def.categoryBits = op->categoryBits;
    def.maskBits = op->maskBits;
    def.groupIndex = op->groupIndex;
    def.userData = op->userData;
    m2ChainId chain = m2_nullChainId;
    m2Result made = m2CreateChain(r->world, op->body, &def, &chain);
    m2WorldFree(r->world, points, (size_t)pointBytes);
    return made == m2_success && m2Chain_GetSegmentCount(r->world, chain) == op->createdCount;
}

static bool ApplyDestroyChain(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2DestroyChain(r->world, p->chainId) == m2_success;
}

static bool ApplyChainFriction(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Chain_SetFriction(r->world, p->chainFloat.chain, p->chainFloat.value);
    return true;
}

static bool ApplyChainRestitution(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Chain_SetRestitution(r->world, p->chainFloat.chain, p->chainFloat.value);
    return true;
}

// Joints.

// One apply per joint kind: create, compare ids.
#define M2_APPLY_CREATE_JOINT(kind)                                                                \
    static bool ApplyCreate##kind##Joint(m2ReplayCursor* r, const m2OpPayload* p)                  \
    {                                                                                              \
        m2OpCreate##kind##Joint op = p->create##kind##Joint;                                       \
        m2JointId id = m2_nullJointId;                                                             \
        return m2Create##kind##Joint(r->world, &op.def, &id) == m2_success &&                      \
               M2_SAME_ID(id, op.expected);                                                        \
    }

M2_APPLY_CREATE_JOINT(Distance)
M2_APPLY_CREATE_JOINT(Revolute)
M2_APPLY_CREATE_JOINT(Prismatic)
M2_APPLY_CREATE_JOINT(Weld)
M2_APPLY_CREATE_JOINT(Wheel)
M2_APPLY_CREATE_JOINT(Filter)
M2_APPLY_CREATE_JOINT(Motor)
M2_APPLY_CREATE_JOINT(Mouse)
M2_APPLY_CREATE_JOINT(Gear)
M2_APPLY_CREATE_JOINT(Pulley)
M2_APPLY_CREATE_JOINT(Ratchet)

#undef M2_APPLY_CREATE_JOINT

static bool ApplyDestroyJoint(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2DestroyJoint(r->world, p->jointId) == m2_success;
}

static bool ApplySetJointParam(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2SetJointParamInternal(r->world, p->jointParam.joint, p->jointParam.param,
                                   p->jointParam.value);
}

static bool ApplyJointUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Joint_SetUserData(r->world, p->jointUserData.joint, p->jointUserData.userData);
    return true;
}

static bool ApplyMotorOffsets(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2MotorJoint_SetOffsets(r->world, p->motorOffsets.joint, p->motorOffsets.linear,
                            p->motorOffsets.angular);
    return true;
}

static bool ApplyMouseTarget(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2MouseJoint_SetTarget(r->world, p->mouseTarget.joint, p->mouseTarget.target);
    return true;
}

// Particles and fluid volumes.

static bool ApplyEmitParticle(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpEmitParticle* op = &p->emitParticle;
    m2ParticleId id = m2_nullParticleId;
    return m2World_EmitParticle(r->world, op->position, op->velocity, op->flags, &id) ==
               m2_success &&
           id.index1 == op->expected.index1;
}

static bool ApplyFillParticles(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpFillParticles* op = &p->fillParticles;
    int32_t made = m2World_FillPolygonWithParticles(r->world, &op->polygon, op->position,
                                                    op->velocity, op->flags);
    return made == op->expected;
}

static bool ApplyDestroyParticle(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2DestroyParticle(r->world, p->particleId) == m2_success;
}

static bool ApplySetParticleVelocity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Particle_SetVelocity(r->world, p->particleVec.id, p->particleVec.value);
    return true;
}

static bool ApplySetParticleLifetime(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Particle_SetLifetime(r->world, p->particleFloat.id, p->particleFloat.value);
    return true;
}

static bool ApplySetParticleUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Particle_SetUserData(r->world, p->particleUserData.id, p->particleUserData.userData);
    return true;
}

static bool ApplyCreateFluidVolume(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2FluidVolumeId id = m2_nullFluidVolumeId;
    return m2CreateFluidVolume(r->world, &p->createFluidVolume.def, &id) == m2_success &&
           id.index1 == p->createFluidVolume.expected.index1;
}

static bool ApplyDestroyFluidVolume(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2DestroyFluidVolume(r->world, p->fluidVolumeId) == m2_success;
}

static bool ApplySetFluidSurface(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2FluidVolume_SetSurface(r->world, p->fluidSurface.id, p->fluidSurface.surface);
    return true;
}

// World-level ops.

static bool ApplySetGravity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_SetGravity(r->world, p->vec.value);
    return true;
}

static bool ApplySetWind(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_SetWind(r->world, p->setWind.velocity, p->setWind.linearDrag);
    return true;
}

static bool ApplyEnableSleeping(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_EnableSleeping(r->world, p->flag.flag != 0);
    return true;
}

static bool ApplyExplode(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_Explode(r->world, &p->explosion);
    return true;
}

static bool ApplyRestore(m2ReplayCursor* r, const m2OpPayload* p)
{
    int32_t bytes = p->restoreSize;
    if (bytes <= 0 || bytes > r->size - r->offset)
    {
        return false; // truncated restore payload
    }
    if (m2World_Restore(r->world, r->data + r->offset, bytes) != m2_success)
    {
        return false;
    }
    r->offset += bytes;
    return true;
}

static bool ApplyShatterBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpShatterHeader* op = &p->shatterHeader;
    if (op->pieceCount < 1 || op->pieceCount > (r->size - r->offset) / (int32_t)sizeof(m2Polygon))
    {
        return false;
    }
    int32_t pieceBytes = op->pieceCount * (int32_t)sizeof(m2Polygon);
    // The stream is unaligned; the pieces are copied out before use.
    m2Polygon* pieces = m2WorldAlloc(r->world, (size_t)pieceBytes);
    if (pieces == NULL)
    {
        return false;
    }
    memcpy(pieces, r->data + r->offset, (size_t)pieceBytes);
    r->offset += pieceBytes;
    m2BodyId first[1] = {m2_nullBodyId};
    int32_t made = 0;
    m2Result shattered =
        m2World_ShatterBody(r->world, op->body, pieces, op->pieceCount, first, 1, &made);
    m2WorldFree(r->world, pieces, (size_t)pieceBytes);
    return shattered == m2_success && made == op->pieceCount &&
           first[0].index1 == (uint32_t)op->expectedFirst;
}

// The command table, indexed by op code. The payload size is the fixed
// part read before the apply runs; variable-length ops read their tail
// through the cursor.
#define M2_COMMAND(op, payload, fn) [op] = {(int32_t)sizeof(payload), fn, NULL, 0}
#define M2_COMMAND_BOOLS(op, payload, fn, bools)                                                   \
    [op] = {(int32_t)sizeof(payload), fn, (bools), (int32_t)(sizeof(bools) / sizeof((bools)[0]))}

// The bool members of the payloads that carry them: the defs' flags.
#define M2_AT(payload, member) (uint16_t)offsetof(payload, member)
static const uint16_t s_bodyBools[] = {M2_AT(m2OpCreateBody, def.motionLocks.linearX),
                                       M2_AT(m2OpCreateBody, def.motionLocks.linearY),
                                       M2_AT(m2OpCreateBody, def.motionLocks.angularZ),
                                       M2_AT(m2OpCreateBody, def.fixedRotation),
                                       M2_AT(m2OpCreateBody, def.enableSleep),
                                       M2_AT(m2OpCreateBody, def.isEnabled),
                                       M2_AT(m2OpCreateBody, def.isBullet)};
static const uint16_t s_shapeBools[] = {M2_AT(m2OpCreateShape, def.isSensor)};
static const uint16_t s_distanceBools[] = {M2_AT(m2OpCreateDistanceJoint, def.enableSpring),
                                           M2_AT(m2OpCreateDistanceJoint, def.collideConnected)};
static const uint16_t s_revoluteBools[] = {M2_AT(m2OpCreateRevoluteJoint, def.enableMotor),
                                           M2_AT(m2OpCreateRevoluteJoint, def.enableLimit),
                                           M2_AT(m2OpCreateRevoluteJoint, def.collideConnected)};
static const uint16_t s_prismaticBools[] = {M2_AT(m2OpCreatePrismaticJoint, def.enableMotor),
                                            M2_AT(m2OpCreatePrismaticJoint, def.enableLimit),
                                            M2_AT(m2OpCreatePrismaticJoint, def.collideConnected)};
static const uint16_t s_weldBools[] = {M2_AT(m2OpCreateWeldJoint, def.collideConnected)};
static const uint16_t s_wheelBools[] = {M2_AT(m2OpCreateWheelJoint, def.enableSpring),
                                        M2_AT(m2OpCreateWheelJoint, def.enableMotor),
                                        M2_AT(m2OpCreateWheelJoint, def.enableLimit),
                                        M2_AT(m2OpCreateWheelJoint, def.collideConnected)};
static const uint16_t s_motorBools[] = {M2_AT(m2OpCreateMotorJoint, def.collideConnected)};
static const uint16_t s_mouseBools[] = {M2_AT(m2OpCreateMouseJoint, def.collideConnected)};
static const uint16_t s_gearBools[] = {M2_AT(m2OpCreateGearJoint, def.collideConnected)};
static const uint16_t s_pulleyBools[] = {M2_AT(m2OpCreatePulleyJoint, def.collideConnected)};
static const uint16_t s_ratchetBools[] = {M2_AT(m2OpCreateRatchetJoint, def.collideConnected)};
#undef M2_AT

static const m2JournalCommand s_commands[m2_opCount] = {
    M2_COMMAND(m2_opStep, m2OpStep, ApplyStep),
    M2_COMMAND_BOOLS(m2_opCreateBody, m2OpCreateBody, ApplyCreateBody, s_bodyBools),
    M2_COMMAND(m2_opDestroyBody, m2BodyId, ApplyDestroyBody),
    M2_COMMAND(m2_opSetLinearVelocity, m2OpBodyVec, ApplySetLinearVelocity),
    M2_COMMAND(m2_opSetAngularVelocity, m2OpBodyFloat, ApplySetAngularVelocity),
    M2_COMMAND_BOOLS(m2_opCreateShape, m2OpCreateShape, ApplyCreateShape, s_shapeBools),
    M2_COMMAND_BOOLS(m2_opCreateDistanceJoint, m2OpCreateDistanceJoint, ApplyCreateDistanceJoint,
                     s_distanceBools),
    M2_COMMAND_BOOLS(m2_opCreateRevoluteJoint, m2OpCreateRevoluteJoint, ApplyCreateRevoluteJoint,
                     s_revoluteBools),
    M2_COMMAND(m2_opDestroyJoint, m2JointId, ApplyDestroyJoint),
    M2_COMMAND_BOOLS(m2_opCreatePrismaticJoint, m2OpCreatePrismaticJoint, ApplyCreatePrismaticJoint,
                     s_prismaticBools),
    M2_COMMAND_BOOLS(m2_opCreateWeldJoint, m2OpCreateWeldJoint, ApplyCreateWeldJoint, s_weldBools),
    M2_COMMAND_BOOLS(m2_opCreateWheelJoint, m2OpCreateWheelJoint, ApplyCreateWheelJoint,
                     s_wheelBools),
    M2_COMMAND(m2_opDestroyShape, m2ShapeId, ApplyDestroyShape),
    M2_COMMAND(m2_opApplyLinearImpulse, m2OpBodyPoint, ApplyLinearImpulse),
    M2_COMMAND(m2_opApplyAngularImpulse, m2OpBodyFloat, ApplyAngularImpulse),
    M2_COMMAND(m2_opSetJointParam, m2OpJointParam, ApplySetJointParam),
    M2_COMMAND(m2_opSetTransform, m2OpSetTransform, ApplySetTransform),
    M2_COMMAND(m2_opSetType, m2OpBodyByte, ApplySetType),
    M2_COMMAND(m2_opRestore, int32_t, ApplyRestore),
    M2_COMMAND(m2_opCreateChain, m2OpChainHeader, ApplyCreateChain),
    M2_COMMAND(m2_opSetGravity, m2OpVec, ApplySetGravity),
    M2_COMMAND(m2_opShapeParam, m2OpShapeParam, ApplyShapeParam),
    M2_COMMAND(m2_opSetFilter, m2OpSetFilter, ApplySetFilter),
    M2_COMMAND(m2_opDestroyChain, m2ChainId, ApplyDestroyChain),
    M2_COMMAND(m2_opBodyParam, m2OpBodyParam, ApplyBodyParam),
    M2_COMMAND(m2_opEnableSleeping, m2OpFlag, ApplyEnableSleeping),
    M2_COMMAND(m2_opApplyForce, m2OpBodyPoint, ApplyApplyForce),
    M2_COMMAND(m2_opApplyForceCenter, m2OpBodyVec, ApplyApplyForceCenter),
    M2_COMMAND(m2_opApplyTorque, m2OpBodyFloat, ApplyApplyTorque),
    M2_COMMAND(m2_opCreateFilterJoint, m2OpCreateFilterJoint, ApplyCreateFilterJoint),
    M2_COMMAND_BOOLS(m2_opCreateMotorJoint, m2OpCreateMotorJoint, ApplyCreateMotorJoint,
                     s_motorBools),
    M2_COMMAND_BOOLS(m2_opCreateMouseJoint, m2OpCreateMouseJoint, ApplyCreateMouseJoint,
                     s_mouseBools),
    M2_COMMAND(m2_opMotorOffsets, m2OpMotorOffsets, ApplyMotorOffsets),
    M2_COMMAND(m2_opMouseTarget, m2OpMouseTarget, ApplyMouseTarget),
    M2_COMMAND(m2_opDisableBody, m2BodyId, ApplyDisableBody),
    M2_COMMAND(m2_opEnableBody, m2BodyId, ApplyEnableBody),
    M2_COMMAND(m2_opSetMassData, m2OpSetMassData, ApplySetMassData),
    M2_COMMAND(m2_opMassFromShapes, m2BodyId, ApplyMassFromShapes),
    M2_COMMAND(m2_opExplode, m2ExplosionDef, ApplyExplode),
    M2_COMMAND(m2_opSetGeometry, m2OpSetGeometry, ApplySetGeometry),
    M2_COMMAND(m2_opChainFriction, m2OpChainFloat, ApplyChainFriction),
    M2_COMMAND(m2_opChainRestitution, m2OpChainFloat, ApplyChainRestitution),
    M2_COMMAND(m2_opImpulseCenter, m2OpBodyVec, ApplyImpulseCenter),
    M2_COMMAND(m2_opSetAwake, m2OpBodyByte, ApplySetAwake),
    M2_COMMAND(m2_opSetBullet, m2OpBodyByte, ApplySetBullet),
    M2_COMMAND(m2_opSetDensity, m2OpShapeFloat, ApplySetDensity),
    M2_COMMAND(m2_opBodyUserData, m2OpBodyUserData, ApplyBodyUserData),
    M2_COMMAND(m2_opShapeUserData, m2OpShapeUserData, ApplyShapeUserData),
    M2_COMMAND(m2_opJointUserData, m2OpJointUserData, ApplyJointUserData),
    M2_COMMAND(m2_opSetDominance, m2OpBodyByte, ApplySetDominance),
    M2_COMMAND_BOOLS(m2_opCreateGearJoint, m2OpCreateGearJoint, ApplyCreateGearJoint, s_gearBools),
    M2_COMMAND_BOOLS(m2_opCreatePulleyJoint, m2OpCreatePulleyJoint, ApplyCreatePulleyJoint,
                     s_pulleyBools),
    M2_COMMAND(m2_opEmitParticle, m2OpEmitParticle, ApplyEmitParticle),
    M2_COMMAND(m2_opDestroyParticle, m2ParticleId, ApplyDestroyParticle),
    M2_COMMAND(m2_opSetParticleVelocity, m2OpParticleVec, ApplySetParticleVelocity),
    M2_COMMAND_BOOLS(m2_opCreateRatchetJoint, m2OpCreateRatchetJoint, ApplyCreateRatchetJoint,
                     s_ratchetBools),
    M2_COMMAND(m2_opFillParticles, m2OpFillParticles, ApplyFillParticles),
    M2_COMMAND(m2_opShatterBody, m2OpShatterHeader, ApplyShatterBody),
    M2_COMMAND(m2_opSetParticleLifetime, m2OpParticleFloat, ApplySetParticleLifetime),
    M2_COMMAND(m2_opSetParticleUserData, m2OpParticleUserData, ApplySetParticleUserData),
    M2_COMMAND(m2_opCreateFluidVolume, m2OpCreateFluidVolume, ApplyCreateFluidVolume),
    M2_COMMAND(m2_opDestroyFluidVolume, m2FluidVolumeId, ApplyDestroyFluidVolume),
    M2_COMMAND(m2_opSetFluidSurface, m2OpFluidSurface, ApplySetFluidSurface),
    M2_COMMAND(m2_opSetWind, m2OpSetWind, ApplySetWind),
};

#undef M2_COMMAND

const m2JournalCommand* m2JournalCommandFor(uint8_t op)
{
    if (op >= m2_opCount || s_commands[op].apply == NULL)
    {
        return NULL;
    }
    return &s_commands[op];
}
