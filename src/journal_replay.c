// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal replay: one apply function per op and the command table that
// maps op codes to them. Every apply rebinds the recorded ids to the
// target world and re-enters through the same public call the recording
// came from, so the setter's validation is replay's validation. Creates
// must land on the id the recording saw; anything else means the tape
// does not belong to this world's history.

#include "body.h"
#include "joint.h"
#include "journal.h"
#include "shape.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

// Ids are compared by slot and generation; the world index differs by
// design (the tape was recorded in another world).
#define M2_SAME_ID(a, b) ((a).index1 == (b).index1 && (a).generation == (b).generation)

static bool ApplyStep(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_Step(r->worldId, p->step.dt, p->step.substepCount);
    return true;
}

static bool ApplyCreateBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2BodyId id = m2CreateBody(r->worldId, &p->createBody.def);
    return M2_SAME_ID(id, p->createBody.expected);
}

// Rebinds a recorded id to the target world; the other kinds below follow suit.
static m2BodyId BodyHere(const m2ReplayCursor* r, m2BodyId id)
{
    id.world0 = r->here;
    return id;
}

static bool ApplyDestroyBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2DestroyBody(BodyHere(r, p->bodyId));
    return true;
}

static bool ApplyDisableBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_Disable(BodyHere(r, p->bodyId));
    return true;
}

static bool ApplyEnableBody(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_Enable(BodyHere(r, p->bodyId));
    return true;
}

static bool ApplyMassFromShapes(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyMassFromShapes(BodyHere(r, p->bodyId));
    return true;
}

static bool ApplySetLinearVelocity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetLinearVelocity(BodyHere(r, p->bodyVec.body), p->bodyVec.value);
    return true;
}

static bool ApplyApplyForceCenter(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyForce(BodyHere(r, p->bodyVec.body), p->bodyVec.value);
    return true;
}

static bool ApplyImpulseCenter(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyLinearImpulse(BodyHere(r, p->bodyVec.body), p->bodyVec.value);
    return true;
}

static bool ApplySetAngularVelocity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetAngularVelocity(BodyHere(r, p->bodyFloat.body), p->bodyFloat.value);
    return true;
}

static bool ApplyApplyTorque(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyTorque(BodyHere(r, p->bodyFloat.body), p->bodyFloat.value);
    return true;
}

static bool ApplyAngularImpulse(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyAngularImpulse(BodyHere(r, p->bodyFloat.body), p->bodyFloat.value);
    return true;
}

static bool ApplyLinearImpulse(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyLinearImpulseAtPoint(BodyHere(r, p->bodyPoint.body), p->bodyPoint.value,
                                     p->bodyPoint.point);
    return true;
}

static bool ApplyApplyForce(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_ApplyForceAtPoint(BodyHere(r, p->bodyPoint.body), p->bodyPoint.value,
                             p->bodyPoint.point);
    return true;
}

static bool ApplyBodyParam(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2SetBodyParamInternal(r->world, BodyHere(r, p->bodyParam.body), p->bodyParam.param,
                                  p->bodyParam.value);
}

static bool ApplySetType(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetType(BodyHere(r, p->bodyByte.body), (m2BodyType)p->bodyByte.value);
    return true;
}

static bool ApplySetAwake(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetAwake(BodyHere(r, p->bodyByte.body), p->bodyByte.value != 0);
    return true;
}

static bool ApplySetBullet(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetBullet(BodyHere(r, p->bodyByte.body), p->bodyByte.value != 0);
    return true;
}

static bool ApplySetDominance(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetDominance(BodyHere(r, p->bodyByte.body), (int8_t)p->bodyByte.value);
    return true;
}

static bool ApplyBodyUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetUserData(BodyHere(r, p->bodyUserData.body), p->bodyUserData.userData);
    return true;
}

static bool ApplySetTransform(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetTransform(BodyHere(r, p->setTransform.body), p->setTransform.position,
                        p->setTransform.rotation);
    return true;
}

static bool ApplySetMassData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Body_SetMassData(BodyHere(r, p->setMassData.body), p->setMassData.data);
    return true;
}

// Shapes.

static m2ShapeId ShapeHere(const m2ReplayCursor* r, m2ShapeId id)
{
    id.world0 = r->here;
    return id;
}

static bool ApplyCreateShape(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpCreateShape* op = &p->createShape;
    m2BodyId body = BodyHere(r, op->body);
    m2ShapeId id;
    switch (op->geometry.type)
    {
    case m2_circleShape:
        id = m2CreateCircleShape(body, &op->def, &op->geometry.circle);
        break;
    case m2_capsuleShape:
        id = m2CreateCapsuleShape(body, &op->def, &op->geometry.capsule);
        break;
    case m2_polygonShape:
        id = m2CreatePolygonShape(body, &op->def, &op->geometry.polygon);
        break;
    case m2_segmentShape:
        id = m2CreateSegmentShape(body, &op->def, &op->geometry.segment);
        break;
    default:
        return false; // no recorder writes another shape type
    }
    return M2_SAME_ID(id, op->expected);
}

static bool ApplyDestroyShape(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2DestroyShape(ShapeHere(r, p->shapeId));
    return true;
}

static bool ApplyShapeParam(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2SetShapeParamInternal(r->world, ShapeHere(r, p->shapeParam.shape), p->shapeParam.param,
                                   p->shapeParam.value);
}

static bool ApplySetDensity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Shape_SetDensity(ShapeHere(r, p->shapeFloat.shape), p->shapeFloat.value);
    return true;
}

static bool ApplySetFilter(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpSetFilter* op = &p->setFilter;
    m2Shape_SetFilter(ShapeHere(r, op->shape), op->categoryBits, op->maskBits, op->groupIndex);
    return true;
}

static bool ApplySetGeometry(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpSetGeometry* op = &p->setGeometry;
    m2ShapeId shape = ShapeHere(r, op->shape);
    switch (op->geometry.type)
    {
    case m2_circleShape:
        m2Shape_SetCircle(shape, &op->geometry.circle);
        return true;
    case m2_capsuleShape:
        m2Shape_SetCapsule(shape, &op->geometry.capsule);
        return true;
    case m2_polygonShape:
        m2Shape_SetPolygon(shape, &op->geometry.polygon);
        return true;
    case m2_segmentShape:
        m2Shape_SetSegment(shape, &op->geometry.segment);
        return true;
    default:
        return false;
    }
}

static bool ApplyShapeUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Shape_SetUserData(ShapeHere(r, p->shapeUserData.shape), p->shapeUserData.userData);
    return true;
}

// Chains.

static m2ChainId ChainHere(const m2ReplayCursor* r, m2ChainId id)
{
    id.world0 = r->here;
    return id;
}

static bool ApplyCreateChain(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpChainHeader* op = &p->chainHeader;
    if (op->count < 3 || op->count > (r->size - r->offset) / (int32_t)sizeof(m2Vec2))
    {
        return false;
    }
    int32_t pointBytes = op->count * (int32_t)sizeof(m2Vec2);
    // The stream is unaligned; the points are copied out before use.
    m2Vec2* points = m2AllocZeroed((size_t)pointBytes);
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
    m2ChainId made = m2CreateChain(BodyHere(r, op->body), &def);
    m2Free(points);
    return m2Chain_GetSegmentCount(made) == op->createdCount;
}

static bool ApplyDestroyChain(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2DestroyChain(ChainHere(r, p->chainId));
    return true;
}

static bool ApplyChainFriction(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Chain_SetFriction(ChainHere(r, p->chainFloat.chain), p->chainFloat.value);
    return true;
}

static bool ApplyChainRestitution(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Chain_SetRestitution(ChainHere(r, p->chainFloat.chain), p->chainFloat.value);
    return true;
}

// Joints.

static m2JointId JointHere(const m2ReplayCursor* r, m2JointId id)
{
    id.world0 = r->here;
    return id;
}

// One apply per joint kind: rebind both bodies, create, compare ids.
#define M2_APPLY_CREATE_JOINT(kind)                                                                \
    static bool ApplyCreate##kind##Joint(m2ReplayCursor* r, const m2OpPayload* p)                  \
    {                                                                                              \
        m2OpCreate##kind##Joint op = p->create##kind##Joint;                                       \
        op.def.bodyIdA.world0 = r->here;                                                           \
        op.def.bodyIdB.world0 = r->here;                                                           \
        m2JointId id = m2Create##kind##Joint(r->worldId, &op.def);                                 \
        return M2_SAME_ID(id, op.expected);                                                        \
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
    m2DestroyJoint(JointHere(r, p->jointId));
    return true;
}

static bool ApplySetJointParam(m2ReplayCursor* r, const m2OpPayload* p)
{
    return m2SetJointParamInternal(r->world, JointHere(r, p->jointParam.joint), p->jointParam.param,
                                   p->jointParam.value);
}

static bool ApplyJointUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Joint_SetUserData(JointHere(r, p->jointUserData.joint), p->jointUserData.userData);
    return true;
}

static bool ApplyMotorOffsets(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2MotorJoint_SetOffsets(JointHere(r, p->motorOffsets.joint), p->motorOffsets.linear,
                            p->motorOffsets.angular);
    return true;
}

static bool ApplyMouseTarget(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2MouseJoint_SetTarget(JointHere(r, p->mouseTarget.joint), p->mouseTarget.target);
    return true;
}

// Particles and fluid volumes.

static m2ParticleId ParticleHere(const m2ReplayCursor* r, m2ParticleId id)
{
    id.world0 = r->here;
    return id;
}

static m2FluidVolumeId FluidVolumeHere(const m2ReplayCursor* r, m2FluidVolumeId id)
{
    id.world0 = r->here;
    return id;
}

static bool ApplyEmitParticle(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpEmitParticle* op = &p->emitParticle;
    m2ParticleId id = m2World_EmitParticle(r->worldId, op->position, op->velocity, op->flags);
    return id.index1 == op->expected.index1;
}

static bool ApplyFillParticles(m2ReplayCursor* r, const m2OpPayload* p)
{
    const m2OpFillParticles* op = &p->fillParticles;
    int32_t made = m2World_FillPolygonWithParticles(r->worldId, &op->polygon, op->position,
                                                    op->velocity, op->flags);
    return made == op->expected;
}

static bool ApplyDestroyParticle(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_DestroyParticle(ParticleHere(r, p->particleId));
    return true;
}

static bool ApplySetParticleVelocity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Particle_SetVelocity(ParticleHere(r, p->particleVec.id), p->particleVec.value);
    return true;
}

static bool ApplySetParticleLifetime(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Particle_SetLifetime(ParticleHere(r, p->particleFloat.id), p->particleFloat.value);
    return true;
}

static bool ApplySetParticleUserData(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2Particle_SetUserData(ParticleHere(r, p->particleUserData.id), p->particleUserData.userData);
    return true;
}

static bool ApplyCreateFluidVolume(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2FluidVolumeId id = m2World_CreateFluidVolume(r->worldId, &p->createFluidVolume.def);
    return id.index1 == p->createFluidVolume.expected.index1;
}

static bool ApplyDestroyFluidVolume(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_DestroyFluidVolume(FluidVolumeHere(r, p->fluidVolumeId));
    return true;
}

static bool ApplySetFluidSurface(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2FluidVolume_SetSurface(FluidVolumeHere(r, p->fluidSurface.id), p->fluidSurface.surface);
    return true;
}

// World-level ops.

static bool ApplySetGravity(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_SetGravity(r->worldId, p->vec.value);
    return true;
}

static bool ApplySetWind(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_SetWind(r->worldId, p->setWind.velocity, p->setWind.linearDrag);
    return true;
}

static bool ApplyEnableSleeping(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_EnableSleeping(r->worldId, p->flag.flag != 0);
    return true;
}

static bool ApplyExplode(m2ReplayCursor* r, const m2OpPayload* p)
{
    m2World_Explode(r->worldId, &p->explosion);
    return true;
}

static bool ApplyRestore(m2ReplayCursor* r, const m2OpPayload* p)
{
    int32_t bytes = p->restoreSize;
    if (bytes <= 0 || bytes > r->size - r->offset)
    {
        return false; // truncated restore payload
    }
    if (!m2World_Restore(r->worldId, r->data + r->offset, bytes))
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
    m2Polygon* pieces = m2AllocZeroed((size_t)pieceBytes);
    if (pieces == NULL)
    {
        return false;
    }
    memcpy(pieces, r->data + r->offset, (size_t)pieceBytes);
    r->offset += pieceBytes;
    m2BodyId first[1] = {m2_nullBodyId};
    int32_t made = m2World_ShatterBody(BodyHere(r, op->body), pieces, op->pieceCount, first, 1);
    m2Free(pieces);
    return made == op->pieceCount && first[0].index1 == op->expectedFirst;
}

// The command table, indexed by op code. The payload size is the fixed
// part read before the apply runs; variable-length ops read their tail
// through the cursor.
#define M2_COMMAND(op, payload, fn) [op] = {(int32_t)sizeof(payload), fn}

static const m2JournalCommand s_commands[m2_opCount] = {
    M2_COMMAND(m2_opStep, m2OpStep, ApplyStep),
    M2_COMMAND(m2_opCreateBody, m2OpCreateBody, ApplyCreateBody),
    M2_COMMAND(m2_opDestroyBody, m2BodyId, ApplyDestroyBody),
    M2_COMMAND(m2_opSetLinearVelocity, m2OpBodyVec, ApplySetLinearVelocity),
    M2_COMMAND(m2_opSetAngularVelocity, m2OpBodyFloat, ApplySetAngularVelocity),
    M2_COMMAND(m2_opCreateShape, m2OpCreateShape, ApplyCreateShape),
    M2_COMMAND(m2_opCreateDistanceJoint, m2OpCreateDistanceJoint, ApplyCreateDistanceJoint),
    M2_COMMAND(m2_opCreateRevoluteJoint, m2OpCreateRevoluteJoint, ApplyCreateRevoluteJoint),
    M2_COMMAND(m2_opDestroyJoint, m2JointId, ApplyDestroyJoint),
    M2_COMMAND(m2_opCreatePrismaticJoint, m2OpCreatePrismaticJoint, ApplyCreatePrismaticJoint),
    M2_COMMAND(m2_opCreateWeldJoint, m2OpCreateWeldJoint, ApplyCreateWeldJoint),
    M2_COMMAND(m2_opCreateWheelJoint, m2OpCreateWheelJoint, ApplyCreateWheelJoint),
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
    M2_COMMAND(m2_opCreateMotorJoint, m2OpCreateMotorJoint, ApplyCreateMotorJoint),
    M2_COMMAND(m2_opCreateMouseJoint, m2OpCreateMouseJoint, ApplyCreateMouseJoint),
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
    M2_COMMAND(m2_opCreateGearJoint, m2OpCreateGearJoint, ApplyCreateGearJoint),
    M2_COMMAND(m2_opCreatePulleyJoint, m2OpCreatePulleyJoint, ApplyCreatePulleyJoint),
    M2_COMMAND(m2_opEmitParticle, m2OpEmitParticle, ApplyEmitParticle),
    M2_COMMAND(m2_opDestroyParticle, m2ParticleId, ApplyDestroyParticle),
    M2_COMMAND(m2_opSetParticleVelocity, m2OpParticleVec, ApplySetParticleVelocity),
    M2_COMMAND(m2_opCreateRatchetJoint, m2OpCreateRatchetJoint, ApplyCreateRatchetJoint),
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
