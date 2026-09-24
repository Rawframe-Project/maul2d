// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The command journal's vocabulary: every op code and the one payload
// struct each op carries. Recorders and replay share these definitions,
// so the bytes a setter writes are, by construction, the bytes replay
// reads back.

#ifndef MAUL2D_SRC_JOURNAL_H
#define MAUL2D_SRC_JOURNAL_H

#include "geometry.h"
#include "world_internal.h"

#include "maul2d/base.h"
#include "maul2d/body.h"
#include "maul2d/joint.h"
#include "maul2d/particle.h"
#include "maul2d/shape.h"
#include "maul2d/world.h"

#include <stdint.h>

// Op codes. The values are the wire format: never renumber, only append.
enum
{
    m2_opStep = 1,
    m2_opCreateBody = 2,
    m2_opDestroyBody = 3,
    m2_opSetLinearVelocity = 4,
    m2_opSetAngularVelocity = 5,
    m2_opCreateShape = 6,
    m2_opCreateDistanceJoint = 7,
    m2_opCreateRevoluteJoint = 8,
    m2_opDestroyJoint = 9,
    m2_opCreatePrismaticJoint = 10,
    m2_opCreateWeldJoint = 11,
    m2_opCreateWheelJoint = 12,
    m2_opDestroyShape = 13,
    m2_opApplyLinearImpulse = 14,
    m2_opApplyAngularImpulse = 15,
    m2_opSetJointParam = 16,
    m2_opSetTransform = 17,
    m2_opSetType = 18,
    m2_opRestore = 19,     // variable length: i32 size + snapshot bytes
    m2_opCreateChain = 20, // variable length: m2OpChainHeader + points
    m2_opSetGravity = 21,
    m2_opShapeParam = 22,
    m2_opSetFilter = 23,
    m2_opDestroyChain = 24,
    m2_opBodyParam = 25,
    m2_opEnableSleeping = 26,
    m2_opApplyForce = 27,
    m2_opApplyForceCenter = 28,
    m2_opApplyTorque = 29,
    m2_opCreateFilterJoint = 30,
    m2_opCreateMotorJoint = 31,
    m2_opCreateMouseJoint = 32,
    m2_opMotorOffsets = 33,
    m2_opMouseTarget = 34,
    m2_opDisableBody = 35,
    m2_opEnableBody = 36,
    m2_opSetMassData = 37,
    m2_opMassFromShapes = 38,
    m2_opExplode = 39,
    m2_opSetGeometry = 40,
    m2_opChainFriction = 41,
    m2_opChainRestitution = 42,
    m2_opImpulseCenter = 43,
    m2_opSetAwake = 44,
    m2_opSetBullet = 45,
    m2_opSetDensity = 46,
    m2_opBodyUserData = 47,
    m2_opShapeUserData = 48,
    m2_opJointUserData = 49,
    m2_opSetDominance = 50,
    m2_opCreateGearJoint = 51,
    m2_opCreatePulleyJoint = 52,
    m2_opEmitParticle = 53,
    m2_opDestroyParticle = 54,
    m2_opSetParticleVelocity = 55,
    m2_opCreateRatchetJoint = 56,
    m2_opFillParticles = 57,
    m2_opShatterBody = 58, // variable length: m2OpShatterHeader + polygons
    m2_opSetParticleLifetime = 59,
    m2_opSetParticleUserData = 60,
    m2_opCreateFluidVolume = 61,
    m2_opDestroyFluidVolume = 62,
    m2_opSetFluidSurface = 63,
    m2_opSetWind = 64,
    m2_opCount
};

// Parameter channels: which field an m2_opBodyParam, m2_opShapeParam
// or m2_opSetJointParam record writes. Wire values, append only.
enum
{
    m2_bodyParamLinearDamping = 0,
    m2_bodyParamAngularDamping = 1,
    m2_bodyParamGravityScale = 2,
    m2_bodyParamFixedRotation = 3,
    m2_bodyParamEnableSleep = 4,
    m2_bodyParamLockLinearX = 5,
    m2_bodyParamLockLinearY = 6,
    m2_bodyParamCount
};

enum
{
    m2_shapeParamFriction = 0,
    m2_shapeParamRestitution = 1,
    m2_shapeParamTangentSpeed = 2,
    m2_shapeParamCount
};

enum
{
    m2_jointParamMotorSpeed = 0,
    m2_jointParamMaxMotor = 1,
    m2_jointParamEnableMotor = 2,
    m2_jointParamEnableLimit = 3,
    m2_jointParamLower = 4,
    m2_jointParamUpper = 5,
    m2_jointParamBreakForce = 6,
    m2_jointParamBreakTorque = 7,
    m2_jointParamHertz = 8,
    m2_jointParamDamping = 9,
    m2_jointParamAngularHertz = 10,
    m2_jointParamAngularDamping = 11,
    m2_jointParamLength = 12,    // distance only; resets impulses
    m2_jointParamMinLength = 13, // distance only; resets impulses
    m2_jointParamMaxLength = 14,
    m2_jointParamGearRatio = 15,
    m2_jointParamPulleyRatio = 16,
    m2_jointParamCount
};

// Payloads. Recorders zero the struct before filling it, so padding
// bytes are deterministic. Ops whose payload is a bare id (destroy,
// enable, disable, mass from shapes) carry the public id type itself.

typedef struct m2OpStep
{
    float dt;
    int32_t substepCount;
} m2OpStep;

typedef struct m2OpCreateBody
{
    m2BodyDef def;
    m2BodyId expected;
} m2OpCreateBody;

// Linear velocity, force to center, impulse to center.
typedef struct m2OpBodyVec
{
    m2BodyId body;
    m2Vec2 value;
} m2OpBodyVec;

// Angular velocity, torque, angular impulse.
typedef struct m2OpBodyFloat
{
    m2BodyId body;
    float value;
} m2OpBodyFloat;

// Linear impulse or force at a world point.
typedef struct m2OpBodyPoint
{
    m2BodyId body;
    m2Vec2 value;
    m2Pos2 point;
} m2OpBodyPoint;

typedef struct m2OpBodyParam
{
    m2BodyId body;
    float value;
    uint8_t param;
} m2OpBodyParam;

// Body type, awake, bullet, dominance: one byte of state.
typedef struct m2OpBodyByte
{
    m2BodyId body;
    uint8_t value;
} m2OpBodyByte;

typedef struct m2OpBodyUserData
{
    m2BodyId body;
    uint64_t userData;
} m2OpBodyUserData;

typedef struct m2OpSetTransform
{
    m2BodyId body;
    m2Pos2 position;
    m2Rot rotation;
} m2OpSetTransform;

typedef struct m2OpSetMassData
{
    m2BodyId body;
    m2MassData data;
} m2OpSetMassData;

typedef struct m2OpCreateShape
{
    m2BodyId body;
    m2ShapeDef def;
    m2ShapeGeometry geometry;
    m2ShapeId expected;
} m2OpCreateShape;

typedef struct m2OpShapeParam
{
    m2ShapeId shape;
    float value;
    uint8_t param;
} m2OpShapeParam;

typedef struct m2OpShapeFloat
{
    m2ShapeId shape;
    float value;
} m2OpShapeFloat;

typedef struct m2OpSetFilter
{
    m2ShapeId shape;
    uint64_t categoryBits;
    uint64_t maskBits;
    int32_t groupIndex;
} m2OpSetFilter;

typedef struct m2OpSetGeometry
{
    m2ShapeId shape;
    m2ShapeGeometry geometry;
} m2OpSetGeometry;

typedef struct m2OpShapeUserData
{
    m2ShapeId shape;
    uint64_t userData;
} m2OpShapeUserData;

typedef struct m2OpChainHeader
{
    m2BodyId body;
    int32_t count;
    int32_t createdCount;
    float friction;
    float restitution;
    uint64_t categoryBits;
    uint64_t maskBits;
    int32_t groupIndex;
    uint64_t userData;
    uint8_t isLoop;
} m2OpChainHeader;

// Chain friction and restitution.
typedef struct m2OpChainFloat
{
    m2ChainId chain;
    float value;
} m2OpChainFloat;

// One create payload per joint kind: the def as given, plus the id the
// recording saw.
#define M2_OP_CREATE_JOINT(kind)                                                                   \
    typedef struct m2OpCreate##kind##Joint                                                         \
    {                                                                                              \
        m2##kind##JointDef def;                                                                    \
        m2JointId expected;                                                                        \
    } m2OpCreate##kind##Joint

M2_OP_CREATE_JOINT(Distance);
M2_OP_CREATE_JOINT(Revolute);
M2_OP_CREATE_JOINT(Prismatic);
M2_OP_CREATE_JOINT(Weld);
M2_OP_CREATE_JOINT(Wheel);
M2_OP_CREATE_JOINT(Filter);
M2_OP_CREATE_JOINT(Motor);
M2_OP_CREATE_JOINT(Mouse);
M2_OP_CREATE_JOINT(Gear);
M2_OP_CREATE_JOINT(Pulley);
M2_OP_CREATE_JOINT(Ratchet);

#undef M2_OP_CREATE_JOINT

typedef struct m2OpJointParam
{
    m2JointId joint;
    float value;
    uint8_t param;
} m2OpJointParam;

typedef struct m2OpJointUserData
{
    m2JointId joint;
    uint64_t userData;
} m2OpJointUserData;

typedef struct m2OpMotorOffsets
{
    m2JointId joint;
    m2Vec2 linear;
    float angular;
} m2OpMotorOffsets;

typedef struct m2OpMouseTarget
{
    m2JointId joint;
    m2Pos2 target;
} m2OpMouseTarget;

typedef struct m2OpEmitParticle
{
    m2Pos2 position;
    m2Vec2 velocity;
    uint32_t flags;
    m2ParticleId expected;
} m2OpEmitParticle;

typedef struct m2OpFillParticles
{
    m2Polygon polygon;
    m2Pos2 position;
    m2Vec2 velocity;
    uint32_t flags;
    int32_t expected;
} m2OpFillParticles;

typedef struct m2OpParticleVec
{
    m2ParticleId id;
    m2Vec2 value;
} m2OpParticleVec;

typedef struct m2OpParticleFloat
{
    m2ParticleId id;
    float value;
} m2OpParticleFloat;

typedef struct m2OpParticleUserData
{
    m2ParticleId id;
    uint64_t userData;
} m2OpParticleUserData;

typedef struct m2OpCreateFluidVolume
{
    m2FluidVolumeDef def;
    m2FluidVolumeId expected;
} m2OpCreateFluidVolume;

typedef struct m2OpFluidSurface
{
    m2FluidVolumeId id;
    double surface;
} m2OpFluidSurface;

typedef struct m2OpShatterHeader
{
    m2BodyId body;
    int32_t pieceCount;
    int32_t expectedFirst; // index1 of the first piece body
} m2OpShatterHeader;

typedef struct m2OpFlag
{
    uint8_t flag;
} m2OpFlag;

typedef struct m2OpVec
{
    m2Vec2 value;
} m2OpVec;

typedef struct m2OpSetWind
{
    m2Vec2 velocity;
    float linearDrag;
} m2OpSetWind;

// Every fixed payload in one union: replay copies each record into it,
// so apply functions read aligned structs, and its size bounds them all.
typedef union m2OpPayload
{
    m2OpStep step;
    m2OpCreateBody createBody;
    m2OpBodyVec bodyVec;
    m2OpBodyFloat bodyFloat;
    m2OpBodyPoint bodyPoint;
    m2OpBodyParam bodyParam;
    m2OpBodyByte bodyByte;
    m2OpBodyUserData bodyUserData;
    m2OpSetTransform setTransform;
    m2OpSetMassData setMassData;
    m2OpCreateShape createShape;
    m2OpShapeParam shapeParam;
    m2OpShapeFloat shapeFloat;
    m2OpSetFilter setFilter;
    m2OpSetGeometry setGeometry;
    m2OpShapeUserData shapeUserData;
    m2OpChainHeader chainHeader;
    m2OpChainFloat chainFloat;
    m2OpCreateDistanceJoint createDistanceJoint;
    m2OpCreateRevoluteJoint createRevoluteJoint;
    m2OpCreatePrismaticJoint createPrismaticJoint;
    m2OpCreateWeldJoint createWeldJoint;
    m2OpCreateWheelJoint createWheelJoint;
    m2OpCreateFilterJoint createFilterJoint;
    m2OpCreateMotorJoint createMotorJoint;
    m2OpCreateMouseJoint createMouseJoint;
    m2OpCreateGearJoint createGearJoint;
    m2OpCreatePulleyJoint createPulleyJoint;
    m2OpCreateRatchetJoint createRatchetJoint;
    m2OpJointParam jointParam;
    m2OpJointUserData jointUserData;
    m2OpMotorOffsets motorOffsets;
    m2OpMouseTarget mouseTarget;
    m2OpEmitParticle emitParticle;
    m2OpFillParticles fillParticles;
    m2OpParticleVec particleVec;
    m2OpParticleFloat particleFloat;
    m2OpParticleUserData particleUserData;
    m2OpCreateFluidVolume createFluidVolume;
    m2OpFluidSurface fluidSurface;
    m2OpShatterHeader shatterHeader;
    m2OpFlag flag;
    m2OpVec vec;
    m2OpSetWind setWind;
    m2ExplosionDef explosion;
    m2BodyId bodyId;
    m2ShapeId shapeId;
    m2ChainId chainId;
    m2JointId jointId;
    m2ParticleId particleId;
    m2FluidVolumeId fluidVolumeId;
    int32_t restoreSize;
} m2OpPayload;

// Recording (src/journal.c). Each call appends one op; a full buffer
// sets the overflow flag, reported by m2World_StopJournal.
void m2JournalRecord(m2World* world, uint8_t op, const void* payload, int32_t bytes);
void m2JournalRecordRestore(m2World* world, const void* snapshot, int32_t size);
void m2JournalRecordShatter(m2World* world, m2BodyId bodyId, const m2Polygon* pieces,
                            int32_t pieceCount, int32_t expectedFirst);
void m2JournalRecordChain(m2World* world, m2BodyId bodyId, const m2ChainDef* def,
                          int32_t createdCount);

// Replay (src/journal_replay.c). The cursor walks one tape; an op's
// apply function receives its fixed payload already copied out and
// aligned, and variable-length ops consume their tail from the cursor.
typedef struct m2ReplayCursor
{
    m2WorldId worldId;
    m2World* world;
    uint16_t here; // the target world's id field, rebinding every id
    const uint8_t* data;
    int32_t size;
    int32_t offset;
} m2ReplayCursor;

typedef bool m2ReplayApplyFn(m2ReplayCursor* cursor, const m2OpPayload* payload);

typedef struct m2JournalCommand
{
    int32_t payloadSize; // fixed part; 0 marks an unknown op
    m2ReplayApplyFn* apply;
} m2JournalCommand;

// The command for an op code, or NULL for a code no build ever wrote.
const m2JournalCommand* m2JournalCommandFor(uint8_t op);

#endif // MAUL2D_SRC_JOURNAL_H
