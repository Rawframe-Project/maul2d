// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's memory layout and nothing else: the global state and one
// block per subsystem. Declarations live with their modules (body.h,
// joint.h, solver.h, ...). Tests include this for white-box oracles; it
// is never installed.

#ifndef MAUL2D_SRC_WORLD_INTERNAL_H
#define MAUL2D_SRC_WORLD_INTERNAL_H

#include "core.h"
#include "dynamic_tree.h"
#include "geometry.h"
#include "maul2d/base.h"
#include "maul2d/body.h"
#include "maul2d/events.h"
#include "maul2d/joint.h"
#include "maul2d/particle.h"
#include "maul2d/world.h"

#define M2_TREE_COUNT 3 // one broadphase tree per body type

// Per-body motion lock bits (the angular lock is fixedRotation).
#define M2_LOCK_LINEAR_X 1u
#define M2_LOCK_LINEAR_Y 2u

// Bodies: parallel POD arrays at fixed capacity, and the id pool
// (FIFO free queue plus generations; saturated slots retire).
typedef struct m2Bodies
{
    int32_t bodyCapacity;
    int32_t maxBodyIndex; // high-water mark of used slots
    m2Transform* transforms;
    m2Vec2* linearVelocities;
    float* angularVelocities;
    float* gravityScales;
    float* linearDampings;   // Pade-damped in integrate (snapshot state)
    float* angularDampings;  // (snapshot state)
    uint8_t* fixedRotations; // invInertia forced 0 (snapshot state)
    uint8_t* motionLocks;    // M2_LOCK_LINEAR_X | M2_LOCK_LINEAR_Y (snapshot state)
    uint8_t* sleepEnables;   // 0 = this body never sleeps (snapshot state)
    m2Vec2* forces;          // accumulated, cleared at step end (snapshot state)
    uint8_t* disabled;       // 1 = outside simulation, shapes proxy-less (snapshot state)
    int8_t* dominances;      // contact priority (snapshot state)
    float* torques;          // (snapshot state)
    uint64_t* userData;
    uint8_t* types;
    uint8_t* alive;
    int32_t* bodyShapeHead; // head of the body's shape list (-1 = none)
    float* invMass;         // dynamic bodies: derived from shapes, floored
    float* invInertia;      // about the body origin; 0 = no rotation response
    m2Vec2* localCenters;   // body-frame center of mass (snapshot state)
    uint8_t* asleep;        // sleep flag (snapshot + hashed: the sleep law)
    float* sleepTimes;      // seconds under tolerance (snapshot + hashed)
    uint8_t* sleepStreak;   // step-ends asleep, saturating at 2 (snapshot state)
    uint8_t* bullets;       // isBullet flag (snapshot + hashed)
    uint16_t* generations;
    int32_t* freeQueue;
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;
} m2Bodies;

// Shapes: the same id discipline as bodies.
typedef struct m2Shapes
{
    int32_t shapeCapacity;
    int32_t maxShapeIndex;
    m2ShapeGeometry* shapeGeometry;
    float* shapeDensity;
    float* shapeFriction;
    float* shapeRestitution;
    uint64_t* shapeUserData;
    int32_t* shapeBody; // owning body index
    int32_t* shapeNext; // body's shape list linkage (-1 = end)
    uint8_t* shapeAlive;
    uint32_t* shapeCategory; // collision filter (snapshot state)
    uint32_t* shapeMask;
    int32_t* shapeGroup;
    uint8_t* shapeSensor;     // overlap-only shapes (snapshot state)
    float* shapeTangentSpeed; // conveyor surface speed (snapshot state)
    int32_t* shapeChain;      // owning chain slot, -1 = free-standing (snapshot state)
    uint16_t* shapeGenerations;
    int32_t* shapeFreeQueue;
    int32_t shapeFreeHead;
    int32_t shapeFreeTail;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;
} m2Shapes;

// Joints: the same id discipline; impulses are warm-start snapshot
// state. The per-body adjacency lists every live joint once under each
// of its bodies as edge 2 * joint + side (side 0 = body A), each list in
// ascending joint order; it is derived state, rebuilt after a restore.
typedef struct m2Joints
{
    int32_t jointCapacity;
    int32_t maxJointIndex;
    uint8_t* jointType;
    uint8_t* jointAlive;
    int32_t* jointBodyA;
    int32_t* jointBodyB;
    m2Vec2* jointLocalAnchorA;
    m2Vec2* jointLocalAnchorB;
    float* jointLength;
    float* jointHertz;
    float* jointDamping;
    float* jointHertz2; // weld: angular row (linear rides jointHertz)
    float* jointDamping2;
    m2Vec2* jointImpulse; // distance uses .x only
    uint8_t* jointFlags;  // bit0 enableMotor, bit1 enableLimit, bit2 enableSpring
    float* jointMotorSpeed;
    float* jointMaxMotor; // torque (revolute) or force (prismatic)
    float* jointLower;
    float* jointUpper;
    m2Vec2* jointLocalAxisA; // prismatic
    float* jointRefAngle;    // relative angle captured at create
    float* jointMotorImpulse;
    float* jointLowerImpulse;
    float* jointUpperImpulse;
    float* jointSpringImpulse; // revolute angular spring accumulator
    float* jointBreakForce;    // 0 = unbreakable (snapshot state)
    float* jointBreakTorque;
    uint8_t* jointCollide;   // 0 = connected bodies never pair (snapshot state)
    m2Pos2* jointTargets;    // mouse joints: world target (snapshot state)
    m2Pos2* jointTargetsB;   // pulley: second ground anchor (snapshot state)
    uint64_t* jointUserData; // opaque (snapshot state)
    uint16_t* jointGenerations;
    int32_t* jointFreeQueue;
    int32_t jointFreeHead;
    int32_t jointFreeTail;
    int32_t jointFreeCount;
    int32_t jointRetiredCount;
    int32_t* bodyJointHead; // first edge per body, -1 = none
    int32_t* jointEdgeNext; // next edge in the same body's list, -1 = end
} m2Joints;

// Chains: a chain is a named group of segment shapes on one body, so a
// whole ground run can be destroyed by id. Slots are bounded by shape
// capacity (every live chain owns a shape); same id discipline.
typedef struct m2Chains
{
    int32_t maxChainIndex;
    uint8_t* chainAlive;
    int32_t* chainBody;
    uint16_t* chainGenerations;
    int32_t* chainFreeQueue;
    int32_t chainFreeHead;
    int32_t chainFreeTail;
    int32_t chainFreeCount;
    int32_t chainRetiredCount;
} m2Chains;

// Particles: slot-stable SoA with FIFO recycling, never compacted; all
// snapshot state, walked only when capacity > 0. The pair, weight and
// particle-body arrays are step-transient scratch rebuilt from positions
// every step; the jelly springs and triads are persistent. The staging
// slabs give the parallel contact build 4 candidate slots per particle,
// compacted serially in index order, so any worker count agrees.
typedef struct m2Particles
{
    int32_t particleCapacity;
    int32_t particleCount;
    int32_t maxParticleIndex;
    m2Pos2* particlePositions;
    m2Vec2* particleVelocities;
    uint8_t* particleAlive;
    uint16_t* particleGenerations;
    uint32_t* particleFlags;    // behavior bits (snapshot state)
    float* particleLifetime;    // seconds left, 0 = immortal (snapshot state)
    uint64_t* particleUserData; // opaque game data (snapshot state)
    int32_t* particleFreeQueue;
    int32_t particleFreeHead;
    int32_t particleFreeCount;
    float particleRadius;
    float particleDensity;
    float particleGravityScale;
    float particlePressureStrength;
    float particleDampingStrength;
    float particleViscousStrength;
    float particleCohesion;
    float particleNearPressure;
    float particlePowderStrength;
    float particleSpringStrength;
    float particleElasticStrength;
    void* particleProxies;    // capacity * 16 bytes (key, index, pad)
    void* particleProxiesTmp; // radix ping-pong buffer, same size
    int32_t* particlePairA;
    int32_t* particlePairB;
    float* particlePairWeight;
    uint32_t* particlePairFlags; // OR of both ends (transient)
    m2Vec2* particlePairNormal;
    int32_t particlePairCapacity; // 12 per particle of capacity
    int32_t particlePairCount;
    int32_t particlePairOverflow;  // deterministic truncation counter
    float* particleWeights;        // step-transient dimensionless density
    float* particleAccumulation;   // step-transient pressure accumulator
    m2Vec2* particleAccumulation2; // step-transient tensile normals
    uint32_t particleFlagsUnion;   // OR over live pairs (transient)
    int32_t* particleSpringA;
    int32_t* particleSpringB;
    float* particleSpringRest;
    int32_t particleSpringCapacity; // 4 per particle of capacity
    int32_t particleSpringCount;
    int32_t* particleTriadA;
    int32_t* particleTriadB;
    int32_t* particleTriadC;
    m2Vec2* particleTriadPA; // rest offsets about the triad centroid
    m2Vec2* particleTriadPB;
    m2Vec2* particleTriadPC;
    int32_t particleTriadCapacity; // 2 per particle of capacity
    int32_t particleTriadCount;
    int32_t* particleBodyParticle;
    int32_t* particleBodyBody;
    float* particleBodyWeight;
    m2Vec2* particleBodyNormal;      // outward, shape toward particle, world frame
    float* particleBodyMass;         // pair-effective mass (reference formula)
    int32_t* particleBodyStageBody;  // 4 * capacity, -1 = empty
    float* particleBodyStageWeight;  // 4 * capacity
    m2Vec2* particleBodyStageNormal; // 4 * capacity
    float* particleBodyStageMass;    // 4 * capacity
    int32_t* particlePairWorkCount;  // per-proxy pair counts (two-pass build)
    int32_t* particleBodyStageDrops; // per-particle candidates beyond 4
    int32_t particleBodyCapacity;    // 4 per particle of capacity
    int32_t particleBodyCount;
    int32_t particleBodyOverflow;
    uint64_t particlePoolFullCount; // cumulative quiet-full refusals
} m2Particles;

// Fluid volumes: particle-free water regions (snapshot state).
typedef struct m2FluidVolumes
{
    int32_t fvCapacity;
    int32_t maxFvIndex;
    m2Pos2* fvLower;
    m2Pos2* fvUpper;
    double* fvSurface;
    float* fvDensity;
    float* fvLinearDrag;
    float* fvAngularDrag;
    m2Vec2* fvFlow;
    uint64_t* fvUserData;
    uint8_t* fvAlive;
    uint16_t* fvGenerations;
    int32_t* fvFreeQueue;
    int32_t fvFreeHead;
    int32_t fvFreeCount;
} m2FluidVolumes;

// Broadphase: one tree per body type whose leaves are shape proxies,
// and the move buffer the next pair update consumes.
typedef struct m2Broadphase
{
    m2DynamicTree trees[M2_TREE_COUNT];
    m2TreeNode* treeNodes[M2_TREE_COUNT];
    int32_t treeNodeCapacity;
    int32_t* proxyIds; // per shape slot; M2_NULL_NODE when absent
    uint8_t* inMoved;  // per shape slot dedup flag
    int32_t* moved;    // shape indices, consumed and cleared by step
    int32_t movedCount;
} m2Broadphase;

// Contacts: the sorted pair list and its manifolds (manifolds[i]
// belongs to pairKeys[i]); warm-start impulses live here, so the block
// is snapshot state. The scratch arrays are step-transient.
typedef struct m2Contacts
{
    uint64_t* pairKeys;    // sorted, deduplicated (minShape << 32 | maxShape)
    uint8_t* pairTouching; // manifold had points last step (snapshot state)
    int32_t pairCount;
    int32_t pairCapacity;
    uint64_t* pairScratch; // step-transient; not hashed, snapshot-benign
    m2Manifold* manifolds;
    int32_t oldPairCount;        // step-transient
    uint64_t* oldPairScratch;    // step-transient
    uint64_t* pairMergeScratch;  // step-transient: the merged pair list before it lands
    int32_t pairOverflow;        // candidate pairs dropped by the last update (table full)
    m2Manifold* manifoldScratch; // step-transient
} m2Contacts;

// Solver scratch: step-transient, zeroed at prepare, plus the last
// step's coloring diagnostics.
typedef struct m2SolverScratch
{
    m2Vec2* deltaPositions; // f32 position deltas within the step
    m2Rot* deltaRotations;
    void* constraintScratch;   // m2ContactConstraint[pairCapacity]
    void* contactBlocks;       // wide SoA blocks (step-transient)
    int32_t* islandParent;     // union-find scratch (step-transient)
    uint8_t* islandDisturbed;  // island flags scratch (step-transient)
    m2Pos2* ccdPrevPositions;  // bullet substep origins (step-transient)
    uint8_t* touchingScratch;  // pair-touching carry scratch (step-transient)
    uint32_t* colorMasks;      // per body: colors already used (step-transient)
    uint8_t* constraintColors; // per constraint (step-transient)
    int32_t* colorOrder;       // constraints sorted by color (step-transient)
    int32_t lastConstraintCount;
    int32_t lastGraphColors;
    int32_t lastOverflow;
} m2SolverScratch;

// Event buffers: the world-owned observer stream, cleared at step start
// and by restore; never snapshot state.
typedef struct m2Events
{
    m2ContactBeginEvent* beginEvents;
    int32_t beginEventCount;
    m2ContactEndEvent* endEvents;
    int32_t endEventCount;
    m2ContactEndEvent* pendingEndEvents; // between-step destroys, flushed at Step
    int32_t pendingEndCount;
    m2ContactBeginEvent* sensorBeginEvents;
    int32_t sensorBeginCount;
    m2ContactEndEvent* sensorEndEvents;
    int32_t sensorEndCount;
    m2ContactEndEvent* pendingSensorEnd;
    int32_t pendingSensorEndCount;
    m2JointBreakEvent* jointBreakEvents;
    int32_t jointBreakEventCount;
} m2Events;

// The journal recorder: observer state, never snapshot state.
typedef struct m2Recorder
{
    uint8_t* journal;
    int32_t journalCapacity;
    int32_t journalCursor;
    uint8_t journalActive;
    uint8_t journalOverflow;
} m2Recorder;

// The world: global state, then one block per subsystem. Every array
// is described once in the state table (world_state.c), which drives
// allocation, snapshots, hashing and memory accounting.
typedef struct m2World
{
    // Global mutable state (snapshot state).
    m2Vec2 gravity;
    m2Vec2 windVelocity;  // ambient wind; snapshot state
    float windLinearDrag; // 0 = wind off (opt-in); snapshot state
    uint64_t stepCount;
    uint8_t sleepEnabled; // world-wide sleep master switch (snapshot state)
    float lastInvH;       // inverse substep dt of the last solve (snapshot state)

    m2Bodies bodies;
    m2Shapes shapes;
    m2Joints joints;
    m2Chains chains;
    m2Particles particles;
    m2FluidVolumes volumes;
    m2Broadphase broadphase;
    m2Contacts contacts;
    m2SolverScratch solver;
    m2Events events;
    m2Recorder recorder;

    // Host hooks and diagnostics (never snapshot state).
    m2EnqueueTaskFn* enqueueTask; // host executor; both NULL =
    m2FinishTaskFn* finishTask;   // serial; never snapshot state
    void* userTaskContext;
    m2Profile profile; // diagnostics only (never walked/hashed)
    volatile long long
        misuseCount;     // cumulative refusals; atomic access only (m2Refuse, m2MisuseCount)
    int64_t memoryBytes; // persistent footprint, from create
    uint16_t worldGeneration;
    uint16_t worldIndex0; // registry slot + 1, for building public ids
} m2World;

#endif // MAUL2D_SRC_WORLD_INTERNAL_H
