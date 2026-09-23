// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The filter joint: no rows at all; it only stops its two bodies from
// colliding.

#include "joint_solver.h"

#include "body.h"
#include "broadphase.h"
#include "joint.h"
#include "journal.h"
#include "solver.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

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
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_filterJoint, bodyA, bodyB,
                                      zero, zero, 0.0f, 0.0f, 0.0f);
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = 0; // its entire purpose
    m2RefilterJointedBodies(world, bodyA, bodyB);
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateFilterJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateFilterJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// No rows and no load: the solver skips it.
const m2JointKind m2_filterJointKind = {NULL, NULL, NULL, NULL};
