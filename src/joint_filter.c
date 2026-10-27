// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The filter joint: no rows at all; it only stops its two bodies from
// colliding.

#include "body.h"
#include "broadphase.h"
#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "object_ids.h"
#include "solver_base.h"
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

// The create record holds the def's fields and zero bytes between them:
// copied field by field into the zeroed record, so no padding of the
// caller's struct reaches the tape.
static void RecordFilterJointDef(m2FilterJointDef* out, const m2FilterJointDef* def)
{
    out->bodyIdA = def->bodyIdA;
    out->bodyIdB = def->bodyIdB;
    out->userData = def->userData;
    out->internalValue = def->internalValue;
}

static M2_NOINLINE m2Result CreateFilterJointWorker(m2World* world, const m2FilterJointDef* def,
                                                    m2JointId* jointOut)
{
    if (jointOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *jointOut = m2_nullJointId;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (def == NULL || def->internalValue != M2_FJOINT_COOKIE)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0)
    {
        return m2Refuse(world, m2_errorStale);
    }
    if (bodyA == bodyB)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        return m2Refuse(world, m2_errorCapacity);
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = m2FinishJoint(world, index, (uint8_t)m2_filterJoint, bodyA, bodyB, zero,
                                      zero, 0.0f, 0.0f, 0.0f);
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = 0; // its entire purpose
    m2RefilterJointedBodies(world, bodyA, bodyB);
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateFilterJoint record;
        memset(&record, 0, sizeof(record));
        RecordFilterJointDef(&record.def, def);
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateFilterJoint, &record, (int32_t)sizeof(record));
    }
    *jointOut = jointId;
    return m2_success;
}

m2Result m2CreateFilterJoint(m2World* world, const m2FilterJointDef* def, m2JointId* jointOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateFilterJointWorker(world, def, jointOut);
    m2FloatEnvLeave(env);
    return result;
}

// No rows and no load: the solver skips it.
const m2JointKind m2_filterJointKind = {NULL, NULL, NULL, NULL};
