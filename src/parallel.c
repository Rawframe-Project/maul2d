// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The task dispatch: the host's task hooks when it installed them, serial
// in the caller otherwise.

#include "parallel.h"

#include "fpenv.h"

// One dispatch's body and context, which every range the host runs
// shares.
typedef struct TaskCall
{
    m2TaskFn* fn;
    void* context;
} TaskCall;

typedef struct TaskRange
{
    const TaskCall* call;
    int32_t start;
    int32_t end;
} TaskRange;

static M2_NOINLINE void RunRange(void* context)
{
    const TaskRange* range = (const TaskRange*)context;
    range->call->fn(range->start, range->end, range->call->context);
}

// What the host's hooks run, on a thread in any floating-point mode: the
// range inside the library's environment.
static void RunInEnvironment(int32_t start, int32_t end, void* context)
{
    TaskRange range = {(const TaskCall*)context, start, end};
    m2FloatEnvRun(RunRange, &range);
}

// The hooks are host code: they run in the host's environment, each
// through a helper whose body is only that call.
static M2_NOINLINE void* Enqueue(const m2World* world, TaskCall* call, int32_t itemCount,
                                 int32_t minRange)
{
    m2FloatHost host = m2FloatHostEnter();
    void* task =
        world->enqueueTask(RunInEnvironment, itemCount, minRange, call, world->userTaskContext);
    m2FloatHostLeave(host);
    return task;
}

static M2_NOINLINE void Finish(const m2World* world, void* task)
{
    m2FloatHost host = m2FloatHostEnter();
    world->finishTask(task, world->userTaskContext);
    m2FloatHostLeave(host);
}

void m2RunParallel(m2World* world, m2TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange)
{
    if (itemCount <= 0)
    {
        return;
    }
    if (world->enqueueTask != NULL)
    {
        TaskCall call = {fn, ctx};
        Finish(world, Enqueue(world, &call, itemCount, minRange));
        return;
    }
    fn(0, itemCount, ctx);
}
