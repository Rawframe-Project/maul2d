// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The task dispatch: the host's task hooks when it installed them, serial
// in the caller otherwise.

#include "parallel.h"

void m2RunParallel(m2World* world, m2TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange)
{
    if (itemCount <= 0)
    {
        return;
    }
    if (world->enqueueTask != NULL)
    {
        void* task = world->enqueueTask(fn, itemCount, minRange, ctx, world->userTaskContext);
        world->finishTask(task, world->userTaskContext);
        return;
    }
    fn(0, itemCount, ctx);
}
