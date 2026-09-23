// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The test harness shared by every suite in the Maul family: CHECK
// records a failure with its location and keeps going, and the suite's
// exit code reports whether any check failed.

#ifndef MAUL_TEST_HARNESS_H
#define MAUL_TEST_HARNESS_H

#include <stdio.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

#endif // MAUL_TEST_HARNESS_H
