// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fixture for tools/check_test_hygiene.py --self-test: each line marked
// "expect" must be reported with that rule, and no other line. It is
// read, never compiled.

#include <stdlib.h>
#include <threads.h>
#include <time.h>

int Bad(void);

int Bad(void)
{
    sleep(1);                     // expect: sleep
    nanosleep(nullptr, nullptr);  // expect: sleep
    thrd_sleep(nullptr, nullptr); // expect: sleep
    time_t now = time(nullptr);   // expect: wall-clock
    struct timespec real;
    clock_gettime(CLOCK_REALTIME, &real);         // expect: wall-clock
    clock_gettime(CLOCK_REALTIME_COARSE, &real);  // expect: wall-clock
    clock_gettime(CLOCK_TAI, &real);              // expect: wall-clock
    timespec_get(&real, TIME_UTC);                // expect: wall-clock
    for (int retries = 0; retries < 3; retries++) // expect: retry
    {
    }
    int maxTries = 3;     // expect: retry
    bool tryAgain = true; // expect: retry
    retries = 0;          // expect: retry
    int retryDue = 0;     // expect: retry
    // A marker must be in a comment and give a reason of three words.
    const char* text = "// hygiene: a marker in a string does not count";
    usleep(1); // expect: sleep
    // hygiene: too short
    usleep(2); // expect: sleep
    // hygiene:
    exit(77);                // expect: skip
    exit(now == 0 ? 77 : 1); // expect: skip
    _Exit(77);               // expect: skip
    GTEST_SKIP();            // expect: skip
    (void)now;
    (void)text;
    if (now == 1)
    {
        return SKIP_CODE; // expect: skip
    }
    if (now == 2)
    {
        return now > 3 ? 1 : SKIP; // expect: skip
    }
    if (now == 4)
    {
        return now > 5 ? SKIP : 1; // expect: skip
    }
    return 77; // expect: skip
}
