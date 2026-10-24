// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The floating-point environment's entries (a family file), through the
// module's own functions under every host test_fenv.h plays: every
// rounding mode, each with flush-to-zero and denormals-are-zero off and
// on. Inside an entry a worker computes the library's bits, a subnormal
// product flush-to-zero would lose and a sum every rounding mode rounds
// apart; outside, the host's setting comes back as it was.

#include "fpenv.h"
#include "test_fenv.h"
#include "test_harness.h"

#include <stdint.h>
#include <string.h>

// Inputs the compiler cannot fold.
static volatile double s_tiny = 0x1p-1022;
static volatile double s_eighth = 0x1p-3;
static volatile double s_one = 1.0;
static volatile double s_nudge = 0x1.8p-53;
static volatile double s_quarter = 0x1p-54;
static volatile double s_subnormal = 0x1p-1040;
static volatile double s_large = 0x1p100;

typedef struct Bits
{
    double product;
    double input;
    double sum;
    double small;
} Bits;

// The library's arithmetic, out of line so it stays inside the entry.
static M2_NOINLINE Bits Compute(void)
{
    Bits bits = {s_tiny * s_eighth, s_subnormal * s_large, s_one + s_nudge, s_one + s_quarter};
    return bits;
}

// What the library's environment gives: the subnormal product 2^-1025
// (flush-to-zero gives 0), 2^-940 from a subnormal input
// (denormals-are-zero gives 0), and 1 + 0.75 ulp and 1 + 0.25 ulp rounded
// to the nearest, 1 + 2^-52 and 1 (down and toward zero give 1 for the
// first, up 1 + 2^-52 for the second).
static bool LibraryBits(Bits bits)
{
    return bits.product == 0x1p-1025 && bits.input == 0x1p-940 && bits.sum == 1.0 + 0x1p-52 &&
           bits.small == 1.0;
}

static bool IsHost(TestFloatSetting host)
{
    return TestFloatSame(TestFloatGet(), host) && m2FloatEnvDepth() == 0;
}

static void TestEnterAndLeave(TestFloatSetting host)
{
    TestFloatSet(host);
    uint64_t token = m2FloatEnvEnter();
    Bits bits = Compute();
    uint32_t depth = m2FloatEnvDepth();
    m2FloatEnvLeave(token);
    CHECK(LibraryBits(bits) && depth == 1, "an entry computes the library's bits");
    CHECK(IsHost(host), "and its leave gives the host its setting back");
}

static void TestNesting(TestFloatSetting host)
{
    TestFloatSet(host);
    uint64_t outer = m2FloatEnvEnter();
    uint64_t inner = m2FloatEnvEnter();
    uint32_t depth = m2FloatEnvDepth();
    m2FloatEnvLeave(inner);
    Bits between = Compute();
    uint32_t after = m2FloatEnvDepth();
    m2FloatEnvLeave(outer);
    CHECK(depth == 2 && after == 1 && LibraryBits(between),
          "an inner leave leaves the outer entry's environment");
    CHECK(IsHost(host), "the outer leave gives the host its setting");
}

typedef struct Callback
{
    TestFloatSetting host;
    TestFloatSetting seen;
    uint32_t depth;
    Bits again;
    uint32_t againDepth;
    bool hostAfter;
    bool change;
    TestFloatSetting changeTo;
} Callback;

// Host code called the way the library calls it: the pair around the one
// call, in a helper of its own.
static M2_NOINLINE void InHost(void (*call)(void* context), void* context)
{
    m2FloatHost host = m2FloatHostEnter();
    call(context);
    m2FloatHostLeave(host);
}

static void HostCode(void* context)
{
    Callback* callback = (Callback*)context;
    callback->seen = TestFloatGet();
    callback->depth = m2FloatEnvDepth();
    // The host calls the library again: an outermost entry of its own.
    uint64_t token = m2FloatEnvEnter();
    callback->again = Compute();
    callback->againDepth = m2FloatEnvDepth();
    m2FloatEnvLeave(token);
    callback->hostAfter = TestFloatSame(TestFloatGet(), callback->host);
    if (callback->change)
    {
        TestFloatSet(callback->changeTo);
    }
}

static void TestHostCode(TestFloatSetting host, TestFloatSetting other)
{
    TestFloatSet(host);
    Callback callback = {host, {0, 0}, 9, {0.0, 0.0, 0.0, 0.0}, 0, false, false, {0, 0}};
    uint64_t token = m2FloatEnvEnter();
    InHost(HostCode, &callback);
    Bits after = Compute();
    uint32_t depth = m2FloatEnvDepth();
    m2FloatEnvLeave(token);
    CHECK(TestFloatSame(callback.seen, host) && callback.depth == 0,
          "host code runs in the host's own setting");
    CHECK(LibraryBits(callback.again) && callback.againDepth == 1 && callback.hostAfter,
          "a call back in computes the library's bits and gives the host code its setting");
    CHECK(LibraryBits(after) && depth == 1, "after host code, the library's environment again");
    CHECK(IsHost(host), "and the host's setting at the end");
    // Host code that changes its setting: the outermost leave restores the
    // host's from its entry all the same.
    TestFloatSet(host);
    // Only the register changes: the host's <fenv.h> mode (x87's control
    // word on x86) is the host code's own, which the library never sets.
    callback.change = true;
    callback.changeTo = (TestFloatSetting){host.rounding, other.control};
    callback.host = host;
    token = m2FloatEnvEnter();
    InHost(HostCode, &callback);
    after = Compute();
    m2FloatEnvLeave(token);
    CHECK(LibraryBits(after), "a setting host code changed is not the library's");
    CHECK(IsHost(host), "the outermost leave restores the host's setting from its entry");
    // Outside any entry host code is just called.
    TestFloatSet(host);
    callback.change = false;
    InHost(HostCode, &callback);
    CHECK(TestFloatSame(callback.seen, host) && IsHost(host), "outside an entry, just a call");
}

typedef struct Task
{
    Bits bits;
    uint32_t depth;
} Task;

static void TaskBody(void* context)
{
    Task* task = (Task*)context;
    task->bits = Compute();
    task->depth = m2FloatEnvDepth();
}

// A task body the host runs inline, from inside its task hook.
static void InlineTask(void* context)
{
    m2FloatEnvRun(TaskBody, context);
}

static void TestRun(TestFloatSetting host)
{
    TestFloatSet(host);
    Task task = {{0.0, 0.0, 0.0, 0.0}, 0};
    m2FloatEnvRun(TaskBody, &task);
    CHECK(LibraryBits(task.bits) && task.depth == 1 && IsHost(host),
          "a task body on a host thread computes the library's bits");
    Task inlined = {{0.0, 0.0, 0.0, 0.0}, 0};
    uint64_t token = m2FloatEnvEnter();
    InHost(InlineTask, &inlined);
    m2FloatEnvLeave(token);
    CHECK(LibraryBits(inlined.bits) && inlined.depth == 1 && IsHost(host),
          "and one the host runs inline in its task hook");
}

// A call that leaves on each of its paths, as the library's do: refused
// early, or after its work.
static int Call(bool refuse)
{
    uint64_t token = m2FloatEnvEnter();
    if (refuse)
    {
        m2FloatEnvLeave(token);
        return -1;
    }
    (void)Compute();
    m2FloatEnvLeave(token);
    return 0;
}

static void TestEveryPath(TestFloatSetting host)
{
    TestFloatSet(host);
    uint64_t token = m2FloatEnvEnter();
    int refused = Call(true);
    uint32_t depth = m2FloatEnvDepth();
    int done = Call(false);
    m2FloatEnvLeave(token);
    CHECK(refused == -1 && done == 0 && depth == 1 && IsHost(host),
          "a refusal's early leave keeps the depth and the host's setting");
}

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
static volatile double s_three = 3.0;

static M2_NOINLINE double Third(void)
{
    return s_one / s_three;
}

// MXCSR's sticky flags come back as the host had them: its invalid flag
// kept, the precision flag the library's division raises not left behind.
static void TestStickyFlags(void)
{
    const uint64_t flags = 0x3fu;
    const uint64_t invalid = 1u << 0;
    uint64_t start = m2FloatControlRead();
    m2FloatControlWrite((start & ~flags) | invalid);
    uint64_t token = m2FloatEnvEnter();
    double third = Third();
    uint64_t inside = m2FloatControlRead() & flags;
    m2FloatEnvLeave(token);
    uint64_t after = m2FloatControlRead() & flags;
    m2FloatControlWrite(start);
    CHECK(third < 0.34 && (inside & (1u << 5)) != 0, "the library's division raises precision");
    CHECK(after == invalid, "the leave gives the host back its own flags");
}
#else
static void TestStickyFlags(void)
{
}
#endif

static void TestTokens(void)
{
    // The register the token holds stays clear of its mark.
    CHECK((m2FloatControlRead() >> 63) == 0, "the control register clears bit 63");
    // As deep as the library could go, every leave still unwinds.
    enum
    {
        s_deep = 64
    };
    uint64_t tokens[s_deep];
    for (int i = 0; i < s_deep; ++i)
    {
        tokens[i] = m2FloatEnvEnter();
    }
    uint32_t deepest = m2FloatEnvDepth();
    for (int i = s_deep; i-- > 0;)
    {
        m2FloatEnvLeave(tokens[i]);
    }
    CHECK(deepest == s_deep && m2FloatEnvDepth() == 0, "64 entries deep and back");
}

int main(void)
{
    TestFloatSetting start = TestFloatGet();
    int hosts = TestFloatHostileCount();
    for (int h = 0; h < hosts; ++h)
    {
        TestFloatSetting host = TestFloatHostileAt(h);
        TestFloatSetting other = TestFloatHostileAt((h + 1) % hosts);
        TestEnterAndLeave(host);
        TestNesting(host);
        TestHostCode(host, other);
        TestRun(host);
        TestEveryPath(host);
    }
    TestFloatSet(start);
    TestTokens();
    TestStickyFlags();
    CHECK(IsHost(start), "the thread's own setting at the end");
    return s_failures == 0 ? 0 : 1;
}
