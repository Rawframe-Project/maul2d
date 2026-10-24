// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A host's floating-point setting for the tests (a family file), set and
// read without the library's own code: the rounding mode through <fenv.h>, and the control
// register's own rounding and flush-to-zero bits (MXCSR's rounding,
// flush-to-zero and denormals-are-zero on x86, FPCR's RMode and FZ on
// AArch64), read from the register itself: on x86 <fenv.h> may report the
// x87 control word alone, and the library's arithmetic follows MXCSR. A
// test plays a host that runs its threads with flush-to-zero and rounding
// toward zero, and checks every call leaves the setting as it found it;
// TestFloatHostileAt walks every rounding mode, each with and without the
// flush bits, for a test that runs a call under each.
// The MSVC paths are built by the clang-cl cell on x64; no CI cell builds
// MSVC for ARM64, so its _ReadStatusReg path is untried.

#ifndef MAUL2D_TEST_FENV_H
#define MAUL2D_TEST_FENV_H

#include <fenv.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(_MSC_VER) && !defined(__clang__) && (defined(_M_X64) || defined(_M_IX86))
#include <xmmintrin.h>
#elif defined(_MSC_VER) && defined(_M_ARM64)
#include <intrin.h>
#endif

typedef struct TestFloatSetting
{
    // As <fenv.h> reports it.
    int rounding;
    // The register's rounding and flush bits, the rest masked off.
    uint64_t control;
} TestFloatSetting;

// MXCSR's flush-to-zero and denormals-are-zero bits and its rounding
// field; FPCR's FZ and RMode. Toward zero is 3 in either field.
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#define TEST_FLUSH_BITS    ((1u << 15) | (1u << 6))
#define TEST_ROUNDING_BITS (3u << 13)
#define TEST_HAS_FLUSH     1
#elif defined(__aarch64__) || defined(_M_ARM64)
#define TEST_FLUSH_BITS    (1ull << 24)
#define TEST_ROUNDING_BITS (3ull << 22)
#define TEST_HAS_FLUSH     1
#else
#define TEST_HAS_FLUSH 0
#endif

#if TEST_HAS_FLUSH
// Volatile asm with a memory clobber where the compiler allows it, so a
// read is never reused across a call into the library.
static inline uint64_t TestControlRead(void)
{
#if defined(__x86_64__) || defined(__i386__)
    uint32_t value = 0;
    __asm__ volatile("stmxcsr %0" : "=m"(value) : : "memory");
    return value;
#elif defined(_M_X64) || defined(_M_IX86)
    return _mm_getcsr();
#elif defined(__aarch64__)
    uint64_t value = 0;
    __asm__ volatile("mrs %0, fpcr" : "=r"(value) : : "memory");
    return value;
#else
    return (uint64_t)_ReadStatusReg(ARM64_SYSREG(3, 3, 4, 4, 0));
#endif
}

static inline void TestControlWrite(uint64_t value)
{
#if defined(__x86_64__) || defined(__i386__)
    uint32_t control = (uint32_t)value;
    __asm__ volatile("ldmxcsr %0" : : "m"(control) : "memory");
#elif defined(_M_X64) || defined(_M_IX86)
    _mm_setcsr((unsigned int)value);
#elif defined(__aarch64__)
    __asm__ volatile("msr fpcr, %0" : : "r"(value) : "memory");
#else
    _WriteStatusReg(ARM64_SYSREG(3, 3, 4, 4, 0), (__int64)value);
#endif
}
#endif

// The thread's setting as it is.
static inline TestFloatSetting TestFloatGet(void)
{
    TestFloatSetting setting = {fegetround(), 0};
#if TEST_HAS_FLUSH
    setting.control = TestControlRead() & (TEST_FLUSH_BITS | TEST_ROUNDING_BITS);
#endif
    return setting;
}

// Sets the thread's rounding mode, then the register's rounding and flush
// bits, leaving its others as they are.
static inline void TestFloatSet(TestFloatSetting setting)
{
    (void)fesetround(setting.rounding);
#if TEST_HAS_FLUSH
    uint64_t mask = TEST_FLUSH_BITS | TEST_ROUNDING_BITS;
    TestControlWrite((TestControlRead() & ~mask) | (setting.control & mask));
#endif
}

static inline bool TestFloatSame(TestFloatSetting a, TestFloatSetting b)
{
    return a.rounding == b.rounding && a.control == b.control;
}

// The hostile host: flush-to-zero and rounding toward zero where the
// platform takes them, in <fenv.h> and in the register, else the setting
// as it is.
static inline TestFloatSetting TestFloatHostile(void)
{
    TestFloatSetting setting = TestFloatGet();
#ifdef FE_TOWARDZERO
    int saved = fegetround();
    if (fesetround(FE_TOWARDZERO) == 0 && fegetround() == FE_TOWARDZERO)
    {
        setting.rounding = FE_TOWARDZERO;
    }
    (void)fesetround(saved);
#endif
#if TEST_HAS_FLUSH
    setting.control = TEST_FLUSH_BITS | TEST_ROUNDING_BITS;
#endif
    return setting;
}

// The rounding modes <fenv.h> names here, nearest first.
static inline int TestFloatRoundings(int* modes)
{
    int count = 0;
    modes[count++] = FE_TONEAREST;
#ifdef FE_DOWNWARD
    modes[count++] = FE_DOWNWARD;
#endif
#ifdef FE_UPWARD
    modes[count++] = FE_UPWARD;
#endif
#ifdef FE_TOWARDZERO
    modes[count++] = FE_TOWARDZERO;
#endif
    return count;
}

// How many hosts TestFloatHostileAt plays: every rounding mode, each with
// the flush bits off and on where the platform has them.
static inline int TestFloatHostileCount(void)
{
    int modes[4];
    return TestFloatRoundings(modes) * (TEST_HAS_FLUSH ? 2 : 1);
}

// The index-th host: a rounding mode as <fenv.h> sets it, in <fenv.h>
// and in the register (read back after setting it, so no encoding of the
// register's field is assumed), and the flush bits for odd indices. The
// thread's setting is left as it was.
static inline TestFloatSetting TestFloatHostileAt(int index)
{
    int modes[4];
    int count = TestFloatRoundings(modes);
    TestFloatSetting saved = TestFloatGet();
    int flushes = TEST_HAS_FLUSH ? 2 : 1;
    (void)fesetround(modes[(index / flushes) % count]);
    TestFloatSetting setting = TestFloatGet();
    TestFloatSet(saved);
#if TEST_HAS_FLUSH
    setting.control =
        (setting.control & TEST_ROUNDING_BITS) | ((index % flushes) == 1 ? TEST_FLUSH_BITS : 0u);
#endif
    return setting;
}

#endif // MAUL2D_TEST_FENV_H
