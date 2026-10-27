// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// x86 keeps the environment in MXCSR: flush-to-zero is bit 15,
// denormals-are-zero bit 6, the rounding mode bits 13 and 14, the
// exception masks bits 7 to 12. On 32-bit x86 the C library's math
// functions also compute on the x87 unit (glibc's ldexp, for one),
// which rounds by its own control word: there the environment is MXCSR
// in the low 32 bits and the x87 control word in bits 32 to 47, set to
// round to nearest with every exception masked and 64-bit precision,
// the C library's own default there, which its math functions assume.
// AArch64 keeps it in FPCR: flush-to-zero is bit 24, half-precision
// flush-to-zero bit 19, default NaN bit 25, the rounding mode bits 22
// and 23, the trap enables bits 8 to 12 and 15, and with FEAT_AFP input
// flush and alternate handling bits 0 and 1. WebAssembly has one
// environment, the one the library needs. Any other architecture is
// refused at build time rather than computed in an unknown environment.
//
// The thread's place in the entries is a thread-local, the only state of
// src/ (the family's rules allow it here alone): it mirrors the control
// register, which is per thread too. An outermost entry's token is the
// host's register, marked by bit 63; a nested one's is 0. Returned by
// value, it takes no local's address.

#include "fpenv.h"

#include <float.h>

#if FLT_EVAL_METHOD != 0
#error "maul2d: float arithmetic must round to float (FLT_EVAL_METHOD 0)"
#endif

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)

// The register is read and written through volatile asm with a memory
// clobber: Clang treats _mm_getcsr as free of side effects and may reuse
// an earlier read, which would undo a restore.
#if defined(_MSC_VER) && !defined(__clang__)
#include <xmmintrin.h>
static uint32_t ReadMxcsr(void)
{
    return _mm_getcsr();
}
static void WriteMxcsr(uint32_t value)
{
    _mm_setcsr(value);
}
#else
static uint32_t ReadMxcsr(void)
{
    uint32_t value = 0;
    __asm__ volatile("stmxcsr %0" : "=m"(value) : : "memory");
    return value;
}
static void WriteMxcsr(uint32_t value)
{
    __asm__ volatile("ldmxcsr %0" : : "m"(value) : "memory");
}
#endif

// Clear flush-to-zero, denormals-are-zero and the rounding mode; mask
// every exception. MXCSR is 32 bits.
enum
{
    s_flushToZero = 1u << 15,
    s_denormalsAreZero = 1u << 6,
    s_roundingMode = 3u << 13,
    s_exceptionMasks = 0x3fu << 7,
};

#if defined(__i386__) || defined(_M_IX86)

// No CI cell builds MSVC for 32-bit x86, so its __asm path is untried.
#if defined(_MSC_VER) && !defined(__clang__)
// clang-format off: MSVC's __asm statement ends at the line's end.
static uint16_t ReadX87(void)
{
    uint16_t value = 0;
    __asm fnstcw value
    return value;
}
static void WriteX87(uint16_t value)
{
    __asm fldcw value
}
// clang-format on
#else
static uint16_t ReadX87(void)
{
    uint16_t value = 0;
    __asm__ volatile("fnstcw %0" : "=m"(value) : : "memory");
    return value;
}
static void WriteX87(uint16_t value)
{
    __asm__ volatile("fldcw %0" : : "m"(value) : "memory");
}
#endif

// The x87 control word: exception masks bits 0 to 5, precision bits 8
// and 9 (3 is 64-bit), rounding bits 10 and 11 (0 is to nearest).
enum
{
    s_x87Masks = 0x3fu,
    s_x87Precision = 3u << 8,
    s_x87Rounding = 3u << 10,
    s_controlBits = 0xffffu,
};

static uint64_t ReadControl(void)
{
    return ReadMxcsr() | ((uint64_t)ReadX87() << 32);
}
static void WriteControl(uint64_t value)
{
    WriteMxcsr((uint32_t)value);
    WriteX87((uint16_t)(value >> 32));
}
static uint64_t Wanted(uint64_t control)
{
    uint32_t clear = s_flushToZero | s_denormalsAreZero | s_roundingMode;
    uint32_t mxcsr = ((uint32_t)control & ~clear) | s_exceptionMasks;
    uint32_t x87 =
        ((uint32_t)(control >> 32) & ~(uint32_t)s_x87Rounding) | s_x87Masks | s_x87Precision;
    return mxcsr | ((uint64_t)(x87 & 0xffffu) << 32);
}

#else

enum
{
    s_controlBits = 0xffffu,
};

static uint64_t ReadControl(void)
{
    return ReadMxcsr();
}
static void WriteControl(uint64_t value)
{
    WriteMxcsr((uint32_t)value);
}
static uint64_t Wanted(uint64_t control)
{
    uint32_t clear = s_flushToZero | s_denormalsAreZero | s_roundingMode;
    return (control & ~(uint64_t)clear) | s_exceptionMasks;
}

#endif

#elif defined(__aarch64__) || defined(_M_ARM64)

#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
// FPCR as _ReadStatusReg names it: op0 3, op1 3, CRn 4, CRm 4, op2 0.
#define FPCR_REGISTER 0x5A20
static uint64_t ReadControl(void)
{
    return (uint64_t)_ReadStatusReg(FPCR_REGISTER);
}
static void WriteControl(uint64_t value)
{
    _WriteStatusReg(FPCR_REGISTER, (__int64)value);
}
#else
static uint64_t ReadControl(void)
{
    uint64_t value = 0;
    __asm__ volatile("mrs %0, fpcr" : "=r"(value) : : "memory");
    return value;
}
static void WriteControl(uint64_t value)
{
    __asm__ volatile("msr fpcr, %0" : : "r"(value) : "memory");
}
#endif

// Clear flush-to-zero (single and half, and input flush), alternate
// handling, default NaN, the rounding mode and the trap enables. FPCR's
// bits from 32 up are reserved, read as 0.
enum
{
    s_flushToZero = 1u << 24,
    s_flushToZeroHalf = 1u << 19,
    s_defaultNan = 1u << 25,
    s_roundingMode = 3u << 22,
    s_trapEnables = (0x1fu << 8) + (1u << 15),
    s_inputFlushAndAlternate = 3u,
    s_controlBits = (1u << 27) - 1u,
};

static uint64_t Wanted(uint64_t control)
{
    uint64_t clear = s_flushToZero | s_flushToZeroHalf | s_defaultNan | s_roundingMode |
                     s_trapEnables | s_inputFlushAndAlternate;
    return control & ~clear;
}

#elif defined(__wasm__)

static uint64_t ReadControl(void)
{
    return 0;
}

static void WriteControl(uint64_t value)
{
    (void)value;
}

enum
{
    s_controlBits = 0,
};

#else
#error "maul2d: unknown floating-point environment for this architecture"
#endif

// Every control bit the library reads or sets, the x87 word's included,
// is below the token's mark.
static_assert((uint64_t)s_controlBits <= UINT32_MAX, "the control bits are below bit 63");

// On WebAssembly the entries are fpenv.h's empty inline functions and
// keep no depth.
#if !defined(__wasm__)

#if defined(_MSC_VER) && !defined(__clang__)
#define M2_THREAD_LOCAL __declspec(thread)
#else
#define M2_THREAD_LOCAL _Thread_local
#endif

// The register goes through a stack slot (stmxcsr takes memory), which
// -fstack-protector-strong would guard with a canary in every entry; the
// slot holds a copy of the register and nothing more.
#if defined(__has_attribute)
#if __has_attribute(no_stack_protector)
#define M2_NO_CANARY __attribute__((no_stack_protector))
#endif
#endif
#ifndef M2_NO_CANARY
#define M2_NO_CANARY
#endif

// The thread's place: the host's control register as the outermost entry
// saved it, and how many entries deep.
static M2_THREAD_LOCAL m2FloatHost s_thread;

// An outermost entry's token: the host's register, marked.
static const uint64_t s_outermost = 1ull << 63;

M2_NO_CANARY uint64_t m2FloatEnvEnter(void)
{
    if (s_thread.depth > 0)
    {
        ++s_thread.depth;
        return 0;
    }
    uint64_t host = ReadControl();
    uint64_t wanted = Wanted(host);
    if (wanted != host)
    {
        WriteControl(wanted);
    }
    s_thread = (m2FloatHost){host, 1};
    return s_outermost | host;
}

M2_NO_CANARY void m2FloatEnvLeave(uint64_t token)
{
    if ((token & s_outermost) == 0)
    {
        --s_thread.depth;
        return;
    }
    uint64_t host = token & ~s_outermost;
    if (ReadControl() != host)
    {
        WriteControl(host);
    }
    s_thread.depth = 0;
}

M2_NO_CANARY m2FloatHost m2FloatHostEnter(void)
{
    m2FloatHost host = s_thread;
    if (host.depth != 0 && ReadControl() != host.saved)
    {
        WriteControl(host.saved);
    }
    s_thread.depth = 0;
    return host;
}

M2_NO_CANARY void m2FloatHostLeave(m2FloatHost host)
{
    s_thread = host;
    if (host.depth != 0)
    {
        uint64_t wanted = Wanted(host.saved);
        if (ReadControl() != wanted)
        {
            WriteControl(wanted);
        }
    }
}

#endif

void m2FloatEnvRun(void (*body)(void* context), void* context)
{
    uint64_t token = m2FloatEnvEnter();
    body(context);
    m2FloatEnvLeave(token);
}

uint64_t m2FloatControlRead(void)
{
    return ReadControl();
}

void m2FloatControlWrite(uint64_t value)
{
    WriteControl(value);
}

uint32_t m2FloatEnvDepth(void)
{
#if defined(__wasm__)
    return 0;
#else
    return s_thread.depth;
#endif
}
