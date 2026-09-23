// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Base definitions shared by every Maul2D header: the version, the
// result codes, the allocator and assert hooks, and hashing.

#ifndef MAUL2D_BASE_H
#define MAUL2D_BASE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define M2_VERSION_MAJOR 0
#define M2_VERSION_MINOR 0
#define M2_VERSION_PATCH 1

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
    /// Thread class: reader (callable from any thread, no world required).
    int32_t m2GetVersion(void);

    /// The SIMD backend this library was COMPILED against: "avx2", "neon"
    /// or "scalar". It is a compile-time choice, and every backend produces
    /// bit-identical results by contract; this only reports which kernels
    /// the binary carries. Thread class: reader.
    const char* m2GetSimdBackend(void);

    /// Whether the CPU running this call actually supports the compiled
    /// backend: 1 if it can run, 0 if not. An "avx2" binary needs AVX2 and
    /// FMA3 with OS wide-register support; "neon" is architectural on
    /// arm64 and "scalar" runs anywhere, so both return 1. Creating a world
    /// on a CPU that returns 0 aborts loudly rather than trapping on an
    /// illegal instruction; check this first for a graceful path, or build
    /// with -DMAUL2D_SIMD=scalar for a portable binary. Thread class: reader.
    int32_t m2CpuSupportsBackend(void);

    /// Routes every internal allocation through your hooks. Set BEFORE
    /// creating any world and never change it while worlds exist. The
    /// zeroFn contract: returned memory must be zero-initialized.
    typedef void* m2AllocZeroedFn(size_t bytes);
    typedef void m2FreeFn(void* memory);
    void m2SetAllocator(m2AllocZeroedFn* allocZeroed, m2FreeFn* freeFn);

    /// FNV-1a 64-bit hash over a byte range. This is the hash used by the
    /// determinism gates; its constants are frozen and will never change.
    /// Thread class: reader (pure function).
    uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount);

/// Seed value for m2Hash64 chains (FNV-1a offset basis).
#define M2_HASH_INIT 14695981039346656037ULL

    /// Why the last call on this thread refused. Every API that rejects
    /// its input (a bad def, a stale or wrong-kind id, a full pool)
    /// records the reason here before it returns its null result. The
    /// slot is per thread and success paths do not clear it: read it
    /// right after the refusal you care about.
    typedef enum m2Result
    {
        m2_success = 0,
        m2_errorInvalid = 1,  // bad def or argument, stale id, wrong body or joint type
        m2_errorCapacity = 2, // a fixed pool, slot table or allocation ran out
        m2_errorConfig = 3,   // the CPU cannot run the compiled SIMD backend
    } m2Result;
    m2Result m2LastResult(void);

    /// Host assert hook (integration audit A2/A5), contextful from
    /// day one: called before the default print-and-abort for every
    /// internal assertion failure AND for the create-time CPU
    /// backend refusal. Return nonzero to declare the failure
    /// handled and suppress the abort (crash reporters, test
    /// harnesses, engine diagnostics). NULL restores the default.
    /// Observer machinery: never touches simulation state.
    typedef int m2AssertFn(const char* condition, const char* file, int line, void* context);
    void m2SetAssertHandler(m2AssertFn* handler, void* context);

    /// Internal assertion failure sink (debug builds only). Prints and traps.
    void m2AssertFail(const char* condition, const char* file, int line);

#if defined(NDEBUG)
#define M2_ASSERT(cond) ((void)0)
#else
#define M2_ASSERT(cond) ((cond) ? (void)0 : m2AssertFail(#cond, __FILE__, __LINE__))
#endif

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BASE_H
