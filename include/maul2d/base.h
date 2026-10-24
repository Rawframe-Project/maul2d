// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The base of the Maul2D API: the library version, the export and
// attribute macros, the result codes every fallible function returns,
// the allocator, and hashing.

#ifndef MAUL2D_BASE_H
#define MAUL2D_BASE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Marks the public API. Shared builds export it (dllexport and
/// dllimport on Windows, default visibility elsewhere); maul2d_EXPORTS
/// is defined by CMake while the library itself compiles.
#if defined(MAUL2D_SHARED) && defined(_WIN32)
#if defined(maul2d_EXPORTS)
#define M2_API __declspec(dllexport) extern
#else
#define M2_API __declspec(dllimport) extern
#endif
#elif defined(MAUL2D_SHARED) && (defined(__GNUC__) || defined(__clang__))
#define M2_API __attribute__((visibility("default"))) extern
#else
#define M2_API extern
#endif

// M2_NODISCARD marks a function whose result must be read: every
// function that returns a status. The attribute is standard in C23 and
// C++17 and left out for older dialects. MSVC keeps __cplusplus at
// 199711L unless /Zc:__cplusplus is given, so _MSVC_LANG is read too.
#if (defined(__cplusplus) && __cplusplus >= 201703L) ||                                            \
    (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
#define M2_NODISCARD [[nodiscard]]
#elif !defined(__cplusplus) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L
#define M2_NODISCARD [[nodiscard]]
#else
#define M2_NODISCARD
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// The library version. CMake reads it from here, and release-please
// moves it between the markers.
// x-release-please-start-major
#define M2_VERSION_MAJOR 0
// x-release-please-end
// x-release-please-start-minor
#define M2_VERSION_MINOR 0
// x-release-please-end
// x-release-please-start-patch
#define M2_VERSION_PATCH 1
// x-release-please-end

// The binary interface a program is built for: the whole version before
// 1.0, when any release may break it, and the major and minor version from
// 1.0 on. The top bit is always set, so a zeroed def never matches. The
// world def carries it (m2DefaultWorldDef), and m2CreateWorld refuses a
// def built for another; every other def is used inside a world.
#define M2_ABI_VERSION                                                                             \
    (UINT32_C(0x80000000) | ((uint32_t)M2_VERSION_MAJOR << 16) |                                   \
     (M2_VERSION_MAJOR == 0 ? (((uint32_t)M2_VERSION_MINOR << 8) | (uint32_t)M2_VERSION_PATCH)     \
                            : (uint32_t)M2_VERSION_MINOR))
#if M2_VERSION_MAJOR > 0x7FFF || M2_VERSION_MINOR > 0xFFFF
#error "M2_VERSION_MAJOR or M2_VERSION_MINOR does not fit M2_ABI_VERSION"
#endif
#if M2_VERSION_MAJOR == 0 && (M2_VERSION_MINOR > 0xFF || M2_VERSION_PATCH > 0xFF)
#error "before 1.0, M2_VERSION_MINOR and M2_VERSION_PATCH must fit 8 bits each"
#endif

    // The status a fallible function returns. Zero is success, negative
    // values are errors. The type has a fixed width so that structs holding
    // a result have one layout in C and C++.
    typedef int32_t m2Result;

    enum
    {
        // The call did what was asked.
        m2_success = 0,
        // An argument or def is invalid, or a snapshot or journal tape
        // fails its checks (another build, another world shape, damaged
        // bytes).
        m2_errorInvalid = -1,
        // A fixed pool of the world, a caller buffer or the allocator is
        // too small for the result.
        m2_errorCapacity = -2,
        // An id names an object that no longer exists.
        m2_errorStale = -3,
        // The CPU cannot run the SIMD backend the library was built for.
        m2_errorUnsupported = -4,
        // The call is not allowed now: from inside m2World_Step, a debug
        // draw callback or a task the world is running.
        m2_errorState = -6,
        // A world def was built from headers of another binary interface
        // than the library's (M2_ABI_VERSION).
        m2_errorVersion = -7,
    };

    // The allocator a world takes in its def and keeps for its lifetime.
    // Alignment is a power of two. A zeroed allocator means the C library's
    // allocation functions.
    typedef struct m2Allocator
    {
        void* (*alloc)(size_t size, size_t alignment, void* context);
        void (*free)(void* memory, size_t size, size_t alignment, void* context);
        void* context;
    } m2Allocator;

    // A library version: major, minor and patch.
    typedef struct m2Version
    {
        uint16_t major;
        uint16_t minor;
        uint16_t patch;
    } m2Version;

    /// Returns the version of the library that was linked, which may differ
    /// from the M2_VERSION macros a program was compiled with.
    ///
    /// @return The library version.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2Version m2GetVersion(void);

    /// Returns the name of a result code, for diagnostics.
    ///
    /// @param result  Any value; an unknown one is named as such.
    /// @return A static, NUL-terminated string such as "m2_errorCapacity".
    /// @par Thread safety
    /// Safe from any thread.
    M2_API const char* m2ResultName(m2Result result);

    /// Returns the SIMD backend the library was compiled for: "avx2",
    /// "neon" or "scalar". Every backend produces the same bits; this only
    /// reports which kernels the binary carries.
    ///
    /// @return A static, NUL-terminated string.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API const char* m2GetSimdBackend(void);

    /// Returns whether the CPU running the call supports the compiled
    /// backend. An "avx2" binary needs AVX2 and FMA3 with the wide registers
    /// enabled by the operating system; "neon" and "scalar" run on every CPU
    /// of their architecture. m2CreateWorld refuses a CPU without the
    /// backend with m2_errorUnsupported, so a program can call this first to
    /// choose another path, or build with -DMAUL2D_SIMD=scalar.
    ///
    /// @return 1 when the CPU can run this library, 0 when it cannot.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API int32_t m2CpuSupportsBackend(void);

    /// The 64-bit hash every determinism check is built on: eight bytes a
    /// round, xored in, multiplied by an odd constant and folded, the
    /// leftover bytes one at a time (FNV-1a). Its constants are frozen.
    ///
    /// @param seed       The running hash; start a chain with M2_HASH_INIT.
    /// @param data       The bytes to fold in; may be null when byteCount is 0.
    /// @param byteCount  The number of bytes, at least 0.
    /// @return The new running hash.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount);

// The seed of an m2Hash64 chain.
#define M2_HASH_INIT 0xCBF29CE484222325ull

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BASE_H
