// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The SIMD backend report and the CPU check. This file is compiled
// without the backend's architecture flags, so the check itself runs on
// any CPU of the target architecture; the build tells it which backend
// the rest of the library was compiled for. It reports and prints
// nothing: a library with a world refuses it on a mismatch, one without
// returns the mismatch from its own entry point.

#include "cpu.h"

#include <stddef.h>

#if defined(MAUL2D_BUILT_FOR_AVX2) && defined(_MSC_VER)
#include <intrin.h> // __cpuid, __cpuidex, _xgetbv
#endif

const char* m2GetSimdBackend(void)
{
#if defined(MAUL2D_BUILT_FOR_AVX2)
    return "avx2";
#elif !defined(MAUL2D_SIMD_FORCE_SCALAR) && (defined(__aarch64__) || defined(_M_ARM64))
    return "neon";
#else
    return "scalar";
#endif
}

int32_t m2CpuSupportsBackend(void)
{
#if defined(MAUL2D_BUILT_FOR_AVX2)
    // The AVX2 kernels need AVX2 and FMA3, and the operating system must
    // have enabled the wide registers (XGETBV). GCC and Clang check all of
    // it with one builtin; MSVC needs cpuid and xgetbv spelled out.
#if defined(_MSC_VER)
    int regs[4];
    __cpuid(regs, 0);
    if (regs[0] < 7)
    {
        return 0; // no leaf 7, so no AVX2
    }
    __cpuidex(regs, 1, 0);
    int osxsave = (regs[2] >> 27) & 1;
    int fma = (regs[2] >> 12) & 1;
    int avx = (regs[2] >> 28) & 1;
    if (!(osxsave && avx && fma))
    {
        return 0;
    }
    unsigned long long xcr = _xgetbv(0);
    if ((xcr & 0x6) != 0x6)
    {
        return 0; // the OS has not enabled XMM and YMM state
    }
    __cpuidex(regs, 7, 0);
    return ((regs[1] >> 5) & 1) ? 1 : 0; // leaf 7 EBX bit 5 = AVX2
#else
    return (__builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma")) ? 1 : 0;
#endif
#else
    // NEON is architectural on arm64, and scalar and wasm run anywhere.
    return 1;
#endif
}

int32_t m2CheckCpu(const char** backend)
{
    if (backend != NULL)
    {
        *backend = m2GetSimdBackend();
    }
    return m2CpuSupportsBackend();
}
