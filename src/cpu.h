// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The SIMD backend report and the CPU check (cpu.c), for the library's
// own sources. A library may also declare the first two in its public
// header.

#ifndef MAUL2D_SRC_CPU_H
#define MAUL2D_SRC_CPU_H

#include <stdint.h>

// The backend the library's vector code was built for: "avx2", "neon"
// or "scalar". The string is static.
const char* m2GetSimdBackend(void);

// 1 when this CPU runs the built backend, else 0.
int32_t m2CpuSupportsBackend(void);

// The CPU check without a world: stores the backend's name in *backend
// when backend is not null and returns 1 when this CPU runs it, 0 when
// it does not (the build should then be configured scalar). Safe from
// any thread; it keeps no state.
int32_t m2CheckCpu(const char** backend);

#endif
