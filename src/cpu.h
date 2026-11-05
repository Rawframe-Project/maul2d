// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The SIMD backend report and the CPU check (cpu.c), for the library's
// own sources. A library may also declare any of the three in its
// public header.

#ifndef MAUL2D_SRC_CPU_H
#define MAUL2D_SRC_CPU_H

#include "maul2d/base.h"

#include <stdbool.h>

// The backend the library's vector code was built for: "avx2", "neon"
// or "scalar". The string is static.
// NOLINTNEXTLINE(readability-redundant-declaration): base.h may declare it
const char* m2GetSimdBackend(void);

// Whether this CPU runs the built backend.
// NOLINTNEXTLINE(readability-redundant-declaration): base.h may declare it
M2_NODISCARD bool m2IsCpuBackendSupported(void);

// The CPU check without a world: stores the backend's name in *backend
// when backend is not null and returns whether this CPU runs it (when it
// does not, the build should be configured scalar). Safe from any thread;
// it keeps no state.
// NOLINTNEXTLINE(readability-redundant-declaration): base.h may declare it
M2_NODISCARD bool m2CheckCpu(const char** backend);

#endif
