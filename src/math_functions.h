// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The engine's own angle functions, for code already inside the
// floating-point environment: the public m2UnwindAngle, m2MakeRot,
// m2Atan2, m2NormalizeRot and m2MulRot enter it around these, and the
// solver's passes call them directly instead of entering again.

#ifndef MAUL2D_SRC_MATH_FUNCTIONS_H
#define MAUL2D_SRC_MATH_FUNCTIONS_H

#include "fpenv.h"

#include "maul2d/core_math.h"

M2_NOINLINE float m2UnwindAngleWork(float radians);
M2_NOINLINE m2Rot m2MakeRotWork(float radians);
M2_NOINLINE float m2Atan2Work(float y, float x);
M2_NOINLINE m2Rot m2NormalizeRotWork(m2Rot q);
M2_NOINLINE m2Rot m2MulRotWork(m2Rot q, m2Rot r);

#endif // MAUL2D_SRC_MATH_FUNCTIONS_H
