// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for graph_color.c.

#ifndef MAUL2D_SRC_GRAPH_COLOR_H
#define MAUL2D_SRC_GRAPH_COLOR_H

#include "contact_solver.h"

void m2ColorConstraints(m2World* world, m2ContactConstraint* constraints, int32_t count,
                        int32_t* colorStart);

#endif // MAUL2D_SRC_GRAPH_COLOR_H
