// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library version and the names of the result codes.

#include "core.h"

#include "maul2d/base.h"

m2Version m2GetVersion(void)
{
    return (m2Version){M2_VERSION_MAJOR, M2_VERSION_MINOR, M2_VERSION_PATCH};
}

const char* m2ResultName(m2Result result)
{
    switch (result)
    {
    case m2_success:
        return "m2_success";
    case m2_errorInvalid:
        return "m2_errorInvalid";
    case m2_errorCapacity:
        return "m2_errorCapacity";
    case m2_errorStale:
        return "m2_errorStale";
    case m2_errorUnsupported:
        return "m2_errorUnsupported";
    case m2_errorState:
        return "m2_errorState";
    case m2_errorVersion:
        return "m2_errorVersion";
    case m2_errorFormat:
        return "m2_errorFormat";
    case m2_errorLimit:
        return "m2_errorLimit";
    case m2_errorBusy:
        return "m2_errorBusy";
    default:
        return "unknown m2Result";
    }
}
