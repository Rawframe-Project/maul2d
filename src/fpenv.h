// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The floating-point environment every computation of the library runs
// in: round to nearest, denormals kept, no traps (a family file). A host
// may run its threads with flush-to-zero, denormals-are-zero or another
// rounding mode; each public call that computes floats enters this
// environment and leaves the host's as it found it, so threads in any mode
// give the same bits.
//
// Entries nest: the outermost on a thread saves the host's control
// register and switches, an entry inside it only counts, and the leave
// that ends the outermost writes back the register it saved, whatever host
// code did in between. A leave on every path, in the reverse order of the
// entries, puts the thread back as it was. The leave compares the whole
// register, so on x86 it also puts MXCSR's sticky exception flags (bits 0
// to 5) back as they were at the entry: flags the library's arithmetic
// raised are not left behind, and the host's earlier ones are kept. FPCR
// holds no flags (they are FPSR's, which the library leaves alone).
//
// Host code the library calls runs in one of two environments:
// - A callback whose result feeds a result the library promises the same
//   everywhere (a host easing function, a host sampler a bake uses) is
//   computing: it is called inside the entry, in the library's
//   environment, with no helper.
// - Any other (an allocator, the task hooks, logging, events) runs in the
//   host's own environment, between m2FloatHostEnter and
//   m2FloatHostLeave. A public call from inside it sees no entry open
//   and is an outermost entry of its own; a setting host code changes
//   lasts until it returns.
//
// Compilers do not order floating-point arithmetic against the writes to
// the control register, so they may move it across them. A public call
// therefore does no arithmetic of its own between entering and leaving:
// it calls a worker marked M2_NOINLINE, whose body cannot move out of
// the call.
//
// WebAssembly has one environment, the library's, and nothing to switch:
// there the entries and the host pairs are empty inline functions that
// keep no depth, and M2_NOINLINE is nothing, so a wrapper and its worker
// compile to the worker alone.

#ifndef MAUL2D_SRC_FPENV_H
#define MAUL2D_SRC_FPENV_H

#include <stdint.h>

#if defined(__wasm__)
#define M2_NOINLINE
#elif defined(_MSC_VER) && !defined(__clang__)
#define M2_NOINLINE __declspec(noinline)
#else
#define M2_NOINLINE __attribute__((noinline))
#endif

// The thread's state outside a call into host code: the register the
// outermost entry saved and the depth.
typedef struct m2FloatHost
{
    uint64_t saved;
    uint32_t depth;
} m2FloatHost;

#if defined(__wasm__)

static inline uint64_t m2FloatEnvEnter(void)
{
    return 0;
}

static inline void m2FloatEnvLeave(uint64_t token)
{
    (void)token;
}

static inline m2FloatHost m2FloatHostEnter(void)
{
    return (m2FloatHost){0, 0};
}

static inline void m2FloatHostLeave(m2FloatHost host)
{
    (void)host;
}

#else

// Enters the library's environment: the outermost entry on a thread (or
// the first inside host code) saves the host's control register and sets
// round to nearest, denormals kept and every exception masked, writing
// the register only when that changes it; an entry inside another only
// counts. Returns the token m2FloatEnvLeave takes: the host's register
// with bit 63 set for an outermost entry, 0 for one inside another. By
// value, so a caller takes no local's address and needs no stack
// protector.
uint64_t m2FloatEnvEnter(void);

// Leaves the entry that returned token: the outermost restores the
// host's control register, writing it only when it differs.
void m2FloatEnvLeave(uint64_t token);

// Before host code: inside an entry, writes the host's saved register
// back and leaves no entry open; outside any, does nothing. Returns what
// m2FloatHostLeave takes. Each call site puts the pair around the one
// host call in a helper of its own marked M2_NOINLINE, whose body is
// only that, so none of the caller's arithmetic moves between them.
m2FloatHost m2FloatHostEnter(void);

// After host code: the thread's state as m2FloatHostEnter found it and,
// inside an entry, the library's environment again, from the host's saved
// register, whatever host code set.
void m2FloatHostLeave(m2FloatHost host);

#endif

// Runs body(context) inside an entry: a task body the library hands the
// host's task hooks, which may run it on a thread in any mode. body is a
// worker that does the arithmetic, so none moves out of the entry.
void m2FloatEnvRun(void (*body)(void* context), void* context);

// The thread's control register as it is: MXCSR on x86 (with the x87
// control word in bits 32 to 47 on 32-bit x86), FPCR on AArch64, 0 on
// WebAssembly. For tests and diagnostics.
uint64_t m2FloatControlRead(void);

// Sets the thread's control register to value; nothing on WebAssembly. For
// tests, which play a host with another setting.
void m2FloatControlWrite(uint64_t value);

// How deep the thread is in entries since its last outermost one, 0
// outside any or in host code. For tests.
uint32_t m2FloatEnvDepth(void);

#endif // MAUL2D_SRC_FPENV_H
