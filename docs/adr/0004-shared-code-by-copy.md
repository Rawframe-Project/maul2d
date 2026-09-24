# 0004. Shared code by copy, checked for drift

Status: Accepted

## Context

The engines share build modules, tool scripts, documents, a SIMD
header, a CPU check and a hash function. A shared library or a git
submodule would keep one copy, but each engine is built, packaged and
consumed on its own, with its own prefix, and a submodule or a third
package would make every consumer and every release depend on both
repositories moving together. The id pools, the journals and the
allocators follow the same contracts (FIFO slot reuse, generations
that retire instead of wrapping, typed refusals, commands replayed
through the same internal functions) but are written around each
engine's world layout.

## Decision

Shared code lives in both repositories as copies. Files in
`tools/family-files.txt` are identical byte for byte. Files in
`tools/family-renamed.txt` (`src/simd.h`, `src/cpu.c`, `src/hash.c`)
are identical once the engine prefix is renamed (`m2`/`m3`,
`maul2d`/`maul3d`). `tools/check_family.py` compares both lists with
the sibling repository, and the weekly CI job runs it. A change to a
shared file lands in both repositories together.

The id pools, journals and allocators stay engine code: they share
their contracts, written down in `docs/conventions.md`, not their
source. Floating point follows one policy in both engines: no
implicit contraction, and fused multiply-add only where a kernel
spells it through `src/simd.h`, the same on every backend.

## Consequences

Each engine builds and ships alone, and the family check catches a
shared file that changed on one side only. A fix to a shared file is
made twice. Code that differs only because it grew apart (a pool, a
journal container) can join the renamed list later, once the engines
store it the same way.
