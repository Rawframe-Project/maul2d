# Determinism

maul2d promises that the same world, given the same calls, reaches the same bits on every supported platform, compiler, SIMD backend and worker count. This page explains how the code keeps that promise and how the tests hold it.

## Arithmetic

- **IEEE operations only.** `cmake/CompilerFlags.cmake` turns off floating-point contraction (`-ffp-contract=off`) and refuses fast math, so the compiler never fuses or reorders an operation on one target and not another. Fused multiply-add appears only where the code spells it through `src/simd.h`, which maps it to `_mm256_fmadd_ps`, `vfmaq_f32` or `fmaf`; all three are correctly rounded, so they agree.
- **No platform math.** The sources call only the correctly rounded functions in `tools/libm-allowed.txt` (square roots, floors, ceilings, absolute values), and `tools/check_source.py` refuses any other. Angles go through the engine's own `m2MakeRot` and `m2Atan2`, polynomials written from the references in [references.md](references.md).
- **Pinned min, max and abs.** `m2MinF` and `m2MaxF` are exactly `a < b ? a : b` and `a > b ? a : b`, signed zeros and NaNs included, and `m2AbsF` clears the sign bit. Compilers fold the plain forms differently (MSVC treated +0 and -0 as equal), so `core_math.h` implements them with instructions whose semantics match on each target.
- **One float crossing per stage.** Positions are f64 and everything local is f32. Each stage moves a body-local value to world space in one place (a body's pose added in double), so precision does not depend on distance from the origin and the rounding is the same everywhere.
- **Renormalized rotations.** Every composition of rotations renormalizes at once, so drift is bounded and identical.

## Order

Every result that depends on an order has a canonical one:

- Pairs live in a list sorted by key (the two shape indices), rebuilt each step by a merge, and `m2World_Validate` checks the order. Manifolds, events and contact data follow it.
- Islands union toward the lowest body index, and every pass over bodies, shapes, joints and particles walks slots in ascending order.
- Ties break to the lower index: the closest hit of a query, the deepest face of a manifold, the first ear of a decomposition.
- A query that keeps only some of its hits keeps the lowest shape slots (`m2TreeKeepLowest`): the particle contact candidates, a particle's crossing candidates and a bullet's sweep candidates. A broadphase tree yields its leaves in an order that follows its history, which is not hashed, so what a capped query keeps must depend only on which leaves overlap.
- A slot freed by a destroy is reused in FIFO order under a new generation, so the same calls hand out the same ids.

## Threads

The task hooks never change results. Parallel work is split into ranges where each item writes only its own output slot (a pair's manifold, a constraint block of one graph color, a particle's staging cell), so any split and any worker order produce the same bytes. Work whose order matters (the force passes of the fluid, which each read the velocities the previous one left; the merge of new pairs) stays serial. The concurrency test runs separate worlds on separate threads, and the golden scenes compare a four-worker run with the serial one.

## State

Everything that affects the future is world state, and every byte of it is in the snapshot: the pools, the warm-start impulses, the sleep timers, the particle lifetimes, the journal recorder excepted. A restore therefore continues exactly as the original would have. `m2World_Hash` folds, with `m2Hash64` and its frozen constants, every piece of that state the step reads: poses, velocities, forces, mass and sleep, every setting of bodies, shapes, joints, particles and volumes, the contacts, each shape's fat box and moved flag (pairs are found by querying them), and each id pool's free order and generations, each keyed by its slot (`snapshots-and-journal.md` has the rules). Two worlds with the same hash step on to the same bits. User data, which the step never reads, and the broadphase tree's shape, which decides only the order the tree yields its leaves in, stay out; diagnostics such as `m2World_GetProfile` are in neither the snapshot nor the hash.

## Golden scenes

A golden value pins a promised output, so it is part of the specification. There are few of them: each scene covers several subsystems at once, so a deliberate change of results regenerates a handful of values. The scenes are in [`test/test_golden.c`](../../test/test_golden.c), and their values are in [`test/hashes.txt`](../../test/hashes.txt).

Five scenes pin simulation results. maul3d uses the same names for the scenes that correspond, so the two engines' golden lists line up.

| Name | What it runs | Extra check |
|---|---|---|
| `STACK` | A settling pyramid, a revolute chain, a sprung two-wheeler on wheel joints, a prismatic elevator and a welded beam, far from the origin. | A four-worker twin must match the serial run. |
| `CONTACTS` | Bullets against a thin wall, a one-way chain floor, a sensor zone, a filtered pair, a bouncy and a sliding body, and a conveyor. | The contact and sensor event streams are hashed with the world state. |
| `FLUID` | Water in a basin, a tensile droplet, a spring-and-triad jelly, a buoyancy volume with a floating and a sinking crate, and wind. | A four-worker twin must match the serial run. |
| `ROLLBACK` | The stack and the fluid in one world: snapshot, a divergent future, restore. | The hash sequence must repeat, and a re-snapshot must be byte-equal. |
| `JOURNAL` | The seeded API walk's ten canonical seeds, each recorded and replayed. | Each replay must match an unjournaled twin and a threaded twin. |

Two lines pin formats rather than results:

- `ROLLBACK_SNAPSHOT`: the bytes of the `ROLLBACK` scene's snapshot.
- `JOURNAL_TAPE`: the bytes of the walk's journal tape.

They sit on their own lines because the snapshot and the tape record public structs as bytes. A change to those structs, or to which calls a tape records, moves a format line while every result line stays the same.

The six `BENCH_*` lines pin the benchmark scenes in [`bench/scenes.c`](../../bench/scenes.c), which the benchmark and the golden test share.

## Checking and regenerating

`test/CMakeLists.txt` turns each line `M2_<NAME>_HASH=<hex>` of `test/hashes.txt` into the definition `M2_GOLDEN_<NAME>`, so every CI cell checks the values compiled in, with no file to read at run time. Configuration fails on a malformed or repeated line. The test fails when a value is missing, because a missing value reads as zero, and when the file holds a line that no scene checks.

Each scene prints its line, so a deliberate change of results is one regeneration on any cell:

```sh
out/<preset>/test_golden | grep -E '^M2_[A-Z0-9_]+_HASH=' > test/hashes.txt
```

The pull request that regenerates values says which lines moved and why, and CI shows every cell agreeing on the new values.

## The walk

The seeded API walk (`test/walk.h`) drives random sessions through every public writer, with rollbacks, journal replays and threaded twins on every seed; each seed's world must stay valid and its replays must match. The walk suite runs a set of seeds on every cell, the golden `JOURNAL` scene pins ten of them, and the nightly simulation runs more seeds and a long soak.
