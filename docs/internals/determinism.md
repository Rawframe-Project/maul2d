# Determinism

maul2d promises that the same world, given the same calls, reaches the same bits on every supported platform, compiler, SIMD backend and worker count. This page describes how the tests hold that promise. The documentation pull request of the migration completes the page.

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

They sit on their own lines because the snapshot and the tape record public structs as bytes. A change to those structs moves a format line while every result line stays the same.

The six `BENCH_*` lines pin the benchmark scenes in [`bench/scenes.c`](../../bench/scenes.c), which the benchmark and the golden test share.

## Checking and regenerating

`test/CMakeLists.txt` turns each line `M2_<NAME>_HASH=<hex>` of `test/hashes.txt` into the definition `M2_GOLDEN_<NAME>`, so every CI cell checks the values compiled in, with no file to read at run time. Configuration fails on a malformed or repeated line. The test fails when a value is missing, because a missing value reads as zero, and when the file holds a line that no scene checks.

Each scene prints its line, so a deliberate change of results is one regeneration on any cell:

```sh
out/<preset>/test_golden | grep -E '^M2_[A-Z0-9_]+_HASH=' > test/hashes.txt
```

The pull request that regenerates values says which lines moved and why, and CI shows every cell agreeing on the new values.
