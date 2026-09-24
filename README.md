# Maul2D

[![ci](https://github.com/Rawframe-Project/maul2d/actions/workflows/ci.yml/badge.svg)](https://github.com/Rawframe-Project/maul2d/actions/workflows/ci.yml)

A deterministic 2D physics engine for games. Written in C17, with a
plain C API, no dependencies and an MIT license.

Same inputs, same bits, on every supported platform and at every
thread count. A snapshot restores a world exactly, and the command
journal records a session and replays it byte for byte. Rollback
netcode, lockstep multiplayer, replays and server-side verification
are a few calls away:

```c
int32_t size = m2World_SnapshotSize(world);
m2World_Snapshot(world, buffer, size);
/* ... mispredicted steps ... */
m2World_Restore(world, buffer, size); // resimulate from here, bit for bit
```

**[Try the testbed in your browser](https://rawframe-project.github.io/maul2d/testbed/)**:
the same engine compiled to WebAssembly. Hold R and time runs
backward.

## The Maul family

Maul2D has a 3D sibling, [Maul3D](https://github.com/Rawframe-Project/maul3d).
The two are one family: two engines, one set of rules.

| | Maul2D | Maul3D |
|---|---|---|
| World | 2D | 3D |
| Built for | platformers, puzzles, top-down games, machines, water | destruction, vehicles, characters, lockstep multiplayer |
| Beyond rigid bodies | particle fluids, one-way chain terrain, geometry tools | voxel destruction, raycast vehicles, character controller, soft bodies, replay tools |
| Repository | this one | [Rawframe-Project/maul3d](https://github.com/Rawframe-Project/maul3d) |

What the family shares:

- **One contract.** Same inputs, same bits; snapshots, restores and
  journal replays are exact; bad input is refused with a reason and
  never corrupts a world quietly.
- **One vocabulary.** The same concept has the same name and only the
  prefix changes: `m2CreateBody` and `m3CreateBody`,
  `m2World_StartJournal` and `m3World_StartJournal`,
  `m2World_GetContactEvents` and `m3World_GetContactEvents`.
- **One set of rules.** The [conventions](docs/conventions.md), the
  [design records](docs/adr/README.md), the build modules, the CI
  workflow and the tools are the same files in both repositories,
  and a check keeps them that way.
- **One program.** Every symbol carries its engine's prefix, so both
  libraries link into the same game without a clash.

## Features

- **Rigid bodies**: circles, capsules, polygons (rounded too),
  segments and one-way chain terrain with ghost corners; static,
  kinematic and dynamic bodies; forces, damping, motion locks,
  dominance groups, conveyor surfaces and explosions.
- **Solver**: soft-step contacts with speculative margins, warm
  starting and graph-colored SIMD solving. Parallel work runs on your
  task system; the engine starts no threads, and serial and threaded
  runs give identical results.
- **Eleven joint types**: distance (with a hard range), revolute
  (with an angular spring), prismatic, weld, wheel, motor, mouse,
  filter, gear, pulley and ratchet, with motors, limits, runtime
  tuning, and breaking with events.
- **Particle fluids**: an optional particle system with water,
  viscosity, surface tension, powder and jelly, coupled both ways
  with rigid bodies. Snapshots, the journal and the hash cover it like
  everything else. Fluid volumes add buoyancy without particles.
- **Continuous collision** for fast bodies, island sleeping, and
  contact and sensor events with strict begin and end pairing.
- **Queries**: rays, shape casts and overlaps, closest-hit and
  all-hits, a character mover kit (collide, solve planes, clip) and
  particle region queries. Results come in canonical order and stay
  exact far from the origin (positions are 64-bit).
- **Geometry tools**: convex hulls from point clouds and convex
  decomposition of concave outlines.
- **Integration**: 288 public functions, full state readback, debug
  draw, counters and profiling, and allocator and assert hooks.

## Getting started

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

A C17 compiler is required; with MSVC that means Visual Studio 2022 or
newer, the first version that can turn off floating-point contraction.
x64 builds use AVX2 and FMA (Haswell, 2013, and later); configure with
`-DMAUL2D_SIMD=scalar` for a portable build that gives identical
results. An AVX2 build started on a CPU without AVX2 refuses to create
worlds instead of crashing.

`cmake --install` installs the library, the headers, a CMake package
and a pkg-config file, so `find_package(maul2d)` and
`pkg-config maul2d` both work. Build a shared library with
`-DMAUL2D_BUILD_SHARED=ON`. Every tagged release carries prebuilt
libraries for Linux (x64 and arm64), Windows and macOS.

## Samples and the testbed

The [samples](docs/samples.md) are small complete programs:

- `sample_car`: a motorized, sprung two-wheeler that drives itself and
  prints telemetry.
- `sample_replay`: records a session, replays it into a fresh world
  and compares the final hashes.
- `samples/minimal`: a standalone project that finds the installed
  package, stacks a tower, snapshots it, knocks it over and rolls the
  wreck back.

The interactive testbed runs in the browser or natively with
`-DMAUL2D_BUILD_TESTBED=ON`. Its twelve scenes include a platformer
on the character mover, a machinery hall, particle goo and a rewind
ring.

## Determinism

- IEEE arithmetic only: no fast math and no floating-point
  contraction, enforced when the build is configured.
- One set of results from the AVX2, NEON and scalar kernels.
- Canonical order on every path, and scheduling that never changes
  results.
- CI compares the hashes the tests print across eleven cells: GCC,
  Clang and MSVC on x64 and arm64 Linux and Windows, Clang on arm64
  macOS, Debug builds, a sanitizer build on the scalar backend, and
  WebAssembly. They must also match the values committed in
  `test/hashes.txt`.
- A fuzzer drives random sessions through journal replay, rollback
  and threaded twins on every seed, and a weekly run adds a long soak.

## Benchmarks

The scenes in `bench/` (two pyramids, hanging chains of 30 and 2000
links, a joint farm and a water tank) print their timings and their
final world hashes. The hashes are pinned in `bench/pins.txt`, and CI
fails when one moves.

## Status

Version 0.0.1. Until 1.0.0 the API, the ABI and the snapshot and
journal formats may change in any minor release; the
[changelog](CHANGELOG.md) records every change. Defs carry a cookie,
so a def that was not set up with its `m2Default...Def` function is
refused. Snapshots and journal tapes belong to one library version.

No language bindings ship yet. The API is plain C with blittable
structs, so it binds directly; Maul3D's C# starter kit shows the
pattern.

## Documentation

- [The documentation site](https://rawframe-project.github.io/maul2d/):
  the guide, the samples and the changelog as web pages, and the
  browser testbed.
- [The guide](docs/guide.md): the contract, rollback netcode, joints,
  water, queries, the character mover and recipes.
- [The API reference](docs/api.md): every public function with its
  documentation, generated from the headers.
- [The conventions](docs/conventions.md): the rules both engines
  follow, from naming to commits.
- [Design records](docs/adr/README.md): the decisions behind the rules
  and the architecture.
- [The changelog](CHANGELOG.md): every release and what changed.
- [CONTRIBUTING.md](CONTRIBUTING.md): how to contribute.

## Acknowledgments

The engine's algorithms follow published work: the soft step schedule
and soft constraints, the collision, distance and hull methods, the
mass properties and the particle model.
[docs/references.md](docs/references.md) lists every source. The
library itself uses no outside code. The testbed, which is not part
of the library, is built on [raylib](https://github.com/raysan5/raylib)
(zlib/libpng license), fetched when the testbed is configured.

## License

MIT. See [LICENSE](LICENSE).
