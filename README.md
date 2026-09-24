# Maul2D

[![ci](https://github.com/Rawframe-Project/maul2d/actions/workflows/ci.yml/badge.svg)](https://github.com/Rawframe-Project/maul2d/actions/workflows/ci.yml)

A deterministic 2D physics engine for games, written in C17 with a
plain C API, no dependencies and an MIT license. Maul2D is the 2D
member of the Maul family; [Maul3D](https://github.com/Rawframe-Project/maul3d)
is its 3D sibling and follows the same rules.

The same inputs produce the same bits on every supported platform.
Snapshots restore a world exactly, and a command journal records a
session and replays it byte for byte. That makes rollback netcode,
lockstep multiplayer, replays and server-side verification a few
calls away:

```c
int32_t size = m2World_SnapshotSize(world);
m2World_Snapshot(world, buffer, size);
/* ... mispredicted steps ... */
m2World_Restore(world, buffer, size); // bit-exact resimulation from here
```

**[Try the testbed in your browser](https://rawframe-project.github.io/maul2d/testbed/)**.
It is the same engine compiled to WebAssembly; hold R and time runs
backward.

## Features

- **Rigid bodies**: circles, capsules, polygons (rounded too),
  segments and one-way chain terrain with ghost corners; static,
  kinematic and dynamic bodies; forces, damping, dominance groups,
  conveyor surfaces and explosions.
- **Solver**: soft-step contacts with speculative margins, warm
  starting and graph-colored SIMD solving. Parallel work runs on the
  host's task system; the engine starts no threads, and serial and
  threaded runs produce identical trajectories.
- **Eleven joint types**: distance (with a hard range), revolute
  (with an angular spring), prismatic, weld, wheel, motor, mouse,
  filter, gear, pulley and ratchet, with motors, limits, runtime
  tuning and breaking with events.
- **Particle fluids**: an optional deterministic particle system with
  water, viscosity, surface tension, powder and jelly, coupled both
  ways with rigid bodies and covered by snapshots, the journal and
  the hash like everything else.
- **Continuous collision** for fast bodies, island sleeping, and
  contact and sensor events with strict begin and end pairing.
- **Queries**: rays, shape casts, overlaps and all-hits variants, a
  character mover kit (collide, solve planes, clip) and particle
  region queries, all in canonical order and exact far from the
  origin (positions are 64-bit).
- **Geometry tools**: convex hulls from point clouds and convex
  decomposition of concave outlines.
- **Integration**: 288 public functions, full state readback, debug
  draw, counters and profiling, allocator and assert hooks.

## Getting started

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

A C17 compiler is required; with MSVC that means Visual Studio 2022 or
newer, the first version that can turn off floating-point
contraction. x64 builds use AVX2 and FMA (Haswell, 2013, and later);
configure with `-DMAUL2D_SIMD=scalar` for a portable build that
produces identical results. An AVX2 build started on a CPU without
AVX2 refuses to create worlds instead of crashing.

`cmake --install` installs the library, the headers, a CMake package
and a pkg-config file, so `find_package(maul2d)` and
`pkg-config maul2d` both work. Tagged releases carry prebuilt
libraries for Linux, Windows and macOS.

The samples show the API at work:

- `samples/minimal`: a standalone project that finds the installed
  package, stacks a tower, snapshots it, knocks it over and rolls
  the wreck back.
- `sample_car`: a motorized, sprung two-wheeler that drives itself
  and prints telemetry.
- `sample_replay`: records a session, replays it into a fresh world
  and compares the final hashes.

The interactive testbed (built on raylib, which the library itself
does not use) runs in the browser or natively with
`-DMAUL2D_BUILD_TESTBED=ON`. Its twelve scenes include a platformer on the
character mover, a machinery hall, particle goo and a rewind ring.

## Determinism

- IEEE arithmetic only: no fast math and no floating-point
  contraction, enforced at configure time.
- One set of results across the AVX2, NEON and scalar kernels.
- Canonical ordering on every path, and scheduling that never changes
  results.
- CI compares the determinism hashes printed by the tests across
  eleven cells: GCC, Clang and MSVC on x64 and arm64 Linux and
  Windows, Clang on arm64 macOS, Debug builds, a sanitizer build with
  the scalar backend, and WebAssembly. They must also match the
  values committed in `test/hashes.txt`.
- A fuzzer drives random sessions through journal replay, rollback,
  unjournaled and threaded twins on every seed, and a weekly
  scheduled run adds a long soak.

## Benchmarks

The scenes in `bench/` (two pyramids, hanging chains of 30 and 2000
links, a joint farm and a water tank) print their timings and their
final world hashes. The hashes are pinned in `bench/pins.txt`, and CI
fails when one moves.

## Status

Current version: 0.0.1. Until 1.0.0 the API, the ABI and the snapshot
and journal formats may change in any minor release; the
[changelog](CHANGELOG.md) records every change. Defs carry a cookie,
so a def that was not initialized with its `m2Default...Def`
function is refused. Snapshots and journal tapes belong to one
library version.

## Documentation

- [The guide](docs/guide.md): the contract, rollback netcode, joints,
  water, queries, the character mover and recipes.
- [The API reference](docs/api.md): every public function with its
  documentation, generated from the headers.
- [The conventions](docs/conventions.md): the rules both engines
  follow, from naming to commits.
- [Design records](docs/adr/README.md): the decisions behind the
  rules and the architecture.
- [The changelog](CHANGELOG.md): every release and what changed.
- [CONTRIBUTING.md](CONTRIBUTING.md): how to contribute.

## Acknowledgments

The engine's algorithms follow published work: the soft step schedule
and soft constraints, the collision, distance and hull methods, the
mass properties and the particle model. [docs/references.md](docs/references.md)
lists each source. The interactive testbed is built on
[raylib](https://github.com/raysan5/raylib); see
[THIRD_PARTY.md](THIRD_PARTY.md).

## License

MIT. See [LICENSE](LICENSE).
