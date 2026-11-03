# Maul2D

[![ci](https://github.com/Rawframe-Project/maul2d/actions/workflows/ci.yml/badge.svg)](https://github.com/Rawframe-Project/maul2d/actions/workflows/ci.yml)

A deterministic 2D physics engine for games. Written in C, with a plain C API, no dependencies and an MIT license.

Same inputs, same bits, on every supported platform and at every thread count. A snapshot restores a world exactly, and the command journal records a session and replays it byte for byte. Rollback netcode, lockstep multiplayer, replays and server-side verification are a few calls away:

```c
int64_t size = m2World_GetSnapshotSize(world);
int64_t written = 0;
m2Result result = m2World_Snapshot(world, buffer, size, &written);
/* ... mispredicted steps ... */
result = m2World_Restore(world, buffer, written); // resimulate from here, bit for bit
```

**[Try the testbed in your browser](https://rawframe-project.github.io/maul2d/testbed/)**: the same engine compiled to WebAssembly. Hold R and time runs backward.

## What it has

- **Rigid bodies**: circles, capsules, rounded polygons, segments and one-way chain terrain; static, kinematic and dynamic bodies; forces, damping, motion locks, dominance, conveyors and explosions.
- **A soft-step solver** with speculative contacts, warm starting and graph-colored SIMD solving. Parallel work runs on the program's own task system; the engine starts no threads.
- **Eleven joint types**: distance, revolute, prismatic, weld, wheel, motor, mouse, filter, gear, pulley and ratchet, with motors, limits, springs, run-time tuning and breaking.
- **Particle fluids**: water, viscosity, surface tension, powder and jelly, coupled both ways with bodies; water volumes for buoyancy without particles.
- **Continuous collision**, island sleeping, and contact, sensor and joint events with strict begin and end pairing.
- **Queries**: rays, shape casts and overlaps, closest and all hits, a character mover kit, and particle region queries, exact far from the origin.
- **Geometry tools**: convex hulls from point clouds, convex decomposition of outlines, and shattering bodies along them.

## Getting started

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

The library is C23 and builds with GCC 14, Clang 19, Apple Clang, or clang-cl on Windows; the public headers are C17 and C++17. `cmake --install` installs a CMake package and a pkg-config file. [Getting started](docs/user/getting-started.md) has the options and a first world; every tagged release carries the source archive with its checksum, an SBOM and a build attestation.

## The Maul family

Maul2D has a 3D sibling, [Maul3D](https://github.com/Rawframe-Project/maul3d). The two share one contract (same inputs, same bits; exact snapshots and replays; bad input refused with a reason), one vocabulary (`m2CreateBody` and `m3CreateBody`), and one set of [conventions](docs/internals/maul-conventions.md) with the other Maul libraries. Every symbol carries its library's prefix, so they all link into one program.

## Status

Maul2D is at 1.y, and there is no 2.0.0. A program that builds against 1.y builds against any later 1.z and means the same: a minor release only adds functions, types, enum values and fields, and a patch release only fixes. A change that gives existing inputs a different result is a `feat`, or a `fix` when it corrects a result the documentation already promised, and the [changelog](CHANGELOG.md) says what changes. A release that changes results is always a minor release, so a snapshot or tape replays onto the same bits or is refused. A world def built from other headers is refused at creation. A snapshot or tape is read within the minor version that wrote it (`M2_ABI_VERSION`), and a release of another minor version refuses it with `m2_errorVersion`.

## Documentation

- [The documentation](docs/README.md): using the library, how it works, maintaining it. Also on the [documentation site](https://rawframe-project.github.io/maul2d/).
- [The API reference](https://rawframe-project.github.io/maul2d/api.html), generated from the headers.
- [Contributing](https://github.com/Rawframe-Project/.github/blob/main/CONTRIBUTING.md).

The engine's algorithms follow published work, listed in [references](docs/internals/references.md); the library uses no outside code. The testbed, which is not part of the library, is built on [raylib](https://github.com/raysan5/raylib) (zlib/libpng license), fetched when the testbed is configured.

## License

MIT. See [LICENSE](LICENSE).
