# Getting started

Maul2D is a deterministic 2D physics engine for games, written in C. It builds with CMake, installs as a CMake package and a pkg-config module, and needs nothing but a C compiler and the C standard library.

## Requirements

- CMake 3.25 or newer.
- GCC 14 or Clang 19 or newer, or Apple Clang. On Windows, `clang-cl`: the Visual Studio component "C++ Clang tools for Windows" (`-T ClangCL`). MSVC's own `cl` compiles the public headers but not the library.
- Emscripten for WebAssembly builds.

The public headers are plain C17 and C++17, so a program that includes them needs neither a C23 compiler nor C at all.

## Build and install

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build --prefix /path/to/prefix
```

Then, from CMake:

```cmake
find_package(maul2d REQUIRED CONFIG)
target_link_libraries(game PRIVATE maul2d::maul2d)
```

or through pkg-config, `pkg-config --cflags --libs maul2d`. A project can also add the source tree with `add_subdirectory`; the tests, samples and benchmarks are then off by default. Every tagged release carries prebuilt libraries for Linux (x64 and arm64), Windows and macOS.

## Build options

| Option | Default | Effect |
|---|---|---|
| `MAUL2D_SIMD` | `auto` | `auto` uses AVX2 and FMA on x64 (Haswell, 2013, and later) and NEON on arm64; `scalar` builds portable kernels. Every backend gives the same bits. |
| `MAUL2D_BUILD_SHARED` | `OFF` | A shared library instead of a static one. |
| `MAUL2D_VALIDATE` | `OFF` | In a build with assertions (Debug, sanitizers), checks every invariant of the world after each step and stops at the first broken one. |
| `MAUL2D_BUILD_TESTBED` | `OFF` | The interactive testbed, which fetches raylib. |
| `MAUL2D_BUILD_TESTS`, `_SAMPLES`, `_BENCH` | on at top level | The test suites, samples and benchmarks. |

An AVX2 build started on a CPU without AVX2 refuses to create worlds with `m2_errorUnsupported` instead of crashing; `m2CpuSupportsBackend` tells a program before it tries.

## A first world

```c
m2WorldDef worldDef = m2DefaultWorldDef();
m2World* world = NULL;
if (m2CreateWorld(&worldDef, &world) != m2_success)
{
    return; // the CPU, the def or the allocator refused
}

m2BodyDef groundDef = m2DefaultBodyDef();
m2BodyId ground = m2_nullBodyId;
m2ShapeDef shapeDef = m2DefaultShapeDef();
m2Polygon floor = m2MakeBox(10.0f, 0.5f);
m2ShapeId floorShape = m2_nullShapeId;

m2BodyDef ballDef = m2DefaultBodyDef();
ballDef.type = m2_dynamicBody;
ballDef.position = (m2Pos2){0.0, 4.0};
m2BodyId ball = m2_nullBodyId;
m2Circle circle = {{0.0f, 0.0f}, 0.5f};
m2ShapeId ballShape = m2_nullShapeId;

bool built =
    m2CreateBody(world, &groundDef, &ground) == m2_success &&
    m2CreatePolygonShape(world, ground, &shapeDef, &floor, &floorShape) == m2_success &&
    m2CreateBody(world, &ballDef, &ball) == m2_success &&
    m2CreateCircleShape(world, ball, &shapeDef, &circle, &ballShape) == m2_success;

for (int32_t i = 0; built && i < 120; ++i)
{
    m2World_Step(world, 1.0f / 60.0f, 4);
}
m2Pos2 resting = m2Body_GetPosition(world, ball);
m2DestroyWorld(world);
```

Step at a fixed timestep, always: determinism starts with feeding the same `dt` every step, and 1/60 with 4 substeps is the tuned default.

## The model

- **A world owns everything.** `m2World*` is an opaque pointer, and every call on what a world holds takes the world first. Worlds share nothing, so separate worlds may live on separate threads. A world takes the memory it keeps when it is created, from the allocator in its def, and its pools never grow.
- **Defs.** Every creation takes a def from its `m2Default...Def` function. A def carries a cookie, so one that was not set up that way is refused. The world def also carries the version of the headers the program was built with, and `m2CreateWorld` refuses a def from other headers with `m2_errorVersion`.
- **Ids.** Bodies, shapes, joints, chains, particles and fluid volumes are named by small value ids, a slot and a generation. Copy them, store them, send them over the network. An id carries no world: passed to another world, it names whatever that world holds in the same slot.
- **Results.** A call that can fail returns an `m2Result`: `m2_success` (0) or a negative error, and `m2ResultName` names it for logs. What a call makes comes back through an `Out` parameter, which holds the null id on failure. Getters return values, and setters return nothing.
- **Misuse.** An argument a correct program never passes, a destroyed object's id, or a change made from inside code the world is running counts one in `m2Counters.misuse`; such a getter returns zero and such a setter changes nothing. Read the counter in release builds to find these bugs. Bytes from outside are not misuse: a corrupt snapshot or tape is refused with `m2_errorInvalid` and counts nothing, so a hostile peer cannot move the counter. Particles are the one kind of object that dies on its own, so a dead particle id counts nothing either ([Fluids](fluids.md)).
- **Threads.** Readers (queries, getters, diagnostics, drawing) may run together; writers (anything that changes the world, the step included) need the world to themselves. The step runs its parallel work on the program's own task system through `enqueueTask` and `finishTask` in the world def; the split changes speed, never results. Code the world calls (those hooks, debug draw callbacks, the allocator) may read the world but not change it: its writers refuse with `m2_errorState`.

Each header documents every function: what it does, each argument, each status it returns and from which threads it may be called. The [API reference](https://rawframe-project.github.io/maul2d/api.html) is generated from them.

## What it leaves out

Some features of other engines are absent by decision. Pre-solve callbacks run program code in the middle of a step, which breaks replay by construction; chains, filters, sensors and the filter joint cover the usual cases deterministically. Solver settings (contact stiffness, speculative distance, warm starting) and per-body sleep thresholds stay fixed, because the physics is part of the determinism contract. There is no way to turn off determinism, the sleep bookkeeping or the pairing of begin and end events.

## Samples and the testbed

The samples build with the library and print something to check:

- `sample_car` drives a chassis on two sprung, motorized wheels along a road and prints telemetry; the final world hash is the same on every machine.
- `sample_replay` records a session into the journal, replays it into a fresh world and compares the hashes.
- [`samples/minimal/`](../../samples/minimal) is a separate project that finds the installed package with `find_package(maul2d)`, stacks a tower, snapshots it, knocks it over and restores it.

The testbed runs in the browser ([try it](https://rawframe-project.github.io/maul2d/testbed/)) or natively with `-DMAUL2D_BUILD_TESTBED=ON`. Its twelve scenes include a car, a platformer on the character mover, a machinery hall, water, jelly, and ragdolls whose time runs backward while R is held.
