# Samples

Each sample in [`samples/`](../samples) is a complete program that uses
only the public headers and prints something you can check. They are
built with the library (turn them off with `-DMAUL2D_BUILD_SAMPLES=OFF`):

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/sample_car
```

## A car

[`samples/car.c`](../samples/car.c) builds a chassis on two sprung,
motorized wheels and drives it along a road, printing telemetry. The
final world hash is the same on every machine.

## Record and replay

[`samples/replay.c`](../samples/replay.c) records a session into the
command journal, replays it into a fresh world and compares the final
hashes. Deterministic lockstep, replays and reproducible bug reports
are built on this.

## The installed package

[`samples/minimal/`](../samples/minimal) is a separate CMake project
that finds the installed package with `find_package(maul2d)` and
nothing else. It stacks a tower, snapshots it, knocks it over and
restores the snapshot; it exits with an error if the restored world
differs. Try it against an install:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --install build --prefix /tmp/maul2d-prefix
cmake -S samples/minimal -B build-minimal -DCMAKE_PREFIX_PATH=/tmp/maul2d-prefix
cmake --build build-minimal
./build-minimal/minimal
```
