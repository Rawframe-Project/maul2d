# Development

How to build, test and change maul2d as a maintainer. Building and installing as a user is in [getting-started.md](../user/getting-started.md).

## Toolchain

The organization's CI image, `ghcr.io/rawframe-project/ci-cpp`, has everything below; CI runs every Linux job in it, pinned by digest. On another machine:

- CMake 3.25 or newer and Ninja.
- GCC 14 and Clang 20 (`clang-20`, `clang-tidy-20`), with the sanitizer runtimes. A system's default GCC 13 is refused.
- `clang-format` at the version the CI image pins; another version formats differently.
- Python 3 for the checks and the documentation site (`pip install markdown==3.6` for the site).
- Optional: Emscripten for the web build and the browser testbed.

## Build options for development

| Option | Effect |
|---|---|
| `MAUL2D_WERROR` | Warnings as errors, as CI builds. |
| `MAUL2D_SANITIZE` | AddressSanitizer and UBSan. |
| `MAUL2D_TSAN` | ThreadSanitizer. |
| `MAUL2D_COVERAGE` | gcov instrumentation. |
| `MAUL2D_VALIDATE` | `m2World_Validate` asserted after every step. |
| `MAUL2D_SIMD=scalar` | The scalar kernels on any CPU; results are the same bits. |
| `MAUL2D_TEST_PACKAGE` | The `package` test: installs the build and builds `samples/minimal` against it. |
| `MAUL2D_FUZZ` | The libFuzzer targets `fuzz_<area>` (Clang only), with the library itself instrumented by AddressSanitizer and UBSan. |

`M2_ASSERT` is on in every build without `NDEBUG`. A broken invariant traps with `__builtin_trap`, so a debugger or a core dump stops at the failing check.

## Presets and running one test

Every CI cell is a CMake preset in `CMakePresets.json`: a configure, a build and a test preset of the same name, built in `out/<preset>`. `linux-clang-20-asan` is a Debug build on the scalar backend with assertions and world validation on; `linux-clang-20-tsan` adds ThreadSanitizer. Every suite is one executable and one CTest test, labeled with its name:

```sh
cmake --preset linux-clang-20-asan
cmake --build --preset linux-clang-20-asan
ctest --preset linux-clang-20-asan -L joints
./out/linux-clang-20-asan/test_joints
```

On the shared development machine, run the full builds through the workspace's heavy-job wrapper. ThreadSanitizer does not start inside the CI image on that machine; run the TSan preset on the host.

## Tests

Each suite (`test/test_<label>.c`) tests through the public headers; `test/helpers.h` wraps each status call into a one-line create for scene setup, and every test about why a call refuses calls the public function and checks its status. Suites marked `THREADS` in `test/CMakeLists.txt` drive the task hooks with `test/test_task_pool.h`; `concurrency` runs whole worlds on separate threads. `walk` is the seeded API walk (`test/walk.h`), and `soak` (manual) is the long run the nightly simulation uses. `test_guide.c` runs the C snippets of `docs/user/` as written.

## Golden values

`test/hashes.txt` pins the golden scenes ([determinism.md](../internals/determinism.md)). When a change moves results on purpose, regenerate the file from any cell and say in the pull request which lines moved and why:

```sh
out/linux-clang-20-asan/test_golden | grep -E '^M2_[A-Z0-9_]+_HASH=' > test/hashes.txt
```

A line that moves unexpectedly is a determinism bug until proven otherwise. Compare the scalar and AVX2 builds first: a value only one backend gives points at a kernel.

## Benchmarks

`bench/` builds `maul2d_bench`, which times the benchmark scenes (two pyramids, hanging chains of 30 and 2000 links, a joint farm, a water tank) and prints their final hashes, which the golden test also checks. Timings are never a CI gate. A change meant to affect speed gives before and after numbers from one machine in its pull request.

## The testbed

`-DMAUL2D_BUILD_TESTBED=ON` (or the `linux-testbed` preset) builds the interactive testbed on raylib, which CMake fetches. The Pages workflow compiles it to WebAssembly with the documentation site.

## Reproducing a fuzz finding

A finding in CI uploads the target's log and the input that failed (`crash-<hash>`) as the job's artifact. Run the target on that input locally:

```sh
cmake -B out/fuzz -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL2D_FUZZ=ON -DMAUL2D_BUILD_TESTS=OFF
cmake --build out/fuzz
./out/fuzz/fuzz_snapshot crash-<hash>
./out/fuzz/fuzz_snapshot -max_total_time=300 -max_len=4096 out/corpus/fuzz_snapshot
```

Every crash a fuzzer finds becomes a test in the suite of its module, with the smallest input that still fails (`-minimize_crash=1`) or the scene it reduces to.

## Shared family files

`docs/internals/maul-conventions.md`, the build modules in `cmake/`, `src/cpu.c`, `src/cpu.h`, `src/hash.c`, `src/simd.h`, `test/test_harness.h` and the Python tools in `tools/` are synced from the canonical copies in the organization's `.github` repository (`maul/family_sync.py --write maul2d`). CI fails when a copy differs; change the canonical copy first.
