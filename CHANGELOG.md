# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [Unreleased]

Work toward 0.0.1, the first release of the reworked library.

### Added

- `m2Counters.pairOverflow`: body pairs dropped by the last pair
  update because the pair table was full. Dropping pairs used to be a
  debug-only assert.
- `bench/`: benchmark scenes that time Maul2D and pin their final
  world hashes in `bench/pins.txt`. CI fails when a pin moves.
- `MAUL2D_BUILD_SHARED` builds a shared library. Public functions
  carry the `M2_API` export macro and everything else is hidden.
- `test/hashes.txt` holds the expected determinism hashes, and
  `tools/check_hashes.py` compares a test run with them, so a change
  that moves a hash the same way on every platform is caught too.
- The library target requires C11 of its consumers through CMake, and
  a test compiles the umbrella header as C++ so the API stays usable
  from C++.

### Changed

- The version history restarts at 0.0.1. Earlier numbered releases
  were withdrawn.
- Refusing a caller's input no longer asserts in debug builds.
  `M2_ASSERT` is kept for internal invariants; every refusal instead
  records its reason for `m2LastResult` and, for a stale or wrong-kind
  id, counts in `m2Counters.misuse`.
- The CMake options follow one family scheme: `MAUL2D_BUILD_TESTS`,
  `_BUILD_SAMPLES`, `_BUILD_BENCH`, `_BUILD_TOOLS`, `_BUILD_TESTBED`,
  `_BUILD_SHARED`, `_INSTALL`, `_WERROR`, `_SANITIZE`, `_TSAN` and
  `_COVERAGE`. Tests, samples, benchmarks and tools default to on only
  when the project is built on its own, not when it is added with
  `add_subdirectory`.
- Warnings are errors only with `MAUL2D_WERROR=ON` (CI turns it on),
  so a newer compiler no longer breaks a consumer's build.
- The CMake modules in `cmake/`, the package config template and the
  pkg-config template are shared with the sibling engine. The pkg-
  config file is relocatable and no longer lists a thread library the
  engine does not use.
- Every test suite uses the shared `test/test_harness.h` instead of
  its own copy of the check macro.
- CI is the family workflow shared with Maul3D: every test suite runs
  on every cell through CTest, hashes are compared across cells and
  with `test/hashes.txt`, and clang-tidy, the package consumer and a
  shared-library build are checked on every push. Releases ship Linux
  (x64 and arm64), macOS and Windows packages; the portable x64
  package is gone, build with `-DMAUL2D_SIMD=scalar` instead. The
  documentation site now carries the guide, the API reference and the
  changelog, and the browser testbed moves to `/testbed/`.
- The public `maul2d/math.h` header is now `maul2d/core_math.h`, so it
  can never shadow the C library's `math.h` when `include/maul2d`
  lands on an include path.
- The world's arrays are described once, in a table in
  `src/world_state.c`. Allocation, release, the snapshot walk and the
  memory footprint all read that table, so adding an array touches the
  struct and one table row instead of seven places.
  `m2World_MemoryBytes` now also counts the broadphase trees and the
  solver scratch.
- m2World_Step with a non-positive dt or fewer than one substep, and
  the joint parameter getters on a stale id, now refuse with
  m2_errorInvalid in every build instead of asserting in debug builds.
  A getter asked for a parameter its joint kind lacks refuses exactly
  once.
- The journal's op codes and payload structs live in one internal
  header shared by the recorders and replay, and replay dispatches
  through a command table instead of an 800-line switch. The wire
  format is unchanged.
- Each joint type lives in its own src/joint_<kind>.c: its def
  defaults, creation, type-specific API and solver rows, reached
  through a kind table instead of type switches. Joint types, joint
  flags and motion-lock bits are named constants. Results are bit-
  identical.
- m2World groups its 248 fields into per-subsystem blocks (bodies,
  shapes, joints, chains, particles, volumes, broadphase, contacts,
  solver scratch, events, recorder), and world_internal.h holds only
  that layout: cookies, kernels and helpers moved to their module
  headers, with new distance.h, island.h, ccd.h, buoyancy.h and
  query.h. maul2d/base.h includes <stdbool.h> like Maul3D's. Results
  are bit-identical.

### Removed

- `m2SetLastResult` from the public header. It was internal.
- The dual-backend harness and the `MAUL2D_BENCHMARKS` option, along
  with the performance comparison in the README that relied on them.
- `m2AssertFail` and the `M2_ASSERT` macro from the public header.
  They are internal; hosts keep `m2SetAssertHandler`.

### Fixed

- The broadphase checked every candidate pair against every joint in
  the world to decide whether jointed bodies may collide. Each body
  now keeps a list of its joints, so the check costs the body's joint
  count. A 2000-link revolute chain steps 2.2 times faster, and
  `m2Body_GetJoints` and body destruction no longer scan all joints.
- Queries documented as safe to call from many threads at once shared
  one world-owned scratch buffer, so concurrent readers could corrupt
  each other's results. Tree walks now run on a cursor that lives on
  the caller's stack, and overlap queries keep their sorted results
  directly in the caller's array. A four-thread reader test covers it.
- `m2World_CastRayAll` and `m2World_CastCircleAll` silently kept at
  most 64 hits even when the caller's array was larger. They now keep
  as many as the array holds.
- Journal replay is now atomic: a tape that fails part way, whether
  truncated, corrupt or recreating an object under a different id than
  the recording saw, is refused and the world is rolled back to where
  it was. An id mismatch used to be a debug-only assert, so release
  builds replayed onward into a different world.
- `m2World_Restore` refuses a snapshot whose header counters point
  outside the world's capacities before overwriting anything, instead
  of indexing past the arrays afterwards.
- A moving shape met at most 256 others per tree in the broadphase and
  silently ignored the rest. The tree walk now has no fixed limit.
- When both shapes of a pair moved, the pair was collected twice
  before duplicates were removed, so a nearly full pair table could
  drop pairs that fit. Each such pair is now recorded once.
- The pair merge wrote its output into the top of the same buffer it
  was still reading candidates from. With a nearly full table it could
  overwrite unread keys. It now merges into a buffer of its own.
- `m2LastResult` was set by only a handful of refusals and lived in
  one process-wide slot. Every refusal now records its reason (invalid
  input, full pool or allocation failure, unsupported CPU) and the
  slot is per thread.
- The misuse counter was incremented without synchronization from
  reader-class calls, a data race when readers run in parallel. It is
  now updated atomically.
- The generated API reference merged declarations that followed a
  trailing comment into the previous entry, so it listed 276 functions
  where the headers declare 289. `tools/gen_api.py` now parses each
  declaration on its own and is shared with Maul3D.
- Input checks that tested `x == x` let infinities through. They now
  use finite checks, so infinite positions, velocities and parameters
  are refused like NaN.
- The public headers used `_Static_assert`, which C++ and MSVC's
  default C mode reject, so a consumer that did not ask for C11 could
  not include them. The layout checks moved into the library's own
  sources.
- The CPU check that lets an AVX2 build refuse to run on an older CPU
  was compiled with the AVX2 flags itself, so the compiler was free to
  use AVX2 instructions before the check ran. It now lives in
  `src/cpu.c`, compiled without the backend's architecture flags.
- MSVC on arm64 ignores the `/fp:contract-` switch, so floating-point
  contraction was not reliably off there. Every source now turns it
  off with `#pragma fp_contract(off)` under MSVC, and the switch is
  only passed where the compiler knows it.
- Joint, shape and body parameter setters validate in one place for
  live calls and replay: non-finite or out-of-range values (a NaN
  motor speed or gear ratio, unordered limits, a negative max motor,
  an infinite friction) and parameters the joint kind lacks are
  refused and counted, where several were accepted before. A tampered
  journal channel byte is rejected instead of asserting in debug
  builds or writing an unrelated field; a stale m2DestroyJoint or
  shape setter now refuses; journal start, stop and replay record a
  reason when they fail (m2_errorConfig for a tape from another world
  shape); a failed shatter replay no longer leaks its pieces.
- The shape casts, overlaps, sweeps and
  m2World_FillPolygonWithParticles copied or read a caller polygon's
  vertices up to its count without a bound, so a count above
  M2_MAX_POLYGON_VERTICES overran a stack buffer; a NULL circle,
  capsule or polygon crashed. Every query now builds its proxy through
  one checked path and refuses bad shapes, non-finite poses, rays and
  boxes, and non-unit rotations.
- Body velocity, force, impulse, torque and teleport setters, gravity,
  wind drag, density, chain materials, motor offsets, mouse targets
  and particle lifetimes refuse non-finite values, which could poison
  the whole world; m2Body_SetTransform also refuses a non-unit
  rotation, which it stored as given and which sheared the body.
- m2World_Step refuses an infinite dt.
- Joint creation validated the bodies and the cookie but none of the
  def's numbers: a NaN or infinite anchor, axis, offset or target, a
  negative stiffness, damping or budget, an inverted limit or length
  range, or a correction factor outside [0, 1] was accepted and fed to
  the solver. Every joint kind now checks its def and refuses such
  input.
