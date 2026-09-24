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
- Test for contacts past the last graph color (a plank carrying 40
  boxes).
- Test for the floating draft of a circle, a capsule and a rounded box
  at half the water's density.

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
- The step reads as its stages: m2World_Step and m2SolveStep hand
  their passes (the event window, hibernation, proxy refits, touch
  events, particle aging, velocity and position integration, joint
  breaks) to named functions. A length rule now holds functions to 80
  lines and source files to 1000: tools/check_lengths.py runs in CI,
  and the existing exceptions sit in tools/length-exceptions.txt with
  their reasons and a ceiling that may only shrink. Results are bit-
  identical.
- Comments no longer carry development-history markers (slice and rule
  codes, issue numbers, format versions); tools/check_comments.py
  enforces the conventions.md rule in CI next to the length rules.
- m2MakeRot and m2Atan2 are the engine's own: sine and cosine from a
  two-part pi/2 reduction and Taylor series, atan2 from an argument-
  halving identity and a Taylor series, accurate to about 4e-7 instead
  of the previous 2e-3. Every golden hash and bench pin was re-pinned
  (identical across gcc and clang, Release and Debug).
  docs/references.md lists the published sources behind the
  algorithms.
- The broadphase tree is rewritten: surface-area-guided insertion,
  strict AVL balance and moves that keep the proxy index without
  touching the free list. Every golden hash and bench pin is
  unchanged.
- Contact manifolds, ray casts, convex hulls, shape mass and the
  distance and shape cast kernels are rewritten from published sources
  (docs/references.md). Hashes and bench pins move; the pyramid bench
  settles exactly as before.
- Contact solver rewritten: one lane kernel runs every stage, and the
  graph-color overflow goes through the same kernel one constraint at
  a time instead of a separate scalar path. The soft push limit now
  applies to the scaled bias, so every contact pushes out at most 3
  m/s. Contact points alternate their solve order each substep, which
  removes most of the sideways drift of dropped stacks (the 30-row
  pyramid top drifted 8.6 cm, now 0.1 cm).
- Joint solver rewritten on scalar rows: each joint kind builds its
  rows (a Jacobian and a drive: rigid, held, spring or limit) from the
  pose the substep reached, and one row solver, a coupled-pair solver
  and a closed-form point pair serve all eleven kinds. Joint rows now
  follow the bodies' rotation within a step (arms, slide axes, rope
  directions and lengths), the distance and pulley ropes use their
  exact current length, and a singular coupled pair falls back to its
  rows one at a time instead of being skipped. Joint-heavy scenes run
  about 10% slower for now.
- The step reads as its stages: prepare, the substep (forces, warm
  start, biased solve, integrate, continuous, relax), then bounce,
  store and joint breaks; per-body integration split into velocity
  caps, locked axes and the pose update. Results are bit for bit
  unchanged.
- Particle neighbor pass rewritten (src/particle_pairs.c): particles
  sorted by cell are cut into one run per occupied cell, and each cell
  pairs with itself and the four cells ahead of it in key order. Pairs
  now come out cell by cell; the pair set is unchanged.
- Buoyancy volumes rewritten: a shape's immersion (area and centroid)
  comes from the exact circular segment or from clipping its outline
  at the surface; lift, drag and spin drag apply per body and volume.
- Particle model rewritten on double density relaxation (Clavet et al.
  2005) applied to velocities: density and near density from the pair
  weights q^2 and q^3, walls counting as neighbors; pressure above the
  rest density of the fill's square packing; cohesion below it and
  near pressure for tensile particles; powder spacing push; XSPH shear
  for viscous particles; approach damping linear and quadratic in the
  approach speed; shape-matched elastic triads and spring nets. Body
  contacts moved to particle_contacts.c. A tensile droplet now settles
  at the spacing where cohesion meets its near pressure.
- World def: particleTensilePressureStrength and
  particleTensileNormalStrength become particleCohesionStrength and
  particleNearPressureStrength; the default particlePressureStrength
  is 0.2 on the new pressure scale (water packs as densely as before).
- Mover kit redesigned: m2World_CollideMover fills m2MoverPlane
  entries; m2SolveMover returns the translation closest to the wish
  that no plane blocks, found exactly by trying the planes it may rest
  on, with a bitmask of the planes it rests on; m2ClipMoverVelocity
  strips the velocity pointing into those planes. m2CollisionPlane,
  pushLimit, m2SolvePlanes and m2ClipVector are gone.
- THIRD_PARTY.md now lists only the testbed's raylib: every kernel
  once adapted from Box2D, LiquidFun or Chipmunk2D has been replaced
  by the engine's own code, and a token-level similarity scan against
  those projects finds nothing beyond trivial shared idioms. The
  README credits the published work in docs/references.md; comments
  and docs that measured the engine against another engine now state
  its own rules.
- Joint point pairs take their mass inverse once per step at the
  prepare arms instead of rebuilding and inverting it every pass
  (chain scenes about 7% faster), and the pose update runs the
  rotation helpers inline; the public m2MakeRot, m2MulRot,
  m2NormalizeRot and m2UnwindAngle are the same inline code, so
  rotations are bit for bit unchanged.
- m2Hash64 hashes eight bytes a round (xor, multiply by an odd
  constant, fold the high half) and the leftover bytes one at a time,
  instead of FNV-1a over every byte: about three times faster on a
  whole world. Every hash value changes; the golden hashes and bench
  pins are re-pinned, and gcc, clang and Debug agree.
- Public names follow the family rules: m2Aabb, m2AabbResult,
  m2World_OverlapAabb, m2World_OverlapParticlesAabb, m2Shape_GetAabb,
  m2Body_ComputeAabb and the drawAabbs flag spell the acronym as a
  word; m2Shape_CastRay, m2World_GetJournalBaseSize and
  m2World_GetHashParts lead with their verb; m2World_GetMemoryUsage
  returns an m2MemoryUsage, as in Maul3D. Forces and impulses take
  Maul3D's names: m2Body_ApplyForce and m2Body_ApplyLinearImpulse act
  at the center of mass, m2Body_ApplyForceAtPoint and
  m2Body_ApplyLinearImpulseAtPoint at a world point.
- m2SetAllocator takes Maul3D's hooks: alloc(size_t bytes, void*
  context) and free(memory, context) with one context pointer, both or
  neither (a half pair is refused). The alloc hook may return
  uninitialized memory; the engine zeroes it.

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
- m2World_Restore checked the header but trusted the arrays: a
  corrupted snapshot could carry out-of-range body, shape, joint,
  particle or tree indices, an oversized polygon count, invalid flags
  or non-finite state straight into the world. Every snapshot block is
  now checked against its description in the state table before any
  byte lands; a failed check refuses (m2_errorConfig for another build
  or world shape, m2_errorInvalid otherwise) and leaves the world
  untouched. m2UnwindAngle no longer asserts on angles beyond 1e6
  radians; it returns the documented clamped boundary.
- `m2World_ShatterBody` reports `m2_errorCapacity` when the world
  cannot seat every piece, and refuses a malformed piece polygon up
  front. Before, a full world returned 0 silently and a bad polygon
  left a shapeless piece body behind.
- `m2CreateChain` checks its points first: a NaN point or a zero-
  length segment is refused as `m2_errorInvalid` (before, the segments
  were built unchecked), and a full chain or shape pool is
  `m2_errorCapacity` instead of `m2_errorInvalid`. A chain that builds
  no segment no longer burns a chain id, which made a replay of the
  session mint different ids.
- Shape defs refuse an infinite density or friction and a NaN or
  infinite tangent speed.
- Capsule rotational inertia missed the offset of the end caps'
  centroids, and rounded polygons had the mass of their unrounded
  core. Both now match their closed forms.
- A ray starting inside a rounded polygon reported the exit point as a
  hit; it now hits at fraction 0 with a zero normal, like every other
  solid shape.
- GJK reported rounding noise as a small positive distance, with a
  noise normal, when the origin fell on the simplex of overlapping
  cores; it now reports the overlap.
- Buoyancy volumes ignored a rounded polygon's radius when measuring
  its submerged area, so rounded shapes floated too low or sank: a
  rounded box of half the water's density sank to the bottom. Rounded
  polygons and capsules now become polygons whose corner fans keep the
  true area, instead of a sharp core or a bounding-box fraction.
- A graph color stopped taking contacts at 256, a leftover of an old
  packing buffer, and every contact past it went to the serial
  overflow: in a 4000-box pile 87% of the contacts solved one by one.
  Colors now take any number of contacts; that pile steps in 3.4 ms
  instead of 5.9.
