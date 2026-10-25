# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [0.1.0](https://github.com/Rawframe-Project/maul2d/compare/v0.0.1...v0.1.0) (2026-10-25)


### ⚠ BREAKING CHANGES

* **api:** defs and setters refuse values past the new ranges (each named in docs/user/limits.md), M2_JOINT_REACH is 1e9 m, and mass summed from shapes is clamped into the mass and inertia ranges.
* **snapshot:** snapshots and tapes from earlier builds are refused (snapshot format 3, tape format 4), and a snapshot whose mass data its shapes do not give is refused.
* **world:** `m2World_Hash` and `m2World_GetHashParts` return different values. They now cover every setting the step reads, slot positions and id pools, and `m2WorldHashParts` gains a `volumes` field.
* **api:** `m2Body_SetMassData` refuses a mass that is not positive and finite, a center beyond `M2_HUGE`, an inertia that is not finite or is below the mass times the squared distance of the center, and a mass or inertia about the center whose inverse overflows, counting one misuse.
* **geometry:** shapes, chains, outlines, joint defs and motor offsets with a body-local length beyond `M2_HUGE` are refused with `m2_errorInvalid`, and a setter counts one misuse. `m2CreateMouseJoint` refuses a target whose coordinate in body B's frame exceeds `M2_HUGE` on either axis. `m2DecomposeOutline` refuses an outline whose vertex comes within 0.005 m of an edge it does not end.
* **api:** m2World_Step refuses a substepCount above M2_MAX_SUBSTEPS (64), counting one misuse and leaving the world unchanged; it used to run any count of at least 1.
* **geometry:** m2DecomposeOutline refuses with m2_errorInvalid an outline in which a vertex lies on a non-adjacent edge, a vertex repeats, or an edge folds straight back over its neighbor; it used to decompose them.
* **api:** m2Body_SetLinearVelocity, m2Body_SetAngularVelocity and m2Body_SetTargetTransform on a static body change nothing and count one misuse; they used to store the velocity. m2CreateBody refuses with m2_errorInvalid and one misuse a static body def whose linearVelocity or angularVelocity is not zero.
* **api:** m2NormalizeRot, and every rotation the library normalizes, keeps the direction of a finite nonzero input of any length; a component above about 1.8e19 used to give {0, 0}, an infinite component NaN, and lengths below about 1e-19 the identity. m2CreateBody refuses a def whose rotation is zero with m2_errorInvalid and one misuse. m2World_Restore refuses a snapshot whose body rotations are not of unit length, and m2World_Validate reports a body rotation that is not of unit length. The snapshot format version is now 2 and the journal format version 3, so snapshots and tapes from earlier builds are refused.
* **api:** m2Body_SetLinearDamping and m2Body_SetAngularDamping refuse a negative value, negative infinity included, counting one misuse and leaving the damping unchanged, instead of storing 0.
* **api:** m2World_ShatterBody returns an m2Result and takes an int32_t* countOut after the capacity; the count it used to return comes back through countOut. A null bodiesOut with a capacity above 0, which was ignored, and a negative capacity are now refused with m2_errorInvalid and count one misuse.
* **api:** m2IsNormalizedRot returns bool instead of int.
* **api:** m2CreateBody refuses with m2_errorInvalid, counting one misuse, a def whose type is not an m2BodyType or whose position, rotation, linearVelocity, angularVelocity or gravityScale is not finite, or whose linearDamping or angularDamping is negative or not finite. m2CreateWorld refuses with m2_errorInvalid a def whose gravity is not finite or, with particles, whose particleRadius, particleDensity or any of the eight particle strengths (pressure, damping, viscous, powder, spring, elastic, cohesion, near pressure) is infinite. m2World_Restore refuses a snapshot with a negative body damping.
* **api:** m2Joint_SetMotorSpeed, m2Joint_EnableMotor, m2Joint_EnableLimit, m2Joint_SetLimits and m2Joint_SetMaxMotor on a joint other than a revolute, prismatic or wheel joint (m2Joint_SetMaxMotor also takes a motor joint) now change nothing and count one misuse; m2Joint_GetMotorSpeed, m2Joint_GetMaxMotor, m2Joint_IsMotorEnabled, m2Joint_IsLimitEnabled and m2Joint_GetLimits on those joints return zero and count one misuse, except m2Joint_GetLimits on a distance joint, which still reads its length range; for a distance joint, use m2DistanceJoint_SetLengthRange.
* **api:** a stale id now counts one misuse in m2Body_SetAwake, m2Body_Disable, m2Body_Enable, m2Body_IsEnabled, the damping, fixed rotation, motion lock and sleep getters, m2Shape_GetSensorOverlaps, GetTangentSpeed, GetFriction and GetRestitution, m2Joint_GetReactionForce and GetReactionTorque, m2FluidVolume_GetSurface and GetUserData, and m2Chain_GetShapes; a stale particle id counts nothing in any call, m2DestroyParticle and the particle setters included.
* **api:** every call on a world's objects takes the world first; creates, destroys, snapshots, restores, the journal calls and the polygon builders return an m2Result; m2WorldId, m2SetAllocator, m2SetAssertHandler, m2LastResult and workerCount are gone.

### Added

* **api:** put the world first and return statuses ([#12](https://github.com/Rawframe-Project/maul2d/issues/12)) ([72678c7](https://github.com/Rawframe-Project/maul2d/commit/72678c7aed3f83a4d26d718d20f18f73a8720aee))
* **joints:** read a distance joint's spring toggle back ([#21](https://github.com/Rawframe-Project/maul2d/issues/21)) ([fd9d5f2](https://github.com/Rawframe-Project/maul2d/commit/fd9d5f2479750f159cc92e32a26929ec3c4b5259))


### Fixed

* **api:** a negative damping counts one misuse and changes nothing ([#22](https://github.com/Rawframe-Project/maul2d/issues/22)) ([188c2d3](https://github.com/Rawframe-Project/maul2d/commit/188c2d303d033b614baaff12c671b4ab132cc2c4))
* **api:** bound every number the step multiplies ([#40](https://github.com/Rawframe-Project/maul2d/issues/40)) ([bbcd05c](https://github.com/Rawframe-Project/maul2d/commit/bbcd05c294968ea3dbaa27036f8bb5d598d564d8))
* **api:** cap a step's substeps at M2_MAX_SUBSTEPS ([#26](https://github.com/Rawframe-Project/maul2d/issues/26)) ([078fb3d](https://github.com/Rawframe-Project/maul2d/commit/078fb3d9a7fe70189c3133e96fb165abd31ef4ea))
* **api:** check every field of m2Body_SetMassData ([#34](https://github.com/Rawframe-Project/maul2d/issues/34)) ([2edb7f0](https://github.com/Rawframe-Project/maul2d/commit/2edb7f0a31c19cb4bd2ec8d67cc29e6c25b6ba17))
* **api:** compute in the library's float setting on any host thread ([#44](https://github.com/Rawframe-Project/maul2d/issues/44)) ([2201e59](https://github.com/Rawframe-Project/maul2d/commit/2201e59745c404e5d98a15ac5b911304a412f9a8))
* **api:** count a stale id in every call, and none for particles ([#14](https://github.com/Rawframe-Project/maul2d/issues/14)) ([7ef3abe](https://github.com/Rawframe-Project/maul2d/commit/7ef3abeffc3b45c3b1d24c9ebd5e5a1262e57ddc))
* **api:** count host code mistakes made during a replay ([#13](https://github.com/Rawframe-Project/maul2d/issues/13)) ([393cdbc](https://github.com/Rawframe-Project/maul2d/commit/393cdbc6f3511de71d89d3683ffb77e576d58ad3))
* **api:** keep motor and limit calls to the joints that have them ([#15](https://github.com/Rawframe-Project/maul2d/issues/15)) ([99a4316](https://github.com/Rawframe-Project/maul2d/commit/99a43161ed5a5d453446530cf84d49d42b5e44b7))
* **api:** keep the direction of a rotation far from unit length ([#23](https://github.com/Rawframe-Project/maul2d/issues/23)) ([a3d555f](https://github.com/Rawframe-Project/maul2d/commit/a3d555fcf7e55a1e56028e74c47c1ad9f8c66d76))
* **api:** refuse non-finite or unknown values in world and body defs ([#17](https://github.com/Rawframe-Project/maul2d/issues/17)) ([033fa1a](https://github.com/Rawframe-Project/maul2d/commit/033fa1a77f8f6de4fed9c90c9b12b6ef6f6ef709))
* **api:** refuse velocities on a static body ([#24](https://github.com/Rawframe-Project/maul2d/issues/24)) ([70aeb3b](https://github.com/Rawframe-Project/maul2d/commit/70aeb3b47a95e88f88803cc3c78aa1bc4a52887c))
* **api:** return a status from m2World_ShatterBody ([#19](https://github.com/Rawframe-Project/maul2d/issues/19)) ([f90209e](https://github.com/Rawframe-Project/maul2d/commit/f90209e821be575defeed69cc9c640a4fc03b6e8))
* **api:** return bool from m2IsNormalizedRot ([#20](https://github.com/Rawframe-Project/maul2d/issues/20)) ([deabc4e](https://github.com/Rawframe-Project/maul2d/commit/deabc4e928607b16433fb973447b8c004b3cf495))
* **fluid:** keep 32 crossing candidates per broadphase tree ([#38](https://github.com/Rawframe-Project/maul2d/issues/38)) ([6dd9382](https://github.com/Rawframe-Project/maul2d/commit/6dd93821fc80fda22885f55650a8fdb9abd58366))
* **fluid:** keep a far particle's grid cell within range ([#28](https://github.com/Rawframe-Project/maul2d/issues/28)) ([c57b78a](https://github.com/Rawframe-Project/maul2d/commit/c57b78af0d2f1bddd9cbab37e1105284c5d24690))
* **geometry:** bound body-local lengths by M2_HUGE ([#32](https://github.com/Rawframe-Project/maul2d/issues/32)) ([16723a1](https://github.com/Rawframe-Project/maul2d/commit/16723a1712d7988668d62e4ff7081f4b0e384c63))
* **geometry:** refuse an outline that touches itself ([#25](https://github.com/Rawframe-Project/maul2d/issues/25)) ([767717f](https://github.com/Rawframe-Project/maul2d/commit/767717fdfc3fd086ac38546cbf46ea504c49227c))
* **joints:** clamp a joint's separation to M2_JOINT_REACH ([#35](https://github.com/Rawframe-Project/maul2d/issues/35)) ([76ee0df](https://github.com/Rawframe-Project/maul2d/commit/76ee0df641f3b449d5b5741a74b1ade8595a4e25))
* **journal:** refuse a tape record whose bool flags are not 0 or 1 ([#29](https://github.com/Rawframe-Project/maul2d/issues/29)) ([2813c61](https://github.com/Rawframe-Project/maul2d/commit/2813c6194cb920fb16ccc6fbed93e3fb0c6a57be))
* **snapshot:** refuse a snapshot whose arrays do not fit together ([#27](https://github.com/Rawframe-Project/maul2d/issues/27)) ([037bae0](https://github.com/Rawframe-Project/maul2d/commit/037bae0179246e1f02f5c652811a1d678e8ad577))
* **snapshot:** refuse mass data a body's shapes do not give it ([#39](https://github.com/Rawframe-Project/maul2d/issues/39)) ([8aeb3ed](https://github.com/Rawframe-Project/maul2d/commit/8aeb3edcfbb594f1281fd28d3cd1a8178e28e9c6))
* **snapshot:** refuse shape geometry the world would not keep ([#31](https://github.com/Rawframe-Project/maul2d/issues/31)) ([7a3bcf3](https://github.com/Rawframe-Project/maul2d/commit/7a3bcf3872d6b98207dfdd4ac1ed323a8c0c7e31))
* **solver:** saturate every velocity and impulse the step forms ([#42](https://github.com/Rawframe-Project/maul2d/issues/42)) ([43c00dc](https://github.com/Rawframe-Project/maul2d/commit/43c00dcbbc90c2875abe67108d034b69acc0515a))
* the rewind ring is actually circular now ([70d557d](https://github.com/Rawframe-Project/maul2d/commit/70d557d3e54902a8eb6c75ee8102383d0501e9a2))
* **world:** hash every setting the step reads, keyed by slot ([#37](https://github.com/Rawframe-Project/maul2d/issues/37)) ([5cfd93e](https://github.com/Rawframe-Project/maul2d/commit/5cfd93e190f117aec6338d7dee01dca47321740c))
* **world:** ignore a stale id in m2DestroyWorld ([#9](https://github.com/Rawframe-Project/maul2d/issues/9)) ([bbe461b](https://github.com/Rawframe-Project/maul2d/commit/bbe461bf66280ea9351dcdc911a0ccce7a0f7bdd)), closes [#8](https://github.com/Rawframe-Project/maul2d/issues/8)

## [Unreleased]

### Changed

- The README is rewritten. A new section introduces the Maul family:
  what each engine is built for, and the contract, vocabulary, rules
  and tools the two share; both libraries link into one program.
  THIRD_PARTY.md is gone: the library uses no outside code, and the
  testbed's raylib is credited in the README.

### Fixed

- `m2DestroyWorld` with the id of a world that was already destroyed
  does nothing. It used to destroy whichever world had been created in
  the same slot since.
- Two joints that can never both hold (a wheel joint and a slider with
  crossing axes on one body, say) no longer turn the world to NaN.
  Their impulses fought without limit until they overflowed; every
  joint row now holds its accumulated impulse within a bound no
  ordinary scene reaches, so the results of existing scenes are
  unchanged. Found by the weekly deep fuzz; the seed joins the
  regression bank.

## [0.0.1] - 2026-09-24

The first release of the reworked library.

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
- m2WorldDef.enableSleeping sets the sleep master switch at creation
  (default true), as in Maul3D.

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
- Object ids carry their world's generation: the world field (was
  world0) holds the world slot in its low M2_WORLD_SLOT_BITS bits and
  the low bits of the world generation above them, so a world
  recycling a slot refuses the ids of the world before it. An id
  naming no live world now records m2_errorInvalid (it returned
  silently before). M2_EVENT_HASH and M2_JOURNAL_HASH, which digest
  raw ids, are re-pinned; gcc, clang and Debug agree.
- Collision and query filters are 64 bits wide, as in Maul3D:
  categoryBits and maskBits in m2ShapeDef, m2ChainDef, m2QueryFilter,
  m2ExplosionDef and m2Shape_SetFilter/GetFilter are uint64_t, and the
  default mask is UINT64_MAX. The journal records the wider fields, so
  M2_JOURNAL_HASH is re-pinned; gcc, clang and Debug agree.
- Lifetime functions follow the family form P + Create/Destroy +
  Object: m2CreateFluidVolume, m2DestroyFluidVolume and
  m2DestroyParticle (were m2World_CreateFluidVolume,
  m2World_DestroyFluidVolume and m2World_DestroyParticle).
- The snapshot header holds only the magic, the format version, a
  build configuration hash and the six world capacities, as in Maul3D.
  Step count, gravity, wind, the pair count and every pool cursor are
  state table rows, checked before any byte lands; a snapshot from
  another build is refused with m2_errorConfig. M2_JOURNAL_HASH, which
  covers the embedded snapshot, is re-pinned; gcc, clang and Debug
  agree.
- Shared code follows ADR 0004: src/hash.c holds the hash in both
  engines, src/simd.h and src/cpu.c are the same file in both once the
  prefix is renamed, and tools/check_family.py checks the shared files
  against the sibling repository (byte for byte, or after the rename
  for tools/family-renamed.txt) in the weekly CI job.
- src/simd.h gains the equality, division, square root and mask-and
  lanes Maul3D's kernel uses, so both engines carry one SIMD header.
- The README says no language bindings ship yet, and points to
  Maul3D's C# starter kit for the pattern; CI runs the bindings check
  where a repository has bindings.

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
