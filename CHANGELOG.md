# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [0.6.0](https://github.com/Rawframe-Project/maul2d/compare/v0.5.0...v0.6.0) (2026-10-28)


### ⚠ BREAKING CHANGES

* **fluid:** m2CreateParticle, m2CreateParticlesInPolygon and m2CreateFluidVolume return m2_errorCapacity, counting no misuse, in a world whose pool for them is sized 0, where they returned m2_errorInvalid and counted one misuse.
* **geometry:** m2ComputeHull builds the hull of the whole cloud, not of its first 8 points. It refuses with m2_errorLimit a cloud of more than M2_MAX_HULL_POINTS (256) points, or one whose hull has more than M2_MAX_POLYGON_VERTICES vertices, where it used to return a hull of the first 8 points.
* **api:** refusals return the family's statuses. A writer from host code the world is running returns m2_errorBusy (-10), not m2_errorState. Bad snapshot and tape bytes, a world of another shape included, return m2_errorFormat (-8), not m2_errorInvalid. An argument past a published bound returns m2_errorLimit (-9), not m2_errorInvalid. The bounds are renamed: M2_HUGE to M2_LOCAL_LIMIT, M2_MAX_SPEED to M2_SPEED_LIMIT, M2_MAX_ACCELERATION to M2_ACCEL_LIMIT, M2_MAX_GRAVITY_SCALE to M2_GRAVITY_SCALE_LIMIT, M2_MAX_RATE to M2_RATE_LIMIT, M2_MAX_COEFFICIENT to M2_COEFFICIENT_LIMIT, M2_MAX_DENSITY to M2_DENSITY_LIMIT, M2_MAX_RATIO to M2_RATIO_LIMIT, M2_MAX_MASS to M2_MASS_LIMIT, M2_MAX_INERTIA to M2_INERTIA_LIMIT, M2_MAX_FORCE to M2_FORCE_LIMIT, M2_MIN_TIME_STEP to M2_MIN_STEP, M2_MAX_TIME_STEP to M2_STEP_LIMIT, M2_MAX_ANGLE to M2_ANGLE_LIMIT and M2_JOINT_REACH to M2_REACH_LIMIT.
* **api:** snapshot and journal sizes are int64_t: m2World_SnapshotSize and m2World_GetJournalBaseSize return int64_t, and m2World_Snapshot, m2World_Restore, m2World_StartJournal, m2World_StopJournal and m2World_ReplayJournal take int64_t sizes and int64_t* outputs. M2_MAX_SNAPSHOT_BYTES is removed; m2CreateWorld no longer refuses a world for its snapshot size, except on a 32-bit target when a size_t cannot hold it. The journal tape header is 24 bytes, the state prefix and a 64-bit snapshot size, and a restore record carries a 64-bit size.
* **api:** m2SensorEvents holds m2SensorBeginEvent and m2SensorEndEvent arrays, {sensorShapeId, visitorShapeId, step}, in place of m2ContactBeginEvent and m2ContactEndEvent. A sensor begin no longer carries a normal, contact points or an approach speed.
* **api:** m2RayCastResult and m2RayHit are replaced by m2CastHit, which has no hit flag. m2World_CastRayClosest, m2Shape_CastRay, m2World_CastCircleClosest, m2World_CastCapsuleClosest and m2World_CastPolygonClosest return bool and write the hit through a new last parameter, m2CastHit* hitOut; a null hitOut counts one misuse. m2World_CastRayAll, m2World_CastCircleAll, m2World_CastCapsuleAll and m2World_CastPolygonAll fill m2CastHit arrays.
* **api:** joint stiffness fields are renamed: distance, wheel, motor and mouse hertz and dampingRatio become springHertz and springDampingRatio; revolute and prismatic hertz and dampingRatio become constraintHertz and constraintDampingRatio; weld linearHertz, linearDampingRatio, angularHertz and angularDampingRatio become linearSpringHertz, linearSpringDampingRatio, angularSpringHertz and angularSpringDampingRatio. The revolute def gains enableSpring, which its angular spring now needs. A distance joint is a spring only with enableSpring on; off, it is a rigid rod. m2Joint_SetSpringHertz and m2Joint_SetSpringDampingRatio set a revolute joint's spring, not its pin, which m2Joint_SetConstraintHertz and m2Joint_SetConstraintDampingRatio set; they refuse prismatic, weld, gear, pulley and ratchet joints. m2Joint_SetAngularSpringHertz, m2Joint_SetAngularSpringDampingRatio and their getters are removed in favor of m2Joint_SetSpringHertz on a revolute and m2WeldJoint_SetAngularSpringHertz on a weld.
* **api:** m2AabbResult is renamed m2WorldAabb, and m2World_OverlapAabb and m2World_OverlapParticlesAabb take an m2WorldAabb instead of separate lower and upper corners.
* **api:** m2CreateChain returns m2_errorCapacity, not m2_success, when the shape pool cuts a chain short; chainOut still names the shorter chain.
* **api:** m2WorldDef.enableSleeping is renamed enableSleep, and m2World_EnableSleeping and m2World_IsSleepingEnabled are renamed m2World_EnableSleep and m2World_IsSleepEnabled. Every joint def's collideConnected is renamed enableConnectedCollision, and m2Joint_GetCollideConnected is renamed m2Joint_IsConnectedCollisionEnabled. m2Body_Enable and m2Body_Disable are replaced by m2Body_SetEnabled. m2DistanceJoint_IsSpringEnabled is removed; m2Joint_IsSpringEnabled reads distance and wheel joints. New: m2Joint_EnableConnectedCollision, m2Joint_EnableSpring and m2Chain_IsLoop. Snapshots and tapes from earlier releases are refused.
* **api:** m2World_EmitParticle is replaced by m2CreateParticle and m2World_FillPolygonWithParticles by m2CreateParticlesInPolygon, both taking an m2ParticleDef from m2DefaultParticleDef. The fill returns m2Result, reports its count through countOut, and returns m2_errorCapacity when the pool cuts it short.
* **fluid:** a snapshot header holds the world's particle radius, and a snapshot restores only into a world of the same capacities and particle radius. M2_SNAPSHOT_VERSION is 7.
* **api:** m2BodyDef.fixedRotation, m2Body_SetFixedRotation and m2Body_IsFixedRotation are removed; lock rotation with m2MotionLocks.angularZ. m2Body_GetRotationalInertia is renamed m2Body_GetCentralInertia, and m2Body_GetMass is declared in maul2d/body.h. The journal tape format is version 8, so tapes recorded by earlier releases are refused.
* **joints:** m2Joint_GetHertz, m2Joint_GetDampingRatio, m2Joint_GetAngularHertz and m2Joint_GetAngularDampingRatio are removed in favor of their m2Joint_GetSpring* equivalents; m2Joint_GetLength is renamed m2DistanceJoint_GetLength; m2Joint_GetLimits no longer reads a distance joint's range, which m2DistanceJoint_GetLengthRange now reads.

### Added

* **api:** return the family's format, limit and busy statuses ([#133](https://github.com/Rawframe-Project/maul2d/issues/133)) ([9d6748e](https://github.com/Rawframe-Project/maul2d/commit/9d6748e1386e7ceccb89d26e09f0322a6876297a))


### Fixed

* **api:** report a chain the shape pool cuts short as m2_errorCapacity ([#117](https://github.com/Rawframe-Project/maul2d/issues/117)) ([e9c27de](https://github.com/Rawframe-Project/maul2d/commit/e9c27def2e74a30df43e9e18eb41bbbc8c03dd4a))
* **collision:** sweep a bullet's segments in its bounding circle ([#137](https://github.com/Rawframe-Project/maul2d/issues/137)) ([9501525](https://github.com/Rawframe-Project/maul2d/commit/950152518155a1d9d3643717d0e3892c7bb1b341)), closes [#134](https://github.com/Rawframe-Project/maul2d/issues/134)
* **fluid:** check a fill's polygon as a shape's and bound its rows ([#102](https://github.com/Rawframe-Project/maul2d/issues/102)) ([a529764](https://github.com/Rawframe-Project/maul2d/commit/a529764e8ca9d21f54d6b9d80fef65a8f53e4833)), closes [#101](https://github.com/Rawframe-Project/maul2d/issues/101)
* **fluid:** keep a far or reshaped particle link finite ([#105](https://github.com/Rawframe-Project/maul2d/issues/105)) ([c6f2d60](https://github.com/Rawframe-Project/maul2d/commit/c6f2d600bb77429d531063807c644fadcbab240f))
* **fluid:** refuse a particle or volume pool of 0 as a full one ([#143](https://github.com/Rawframe-Project/maul2d/issues/143)) ([2692811](https://github.com/Rawframe-Project/maul2d/commit/26928112d6034f9f859f63b067a6a83e1e116172))
* **geometry:** hull a whole cloud, past 8 points included ([#141](https://github.com/Rawframe-Project/maul2d/issues/141)) ([7888878](https://github.com/Rawframe-Project/maul2d/commit/7888878be793c6a865395e5a30e99e915fd85037))
* **journal:** copy every recorded def field by field ([#107](https://github.com/Rawframe-Project/maul2d/issues/107)) ([d3b289b](https://github.com/Rawframe-Project/maul2d/commit/d3b289bdfa9447395166405819df1716badb67fc)), closes [#106](https://github.com/Rawframe-Project/maul2d/issues/106)
* **journal:** record a caller's polygon in its canonical form ([#116](https://github.com/Rawframe-Project/maul2d/issues/116)) ([4a2bb54](https://github.com/Rawframe-Project/maul2d/commit/4a2bb54c4d6e75675c394fd21d7ee1354404b9b3)), closes [#110](https://github.com/Rawframe-Project/maul2d/issues/110)
* **query:** cast at circles about the ray's closest approach ([#138](https://github.com/Rawframe-Project/maul2d/issues/138)) ([cc210e3](https://github.com/Rawframe-Project/maul2d/commit/cc210e3e9cec19e35bee788bfe95f115faca6757)), closes [#135](https://github.com/Rawframe-Project/maul2d/issues/135)
* **query:** hold a circle cast's root at 0; take .github v0.0.33 ([#140](https://github.com/Rawframe-Project/maul2d/issues/140)) ([754765e](https://github.com/Rawframe-Project/maul2d/commit/754765ed51fa50d9242e11d330e32de2595ee7cd)), closes [#139](https://github.com/Rawframe-Project/maul2d/issues/139)


### Changed

* **api:** create particles from a def like every other object ([#109](https://github.com/Rawframe-Project/maul2d/issues/109)) ([5d92cb8](https://github.com/Rawframe-Project/maul2d/commit/5d92cb8bc173f437be20d30d0a4941d92b5762fc))
* **api:** drop fixed rotation, keep the mass readers in body.h ([#100](https://github.com/Rawframe-Project/maul2d/issues/100)) ([803a31a](https://github.com/Rawframe-Project/maul2d/commit/803a31aa038fd57eb3a8e7ef0fe95af51496fefa))
* **api:** give sensor events their own sensor and visitor ids ([#125](https://github.com/Rawframe-Project/maul2d/issues/125)) ([550352c](https://github.com/Rawframe-Project/maul2d/commit/550352ced8c3c907845e43f03d95ef6f09291c7a))
* **api:** name def switches and their accessors by one rule ([#114](https://github.com/Rawframe-Project/maul2d/issues/114)) ([2b3af14](https://github.com/Rawframe-Project/maul2d/commit/2b3af144b311a6cad95a4544983bef1f294d58bd))
* **api:** name the world box m2WorldAabb, taken by box queries ([#119](https://github.com/Rawframe-Project/maul2d/issues/119)) ([4d3f1e7](https://github.com/Rawframe-Project/maul2d/commit/4d3f1e792374947fced2d68eab9e9196995bab08))
* **api:** one cast hit type, closest casts return bool ([#121](https://github.com/Rawframe-Project/maul2d/issues/121)) ([e2b46b0](https://github.com/Rawframe-Project/maul2d/commit/e2b46b0d8720ff888c7d2e5a3c909f78054ce4d2))
* **api:** one spring and constraint vocabulary for joints ([#123](https://github.com/Rawframe-Project/maul2d/issues/123)) ([09130be](https://github.com/Rawframe-Project/maul2d/commit/09130be7eeba95c03b8bd00abcde5b5ac04e7c5e))
* **api:** size snapshots and journals in 64 bits, with no cap ([#127](https://github.com/Rawframe-Project/maul2d/issues/127)) ([b10ccb7](https://github.com/Rawframe-Project/maul2d/commit/b10ccb7dc6904f143fee3fed9e64edc7a3f57fa1))
* **joints:** drop alias getters, type the distance readers ([#98](https://github.com/Rawframe-Project/maul2d/issues/98)) ([588c81f](https://github.com/Rawframe-Project/maul2d/commit/588c81f8fc108121cd2a5a9534349462806091f8))

## [0.5.0](https://github.com/Rawframe-Project/maul2d/compare/v0.4.0...v0.5.0) (2026-10-27)

### Changed

- **Breaking:** `m2CreateWorld` refuses with `m2_errorCapacity` a definition whose world would snapshot to more than `M2_MAX_SNAPSHOT_BYTES` (1,073,741,801 bytes, about 840,000 shape slots), before it allocates anything. Every world within that size snapshots, restores and records a journal ([#92](https://github.com/Rawframe-Project/maul2d/pull/92)).

- No simulation result changes. The snapshot (version 6) and journal tape (version 7) formats are unchanged; as before 1.0.0, 0.5.0 reads only its own snapshots and tapes and refuses 0.4.0's with `m2_errorVersion`.

### Fixed

- A world's snapshot, restore and journal sizes are computed in 64 bits, and a snapshot is measured with the same per-array sizes the snapshot walk writes and the restore reads, so very large worlds no longer get a wrong size ([#92](https://github.com/Rawframe-Project/maul2d/pull/92)).
- `m2Shape_GetSensorOverlaps` and `m2World_GetContactData` measure with a null array at any capacity, as documented; a null array with a capacity above 0 used to crash ([#95](https://github.com/Rawframe-Project/maul2d/pull/95)).

## [0.4.0](https://github.com/Rawframe-Project/maul2d/compare/v0.3.0...v0.4.0) (2026-10-26)

### Changed

- **Breaking: snapshots and journal tapes are read only by the release
  that wrote them.** Both formats now start with the same 16 bytes as
  maul3d's: a magic, the writer's `M2_ABI_VERSION`, the format's own
  version and the header's size. Bytes from another release, another
  format version or another build are refused with `m2_errorVersion`
  instead of `m2_errorInvalid`, and so is a tape that starts with another
  release's snapshot. Before 1.0.0 every release is its own ABI version
  ([#65](https://github.com/Rawframe-Project/maul2d/pull/65)).
- **Breaking: the snapshot format is version 6.** It starts with the new
  prefix, and its magic is stored so that a hex dump reads `M2SN`. A
  snapshot written by an earlier release is refused
  ([#65](https://github.com/Rawframe-Project/maul2d/pull/65),
  [#76](https://github.com/Rawframe-Project/maul2d/pull/76)).
- **Breaking: the journal tape format is version 7.** It starts with the
  new prefix, its magic reads `M2JN` in a hex dump, and its
  set-mass-data record carries the inertia about the center of mass. A
  tape written by an earlier release is refused
  ([#65](https://github.com/Rawframe-Project/maul2d/pull/65),
  [#76](https://github.com/Rawframe-Project/maul2d/pull/76),
  [#80](https://github.com/Rawframe-Project/maul2d/pull/80)).
- **Breaking: `m2MassData` takes the inertia about the center of mass.**
  `centralInertia` replaces `rotationalInertia`, which was the inertia
  about the body origin, in `m2Body_SetMassData` and
  `m2Body_GetMassData`. Pass the inertia about the centroid, as most
  formulas give it (a disc's is m r^2 / 2), and drop any parallel-axis
  shift. A small inertia beside a far center is now kept as given: a
  1 kg body with its center 1 m from its origin and a central inertia of
  1e-10 used to read back as unable to turn
  ([#80](https://github.com/Rawframe-Project/maul2d/pull/80)).
- **Breaking on 32-bit x86: the public structs lay out as on x86-64.**
  Every 64-bit member of a public struct (a position's coordinates,
  `userData`, the filter bits, the event steps, the counters) is 8-byte
  aligned through `M2_ALIGN8`, so a program built against the old
  headers on 32-bit x86 must be rebuilt. Other targets keep their layout
  ([#78](https://github.com/Rawframe-Project/maul2d/pull/78)).
- The mouse joint pulls with a spring of the body's own mass, equally
  hard in every direction, with stiffness mass (2 pi hertz)^2. A body
  gripped at its center feels the same pull as before; gripped off its
  center, it is no longer slack across the arm
  ([#79](https://github.com/Rawframe-Project/maul2d/pull/79)).

### Fixed

- 32-bit x86 builds and gives the same bits as every other target. It
  did not build, and its tapes would have differed from x86-64's. GCC 14
  and Clang 20 on 32-bit x86 are now CI cells
  ([#78](https://github.com/Rawframe-Project/maul2d/pull/78)).
- A joint that pins a point on a long arm against a small inertia holds
  it. A 1 kg ball of radius 5 cm on a 1 m revolute pendulum flew about
  760 m within 5 s. The joints now build their point mass from the arms
  each substep reaches, in double, so the revolute and motor joints hold
  a 1 kg body by a point 1 m from its center within 1 cm while its mass
  times arm squared over inertia is up to 1e7
  ([#70](https://github.com/Rawframe-Project/maul2d/pull/70)). With its
  inertia stated about the center, the motor joint holds such a body up
  to 1e8 ([#80](https://github.com/Rawframe-Project/maul2d/pull/80)).
- A rigid weld holds a long arm within 0.25 mm up to a ratio of 1e8: it
  solves its point and its angle together, where solving them in turn
  let the anchor sag 1 cm at a ratio of 100 and more than 1 m from 1e4
  on. The mouse joint no longer throws such a body: from a ratio of 1e4
  on it flung it 90 to 155 m, and now holds it within 7 mm up to 1e7
  ([#79](https://github.com/Rawframe-Project/maul2d/pull/79)).
- A joint spring whose hertz is a subnormal float, with a damping ratio
  of 0, no longer turns every position it touches into NaN on the first
  step. Such a value is accepted by every def, setter and snapshot, and
  the bug was in every release since 0.1.0
  ([#72](https://github.com/Rawframe-Project/maul2d/pull/72)).
- `m2World_Restore` refuses a snapshot whose particle springs or triads
  join a dead particle, where it accepted it and left a world that
  `m2World_Validate` refuses
  ([#73](https://github.com/Rawframe-Project/maul2d/pull/73)).

## [0.3.0](https://github.com/Rawframe-Project/maul2d/releases/tag/v0.3.0) (2026-10-25)

### Changed

- **Breaking: the snapshot format is version 4.** Snapshots carry 32-bit
  slot generations and the particle and fluid volume pools' retired
  counts. A version 3 snapshot, or a tape that starts with one, is
  refused. A tape's records and header do not change, except the size of
  the snapshot it starts with, and the journal stays at version 4. The
  public ids, the API and the ABI do not change. Every golden hash
  moves, but only because the hash covers the wider generations: hashed
  at 16 bits, every world hash keeps its old value
  ([#61](https://github.com/Rawframe-Project/maul2d/pull/61)).

### Fixed

- A destroyed particle or fluid volume id no longer names a live object
  again after 65536 reuses of its slot. A fountain with 1000 particle
  slots emitting 1000 a second aliased the ids it held after about 18
  hours. Every pool now keeps 32-bit generations and retires a slot at
  its last one, so a pool of capacity n makes n × 2^32 objects over a
  world's life (about 136 years with 1000 slots at 1000 a second) where
  bodies, shapes, joints and chains used to run out after n × 65536. A
  restore keeps the pool's spent budget
  ([#61](https://github.com/Rawframe-Project/maul2d/pull/61)).

## [0.2.0](https://github.com/Rawframe-Project/maul2d/releases/tag/v0.2.0) (2026-10-25)

### Changed

- **Breaking:** `m2DecomposeOutline` refuses with `m2_errorInvalid`
  an outline whose pieces all fail validation, such as a triangle
  100 km long and 1 cm tall, where it returned `m2_success` with a
  count of 0. In both the fill and the measure call the count is left
  as it was, as for every refusal
  ([#53](https://github.com/Rawframe-Project/maul2d/pull/53)).
- On AVX2 the solver's min and max are one instruction each, with the
  same bits. The pyramid bench scenes run faster on GCC 14 (best of
  nine rounds in each of two runs: pyramid15 8.1 and 8.9 ms before, 7.6
  and 7.8 ms after; pyramid30 57.7 and 59.7 ms before, 49.7 and 54.5 ms
  after), and every bench hash is unchanged.
  The library is also built with SSE math on 32-bit x86 and the
  compiler's default trapping math, from the family's shared flags
  ([#56](https://github.com/Rawframe-Project/maul2d/pull/56)).

### Fixed

- `m2DecomposeOutline` refuses a clockwise outline far from the
  origin. Its winding test summed the area in float, so near 98304 m a
  small clockwise hexagon summed positive and split into two pieces.
  The area is now summed in double
  ([#53](https://github.com/Rawframe-Project/maul2d/pull/53)).
- Polygon validation accepts small polygons far from their body's
  origin. It summed the area for its thinness test in float, so
  `m2MakePolygon` refused every 0.25 m to 2 m box and right triangle at
  65536 m and 98304 m, most of them at 10000 m, and one at 1000 m.
  `m2ComputeHull`, the pieces of `m2DecomposeOutline`, the shape
  constructors, `m2Shape_SetPolygon`, `m2World_ShatterBody` and
  restore all go through the same check. The area is now summed in
  double, against the same threshold
  ([#53](https://github.com/Rawframe-Project/maul2d/pull/53)).
- A body woken and put back to sleep before a step end, by a long step
  that moves it or by a teleport followed by `m2Body_SetAwake(false)`,
  refreshes its contacts. It kept its old sleep streak, so its pairs
  kept the manifold from before the move: in the reproduction the
  contact points sat 0.4 m behind the box. The streak now starts from
  zero wherever a body is put to sleep
  ([#51](https://github.com/Rawframe-Project/maul2d/pull/51)).

## [0.1.1](https://github.com/Rawframe-Project/maul2d/releases/tag/v0.1.1) (2026-10-25)

### Fixed

- A static shape whose geometry a setter changes (`m2Shape_SetCircle`,
  `m2Shape_SetCapsule`, `m2Shape_SetPolygon`, `m2Shape_SetSegment`)
  collides over its new extent. Its broadphase box used to keep the old
  extent, so a ball dropped over the new part of a widened static box
  fell through it. On a dynamic body the box is refitted one step
  earlier than before, with the same result. The world hash differs
  from 0.1.0's after such a change, since the box is part of it
  ([#48](https://github.com/Rawframe-Project/maul2d/pull/48)).
- A restore holds each number that a def, a setter or a tape holds to
  a range to that same range, and checks each joint kind's own fields
  with that kind's validators. A snapshot from outside could restore
  values that turned the world invalid in its first step; such a
  snapshot is now refused, changes nothing and counts nothing. A
  snapshot a world writes still restores, and neither format version
  changes. Two solver paths could store an impulse one float past its
  limit; both now store the limit
  ([#45](https://github.com/Rawframe-Project/maul2d/pull/45)).
- A restore refuses a snapshot in which a shape of a static or sleeping
  body lies outside its broadphase box. Such a snapshot left the step,
  and the host's queries, finding the body where it was not. So that a
  world's own snapshots keep restoring, a body's boxes are refitted
  where it falls asleep, both in the step and in
  `m2Body_SetAwake(false)`. The world hash can therefore differ from
  0.1.0's when a body falls asleep past its box margin; in no golden
  scene does one
  ([#49](https://github.com/Rawframe-Project/maul2d/pull/49)).

## [0.1.0](https://github.com/Rawframe-Project/maul2d/releases/tag/v0.1.0) (2026-10-25)

### Changed

- **Breaking: the family API shape.** `m2World` is an opaque pointer,
  and `m2CreateWorld(def, &worldOut)` returns an `m2Result`. Every call
  on a world's objects takes the world first, and readers take
  `const m2World*`. Ids are `{index1, generation}`, with no world field.
  Creates, destroys, snapshots, restores, the journal calls and the
  polygon builders return an `m2Result` and write through `Out`
  parameters. The world table, `m2WorldId`, `m2World_IsValid`, the
  `*_GetWorld` getters, `m2SetAllocator`, `m2SetAssertHandler`,
  `m2LastResult`, `m2_errorConfig` and `m2WorldDef.workerCount` are
  removed. The allocator is part of `m2WorldDef`, which begins with its
  cookie and an ABI version that `m2CreateWorld` checks
  (`m2_errorVersion`). Worlds share nothing, so separate worlds may be
  created, stepped and destroyed on separate threads.
  ([#12](https://github.com/Rawframe-Project/maul2d/pull/12))
- **Breaking: misuse counting.** `m2Counters.misuse` counts the calling
  code's bugs: an invalid argument, a stale id, a writer called from
  host code. A stale id counts one misuse in every call, while a stale
  particle id counts nothing, since particles die on their own. Corrupt
  snapshot or tape bytes count nothing. Host code a replay runs (the
  task hooks and the allocator) counts like any other caller.
  Motor and limit calls on a joint without a motor or limit change
  nothing and count one misuse; for a distance joint, use
  `m2DistanceJoint_SetLengthRange`.
  ([#13](https://github.com/Rawframe-Project/maul2d/pull/13),
  [#14](https://github.com/Rawframe-Project/maul2d/pull/14),
  [#15](https://github.com/Rawframe-Project/maul2d/pull/15))
- **Breaking: ranges.** Defs, setters and tapes refuse values past the
  ranges in `base.h`, each listed in `docs/user/limits.md`: speeds up to
  1e5 m/s, masses from 1e-9 to 1e12 kg, time steps from 1e-6 to 10 s,
  `M2_JOINT_REACH` of 1e9 m, and the rest. These are the other refusals:
  - **Body-local lengths** beyond `M2_HUGE` (1e5 m) in shapes, chains,
    outlines and joint defs.
  - **Substeps:** more than `M2_MAX_SUBSTEPS` (64) in `m2World_Step`.
  - **Body defs** with a non-finite value, an unknown type, a negative
    damping or a zero rotation, and **world defs** with a non-finite
    gravity or particle setting.
  - **Velocities on a static body.**
  - **Negative dampings**, which used to be stored as 0.
  - **Mass data** whose fields do not fit together in
    `m2Body_SetMassData`.
  - **Outlines:** `m2DecomposeOutline` refuses an outline that touches
    itself, or one whose vertex comes within 0.005 m of an edge it does
    not end.

  ([#17](https://github.com/Rawframe-Project/maul2d/pull/17),
  [#22](https://github.com/Rawframe-Project/maul2d/pull/22),
  [#23](https://github.com/Rawframe-Project/maul2d/pull/23),
  [#24](https://github.com/Rawframe-Project/maul2d/pull/24),
  [#25](https://github.com/Rawframe-Project/maul2d/pull/25),
  [#26](https://github.com/Rawframe-Project/maul2d/pull/26),
  [#32](https://github.com/Rawframe-Project/maul2d/pull/32),
  [#34](https://github.com/Rawframe-Project/maul2d/pull/34),
  [#40](https://github.com/Rawframe-Project/maul2d/pull/40))
- **Breaking: what a restore or replay accepts.** `m2World_Restore`
  refuses a snapshot in any of these cases, and leaves the world as it
  was:
  - its arrays do not fit together (free queues, shape lists, owners,
    broadphase trees);
  - it holds shape geometry the world would not build;
  - its mass data is not what the body's shapes give it;
  - a body rotation in it is not of unit length.

  Replay refuses a tape record whose bool flags are not 0 or 1.
  Snapshots are format 3 and journal tapes format 4, so those written
  by 0.0.1 are refused.
  ([#23](https://github.com/Rawframe-Project/maul2d/pull/23),
  [#27](https://github.com/Rawframe-Project/maul2d/pull/27),
  [#29](https://github.com/Rawframe-Project/maul2d/pull/29),
  [#31](https://github.com/Rawframe-Project/maul2d/pull/31),
  [#39](https://github.com/Rawframe-Project/maul2d/pull/39))
- **Breaking: the world hash.** `m2World_Hash` and
  `m2World_GetHashParts` now also cover:
  - every setting the step reads;
  - the live slots, keyed by index;
  - the id pools;
  - the broadphase's fat boxes.

  So two worlds with the same hash step on to the same bits. Every
  hash value changes, and `m2WorldHashParts` gains a `volumes` part.
  Queries capped at a number of hits keep the lowest shape slots, so
  they choose the same shapes in every world that hashes the same.
  ([#37](https://github.com/Rawframe-Project/maul2d/pull/37),
  [#38](https://github.com/Rawframe-Project/maul2d/pull/38))
- **Breaking:** `m2World_ShatterBody` returns an `m2Result` and hands
  the count back through `countOut`, and `m2IsNormalizedRot` returns
  `bool`.
  ([#19](https://github.com/Rawframe-Project/maul2d/pull/19),
  [#20](https://github.com/Rawframe-Project/maul2d/pull/20))

### Added

- `m2DistanceJoint_IsSpringEnabled` reads a distance joint's spring
  toggle back.
  ([#21](https://github.com/Rawframe-Project/maul2d/pull/21))

### Fixed

- **The host's floating-point setting.** The library computes in its
  own setting (round to nearest, subnormals kept, every exception
  masked) on any host thread. A thread left in flush-to-zero,
  denormals-are-zero or another rounding mode therefore gets the same
  bits. Task hooks, draw callbacks and the allocator run in the host's
  own setting.
  ([#44](https://github.com/Rawframe-Project/maul2d/pull/44))
- **Saturation in the step.** Every velocity saturates at
  `M2_MAX_SPEED`, and every joint and contact impulse at
  `M2_IMPULSE_LIMIT`, and water and wind loads at `M2_MAX_FORCE`. A 2x2
  joint solve whose couplings would overflow solves its rows one at a
  time instead.
  ([#42](https://github.com/Rawframe-Project/maul2d/pull/42))
- **Far-apart joints.** A joint between two bodies far apart clamps
  its separation to `M2_JOINT_REACH`, where it used to leave NaN in the
  bodies.
  ([#35](https://github.com/Rawframe-Project/maul2d/pull/35))
- **Rotations.** Normalizing a rotation keeps its direction for any
  finite nonzero input. Very long, very short or infinite components
  used to give `{0, 0}`, NaN or the identity.
  ([#23](https://github.com/Rawframe-Project/maul2d/pull/23))
- **Far particles.** A particle far beyond the grid's reach no longer
  overflows its cell coordinate.
  ([#28](https://github.com/Rawframe-Project/maul2d/pull/28))
- **Polygons.** A created or set polygon is rebuilt from its vertices,
  so its normals are always the world's own.
  ([#31](https://github.com/Rawframe-Project/maul2d/pull/31))

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
