# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [0.6.0](https://github.com/Rawframe-Project/maul2d/compare/v0.5.0...v0.6.0) (2026-10-28)

This release settles most of the API before 1.0.0, so nearly every program needs changes to build against it. Each **Breaking** item says what to change.

### Added

- Three new statuses: `m2_errorFormat` (-8), `m2_errorLimit` (-9) and `m2_errorBusy` (-10), named by `m2ResultName` ([#133](https://github.com/Rawframe-Project/maul2d/pull/133)).
- Every fixed bound a call checks is now a public constant. `M2_MAX_*` is a count, `M2_*_LIMIT` a magnitude and `M2_MIN_*` a floor. New ones: `M2_MAX_OUTLINE_POINTS` and `M2_MAX_SHATTER_PIECES` (64 each), `M2_MAX_HULL_POINTS` (256), `M2_MAX_FILL_COLUMNS` (256) and `M2_MIN_PARTICLE_RADIUS` (0.02 m) ([#133](https://github.com/Rawframe-Project/maul2d/pull/133), [#141](https://github.com/Rawframe-Project/maul2d/pull/141)).
- Particles are created from a def like every other object: `m2ParticleDef` and `m2DefaultParticleDef` ([#109](https://github.com/Rawframe-Project/maul2d/pull/109)).
- New calls ([#98](https://github.com/Rawframe-Project/maul2d/pull/98), [#114](https://github.com/Rawframe-Project/maul2d/pull/114), [#123](https://github.com/Rawframe-Project/maul2d/pull/123)):
  - `m2Joint_EnableConnectedCollision` changes a live joint's connected collision;
  - `m2Joint_EnableSpring` turns a distance, wheel or revolute spring on or off;
  - `m2Chain_IsLoop` reads a chain's kind;
  - `m2DistanceJoint_GetLengthRange` reads what `m2DistanceJoint_SetLengthRange` sets;
  - `m2Joint_SetConstraintHertz` and `m2Joint_SetConstraintDampingRatio`, with their getters, soften the rows that hold a revolute, prismatic, gear, pulley or ratchet joint together;
  - `m2WeldJoint_SetLinearSpringHertz`, `SetLinearSpringDampingRatio`, `SetAngularSpringHertz` and `SetAngularSpringDampingRatio`, with their getters, tune a weld's two springs;
  - a motor joint's drive can change while the world runs.

### Changed

- **Breaking:** refusals return the family's statuses ([#133](https://github.com/Rawframe-Project/maul2d/pull/133)):
  - a writer called from host code the world is running (a task hook, a debug draw callback, the allocator) returns `m2_errorBusy`, not `m2_errorState`;
  - bad snapshot and tape bytes, a world of another shape included, return `m2_errorFormat`, not `m2_errorInvalid`, and count no misuse;
  - a value past a published bound, infinity included, returns `m2_errorLimit`, not `m2_errorInvalid`;
  - a NaN, or a value outside its parameter's documented range, is still `m2_errorInvalid`.
  Code that compares a status with `m2_errorInvalid` or `m2_errorState` needs the new cases.
- **Breaking:** bounds are renamed, with their values unchanged ([#133](https://github.com/Rawframe-Project/maul2d/pull/133)):

  | Old name | New name |
  |---|---|
  | `M2_HUGE` | `M2_LOCAL_LIMIT` |
  | `M2_MAX_SPEED` | `M2_SPEED_LIMIT` |
  | `M2_MAX_ACCELERATION` | `M2_ACCEL_LIMIT` |
  | `M2_MAX_GRAVITY_SCALE` | `M2_GRAVITY_SCALE_LIMIT` |
  | `M2_MAX_RATE` | `M2_RATE_LIMIT` |
  | `M2_MAX_COEFFICIENT` | `M2_COEFFICIENT_LIMIT` |
  | `M2_MAX_DENSITY` | `M2_DENSITY_LIMIT` |
  | `M2_MAX_RATIO` | `M2_RATIO_LIMIT` |
  | `M2_MAX_MASS` | `M2_MASS_LIMIT` |
  | `M2_MAX_INERTIA` | `M2_INERTIA_LIMIT` |
  | `M2_MAX_FORCE` | `M2_FORCE_LIMIT` |
  | `M2_MIN_TIME_STEP` | `M2_MIN_STEP` |
  | `M2_MAX_TIME_STEP` | `M2_STEP_LIMIT` |
  | `M2_MAX_ANGLE` | `M2_ANGLE_LIMIT` |
  | `M2_JOINT_REACH` | `M2_REACH_LIMIT` |
- **Breaking:** joints use one spring and constraint vocabulary ([#123](https://github.com/Rawframe-Project/maul2d/pull/123)).
  - Rename the def fields:
    - `hertz` and `dampingRatio` become `springHertz` and `springDampingRatio` on distance, wheel, motor and mouse joints;
    - they become `constraintHertz` and `constraintDampingRatio` on revolute and prismatic joints;
    - weld `linearHertz`, `linearDampingRatio`, `angularHertz` and `angularDampingRatio` become `linearSpringHertz`, `linearSpringDampingRatio`, `angularSpringHertz` and `angularSpringDampingRatio`.
  - A distance joint is a spring only with `enableSpring` on, and is a rigid rod with it off. A revolute joint's angular spring now needs the def's new `enableSpring`. Turn `enableSpring` on wherever a spring used to be on through its hertz alone.
  - `m2Joint_SetSpringHertz` and `m2Joint_SetSpringDampingRatio` set a revolute joint's spring, not its pin, and refuse prismatic, weld, gear, pulley and ratchet joints. Use `m2Joint_SetConstraintHertz` and `m2Joint_SetConstraintDampingRatio` for the pin.
- **Breaking:** one cast hit type ([#121](https://github.com/Rawframe-Project/maul2d/pull/121)).
  - `m2CastHit` replaces `m2RayCastResult` and `m2RayHit`, and has no hit flag.
  - `m2World_CastRayClosest`, `m2Shape_CastRay`, `m2World_CastCircleClosest`, `m2World_CastCapsuleClosest` and `m2World_CastPolygonClosest` return `bool` and write the hit through a new last parameter, `m2CastHit* hitOut`. A miss leaves `hitOut` as it was, and a null `hitOut` counts one misuse.
  - The casts that list every hit fill `m2CastHit` arrays.
- **Breaking:** sensor events have their own types, `m2SensorBeginEvent` and `m2SensorEndEvent`: {`sensorShapeId`, `visitorShapeId`, `step`} ([#125](https://github.com/Rawframe-Project/maul2d/pull/125)).
  - Read the sensor and the visitor directly, instead of testing both shapes of a contact pair.
  - A sensor begin no longer carries a normal, contact points or an approach speed.
- **Breaking:** snapshot and journal sizes are `int64_t` ([#127](https://github.com/Rawframe-Project/maul2d/pull/127)).
  - `m2World_SnapshotSize` and `m2World_GetJournalBaseSize` return it.
  - `m2World_Snapshot`, `m2World_Restore`, `m2World_StartJournal`, `m2World_StopJournal` and `m2World_ReplayJournal` take `int64_t` sizes and `int64_t*` outputs.
  - `M2_MAX_SNAPSHOT_BYTES` is gone, and `m2CreateWorld` no longer refuses a large world for its snapshot size. The one exception is a 32-bit target where a `size_t` cannot hold the size.
- **Breaking:** particles are created like other objects ([#109](https://github.com/Rawframe-Project/maul2d/pull/109)).
  - `m2CreateParticle` replaces `m2World_EmitParticle`.
  - `m2CreateParticlesInPolygon` replaces `m2World_FillPolygonWithParticles`. It returns `m2Result` and reports its count through `countOut`.
  - A fill the pool cuts short keeps what it made and returns `m2_errorCapacity`.
- **Breaking:** names follow one rule ([#114](https://github.com/Rawframe-Project/maul2d/pull/114), [#119](https://github.com/Rawframe-Project/maul2d/pull/119), [#98](https://github.com/Rawframe-Project/maul2d/pull/98), [#100](https://github.com/Rawframe-Project/maul2d/pull/100)):
  - `m2WorldDef.enableSleeping` becomes `enableSleep`, with `m2World_EnableSleep` and `m2World_IsSleepEnabled`;
  - every joint def's `collideConnected` becomes `enableConnectedCollision`, and `m2Joint_GetCollideConnected` becomes `m2Joint_IsConnectedCollisionEnabled`;
  - `m2Body_SetEnabled(world, body, bool)` replaces `m2Body_Enable` and `m2Body_Disable`;
  - `m2AabbResult` becomes `m2WorldAabb`, and `m2World_OverlapAabb` and `m2World_OverlapParticlesAabb` take one `m2WorldAabb` instead of two corners;
  - `m2Joint_GetLength` becomes `m2DistanceJoint_GetLength`, and `m2Joint_GetLimits` no longer reads a distance joint;
  - `m2Body_GetRotationalInertia` becomes `m2Body_GetCentralInertia`, and `m2Body_GetMass` moves to `maul2d/body.h`.
- **Breaking:** a pool sized 0 is full from the start ([#143](https://github.com/Rawframe-Project/maul2d/pull/143), [#117](https://github.com/Rawframe-Project/maul2d/pull/117)).
  - `m2CreateParticle`, `m2CreateParticlesInPolygon` and `m2CreateFluidVolume` return `m2_errorCapacity` and count no misuse in a world whose pool for them is sized 0. They used to return `m2_errorInvalid`.
  - `m2CreateChain` returns `m2_errorCapacity` when the shape pool cuts a chain short. `chainOut` still names the shorter chain.
- **Breaking:** `m2ComputeHull` hulls the whole cloud, up to `M2_MAX_HULL_POINTS` points. It returns `m2_errorLimit` for a larger cloud, or for one whose hull has more than `M2_MAX_POLYGON_VERTICES` vertices ([#141](https://github.com/Rawframe-Project/maul2d/pull/141)).
- **Breaking:** both formats change, so 0.6.0 refuses the snapshots and tapes of 0.5.0 with `m2_errorVersion`.
  - **Snapshot version 7.** The header holds the world's particle radius, and a snapshot restores only into a world with the same capacities and particle radius ([#105](https://github.com/Rawframe-Project/maul2d/pull/105)). Each chain's loop flag is part of the state ([#114](https://github.com/Rawframe-Project/maul2d/pull/114)).
  - **Journal tape version 8.** The header is 24 bytes and a restore record's size is 64 bits ([#127](https://github.com/Rawframe-Project/maul2d/pull/127)). The body and revolute records follow their new defs ([#100](https://github.com/Rawframe-Project/maul2d/pull/100), [#123](https://github.com/Rawframe-Project/maul2d/pull/123)).
- Results: a program that makes the same calls, with the renames above and `enableSpring` turned on wherever a spring was on, steps its bodies, contacts and particles to the same bits as on 0.5.0. Only the fixes below change results. `m2World_Hash` covers each joint's spring switch, so a world whose joints now turn `enableSpring` on hashes differently from 0.5.0, even though it steps the same.

### Removed

- **Breaking:** these have no replacement under the old name:
  - `m2BodyDef.fixedRotation`, `m2Body_SetFixedRotation` and `m2Body_IsFixedRotation`: lock rotation with `m2MotionLocks.angularZ` ([#100](https://github.com/Rawframe-Project/maul2d/pull/100));
  - `m2Joint_GetHertz`, `m2Joint_GetDampingRatio`, `m2Joint_GetAngularHertz` and `m2Joint_GetAngularDampingRatio`: use the `m2Joint_GetSpring*` calls ([#98](https://github.com/Rawframe-Project/maul2d/pull/98));
  - `m2Joint_SetAngularSpringHertz`, `m2Joint_SetAngularSpringDampingRatio` and their getters: use `m2Joint_SetSpringHertz` on a revolute and `m2WeldJoint_SetAngularSpringHertz` on a weld ([#123](https://github.com/Rawframe-Project/maul2d/pull/123));
  - `m2DistanceJoint_IsSpringEnabled`: use `m2Joint_IsSpringEnabled` ([#114](https://github.com/Rawframe-Project/maul2d/pull/114)).

### Fixed

- A bullet whose only shapes are segments or chain segments no longer passes through thin walls. Such a bullet's results change ([#137](https://github.com/Rawframe-Project/maul2d/pull/137)).
- Circle, capsule and rounded polygon casts give the right hit far from the origin and for very long rays, from about 1e9 m. They used to hit the center line, or report a hit at the ray's origin. A ray starting on a circle's surface hits at a fraction of 0, never slightly below it ([#138](https://github.com/Rawframe-Project/maul2d/pull/138), [#140](https://github.com/Rawframe-Project/maul2d/pull/140)).
- `m2ComputeHull` no longer ignores every point after the eighth ([#141](https://github.com/Rawframe-Project/maul2d/pull/141)).
- A particle spring or elastic triad whose ends are farther apart than `M2_REACH_LIMIT` stays finite, by treating the ends as that far apart. Before, it made the world's particle state non-finite. Nearer links keep their results ([#105](https://github.com/Rawframe-Project/maul2d/pull/105)).

### Security

- Recording a create, a particle fill or a shatter no longer writes bytes of the caller's memory into the journal tape. A def's padding and a polygon's normals and unused slots used to reach the tape, so two recordings of the same calls could differ. Every recorded def and polygon is now copied field by field ([#107](https://github.com/Rawframe-Project/maul2d/pull/107), [#116](https://github.com/Rawframe-Project/maul2d/pull/116)).
- A particle fill, or the replay of a tape carrying one, no longer hangs on a huge polygon. The fill checks its polygon as a shape's and bounds its rows ([#102](https://github.com/Rawframe-Project/maul2d/pull/102)).
- A snapshot whose spring rest lengths or triad corners lie outside what a fill makes is refused with `m2_errorFormat`, before it changes anything ([#105](https://github.com/Rawframe-Project/maul2d/pull/105)).

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
