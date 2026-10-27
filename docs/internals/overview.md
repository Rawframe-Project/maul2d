# Overview

Maul2D simulates rigid bodies, joints and particle fluids in a world that snapshots, restores and replays to the bit on every platform it supports. It follows the family rules in [maul-conventions.md](maul-conventions.md); this page states what the library's domain adds and how the parts fit.

## Library profile

- **Determinism:** bit-exact on every platform, compiler, SIMD backend and worker count. It covers the simulation state, snapshots, journal tapes, `m2World_Hash`, the event streams, the counters and every reader. Only `m2World_GetProfile`'s wall-clock times are exempt. [determinism.md](determinism.md) explains how it holds.
- **Threads:** the library starts none. The step hands its parallel work to the host's task system through the world def's `enqueueTask` and `finishTask`; without them the work runs on the caller's thread.
- **Owner objects:** the world, `m2World*`, created by `m2CreateWorld` from a root def that carries `M2_ABI_VERSION`. Bodies, shapes, chains, joints, particles and fluid volumes are objects of a world, named by typed ids.
- **Memory:** a world allocates every byte it keeps at creation, from the allocator in its def; its pools have fixed capacities and never grow, and step scratch lives in them, as do the restore marks the snapshot check uses (one byte per slot of the largest pool). Afterwards only `m2World_ReplayJournal` allocates (its backup and the scratch of chain and shatter records) and frees before it returns. The functions without a world (`m2MakePolygon`, `m2ComputeHull`, `m2DecomposeOutline`, the mover math) never allocate.
- **Data formats:** two state formats, the snapshot and the journal tape, and no asset format. Neither is read across 1.y: only the release that wrote one reads it, by its `M2_ABI_VERSION` (from 1.0.0, the patch releases of its minor version too), and any other refuses it with `m2_errorVersion`. A host keeps its own game state, not snapshots or tapes, across an upgrade. [snapshots-and-journal.md](snapshots-and-journal.md) has the layout.
- **Platform dependencies:** none beyond the C standard library; the `<math.h>` functions it may call are the correctly rounded ones in `tools/libm-allowed.txt`. The testbed, which is not part of the library, uses raylib.

## API names

Every public name follows these rules, so a new one is decided by them rather than by its neighbors.

- **Objects.** An object is made by `m2CreateX(world, def, idOut)` from a def that `m2DefaultXDef` initializes, returns `m2Result`, and goes with `m2DestroyX(world, id)`. A create that makes several (`m2CreateParticlesInPolygon`) reports its count through an out argument.
- **Values.** A value an object keeps is read by `<Obj>_GetX` and changed by `<Obj>_SetX`.
- **Switches.** A def's bool fields come in two kinds. `isX` names what an object is or the state it starts in (`isSensor`, `isLoop`, `isBullet`, `isEnabled`); `enableX` names an optional behavior it runs (`enableSleep`, `enableSpring`, `enableMotor`, `enableLimit`, `enableConnectedCollision`). An `enableX` field gets `<Obj>_EnableX(obj, bool)` and `<Obj>_IsXEnabled(obj)`. An `isX` field that can change gets `<Obj>_SetX(obj, bool)` and `<Obj>_IsX(obj)`. A kind fixed at creation (`isSensor`, `isLoop`) gets only `<Obj>_IsX`.
- **Joints.** A feature several joint kinds share is `m2Joint_X`, and its doc names the kinds; another kind counts one misuse. A feature one kind has uses that kind's prefix (`m2DistanceJoint_SetLength`).

## Ids and misuse

A correct program never uses an id after destroying its object, so a stale id is misuse: it counts one in `m2Counters.misuse`, a getter returns the zeroed value, a setter changes nothing, and a call with a status returns `m2_errorStale`. Joints break on their own, but every break is reported in the joint event stream, so a program that reads its events never holds a broken joint's id unknowingly; joints follow the same rule.

Particles are the exception. A particle with a lifetime dies at the end of a step with no event, so a correct program can hold the id of a particle that is gone. A stale particle id counts nothing in any call: getters return zero, setters change nothing, and `m2DestroyParticle` returns `m2_errorStale`. `m2Particle_IsValid` tells the program.

Untrusted bytes are not misuse either. A snapshot or tape that fails its checks, and a value a tape operation refuses while it replays, return `m2_errorInvalid`, or `m2_errorVersion` for bytes another release or build wrote, and count nothing; the world marks itself as replaying, and `m2Refuse` skips the count for refusals on the replay's own path. Host code the replay runs (a task hook, the allocator) is the program's, and what it gets wrong counts.

## Host code

The world calls host code in three places: the task hooks during a step, the debug draw callbacks during `m2World_Draw`, and the allocator. While it runs, an atomic count on the world is raised, and every writer checks it first: every writer counts one misuse and changes nothing, and a writer with a status also returns `m2_errorState`. Readers stay open, so a draw callback can ask the world about what it draws. The count is atomic because several readers may draw on separate threads at once.

## A step

`m2World_Step` checks its arguments, records the step when a journal is running, and runs the step with the host-code count raised:

1. **Collide.** Moved shapes update their broadphase trees, the pair list is rebuilt in key order, and every pair's manifold is computed again from the current positions. Begin and end events are emitted from the difference.
2. **Fluids and fields.** Particles solve once per step against frozen pairs and age; fluid volumes and wind add their forces to the same accumulators gravity uses.
3. **Islands.** Bodies joined by touching contacts and joints are unioned into islands; an island with an awake member or a moving kinematic neighbor wakes whole.
4. **Solve.** Contacts are planned into graph-colored blocks and solved with the joints in substeps: forces in, warm start, a biased pass that pushes overlap and joint error out, integration, continuous collision for fast bodies, then an unbiased relax pass. Restitution, the stored impulses for the next step's warm start, and the break check run once per step after the substeps.
5. **Sleep.** Each island accounts its members' rest time; one that has rested long enough sleeps whole.

When every dynamic body sleeps and nothing kinematic moves, the world hibernates: the step only counts itself.

## How the parts fit

`tools/modules.txt` declares which module may include which, and `tools/check_modules.py` keeps the graph one-way.

- **World and state:** `world.c` (creation, the step, settings), `world_state.c` (the pools and the state table, which also drives the hash), `world_hash.c` (the id pools' share of the hash), `world_internal.c` (refusals, host code, the bracketed allocator), `allocator.c` (the one file that reaches `malloc`).
- **Objects:** `body*.c`, `shape.c`, `chain.c`, `joint*.c` (one file per joint type), `particle*.c`, `buoyancy.c`, `destruction.c` (shatter and explosions), `object_ids.c`.
- **Collision:** `dynamic_tree.c` and `broadphase.c`, `contact.c` (pairs and events), `manifold.c`, `distance.c`, `geometry.c` (shape math, hulls, decomposition, mass), `ccd.c`.
- **Solver:** `solver.c` (the substep order), `contact_solver.c`, `contact_kernel.c` (the SIMD lanes), `graph_color.c`, `joint_solver.c`, `joint_row.c`, `island.c`, `parallel.c` (the task hooks).
- **Queries:** `query*.c`, `ray_cast.c`, `mover.c`.
- **Recording:** `snapshot.c`, `snapshot_check.c` (how an incoming snapshot's arrays fit together), `journal.c`, `journal_replay.c`. See [snapshots-and-journal.md](snapshots-and-journal.md).
- **Family files:** `cpu.c` and `cpu.h`, `hash.c`, `simd.h`, synced from the organization's `.github` repository and never edited here.

## Assertions

`M2_ASSERT` (`src/core.h`) checks internal invariants only: it traps in builds without `NDEBUG` and compiles out of Release. Caller input is validated and refused with a status in every build. `m2World_Validate` walks the invariants a world can state about itself and returns false at the first broken one; the validate build asserts it after every step.
