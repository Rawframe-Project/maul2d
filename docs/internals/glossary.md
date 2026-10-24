# Glossary

**Body.** A rigid object with a pose and a velocity: static, kinematic or dynamic. Its shapes give it collision and mass.

**Bullet.** A dynamic body swept continuously against non-bullets, so it cannot tunnel through thin walls.

**Chain.** A run of one-sided segments with ghost points, one id for the whole run.

**Cookie.** The value in a def's `internalValue` that its default function sets. A create refuses a def without it, which catches a def that was never initialized.

**Def.** The struct a create takes, initialized by its `m2Default...Def` function.

**Dominance.** A body's rank in contacts: the higher side of a pair acts as static toward the lower one.

**Ghost point.** The neighbor point stored with a chain segment, which keeps a body sliding over a seam from catching on the segment's end.

**Golden value.** A hash in `test/hashes.txt` that pins a promised output; see [determinism.md](determinism.md).

**Graph coloring.** The planning of contacts into groups whose members touch no common body, so a group solves in SIMD lanes and in parallel without changing the result.

**Hibernation.** The state of a world where every dynamic body sleeps and nothing kinematic moves: a step only counts itself.

**Host code.** Code of the program that the world calls: the task hooks, the debug draw callbacks, the allocator. It may read the world but not change it.

**Id.** A small value naming an object of a world: a 1-based slot and a generation. Stale when its object is gone.

**Island.** A set of bodies connected by touching contacts and joints. An island sleeps and wakes as one.

**Journal.** The record of the calls made to a world, as a tape in caller memory; see [snapshots-and-journal.md](snapshots-and-journal.md).

**Manifold.** The contact points, normal and separations between two touching shapes.

**Misuse.** A call a correct program never makes: an invalid argument, a stale id, a writer called from host code. Counted in `m2Counters.misuse`. Untrusted bytes and full pools are not misuse.

**Mover.** The character kit: `m2World_CollideMover`, `m2SolveMover` and `m2ClipMoverVelocity`, for a capsule the game moves itself.

**Pair.** Two shapes whose fattened boxes overlap, kept in a list sorted by key. A pair touches when its manifold has points.

**Reader and writer.** A reader takes a const world and may run with other readers; a writer changes the world and needs it alone.

**Sensor.** A shape that reports overlaps and never pushes back.

**Slop.** The engine's linear tolerance, 0.005 m: shapes within it count as touching, a distance joint's range is floored at it, and a particle's radius at four times it.

**Snapshot.** The whole simulation state as one flat block of bytes.

**Soft step.** The solver's substep schedule: a biased pass with soft constraints that pushes error out, integration, and an unbiased relax pass that removes the velocity the push added.

**Substep.** One of the solver's subdivisions of a step.

**Tape.** A journal's bytes: a header, a snapshot and one record per call.

**Warm start.** Starting a constraint's solve from the impulse it ended the last step with, kept per contact point and joint row.
