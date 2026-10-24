# Diagnostics

## Counters

`m2World_GetCounters` reports the world's state (live objects, awake bodies, pairs, solver constraints, graph colors) and the quiet facts a release build cannot assert about. Poll it once per frame and alarm on growth:

- `misuse`: calls the library refused as bugs of the calling code (an argument a correct program never passes, a destroyed object's id, a change made from code the world is running).
- `particlePoolFull`: emits refused by a full particle pool.
- `pairOverflow`, `particlePairOverflow`, `particleBodyOverflow`: pairs and contacts dropped last step because a fixed budget was full.

```c
m2Counters counters = m2World_GetCounters(world);
if (counters.misuse > lastMisuse)
{
    lastMisuse = counters.misuse; // log the frame: some call this frame was a bug
}
```

The counters come from world state, so two worlds given the same calls report the same numbers. `m2World_GetProfile` reports the wall-clock time of the last step's stages; times are never deterministic, and neither enters a snapshot or a hash. `m2World_GetMemoryUsage` reports what the world allocated, which stays fixed for its life.

## Validation

`m2World_Validate` walks every invariant the world keeps (finite poses, velocities and particles, joints and jelly nets attached to live objects, consistent counts, pairs in key order) and returns false at the first broken one. It never stops the program. A build configured with `-DMAUL2D_VALIDATE=ON` and assertions on checks it after every step and stops at the fault.

## Debug drawing

`m2World_Draw` walks the world and calls the drawing callbacks in an `m2DebugDraw`; a null callback skips its primitive. Polygon vertices arrive in the body's frame with its f64 origin and rotation, so a renderer can subtract its camera in doubles and stay steady far from the origin. Shapes, joints, contacts, contact forces and bounding boxes switch on separately. The callbacks may read the world but not change it.

## Hashes

`m2World_Hash` hashes the simulation state, in every build: it is how two runs are compared. `m2World_GetHashParts` hashes each subsystem apart, for finding where two runs diverge ([Rollback and replay](rollback-and-replay.md#hunting-a-divergence)).
