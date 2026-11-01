# Diagnostics

## Counters

`m2World_GetCounters` reports what the world holds (live objects, awake dynamic bodies, pairs), what its last step did (solver constraints, graph colors) and the quiet facts a release build cannot assert about. Poll it once per frame and alarm on growth:

- `misuseCount`: calls the library refused as bugs of the calling code (an argument a correct program never passes, a destroyed object's id, a change made from code the world is running).
- `particleCreatesRefused`: particle creates refused by a full particle pool.
- `pairsDropped`, `particlePairsDropped`, `particleBodyContactsDropped`: pairs and contacts dropped last step because a fixed budget was full. Each stops at `INT32_MAX` instead of wrapping.

A field ending in `Count` counts things the world holds or did, one ending in `Dropped` counts work lost without telling the caller, and one ending in `Refused` counts calls refused with a status.

```c
m2Counters counters = m2World_GetCounters(world);
if (counters.misuseCount > lastMisuse)
{
    lastMisuse = counters.misuseCount; // log the frame: some call this frame was a bug
}
```

Two worlds given the same calls report the same counters, but only the counts of what the world holds and its step count are world state, which a restore brings back. The last-step counts (constraints, colors, the serial bucket, the drops) describe the last step this world completed, and `misuseCount` and `particleCreatesRefused` add up over its whole life. A restore leaves both kinds as they are, so right after a rollback the last-step counts still describe the step before it. A refused replay leaves every count as it was before the call, though misuse that code the replay ran for the program counted stays counted. `m2World_GetProfile` reports the wall-clock time of the last step's stages; times are never deterministic. Neither the counters nor the times enter a snapshot or a hash. `m2World_GetMemoryUsage` reports what the world allocated, which stays fixed for its life.

## Validation

`m2World_Validate` walks every invariant the world keeps (finite poses, velocities and particles, joints and jelly nets attached to live objects, consistent counts, pairs in key order) and returns false at the first broken one. It never stops the program. A build configured with `-DMAUL2D_VALIDATE=ON` and assertions on checks it after every step and stops at the fault.

## Debug drawing

`m2World_Draw` walks the world and calls the drawing callbacks in an `m2DebugDraw`; a null callback skips its primitive. Start from a zeroed one (`m2DebugDraw draw = {0};`), so a field a later minor release adds stays off. Polygon vertices arrive in the body's frame with its f64 origin and rotation, so a renderer can subtract its camera in doubles and stay steady far from the origin. Shapes, joints, contacts, contact forces and bounding boxes switch on separately. The callbacks may read the world but not change it.

## Hashes

`m2World_Hash` hashes every piece of state the step reads and a call can change, in every build: it is how two runs are compared. That covers poses, velocities, forces and contacts, and every setting too: damping, locks, materials, filters, shape geometry, joint parameters, wind, particles and water volumes. Each value is keyed by the slot it sits in, and each id pool's free order is included, so two worlds with the same hash step on to the same bits and give their next object the same id. User data, events and the shape of the broadphase tree are left out; the header lists the rest. `m2World_GetHashParts` hashes each subsystem apart, for finding where two runs diverge ([Rollback and replay](rollback-and-replay.md#hunting-a-divergence)).
