# Snapshots and the journal

A snapshot is the whole simulation as one flat block; a journal tape is a snapshot followed by the calls made since. Both are built on one description of the world's state, so neither can forget a field the other keeps.

## The state table

`src/world_state.c` lists every array a world owns, with its element size, its count (a capacity or a live cursor), whether it is state or step scratch, and the rule its values must follow. Creation allocates the pools from that table and destruction frees them, and one walk over the state entries sizes, writes, checks and restores a snapshot. A new array added to the table is therefore allocated, saved, checked and restored at once. The walk's measuring mode is the only definition of a snapshot's size. `m2World_Hash` comes from the same table: an entry marked `M2_STATE_HASH` is folded in, a per-object array by live slot keyed by the slot index, a counted array (pairs, springs, triads) up to its count. Each id pool's cursors, free queue in order and generations follow, since they decide where the next object lands and the id it gets. `m2World_GetHashParts` folds the same entries by part (world, bodies, contacts, joints, particles, volumes), each from its own seed. A new state array therefore states whether it is hashed. Each live shape's fat box and moved flag are hashed too, since pairs are found by querying the moved shapes' fat boxes. Left out on purpose: user data, which the step never reads; the tree's shape around the boxes, which decides only the order a query yields leaves in, so every capped query keeps the lowest shape slots ([determinism.md](determinism.md)); and the liveness arrays, which the slot keys already show.

## Snapshot format

A snapshot is a 40-byte header followed by the arrays in table order, raw and little-endian, floats as their IEEE bits:

- `magic` (`M2SN`) and `formatVersion`, the snapshot format number `M2_SNAPSHOT_VERSION`;
- `configHash`: the format number and the settings that change what the bytes mean (float and position widths, the floating-point policy). The SIMD backend and the worker count are not in it, because the bytes are the same under all of them;
- the six capacities of the world that wrote it.

The format number is never the library version: a release that keeps the format reads the snapshots of the one before. Any change to the bytes bumps it.

## Restoring untrusted bytes

`m2World_Restore` treats its buffer as hostile and checks all of it before a byte lands: the header against this build and this world's shape, then each array by its rule (indices within their pools, counts within capacities, flags that are flags, pair keys in order, tree links within the node pool, finite positions and velocities, joint anchors, axes and body centers of mass within `M2_HUGE`, rotations of unit length). Every body slot holds a unit rotation, a slot never used the identity, so the rotation rule holds for the whole array.

Then `src/snapshot_check.c` checks how the arrays fit together, still on the incoming bytes: each free queue holds every dead slot that is not retired exactly once; each body's shape list holds exactly its live shapes and ends; chains and joints belong to live bodies; each broadphase tree is balanced, with links that agree both ways, heights and boxes that follow the children, leaves that are exactly the live shapes of enabled bodies of its type, and a free list holding every other node; the moved list matches its flags; and the pairs are in order and join live shapes in the broadphase. These are the structures a later call or step indexes through, so no snapshot that passes can drive the world out of its pools, around a loop or past the query stack. The check needs no allocation: trees are walked without a stack, down a child only after the child names its parent, and the free queues are proven distinct with `restoreMarks`, one byte per slot of the largest pool, which the world allocates at creation and the check clears before each use. Only then is the block copied in. A refusal returns `m2_errorInvalid` and counts no misuse, so a hostile peer cannot move the counter; the world is untouched.

Together the checks make every index a later call or step follows safe, every list a list and every tree a balanced tree, so a crafted snapshot can never drive the world outside its pools, around a loop or past the query stack. They also hold derived data to what the world would compute: a body's mass data must equal, bit for bit, what its shapes give it through the same sum the world runs (`m2BodyMassFromShapes` in `src/mass.c`), unless the body carries the override flag `m2Body_SetMassData` sets, and then it must be a dynamic body's data that the setter's own rules accept. Other derived values, such as manifolds and warm-start impulses, are held only to their per-array rules. A restored world is otherwise exactly the one that was saved: the event streams clear, and the step continues as the original would have.

## Tape format

A tape is a 32-byte header, the embedded snapshot, then records:

- the header: `magic` (`M2JN`), `M2_JOURNAL_VERSION`, the recording world's gravity and its body, shape and joint capacities, and the snapshot's size;
- each record: one op byte and that op's payload, a raw struct from `src/journal.h`. Most ops are one public call with its arguments; a chain, a shatter and a restore carry their variable-length data after a fixed header.

Every public writer records itself when a journal is running, after its checks pass, so a refused call leaves no record. A writer that is one operation records once: a chain records itself, not the segment shapes it creates, and its replay creates them again. A step is a record too, with its `dt` and substep count. A record that does not fit marks the tape as overflowed, and `m2World_StopJournal` reports it rather than return a tape that is silently short.

## Replay

`m2World_ReplayJournal` checks the header against the world, restores the embedded snapshot, then applies each record through a table of commands (`src/journal_replay.c`) that call the same public entry points the recording called. A create must land on the id the recording saw, and a destroy that succeeded must succeed again; anything else means the tape does not describe this world.

Replay is atomic. Before it starts, the world is snapshotted into scratch memory from its own allocator; a replay that fails part way restores that backup, so a malformed, truncated or mismatched tape never leaves a half-built world. While the records apply, the world is marked as replaying: a value a record carries that the setter refuses is the tape's fault and counts no misuse, while a refusal from host code the replay runs (a task hook, the allocator) is the program's and counts.

The journal version moves whenever the tape's bytes change for the same calls. The golden `JOURNAL_TAPE` line pins those bytes, so a change to a recorded struct or to which calls are recorded shows in review.
