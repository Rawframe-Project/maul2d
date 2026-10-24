# Snapshots and the journal

A snapshot is the whole simulation as one flat block; a journal tape is a snapshot followed by the calls made since. Both are built on one description of the world's state, so neither can forget a field the other keeps.

## The state table

`src/world_state.c` lists every array a world owns, with its element size, its count (a capacity or a live cursor), whether it is state or step scratch, and the rule its values must follow. Creation allocates the pools from that table and destruction frees them, and one walk over the state entries sizes, writes, checks and restores a snapshot. A new array added to the table is therefore allocated, saved, checked and restored at once. The walk's measuring mode is the only definition of a snapshot's size. `m2World_Hash` folds the part of that state a step moves, subsystem by subsystem (world, bodies, contacts, joints, volumes, particles); `m2World_GetHashParts` reports the same parts apart, volumes excepted. Settings, shapes and the user data of bodies, shapes, joints and volumes are in the snapshot but not the hash.

## Snapshot format

A snapshot is a 40-byte header followed by the arrays in table order, raw and little-endian, floats as their IEEE bits:

- `magic` (`M2SN`) and `formatVersion`, the snapshot format number `M2_SNAPSHOT_VERSION`;
- `configHash`: the format number and the settings that change what the bytes mean (float and position widths, the floating-point policy). The SIMD backend and the worker count are not in it, because the bytes are the same under all of them;
- the six capacities of the world that wrote it.

The format number is never the library version: a release that keeps the format reads the snapshots of the one before. Any change to the bytes bumps it.

## Restoring untrusted bytes

`m2World_Restore` treats its buffer as hostile and checks all of it before a byte lands: the header against this build and this world's shape, then each array by its rule (indices within their pools, counts within capacities, flags that are flags, pair keys in order, tree links within the node pool, finite positions and velocities, rotations of unit length). Every body slot holds a unit rotation, a slot never used the identity, so the rotation rule holds for the whole array. Only then is the block copied in. A refusal returns `m2_errorInvalid` and counts no misuse, so a hostile peer cannot move the counter; the world is untouched.

The checks make every index the step will follow safe. They do not rebuild every structural invariant (a tree's balance, a list's links being a list), which would cost a full rebuild, so a crafted snapshot can make a world behave strangely but never read or write outside its pools. A restored world is otherwise exactly the one that was saved: the event streams clear, and the step continues as the original would have.

## Tape format

A tape is a 32-byte header, the embedded snapshot, then records:

- the header: `magic` (`M2JN`), `M2_JOURNAL_VERSION`, the recording world's gravity and its body, shape and joint capacities, and the snapshot's size;
- each record: one op byte and that op's payload, a raw struct from `src/journal.h`. Most ops are one public call with its arguments; a chain, a shatter and a restore carry their variable-length data after a fixed header.

Every public writer records itself when a journal is running, after its checks pass, so a refused call leaves no record. A writer that is one operation records once: a chain records itself, not the segment shapes it creates, and its replay creates them again. A step is a record too, with its `dt` and substep count. A record that does not fit marks the tape as overflowed, and `m2World_StopJournal` reports it rather than return a tape that is silently short.

## Replay

`m2World_ReplayJournal` checks the header against the world, restores the embedded snapshot, then applies each record through a table of commands (`src/journal_replay.c`) that call the same public entry points the recording called. A create must land on the id the recording saw, and a destroy that succeeded must succeed again; anything else means the tape does not describe this world.

Replay is atomic. Before it starts, the world is snapshotted into scratch memory from its own allocator; a replay that fails part way restores that backup, so a malformed, truncated or mismatched tape never leaves a half-built world. While the records apply, the world is marked as replaying: a value a record carries that the setter refuses is the tape's fault and counts no misuse, while a refusal from host code the replay runs (a task hook, the allocator) is the program's and counts.

The journal version moves whenever the tape's bytes change for the same calls. The golden `JOURNAL_TAPE` line pins those bytes, so a change to a recorded struct or to which calls are recorded shows in review.
