# Snapshots and the journal

A snapshot is the whole simulation as one flat block; a journal tape is a snapshot followed by the calls made since. Both are built on one description of the world's state, so neither can forget a field the other keeps.

## The state table

`src/world_state.c` lists every array a world owns, with its element size, its count (a capacity or a live cursor), whether it is state or step scratch, and the rule its values must follow. Creation allocates the pools from that table and destruction frees them, and one walk over the state entries sizes, writes, checks and restores a snapshot. A new array added to the table is therefore allocated, saved, checked and restored at once. The walk's measuring mode is the only definition of a snapshot's size. `m2World_Hash` comes from the same table: an entry marked `M2_STATE_HASH` is folded in, a per-object array by live slot keyed by the slot index, a counted array (pairs, springs, triads) up to its count. Each id pool's cursors, free queue in order and generations follow, since they decide where the next object lands and the id it gets. `m2World_GetHashParts` folds the same entries by part (world, bodies, contacts, joints, particles, volumes), each from its own seed. A new state array therefore states whether it is hashed. Each live shape's fat box and moved flag are hashed too, since pairs are found by querying the moved shapes' fat boxes. Left out on purpose: user data, which the step never reads; the tree's shape around the boxes, which decides only the order a query yields leaves in, so every capped query keeps the lowest shape slots ([determinism.md](determinism.md)); and the liveness arrays, which the slot keys already show.

## The state prefix

Every state format starts with the same 16 bytes, shared with maul3d, each a little-endian `uint32_t`:

| Offset | Field | Meaning |
|---|---|---|
| 0 | `magic` | Four ASCII bytes naming the library and the format, in order, so a hex dump of the bytes reads the name. Read as a little-endian `uint32_t`, it is the value whose bytes spell the name. |
| 4 | `abiVersion` | The writer's `M2_ABI_VERSION`. A reader refuses any other value with `m2_errorVersion`. |
| 8 | `formatVersion` | The format's own number, bumped whenever its bytes change. A reader refuses any other value with `m2_errorVersion`. |
| 12 | `headerBytes` | The size of the whole header, this prefix and the format's own fields, a multiple of 8. The content starts there. A reader refuses any other value with `m2_errorInvalid`. |

A reader checks in this order:

- Bytes shorter than the prefix, or with another magic, are not this format: `m2_errorInvalid`.
- Another ABI version or format number means another release wrote them: `m2_errorVersion`.
- Another header size, or bytes too short to hold the header, are corrupt: `m2_errorInvalid`.
- The format's own fields follow. Another build hash is `m2_errorVersion`; another world shape is `m2_errorInvalid`.

None of these refusals counts as misuse. A header embedded in another, such as a tape's snapshot, is checked the same way, and the outer reader returns its refusal.

The golden lines that pin a format's bytes hash them from offset 8, with each embedded header's `abiVersion` zeroed. A separate check confirms that each of those words is this release's, so a release bump moves no line.

`src/world_internal.c` writes and checks the prefix for both formats (`m2MakeStatePrefix`, `m2CheckStatePrefix`). The ABI version names the release. Before 1.0.0 it is the whole version, so only the release that wrote a snapshot or tape reads it. From 1.0.0 it is the major and minor version: the patch releases of one minor version keep the state layout and read each other's bytes, and another minor version refuses them.

## Snapshot format

A snapshot is a 48-byte header followed by the arrays in table order, raw and little-endian, floats as their IEEE bits:

- the prefix, with `magic` `M2SN` and the snapshot format number `M2_SNAPSHOT_VERSION`;
- `configHash` at offset 16: the format number and the settings that change what the bytes mean (float and position widths, the floating-point policy). The SIMD backend and the worker count are not in it, because the bytes are the same under all of them;
- the six capacities of the world that wrote it, at offset 24.

Any change to the bytes bumps the format number.

## Restoring untrusted bytes

`m2World_Restore` treats its buffer as hostile and checks all of it before a byte lands: the header against this build and this world's shape, then each array by its rule (indices within their pools, counts within capacities, flags that are flags, pair keys in order, tree links within the node pool, finite positions, joint anchors, axes and body centers of mass within `M2_HUGE`, rotations of unit length, and every number the step multiplies within its range in `base.h`: velocities and speeds, gravity and scales, dampings, drags and spring rates, coefficients, densities, forces and budgets, every joint and contact impulse within `M2_IMPULSE_LIMIT`, and the last step's 1 / substep). Each range check is the validator the defs, the setters and replay use (`src/core.h`), so a value one path accepts every path accepts. Every body slot holds a unit rotation, a slot never used the identity, so the rotation rule holds for the whole array.

Then `src/snapshot_check.c` checks how the arrays fit together, still on the incoming bytes: each free queue holds every dead slot that is not retired exactly once; each body's shape list holds exactly its live shapes and ends; chains and joints belong to live bodies; particle springs and triads join live particles; each live joint holds, in the fields its kind gives its own meaning, what its def and setters allow there (a distance joint's rest length and range, a gear's or pulley's ratio, a motor's or mouse's budget, a ratchet's tooth, angle and translation limits), while the state the step grows without a bound (a gear's phase, a ratchet's angle and tooth) need only be finite; each broadphase tree is balanced, with links that agree both ways, heights and boxes that follow the children, leaves that are exactly the live shapes of enabled bodies of its type, and a free list holding every other node; the moved list matches its flags; the pairs are in order and join live shapes in the broadphase; and each shape of a static or sleeping body lies inside its fat box at the body's pose. The step refits a body's boxes only while it is awake and not static, so those boxes are used as they are; a world refits a body's boxes as it falls asleep, so every snapshot it writes keeps the rule. An awake body's boxes lag its pose by the last step in every snapshot, and the next step refits them before it queries anything. These are the structures a later call or step indexes through, so no snapshot that passes can drive the world out of its pools, around a loop or past the query stack. The check needs no allocation: trees are walked without a stack, down a child only after the child names its parent, and the free queues are proven distinct with `restoreMarks`, one byte per slot of the largest pool, which the world allocates at creation and the check clears before each use. Only then is the block copied in. A refusal of the arrays returns `m2_errorInvalid`, and a refusal of the header follows [the state prefix](#the-state-prefix); neither counts misuse, so a hostile peer cannot move the counter, and the world is untouched.

Together the checks make every index a later call or step follows safe, every list a list and every tree a balanced tree, so a crafted snapshot can never drive the world outside its pools, around a loop or past the query stack. They also hold derived data to what the world would compute: a body's mass data must equal, bit for bit, what its shapes give it through the same sum the world runs (`m2BodyMassFromShapes` in `src/mass.c`), unless the body carries the override flag `m2Body_SetMassData` sets, and then it must be a dynamic body's data that the setter's own rules accept. Other derived values, such as manifolds and warm-start impulses, are held only to their per-array rules and ranges. A restored world is otherwise exactly the one that was saved: the event streams clear, and the step continues as the original would have.

## Tape format

A tape is a 40-byte header, the embedded snapshot with its own header, then records:

- the header: the prefix, with `magic` `M2JN` and `M2_JOURNAL_VERSION`; then the recording world's gravity at offset 16, and its body, shape and joint capacities and the snapshot's size from offset 24;
- each record: one op byte and that op's payload, a raw struct from `src/journal.h`. Most ops are one public call with its arguments; a chain, a shatter and a restore carry their variable-length data after a fixed header.

Every public writer records itself when a journal is running, after its checks pass, so a refused call leaves no record. A writer that is one operation records once: a chain records itself, not the segment shapes it creates, and its replay creates them again. A step is a record too, with its `dt` and substep count. A record that does not fit marks the tape as overflowed, and `m2World_StopJournal` reports it rather than return a tape that is silently short.

## Replay

`m2World_ReplayJournal` checks the header against the world, restores the embedded snapshot (a refusal of the snapshot is the tape's, `m2_errorVersion` included), then applies each record through a table of commands (`src/journal_replay.c`) that call the same public entry points the recording called. A create must land on the id the recording saw, and a destroy that succeeded must succeed again; anything else means the tape does not describe this world.

Replay is atomic. Before it starts, the world is snapshotted into scratch memory from its own allocator; a replay that fails part way restores that backup, so a malformed, truncated or mismatched tape never leaves a half-built world. While the records apply, the world is marked as replaying: a value a record carries that the setter refuses is the tape's fault and counts no misuse, while a refusal from host code the replay runs (a task hook, the allocator) is the program's and counts.

The journal version moves whenever the tape's bytes change for the same calls. The golden `JOURNAL_TAPE` line pins those bytes, so a change to a recorded struct or to which calls are recorded shows in review.
