# Rollback and replay

Maul2D promises three things together:

1. **The same bits everywhere.** The same calls on the same world reach the same bits on Linux, Windows, macOS and WebAssembly, on x64, arm64 and wasm32, under GCC, Clang and clang-cl, in Debug and Release, at every worker count. Every CI cell checks it on every commit.
2. **Exact rollback.** `m2World_Snapshot` captures the whole simulation as one flat block, and `m2World_Restore` brings it back exactly. Simulating again from a snapshot reproduces the original run to the last bit, events included.
3. **Replayable history.** The journal records everything done to a world (creation, steps, impulses, teleports, tuning, even restores) and replays it into a fresh world onto the same bits.

## Fixed steps and smooth rendering

Step the world at a fixed `dt`, never tied to the frame rate. For presentation, keep the previous step's poses on the game's side (enumerate the bodies and copy their transforms), and each frame draw the blend between the previous and current poses by alpha = accumulator / dt. The readers are cheap, and the simulation stays exact because the blend never feeds back. The testbed's platformer shows the difference: I cycles through 60 Hz, 30 Hz snapped and 30 Hz blended.

## Snapshots and rollback netcode

Snapshot every confirmed frame. When a late input arrives, restore, apply the corrected inputs and step again to the present. The re-simulation lands on the bits the original would have reached, so a divergence can only come from the game's own input handling.

```c
int32_t size = m2World_SnapshotSize(world);
void* frame = malloc((size_t)size);
int32_t written = 0;
bool saved = frame != NULL && m2World_Snapshot(world, frame, size, &written) == m2_success;

m2Body_ApplyLinearImpulse(world, ball, (m2Vec2){3.0f, 0.0f}); // a mispredicted input
m2World_Step(world, 1.0f / 60.0f, 4);

if (saved && m2World_Restore(world, frame, written) == m2_success)
{
    m2World_Step(world, 1.0f / 60.0f, 4); // the corrected frame
}
free(frame);
```

A snapshot's size is fixed for a world's life, so one buffer per frame of history can be allocated up front. `m2World_Snapshot` with a buffer that is too small returns `m2_errorCapacity` and the size it needs.

## The journal

A journal records into a buffer the program owns: a snapshot of the world when recording starts, then one record per call that changes the world. `m2World_GetJournalBaseSize` is the fixed part; size the buffer as that plus room for the session. A buffer that overflows is reported when recording stops, never cut short silently.

```c
int32_t capacity = m2World_GetJournalBaseSize(world) + 64 * 1024;
void* tape = malloc((size_t)capacity);
int32_t tapeSize = 0;
bool recorded = tape != NULL && m2World_StartJournal(world, tape, capacity) == m2_success;
for (int32_t i = 0; recorded && i < 60; ++i)
{
    m2World_Step(world, 1.0f / 60.0f, 4);
}
recorded = recorded && m2World_StopJournal(world, &tapeSize) == m2_success;

// In another world built from the same def:
bool same = recorded && m2World_ReplayJournal(replica, tape, tapeSize) == m2_success &&
            m2World_Hash(replica) == m2World_Hash(world);
free(tape);
```

A replay is all or nothing: a tape that is malformed, truncated, or that recreates an object under another id than the recording saw is refused, and the world stays as it was. A rollback during recording is recorded too, and replays faithfully. Two tapes of the same inputs replay to the same bits.

## Hunting a divergence

When two runs that should match drift apart, `m2World_GetHashParts` says where and when. Compare the parts after every step: the first step where any part differs names the moment, and the part names the subsystem (bodies: a pose or a velocity moved; contacts: manifolds disagree; joints: accumulated impulses; particles: the fluid). If the engine's own checks are green on the platforms involved, the cause is almost always floating-point work in game code feeding the world different inputs; recording a journal on both sides confirms it, since equal tapes replay to equal bits.

## Bytes from outside

Snapshots and tapes are checked before any byte lands. One from another library build or another world shape, an index or count out of range, a flag that is not a flag, or a non-finite number is refused with `m2_errorInvalid`, and the world stays as it was. Such a refusal counts no misuse, so a corrupt file or a hostile peer cannot move `m2Counters.misuse`. The checks make every index safe to use; they do not rebuild every structural invariant, so restore only snapshots this library wrote.

## Versions and formats

Snapshots and tapes are exact-byte records of one library version. Every snapshot carries a version stamp, and every tape carries one and an echo of the world def that recorded it; a mismatch is refused before a byte lands. They are tools for rollback and replay, not archives: when a release changes the snapshot or tape format, old bytes refuse to load by design, because a silently reinterpreted byte would be a corrupted world. Persist the game state that matters at the game's own layer, and keep Maul bytes only next to the build that wrote them.
