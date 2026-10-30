# Rollback and replay

Maul2D promises three things together:

1. **The same bits everywhere.** The same calls on the same world reach the same bits on Linux, Windows, macOS and WebAssembly, on x64, 32-bit x86, arm64 and wasm32, under GCC, Clang and clang-cl, in Debug and Release, at every worker count, and whatever rounding mode or flush-to-zero setting the host's threads run in. Every CI cell checks it on every commit.
2. **Exact rollback.** `m2World_Snapshot` captures the whole simulation as one flat block, and `m2World_Restore` brings it back exactly. Simulating again from a snapshot reproduces the original run to the last bit, events included.
3. **Replayable history.** The journal records everything done to a world (creation, steps, impulses, teleports, tuning, even restores) and replays it into a fresh world onto the same bits.

## Fixed steps and smooth rendering

Step the world at a fixed `dt`, never tied to the frame rate. For presentation, keep the previous step's poses on the game's side (enumerate the bodies and copy their transforms), and each frame draw the blend between the previous and current poses by alpha = accumulator / dt. The readers are cheap, and the simulation stays exact because the blend never feeds back. The testbed's platformer shows the difference: I cycles through 60 Hz, 30 Hz snapped and 30 Hz blended.

## Snapshots and rollback netcode

Snapshot every confirmed frame. When a late input arrives, restore, apply the corrected inputs and step again to the present. The re-simulation lands on the bits the original would have reached, so a divergence can only come from the game's own input handling.

```c
int64_t size = m2World_GetSnapshotSize(world);
void* frame = malloc((size_t)size);
int64_t written = 0;
bool saved = frame != NULL && m2World_Snapshot(world, frame, size, &written) == m2_success;

m2Body_ApplyLinearImpulse(world, ball, (m2Vec2){3.0f, 0.0f}); // a mispredicted input
m2World_Step(world, 1.0f / 60.0f, 4);

if (saved && m2World_Restore(world, frame, written) == m2_success)
{
    m2World_Step(world, 1.0f / 60.0f, 4); // the corrected frame
}
free(frame);
```

A snapshot's size is fixed for a world's life, so one buffer per frame of history can be allocated up front. `m2World_Snapshot` with a buffer that is too small returns `m2_errorCapacity` and the size it needs. A snapshot that fails leaves the buffer zeroed over its capacity, so an earlier snapshot in it is never taken for the new one.

## The journal

A journal records into a buffer the program owns: a snapshot of the world when recording starts, then one record per call that changes the world. `m2World_GetJournalBaseSize` is the fixed part; size the buffer as that plus room for the session. A buffer that overflows is reported when recording stops, never cut short silently.

```c
int64_t capacity = m2World_GetJournalBaseSize(world) + 64 * 1024;
void* tape = malloc((size_t)capacity);
int64_t tapeSize = 0;
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

When two runs that should match drift apart, `m2World_GetHashParts` says where and when. Compare the parts after every step: the first step where any part differs names the moment, and the part names the subsystem (world: gravity, wind or sleeping; bodies: a body, a shape, a chain or their id pools; contacts: pairs or manifolds; joints: settings or impulses; particles: the fluid; volumes: the buoyant water). If the engine's own checks are green on the platforms involved, the cause is almost always floating-point work in game code feeding the world different inputs; recording a journal on both sides confirms it, since equal tapes replay to equal bits.

## Bytes from outside

Snapshots and tapes are checked before any byte lands. One from another world shape (other capacities or another particle radius), an index or count out of range, a flag that is not a flag, a non-finite number, or a value past its range in [limits.md](limits.md) is refused with `m2_errorFormat`, and one another release or build wrote with `m2_errorVersion` ([below](#versions-and-formats)); either way the world stays as it was. Such a refusal counts no misuse, so a corrupt file or a hostile peer cannot move `m2Counters.misuse`. Code the world runs for the program during a replay (a task hook, the allocator) is still the program's, and what it gets wrong counts. The checks make every index safe to use; they do not rebuild every structural invariant, so restore only snapshots this library wrote.

## Versions and formats

A snapshot and a tape are read only by the release that wrote them. Each carries the library's binary interface version (`M2_ABI_VERSION`), its own format number and, for a snapshot, a hash of the build settings that change what its bytes mean; bytes from another release or build are refused with `m2_errorVersion` before a byte lands, and bytes from a world of another shape with `m2_errorFormat`. Before 1.0.0 every release refuses the bytes of every other. From 1.0.0 a patch release reads the snapshots and tapes of the minor version it belongs to, and a minor release refuses them. A silently reinterpreted byte would be a corrupted world. Snapshots and tapes are tools for rollback and replay within one release, not archives: persist the game state that matters at the game's own layer, and keep it, not snapshots or tapes, across an upgrade.
