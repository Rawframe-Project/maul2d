# Maul2D

Maul2D is a deterministic 2D physics engine for games in C: rigid bodies, eleven joint types, particle fluids, queries and a character mover, in a world that snapshots, restores and replays to the bit. It is part of the Maul family; the family rules are in [docs/internals/maul-conventions.md](docs/internals/maul-conventions.md), and [docs/internals/overview.md](docs/internals/overview.md) explains how the parts fit.

It never compromises on:

1. **Same bits everywhere.** The same calls reach the same bits on every platform, compiler, SIMD backend and worker count. Every CI cell checks the golden values in `test/hashes.txt`.
2. **Exact rollback and replay.** A restore continues exactly as the original run; a journal replays onto the same bits, or is refused and changes nothing.
3. **Hostile input.** Every argument, snapshot and tape is checked and refused with a status in every build, before anything changes. Untrusted bytes never count as misuse.
4. **Fixed memory.** A world allocates its pools once, from its def's allocator; the step never allocates.

## Commands

```sh
tools/check.sh changed          # before every push: rules, format, the asan preset for the labels touched (< 2 min)
tools/check.sh pr               # what the pull request check runs (< 10 min)
tools/check.sh full             # adds TSan, Release, the testbed and the nightly simulation (< 20 min)
cmake --preset linux-clang-20-asan && cmake --build --preset linux-clang-20-asan   # one CI cell
ctest --preset linux-clang-20-asan -L joints   # one suite, by label
out/linux-clang-20-asan/test_golden | grep -E '^M2_[A-Z0-9_]+_HASH=' > test/hashes.txt   # regenerate golden values
```

`tools/check.sh pr` needs `markdown` for the site (`pip install markdown==3.6`). Run the `pr` and `full` tiers through the workspace's heavy-job wrapper on the shared machine. Every CI cell is a preset in `CMakePresets.json`; add a cell there, never as commands in a workflow. Each suite is `test/test_<label>.c`, labeled with its name.

## Ways to hurt yourself

- **Killing by pattern.** Never `pkill -f`, `pgrep | kill` or `kill` a PID found by matching a name or path: other sessions build and test on the same machine with the same binary names. Kill only a PID you started.
- **The machine's GCC.** The default `gcc` is 13, which the build refuses. Use `gcc-14` or `clang-20`, as the presets do.
- **ThreadSanitizer in the CI image.** It does not start inside the container on the shared machine; run the TSan preset on the host.
- **Regenerating golden values to make a test pass.** A moved line is a determinism bug until a pull request says which change moved it and why, and the scalar and AVX2 builds agree on the new value. From 1.0.0 such a change is titled `feat`, since a patch release never changes results (`docs/operations/release.md`).
- **Editing shared family files.** These are synced from the canonical copies in the organization's `.github` repository (`maul/family_sync.py`) and fail CI when they differ: `docs/internals/maul-conventions.md`; `.clang-format`, `.clang-tidy`, `.editorconfig`, `.gitattributes`, `.gitignore`; `cmake/CompilerFlags.cmake`, `cmake/Config.cmake.in`, `cmake/Install.cmake`, `cmake/Library.cmake`, `cmake/pkg-config.pc.in`, `cmake/ReadVersion.cmake`, `cmake/Testing.cmake`, `cmake/HeaderCheck/CMakeLists.txt`; `bench/CMakeLists.txt`; `src/cpu.c`, `src/cpu.h`, `src/fpenv.c`, `src/fpenv.h`, `src/hash.c`, `src/simd.h`; `test/test_fenv.h`, `test/test_fpenv.c`, `test/test_harness.h`; `tools/test_hygiene/`; and `tools/build_docs.py`, `check_docs.py`, `check_exports.py`, `check_float_env.py`, `check_guide.py`, `check_lengths.py`, `check_modules.py`, `check_source.py`, `check_test_hygiene.py`, `gen_api.py`. Change the canonical copy first.

## Invariants

- **Floating point:** no fast math and no contraction (`cmake/CompilerFlags.cmake`); fused multiply-add only through `src/simd.h`. `src/` calls only the `<math.h>` functions in `tools/libm-allowed.txt`, all correctly rounded; trigonometry is the engine's own. Min, max and abs go through `m2MinF`, `m2MaxF`, `m2AbsF`.
- **Order:** every pass walks slots in ascending order, pairs stay sorted by key, ties break to the lower index, freed slots are reused in FIFO order. Parallel work writes only per-item slots, so the split never changes results.
- **State:** every array a world owns is listed in `src/world_state.c`, which allocates, snapshots, checks, restores and hashes it. A new state array goes there, with its check and, when the step reads it, `M2_STATE_HASH`, or rollback or the hash silently breaks.
- **Formats:** a change to the snapshot's bytes bumps `M2_SNAPSHOT_VERSION`; a change to a tape's bytes bumps `M2_JOURNAL_VERSION`. The golden `ROLLBACK_SNAPSHOT` and `JOURNAL_TAPE` lines show both. Every public writer records itself in the journal after its checks pass, and every record replays through `src/journal_replay.c`.
- **Ids and misuse:** a stale id counts one misuse, except a particle's; untrusted bytes and full pools count none. Writers check `m2CheckWriter` first, so host code the world runs cannot change it. `docs/internals/overview.md` has the full contract.
- **Version:** `M2_VERSION_*` between the release-please markers in `include/maul2d/base.h`; CMake and `M2_ABI_VERSION` read it. Only release-please changes it.
- **Public headers** are the common subset of C17 and C++17 and declare only the API; every public function has a `///` block with `@param`, `@return` and a thread safety paragraph that `tools/check_docs.py` accepts. Writers say "a world is used by one thread at a time", readers add "or by several readers at once".
- **Modules:** `tools/modules.txt` lists which module may include which. A new include needs a line there.
- **Lengths and complexity:** a function past 80 lines or the clang-tidy complexity threshold is split, or listed in `tools/length-exceptions.txt` with a `NOLINTNEXTLINE` giving the same reason.
- **Assertions:** `M2_ASSERT` (`src/core.h`) checks internal invariants only, never caller input; it traps without `NDEBUG` and compiles out of Release.
- **Workflows:** every action is pinned by commit SHA with its version in a comment, the CI image by digest, and each job declares the least `permissions` it needs.

## Testing

Follow the [repository standard's Tests section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#tests); in this repository:

- Test through the public headers. `test/helpers.h` wraps status calls for scene setup; a test about why a call refuses calls the public function and checks its status and the misuse count.
- A promised output is pinned by a golden scene in `test/test_golden.c`, not by a new hash in a suite. Prefer extending a scene to adding one.
- Determinism is tested with twins: the same calls in two worlds (serial and on the task hooks, recorded and replayed, rolled back and not) must reach the same `m2World_Hash`.
- Code the task hooks run is covered by a `THREADS` suite; mark a new suite `THREADS` when it steps with workers.
- Every C snippet of `docs/user/` appears unchanged in `test/test_guide.c`; `tools/check_guide.py` enforces it.
- A bug found by the walk or the soak becomes a test with the seed or the minimal scene.

## Documentation

Follow the [repository standard's Documentation section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#documentation). A change in what a caller must do or can rely on updates the header's `///` block and the page of `docs/user/` for that part, and the matching code in `test/test_guide.c` when it is in a snippet. The API reference is generated from the headers into the documentation site; never commit it.

## Pull requests

Titles are conventional commits. Scopes: `api`, `world`, `collision`, `solver`, `joints`, `fluid`, `query`, `geometry`, `journal`, `snapshot`, `simd`, `testbed`, `samples`, `build`, `bench`, `family`. Example: `fix(joints): keep the ratchet's teeth across a rollback`.

A change of results names the golden lines it moves and why. A change to the snapshot or tape bytes names the format version it bumps.
