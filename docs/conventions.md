# Maul conventions

These rules govern Maul2D and Maul3D alike. The two engines are one
product family: this file is identical in both repositories and
changes to it land in both together. New code follows every rule.
Where older code does not yet, the gap is a bug to fix, not a
precedent to follow.

In the rules below, `P` stands for the engine prefix (`m2` or `m3`),
`PP` for its macro prefix (`M2` or `M3`) and `lib` for the library
name (`maul2d` or `maul3d`).

## 1. Language

- Everything in the repository is written in English: code,
  comments, documentation, commit messages, issues and pull requests.
- Spelling is American English (`behavior`, `normalize`, `color`).
- Write plainly. Say what a thing does and why. No slogans, no
  metaphors that need decoding, no editorial asides about other
  projects.

## 2. Repository layout

Both repositories have the same top level:

| Path | Contents |
|---|---|
| `include/lib/` | Public headers, the only API. |
| `src/` | Implementation and internal headers. |
| `test/` | Test suites, one executable per suite. |
| `bench/` | Benchmarks and their pinned results. |
| `samples/` | Small standalone programs that use the installed package. |
| `testbed/` | The interactive viewer. Not part of the library. |
| `tools/` | Developer scripts and command-line tools. |
| `cmake/` | CMake modules and package templates. |
| `docs/` | Guides, the API reference and design records (`docs/adr/`). |
| `.github/` | CI workflows. |

Root files are `README.md`, `CHANGELOG.md`, `CONTRIBUTING.md`,
`LICENSE`, `CMakeLists.txt` and the tool configuration files
(`.clang-format`, `.clang-tidy`, `.editorconfig`, `.gitattributes`,
`.gitignore`).

## 3. Naming

### C code

| Kind | Form | Example |
|---|---|---|
| Public type | `P` + PascalCase | `m2BodyDef`, `m3ShapeId` |
| Public function on an object | `P` + Object `_` + Verb[Noun] | `m2Body_GetPosition` |
| Public lifetime function | `P` + `Create`/`Destroy` + Object | `m3CreateWorld` |
| Public free function | `P` + Verb[Noun] | `m2MakeBox`, `m3Hash64` |
| Def initializer | `P` + `Default` + Type | `m2DefaultBodyDef` |
| Enum type | `P` + PascalCase | `m2BodyType` |
| Enum value | `P` + `_` + camelCase | `m2_dynamicBody` |
| Macro, constant | `PP_` + UPPER_SNAKE | `M2_MAX_WORLDS` |
| Internal function shared across files | `P` + PascalCase, no `_` | `m2SolveContacts`, `m3TreeInsert` |
| `static` function | PascalCase | `UpdatePairs` |
| Local variable, parameter, struct field | camelCase | `bodyIndex`, `pairCount` |
| File-scope `static` variable | `s_` + camelCase | `s_worlds` |

- Verbs are consistent across both engines: `Get`, `Set`, `Is`,
  `Has`, `Enable`, `Disable`, `Create`, `Destroy`, `Apply`, `Cast`,
  `Overlap`, `Collide`.
- Acronyms are written as words: `Aabb`, `Gjk`, `Id`, `Ccd`.
- The same concept has the same name in both engines. When one engine
  gains a function the other already has, it takes the existing name.

### Files and directories

- C sources and headers: `snake_case.c`, `snake_case.h`.
- Test suites: `test/test_<area>.c`; test functions `Test<Behavior>`.
- Markdown in `docs/`: `kebab-case.md`. Design records:
  `docs/adr/NNNN-kebab-case-title.md`.
- Directories: lowercase, no separators where one word suffices.
- Git branches: `kebab-case`, prefixed with the commit area when
  useful (`solver-joint-vtable`).

### CMake

- Options: `MAUL2D_...` / `MAUL3D_...`, upper snake case, named for
  what they switch (`MAUL3D_BUILD_TESTS`, `MAUL3D_SANITIZE`).
- Functions and macros shared by the family: `maul_snake_case`.
  Engine-specific ones: `maul2d_snake_case` / `maul3d_snake_case`.
- Targets: the library is `lib`, exported as `lib::lib`; tests are
  `test_<area>`; tools are `P<tool>` (`m3replay`).

## 4. Formatting

- `clang-format` with the checked-in `.clang-format`, at the version
  CI pins. Run it; do not argue with it. CI rejects unformatted code.
- 4 spaces, no tabs, 100 columns, Allman braces, braces on every
  `if`, `else`, `for`, `while` and `do`.
- One declaration per line. Declare variables at first use.
- Files end with one newline, use LF line endings and UTF-8 without a
  byte order mark (`.editorconfig`, `.gitattributes`).

## 5. Files and modules

- A module is one `.c` file with one internal header of the same name
  in `src/`. The header declares exactly what other files may call.
- Every `.c` file includes its own header first, then other internal
  headers, then public headers (`"lib/x.h"`), then system headers
  (`<...>`), one blank line between groups, each group sorted.
- Every function with external linkage is declared in a header;
  `-Wmissing-prototypes` is on.
- Header guards are `LIB_PATH_NAME_H` (`MAUL2D_DYNAMIC_TREE_H`).
- Public headers declare only the API. Nothing internal appears in
  `include/`, not even "for internal use" declarations.
- A source file stays under 1000 lines and a function under 80 lines.
  A longer function carries a one-line comment at its top saying why
  it is kept whole. CI measures both.
- No global mutable state except the documented process-wide hooks
  (allocator, assert handler), which are set before the first world
  exists, and the thread-local error slot.

## 6. Comments and documentation in code

- Comments explain why, and what is not obvious from the code. They
  do not narrate the code line by line.
- Full sentences, capitalized, ending with a period. `//` comments
  only; public API documentation uses `///` directly above the
  declaration.
- Every public function documents its contract: what it does, what it
  refuses, and its thread class (`Thread class: reader` or `writer`).
- No development history in the source: no ticket or task numbers,
  review round names, phase or slice labels, dates, version stamps,
  "was/used to" stories or changelogs. History lives in git and in
  `CHANGELOG.md`; design reasoning lives in `docs/adr/`.
- No `TODO` or `FIXME` comments on `main`. Open an issue instead.
- No comparisons with other engines in code comments.

## 7. Errors and assertions

- A function that rejects its input refuses: it returns its null
  result (a null id, `false`, `0` or an empty result) and records the
  reason through the engine's refusal path, which sets the
  thread-local last result and, for an invalid argument against a live
  world, counts one misuse. Refusals never assert and never abort.
- The result codes are the same in both engines: success, invalid
  input, capacity (a full pool, slot table or failed allocation) and
  configuration (the CPU cannot run the compiled backend).
- `PP_ASSERT` is for internal invariants only: states that cannot
  happen unless the engine itself is wrong. Library code never calls
  `abort` directly.
- Running out of memory is a refusal. Every allocation is checked.
  Byte sizes derived from counts are computed in 64 bits and refused
  when they do not fit.
- A validity query (`P..._IsValid`) on a stale id is not misuse and
  is not counted.

## 8. Determinism

- Same build, same inputs, same bits: on every supported platform,
  compiler, architecture and worker count.
- No fast math and no floating-point contraction; the compiler flags
  in `cmake/` enforce this and refuse to configure otherwise.
- Only `sqrt` and the operations IEEE 754 rounds exactly come from the
  platform. Transcendentals are the engine's own.
- Every loop whose order can reach state runs in a canonical order,
  normally ascending slot index. Results handed to the caller are in
  ascending slot order unless the function documents another order.
- Snapshot and hash data contain no padding bytes, no pointers and no
  uninitialized memory. Derived state is rebuilt after a restore, not
  stored.
- The hash lines printed by the tests (`PP_<AREA>_HASH=<16 hex>`) and
  the benchmark pins are compared across every CI cell and against
  the values committed in the tests. A change that moves one says so
  in its commit message and in `CHANGELOG.md`, with the reason.

## 9. Memory

- A world allocates everything it needs when it is created. Stepping
  never allocates.
- All engine memory goes through the allocator hooks.
- Three lifetimes are kept apart: persistent world state (snapshotted),
  per-step scratch (never snapshotted) and immutable shared data.

## 10. Threading

- Every public function is either a reader (may run concurrently with
  other readers on the same world) or a writer (exclusive access to
  its world). Readers never write shared state; if one must count
  something, it uses an atomic.
- Parallel work runs on the host's task system through the task hooks.
  The engine starts no threads of its own. Scheduling never changes
  results.

## 11. API design

- Objects are addressed by value ids carrying a slot, a generation and
  their world. A stale id is detected, never followed.
- Every creation takes a def struct initialized by `P..Default..Def`.
  Defs carry a cookie so that an uninitialized def is refused.
- Functions that fill a caller array take the array and its capacity
  and return the true total, which may exceed the capacity. A NULL
  array with capacity 0 counts.
- Getters take ids and return values; they do not take output
  pointers unless they return more than one value.

## 12. Tests

- Tests use the shared harness header in `test/` and register with
  CTest through the helper in `CMakeLists.txt`.
- Each test function checks one behavior and is named for it.
- Every fixed bug comes with a test that fails without the fix.
- Tests that need internals include internal headers and are marked
  white-box in `CMakeLists.txt`; all others use the public API only.
- Long-running soaks and timing runs stay out of the default CTest set
  and run in scheduled CI.

## 13. Versioning and the changelog

- Semantic versioning. The version lives in one place, the
  `PP_VERSION_*` macros in `include/lib/base.h`; CMake reads it from
  there.
- Before 1.0.0 any minor release may change the API, the ABI and the
  snapshot and journal formats. The snapshot and journal format
  versions are bumped whenever their bytes change.
- `CHANGELOG.md` follows Keep a Changelog. Every user-visible change
  adds a line under `[Unreleased]` in the same commit, in the section
  order Added, Changed, Deprecated, Removed, Fixed, Security.
- Releases are tags `vX.Y.Z` on `main`.

## 14. Commits

- Subject: `area: imperative summary`, lowercase area, at most 72
  characters, no trailing period. Example:
  `journal: make replay atomic and refuse ids that do not match`.
- Areas: `api`, `allocator`, `bench`, `bindings`, `broadphase`,
  `build`, `ccd`, `character`, `ci`, `collision`, `core`, `docs`,
  `draw`, `errors`, `island`, `joint`, `journal`, `math`, `particle`,
  `query`, `replay`, `samples`, `shape`, `simd`, `snapshot`,
  `softbody`, `solver`, `testbed`, `tests`, `tools`, `vehicle`,
  `voxel`, `world`. A commit that spans areas takes the one that
  matters most.
- Body: wrapped at 72 columns, says what was wrong or missing and why
  this change is right. Mention moved hashes and how they were
  checked.
- No trailers (`Signed-off-by`, `Co-authored-by`, generated-by lines).
- One logical change per commit. Every commit builds, passes the tests
  and is formatted.

## 15. Markdown

- One `#` title per file, sentence case headings.
- Prose wrapped at 72 columns. Tables and code blocks are exempt.
- Code blocks name their language (`c`, `cmake`, `sh`).
- Links between documents are relative.

## 16. Outside code

- No code is copied or adapted from other projects. Algorithms from
  the literature are implemented from their published descriptions,
  and `docs/references.md` lists those sources.
- Code adapted from other projects in the past keeps its origin note
  and its license in `THIRD_PARTY.md` until it has been replaced.
- The testbed may use outside libraries (it is not part of the
  engine); they are listed in `README.md` under the testbed.
