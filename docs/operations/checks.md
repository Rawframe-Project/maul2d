# Checks

What verifies a change, locally and in CI, and the time each may take.

## Locally

`tools/check.sh` runs the checks in three tiers; each includes the one before it. Its builds are the CI cells' presets from `CMakePresets.json`.

| Tier | Runs | Budget |
|---|---|---|
| `changed` | The source, module, length, API documentation and snippet rules, test hygiene, the organization's documentation check, `clang-format`, and the `linux-clang-20-asan` preset (scalar backend, assertions and world validation on) with the CTest labels the changed files reach. Every preset it configures must label each of its tests. | under 2 minutes |
| `pr` | Every rule on every file, clang-tidy over `src/` with the assertions compiled out and live, the documentation site, and the `linux-clang-20-asan`, `linux-gcc-14` and `linux-shared` presets. | under 10 minutes |
| `full` | Also the other Linux presets (ThreadSanitizer, Clang Release), the testbed, the nightly simulation, and each fuzz target for `FUZZ_SECONDS` (default 20). | under 20 minutes |

Run `changed` before every push. On the shared development machine, run `pr` and `full` through the workspace's heavy-job wrapper. Builds go to `out/<preset>`, each with its log beside it.

## In CI

`.github/workflows/ci.yml` runs on every pull request, on every push to `main` and every night. The Linux jobs run in the organization's CI image, pinned by digest. The ruleset requires only the `required` job, which passes when every other job passed or was skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether a pull request changes anything beyond documentation; when it does not, the build jobs are skipped. |
| `docs` | The organization's documentation check, run by its `docs-check` action from the pinned commit of `.github`, so a rule reaches this repository when the pin moves: allowed files, the em dash, record numbers, page sizes, links. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`, the family's source rules (with the `<math.h>` allow list), the module graph, lengths and complexity exceptions, API documentation, documentation snippets, test hygiene, that the documentation site builds, and clang-tidy twice: on the Release preset, where the assertions compile out and a value only an assertion reads is a dead store, and with them live, where their branches count toward a function's complexity. It runs on documentation changes too. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 and Clang 20 on x64, 32-bit x86 (SSE2 arithmetic) and arm64 Linux, Clang with AddressSanitizer and UBSan on the scalar backend (each test repeated up to three times on a pull request, so a flaky one fails before it merges), Clang with ThreadSanitizer, the shared library with the package test, Clang on arm64 macOS, and `clang-cl` on x64 and arm64 Windows. All build with warnings as errors, and every cell checks the golden values. |
| `msvc-headers` | Every public header compiled by MSVC's `cl` as C17 and C++17. |
| `testbed` | The native testbed builds. |
| `web` | The library and its tests under Emscripten and Node, the golden test included, and the size of the static library. |
| `fuzz` | One job per fuzz target. A pull request fuzzes each for 20 seconds from the inputs the targets build; `main` fuzzes for 90 seconds and the nightly or a dispatch for 600 (`fuzz_walk` for 3600), each from the target's cached corpus. On `main` and at night each corpus is then minimized and, when the run found nothing, saved. A finding prints a report without its input, from `tools/fuzz_target.sh`, and nothing is uploaded. |
| `simulation` | At night only: the quarantined tests, a long soak and a deep API walk. |

`pr-title.yml` checks that the pull request title is a conventional commit with a scope from `AGENTS.md`, and `pr-gate.yml` closes a pull request from outside the organization unless it closes an issue labeled `accepted`. `codeql.yml` runs CodeQL on pull requests, on `main` and weekly. `scorecard.yml` grades the repository's security practices weekly, and Dependabot proposes updates of the pinned actions. `pages.yml` publishes the documentation site and the browser testbed from `main`.

## Test hygiene

`tools/check_test_hygiene.py`, shared by the family, rejects sleeps, wall-clock reads, retry loops and skips in the test sources and skip properties in the CMake files. A use that must stay carries `// hygiene: <reason>`, in a comment and with a reason of at least three words, on its line or the one before. With `--ctest` it also rejects a test registered without a label: CI checks the default and the shared configurations, and `tools/check.sh` every preset it configures. `--self-test` runs it over the fixtures in `tools/test_hygiene/`.

## Quarantine

A failing test is never rerun to pass. One that fails without a fault in the change under test is quarantined: an issue records the failure and its run, and the test (or the one check that failed, split into a test of its own) is registered with `maul_add_test(<name> QUARANTINE)` or labeled with `maul_quarantine(<name>)`. Every test preset leaves out the `quarantine` label, so the required runs and `tools/check.sh` skip it. The nightly `simulation` job runs it through `tools/run_quarantine.sh out/linux-clang-20`, which gives each test the test presets' 300 seconds: a failure there is a warning and a line in the job summary, while a skipped test, a sanitizer report or a crash fails the night. A test leaves quarantine when its issue's fix lands and 14 nights in a row have passed it. maul2d has no quarantined test today.

## Fuzzing

Every fuzz target (`test/fuzz_<area>.c`) checks promises that must hold for any input, and crashes when one breaks:

- `fuzz_snapshot` restores hostile snapshot bytes, raw or as edits to a valid snapshot of a fuzz scene. A refused snapshot changes nothing and counts no misuse; an accepted one validates, snapshots back to the same bytes and steps.
- `fuzz_journal` replays hostile tapes the same two ways. A refused tape changes nothing and counts no misuse; an accepted one leaves a world that validates and steps.
- `fuzz_geometry` feeds points to `m2MakePolygon`, `m2ComputeHull` and `m2DecomposeOutline`. Every accepted polygon makes a shape, and an accepted outline gives at least one piece, with pieces that never cover more than its area.
- `fuzz_walk` drives the seeded API walk from its input, with every promise of the walk: replay, rollback and the threaded twin. Once the input's bytes run out, the walk draws from a seed hashed from the input, so every input is a distinct full walk.

The library is instrumented as well as the targets, with AddressSanitizer and UBSan stopping at the first fault (`-fno-sanitize-recover=all`). The targets build their valid inputs at startup, so a format bump never leaves seeds stale. Each target's corpus carries from run to run in an Actions cache of its own, `fuzz-corpus-<target>-<run id>`; every run but a pull request's restores the newest, and runs on `main` and at night minimize it with libFuzzer's `-merge=1` into a fresh directory, swap it in only when the merge finished, and save it; a run that found something saves nothing. A pull request starts from the inputs the targets build, and neither restores nor saves a corpus.

## Determinism in CI

Every cell runs the golden scenes against the values compiled in from `test/hashes.txt`, so a result that differs on one platform, compiler or backend fails that cell. The AddressSanitizer cell runs the scalar backend and the x64 cells AVX2; the arm64 cells run NEON. Twins inside the golden scenes compare a four-worker run with a serial one on every cell.

## Budgets

Over a budget, tests are pruned or merged before anything else is added. A pull request's checks finish in about ten minutes, most of it the Windows and macOS cells; the nightly simulation takes longer and runs on `main` only.
