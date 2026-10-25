# Checks

What verifies a change, locally and in CI, and the time each may take.

## Locally

`tools/check.sh` runs the checks in three tiers; each includes the one before it. Its builds are the CI cells' presets from `CMakePresets.json`.

| Tier | Runs | Budget |
|---|---|---|
| `changed` | The source, module, length, API documentation and snippet rules, test hygiene, the organization's documentation check, `clang-format`, and the `linux-clang-20-asan` preset (scalar backend, assertions and world validation on) with the CTest labels the changed files reach. Every preset it configures must label each of its tests. | under 2 minutes |
| `pr` | Every rule on every file, clang-tidy over `src/`, the documentation site, and the `linux-clang-20-asan`, `linux-gcc-14` and `linux-shared` presets. | under 10 minutes |
| `full` | Also the other Linux presets (ThreadSanitizer, Clang Release), the testbed and the nightly simulation. | under 20 minutes |

Run `changed` before every push. On the shared development machine, run `pr` and `full` through the workspace's heavy-job wrapper. Builds go to `out/<preset>`, each with its log beside it.

## In CI

`.github/workflows/ci.yml` runs on every pull request, on every push to `main` and every night. The Linux jobs run in the organization's CI image, pinned by digest. The ruleset requires only the `required` job, which passes when every other job passed or was skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether a pull request changes anything beyond documentation; when it does not, the build jobs are skipped. |
| `docs` | The organization's documentation check, run by its `docs-check` action from the pinned commit of `.github`, so a rule reaches this repository when the pin moves: allowed files, the em dash, record numbers, page sizes, links. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`, the family's source rules (with the `<math.h>` allow list), the module graph, lengths and complexity exceptions, API documentation, documentation snippets, test hygiene, that the documentation site builds, and clang-tidy. It runs on documentation changes too. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 and Clang 20 on x64 and arm64 Linux, Clang with AddressSanitizer and UBSan on the scalar backend (each test repeated up to three times on a pull request, so a flaky one fails before it merges), Clang with ThreadSanitizer, the shared library with the package test, Clang on arm64 macOS, and `clang-cl` on x64 and arm64 Windows. All build with warnings as errors, and every cell checks the golden values. |
| `msvc-headers` | Every public header compiled by MSVC's `cl` as C17 and C++17. |
| `testbed` | The native testbed builds. |
| `web` | The library and its tests under Emscripten and Node, the golden test included, and the size of the static library. |
| `simulation` | At night only: a long soak and a deep API walk. |

`pr-title.yml` checks that the pull request title is a conventional commit with a scope from `AGENTS.md`, and `pr-gate.yml` closes a pull request from outside the organization unless it closes an issue labeled `accepted`. `codeql.yml` runs CodeQL on pull requests, on `main` and weekly. `scorecard.yml` grades the repository's security practices weekly, and Dependabot proposes updates of the pinned actions. `pages.yml` publishes the documentation site and the browser testbed from `main`.

## Test hygiene

`tools/check_test_hygiene.py`, shared by the family, rejects sleeps, wall-clock reads, retry loops and skips in the test sources and skip properties in the CMake files. A use that must stay carries `// hygiene: <reason>`, in a comment and with a reason of at least three words, on its line or the one before. With `--ctest` it also rejects a test registered without a label: CI checks the default and the shared configurations, and `tools/check.sh` every preset it configures. `--self-test` runs it over the fixtures in `tools/test_hygiene/`.

## Determinism in CI

Every cell runs the golden scenes against the values compiled in from `test/hashes.txt`, so a result that differs on one platform, compiler or backend fails that cell. The AddressSanitizer cell runs the scalar backend and the x64 cells AVX2; the arm64 cells run NEON. Twins inside the golden scenes compare a four-worker run with a serial one on every cell.

## Budgets

Over a budget, tests are pruned or merged before anything else is added. A pull request's checks finish in about ten minutes, most of it the Windows and macOS cells; the nightly simulation takes longer and runs on `main` only.
