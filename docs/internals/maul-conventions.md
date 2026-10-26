# Maul conventions

The rules every Maul library follows and the reasons behind them. This page is identical in every Maul repository: its canonical copy lives in the organization's `.github` repository, a sync tool writes it here, and CI fails when the copies differ. Change the canonical copy, never this one.

Rules that a tool enforces are not repeated here. The tool's header comment and its error message state the rule and why it exists; [Checked by tools](#checked-by-tools) lists them. What follows is what only review can check.

What depends on a library's domain (its determinism class, the threads it may start, its owner objects, its platform dependencies) is stated in that library's `docs/internals/overview.md`. A library may add to these rules there, never loosen them.

In the rules, `P` is the library's API prefix (`m2`, `muni`, `mwin`), `PP` its macro prefix (`M2`, `MUNI`) and `lib` its name (`maul2d`, `maul-unicode`).

## Contents

- [One family](#one-family)
- [Language and headers](#language-and-headers)
- [Errors and assertions](#errors-and-assertions)
- [Memory](#memory)
- [Threads](#threads)
- [API shape](#api-shape)
- [Floating point and determinism](#floating-point-and-determinism)
- [Structure](#structure)
- [Tests](#tests)
- [Performance](#performance)
- [Compatibility](#compatibility)
- [Outside code and data](#outside-code-and-data)
- [Checked by tools](#checked-by-tools)

## One family

The libraries are one product family: a program may link several of them, and a developer who knows one should be able to read another. Where two libraries differ for no reason of their domain, the difference is a bug in one of them.

- The same concept has the same name in every library. When a library gains a function another already has, it takes the existing name, even where its own naming pattern would suggest another.
- Every symbol carries the library's prefix, `m` plus a short name of at most four letters, so all libraries link into one program. Two-letter prefixes are crowded (`ma_` is miniaudio's, `mu_` microui's). Windows' `winnls.h` defines `MUI_*` macros, so public headers are also compiled together with `windows.h` there.
- Shared code lives in each repository as a copy, not in a shared base library or a submodule: each library builds, packages and ships alone. The sync tool keeps the copies identical; files that differ only by prefix are written from one template.

## Language and headers

- Library code is C23: checked integer arithmetic, `nullptr`, `constexpr` objects and fixed-width enumerations make it safer and clearer. It uses only what the compiler provides; `<stdbit.h>` comes from the C library, is missing on some targets, and is not used. Bit operations go through compiler builtins.
- Public headers are written in the common subset of C17 and C++17, which is also valid C23, because every consumer's compiler reads them. Attributes only newer dialects accept sit behind library macros (`PP_NODISCARD`). An enumeration whose size matters in a public struct is a `typedef` of a fixed-width integer with named constants.
- MSVC keeps `__cplusplus` at 199711L unless `/Zc:__cplusplus` is given, so the nodiscard macro also reads `_MSVC_LANG`.
- Minimum compilers are GCC 14 and Clang 19. On Windows the library is built with `clang-cl`, which Visual Studio installs and which produces compatible objects; MSVC's `cl` compiles the headers but not the sources.

## Errors and assertions

- A function that can fail returns a status from the library's closed result enum, success being zero, and `PP_NODISCARD` comes first in its declaration because C++ accepts a standard attribute only there. What it produces goes through out-parameters. Where the position of a failure matters (validation, decoding, parsing) it returns a small struct with the status and the byte offset. `PResultName` names any status.
- No library keeps a failure reason in thread-local or global state. A reason kept apart from the call can be stale, read on the wrong thread, or overwritten by another thread's failure.
- An argument a correct program never passes, given to a live owner object (a world, a device, a context), counts one misuse on that object, readable at any time, so a host can catch bugs in release builds. Bad untrusted data (a file, a snapshot, a tape, a packet) is refused with its status and counts no misuse: the counter shows bugs in the calling code, and whoever controls the data must not be able to move it. A validity query on a stale id is not misuse.
- A call whose only refusal is misuse (an argument a correct program never passes: a setter, an impulse, an edit) may return `void`: it changes nothing and counts the misuse. Anything a correct program can meet (a full pool, the wrong state, bad untrusted input, a legitimate refusal) returns a status.
- Whether a stale id is misuse is part of a library's contract, stated in its overview. Where a correct program never uses an id after destroying its object (a physics body), a stale id counts one misuse and a getter returns the zeroed value. Where ids outlive their objects by design (a window whose records are still being read), a stale id counts nothing and the call returns the stale status.
- Every out-parameter documents what a failure leaves in it. A call whose first parameter is an owner object clears its outputs on any failure, before it checks the owner, unless the failure itself defines the output (the length a too-small buffer needed). A creation sets the handle or id it gives to NULL or the null id on any failure, whether or not it takes an owner, so a failed creation never leaves a value the caller could destroy. Any other call without an owner leaves its outputs as they were, so a failed remake keeps the caller's last good value. An output never aliases an input.
- Reserved fields of public structs are zero. A call that takes such a struct refuses nonzero reserved bytes where it takes the struct in, not on hot sampling paths, and every struct a library writes has them zero.
- Running out of memory is a returned status. Every allocation is checked, and byte sizes derived from counts use checked arithmetic and are refused when they do not fit.
- Untrusted input (files, network data, platform payloads, API arguments) is validated and refused with a status in every build. It is never handled by an assertion: a program may legitimately produce bad input, and the library must not stop it.
- `PP_ASSERT` checks internal invariants: states that cannot occur unless the library itself is wrong, both what is expected and what is excluded. It is on in debug, sanitizer and fuzz builds, where it stops the run at the fault, and may compile out of release builds. Review checks that new code asserts what it assumes. Library code never calls `assert` or `abort`.

## Memory

- No library has a process-wide allocator hook: it would have to be set before first use, never change, and give a whole program one allocator per library. Every owner object takes an allocator in its def and keeps it for its lifetime: `alloc(size, alignment, context)`, `free(memory, size, alignment, context)` and a context, alignment a power of two, no realloc. A zeroed allocator means the C library's.
- A function without an owner object does not allocate. It works in caller memory, a buffer and its capacity, and reports the size it needs when the buffer is too small.
- Hot paths never allocate.

## Threads

- A library starts no thread unless a platform leaves no other way to meet a requirement. Such a thread is declared in the library's overview with its purpose, started by the object that needs it only while needed, joined when that object stops and never later than the root's destroy, and never runs application code.
- On a thread the platform owns (an audio callback, a platform callback), a library only publishes into bounded queues or reads what was published: no allocation, lock or wait, and no application code except a callback the overview declares real-time.
- Parallel work runs on the host's task system through the library's task hooks. Scheduling never changes results.
- Host code a library calls (the allocator, task hooks, callbacks) calls no function of the object it serves unless the library's overview allows it. The object's mutating functions refuse such a call with the state error, one misuse counted. A callback that must call back in is given a named list of allowed calls, kept free of the state the outer call is changing.

## API shape

- Every creation takes a def struct initialized by `P...Default...Def`. Defs carry a cookie, so an uninitialized def is refused instead of read.
- A root def (the def of a creation that takes no other library object, so a program can call it first) also carries `PP_ABI_VERSION`, the version of the headers the program was built with. A library may have several roots, and each one carries the stamp. Its default is built in the program, a `static inline` function or a macro, so it stamps the program's version rather than the library's. A root creation checks the cookie first and the version second, and refuses a mismatch with `P_errorVersion` before it reads anything else; every other def is used under an object a root made, so it carries only its cookie. The number names the releases that share a binary interface: the whole version before 1.0, one major and minor version from 1.0 on. Its top bit is always set, so a zeroed or hand-written value never matches, and `base.h` rejects at compile time a version the encoding cannot hold. Stored data uses its own format number, never this one. A library without a root def has no stamp.
- A function that fills a caller array takes the array and its capacity and reports the true total, which may exceed the capacity. A NULL array with capacity 0 measures.
- Getters that cannot fail take ids and return values. Out-parameters end in `Out`.
- Strings cross the API as UTF-8 with an explicit byte length, never as NUL-terminated text alone, so text from hostile sources is never measured by searching for a terminator.
- A library's root object is a typed opaque pointer. Every object a root owns is named by a typed value id, a 1-based slot index (0 is null) and a generation, one struct per kind. A stale id is refused with a typed result, never dereferenced, which costs one array read and a compare. Platform handles are never identities.
- An operation that finishes later is a request: it returns its status at once and, accepted, a typed request id that exactly one completion record answers, in the owner's notification queue, with a typed outcome. Callbacks that deliver completions fire at unsafe points and leave re-entrancy rules to every caller; a queue does not.

## Floating point and determinism

- Each library's overview states its determinism class: bit-exact on every platform, reproducible on one platform and build, or exempt, and which outputs it covers.
- No fast math, and no implicit floating-point contraction: both change results between compilers. Fused multiply-add appears only where code spells it through the family's SIMD header, the same on every backend.
- A library that promises the same bits everywhere calls only exactly rounded `<math.h>` functions, listed in `tools/libm-allowed.txt`, and writes its own where it needs another.
- Such a library also computes in its own floating-point environment, whatever the host's flush-to-zero, denormals-are-zero or rounding mode. Every public function that compares or computes floats is a wrapper: it enters with `PFloatEnvEnter()`, calls one `PP_NOINLINE` worker with its arguments unchanged (names, their addresses, pointer casts or constants), and leaves with `PFloatEnvLeave()`. Task bodies on host threads run through `PFloatEnvRun`. Host code that computes nothing the library promises, such as an allocator, task hooks or a draw callback, runs between `PFloatHostEnter` and `PFloatHostLeave`, in the host's own setting. A public function that does no float work is named, under its reason, in `tools/float-env-allowed.txt`.

## Structure

- A module is one `.c` file and one internal header of the same name in `src/`; the header declares exactly what other files may call. The module graph in `tools/modules.txt` keeps dependencies one-way.
- A struct holds one concept. An owner struct composes subsystem structs instead of collecting loose fields.
- No global mutable state. Constant tables are immutable. The one exception is `src/fpenv.c`, the family's floating-point environment module: it keeps the thread's saved control register and entry depth in thread-local storage, mirroring the processor's own per-thread register.
- A part built as a library of its own (a renderer on another Maul library) lives beside `src/` in `DIR/src` and `DIR/include`, listed in `tools/source-dirs.txt`, and includes its own and public headers only, so it stays as separable as its build target.
- Generated sources live in `src/generated/`, written by a generator in `tools/`, and are never edited by hand. Each names its generator and inputs. Where CI can regenerate them, it does and fails on any difference.
- Comments say why. No history, no task or record numbers, no `TODO`: history lives in git and work in issues.

## Tests

- Tests use the family's harness header and register with CTest through `maul_add_test`. A suite that reaches internal headers is marked `WHITEBOX` and is skipped in shared builds, where internal symbols are hidden.
- CI runs the tests under AddressSanitizer, UndefinedBehaviorSanitizer and ThreadSanitizer, with warnings as errors.
- Every parser of external bytes treats its input as hostile and has a fuzz target.
- A fuzz run publishes no input. CI runs each target through `tools/fuzz_target.sh`, which keeps a finding, its seed and the target's log on the runner and prints only the finding's kind and sanitizer report. No workflow uploads a finding, a fuzz log or a corpus as an artifact; a corpus is kept in the Actions cache. The library's own pull_request runs do not restore it and start from the committed seeds, and a run that found something does not save it, since its corpus holds inputs near the finding. The cache is not secret: a fork's pull request can run its own workflow and restore it, so it holds nothing the committed seeds would not lead to. A harness's own failure message names the broken promise, never the input's bytes.
- A failing test is never rerun to pass. When a test fails without a fault in the change under test, it is quarantined: an issue records the failure and its run, the test (or the one check that failed, split into a test of its own) takes the `quarantine` label, the required runs exclude that label, and a nightly run executes every quarantined test, counting a skip as a failure. A test leaves quarantine when its issue's fix lands and the nightly has passed it 14 nights in a row.
- Every C snippet of `docs/user/` is built and run by a test as written, so documentation cannot drift from the API. A fragment no test can build everywhere is marked `<!-- guide: not run: REASON -->` right above its fence.

## Performance

- Each library has plain C benchmarks in `bench/` with fixed inputs and no dependencies, which print their timings. Timings are never a CI gate: wall time is not repeatable on shared runners.
- A change meant to affect performance is measured before and after on the same machine, and the pull request gives the numbers.
- CI reports the library's size, in total and per optional feature.
- Hot paths take no locks, make no system calls and call nothing locale-dependent.

## Compatibility

- Versions follow Semantic Versioning, and the version lives in one place, the `PP_VERSION_*` macros of the base header, which CMake reads.
- Before 1.0.0 any release may change the API, the ABI and every data format. A data format's version is bumped whenever its bytes change.
- A library reaches 1.0.0 only once its API is settled: its public headers have been reviewed one by one for shape, every breaking change they still need has landed in 0.x, its asset formats are extensible (below), and every Maul library its public headers name has reached 1.0.0 first. The family plans no 2.0.0: from 1.0.0 a change that would break 1.x is designed to be additive or does not ship.
- From 1.0.0, within a major version: a program that builds against 1.y builds against any later 1.z and means the same (functions, types, enum values and fields are added, never removed or renamed; a field is added only at the end of its struct; deprecation names the replacement); a 1.z library reads everything an earlier 1.y wrote; results that change between versions are stated in the changelog.
- Data a library writes is of three kinds, and each has its own rule.
  - An asset format holds what a host builds once and ships: graphs, clips, tiles, bakes, effect files, shader containers, message catalogs. It is extensible. After its header, its content is made of fields, each named by a 32-bit tag and given a byte length, little-endian like every family format: either a sequence of fields, each carrying its tag and length, or a table of sections giving each one's tag, offset and length. Each payload starts at a multiple of the format's alignment (8 or 16 bytes, stated in its internals page) and is padded with zero bytes to the next, so a field may be a whole section the runtime reads in place. A reader takes each field it knows and checks it as it checks any input. It gives a field that is absent its documented default, and it refuses a tag it does not know with `P_errorVersion`, since only a later version writes one. A writer leaves out a field that holds its default, so a file that uses no later field is read by an earlier reader too. A field whose meaning changes gets a new tag. From 1.0.0 the format's version no longer changes, and a 1.z reader reads every tag a 1.y wrote, with one reader. Before 1.0.0 the version still changes whenever the bytes do, and a reader takes only the current one.
  - A state format holds what a running program saves: snapshots, journals, replay tapes, saved instances. The library's overview states whether each of its state formats is read across 1.y. One that is not records its writer's `PP_ABI_VERSION` (the whole version before 1.0.0, the major and minor version from 1.0.0) and is read only by a release of that ABI version, which refuses another with `P_errorVersion`: before 1.0.0 only the release that wrote it reads it, and from 1.0.0 a patch release keeps it while a minor release does not. A host keeps assets, not state, across an upgrade that changes the ABI version.
  - Data a library generates from its own pinned sources together with code compiled into it, such as maul-locale's CLDR data files, belongs to the release that generated it, as state does. The library opens only data of its own data version, refuses other data with its own error, and ships the data its release needs; the overview says so.
  - A library converts its asset formats to fields in 0.x, as a breaking change, before its 1.0.0.
- Before 1.0.0 no two releases share an ABI or an API: a shared library's `SOVERSION` is its whole version and the CMake package's version check is ExactVersion. From 1.0.0 the ABI holds within a minor version (a minor release may grow a def) and the API within a major one: `SOVERSION` is major.minor and the check is SameMajorVersion.

## Outside code and data

- No code is copied or adapted from other projects. Algorithms from the literature are implemented from their published descriptions, cited where the library's internals pages explain them.
- A library takes no dependencies beyond the platform's own APIs and the system libraries its overview names. Tests, benchmarks and test-only harnesses may use outside libraries as oracles; they are never linked into the library.
- Data from outside (the Unicode Character Database, a gamepad mapping database) is kept exactly as published in a directory listed in `tools/external-dirs.txt`, with a `README.md` naming the source, version, license and SHA-256 of every file. The source checks skip it.

## Checked by tools

| Rule | Tool |
|---|---|
| Formatting, include order, braces, `const` placement | `clang-format` at the version CI pins |
| Naming of locals, parameters and fields; cognitive complexity | `clang-tidy` |
| Public API C17, C23 and C++17 clean; MSVC header build | header tests (`cmake/Testing.cmake`, `cmake/HeaderCheck/`) |
| SPDX lines, comment style, history markers, banned calls, mutable file-scope state, `<math.h>` allow list, file names, shell scripts through shellcheck | `tools/check_source.py` |
| Module graph | `tools/check_modules.py` |
| Every public function enters the floating-point environment through a wrapper of one shape, or is allowed by name with a reason | `tools/check_float_env.py` |
| A shared build exports the public functions only | `tools/check_exports.py`, registered by `maul_add_exports_test()` in `cmake/Testing.cmake`, which names the headers a build leaves out and the functions the program defines |
| File and function lengths, complexity exceptions | `tools/check_lengths.py` |
| Public documentation blocks and thread safety statements, in the library's headers and each part's | `tools/check_docs.py` |
| Documentation snippets run by tests | `tools/check_guide.py` |
| No sleeps, wall-clock reads, retries or unexplained skips in tests; every test labeled | `tools/check_test_hygiene.py`, with its self-test over `tools/test_hygiene/` |
| Fast math refused | `cmake/CompilerFlags.cmake` |
| Allowed documentation files, em dash, record numbers, links | the organization's documentation check |
| Commit and pull request titles | the organization's title check |
