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
- Invalid input against a live owner object (a world, a device, a context) counts one misuse on that object, readable at any time, so a host can catch bugs in release builds. A validity query on a stale id is not misuse.
- Every out-parameter documents what a failure leaves in it. A call whose first parameter is an owner object clears its outputs on any failure, before it checks the owner, unless the failure itself defines the output (the length a too-small buffer needed). A call without an owner leaves its outputs as they were, so a failed remake keeps the caller's last good value. An output never aliases an input.
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
- The root def (the def of the object a program creates first) also carries `PP_ABI_VERSION`, the version of the headers the program was built with. Its default is built in the program, a `static inline` function or a macro, so it stamps the program's version rather than the library's. The root creation checks the cookie first and the version second, and refuses a mismatch with `P_errorVersion` before it reads anything else; every other def is used under a root that already passed, so it carries only its cookie. The number names the releases that share a binary interface: the whole version before 1.0, one major and minor version from 1.0 on. Its top bit is always set, so a zeroed or hand-written value never matches, and `base.h` rejects at compile time a version the encoding cannot hold. Stored data uses its own format number, never this one. A library without a root def has no stamp.
- A function that fills a caller array takes the array and its capacity and reports the true total, which may exceed the capacity. A NULL array with capacity 0 measures.
- Getters that cannot fail take ids and return values. Out-parameters end in `Out`.
- Strings cross the API as UTF-8 with an explicit byte length, never as NUL-terminated text alone, so text from hostile sources is never measured by searching for a terminator.
- A library's root object is a typed opaque pointer. Every object a root owns is named by a typed value id, a 1-based slot index (0 is null) and a generation, one struct per kind. A stale id is refused with a typed result, never dereferenced, which costs one array read and a compare. Platform handles are never identities.
- An operation that finishes later is a request: it returns its status at once and, accepted, a typed request id that exactly one completion record answers, in the owner's notification queue, with a typed outcome. Callbacks that deliver completions fire at unsafe points and leave re-entrancy rules to every caller; a queue does not.

## Floating point and determinism

- Each library's overview states its determinism class: bit-exact on every platform, reproducible on one platform and build, or exempt, and which outputs it covers.
- No fast math, and no implicit floating-point contraction: both change results between compilers. Fused multiply-add appears only where code spells it through the family's SIMD header, the same on every backend.
- A library that promises the same bits everywhere calls only exactly rounded `<math.h>` functions, listed in `tools/libm-allowed.txt`, and writes its own where it needs another.

## Structure

- A module is one `.c` file and one internal header of the same name in `src/`; the header declares exactly what other files may call. The module graph in `tools/modules.txt` keeps dependencies one-way.
- A struct holds one concept. An owner struct composes subsystem structs instead of collecting loose fields.
- No global mutable state. Constant tables are immutable.
- A part built as a library of its own (a renderer on another Maul library) lives beside `src/` in `DIR/src` and `DIR/include`, listed in `tools/source-dirs.txt`, and includes its own and public headers only, so it stays as separable as its build target.
- Generated sources live in `src/generated/`, written by a generator in `tools/`, and are never edited by hand. Each names its generator and inputs. Where CI can regenerate them, it does and fails on any difference.
- Comments say why. No history, no task or record numbers, no `TODO`: history lives in git and work in issues.

## Tests

- Tests use the family's harness header and register with CTest through `maul_add_test`. A suite that reaches internal headers is marked `WHITEBOX` and is skipped in shared builds, where internal symbols are hidden.
- CI runs the tests under AddressSanitizer, UndefinedBehaviorSanitizer and ThreadSanitizer, with warnings as errors.
- Every parser of external bytes treats its input as hostile and has a fuzz target.
- Every C snippet of `docs/user/` is built and run by a test as written, so documentation cannot drift from the API. A fragment no test can build everywhere is marked `<!-- guide: not run: REASON -->` right above its fence.

## Performance

- Each library has plain C benchmarks in `bench/` with fixed inputs and no dependencies, which print their timings. Timings are never a CI gate: wall time is not repeatable on shared runners.
- A change meant to affect performance is measured before and after on the same machine, and the pull request gives the numbers.
- CI reports the library's size, in total and per optional feature.
- Hot paths take no locks, make no system calls and call nothing locale-dependent.

## Compatibility

- Versions follow Semantic Versioning, and the version lives in one place, the `PP_VERSION_*` macros of the base header, which CMake reads.
- Before 1.0.0 any release may change the API, the ABI and every data format. A data format's version is bumped whenever its bytes change.
- From 1.0.0, within a major version: a program that builds against 1.y builds against any later 1.z and means the same (functions, types, enum values and fields are added, never removed or renamed; a field is added only at the end of its struct; deprecation names the replacement); a 1.z library reads everything an earlier 1.y wrote; results that change between versions are stated in the changelog.
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
| SPDX lines, comment style, history markers, banned calls, mutable file-scope state, `<math.h>` allow list, file names | `tools/check_source.py` |
| Module graph | `tools/check_modules.py` |
| File and function lengths, complexity exceptions | `tools/check_lengths.py` |
| Public documentation blocks and thread safety statements | `tools/check_docs.py` |
| Documentation snippets run by tests | `tools/check_guide.py` |
| Fast math refused | `cmake/CompilerFlags.cmake` |
| Allowed documentation files, em dash, record numbers, links | the organization's documentation check |
| Commit and pull request titles | the organization's title check |
