# Contributing

Maul2D and Maul3D are one product family and share their rules. Read
[docs/conventions.md](docs/conventions.md) before writing code; it is
the same file in both repositories.

## Before you open a pull request

- The build is warning-free and every test passes in Debug and
  Release:

  ```sh
  cmake -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build
  ctest --test-dir build
  ```

- The code is formatted with the pinned `clang-format`.
- A bug fix comes with a test that fails without it.
- A user-visible change adds a line to `CHANGELOG.md` under
  `[Unreleased]`.
- A change that moves a determinism hash or a benchmark pin explains
  which arithmetic changed and why in the commit message and the
  changelog. CI compares every hash across all platform cells.

## Commits and pull requests

Commit subjects read `area: imperative summary`; the body says what
was wrong and why the change is right. The full rules are in the
conventions. A pull request carries one topic, and its title follows
the same form as a commit subject.

## Reporting bugs

A report with a small program or scene that reproduces the problem
is the most useful kind. For a determinism problem, include the two
platforms and the hash lines each one printed.

## License

Contributions are accepted under the MIT license that covers the
project.
