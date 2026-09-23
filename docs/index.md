# Maul2D

A deterministic 2D physics engine for games in C17, with no
dependencies and an MIT license. The whole world snapshots, restores
and replays to the bit on every platform CI covers: x64 and arm64
Linux and Windows, arm64 macOS, and WebAssembly.

- [Try the testbed](testbed/): the engine compiled to WebAssembly,
  running in your browser. Hold R and time runs backward.
- [The guide](guide.html): the contract, rollback netcode, joints,
  water, queries, the character mover and recipes.
- [API reference](api.html): every public function, generated from
  the headers.
- [Samples](samples.html): small complete programs.
- [The changelog](changelog.html): every release and what changed.
- [The repository](https://github.com/Rawframe-Project/maul2d):
  source, releases and the test suites.

## The promises

1. Same inputs, same bits, every platform, every thread count.
2. Snapshot, restore, journal replay: bit-exact, always.
3. Bad input is refused with a reason; it never corrupts quietly.
