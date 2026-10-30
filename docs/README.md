# Documentation

## Using the library

- [Getting started](user/getting-started.md): requirements, building, installing, build options, a first world, the model the API follows, samples and the testbed.
- [Bodies and shapes](user/bodies-and-shapes.md): moving bodies, sleep, dominance, shapes and materials, filters and sensors, chains, geometry from points, shattering, explosions.
- [Joints](user/joints.md): the eleven joint types, motors and limits, tuning, breaking.
- [Events and queries](user/events-and-queries.md): contact, sensor and joint events; rays, sweeps and overlaps; the character mover; walking the world.
- [Fluids](user/fluids.md): particle pools, behaviors and jelly, particle lifetimes, water volumes, wind.
- [Rollback and replay](user/rollback-and-replay.md): fixed steps, snapshots and rollback netcode, the journal, hunting a divergence, bytes from outside, formats.
- [Diagnostics](user/diagnostics.md): counters, validation, debug drawing, hashes.
- [Limits](user/limits.md): the range of every number the step multiplies, what is clamped, and why.
- [Recipes](user/recipes.md): top-down worlds, platformers, vehicles, destructible geometry, water, lockstep.
- [API reference](https://rawframe-project.github.io/maul2d/api.html), generated from the headers.

## How it works

- [Overview](internals/overview.md): the library profile, ids and misuse, host code, a step, how the parts fit.
- [Glossary](internals/glossary.md).
- [Determinism](internals/determinism.md): arithmetic, order, threads, state, and the golden scenes that hold them.
- [Snapshots and the journal](internals/snapshots-and-journal.md): the state table, the formats, untrusted bytes, replay.
- [References](internals/references.md): the published sources behind the algorithms.
- [Maul conventions](internals/maul-conventions.md): the rules every Maul library follows.

## Maintaining it

- [Development](operations/development.md): toolchain, presets, tests, golden values, benchmarks, the testbed, shared family files.
- [Checks](operations/checks.md): the local tiers, the CI jobs and their budgets.
- [Release](operations/release.md).
