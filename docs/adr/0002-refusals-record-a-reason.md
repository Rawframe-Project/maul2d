# 0002. Refusals record a reason and never assert

Status: Accepted

## Context

The engines reject bad input: stale ids, uninitialized defs,
non-finite numbers, full pools. They used to do it by asserting in
debug builds and returning silently in release builds, with a result
code set on only a few paths and stored in one process-wide variable.
A host could not tell why a call failed, debug builds aborted on input
a game may legitimately produce, and two threads refusing at once
overwrote each other's reason.

## Decision

Rejecting input is a refusal, not an internal fault. A refusing
function returns its null result and records why through one internal
path, which sets a thread-local last result and, for invalid input
against a live world, adds one to the world's misuse counter
atomically. The result codes are the same in both engines. Asserts are
kept for internal invariants: states that cannot happen unless the
engine itself is wrong. Running out of memory is a refusal too.

## Consequences

Hosts read the reason right after a failed call and poll the misuse
counter once per frame to catch bugs in release builds. Debug builds no
longer stop on bad input, so tests can exercise every refusal. Every
new public function must route its refusals through the refusal path.
