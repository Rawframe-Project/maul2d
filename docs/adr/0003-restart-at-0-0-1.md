# 0003. Restart the version history at 0.0.1

Status: Accepted

## Context

Both engines had been tagged 1.x and promised a frozen 1.x API while
the design was still moving: new options went into setters instead of
defs to avoid growing def structs, draw callbacks were split across
three structs to keep old layouts, and the version numbers in CMake,
the headers and the README disagreed. No project depends on either
engine yet.

## Decision

The earlier releases and tags are withdrawn and both engines restart
at 0.0.1. Until 1.0.0 any minor release may change the API, the ABI
and the snapshot and journal formats. The version lives only in the
public base header; the build reads it from there. The snapshot and
journal format versions restart at 1.

## Consequences

The API can be cleaned up and the two engines aligned without
compatibility shims. Users of the old tags must move to the new
versions by hand. 1.0.0 becomes the point where the API is frozen for
real.
