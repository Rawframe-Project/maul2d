# 0001. One family, one set of rules

Status: Accepted

## Context

Maul2D and Maul3D solve the same problem in two dimensions and in
three, and a game studio may use both. They were developed one after
the other, and their names, error handling, layouts and habits drifted
apart: the same concept had different names (`StartJournal` and
`JournalBegin`), the allocator hook took different size types, and
each repository had its own idea of where benchmarks and samples live.

## Decision

The two engines are one product family. `docs/conventions.md` states
the family's rules and is identical in both repositories, as are
`CONTRIBUTING.md`, these design records, the tool configuration files
and the scripts that do not depend on the engine. When the engines
differ where they need not, the difference is a bug in one of them.

## Consequences

A change to a shared rule or file lands in both repositories together.
A developer who knows one engine can read the other. Some changes
touch twice the code, and the API of one engine may change only to
match the other.
