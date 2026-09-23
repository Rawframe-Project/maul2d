# Design records

Each record explains one decision that shaped both engines: the
problem, the choice and what it costs. Records are the same in Maul2D
and Maul3D. A record that applies to one engine only says so in its
title.

Records are numbered in order and never renumbered. A decision that
is reversed gets a new record that supersedes the old one; the old one
stays, marked superseded.

| Number | Title | Status |
|---|---|---|
| [0001](0001-one-family-one-set-of-rules.md) | One family, one set of rules | Accepted |
| [0002](0002-refusals-record-a-reason.md) | Refusals record a reason and never assert | Accepted |
| [0003](0003-restart-at-0-0-1.md) | Restart the version history at 0.0.1 | Accepted |

## Template

```md
# NNNN. Title in sentence case

Status: Proposed | Accepted | Superseded by NNNN

## Context

What problem forced a decision, with the facts that matter.

## Decision

What we do, stated so that code review can check it.

## Consequences

What this costs, what it enables, and what we give up.
```
