#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The journal rule for defs: a record copies a caller's def field by
# field into zeroed memory, never whole. A whole copy carries the bytes
# between the def's fields, which the caller never set, onto the tape, so
# two recordings of the same calls could differ. In src/, this refuses
#
#   - a dereferenced def as a whole value: assigned (record.def = *def;)
#     or in an initializer ({.def = *def, ...});
#   - a def as the source of memcpy or memmove;
#   - a def passed straight to m2JournalRecord as its payload.
#
# A def is a pointer named def or ending in Def. --self-test checks the
# patterns against a table of lines they must refuse and accept.
#
# usage: check_records.py [--self-test]

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEF = r"(?:def|\w+Def)"
WHOLE = "copies a def whole"
RULES = (
    (re.compile(r"=\s*\(?\s*\*\s*" + DEF + r"\s*\)?\s*[;,}]"), WHOLE),
    (re.compile(r"\bmem(?:cpy|move)\s*\([^;]*?,\s*\(?\s*\*?\s*" + DEF + r"\s*\)?\s*,"), WHOLE),
    (re.compile(r"\bm2JournalRecord\s*\([^;]*?,[^;]*?,\s*" + DEF + r"\s*,"), "records a def whole"),
)

REFUSED = (
    "record.def = *def;",
    "record.def = * def ;",
    "record.def = (*def);",
    "m2OpCreateShape record = {.body = bodyId, .def = *def, .expected = id};",
    "m2OpCreateShape record = {.def = *def};",
    "out.shape = *shapeDef;",
    "memcpy(&record.def, def, sizeof(record.def));",
    "memcpy(&record.def, shapeDef, sizeof(*shapeDef));",
    "memmove(&record.def, (def), sizeof(record.def));",
    "m2JournalRecord(world, m2_opExplode, def, (int32_t)sizeof(*def));",
)

ACCEPTED = (
    "RecordShapeDef(&record.def, def);",
    "record.def.density = def->density;",
    "header.count = def->count;",
    "memcpy(out + sizeof(header), def->points, (size_t)pointBytes);",
    "memcpy(&record, &op, sizeof(record));",
    "m2JournalRecord(world, m2_opExplode, &record, (int32_t)sizeof(record));",
    "if (def == NULL || def->internalValue != M2_SHAPE_COOKIE)",
    "m2Result result = m2CreateShape(world, bodyId, def, geometry, shapeOut);",
    "float density = *densityOut;",
)


def findings_in(text):
    """(line, what) for each whole copy of a def in text."""
    found = []
    for pattern, what in RULES:
        for match in pattern.finditer(text):
            found.append((text.count("\n", 0, match.start()) + 1, what))
    return sorted(found)


def self_test():
    wrong = [f"not refused: {line}" for line in REFUSED if not findings_in(line)]
    wrong += [f"not accepted: {line}" for line in ACCEPTED if findings_in(line)]
    for line in wrong:
        print(f"check_records.py self-test: {line}")
    if wrong:
        sys.exit(1)


def main():
    if sys.argv[1:] == ["--self-test"]:
        self_test()
        return
    if sys.argv[1:]:
        sys.exit("usage: check_records.py [--self-test]")
    src = os.path.join(ROOT, "src")
    found = []
    for name in sorted(os.listdir(src)):
        if name.endswith(".c"):
            text = open(os.path.join(src, name), encoding="utf-8").read()
            for number, what in findings_in(text):
                found.append(f"src/{name}:{number}: {what}; copy it field by field instead")
    for line in found:
        print(line)
    if found:
        sys.exit(1)


if __name__ == "__main__":
    main()
