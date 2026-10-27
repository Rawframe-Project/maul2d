#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The journal rule for defs: a record copies a caller's def field by
# field into zeroed memory, never whole. A whole copy carries the bytes
# between the def's fields, which the caller never set, onto the tape, so
# two recordings of the same calls could differ. In src/, this refuses
#
#   - an assignment from a dereferenced def pointer (record.def = *def);
#   - a def pointer passed straight to m2JournalRecord as its payload.
#
# usage: check_records.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WHOLE_COPY = re.compile(r"=\s*\*\s*(def|\w+Def)\s*;")
DEF_PAYLOAD = re.compile(r"\bm2JournalRecord\s*\([^;]*?,[^;]*?,\s*(def|\w+Def)\s*,")


def findings():
    src = os.path.join(ROOT, "src")
    found = []
    for name in sorted(os.listdir(src)):
        if not name.endswith(".c"):
            continue
        text = open(os.path.join(src, name), encoding="utf-8").read()
        rules = ((WHOLE_COPY, "copies a def whole"), (DEF_PAYLOAD, "records a def whole"))
        for pattern, what in rules:
            for match in pattern.finditer(text):
                number = text.count("\n", 0, match.start()) + 1
                found.append(f"src/{name}:{number}: {what}; copy it field by field instead")
    return found


def main():
    found = findings()
    for line in found:
        print(line)
    if found:
        sys.exit(1)


if __name__ == "__main__":
    main()
