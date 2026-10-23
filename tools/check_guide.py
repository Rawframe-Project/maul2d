#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The documentation's C snippets are built and run by the tests: every
# ```c block of docs/user/*.md must appear unchanged in one of
# test/test_guide*.c,
# every non-blank line indented by the same number of spaces (blank lines
# stay blank), so a snippet that no longer builds or runs fails the tests
# instead of drifting from the API. A block whose fence line comes right
# after the comment
#
#     <!-- guide: not run: REASON -->
#
# is not checked, for a fragment no test can build everywhere (another
# platform's API, say); the reason is printed. Pages without C blocks
# pass; a library without docs/user/ has nothing to check.
#
# usage: check_guide.py

import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXEMPT = re.compile(r"<!--\s*guide:\s*not run:\s*(.+?)\s*-->\s*$")
MAX_INDENT = 16


def snippets(guide):
    """Each C block of the guide as (number, first line, lines, the reason
    it is not run or None)."""
    found = []
    lines = guide.split("\n")
    i = 0
    while i < len(lines):
        if lines[i].strip() == "```c":
            before = lines[i - 1] if i > 0 else ""
            exempt = EXEMPT.search(before)
            end = i + 1
            while end < len(lines) and lines[end].strip() != "```":
                end += 1
            body = lines[i + 1:end]
            first = next((line.strip() for line in body if line.strip()), "")
            found.append((len(found) + 1, i + 1, first, body,
                          exempt.group(1) if exempt else None))
            i = end
        i += 1
    return found


def indented(body, indent):
    pad = " " * indent
    return "\n".join(pad + line if line else "" for line in body) + "\n"


def run_by(body, tests):
    """Whether a test file holds the block at one indentation."""
    for text in tests:
        for indent in range(MAX_INDENT + 1):
            if "\n" + indented(body, indent) in text:
                return True
    return False


def main():
    pages = sorted(glob.glob(os.path.join(ROOT, "docs", "user", "*.md")))
    if not pages:
        print("guide: no docs/user/ pages")
        return 0
    tests = []
    for name in sorted(glob.glob(os.path.join(ROOT, "test", "test_guide*.c"))):
        with open(name, encoding="utf-8") as f:
            tests.append("\n" + f.read())
    total = 0
    missing = 0
    exempt = 0
    for path in pages:
        rel = os.path.relpath(path, ROOT)
        with open(path, encoding="utf-8") as f:
            blocks = snippets(f.read())
        total += len(blocks)
        for number, line, first, body, reason in blocks:
            if reason is not None:
                print(f"{rel}:{line}: C snippet {number} not run: {reason}")
                exempt += 1
            elif not run_by(body, tests):
                print(f"{rel}:{line}: C snippet {number} ({first}) is not in "
                      "test/test_guide*.c as written")
                missing += 1
    if missing:
        print(f"{missing} of {total} documentation snippets not run by the tests")
        return 1
    print(f"guide: {total - exempt} C snippets in test/test_guide*.c, {exempt} not run")
    return 0


if __name__ == "__main__":
    sys.exit(main())
