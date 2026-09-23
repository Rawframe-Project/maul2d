#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The comment rule: comments describe the code as it is, not how it got
# there. Development-history markers (slice, task, round and issue
# numbers, bug-report codes, format and revision versions, "pre-N"
# behavior) belong in the changelog and the commit messages, not in the
# source. This script finds them in comments and fails if any remain.
#
# usage: check_comments.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIRS = ["include", "src", "test", "bench", "samples", "testbed"]

MARKERS = re.compile(
    r"(?<![\w.#/~])\d+[a-df-z]?-\d+[a-z]?(?![\w.-])"  # slice codes like 2b-9 or 6-3
    r"|\bR\d+-\d+\b|\bS-\d+[a-z]?\b|\btopic-\d+|\bslice \d+|\brev \d+|\btask \d+"
    r"|\bADR-\d+|\bRT\d-[A-Z]+|\bF-T\d+|\bV-[A-Z]{3,}\b|\bD\d\b|\bv\d{2}\b|\bpre-\d+\b"
    r"|\b[Bb]2 #\d+|(?<![\w{])#\d{3}\b(?!\d)|(?<![\w/.'])[ABDMS]\d{1,2}(?![\w.'])"
    r"|\bregistry [A-Z]\d|\b[Pp]hase \d\b|\b[Rr]ound \d+\b"
    r"|\bintegration audit|\baudit [A-Z]\d|\bred[- ]team|\blesson\b|\bledger\b"
    r"|\bhonest(?:ly)?\b|\bgolden rule\b|\bverbatim\b|\bconvict"
)


def comment_text(line):
    """The comment part of a C line, or None."""
    at = line.find("//")
    if at >= 0:
        return line[at:]
    stripped = line.lstrip()
    if stripped.startswith("*") or stripped.startswith("/*"):
        return stripped
    return None


def main():
    findings = []
    for top in DIRS:
        for folder, _, files in os.walk(os.path.join(ROOT, top)):
            for name in sorted(files):
                if not name.endswith((".c", ".h")):
                    continue
                path = os.path.join(folder, name)
                for number, line in enumerate(open(path, encoding="utf-8", errors="replace"), 1):
                    text = comment_text(line)
                    if text is not None and MARKERS.search(text):
                        rel = os.path.relpath(path, ROOT)
                        findings.append(f"{rel}:{number}: {line.strip()}")
    for finding in findings:
        print(finding)
    if findings:
        print(f"{len(findings)} comment(s) carry development-history markers; "
              "describe the code instead (docs/conventions.md)")
        return 1
    print("comments: no development-history markers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
