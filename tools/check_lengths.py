#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The length rules: a function body in src/ stays within LIMIT lines and
# a source file within FILE_LIMIT, unless tools/length-exceptions.txt lists
# it with a ceiling and a reason (a file is listed with the function name
# "*"). The list only shrinks: a listed item may not grow past its
# ceiling, and an entry that is back under its limit (or gone) must be
# deleted.
#
# usage: check_lengths.py [--list]
#   --list  print every function and file over its limit with its length

import os
import re
import sys

LIMIT = 80
FILE_LIMIT = 1000
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def functions(path):
    """Yields (name, body line count) for each top-level function."""
    lines = open(path, encoding="utf-8").read().split("\n")
    i = 0
    while i < len(lines):
        if lines[i] == "{" and i > 0 and lines[i - 1].rstrip().endswith(")"):
            start = i - 1
            while start > 0 and not re.match(r"[A-Za-z]", lines[start]):
                start -= 1
            match = re.search(r"(\w+)\(", lines[start])
            depth = 0
            end = i
            for end in range(i, len(lines)):
                depth += lines[end].count("{") - lines[end].count("}")
                if depth == 0:
                    break
            if match:
                yield match.group(1), end - i - 1
            i = end
        i += 1


def measured():
    found = {}
    src = os.path.join(ROOT, "src")
    for name in sorted(os.listdir(src)):
        if name.endswith(".c"):
            path = os.path.join(src, name)
            for function, length in functions(path):
                found[(name, function)] = length
            found[(name, "*")] = len(open(path, encoding="utf-8").read().split("\n")) - 1
    return found


def limit_of(key):
    return FILE_LIMIT if key[1] == "*" else LIMIT


def allowed():
    entries = {}
    path = os.path.join(ROOT, "tools", "length-exceptions.txt")
    if not os.path.exists(path):
        return entries
    for number, line in enumerate(open(path, encoding="utf-8"), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.match(r"(\S+)\s+(\w+|\*)\s+(\d+)\s+(.+)$", line)
        if not match:
            sys.exit(f"length-exceptions.txt:{number}: expected 'file function ceiling reason'")
        entries[(match.group(1), match.group(2))] = int(match.group(3))
    return entries


def main():
    found = measured()
    if "--list" in sys.argv:
        for (name, function), length in sorted(found.items(), key=lambda kv: -kv[1]):
            if length > limit_of((name, function)):
                print(f"{name} {function} {length}")
        return 0
    allow = allowed()
    errors = []
    for key, length in sorted(found.items()):
        if length <= limit_of(key):
            continue
        what = "the file" if key[1] == "*" else key[1]
        if key not in allow:
            errors.append(f"{key[0]}: {what} is {length} lines (limit {limit_of(key)}); "
                          "split it or list it with a reason")
        elif length > allow[key]:
            errors.append(f"{key[0]}: {what} grew to {length} lines (ceiling {allow[key]})")
    for key, ceiling in sorted(allow.items()):
        if found.get(key, 0) <= limit_of(key):
            errors.append(f"length-exceptions.txt: {key[0]} {key[1]} is gone or within the "
                          "limit; delete the entry")
    for error in errors:
        print(error)
    if errors:
        return 1
    print(f"length rules: {len(allow)} listed exceptions, all within their ceilings")
    return 0


if __name__ == "__main__":
    sys.exit(main())
