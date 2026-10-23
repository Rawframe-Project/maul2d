#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The length rules: a function body in the library's sources (src/, and
# DIR/src for each directory tools/source-dirs.txt lists) stays within
# LIMIT lines and a source file within FILE_LIMIT, unless
# tools/length-exceptions.txt lists it with a ceiling and a reason (a
# file is listed with the function name "*"; a file of src/ by its name,
# one of DIR/src by its path). A GPU kernel source (*.wgsl, anywhere
# but build trees and tools/external-dirs.txt's directories) stays
# within FILE_LIMIT too, listed by its path; its functions are not
# measured. The list only shrinks: a listed item may not grow past its
# ceiling, and an entry that is back under its limit (or gone) must be
# deleted.
#
# Cognitive complexity is clang-tidy's check; a function kept above its
# threshold carries NOLINT(readability-function-cognitive-complexity) and
# a reason on its line, and this script lists those exceptions.
#
# usage: check_lengths.py [--list | --tighten | --structs]
#   --list     print every function and file over its limit with its length
#   --tighten  lower every ceiling to the measured length and delete the
#              entries that are back under their limit
#   --structs  print the field count of every struct in src/ and include/,
#              largest first, and the complexity exceptions

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


def source_dirs():
    """The directories tools/source-dirs.txt lists beside src/, each holding
    more of the library's own sources in DIR/src and its headers in
    DIR/include (a part built as a library of its own)."""
    path = os.path.join(ROOT, "tools", "source-dirs.txt")
    if not os.path.exists(path):
        return []
    result = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if line:
            result.append(os.path.normpath(line))
    return result


SOURCE_DIRS = source_dirs()


def library_sources():
    """(directory, prefix) of the library's sources: src/, whose files are
    named alone, and each listed directory's src/, named by their path."""
    return [("src", "")] + [(os.path.join(top, "src"), top.replace(os.sep, "/") + "/src/")
                            for top in SOURCE_DIRS]


def external_dirs():
    """Directories, relative to the root, whose files come from outside."""
    path = os.path.join(ROOT, "tools", "external-dirs.txt")
    if not os.path.exists(path):
        return set()
    result = set()
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if line:
            result.add(os.path.normpath(line))
    return result


def kernel_sources():
    """The paths, relative to the root, of the library's GPU kernels."""
    external = external_dirs()
    for folder, dirs, files in os.walk(ROOT):
        relative = os.path.relpath(folder, ROOT)
        dirs[:] = sorted(d for d in dirs if not d.startswith((".", "build", "_deps"))
                         and os.path.normpath(os.path.join(relative, d)) not in external)
        for name in sorted(files):
            if name.endswith(".wgsl"):
                yield os.path.relpath(os.path.join(folder, name), ROOT).replace(os.sep, "/")


def measured():
    found = {}
    for path in kernel_sources():
        found[(path, "*")] = len(open(os.path.join(ROOT, path), encoding="utf-8").read().split("\n")) - 1
    for top, prefix in library_sources():
        src = os.path.join(ROOT, top)
        for name in sorted(os.listdir(src)):
            if name.endswith((".c", ".m")):
                path = os.path.join(src, name)
                for function, length in functions(path):
                    found[(prefix + name, function)] = length
                found[(prefix + name, "*")] = len(open(path, encoding="utf-8").read().split("\n")) - 1
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


def struct_fields():
    """(field count, struct name, file) for every struct definition."""
    result = []
    for top in ["src", "include"] + SOURCE_DIRS:
        for folder, _, files in os.walk(os.path.join(ROOT, top)):
            for name in sorted(files):
                if not name.endswith(".h"):
                    continue
                path = os.path.join(folder, name)
                text = open(path, encoding="utf-8").read()
                for match in re.finditer(r"struct\s+(\w+)\s*\{(.*?)\n\s*\}", text, re.S):
                    body = match.group(2)
                    fields = [line for line in body.split("\n")
                              if re.search(r";\s*(//.*)?$", line) and not line.strip().startswith("//")]
                    result.append((len(fields), match.group(1), os.path.relpath(path, ROOT)))
    return sorted(result, reverse=True)


def complexity_exceptions():
    found = []
    for top, _ in library_sources():
        src = os.path.join(ROOT, top)
        for name in sorted(os.listdir(src)):
            if name.endswith((".c", ".m")):
                for number, line in enumerate(open(os.path.join(src, name), encoding="utf-8"), 1):
                    if "NOLINT(readability-function-cognitive-complexity)" in line:
                        found.append(f"{top}/{name}:{number}: {line.strip()}")
    return found


def main():
    if "--structs" in sys.argv:
        for count, name, path in struct_fields():
            print(f"{count:4d} {name} ({path})")
        for line in complexity_exceptions():
            print("complexity exception: " + line)
        return 0
    found = measured()
    if "--list" in sys.argv:
        for (name, function), length in sorted(found.items(), key=lambda kv: -kv[1]):
            if length > limit_of((name, function)):
                print(f"{name} {function} {length}")
        return 0
    allow = allowed()
    if "--tighten" in sys.argv:
        path = os.path.join(ROOT, "tools", "length-exceptions.txt")
        kept = []
        for line in open(path, encoding="utf-8").read().split("\n"):
            match = re.match(r"(\S+)\s+(\w+|\*)\s+(\d+)\s+(.+)$", line.strip())
            if not match:
                kept.append(line)
                continue
            key = (match.group(1), match.group(2))
            length = found.get(key, 0)
            if length <= limit_of(key):
                continue
            ceiling = min(int(match.group(3)), length)
            kept.append(f"{key[0]} {key[1]} {ceiling} {match.group(4)}")
        open(path, "w", encoding="utf-8").write("\n".join(kept))
        return 0
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
