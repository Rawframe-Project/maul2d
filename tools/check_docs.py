#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The public API documentation contract. Every function a public header
# declares with the library's API macro has a
# /// block directly above it with:
#
# - a summary sentence as its first line;
# - @param for each named parameter, and none for a parameter that does
#   not exist;
# - @return, unless the function returns void;
# - a "@par Thread safety" paragraph that opens with one of the
#   statements below (THREAD_SAFETY), so a caller always learns whether
#   and how a function may be called from several threads.
#
# The API reference is generated from these blocks, so a missing one is a
# hole in the reference.
#
# usage: check_docs.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The openings a thread safety paragraph may have.
THREAD_SAFETY = [
    re.compile(r"Safe from any thread\.(\s|$)"),
    re.compile(r"Safe from any thread; [^.;]+ (is|are) used by one thread at a time[.,;]"),
    re.compile(r"Main thread only\.(\s|$)"),
    re.compile(r"Safe from any thread; it runs on the main thread, waiting at most the marshal "
               r"deadline\.(\s|$)"),
    re.compile(r"Real-time safe: no allocation, lock or wait\.(\s|$)"),
]


def cmake_setting(name):
    """A value the library's CMakeLists.txt sets with set(NAME value)."""
    with open(os.path.join(ROOT, "CMakeLists.txt"), encoding="utf-8") as f:
        match = re.search(r"^set\(" + name + r" \"?([^\")]+)\"?\)", f.read(), re.M)
    if not match:
        raise SystemExit("CMakeLists.txt does not set " + name)
    return match.group(1)


def split_top_level(text):
    """Splits a parameter list at the commas outside parentheses."""
    parts, depth, current = [], 0, ""
    for char in text:
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
        if char == "," and depth == 0:
            parts.append(current)
            current = ""
        else:
            current += char
    parts.append(current)
    return [part.strip() for part in parts if part.strip()]


def parameter_name(parameter):
    pointer = re.search(r"\(\s*\*\s*(\w+)\s*\)", parameter)
    if pointer:
        return pointer.group(1)
    names = re.findall(r"[A-Za-z_]\w*", re.sub(r"\[[^\]]*\]", "", parameter))
    return names[-1] if names else None


def declarations(lines, macro):
    """Yields (line number, doc lines, declaration text) for each API function."""
    start_pattern = re.compile(r"^\s*(?:" + macro + r"_NODISCARD\s+)?" + macro + r"_API\b")
    i = 0
    while i < len(lines):
        if start_pattern.match(lines[i]):
            first = i
            text = lines[i]
            while ";" not in text and i + 1 < len(lines):
                i += 1
                text += " " + lines[i].strip()
            doc = []
            j = first - 1
            while j >= 0 and lines[j].strip().startswith("///"):
                doc.insert(0, lines[j].strip()[3:].strip())
                j -= 1
            yield first + 1, doc, text
        i += 1


def check(rel, number, doc, text, macro, errors):
    match = re.search(r"(\w+)\s*\((.*)\)\s*;", text)
    if not match:
        errors.append(f"{rel}:{number}: cannot read the declaration")
        return
    name, params = match.group(1), match.group(2)
    head = text[:match.start(1)]
    head = re.sub(r"\b" + macro + r"_(API|NODISCARD)\b", "", head).strip()
    where = f"{rel}:{number}: {name}"
    if not doc:
        errors.append(f"{where}: no /// documentation")
        return
    if not doc[0] or doc[0].startswith("@"):
        errors.append(f"{where}: the first /// line is not a summary sentence")
    names = [] if params.strip() in ("", "void") else [
        parameter_name(p) for p in split_top_level(params)]
    documented = [m.group(1) for line in doc for m in [re.match(r"@param(?:\[\w+\])?\s+(\w+)", line)]
                  if m]
    for param in names:
        if param and param not in documented:
            errors.append(f"{where}: no @param for {param}")
    for param in documented:
        if param not in names:
            errors.append(f"{where}: @param {param} names no parameter")
    returns_void = re.fullmatch(r"(?:static\s+)?(?:inline\s+)?void", head) is not None
    has_return = any(line.startswith("@return") for line in doc)
    if not returns_void and not has_return:
        errors.append(f"{where}: no @return")
    if returns_void and has_return:
        errors.append(f"{where}: @return on a void function")
    starts = [i for i, line in enumerate(doc) if line.startswith("@par Thread safety")]
    if not starts:
        errors.append(f"{where}: no '@par Thread safety' paragraph")
        return
    paragraph = []
    for line in doc[starts[0] + 1:]:
        if not line or line.startswith("@"):
            break
        paragraph.append(line)
    text = " ".join(paragraph)
    if not any(pattern.match(text) for pattern in THREAD_SAFETY):
        errors.append(f"{where}: the thread safety paragraph opens with none of the "
                      "statements in tools/check_docs.py")


def main():
    lib = cmake_setting("MAUL_LIBRARY")
    macro = cmake_setting("MAUL_MACRO_PREFIX")
    folder = os.path.join(ROOT, "include", lib)
    errors = []
    count = 0
    for name in sorted(os.listdir(folder)):
        if not name.endswith(".h"):
            continue
        rel = f"include/{lib}/{name}"
        lines = open(os.path.join(folder, name), encoding="utf-8").read().split("\n")
        for number, doc, text in declarations(lines, macro):
            count += 1
            check(rel, number, doc, text, macro, errors)
    for error in errors:
        print(error)
    if errors:
        print(f"{len(errors)} finding(s) in {count} public functions")
        return 1
    print(f"documentation: all {count} public functions documented")
    return 0


if __name__ == "__main__":
    sys.exit(main())
