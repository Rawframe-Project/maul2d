#!/usr/bin/env python3
# Generates the API reference from the public headers: every function
# declaration with the documentation comment above it, grouped by
# header. Plain text extraction with no compiler, so the reference
# shows exactly what the headers say. The same script serves every Maul
# library; it reads the library's name and prefix from CMakeLists.txt.
# A part tools/source-dirs.txt lists (DIR/src and DIR/include) gets a
# section of its own for each DIR/include/NAME.
#
# The reference is built into the documentation site by
# tools/build_docs.py and is not committed.
#
# Usage: python3 tools/gen_api.py OUTPUT.md

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def cmake_setting(name):
    """A value the library's CMakeLists.txt sets with set(NAME value)."""
    with open(os.path.join(ROOT, "CMakeLists.txt"), encoding="utf-8") as f:
        match = re.search(r"^set\(" + name + r" \"?([^\")]+)\"?\)", f.read(), re.M)
    if not match:
        raise SystemExit("CMakeLists.txt does not set " + name)
    return match.group(1)


LIB = cmake_setting("MAUL_LIBRARY")
PREFIX = cmake_setting("MAUL_API_PREFIX")
TITLE = cmake_setting("MAUL_TITLE")
HEADER_DIR = os.path.join(ROOT, "include", LIB)
FIRST = ["base.h"]


def header_order(folder, umbrella):
    names = [n for n in os.listdir(folder) if n.endswith(".h") and n != umbrella + ".h"]
    rest = sorted(n for n in names if n not in FIRST)
    return [n for n in FIRST if n in names] + rest


def parts():
    """(name, header directory) of each part's DIR/include/NAME, for the
    directories tools/source-dirs.txt lists, in its order."""
    path = os.path.join(ROOT, "tools", "source-dirs.txt")
    if not os.path.exists(path):
        return []
    result = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        include = os.path.join(ROOT, os.path.normpath(line), "include") if line else ""
        if include and os.path.isdir(include):
            result += [(name, os.path.join(include, name)) for name in sorted(os.listdir(include))
                       if os.path.isdir(os.path.join(include, name))]
    return result


def summary(lines):
    """The header's opening comment, after the license lines."""
    text = []
    for line in lines[2:]:
        if not line.startswith("//"):
            break
        text.append(line[2:].strip())
    return " ".join(t for t in text if t)


def collect(lines):
    """(doc comment, declaration) pairs for every public function,
    including static inline helpers (shown by their signature)."""
    entries = []
    comment = []
    decl = re.compile(r"^\s*(?:" + cmake_setting("MAUL_MACRO_PREFIX") + r"_API\s+)?[A-Za-z_][A-Za-z0-9_ \*]*\b"
                      + PREFIX + r"[A-Za-z0-9_]+\s*\(")
    first_name = re.compile(r"^\s*" + PREFIX + r"[A-Za-z0-9_]+\s*\(")
    depth = 0  # brace depth; declarations live at the extern "C" level
    i = 0
    while i < len(lines):
        stripped = lines[i].strip()
        code = stripped.split("//")[0]
        if stripped.startswith("///"):
            comment.append(stripped[3:].strip())
            i += 1
            continue
        # A long declaration may break after its result type, which
        # clang-format puts on a line of its own: read the two as one.
        line, j = lines[i], i
        if (depth <= 1 and i + 1 < len(lines) and re.match(r"^\s*[A-Za-z_][A-Za-z0-9_ \*]*$", code)
                and first_name.match(lines[i + 1])):
            line, j = lines[i].rstrip() + " " + lines[i + 1].strip(), i + 1
        if (depth <= 1 and decl.match(line) and not stripped.startswith("typedef")
                and not stripped.startswith("#")):
            sig = line.strip()
            while not re.search(r"[;{]\s*$", sig.split("//")[0].rstrip()) and j < len(lines) - 1:
                j += 1
                sig += " " + lines[j].strip()
            inline = sig.split("//")[0].rstrip().endswith("{")
            sig = re.sub(r"\s+", " ", sig.split("//")[0]).strip()
            sig = re.sub(r"^" + cmake_setting("MAUL_MACRO_PREFIX") + r"(?:_[A-Z0-9]+)*_API\s+", "", sig)
            if inline:
                sig = sig[:-1].rstrip() + ";"
                depth += 1
            entries.append((comment, sig))
            comment = []
            i = j + 1
            continue
        depth += code.count("{") - code.count("}")
        if stripped and not stripped.startswith("//"):
            comment = []
        i += 1
    return entries


def render():
    """The API reference as Markdown: a contents list, then each header."""
    body = []
    contents = []
    total = 0
    headers = 0
    sections = [(None, HEADER_DIR, LIB, "## ")] + [(name, folder, name, "### ") for name, folder in parts()]
    for part, folder, umbrella, level in sections:
        names = header_order(folder, umbrella)
        headers += len(names)
        if part is not None:
            body += ["## The `" + part + "` part", ""]
        for name in names:
            lines = open(os.path.join(folder, name), encoding="utf-8").read().split("\n")
            entries = collect(lines)
            if not entries:
                continue
            contents.append("- [`%s`](#%s)" % (name, re.sub(r"[^a-z0-9]+", "", name.lower())))
            body += [level + "`" + name + "`", "", summary(lines), ""]
            for comment, sig in entries:
                body += ["```c", sig, "```"]
                if comment:
                    body.append(" ".join(comment))
                body.append("")
                total += 1
    head = [
        "# " + TITLE + " API reference",
        "",
        "Generated from the public headers, which are the source of truth: %d functions"
        " across %d headers." % (total, headers),
        "",
    ]
    return "\n".join(head + contents + [""] + body)


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: gen_api.py OUTPUT.md")
    with open(sys.argv[1], "w", encoding="utf-8") as f:
        f.write(render())
    print("wrote " + sys.argv[1])


if __name__ == "__main__":
    main()
