#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Checks that every internal function returning bool carries the
# library's nodiscard macro: each one declared in a header under src/,
# and each static one in a source file under src/, its subdirectories
# (generated files among them) included, and the same under DIR/src for
# each directory tools/source-dirs.txt lists. A bool answers or refuses,
# and a caller that drops it goes on past a refusal or with an out value
# left unset; a call that means to drop it says so with (void). The macro
# is any <PP>_NODISCARD that a base.h under include/ or DIR/include
# defines. A declaration starts at the line's first column, with only
# the library's macros, standard attributes, static, inline and extern
# before bool; bool may end its line, the name then starting the next.
# The check fails closed: a bool declaration in any other shape (indented,
# or with something else before bool) is an error naming its place, so it
# never passes what it did not check. Run with --self-test to check the
# matching on its own examples.

import pathlib
import re
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent

MACRO = re.compile(r"^#define ([A-Z0-9]+)_NODISCARD\b", re.MULTILINE)


def Prefixes(root, tops):
    """The prefixes whose <PP>_NODISCARD the base.h files under
    root/include and each top's include define."""
    found = set()
    for top in [root] + [root / t for t in tops]:
        for base in sorted((top / "include").glob("*/base.h")):
            found.update(MACRO.findall(base.read_text(encoding="utf-8")))
    return found


def Patterns(prefixes):
    """The declaration, indented, wrapped, name-line and unread patterns
    for the given prefixes."""
    macros = "|".join(sorted(p + r"_\w+" for p in prefixes)) or r"(?!)"
    specifiers = r"((?:(?:" + macros + r"|\[\[\w+\]\]|static|inline|extern)\s+)*)"
    return (
        # A function declaration or definition starting at the line's
        # first column: its specifiers and macros, bool, the name and its
        # parenthesis.
        re.compile(r"^" + specifiers + r"bool\s+(\w+)\s*\("),
        # The same, indented, on one line or ending with bool: not read,
        # so refused.
        re.compile(r"^\s+" + specifiers + r"bool(?:\s+(\w+)\s*\(|\s*$)"),
        # A declaration whose line ends with bool, the name starting the
        # next line, which the next pattern reads.
        re.compile(r"^" + specifiers + r"bool\s*$"),
        re.compile(r"^(\w+)\s*\("),
        # Any other line at the first column naming a bool function, such
        # as one with an attribute before bool: not read, so refused.
        re.compile(r"^(?![\s#/*]|typedef\b).*\bbool\s+(\w+)\s*\("),
    )


def Problems(text, header, prefixes):
    """The (line, message) of each bool function the text declares without
    a nodiscard macro (every one in a header, the static ones elsewhere),
    and of each declaration in a shape the check does not read."""
    declaration, indented, wrapped, name_line, unread = Patterns(prefixes)
    marks = {p + "_NODISCARD" for p in prefixes}
    lines = text.splitlines()
    found = []
    for number, line in enumerate(lines, 1):
        if indented.match(line):
            found.append((number, "an indented bool declaration, which this check does not read: "
                                  "start it at the line's first column"))
            continue
        match = declaration.match(line)
        if match is not None:
            specifiers, name = match.group(1).split(), match.group(2)
        else:
            ends = wrapped.match(line)
            following = name_line.match(lines[number]) if ends and number < len(lines) else None
            if following is None:
                if ends or unread.match(line):
                    found.append((number, "a bool declaration this check does not read: put only "
                                          "the library's macros, attributes, static, inline and "
                                          "extern before bool, and the name after it or first on "
                                          "the next line"))
                continue
            specifiers, name = ends.group(1).split(), following.group(1)
        if not header and "static" not in specifiers:
            continue
        if not marks.intersection(specifiers):
            found.append((number, f"{name} returns bool without its nodiscard macro"))
    return found


def SourceDirs(root):
    """The directories tools/source-dirs.txt lists beside src/."""
    path = root / "tools" / "source-dirs.txt"
    if not path.exists():
        return []
    tops = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            tops.append(line)
    return tops


def Scan(root):
    """Every problem in the C files under root/src and each listed
    directory's src, as printable lines; and the prefixes read."""
    tops = SourceDirs(root)
    prefixes = Prefixes(root, tops)
    lines = []
    for top in [root] + [root / t for t in tops]:
        for path in sorted((top / "src").rglob("*.[ch]")):
            text = path.read_text(encoding="utf-8")
            for number, message in Problems(text, path.suffix == ".h", prefixes):
                lines.append(f"{path.relative_to(root)}:{number}: {message}")
    return lines, prefixes


def SelfTest():
    cases = [
        ("bool ppA(int x);", True, 1),
        ("PP_NODISCARD bool ppA(int x);", True, 0),
        ("static inline bool ppB(void)", True, 1),
        ("PP_NODISCARD static inline bool ppB(void)", True, 0),
        ("static bool Step(Parser* p)", False, 1),
        ("PP_NOINLINE static bool Step(Parser* p)", False, 1),
        ("PP_NODISCARD PP_NOINLINE static bool Step(Parser* p)", False, 0),
        ("bool ppA(int x)", False, 0),
        ("static bool s_ready = false;", False, 0),
        ("    bool done;", True, 0),
        ("    bool ok = ppA(1);", False, 0),
        ("typedef bool (*Visit)(void* context);", True, 0),
        ("// bool ppA(int x) answers", True, 0),
        (" * bool ppA(int x) answers", True, 0),
        ("#define PP_IS(x) bool ppIs(x)", True, 0),
        # Another library's macro does not count.
        ("QQ_NODISCARD bool ppA(int x);", True, 1),
        # The shapes the check cannot read fail, marked or not.
        ("    PP_NODISCARD bool ppA(int x);", True, 1),
        ("    static bool Step(Parser* p)", False, 1),
        ("bool\nppA(int x);", True, 1),
        ("PP_NODISCARD static bool\nStep(Parser* p)", False, 0),
        ("[[maybe_unused]] static PP_NOINLINE bool\nStep(Parser* p)", False, 1),
        ("static bool\n", False, 1),
        ("    static bool\nStep(Parser* p)", False, 1),
        # An indented line merely ending in bool is no declaration.
        ("    // the answer is a plain bool", True, 0),
        ("    int width; // as wide as a bool", True, 0),
        ("[[maybe_unused]] static bool Step(Parser* p)", False, 1),
        ("PP_NODISCARD [[maybe_unused]] static bool Step(Parser* p)", False, 0),
        ("PP_NODISCARD __attribute__((unused)) bool ppA(int x);", True, 1),
    ]
    failed = 0
    for text, header, expected in cases:
        got = len(Problems(text, header, {"PP"}))
        if got != expected:
            print(f"check_nodiscard self-test: {text!r} gave {got} problems, expected {expected}")
            failed += 1
    # The macro comes from base.h, and a header under a subdirectory of
    # src/ and a listed part's src/ is read too.
    with tempfile.TemporaryDirectory() as directory:
        root = pathlib.Path(directory)
        (root / "include" / "pp").mkdir(parents=True)
        (root / "include" / "pp" / "base.h").write_text("#define PP_NODISCARD [[nodiscard]]\n",
                                                         encoding="utf-8")
        (root / "src" / "nested").mkdir(parents=True)
        (root / "src" / "nested" / "deep.h").write_text("bool ppDeep(void);\n", encoding="utf-8")
        (root / "src" / "fine.h").write_text("PP_NODISCARD bool ppFine(void);\n", encoding="utf-8")
        (root / "tools").mkdir()
        (root / "tools" / "source-dirs.txt").write_text("part # a part\n", encoding="utf-8")
        (root / "part" / "src").mkdir(parents=True)
        (root / "part" / "src" / "part.h").write_text("bool ppPart(void);\n", encoding="utf-8")
        found, prefixes = Scan(root)
        if prefixes != {"PP"} or len(found) != 2 or "src/nested/deep.h:1:" not in found[0] \
                or "part/src/part.h:1:" not in found[1]:
            print(f"check_nodiscard self-test: the tree gave {found} with prefixes {prefixes}")
            failed += 1
    return failed == 0


def main():
    if "--self-test" in sys.argv[1:]:
        return 0 if SelfTest() else 1
    problems, prefixes = Scan(ROOT)
    if not prefixes:
        print("nodiscard: no base.h under include/ defines a <PP>_NODISCARD")
        return 1
    for line in problems:
        print(line)
    if problems:
        print(f"nodiscard: {len(problems)} bool declarations unmarked or unread")
        return 1
    print("nodiscard: every internal bool function carries its nodiscard macro")
    return 0


if __name__ == "__main__":
    sys.exit(main())
