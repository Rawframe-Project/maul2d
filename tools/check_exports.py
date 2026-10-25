#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The symbols a shared build exports are the public functions, no more
# and no fewer: every function the library's public headers declare with
# its API macro, and nothing else, so internal functions, tables and
# globals stay out of the binary interface. The headers are the ones in
# include/<MAUL_LIBRARY>/ that tools/gen_api.py reads. A part
# tools/source-dirs.txt lists is a library of its own and is not counted.
# Their functions are what gen_api.collect finds, less the static inline
# helpers, which no library exports. Every one carries the API macro,
# except a function the program defines and the library calls, which
# --program names; a named one that is missing, carries the macro or is
# exported fails the check. The library's name and prefixes come from
# CMakeLists.txt through gen_api.
# A header whose functions the build leaves out, such as a platform's
# adapter on another platform or a companion static library, is named
# with --exclude; an excluded header that is missing, or one whose
# functions the library exports, fails the check, so the list stays
# true. Every defined dynamic symbol counts, whatever its name or kind,
# and a versioned one by its name before the '@': one without the
# prefix, one holding data, or one no header declares fails it, as does
# a declared function missing from the library. Each failure names the
# symbol.
#
#   python3 tools/check_exports.py [--nm NM] [--exclude HEADER ...] [--program FUNCTION ...] \
#       out/linux-shared/lib<library>.so
#
# Reads the dynamic symbol table with nm (CMake passes CMAKE_NM); for ELF
# shared libraries.

import argparse
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_api  # noqa: E402

# nm's letters for code: global, local or weak.
CODE = set("TtWw")


def declared(excluded):
    """The functions the library's public headers declare: those of the
    counted headers, the subset declared without the API macro, and those
    of each excluded header."""
    macro = re.compile(r"\b" + gen_api.cmake_setting("MAUL_MACRO_PREFIX") + r"_API\b[\w\s*]*?\b("
                       + gen_api.PREFIX + r"\w+)\s*\(")
    public, unmarked, left_out = set(), set(), {}
    for header in gen_api.header_order(gen_api.HEADER_DIR, gen_api.LIB):
        with open(os.path.join(gen_api.HEADER_DIR, header), encoding="utf-8") as f:
            text = f.read()
        marked = set(macro.findall(text))
        names = set()
        for _, signature in gen_api.collect(text.split("\n")):
            found = re.search(r"\b(" + gen_api.PREFIX + r"\w+)\s*\(", signature)
            if found and "static" not in signature[:found.start()].split():
                names.add(found.group(1))
        if header in excluded:
            left_out[header] = names
        else:
            public |= names
            unmarked |= names - marked
    return public, unmarked, left_out


def exported(nm, library):
    """Each defined dynamic symbol's name, before any version, and nm's
    letter for it."""
    out = subprocess.run([nm, "-D", "--defined-only", library], check=True,
                         capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        fields = line.split()
        if len(fields) == 3:
            name, _, version = fields[2].partition("@")
            found.append((name, version.lstrip("@"), fields[1]))
    # A version script makes the linker define each version node as an
    # absolute symbol named after it (GNU nm prints V, llvm-nm V@@V); it
    # is no function or data of the library's.
    versions = {version for _, version, _ in found if version}
    return {name: letter for name, _, letter in found
            if not (letter == "A" and name in versions)}


def main():
    parser = argparse.ArgumentParser(description="Checks a shared library's exports.")
    parser.add_argument("--nm", default="nm", help="the nm to read the library with")
    parser.add_argument("--exclude", action="append", default=[], metavar="HEADER",
                        help="a header in include/<library>/ whose functions the build leaves out")
    parser.add_argument("--program", action="append", default=[], metavar="FUNCTION",
                        help="a function the headers declare without the API macro for the program to define")
    parser.add_argument("library")
    args = parser.parse_args()
    public, unmarked, left_out = declared(set(args.exclude))
    program = set(args.program)
    symbols = exported(args.nm, args.library)
    prefix = gen_api.PREFIX
    macro = gen_api.cmake_setting("MAUL_MACRO_PREFIX") + "_API"
    wrong = []
    for header in sorted(set(args.exclude) - left_out.keys()):
        wrong.append(f"excluded but not a public header: {header}")
    for header, names in sorted(left_out.items()):
        for name in sorted(names & symbols.keys()):
            wrong.append(f"exported but its header {header} is excluded: {name}")
    for name in sorted(unmarked - program):
        wrong.append(f"declared without {macro}: {name}")
    for name in sorted(program - unmarked):
        wrong.append(f"named as the program's but not declared without {macro}: {name}")
    for name in sorted(program & symbols.keys()):
        wrong.append(f"exported but the program defines it: {name}")
    hidden = set().union(*left_out.values()) | program
    public -= program
    for name, letter in sorted(symbols.items()):
        if not name.startswith(prefix):
            wrong.append(f"exported without the {prefix} prefix: {name}")
        elif letter not in CODE:
            wrong.append(f"exported data: {name}")
        elif name not in public and name not in hidden:
            wrong.append(f"exported but not public: {name}")
    for name in sorted(public - symbols.keys()):
        wrong.append(f"public but not exported: {name}")
    for line in wrong:
        print(line)
    if wrong or not public:
        return 1
    print(f"exports: the {len(public)} public functions, nothing else")
    return 0


if __name__ == "__main__":
    sys.exit(main())
