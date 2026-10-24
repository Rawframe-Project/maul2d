#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The source rules that neither the compiler nor clang-format checks:
#
# - every C file, Objective-C (.m) included, starts with its SPDX line;
#   comments are // or ///;
# - comments carry no development history (history lives in git) and no
#   TODO or FIXME (open an issue instead);
# - src/ calls no <math.h> function but those tools/libm-allowed.txt
#   lists, when the library has that file;
# - src/ calls no function the family bans (memory goes through the
#   allocator, failures are returned statuses, the library prints
#   nothing, no unsafe string functions, nothing locale-dependent) and
#   none the library bans in tools/source-bans.txt;
# - src/ keeps no thread-local or mutable file-scope state and assigns
#   nothing inside a condition; src/fpenv.c, the family's floating-point
#   environment module, alone keeps the thread's saved control register
#   and entry depth, which mirror the processor's per-thread register;
# - C file names are snake_case.
#
# Directories listed in tools/source-dirs.txt hold more of the library's
# sources, in DIR/src, which the rules for src/ hold too.
# Directories listed in tools/external-dirs.txt hold data from outside
# (the Unicode Character Database, for example) and are not checked.
# Markdown and the em dash are the organization's documentation check's.
#
# usage: check_source.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
C_DIRS = ["include", "src", "test", "bench", "samples", "testbed", "tools"]
SKIP_DIRS = {".git", "build", "_site", "out"}

MARKERS = re.compile(
    r"(?<![\w.#/~])\d+[a-df-z]?-\d+[a-z]?(?![\w.-])"  # slice codes like 2b-9 or 6-3
    r"|\bR\d+-\d+\b|\bS-\d+[a-z]?\b|\btopic-\d+|\bslice \d+|\brev \d+|\btask \d+"
    r"|\bADR-\d+|\bRT\d-[A-Z]+|\bF-T\d+|\bV-[A-Z]{3,}\b|\bD\d\b|\bv\d{2}\b|\bpre-\d+\b"
    r"|(?<![\w{])#\d{3}\b(?!\d)|(?<![\w/.'])[ABDMS]\d{1,2}(?![\w.'])"
    r"|\([FRUW]\d{1,2}\b|\b[FRUW]\d{1,2}'s\b|\b(?:decisions?|records?|items?) [FRUW]\d{1,2}\b"
    r"|\bregistry [A-Z]\d|\b[Pp]hase \d\b|\b[Rr]ound \d+\b"
    r"|\bintegration audit|\baudit [A-Z]\d|\bred[- ]team|\blesson\b|\bledger\b"
)
TODO = re.compile(r"\b(TODO|FIXME|XXX)\b")

FAMILY_BANS = {
    "malloc": "memory goes through the object's allocator",
    "calloc": "memory goes through the object's allocator",
    "realloc": "memory goes through the object's allocator",
    "free": "memory goes through the object's allocator",
    "aligned_alloc": "memory goes through the object's allocator",
    "assert": "invariants use the library's PP_ASSERT",
    "abort": "a library returns statuses; invariants use PP_ASSERT",
    "exit": "a library never ends the process",
    "printf": "a library prints nothing",
    "fprintf": "a library prints nothing",
    "puts": "a library prints nothing",
    "strcpy": "unbounded string function",
    "strcat": "unbounded string function",
    "sprintf": "unbounded string function",
    "vsprintf": "unbounded string function",
    "gets": "unbounded string function",
    "atoi": "no error reporting",
    "atol": "no error reporting",
    "atof": "locale-dependent and no error reporting",
    "strtod": "locale-dependent",
    "setlocale": "locale-dependent",
    "isalpha": "locale-dependent",
    "isdigit": "locale-dependent",
    "isspace": "locale-dependent",
    "isalnum": "locale-dependent",
    "isupper": "locale-dependent",
    "islower": "locale-dependent",
    "toupper": "locale-dependent",
    "tolower": "locale-dependent",
}
# The one place a zeroed allocator reaches the C library.
ALLOCATOR_FILE = "allocator.c"
ALLOCATOR_CALLS = {"malloc", "calloc", "realloc", "free", "aligned_alloc"}
# The family's floating-point environment module keeps the thread's saved
# control register and entry depth: state that mirrors the processor's
# own per-thread register and carries no failure.
FLOAT_ENV_FILE = "src/fpenv.c"

THREAD_LOCAL = re.compile(r"\b(_Thread_local|thread_local|__thread)\b|__declspec\s*\(\s*thread\s*\)")
FILE_SCOPE_STATE = re.compile(r"^static\s+(?!const\b)(?!inline\b)(?![^(=;]*\()[^=;]*[=;]")
ASSIGN_IN_CONDITION = re.compile(
    r"\b(?:if|while)\s*\((?:[^()]|\([^()]*\))*?[^=!<>+\-*/%&|^]=(?!=)")
SNAKE_CASE = re.compile(r"^[a-z0-9_]+\.[chm]$")


def code_text(line):
    """A C line without its // comment and its string and char literals."""
    line = re.sub(r'"(?:[^"\\]|\\.)*"', '""', line)
    line = re.sub(r"'(?:[^'\\]|\\.)*'", "''", line)
    at = line.find("//")
    return line if at < 0 else line[:at]


def comment_text(line):
    """The // comment part of a C line, or None."""
    code = re.sub(r'"(?:[^"\\]|\\.)*"', '""', line)
    at = code.find("//")
    return None if at < 0 else line[at:]


def library_bans():
    """The library's own bans: one 'name reason' pair per line."""
    bans = {}
    path = os.path.join(ROOT, "tools", "source-bans.txt")
    if not os.path.exists(path):
        return bans
    for number, line in enumerate(open(path, encoding="utf-8"), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.match(r"(\w+)\s+(.+)$", line)
        if not match:
            sys.exit(f"source-bans.txt:{number}: expected 'name reason'")
        bans[match.group(1)] = match.group(2)
    return bans


# The functions of C23's <math.h> (with their f and l forms), for a
# library that lists in tools/libm-allowed.txt the ones it may call: any
# other is refused in its own sources, because its results differ between
# C libraries. The classification macros (isnan, signbit and the like) are
# exact and not listed.
_LIBM_BASES = """
acos asin atan atan2 cos sin tan acospi asinpi atanpi atan2pi cospi sinpi
tanpi acosh asinh atanh cosh sinh tanh exp exp10 exp10m1 exp2 exp2m1 expm1
frexp ilogb ldexp llogb log log10 log10p1 log1p logp1 log2 log2p1 logb modf
scalbn scalbln cbrt compoundn fabs hypot pow pown powr rootn rsqrt sqrt erf
erfc lgamma tgamma ceil floor nearbyint rint lrint llrint round lround
llround roundeven trunc fromfp ufromfp fromfpx ufromfpx fmod remainder
remquo copysign nan nextafter nexttoward nextup nextdown canonicalize fdim
fmax fmin fmaximum fminimum fmaximum_mag fminimum_mag fmaximum_num
fminimum_num fmaximum_mag_num fminimum_mag_num fma
""".split()
LIBM = {base + suffix for base in _LIBM_BASES for suffix in ("", "f", "l")} | {
    narrow + op + wide
    for op in ("add", "sub", "mul", "div", "fma", "sqrt")
    for narrow, wides in (("f", ("", "l")), ("d", ("l",)))
    for wide in wides
}


def libm_allowed():
    """The <math.h> functions the library may call, from
    tools/libm-allowed.txt, or None when it lists none (no limit)."""
    path = os.path.join(ROOT, "tools", "libm-allowed.txt")
    if not os.path.exists(path):
        return None
    allowed = set()
    for number, line in enumerate(open(path, encoding="utf-8"), 1):
        name = line.split("#", 1)[0].strip()
        if not name:
            continue
        if name not in LIBM:
            sys.exit(f"libm-allowed.txt:{number}: {name} is no <math.h> function")
        allowed.add(name)
    return allowed


LIBM_ALLOWED = libm_allowed()

# The family's SIMD header is the one place fused multiply-add is spelled,
# for its scalar backend, so that file alone may call fmaf whatever
# tools/libm-allowed.txt lists.
LIBM_BY_FILE = {os.path.join("src", "simd.h"): {"fmaf"}}


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


EXTERNAL = external_dirs()


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


def walk(top, suffixes):
    for folder, dirs, files in os.walk(os.path.join(ROOT, top)):
        if os.path.relpath(folder, ROOT) in EXTERNAL:
            dirs[:] = []
            continue
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        for name in sorted(files):
            if name.endswith(suffixes):
                yield os.path.join(folder, name)


def check_c_file(path, rel, in_src, bans, findings):
    lines = open(path, encoding="utf-8", errors="replace").read().split("\n")
    if not lines[0].startswith("// SPDX-License-Identifier: MIT"):
        findings.append(f"{rel}:1: the file does not start with its SPDX line")
    if not SNAKE_CASE.match(os.path.basename(path)):
        findings.append(f"{rel}: file names are snake_case")
    for number, line in enumerate(lines, 1):
        if "/*" in code_text(line):
            findings.append(f"{rel}:{number}: use // comments, not /* */")
        comment = comment_text(line)
        if comment is not None:
            if MARKERS.search(comment):
                findings.append(f"{rel}:{number}: development-history marker: {line.strip()}")
            if TODO.search(comment):
                findings.append(f"{rel}:{number}: TODO or FIXME; open an issue instead")
        if not in_src:
            continue
        code = code_text(line)
        for name in re.findall(r"(?<![\w.>])([A-Za-z_]\w*)\s*\(", code):
            if name in ALLOCATOR_CALLS and os.path.basename(path) == ALLOCATOR_FILE:
                continue
            reason = bans.get(name)
            if reason:
                findings.append(f"{rel}:{number}: banned call {name}(): {reason}")
            elif (LIBM_ALLOWED is not None and name in LIBM and name not in LIBM_ALLOWED
                  and name not in LIBM_BY_FILE.get(rel, ())):
                findings.append(
                    f"{rel}:{number}: libm call {name}() not in tools/libm-allowed.txt:"
                    " its results may differ between C libraries"
                )
        if rel.replace(os.sep, "/") == FLOAT_ENV_FILE:
            pass
        elif THREAD_LOCAL.search(code):
            findings.append(f"{rel}:{number}: no thread-local state; failures are returned")
        elif FILE_SCOPE_STATE.match(code):
            findings.append(f"{rel}:{number}: mutable file-scope state: {line.strip()}")
        if ASSIGN_IN_CONDITION.search(code):
            findings.append(f"{rel}:{number}: assignment inside a condition: {line.strip()}")


def main():
    findings = []
    bans = dict(FAMILY_BANS)
    bans.update(library_bans())
    library = tuple(os.path.join(top, "src") + os.sep for top in SOURCE_DIRS) + ("src" + os.sep,)
    for top in C_DIRS + [top for top in SOURCE_DIRS if top not in C_DIRS]:
        for path in walk(top, (".c", ".h", ".m")):
            rel = os.path.relpath(path, ROOT)
            check_c_file(path, rel, rel.startswith(library), bans, findings)
    for finding in findings:
        print(finding)
    if findings:
        print(f"{len(findings)} finding(s); each line says which rule")
        return 1
    print("source rules: all files pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
