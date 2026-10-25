#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Test hygiene, as the repository standard's Enforcement asks: a test is
# deterministic, hermetic and fast, and never skips unexplained. In the
# test sources (test/, and test/ beside each part tools/source-dirs.txt
# lists) it rejects:
#
# - sleep: sleep, usleep, nanosleep, Sleep, thrd_sleep and C++'s
#   sleep_for and sleep_until, which make a test slow and its result
#   depend on timing;
# - wall-clock: time(), gettimeofday, timespec_get, every CLOCK_REALTIME
#   clock and CLOCK_TAI, and the std::chrono wall clocks (system_clock,
#   utc_clock, tai_clock, gps_clock, file_clock, and high_resolution_clock,
#   which libstdc++ makes system_clock; a monotonic clock is steady_clock),
#   whose answers differ from run to run;
# - retry: an identifier or label with a part retry, retries, attempt,
#   attempts or tries, or the parts try and again together (as in maxTries
#   or try_again, not entries, against, or again alone, which tests use
#   for a result computed a second time), the shape of a loop that runs an
#   assertion until it passes. A name of more than one part that a header
#   of the library declares (include/ and src/, and each listed part's) is
#   the library's own API, which a test may call, and does not count;
# - skip: 77 anywhere in a return or exit expression, a name with SKIP in
#   it anywhere in a return expression, and macros named like SKIP
#   (GTEST_SKIP), and in the CMake files the SKIP_RETURN_CODE,
#   SKIP_REGULAR_EXPRESSION and DISABLED test properties.
#
# A use that must stay carries "// hygiene: <reason>" on its line or the
# line before (in CMake, "# hygiene: <reason>"), in a comment, with a
# reason of at least three words. Comments and string literals, raw ones
# included, are not searched.
#
# With --ctest it also reads each configured build's tests from
# "ctest --show-only=json-v1" and rejects a test without a label, and a
# build that registers none. Each finding names the file and line, or the
# build and test, with its rule; any finding fails the check.
#
#   python3 tools/check_test_hygiene.py [--ctest BUILD_DIR ...]
#   python3 tools/check_test_hygiene.py --self-test
#
# --self-test runs the rules over tools/test_hygiene/, whose files mark
# each line that must be found with "expect: <rule>", and fails unless
# exactly those are found.

import argparse
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURES = os.path.join(ROOT, "tools", "test_hygiene")
C_SUFFIXES = (".c", ".h", ".cc", ".cpp", ".hpp", ".m", ".mm")
SKIP_DIRS = {".git", "out", "build", "_site", "node_modules"}

C_RULES = [
    ("sleep",
     re.compile(r"(?<![\w.>])(sleep|usleep|nanosleep|Sleep|thrd_sleep|sleep_for|sleep_until)"
                r"\s*\("),
     "a sleep makes the test slow and its result depend on timing"),
    ("wall-clock",
     re.compile(r"(?<![\w.>])(time|gettimeofday|timespec_get)\s*\(|\bCLOCK_REALTIME\w*|"
                r"\bCLOCK_TAI\b|"
                r"\b(system_clock|utc_clock|tai_clock|gps_clock|file_clock|high_resolution_clock)\b"),
     "the wall clock differs from run to run"),
    ("retry", None, "a test that passes on a later try hides a flaky one"),
    ("skip",
     re.compile(r"\b(exit|_Exit|quick_exit)\s*\([^;]*\b77\b|\breturn\b[^;]*\b77\b|"
                r"\breturn\b[^;]*\b\w*SKIP\w*\b|\b\w*SKIP\w*\s*\("),
     "a skipped test checks nothing"),
]
RETRY_WORDS = {"retry", "retries", "attempt", "attempts", "tries"}
IDENTIFIER = re.compile(r"\b[A-Za-z_]\w*")
WORD_PART = re.compile(r"[A-Z]+(?![a-z])|[A-Z]?[a-z]+|[0-9]+")
CMAKE_RULES = [
    ("skip", re.compile(r"\b(SKIP_RETURN_CODE|SKIP_REGULAR_EXPRESSION|DISABLED)\b"),
     "a skipped or disabled test checks nothing"),
]
MARKER = re.compile(r"\bhygiene:(.*)")
EXPECT = re.compile(r"\bexpect:\s*([\w-]+)")
RAW_PREFIX = re.compile(r"(?:^|[^\w])(?:u8|u|U|L)?R$")
NUMBER_TAIL = re.compile(r"(?:^|[^\w.])[0-9][\w.']*$")


def tokens(text):
    """The C or C++ text line by line as (code, comments): the code with
    comments and string and character literals blanked, raw strings
    included, and the text of each comment on the line."""
    codes, comments = [], []
    line, notes, note = [], [], None
    i = 0
    state = None  # None, "line", "block", '"' or "'"
    while i < len(text):
        c = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if c == "\n":
            if note is not None:
                notes.append("".join(note))
                note = [] if state == "block" else None
            codes.append("".join(line))
            comments.append(notes)
            line, notes = [], []
            if state in ("line", '"', "'"):
                state = None
        elif state is None:
            code = "".join(line)
            if c == "/" and nxt in "/*":
                state = "line" if nxt == "/" else "block"
                note = []
                i += 1
            elif c == '"' and RAW_PREFIX.search(code):
                # R"delim( ... )delim" may span lines; keep them.
                open_at = text.find("(", i + 1)
                delimiter = text[i + 1:open_at] if open_at != -1 else ""
                close_at = text.find(")" + delimiter + '"', open_at + 1) if open_at != -1 else -1
                stop = len(text) if close_at == -1 else close_at + len(delimiter) + 2
                for _ in range(text.count("\n", i, stop)):
                    codes.append("".join(line))
                    comments.append(notes)
                    line, notes = [], []
                line.append(" ")
                i = stop
                continue
            elif c == "'" and NUMBER_TAIL.search(code) and nxt.isalnum():
                pass  # a digit separator, as in 1'000
            elif c in "\"'":
                state = c
                line.append(" ")
            else:
                line.append(c)
        elif state in ("line", "block"):
            if state == "block" and c == "*" and nxt == "/":
                notes.append("".join(note))
                note = None
                state = None
                i += 1
            else:
                note.append(c)
        elif state in "\"'":
            if c == "\\":
                i += 1
            elif c == state:
                state = None
        i += 1
    if note is not None:
        notes.append("".join(note))
    codes.append("".join(line))
    comments.append(notes)
    return codes, comments


def cmake_tokens(raw):
    """A CMake file line by line as (code, comments): what precedes a #
    outside a quoted argument, and what follows it."""
    codes, comments = [], []
    for line in raw:
        quoted = False
        cut = len(line)
        i = 0
        while i < len(line):
            if line[i] == "\\":
                i += 1
            elif line[i] == '"':
                quoted = not quoted
            elif line[i] == "#" and not quoted:
                cut = i
                break
            i += 1
        codes.append(line[:cut])
        comments.append([line[cut + 1:]] if cut < len(line) else [])
    return codes, comments


def marked(notes):
    """Whether a comment on the line carries the marker with a reason of
    at least three words."""
    for note in notes:
        found = MARKER.search(note)
        if found and len(re.findall(r"\w+", found.group(1))) >= 3:
            return True
    return False


def retry_word(code, library):
    for identifier in IDENTIFIER.findall(code):
        parts = [p.lower() for piece in identifier.split("_") for p in WORD_PART.findall(piece)]
        if len(parts) > 1 and identifier in library:
            continue
        if RETRY_WORDS.intersection(parts) or {"try", "again"} <= set(parts):
            return True
    return False


def library_names(folders):
    """Every identifier in the code of the headers under the folders: the
    names the library declares, which its tests call."""
    names = set()
    for folder in folders:
        if os.path.isdir(folder):
            for path in walk(folder, lambda n: n.endswith((".h", ".hpp"))):
                codes, _ = tokens(open(path, encoding="utf-8", errors="replace").read())
                for code in codes:
                    names.update(IDENTIFIER.findall(code))
    return names


def scan_file(path, rel, findings, library=frozenset()):
    text = open(path, encoding="utf-8", errors="replace").read()
    if path.endswith(C_SUFFIXES):
        rules = C_RULES
        codes, comments = tokens(text)
    else:
        rules = CMAKE_RULES
        codes, comments = cmake_tokens(text.split("\n"))
    for index, code in enumerate(codes):
        for rule, pattern, reason in rules:
            hit = retry_word(code, library) if pattern is None else pattern.search(code)
            if not hit:
                continue
            if marked(comments[index]) or (index > 0 and marked(comments[index - 1])):
                continue
            findings.append((rel, index + 1, rule,
                             f"{rel}:{index + 1}: {rule}: {reason}; mark it"
                             " '// hygiene: <reason>' if it must stay"))


def walk(top, accept):
    for folder, dirs, files in os.walk(top):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS and not d.startswith("."))
        for name in sorted(files):
            if accept(name):
                yield os.path.join(folder, name)


def source_dirs():
    path = os.path.join(ROOT, "tools", "source-dirs.txt")
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return [line.split("#", 1)[0].strip() for line in f if line.split("#", 1)[0].strip()]


def external_dirs():
    path = os.path.join(ROOT, "tools", "external-dirs.txt")
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return [line.split("#", 1)[0].strip() for line in f if line.split("#", 1)[0].strip()]


def is_cmake(name):
    return name == "CMakeLists.txt" or name.endswith(".cmake")


def scan_tree(findings):
    parts = [""] + source_dirs()
    library = library_names([os.path.join(ROOT, d, sub) for d in parts for sub in ("include", "src")])
    for top in [os.path.join(d, "test") for d in parts]:
        folder = os.path.join(ROOT, top)
        if os.path.isdir(folder):
            for path in walk(folder, lambda n: n.endswith(C_SUFFIXES)):
                scan_file(path, os.path.relpath(path, ROOT), findings, library)
    skipped = [os.path.join(ROOT, d) for d in external_dirs()] + [FIXTURES]
    for path in walk(ROOT, is_cmake):
        if not any(path.startswith(d + os.sep) for d in skipped):
            scan_file(path, os.path.relpath(path, ROOT), findings)


def check_labels(name, data, findings):
    tests = data.get("tests", [])
    if not tests:
        findings.append((name, 0, "unlabeled-test", f"{name}: no test is registered"))
    for test in tests:
        labels = [p.get("value") for p in test.get("properties", []) if p.get("name") == "LABELS"]
        if not any(labels):
            findings.append((name, 0, "unlabeled-test",
                             f"{name}: test {test.get('name')} has no label; give it its suite's"))


def ctest_json(build):
    out = subprocess.run(["ctest", "--show-only=json-v1", "--test-dir", build], check=True,
                         capture_output=True, text=True).stdout
    return json.loads(out)


def self_test():
    findings = []
    expected = set()
    library = library_names([os.path.join(FIXTURES, "include")])
    for path in walk(FIXTURES, lambda n: n.endswith(C_SUFFIXES) or is_cmake(n)):
        rel = os.path.relpath(path, ROOT)
        scan_file(path, rel, findings, library)
        for number, line in enumerate(open(path, encoding="utf-8").read().split("\n"), 1):
            for rule in EXPECT.findall(line):
                expected.add((rel, number, rule))
    labels = os.path.join(FIXTURES, "ctest.json")
    with open(labels, encoding="utf-8") as f:
        check_labels("tools/test_hygiene/ctest.json", json.load(f), findings)
    check_labels("an empty build", {"tests": []}, findings)
    expected |= {("tools/test_hygiene/ctest.json", 0, "unlabeled-test"),
                 ("an empty build", 0, "unlabeled-test")}
    found = {(rel, line, rule) for rel, line, rule, _ in findings}
    wrong = False
    for item in sorted(expected - found):
        print(f"missed: {item[0]}:{item[1]}: {item[2]}")
        wrong = True
    for item in sorted(found - expected):
        print(f"unexpected: {item[0]}:{item[1]}: {item[2]}")
        wrong = True
    if wrong or not expected:
        return 1
    print(f"test hygiene self-test: the {len(expected)} expected findings, nothing else")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Checks test hygiene and test labels.")
    parser.add_argument("--ctest", action="append", default=[], metavar="BUILD_DIR",
                        help="a configured build whose tests must all carry a label")
    parser.add_argument("--self-test", action="store_true",
                        help="run the rules over tools/test_hygiene/")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    findings = []
    scan_tree(findings)
    for build in args.ctest:
        check_labels(build, ctest_json(build), findings)
    for finding in findings:
        print(finding[3])
    if findings:
        return 1
    builds = f", every test labeled in {len(args.ctest)} build(s)" if args.ctest else ""
    print(f"test hygiene: no sleep, wall clock, retry or skip{builds}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
