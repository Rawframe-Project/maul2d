#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Every public function that computes runs inside the library's
# floating-point environment (src/fpenv.h): its body enters, calls one
# worker and leaves, so no arithmetic of its own can move out of the entry.
# The functions that do no floating-point arithmetic are named one by one
# in tools/float-env-exempt.txt and are left as they are. Each public
# function the headers declare must be one or the other, and each name in
# the list must be a public function. A function a macro defines counts as
# wrapped when the macro's own body enters the environment.
#
#   python3 tools/check_float_env.py

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_exports  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXEMPT = os.path.join(ROOT, "tools", "float-env-exempt.txt")

DEFINITION = re.compile(r"^[A-Za-z_][\w \*]*?[\s\*](m2\w+)\(")
MACRO = re.compile(r"^#define\s+(M2_\w+)\(")
WRAPPER = [
    re.compile(r"^uint64_t env = m2FloatEnvEnter\(\)$"),
    re.compile(r"^(?:[\w \*]+ result = )?\w+Work(?:er)?\([\w, ]*\)$"),
    re.compile(r"^m2FloatEnvLeave\(env\)$"),
    re.compile(r"^return result$"),
]


def exempt():
    names = []
    with open(EXEMPT, encoding="utf-8") as f:
        for line in f:
            name = line.split("#", 1)[0].strip()
            if name:
                names.append(name)
    return names


def body_is_wrapper(body):
    """True when the body is enter, one worker call, leave and maybe a
    return of the worker's result."""
    text = " ".join(line.strip() for line in body).strip()
    if not (text.startswith("{") and text.endswith("}")):
        return False
    statements = [re.sub(r"\s+", " ", s).strip()
                  for s in text[1:-1].split(";")]
    statements = [s for s in statements if s]
    if len(statements) not in (3, 4):
        return False
    return all(WRAPPER[i].match(s) for i, s in enumerate(statements))


def definitions(public):
    """{name: 'wrapped' | 'plain'} for every public function defined in
    src/, directly or by a macro."""
    found = {}
    src = os.path.join(ROOT, "src")
    for name in sorted(os.listdir(src)):
        if not name.endswith(".c"):
            continue
        with open(os.path.join(src, name), encoding="utf-8") as f:
            lines = f.read().split("\n")
        macros = {}
        i = 0
        while i < len(lines):
            line = lines[i]
            macro = MACRO.match(line)
            if macro:
                text = []
                while lines[i].rstrip().endswith("\\"):
                    text.append(lines[i])
                    i += 1
                text.append(lines[i])
                macros[macro.group(1)] = "m2FloatEnvEnter()" in "\n".join(text)
                i += 1
                continue
            for macro_name, wraps in macros.items():
                if line.startswith(macro_name + "("):
                    for word in re.findall(r"\bm2\w+", line):
                        if word in public:
                            found[word] = "wrapped" if wraps else "plain"
            match = DEFINITION.match(line)
            if match and match.group(1) in public and not line.rstrip().endswith(";"):
                j = i
                while not lines[j].rstrip().endswith(")"):
                    j += 1
                if j + 1 < len(lines) and lines[j + 1].rstrip() == "{":
                    k = j + 1
                    while lines[k].rstrip() != "}":
                        k += 1
                    body = lines[j + 1:k + 1]
                    found[match.group(1)] = "wrapped" if body_is_wrapper(body) else "plain"
                    i = k
            i += 1
    return found


def main():
    public = check_exports.declared(())[0]
    names = exempt()
    findings = []
    for name in names:
        if name not in public:
            findings.append(f"tools/float-env-exempt.txt: {name} is no public function")
    if len(set(names)) != len(names):
        findings.append("tools/float-env-exempt.txt: a name is listed twice")
    found = definitions(public)
    for name in sorted(public):
        kind = found.get(name)
        if kind is None:
            findings.append(f"{name}: no definition found in src/")
        elif name in names and kind == "wrapped":
            findings.append(f"{name}: listed as exempt but enters the environment")
        elif name not in names and kind == "plain":
            findings.append(f"{name}: computes outside the floating-point environment; enter it"
                            " around a worker, or list it in tools/float-env-exempt.txt if it"
                            " does no float arithmetic")
    for finding in findings:
        print(finding)
    if findings:
        print(f"float environment: {len(findings)} finding(s)")
        return 1
    print(f"float environment: {len(public) - len(names)} public functions enter it,"
          f" {len(names)} exempt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
