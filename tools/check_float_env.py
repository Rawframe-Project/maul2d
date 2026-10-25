#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The floating-point environment rule: every public function enters the
# library's floating-point environment, so a host's flush-to-zero,
# denormals-are-zero or rounding mode never changes a result, unless it
# computes no floats and tools/float-env-allowed.txt names it.
#
# The public functions are those the headers in include/<library>/
# declare with the <MACRO>_API marker and the <prefix> name prefix, both
# read from CMakeLists.txt (MAUL_LIBRARY, MAUL_MACRO_PREFIX,
# MAUL_API_PREFIX). Each must be defined once in src/ (or a directory
# tools/source-dirs.txt lists), directly or by a macro of the source
# file. A body that enters the environment is a wrapper of one shape, so
# no arithmetic runs in the host's setting around the switch:
#
#     uint64_t env = <prefix>FloatEnvEnter();
#     [type result =] Worker(arguments);
#     <prefix>FloatEnvLeave(env);
#     [return result;]
#
# with any names; arguments that are names, addresses of names, pointer
# casts of names or plain constants, since even a conversion to a float
# type rounds in the host's setting; and a worker the sources declare
# <MACRO>_NOINLINE, so the compiler cannot move its arithmetic across
# the register writes. The allowed list gives names, never patterns, under the
# reason they need not enter: a reason is a line ending in a colon, its
# names indented below it:
#
#     reads a stored identity:
#         <prefix>ClipIdentity
#         <prefix>SkeletonIdentity
#
# so a new public function fails until someone decides. A listed name
# that is no public function, one listed twice, and one that enters the
# environment anyway are reported too, so the list stays true.
#
# usage: check_float_env.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ALLOWED = os.path.join(ROOT, "tools", "float-env-allowed.txt")


def cmake_setting(name):
    """A value the library's CMakeLists.txt sets with set(NAME value)."""
    with open(os.path.join(ROOT, "CMakeLists.txt"), encoding="utf-8") as f:
        match = re.search(r"^set\(" + name + r" \"?([^\")]+)\"?\)", f.read(), re.M)
    if not match:
        raise SystemExit("CMakeLists.txt does not set " + name)
    return match.group(1)


LIB = cmake_setting("MAUL_LIBRARY")
PREFIX = cmake_setting("MAUL_API_PREFIX")
MACRO = cmake_setting("MAUL_MACRO_PREFIX")

# What entering the environment looks like in a body: the family module's
# entry, or its run of a worker inside one, by name with the library's
# prefix.
GUARD = re.compile(r"\b" + PREFIX + r"FloatEnv(?:Enter|Run)\b")
ENTER = re.compile(r"(?:const )?uint64_t (\w+) = " + PREFIX + r"FloatEnvEnter\(\)")
CALL = re.compile(r"(?:(?:const )?[\w ]+?[\s*]+(\w+) = )?(\w+)\((.*)\)")
ARGUMENT = re.compile(r"&?\w+|\((?:const )?[\w ]+\*+\s*(?:const\s*)?\)\s*\w+|\d+[uUlLfF]*")


def statements(body):
    """The statements of a body, split at its top level, spaces folded."""
    result, depth, current = [], 0, ""
    for c in body.strip()[1:-1]:
        depth += {"(": 1, ")": -1, "{": 1, "}": -1}.get(c, 0)
        if c == ";" and depth == 0:
            result.append(current)
            current = ""
        else:
            current += c
    result.append(current)
    return [s for s in (re.sub(r"\s+", " ", s).strip() for s in result) if s]


def arguments(text):
    """A call's arguments, split at their top level; an empty one kept."""
    result, depth, current = [], 0, ""
    for c in text:
        depth += {"(": 1, ")": -1, "{": 1, "}": -1}.get(c, 0)
        if c == "," and depth == 0:
            result.append(current.strip())
            current = ""
        else:
            current += c
    result.append(current.strip())
    return result if any(result) else []


def wrapper_worker(body):
    """The worker a wrapper body calls, or None when the body is not of
    the wrapper's shape."""
    s = statements(body)
    enter = ENTER.fullmatch(s[0]) if s else None
    if not enter or len(s) not in (3, 4):
        return None
    call = CALL.fullmatch(s[1])
    if not call or not all(ARGUMENT.fullmatch(a) for a in arguments(call.group(3))):
        return None
    if s[2] != PREFIX + "FloatEnvLeave(" + enter.group(1) + ")":
        return None
    result = call.group(1)
    if len(s) == 3 and result is None:
        return call.group(2)
    if len(s) == 4 and result is not None and s[3] == "return " + result:
        return call.group(2)
    return None


def source_dirs():
    dirs = [os.path.join(ROOT, "src")]
    listed = os.path.join(ROOT, "tools", "source-dirs.txt")
    if os.path.exists(listed):
        with open(listed, encoding="utf-8") as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if line:
                    dirs.append(os.path.join(ROOT, line, "src"))
    return dirs


def strip_comments(text):
    """The text with comments and string literals blanked, lines kept."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end < 0 else end
            out.append(" " * (end - i))
            i = end
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            out.append("".join("\n" if c == "\n" else " " for c in text[i:end]))
            i = end
        elif text[i] in "\"'":
            quote = text[i]
            j = i + 1
            while j < n and text[j] != quote:
                j += 2 if text[j] == "\\" else 1
            out.append(quote + " " * (min(j, n) - i - 1) + (quote if j < n else ""))
            i = j + 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def public_functions():
    """Every function the public headers declare with the API marker."""
    folder = os.path.join(ROOT, "include", LIB)
    decl = re.compile(r"\b" + MACRO + r"_API\b[^;{(]*?\b(" + PREFIX + r"[A-Za-z0-9_]*)\s*\(")
    names = set()
    for name in sorted(os.listdir(folder)):
        if name.endswith(".h"):
            with open(os.path.join(folder, name), encoding="utf-8") as f:
                names.update(decl.findall(strip_comments(f.read())))
    return names


def expand_macros(text):
    """The text with each invocation, at the start of a line, of a
    function-like macro the same text defines moved to the end, as the
    macro's lines with the arguments put in and '##' pasted, so a public
    function a macro defines is found like any other. The invocation's
    lines stay as blank lines. Also each moved part's (start, end, line
    of the invocation)."""
    macros = {}
    for match in re.finditer(r"^#\s*define\s+(\w+)\(([^)]*)\)((?:[^\n]*\\\n)*[^\n]*)", text, re.M):
        params = [p.strip() for p in match.group(2).split(",") if p.strip()]
        lines = match.group(3).split("\\\n")
        macros[match.group(1)] = (params, "\n".join(line.strip() for line in lines))
    if not macros:
        return text, []
    out, moved, last = [], [], 0
    for match in re.finditer(r"^(" + "|".join(map(re.escape, macros)) + r")\(", text, re.M):
        if match.start() < last:
            continue
        depth, i = 0, match.end() - 1
        while i < len(text):
            depth += {"(": 1, ")": -1}.get(text[i], 0)
            i += 1
            if depth == 0:
                break
        params, body = macros[match.group(1)]
        values = dict(zip(params, arguments(text[match.end():i - 1])))
        if values:
            body = re.sub(r"\b(" + "|".join(map(re.escape, values)) + r")\b",
                          lambda m: values[m.group(1)], body)
        out.append(text[last:match.start()] + "\n" * text.count("\n", match.start(), i))
        moved.append((re.sub(r"\s*##\s*", "", body), text.count("\n", 0, match.start()) + 1))
        last = i
    out.append(text[last:])
    result = "".join(out)
    regions = []
    for body, line in moved:
        result += "\n"
        regions.append((len(result), len(result) + len(body), line))
        result += body
    return result, regions


def definitions(names):
    """Each public function's body, found where it is defined in a source
    file, directly or by a macro: its name at the start of a line's
    declarator, then its parameters and a brace. A list per name, to find
    one defined twice; and the text of every source and internal header,
    to find how a worker is declared."""
    found = {name: [] for name in names}
    every = []
    head = re.compile(r"^[A-Za-z_][\w \*]*?\b(" + PREFIX + r"\w*)\s*\(", re.M)
    for folder in source_dirs():
        for dirpath, _, files in os.walk(folder):
            for file in sorted(files):
                if not file.endswith((".c", ".m", ".h")):
                    continue
                path = os.path.join(dirpath, file)
                with open(path, encoding="utf-8") as f:
                    text = strip_comments(f.read())
                if file.endswith(".h"):
                    every.append(text)
                    continue
                text, regions = expand_macros(text)
                every.append(text)
                for match in head.finditer(text):
                    name = match.group(1)
                    if name not in found:
                        continue
                    # The parameters, then a body rather than a ';'.
                    depth = 0
                    i = match.end() - 1
                    while i < len(text):
                        depth += {"(": 1, ")": -1}.get(text[i], 0)
                        i += 1
                        if depth == 0:
                            break
                    rest = text[i:].lstrip()
                    if not rest.startswith("{"):
                        continue
                    start = i + text[i:].index("{")
                    depth = 0
                    j = start
                    while j < len(text):
                        depth += {"{": 1, "}": -1}.get(text[j], 0)
                        j += 1
                        if depth == 0:
                            break
                    line = next((at for start, end, at in regions if start <= match.start() < end),
                                text.count("\n", 0, match.start()) + 1)
                    found[name].append((os.path.relpath(path, ROOT), line, text[start:j]))
    return found, "\n".join(every)


def allowed():
    """The allowed list: {name: (reason, line)}; problems in its form."""
    names = {}
    problems = []
    if not os.path.exists(ALLOWED):
        return names, problems
    where = os.path.relpath(ALLOWED, ROOT)
    reason = None
    with open(ALLOWED, encoding="utf-8") as f:
        for number, line in enumerate(f, 1):
            text = line.split("#", 1)[0].rstrip()
            if not text.strip():
                continue
            if not text[0].isspace():
                if not text.endswith(":") or not text[:-1].strip():
                    problems.append(f"{where}:{number}: a reason is a line ending in a colon")
                reason = text[:-1].strip()
            elif reason is None:
                problems.append(f"{where}:{number}: a name before any reason")
            elif not re.fullmatch(PREFIX + r"\w+", text.strip()):
                problems.append(f"{where}:{number}: {text.strip()} is not a name of the API")
            elif text.strip() in names:
                problems.append(f"{where}:{number}: {text.strip()} is listed twice")
            else:
                names[text.strip()] = (reason, number)
    return names, problems


def main():
    names = public_functions()
    bodies, sources = definitions(names)
    listed, problems = allowed()
    where = os.path.relpath(ALLOWED, ROOT)
    for name in sorted(names):
        places = bodies[name]
        if not places:
            problems.append(f"{name}: declared public, defined nowhere in the sources")
            continue
        if len(places) > 1:
            found = ", ".join(f"{p}:{l}" for p, l, _ in places)
            problems.append(f"{name}: defined more than once ({found})")
            continue
        path, line, body = places[0]
        guarded = GUARD.search(body) is not None
        worker = wrapper_worker(body) if guarded else None
        if guarded and worker is None:
            problems.append(f"{path}:{line}: {name} enters the environment but is not a wrapper:"
                            " enter, one call of a worker with plain arguments, leave, return")
        elif worker and not re.search(r"\b" + MACRO + r"_NOINLINE\b[^;{}]*\b" + worker + r"\s*\(",
                                      sources):
            problems.append(f"{path}:{line}: {name}'s worker {worker} is not declared {MACRO}_NOINLINE")
        if guarded and name in listed:
            problems.append(f"{path}:{line}: {name} enters the environment but {where}:"
                            f"{listed[name][1]} lists it")
        elif not guarded and name not in listed:
            problems.append(f"{path}:{line}: {name} does not enter the floating-point environment")
    for name, (_, number) in sorted(listed.items(), key=lambda item: item[1][1]):
        if name not in names:
            problems.append(f"{where}:{number}: {name} is no public function")
    for problem in problems:
        print(problem)
    print(f"float environment: {len(names)} public functions, {len(listed)} listed, "
          f"{len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
