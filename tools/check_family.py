#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The family check (docs/adr/0004-shared-code-by-copy.md): the files in
# tools/family-files.txt must be byte for byte the sibling repository's,
# and the files in tools/family-renamed.txt must be the sibling's once
# the engine prefix is renamed (m2 and m3, M2 and M3, maul2d and
# maul3d become one name).
#
# usage: check_family.py SIBLING
#   SIBLING is a checkout of the other engine, or the base URL of its
#   raw files (https://raw.githubusercontent.com/OWNER/REPO/main).

import os
import re
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_sibling(base, path):
    if base.startswith("http://") or base.startswith("https://"):
        try:
            with urllib.request.urlopen(base.rstrip("/") + "/" + path) as response:
                return response.read()
        except OSError:
            return None
    full = os.path.join(base, path)
    if not os.path.exists(full):
        return None
    with open(full, "rb") as f:
        return f.read()


def neutral(text):
    """The text with the engine prefix renamed to one neutral name. Only
    whole prefixes change: _mm256 stays, m2Hash64 and M3_LANES do not."""
    text = re.sub(rb"(?<![A-Za-z0-9_])m[23](?=[A-Za-z_])", b"mN", text)
    text = re.sub(rb"(?<![A-Za-z0-9_])M[23](?=[A-Za-z_])", b"MN", text)
    for a, b in ((b"maul2d", b"maulNd"), (b"maul3d", b"maulNd"), (b"MAUL2D", b"MAULND"),
                 (b"MAUL3D", b"MAULND"), (b"Maul2D", b"MaulND"), (b"Maul3D", b"MaulND")):
        text = text.replace(a, b)
    return text


def listed(name):
    with open(os.path.join(ROOT, "tools", name)) as f:
        return [line.strip() for line in f if line.strip() and not line.startswith("#")]


def main():
    if len(sys.argv) != 2:
        print(__doc__ if __doc__ else "usage: check_family.py SIBLING")
        return 2
    base = sys.argv[1]
    failures = 0
    for renamed, name in ((False, "family-files.txt"), (True, "family-renamed.txt")):
        for path in listed(name):
            with open(os.path.join(ROOT, path), "rb") as f:
                ours = f.read()
            theirs = read_sibling(base, path)
            if theirs is None:
                print(f"missing in the sibling: {path}")
                failures += 1
            elif (neutral(ours) != neutral(theirs)) if renamed else (ours != theirs):
                print(f"differs from the sibling: {path}")
                failures += 1
    if failures == 0:
        print("family files in sync")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
