#!/usr/bin/env python3
# Compares the determinism hashes printed by the test suites with the
# values committed in test/hashes.txt. CI already requires every
# platform cell to print the same hashes; this also catches a change
# that moves a hash the same way everywhere.
#
# Usage:
#   python3 tools/check_hashes.py <build-dir>     run ctest there and compare
#   python3 tools/check_hashes.py --file <log>    compare a saved test log
#   python3 tools/check_hashes.py <build-dir> --update   rewrite test/hashes.txt

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOLDEN = os.path.join(ROOT, "test", "hashes.txt")
PATTERN = re.compile(r"\bM[23]_[A-Z0-9_]+_HASH=[0-9a-f]{16}\b")


def hashes_in(text):
    return sorted(set(PATTERN.findall(text)))


def main(argv):
    if len(argv) >= 2 and argv[0] == "--file":
        text = open(argv[1], encoding="utf-8", errors="replace").read()
    elif argv:
        run = subprocess.run(["ctest", "--test-dir", argv[0], "-V"], capture_output=True,
                             text=True)
        text = run.stdout
    else:
        print(__doc__ or "usage: check_hashes.py <build-dir> | --file <log>")
        return 2
    found = hashes_in(text)
    if "--update" in argv:
        open(GOLDEN, "w").write("\n".join(found) + "\n")
        print("wrote %d hashes to %s" % (len(found), GOLDEN))
        return 0
    expected = [line.strip() for line in open(GOLDEN) if line.strip()]
    missing = sorted(set(expected) - set(found))
    extra = sorted(set(found) - set(expected))
    for line in missing:
        print("expected, not printed: " + line)
    for line in extra:
        print("printed, not expected: " + line)
    if missing or extra:
        print("hashes differ from test/hashes.txt")
        return 1
    print("all %d hashes match test/hashes.txt" % len(expected))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
