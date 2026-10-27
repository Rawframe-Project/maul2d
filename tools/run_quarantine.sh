#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Runs a build's quarantined tests and reports them without failing on
# their known flakes.
#
#   tools/run_quarantine.sh BUILD_DIR [CTEST ARGUMENT...]
#
# ctest runs by build directory, not by preset, so a preset's options do
# not apply: pass the ones the run needs after BUILD_DIR. Each test gets
# 300 seconds unless its TIMEOUT property or a later --timeout says
# otherwise, so a hung flake is reported instead of outlasting the job.
#
# Every test labeled quarantine runs. A quarantined test is one that
# fails now and then without a fault in the code, with an issue that
# tracks it, so its failure is reported, not fatal: each failed test gets
# a warning and a line in the job summary, and the script exits 0. A
# quarantined test that is skipped or not run, or a failure of ctest
# itself, still fails the step: neither is a known flake, and each would
# stop the nightly from counting the green nights a test needs to leave
# quarantine. So does a sanitizer report or a crash on a bad address: a
# known flake is timing or a daemon, never a memory error, and the report
# is in the log since ctest prints a failed test's output. A build with no
# quarantined test passes.
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: tools/run_quarantine.sh BUILD_DIR [CTEST ARGUMENT...]" >&2
  exit 2
fi
dir=$1
shift
log=$(mktemp)
trap 'rm -f "$log"' EXIT

status=0
ctest --test-dir "$dir" -L '^quarantine$' --output-on-failure --no-tests=ignore --timeout 300 "$@" |
  tee "$log" || status=$?

if grep -E '\*\*\*(Skipped|Not Run)' "$log"; then
  echo "::error::a quarantined test did not run"
  exit 1
fi
if grep -E '^==[0-9]+== *ERROR: |: runtime error: |^WARNING: ThreadSanitizer: |\*\*\*Exception: (SegFault|Bus error)' "$log"; then
  echo "::error::a quarantined test hit a sanitizer report or a crash"
  exit 1
fi
# ctest exits 8 when tests ran and some failed; anything else is ctest's
# own failure.
if [ "$status" -ne 0 ] && [ "$status" -ne 8 ]; then
  echo "::error::ctest failed with status $status"
  exit "$status"
fi

failed=$(sed -n '/^The following tests FAILED:/,$p' "$log" |
  sed -n 's/^[[:space:]]*[0-9]* - \([^ ]*\) .*/\1/p')
if [ "$status" -eq 8 ] && [ -z "$failed" ]; then
  echo "::error::ctest reported failed tests that could not be read from its output"
  exit 1
fi
summary=${GITHUB_STEP_SUMMARY:-/dev/null}
if [ -z "$failed" ]; then
  echo "Quarantined tests in $dir: all passed." >>"$summary"
  exit 0
fi
{
  echo "Quarantined tests that failed in $dir (known flakes; each has an issue):"
  echo
  for test in $failed; do
    echo "- \`$test\`"
  done
} >>"$summary"
for test in $failed; do
  echo "::warning::quarantined test $test failed"
done
exit 0
