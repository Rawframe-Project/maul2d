#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Runs one libFuzzer target and reports a finding without its input.
#
#   tools/fuzz_target.sh TARGET [LIBFUZZER ARGUMENT...]
#
# The target runs with an explicit seed, never 0 (which libFuzzer takes
# as "pick one"), and writes its findings to out/fuzz-crashes/NAME/ and
# its log to out/fuzz-logs/NAME.log; OUT overrides out. Both stay on the
# machine that ran it. Every run's log is public in CI, and libFuzzer
# prints a finding's input there as bytes, text and Base64, so on a
# finding this reports only the target, the kind of each finding (crash,
# leak, timeout, oom), the sanitizers' runtime error lines and the
# report from its "==PID== ERROR:" line to its "SUMMARY:" line, all
# taken from the log above libFuzzer's first input dump.
#
# Neither the seed nor a finding's file name is printed. A run that
# starts from the committed seeds builds from public sources, so its
# seed would let anyone replay the finding, and the file name is the
# SHA-1 of the input, which gives a short input away. Both stay in the
# log on the machine that ran it; the maintainers reproduce a finding by
# fuzzing the target locally.
#
# The report goes to out/fuzz-reports/NAME.txt, written fresh by each
# run, and the script prints an error line and exits 1; it exits 0 when
# the target found nothing. A job that runs targets side by side prints
# the reports once they have all finished, so no report interleaves
# with another at any length, and clears them first, so that only this
# step's targets report:
#
#   rm -rf out/fuzz-reports
#   status=0
#   printf '%s\n' out/fuzz/fuzz_* |
#     xargs -P 4 -I {} tools/fuzz_target.sh {} ... || status=$?
#   cat out/fuzz-reports/*.txt 2>/dev/null || true
#   exit "$status"
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: tools/fuzz_target.sh TARGET [LIBFUZZER ARGUMENT...]" >&2
  exit 2
fi
target=$1
shift
name=$(basename "$target")
out=${OUT:-out}
crashes="$out/fuzz-crashes/$name"
log="$out/fuzz-logs/$name.log"
report="$out/fuzz-reports/$name.txt"
stamp="$out/fuzz-logs/$name.start"
mkdir -p "$crashes" "$out/fuzz-logs" "$out/fuzz-reports"
rm -f "$report"
touch "$stamp"

seed=$(($(od -An -N4 -tu4 /dev/urandom) % 2147483647 + 1))
echo "seed $seed" >"$log"
if "$target" -seed="$seed" -artifact_prefix="$crashes/" "$@" >>"$log" 2>&1; then
  echo "$name: no finding"
  exit 0
fi

# Everything from the first mutation line on is libFuzzer's dump of the
# input; no line of it is read.
head=$(sed '/^MS: [0-9]/,$d' "$log")
{
  echo "$name:"
  # Only this run's findings, not those an earlier run left, by kind.
  find "$crashes" -maxdepth 1 -type f -newer "$stamp" -printf '%f\n' |
    sed 's/-.*//' | sort | uniq -c
  grep -E -m 5 '^[^ ]+:[0-9]+:[0-9]+: runtime error: ' <<<"$head" || true
  # head may close the pipe early on a long report; that is not an error.
  sed -n '/^==[0-9]*== *ERROR: /,/^SUMMARY: /p' <<<"$head" | head -n 80 || true
} >"$report"
echo "::error::$name found something"
exit 1
