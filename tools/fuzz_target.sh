#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Runs one libFuzzer target, or merges a corpus with it, and reports a
# finding without its input.
#
#   tools/fuzz_target.sh TARGET [LIBFUZZER ARGUMENT...]
#   tools/fuzz_target.sh --merge TARGET MERGED INPUT_DIR...
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
#
# --merge minimizes INPUT_DIR... into MERGED with libFuzzer's -merge=1,
# which runs every input again. Its log stays on the machine too, and it
# prints one line. libFuzzer's merge exits 0 even when an input crashes,
# so the script looks for a finding itself (a new file under
# out/fuzz-crashes/NAME/ or a sanitizer report in the log) and reports
# it as a run does, exiting 1, so that a corpus holding a finding is not
# saved.
#
# AddressSanitizer's register dump is turned off: registers may hold the
# input's bytes.
set -euo pipefail

merge=0
if [ "${1:-}" = --merge ]; then
  merge=1
  shift
fi
if [ $# -lt 1 ] || { [ "$merge" = 1 ] && [ $# -lt 3 ]; }; then
  echo "usage: tools/fuzz_target.sh TARGET [LIBFUZZER ARGUMENT...]" >&2
  echo "       tools/fuzz_target.sh --merge TARGET MERGED INPUT_DIR..." >&2
  exit 2
fi
export ASAN_OPTIONS="${ASAN_OPTIONS:+$ASAN_OPTIONS:}dump_registers=0"
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

if [ "$merge" = 1 ]; then
  log="$out/fuzz-logs/$name.merge.log"
  merged=$1
  shift
  mkdir -p "$merged"
  if "$target" -merge=1 -artifact_prefix="$crashes/" "$merged" "$@" >"$log" 2>&1 &&
    [ -z "$(find "$crashes" -maxdepth 1 -type f -newer "$stamp")" ] &&
    ! grep -q -E '^==[0-9]+== *ERROR: |: runtime error: ' "$log"; then
    echo "$name: merged into $(find "$merged" -maxdepth 1 -type f | wc -l) files"
    exit 0
  fi
else
  seed=$(($(od -An -N4 -tu4 /dev/urandom) % 2147483647 + 1))
  echo "seed $seed" >"$log"
  if "$target" -seed="$seed" -artifact_prefix="$crashes/" "$@" >>"$log" 2>&1; then
    echo "$name: no finding"
    exit 0
  fi
fi

# Everything from the first line of libFuzzer's dump of an input on (its
# mutation line, the bytes, where it was written, its Base64) is never
# read. A merge's dump has no mutation line, so each form ends the part
# that is read.
head=$(sed -E '/^(MS: [0-9]|0x[0-9a-fA-F]{1,2},|artifact_prefix=|Base64: )/,$d' "$log")
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
