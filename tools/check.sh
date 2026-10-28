#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The checks, in three tiers. Each tier includes the one before it.
#
#   tools/check.sh changed   the files changed against origin/main: the
#                            family rules, test hygiene, formatting, the
#                            documentation checks, and the
#                            linux-clang-20-asan preset (scalar backend,
#                            assertions and world validation on) for the
#                            test labels the changes reach. Every
#                            preset's tests must carry a label. Under two
#                            minutes; run it before every push.
#   tools/check.sh pr        what the pull request check runs on this
#                            machine: every check, clang-tidy with the
#                            assertions compiled out and live, the
#                            documentation site, and the linux-clang-20-asan,
#                            linux-gcc-14 and linux-shared presets.
#   tools/check.sh full      also the other Linux presets (ThreadSanitizer,
#                            Clang Release), the testbed, the nightly
#                            simulation, and each fuzz target for
#                            FUZZ_SECONDS (default 20).
#
# The builds are the CI cells' CMake presets (CMakePresets.json), in
# out/<preset>. Needs gcc-14, clang-20, clang-tidy-20, ninja, python3
# with markdown, and the clang-format version CI pins: the organization's
# CI image has them all. The organization's documentation check runs when its .github
# checkout is at ORG_DEFAULTS (default: ../../.github from the repository).
set -euo pipefail

tier=${1:-changed}
case "$tier" in
changed | pr | full) ;;
*)
    echo "usage: tools/check.sh [changed|pr|full]" >&2
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."
root=$PWD
out=$root/out
mkdir -p "$out"
org=${ORG_DEFAULTS:-$root/../../.github}
jobs=${JOBS:-8}

step() { printf '\n== %s\n' "$*"; }

# A CI cell: its configure, build and test presets. More ctest arguments
# (a label filter) follow the preset's name.
preset() {
    local name=$1
    shift
    step "preset $name"
    cmake --preset "$name" >"$out/$name.log"
    python3 tools/check_test_hygiene.py --ctest "out/$name"
    cmake --build --preset "$name" -j "$jobs" >>"$out/$name.log"
    ctest --preset "$name" -j "$jobs" "$@"
}

# The ctest labels a changed path reaches; "all" when it reaches every
# suite (the library's sources, the build, the golden values).
labels_for() {
    case "$1" in
    *.md | LICENSE | .github/* | tools/*.py | tools/*.txt | tools/check.sh | tools/test_hygiene/*) ;;
    test/test_harness.h | test/test_task_pool.h | test/walk.h | test/hashes.txt) echo all ;;
    test/test_*.c)
        name=${1#test/test_}
        echo "${name%.c} golden"
        ;;
    bench/scenes.*) echo golden ;;
    bench/* | samples/* | testbed/*) ;;
    *) echo all ;;
    esac
}

source_checks() {
    step "family rules"
    python3 tools/check_source.py
    python3 tools/check_modules.py
    python3 tools/check_docs.py
    python3 tools/check_lengths.py
    python3 tools/check_guide.py
    python3 tools/check_float_env.py
    python3 tools/check_records.py --self-test
    python3 tools/check_records.py
    python3 tools/check_patch_results.py --self-test
    python3 tools/check_patch_results.py
    python3 tools/check_test_hygiene.py --self-test
    python3 tools/check_test_hygiene.py
    if [ -f "$org/scripts/docs_check.py" ]; then
        python3 "$org/scripts/docs_check.py"
    else
        echo "skipped the organization's documentation check: no checkout at $org"
    fi
}

all_c_files() {
    git ls-files 'include/*.h' 'src/*.[ch]' 'test/*.[ch]' 'tools/*.[ch]' 'bench/*.[ch]' 'samples/*.[ch]' \
        'testbed/*.[ch]'
}

if [ "$tier" = changed ]; then
    base=$(git merge-base HEAD origin/main 2>/dev/null || echo HEAD)
    mapfile -t changed < <({
        git diff --name-only "$base"
        git ls-files --others --exclude-standard
    } | sort -u)
    if [ ${#changed[@]} -eq 0 ]; then
        echo "nothing changed against origin/main"
        exit 0
    fi
    source_checks
    mapfile -t c_files < <(printf '%s\n' "${changed[@]}" | grep -E '\.[ch]$' || true)
    present=()
    for f in "${c_files[@]+"${c_files[@]}"}"; do [ -f "$f" ] && present+=("$f"); done
    if [ ${#present[@]} -gt 0 ]; then
        step "clang-format on ${#present[@]} changed files"
        clang-format --dry-run -Werror "${present[@]}"
    fi
    labels=""
    for f in "${changed[@]}"; do labels="$labels $(labels_for "$f")"; done
    if [ -z "${labels// /}" ]; then
        echo "no test reached by the changes"
        exit 0
    fi
    if [[ " $labels " == *" all "* ]]; then
        preset linux-clang-20-asan
    else
        regex=$(printf '%s\n' $labels | sort -u | paste -sd '|')
        echo "tests labeled $regex"
        preset linux-clang-20-asan -L "^($regex)\$"
    fi
    exit 0
fi

source_checks

step "clang-format on every file"
all_c_files | xargs clang-format --dry-run -Werror

# As CI's checks job: on the Release preset, where the assertions compile
# out, and with them live. Each hides findings the other shows.
for build in Release Debug; do
    step "clang-tidy, $build"
    cmake --preset linux-clang-20 -B "out/check-tidy-$build" -DCMAKE_BUILD_TYPE=$build \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMAUL2D_BUILD_TESTS=OFF -DMAUL2D_BUILD_BENCH=OFF \
        -DMAUL2D_BUILD_SAMPLES=OFF >/dev/null
    git ls-files 'src/*.c' | xargs -P "$jobs" -n 1 clang-tidy-20 -p "out/check-tidy-$build" --quiet \
        --warnings-as-errors='*'
done

step "the documentation site builds"
python3 tools/build_docs.py

preset linux-clang-20-asan
preset linux-gcc-14
preset linux-shared

[ "$tier" = full ] || exit 0

preset linux-clang-20-tsan
preset linux-clang-20

step "the testbed builds"
cmake --preset linux-testbed >"$out/linux-testbed.log"
cmake --build --preset linux-testbed -j "$jobs" >>"$out/linux-testbed.log"

step "the nightly simulation"
"$out/linux-clang-20/test_soak" 300000
"$out/linux-clang-20/test_walk" 200

step "the fuzz targets"
cmake -B "$out/fuzz" -G Ninja -DCMAKE_C_COMPILER=clang-20 -DMAUL2D_FUZZ=ON \
    -DMAUL2D_BUILD_TESTS=OFF -DMAUL2D_WERROR=ON >"$out/fuzz.log"
cmake --build "$out/fuzz" -j "$jobs" >>"$out/fuzz.log"
for target in "$out"/fuzz/fuzz_*; do
    name=$(basename "$target")
    mkdir -p "$out/corpus/$name"
    OUT="$out" "$root/tools/fuzz_target.sh" "$target" -max_total_time="${FUZZ_SECONDS:-20}" -max_len=4096 \
        "$out/corpus/$name" ||
        { cat "$out/fuzz-reports/$name.txt"; echo "log: $out/fuzz-logs/$name.log"; exit 1; }
done

step "all checks passed"
