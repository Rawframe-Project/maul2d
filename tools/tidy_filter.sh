#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Prints clang-tidy's --header-filter for a build tree: this library's
# headers under src/ and include/, anchored at the source directory the
# tree was configured from. CMake writes the include paths through that
# directory as it was given, so in a checkout reached through a symbolic
# link neither pwd nor pwd -P need match them; the tree's
# CMAKE_HOME_DIRECTORY does. The directory is escaped, so that a dot or
# a plus in it matches only itself.
#
# A library whose own sources also sit in subdirectories with their own
# src/ and include/ (such as rhi/ or window/) names them after the tree,
# and their headers are kept too.
#
# Assign the output to a variable of its own before passing it on, so
# that a failure here stops the caller instead of leaving an empty
# filter:
#
#   filter=$(tools/tidy_filter.sh out/tidy)
#   clang-tidy-20 -p out/tidy --header-filter="$filter" ...
#
# usage: tidy_filter.sh <build tree> [subdirectory ...]
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "usage: tidy_filter.sh <build tree> [subdirectory ...]" >&2
    exit 2
fi

escape() {
    printf '%s' "$1" | sed 's/[][\\.^$*+?(){}|]/\\&/g'
}

home=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$1/CMakeCache.txt")
if [ -z "$home" ]; then
    echo "no CMAKE_HOME_DIRECTORY in $1/CMakeCache.txt" >&2
    exit 1
fi
shift

prefix=""
if [ $# -gt 0 ]; then
    names=""
    for dir in "$@"; do
        names="${names:+$names|}$(escape "$dir")"
    done
    prefix="(($names)/)?"
fi
printf '^%s/%s(src|include)/\n' "$(escape "$home")" "$prefix"
