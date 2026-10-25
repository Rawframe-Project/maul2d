// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fixture for tools/check_test_hygiene.py --self-test, C++: nothing here
// may be reported. It is read, never compiled.

#include <chrono>

void Good()
{
    // A raw string is not code, across lines and with quotes inside.
    const char* text = R"x(sleep(1); "time(nullptr)" )"
retry exit(77) )x";
    // Digit separators are not character literals.
    int big = 1'000'000;
    char c = 'x';
    auto start = std::chrono::steady_clock::now();
    (void)text;
    (void)big;
    (void)c;
    (void)start;
}
