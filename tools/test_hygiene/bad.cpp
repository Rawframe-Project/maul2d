// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fixture for tools/check_test_hygiene.py --self-test, C++: each line
// marked "expect" must be reported with that rule, and no other line. It
// is read, never compiled.

#include <chrono>
#include <thread>

void Bad()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // expect: sleep
    const char* text = R"(a raw string
over two lines)";
    std::this_thread::sleep_until(std::chrono::steady_clock::now()); // expect: sleep
    auto now = std::chrono::system_clock::now();                     // expect: wall-clock
    auto utc = std::chrono::utc_clock::now();                        // expect: wall-clock
    auto high = std::chrono::high_resolution_clock::now();           // expect: wall-clock
    // A digit separator before a sleep on the same line must not hide it.
    int big = 1'000 + sleep(1); // expect: sleep
    (void)text;
    (void)now;
    (void)utc;
    (void)high;
    (void)big;
}
