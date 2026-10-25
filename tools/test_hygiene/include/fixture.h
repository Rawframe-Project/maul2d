// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fixture for tools/check_test_hygiene.py --self-test: the library header
// whose names a test may use. A name of more than one part declared here
// is the library's API; a bare retry word is reported even here. It is
// read, never compiled.

int fixRetryDue(int attemptCount);
extern int retries; // expect: retry
