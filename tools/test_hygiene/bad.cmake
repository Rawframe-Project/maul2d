# Fixture for tools/check_test_hygiene.py --self-test, never included.
set_tests_properties(slow PROPERTIES SKIP_RETURN_CODE 77) # expect: skip
set_tests_properties(old PROPERTIES DISABLED ON) # expect: skip
# hygiene: the fixture shows a marked property on the line after
set_tests_properties(marked PROPERTIES SKIP_REGULAR_EXPRESSION "absent")
