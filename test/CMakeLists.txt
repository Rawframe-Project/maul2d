# Test suites. See cmake/Testing.cmake for the flags.
maul_add_test(core WHITEBOX)
maul_add_test(world)
maul_add_test(draw)
maul_add_test(broadphase)
maul_add_test(ccd)
maul_add_test(contacts)
maul_add_test(events)
maul_add_test(fluid THREADS)
maul_add_test(islands)
maul_add_test(joints)
maul_add_test(journal)
maul_add_test(queries)
maul_add_test(hostile)
maul_add_test(shapes)
maul_add_test(solver)
maul_add_test(threads THREADS)
maul_add_test(walk THREADS)
maul_add_test(golden THREADS)
maul_add_test(concurrency POSIX)
maul_add_test(soak MANUAL)
maul_add_header_tests()

# The golden scenes check the benchmark scenes too, and take their
# committed values from test/hashes.txt: each line M2_<NAME>_HASH=<hex>
# becomes the definition M2_GOLDEN_<NAME>.
target_sources(test_golden PRIVATE ${PROJECT_SOURCE_DIR}/bench/scenes.c)
file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/hashes.txt golden_lines REGEX "^M2_[A-Z0-9_]+_HASH=[0-9a-f]+$")
foreach(line IN LISTS golden_lines)
    string(REGEX REPLACE "^M2_([A-Z0-9_]+)_HASH=([0-9a-f]+)$" "M2_GOLDEN_\\1=0x\\2ull" definition
           "${line}")
    target_compile_definitions(test_golden PRIVATE ${definition})
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/hashes.txt)
