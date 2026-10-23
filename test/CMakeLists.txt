# Test suites. See cmake/Testing.cmake for the flags.
maul_add_test(core WHITEBOX)
maul_add_test(world)
maul_add_test(draw)
maul_add_test(broadphase WHITEBOX)
maul_add_test(ccd WHITEBOX)
maul_add_test(contacts WHITEBOX)
maul_add_test(distance WHITEBOX)
maul_add_test(events WHITEBOX)
maul_add_test(fluid WHITEBOX THREADS)
maul_add_test(islands WHITEBOX)
maul_add_test(joints WHITEBOX)
maul_add_test(journal WHITEBOX)
maul_add_test(queries WHITEBOX)
maul_add_test(redteam WHITEBOX)
maul_add_test(shapes WHITEBOX)
maul_add_test(solver WHITEBOX)
maul_add_test(threads WHITEBOX)
maul_add_test(fuzz THREADS)
maul_add_test(concurrency POSIX)
maul_add_test(perf MANUAL)
maul_add_test(soak MANUAL)
maul_add_header_tests()

# The determinism hashes the suites print, checked against test/hashes.txt
# in every cell (cmake/GoldenHashes.cmake).
set(golden_suites "")
foreach(name broadphase ccd contacts core events fluid fuzz islands joints journal queries redteam
        shapes solver threads world)
    if(TARGET test_${name})
        list(APPEND golden_suites $<TARGET_FILE:test_${name}>)
    endif()
endforeach()
if(MAUL2D_BUILD_SHARED)
    set(golden_all OFF)
else()
    set(golden_all ON)
endif()
add_test(NAME golden
         COMMAND ${CMAKE_COMMAND} "-DSUITES=${golden_suites}"
                 -DHASHES=${PROJECT_SOURCE_DIR}/test/hashes.txt
                 "-DEMULATOR=${CMAKE_CROSSCOMPILING_EMULATOR}" -DALL=${golden_all}
                 -P ${PROJECT_SOURCE_DIR}/cmake/GoldenHashes.cmake)
set_tests_properties(golden PROPERTIES LABELS golden)
