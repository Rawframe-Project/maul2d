# The golden check: runs each suite that prints determinism hashes and
# compares every M2_<NAME>_HASH= line it prints with test/hashes.txt, so
# each CI cell checks the same bits on its own. With ALL set, every line
# of test/hashes.txt must be printed too. Run by CTest as "golden".
#
#   cmake -DSUITES=a;b -DHASHES=... [-DEMULATOR=...] [-DALL=ON] -P GoldenHashes.cmake

cmake_minimum_required(VERSION 3.25)

file(STRINGS ${HASHES} expected REGEX "^M2_[A-Z0-9_]+_HASH=[0-9a-f]+$")
set(printed "")
foreach(suite IN LISTS SUITES)
    execute_process(COMMAND ${EMULATOR} ${suite} RESULT_VARIABLE result OUTPUT_VARIABLE output
                    ERROR_VARIABLE output)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${suite} failed (${result}):\n${output}")
    endif()
    string(REGEX MATCHALL "M2_[A-Z0-9_]+_HASH=[0-9a-f]+" lines "${output}")
    list(APPEND printed ${lines})
endforeach()
list(REMOVE_DUPLICATES printed)
if(NOT printed)
    message(FATAL_ERROR "no suite printed a hash")
endif()
set(wrong "")
foreach(line IN LISTS printed)
    if(NOT line IN_LIST expected)
        list(APPEND wrong ${line})
    endif()
endforeach()
if(wrong)
    list(JOIN wrong "\n  " wrong)
    message(FATAL_ERROR "hashes differ from test/hashes.txt:\n  ${wrong}")
endif()
if(ALL)
    foreach(line IN LISTS expected)
        if(NOT line IN_LIST printed)
            message(FATAL_ERROR "no suite printed ${line}")
        endif()
    endforeach()
endif()
list(LENGTH printed count)
message(STATUS "${count} hashes match test/hashes.txt")
