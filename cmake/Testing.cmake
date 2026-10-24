# maul_add_test(<name> [WHITEBOX] [THREADS] [POSIX] [MANUAL])
#
# Builds test/test_<name>.c into test_<name> and registers it with
# CTest as <name>, labeled <name>, so a check can run the suites a change
# reaches with ctest -L.
#   WHITEBOX  includes src/ and reaches internal symbols, which a shared
#             build hides, so the suite is skipped in shared builds.
#   THREADS   links the platform thread library.
#   POSIX     uses POSIX threads directly and is skipped on Windows.
#   MANUAL    is built but not registered: soaks and timing runs.

function(maul_add_test name)
    cmake_parse_arguments(ARG "WHITEBOX;THREADS;POSIX;MANUAL" "" "" ${ARGN})
    if(ARG_POSIX AND WIN32)
        return()
    endif()
    if(ARG_WHITEBOX AND ${MAUL_PREFIX}_BUILD_SHARED)
        return()
    endif()
    set(target test_${name})
    add_executable(${target} ${PROJECT_SOURCE_DIR}/test/test_${name}.c)
    target_link_libraries(${target} PRIVATE ${PROJECT_NAME})
    if(ARG_WHITEBOX)
        target_include_directories(${target} PRIVATE ${PROJECT_SOURCE_DIR}/src)
    endif()
    if(ARG_THREADS OR ARG_POSIX)
        find_package(Threads REQUIRED)
        target_link_libraries(${target} PRIVATE Threads::Threads)
    endif()
    maul_apply_flags(${target})
    if(NOT ARG_MANUAL)
        add_test(NAME ${name} COMMAND ${target})
        set_tests_properties(${name} PROPERTIES LABELS ${name})
    endif()
endfunction()

# maul_add_exports_test([EXCLUDE <header> ...] [PROGRAM <function> ...])
#
# In a Linux shared build, registers the exports test: the library
# exports the public functions and nothing else (tools/check_exports.py),
# so internal functions and tables stay hidden. EXCLUDE names the
# headers in include/<library>/ whose functions this build leaves out,
# such as another platform's adapter or a companion static library; the
# check fails if one of them is missing or exported. PROGRAM names the
# functions a header declares without the API macro because the program
# defines them; every other public function carries the macro. A shared
# build without Python fails to configure rather than pass without the
# check.
function(maul_add_exports_test)
    cmake_parse_arguments(ARG "" "" "EXCLUDE;PROGRAM" ${ARGN})
    if(${MAUL_PREFIX}_BUILD_SHARED AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        set(options)
        foreach(header IN LISTS ARG_EXCLUDE)
            list(APPEND options --exclude ${header})
        endforeach()
        foreach(function IN LISTS ARG_PROGRAM)
            list(APPEND options --program ${function})
        endforeach()
        add_test(NAME exports COMMAND Python3::Interpreter
                                      ${PROJECT_SOURCE_DIR}/tools/check_exports.py
                                      --nm ${CMAKE_NM} ${options} $<TARGET_FILE:${MAUL_LIBRARY}>)
        set_tests_properties(exports PROPERTIES LABELS exports)
    endif()
endfunction()

# maul_add_header_tests([ROOT <dir>] [NAME <library>])
#
# Compiles every public header on its own as C17, C23 and C++17, with
# pedantic warnings as errors, so the headers stay in the common subset
# the family requires. ROOT and NAME default to the project's
# source directory and name; cmake/HeaderCheck passes them to check a
# library's headers alone. MSVC's cl has no C23 mode (its /std:clatest is
# a draft mode), so the C23 check is skipped there. In C++17 the
# library's <PP>_NODISCARD, found in base.h, must not be empty. A macro,
# because enable_language must run at directory scope. Without a C++
# compiler the C++ checks are skipped.
macro(maul_add_header_tests)
    cmake_parse_arguments(MAUL_HT "" "ROOT;NAME" "" ${ARGN})
    if(NOT MAUL_HT_ROOT)
        set(MAUL_HT_ROOT ${PROJECT_SOURCE_DIR})
    endif()
    if(NOT MAUL_HT_NAME)
        set(MAUL_HT_NAME ${PROJECT_NAME})
    endif()
    include(CheckLanguage)
    check_language(CXX)
    if(CMAKE_CXX_COMPILER)
        enable_language(CXX)
    endif()
    file(GLOB maul_headers RELATIVE ${MAUL_HT_ROOT}/include
         ${MAUL_HT_ROOT}/include/${MAUL_HT_NAME}/*.h)
    set(maul_header_dir ${PROJECT_BINARY_DIR}/header_tests)
    set(maul_c17_sources "")
    set(maul_c23_sources "")
    set(maul_cxx_sources "")
    foreach(maul_header ${maul_headers})
        string(MAKE_C_IDENTIFIER ${maul_header} maul_stem)
        file(WRITE ${maul_header_dir}/c17_${maul_stem}.c "#include \"${maul_header}\"\n")
        file(WRITE ${maul_header_dir}/c23_${maul_stem}.c "#include \"${maul_header}\"\n")
        file(WRITE ${maul_header_dir}/cxx_${maul_stem}.cpp "#include \"${maul_header}\"\n")
        list(APPEND maul_c17_sources ${maul_header_dir}/c17_${maul_stem}.c)
        list(APPEND maul_c23_sources ${maul_header_dir}/c23_${maul_stem}.c)
        list(APPEND maul_cxx_sources ${maul_header_dir}/cxx_${maul_stem}.cpp)
    endforeach()
    file(STRINGS ${MAUL_HT_ROOT}/include/${MAUL_HT_NAME}/base.h maul_nodiscard
         REGEX "^#define [A-Z0-9_]+_NODISCARD \\[\\[nodiscard\\]\\]")
    if(maul_nodiscard)
        list(GET maul_nodiscard 0 maul_nodiscard)
        string(REGEX REPLACE "^#define ([A-Z0-9_]+_NODISCARD) .*" "\\1" maul_nodiscard
               "${maul_nodiscard}")
        file(WRITE ${maul_header_dir}/cxx_nodiscard.cpp
             "#include \"${MAUL_HT_NAME}/base.h\"\n"
             "#define MAUL_TEXT(x) MAUL_TEXT_OF(x)\n"
             "#define MAUL_TEXT_OF(x) #x\n"
             "static_assert(sizeof(MAUL_TEXT(${maul_nodiscard})) > 1,\n"
             "              \"${maul_nodiscard} is empty in C++17\");\n")
        list(APPEND maul_cxx_sources ${maul_header_dir}/cxx_nodiscard.cpp)
    endif()
    if(MSVC)
        set(maul_header_flags /W4 /WX)
    else()
        set(maul_header_flags -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_library(header_test_c17 OBJECT ${maul_c17_sources})
    set_target_properties(header_test_c17 PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    set(maul_header_targets header_test_c17)
    if(NOT CMAKE_C_COMPILER_ID STREQUAL "MSVC")
        add_library(header_test_c23 OBJECT ${maul_c23_sources})
        set_target_properties(header_test_c23 PROPERTIES C_STANDARD 23 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
        # clang-cl is asked for C23 directly, as maul_apply_flags does.
        if(MSVC)
            target_compile_options(header_test_c23 PRIVATE /clang:-std=c23)
        endif()
        list(APPEND maul_header_targets header_test_c23)
    endif()
    if(CMAKE_CXX_COMPILER)
        add_library(header_test_cxx17 OBJECT ${maul_cxx_sources})
        set_target_properties(header_test_cxx17 PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                           CXX_EXTENSIONS OFF)
        list(APPEND maul_header_targets header_test_cxx17)
    endif()
    foreach(maul_target ${maul_header_targets})
        target_include_directories(${maul_target} PRIVATE ${MAUL_HT_ROOT}/include)
        target_compile_options(${maul_target} PRIVATE ${maul_header_flags})
    endforeach()
endmacro()
