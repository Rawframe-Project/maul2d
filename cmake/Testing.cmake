# maul_add_test(<name> [WHITEBOX] [THREADS] [POSIX] [MANUAL])
#
# Builds test/test_<name>.c into test_<name> and registers it with
# CTest as <name>.
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
    endif()
endfunction()

# maul_add_cxx_header_test()
#
# Compiles the umbrella header as C++ and runs it, when a C++ compiler is
# available, so the public API stays usable from C++. A macro, because
# enable_language must run at directory scope.
macro(maul_add_cxx_header_test)
    include(CheckLanguage)
    check_language(CXX)
    if(CMAKE_CXX_COMPILER)
        enable_language(CXX)
        string(SUBSTRING ${PROJECT_NAME} 4 1 maul_dimension)
        set(maul_cxx_source ${PROJECT_BINARY_DIR}/cxx_headers.cpp)
        file(WRITE ${maul_cxx_source}
            "#include \"${PROJECT_NAME}/${PROJECT_NAME}.h\"\n"
            "int main() { return m${maul_dimension}GetVersion() > 0 ? 0 : 1; }\n")
        add_executable(test_cxx_headers ${maul_cxx_source})
        target_link_libraries(test_cxx_headers PRIVATE ${PROJECT_NAME})
        maul_apply_flags(test_cxx_headers)
        add_test(NAME cxx_headers COMMAND test_cxx_headers)
    endif()
endmacro()
