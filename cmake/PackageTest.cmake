# The package test (MAUL2D_TEST_PACKAGE): installs the build into
# BINARY_DIR/package-test/prefix, then configures, builds and runs
# samples/minimal against it with nothing else on its search path, and on
# Linux checks that pkg-config finds the package. Run by CTest as the test
# "package" in the cells whose platforms the release ships; the build must
# be complete. With a multi-config generator (Visual Studio) the consumer
# builds with the generator's own compiler, as a program using the
# released library would.
#
#   cmake -DBINARY_DIR=... -DSOURCE_DIR=... -DGENERATOR=... -DC_COMPILER=... [-DCONFIG=...]
#         -P PackageTest.cmake

cmake_minimum_required(VERSION 3.25)

set(root ${BINARY_DIR}/package-test)
file(REMOVE_RECURSE ${root})

function(run)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        list(JOIN ARGN " " command)
        message(FATAL_ERROR "failed (${result}): ${command}")
    endif()
endfunction()

set(config_args "")
if(CONFIG)
    set(config_args --config ${CONFIG})
endif()
set(compiler_args "")
if(NOT GENERATOR MATCHES "^Visual Studio")
    set(compiler_args -DCMAKE_C_COMPILER=${C_COMPILER})
endif()

run(${CMAKE_COMMAND} --install ${BINARY_DIR} --prefix ${root}/prefix ${config_args})
run(${CMAKE_COMMAND} -S ${SOURCE_DIR}/samples/minimal -B ${root}/build -G ${GENERATOR}
    ${compiler_args} -DCMAKE_PREFIX_PATH=${root}/prefix)
run(${CMAKE_COMMAND} --build ${root}/build ${config_args})
if(CONFIG AND EXISTS ${root}/build/${CONFIG}/minimal.exe)
    set(program ${root}/build/${CONFIG}/minimal.exe)
elseif(EXISTS ${root}/build/minimal.exe)
    set(program ${root}/build/minimal.exe)
else()
    set(program ${root}/build/minimal)
endif()
run(${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${root}/prefix/lib:${root}/prefix/lib64" ${program})

if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    find_program(pkg_config pkg-config REQUIRED)
    file(GLOB pc_dirs ${root}/prefix/lib*/pkgconfig)
    list(JOIN pc_dirs ":" pc_path)
    run(${CMAKE_COMMAND} -E env "PKG_CONFIG_PATH=${pc_path}" ${pkg_config} --exists --print-errors
        maul2d)
endif()
