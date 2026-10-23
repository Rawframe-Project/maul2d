# The package test (MAUL2D_TEST_PACKAGE): installs the build into
# BINARY_DIR/package-test/prefix, then configures, builds and runs
# samples/minimal against it with nothing else on its search path. Run by
# CTest as the test "package"; the build must be complete.
#
#   cmake -DBINARY_DIR=... -DSOURCE_DIR=... -DGENERATOR=... -DC_COMPILER=... -P PackageTest.cmake

set(root ${BINARY_DIR}/package-test)
file(REMOVE_RECURSE ${root})

function(run)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        list(JOIN ARGN " " command)
        message(FATAL_ERROR "failed (${result}): ${command}")
    endif()
endfunction()

run(${CMAKE_COMMAND} --install ${BINARY_DIR} --prefix ${root}/prefix)
run(${CMAKE_COMMAND} -S ${SOURCE_DIR}/samples/minimal -B ${root}/build -G ${GENERATOR}
    -DCMAKE_C_COMPILER=${C_COMPILER} -DCMAKE_PREFIX_PATH=${root}/prefix)
run(${CMAKE_COMMAND} --build ${root}/build)
run(${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${root}/prefix/lib:${root}/prefix/lib64"
    ${root}/build/minimal)
