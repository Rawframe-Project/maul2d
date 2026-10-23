# The benchmarks: never a CTest gate, because wall time is not
# repeatable. They print their timings.
add_executable(${PROJECT_NAME}_bench bench_main.c)
target_link_libraries(${PROJECT_NAME}_bench PRIVATE ${PROJECT_NAME})
maul_apply_flags(${PROJECT_NAME}_bench)

# A library's further benchmarks, if it has any, in bench/Local.cmake,
# so this file stays the same in every library.
include(${CMAKE_CURRENT_SOURCE_DIR}/Local.cmake OPTIONAL)
