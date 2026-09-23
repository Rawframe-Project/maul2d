# The benchmarks: never a CTest gate, because wall time is not
# deterministic. CI compares the hashes they print with bench/pins.txt.
add_executable(${PROJECT_NAME}_bench bench_main.c)
target_link_libraries(${PROJECT_NAME}_bench PRIVATE ${PROJECT_NAME})
maul_apply_flags(${PROJECT_NAME}_bench)
