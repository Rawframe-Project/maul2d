# The benchmark scenes live in their own file, so the golden test checks
# the same scenes the benchmark times.
target_sources(${PROJECT_NAME}_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/scenes.c)
