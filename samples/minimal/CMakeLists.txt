# A separate project that knows nothing about the maul2d source tree
# and consumes only the installed package. CI installs the library and
# builds this; if it builds, links and runs, the package works.
cmake_minimum_required(VERSION 3.22)
project(maul2d_minimal LANGUAGES C)

find_package(maul2d REQUIRED CONFIG)

add_executable(minimal main.c)
target_link_libraries(minimal PRIVATE maul2d::maul2d)
# The public headers are C17, so a consumer may stay on C17.
set_target_properties(minimal PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
