# The public headers alone, compiled by whichever compiler configures
# this project: Windows CI configures it with Visual Studio's
# default toolset, so MSVC's cl, which cannot build the C23 library,
# still compiles every header as C17 and C++17 with /W4 /WX.
#
#   cmake -S cmake/HeaderCheck -B build-headers
#   cmake --build build-headers --config Release
cmake_minimum_required(VERSION 3.25)
project(maul_header_check LANGUAGES C)

get_filename_component(maul_root ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
file(GLOB maul_libraries LIST_DIRECTORIES true RELATIVE ${maul_root}/include
     ${maul_root}/include/*)
list(FILTER maul_libraries INCLUDE REGEX "^maul")
list(LENGTH maul_libraries maul_count)
if(NOT maul_count EQUAL 1)
    message(FATAL_ERROR "expected one include/maul* directory, found: ${maul_libraries}")
endif()

include(${maul_root}/cmake/Testing.cmake)
maul_add_header_tests(ROOT ${maul_root} NAME ${maul_libraries})
