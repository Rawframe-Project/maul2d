cmake_minimum_required(VERSION 3.22)

include(cmake/ReadVersion.cmake)
maul_read_version(${CMAKE_CURRENT_SOURCE_DIR}/include/maul2d/base.h M2 MAUL_VERSION)
project(maul2d
    VERSION ${MAUL_VERSION}
    DESCRIPTION "Deterministic 2D physics with bit-exact rollback and record/replay"
    LANGUAGES C)

# The family modules read their options through this prefix.
set(MAUL_PREFIX MAUL2D)

option(MAUL2D_BUILD_TESTS "Build the test suites" ${PROJECT_IS_TOP_LEVEL})
option(MAUL2D_BUILD_SAMPLES "Build the samples" ${PROJECT_IS_TOP_LEVEL})
option(MAUL2D_BUILD_BENCH "Build the benchmarks" ${PROJECT_IS_TOP_LEVEL})
option(MAUL2D_BUILD_TOOLS "Build the command-line tools" ${PROJECT_IS_TOP_LEVEL})
option(MAUL2D_BUILD_TESTBED "Build the interactive testbed (fetches raylib)" OFF)
option(MAUL2D_BUILD_SHARED "Build a shared library instead of a static one" OFF)
option(MAUL2D_INSTALL "Generate install rules" ${PROJECT_IS_TOP_LEVEL})
option(MAUL2D_WERROR "Treat compiler warnings as errors" OFF)
option(MAUL2D_SANITIZE "Instrument with AddressSanitizer and UBSan" OFF)
option(MAUL2D_TSAN "Instrument with ThreadSanitizer" OFF)
option(MAUL2D_COVERAGE "Instrument for gcov coverage" OFF)

# Engine-specific options.
set(MAUL2D_SIMD "auto" CACHE STRING "SIMD backend: auto or scalar")
set_property(CACHE MAUL2D_SIMD PROPERTY STRINGS auto scalar)
option(MAUL2D_VALIDATE "Validate the world after every step" OFF)

# Every executable (tests, samples, benchmarks, tools) lands in the top
# of the build tree, so scripts and CI find them in one place.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

include(GNUInstallDirs)
include(cmake/CompilerFlags.cmake)
include(cmake/Library.cmake)
include(cmake/Testing.cmake)

# x64 builds use AVX2 and FMA unless the portable scalar backend is
# chosen; every backend produces identical results.
if(MAUL2D_SIMD STREQUAL "scalar")
    list(APPEND MAUL_EXTRA_DEFINITIONS MAUL2D_SIMD_FORCE_SCALAR=1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    if(MSVC)
        set(MAUL_ARCH_FLAGS /arch:AVX2)
    else()
        set(MAUL_ARCH_FLAGS -mavx2 -mfma)
    endif()
    list(APPEND MAUL_EXTRA_DEFINITIONS MAUL2D_BUILT_FOR_AVX2=1)
endif()
if(MAUL2D_VALIDATE)
    list(APPEND MAUL_EXTRA_DEFINITIONS MAUL2D_VALIDATE=1)
endif()

if(MAUL2D_BUILD_SHARED)
    set(MAUL_LIBRARY_KIND SHARED)
else()
    set(MAUL_LIBRARY_KIND STATIC)
endif()
add_library(maul2d ${MAUL_LIBRARY_KIND}
    src/body.c
    src/broadphase.c
    src/buoyancy.c
    src/ccd.c
    src/chain.c
    src/contact.c
    src/contact_solver.c
    src/contact_solver_wide.c
    src/core.c
    src/destruction.c
    src/distance.c
    src/draw.c
    src/dynamic_tree.c
    src/geometry.c
    src/graph_color.c
    src/island.c
    src/joint.c
    src/joint_solver.c
    src/journal.c
    src/journal_replay.c
    src/manifold.c
    src/math_functions.c
    src/particle.c
    src/particle_solver.c
    src/platform_time.c
    src/query.c
    src/shape.c
    src/snapshot.c
    src/solver.c
    src/world.c
    src/world_state.c)
maul_configure_library(maul2d)

# The CPU check must run on CPUs the backend does not support, so it is
# compiled without the architecture flags and linked into the library.
add_library(maul2d_cpu OBJECT src/cpu.c)
set(maul_saved_arch_flags "${MAUL_ARCH_FLAGS}")
set(MAUL_ARCH_FLAGS "")
maul_apply_flags(maul2d_cpu)
set(MAUL_ARCH_FLAGS "${maul_saved_arch_flags}")
target_include_directories(maul2d_cpu PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
set_target_properties(maul2d_cpu PROPERTIES C_VISIBILITY_PRESET hidden POSITION_INDEPENDENT_CODE ON)
if(MAUL2D_BUILD_SHARED)
    target_compile_definitions(maul2d_cpu PRIVATE MAUL2D_SHARED maul2d_EXPORTS)
endif()
target_sources(maul2d PRIVATE $<TARGET_OBJECTS:maul2d_cpu>)

if(MAUL2D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
if(MAUL2D_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()
if(MAUL2D_BUILD_BENCH)
    add_subdirectory(bench)
endif()
if(MAUL2D_BUILD_TOOLS AND EXISTS ${PROJECT_SOURCE_DIR}/tools/CMakeLists.txt)
    add_subdirectory(tools)
endif()
if(MAUL2D_BUILD_TESTBED)
    add_subdirectory(testbed)
endif()
if(MAUL2D_INSTALL)
    include(cmake/Install.cmake)
endif()
