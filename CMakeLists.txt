cmake_minimum_required(VERSION 3.22)

include(cmake/ReadVersion.cmake)
maul_read_version(${CMAKE_CURRENT_SOURCE_DIR}/include/maul2d/base.h M2 MAUL_VERSION)
project(maul2d
    VERSION ${MAUL_VERSION}
    DESCRIPTION "Deterministic 2D physics with bit-exact rollback and record/replay"
    LANGUAGES C)

# The family modules and tools read the library's names and option prefix.
set(MAUL_PREFIX MAUL2D)
set(MAUL_LIBRARY maul2d)
set(MAUL_API_PREFIX m2)
set(MAUL_MACRO_PREFIX M2)
set(MAUL_TITLE "Maul2D")

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
option(MAUL2D_TEST_PACKAGE "Test the installed package by building samples/minimal against it" OFF)

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
    src/allocator.c
    src/body.c
    src/body_motion.c
    src/body_read.c
    src/broadphase.c
    src/buoyancy.c
    src/ccd.c
    src/chain.c
    src/contact.c
    src/contact_solver.c
    src/contact_kernel.c
    src/core.c
    src/destruction.c
    src/distance.c
    src/draw.c
    src/dynamic_tree.c
    src/fpenv.c
    src/geometry.c
    src/graph_color.c
    src/hash.c
    src/island.c
    src/joint.c
    src/joint_read.c
    src/joint_row.c
    src/joint_solver.c
    src/joint_distance.c
    src/joint_edges.c
    src/joint_filter.c
    src/joint_gear.c
    src/joint_motor.c
    src/joint_mouse.c
    src/joint_prismatic.c
    src/joint_pulley.c
    src/joint_ratchet.c
    src/joint_revolute.c
    src/joint_weld.c
    src/joint_wheel.c
    src/journal.c
    src/journal_replay.c
    src/manifold.c
    src/mass.c
    src/math_functions.c
    src/mover.c
    src/object_ids.c
    src/parallel.c
    src/particle.c
    src/particle_contacts.c
    src/particle_pairs.c
    src/particle_solver.c
    src/platform_time.c
    src/query.c
    src/query_proxy.c
    src/ray_cast.c
    src/shape.c
    src/shape_read.c
    src/snapshot.c
    src/snapshot_check.c
    src/solver.c
    src/world.c
    src/world_validate.c
    src/world_hash.c
    src/world_internal.c
    src/world_state.c)
maul_configure_library(maul2d)
if(NOT MSVC)
    # sqrt and floor are the only libm functions the library calls.
    target_link_libraries(maul2d PUBLIC m)
    set(MAUL_PKG_LIBS_PRIVATE "-lm")
endif()

# The CPU check must run on CPUs the backend does not support, so it is
# compiled without the architecture flags and linked into the library.
# The public header comes first, so its export marks reach the
# definitions in the family's cpu.c.
add_library(maul2d_cpu OBJECT src/cpu.c)
set(maul_saved_arch_flags "${MAUL_ARCH_FLAGS}")
set(MAUL_ARCH_FLAGS "")
maul_apply_flags(maul2d_cpu)
set(MAUL_ARCH_FLAGS "${maul_saved_arch_flags}")
target_include_directories(maul2d_cpu PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
if(MSVC)
    target_compile_options(maul2d_cpu PRIVATE /FImaul2d/base.h)
else()
    target_compile_options(maul2d_cpu PRIVATE -include maul2d/base.h)
endif()
set_target_properties(maul2d_cpu PROPERTIES C_VISIBILITY_PRESET hidden POSITION_INDEPENDENT_CODE ON)
if(MAUL2D_BUILD_SHARED)
    target_compile_definitions(maul2d_cpu PRIVATE MAUL2D_SHARED maul2d_EXPORTS)
endif()
target_sources(maul2d PRIVATE $<TARGET_OBJECTS:maul2d_cpu>)

if(MAUL2D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
    # Installs the build into the binary tree and builds and runs
    # samples/minimal against it, as a consumer of the package would.
    if(MAUL2D_TEST_PACKAGE AND MAUL2D_INSTALL)
        add_test(NAME package
                 COMMAND ${CMAKE_COMMAND} -DBINARY_DIR=${PROJECT_BINARY_DIR}
                         -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DGENERATOR=${CMAKE_GENERATOR}
                         -DC_COMPILER=${CMAKE_C_COMPILER} -DCONFIG=$<CONFIG>
                         -P ${PROJECT_SOURCE_DIR}/cmake/PackageTest.cmake)
        set_tests_properties(package PROPERTIES LABELS package)
    endif()
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
