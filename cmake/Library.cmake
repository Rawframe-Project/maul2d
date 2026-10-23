# maul_configure_library(target) finishes a library target: the
# namespaced alias, the include directories, the export macro for shared
# builds, hidden visibility, the shared-library version and the compiler
# settings. A library that needs libm links it itself.

function(maul_configure_library target)
    add_library(${target}::${target} ALIAS ${target})
    target_include_directories(${target}
        PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE ${PROJECT_SOURCE_DIR}/src)
    set_target_properties(${target} PROPERTIES C_VISIBILITY_PRESET hidden)
    # The public headers are written in the common subset of C17 and
    # C++17; consumers inherit only that requirement.
    target_compile_features(${target} PUBLIC c_std_17)
    if(${MAUL_PREFIX}_BUILD_SHARED)
        # The SOVERSION names the releases that share an ABI, so a loader
        # never pairs a program with another: the whole version before 1.0,
        # major.minor from 1.0 on (a minor release may grow a def).
        if(PROJECT_VERSION_MAJOR EQUAL 0)
            set(soversion ${PROJECT_VERSION})
        else()
            set(soversion ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
        endif()
        set_target_properties(${target} PROPERTIES VERSION ${PROJECT_VERSION}
                              SOVERSION ${soversion})
        target_compile_definitions(${target} PUBLIC ${MAUL_PREFIX}_SHARED)
    endif()
    maul_apply_flags(${target})
endfunction()
