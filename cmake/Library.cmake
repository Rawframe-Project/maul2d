# maul_configure_library(target) finishes the engine library target:
# the namespaced alias, the include directories, the export macro for
# shared builds, hidden visibility, the shared-library version, libm and
# the compiler settings.

function(maul_configure_library target)
    add_library(${target}::${target} ALIAS ${target})
    target_include_directories(${target}
        PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE ${PROJECT_SOURCE_DIR}/src)
    set_target_properties(${target} PROPERTIES C_VISIBILITY_PRESET hidden)
    if(${MAUL_PREFIX}_BUILD_SHARED)
        # Before 1.0 every minor release may break the ABI, so the
        # SOVERSION carries major.minor; from 1.0 on the major alone.
        if(PROJECT_VERSION_MAJOR EQUAL 0)
            set(soversion ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
        else()
            set(soversion ${PROJECT_VERSION_MAJOR})
        endif()
        set_target_properties(${target} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${soversion})
        target_compile_definitions(${target} PUBLIC ${MAUL_PREFIX}_SHARED)
    endif()
    if(NOT MSVC)
        # sqrt and floor are the only libm functions the engine calls.
        target_link_libraries(${target} PUBLIC m)
    endif()
    maul_apply_flags(${target})
endfunction()
