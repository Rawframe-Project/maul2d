# Install rules: the library, the public headers, an exported CMake
# package (find_package(<lib>) with target <lib>::<lib>) and a
# relocatable pkg-config file.

include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# A library whose parts build alone sets MAUL_PUBLIC_HEADERS, before
# including this file, to the headers the configured build has, as
# paths under include/<lib>/; without it every header installs.
if(DEFINED MAUL_PUBLIC_HEADERS)
    foreach(header IN LISTS MAUL_PUBLIC_HEADERS)
        get_filename_component(directory ${header} DIRECTORY)
        install(FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/${header}
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/${directory})
    endforeach()
else()
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
# Before 1.0 any release may break compatibility; from 1.0 on only a major
# release may.
if(PROJECT_VERSION_MAJOR EQUAL 0)
    set(compatibility ExactVersion)
else()
    set(compatibility SameMajorVersion)
endif()
write_basic_package_version_file(${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY ${compatibility})
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

if(${MAUL_PREFIX}_BUILD_SHARED)
    set(MAUL_PKG_CFLAGS "-D${MAUL_PREFIX}_SHARED")
else()
    set(MAUL_PKG_CFLAGS "")
endif()
# A library that links system libraries sets MAUL_PKG_LIBS_PRIVATE
# (for example "-lm"), and one that uses another package sets
# MAUL_PKG_REQUIRES_PRIVATE (for example "maul-unicode >= 0.2"), before
# including this file.
configure_file(${PROJECT_SOURCE_DIR}/cmake/pkg-config.pc.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
