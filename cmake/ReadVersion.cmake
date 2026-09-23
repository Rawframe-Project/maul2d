# Reads the library version from the public base header. The header is
# the single source of truth: CMake, the package version file,
# pkg-config and the code itself can never disagree.
function(maul_read_version header prefix out_var)
    file(STRINGS "${header}" lines REGEX "^#define ${prefix}_VERSION_(MAJOR|MINOR|PATCH) [0-9]+$")
    set(parts "")
    foreach(part MAJOR MINOR PATCH)
        string(REGEX MATCH "#define ${prefix}_VERSION_${part} ([0-9]+)" match "${lines}")
        if(NOT match)
            message(FATAL_ERROR "${header}: missing ${prefix}_VERSION_${part}")
        endif()
        list(APPEND parts "${CMAKE_MATCH_1}")
    endforeach()
    list(JOIN parts "." version)
    set(${out_var} "${version}" PARENT_SCOPE)
endfunction()
