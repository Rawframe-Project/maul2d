# Compiler settings for every target that holds engine code, tests,
# samples or tools. maul_apply_flags(target) sets C17, the flags the
# determinism contract depends on, the warning set and the options
# named ${MAUL_PREFIX}_WERROR, _SANITIZE, _TSAN and _COVERAGE. An
# engine adds architecture flags through MAUL_ARCH_FLAGS and
# definitions through MAUL_EXTRA_DEFINITIONS before calling it.

function(maul_apply_flags target)
    # Fast math anywhere in the global flags would void the determinism
    # contract, so configuration stops instead.
    foreach(config "" _DEBUG _RELEASE _RELWITHDEBINFO _MINSIZEREL)
        string(REGEX MATCH "fast-math|/fp:fast|-Ofast" fast "${CMAKE_C_FLAGS${config}}")
        if(fast)
            message(FATAL_ERROR "CMAKE_C_FLAGS${config} contains ${fast}, which the determinism contract forbids")
        endif()
    endforeach()

    set_target_properties(${target} PROPERTIES
        C_STANDARD 17
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF)

    if(MSVC)
        # Before Visual Studio 2022, /fp:precise still allowed contraction
        # into FMA and /fp:contract- did not exist to forbid it.
        if(MSVC_VERSION LESS 1930)
            message(FATAL_ERROR "MSVC 19.30 (Visual Studio 2022) or newer is required to turn off floating-point contraction")
        endif()
        # C4127 (conditional expression is constant) fires on sizeof
        # checks and the do-while(0) idiom; it is noise here.
        target_compile_options(${target} PRIVATE /W4 /wd4127 /fp:precise)
        # arm64 MSVC does not know /fp:contract-; there the pragma in
        # src/core.h turns contraction off instead.
        if(NOT CMAKE_C_COMPILER_ARCHITECTURE_ID MATCHES "ARM64")
            target_compile_options(${target} PRIVATE /fp:contract-)
        endif()
        if(${MAUL_PREFIX}_WERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE
            -ffp-contract=off -fno-trapping-math -fno-fast-math -fno-unsafe-math-optimizations
            -Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion
            $<$<COMPILE_LANGUAGE:C>:-Wmissing-prototypes>)
        if(${MAUL_PREFIX}_WERROR)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()

    if(MAUL_ARCH_FLAGS)
        target_compile_options(${target} PRIVATE ${MAUL_ARCH_FLAGS})
    endif()
    if(MAUL_EXTRA_DEFINITIONS)
        target_compile_definitions(${target} PRIVATE ${MAUL_EXTRA_DEFINITIONS})
    endif()

    if(NOT MSVC)
        if(${MAUL_PREFIX}_SANITIZE)
            target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all
                                                     -fno-omit-frame-pointer)
            target_link_options(${target} PRIVATE -fsanitize=address,undefined)
        endif()
        if(${MAUL_PREFIX}_TSAN)
            target_compile_options(${target} PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
            target_link_options(${target} PRIVATE -fsanitize=thread)
        endif()
        # Coverage only adds counters, so it rides on the normal flags.
        if(${MAUL_PREFIX}_COVERAGE)
            target_compile_options(${target} PRIVATE --coverage -O0)
            target_link_options(${target} PRIVATE --coverage)
        endif()
    endif()
endfunction()
