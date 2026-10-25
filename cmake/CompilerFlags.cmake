# Compiler settings for every target that holds library code, tests,
# benchmarks, samples or tools. maul_apply_flags(target) sets C23, the
# floating-point flags the family requires, the warning set and the
# options named ${MAUL_PREFIX}_WERROR, _SANITIZE, _TSAN and _COVERAGE. A
# library adds architecture flags through MAUL_ARCH_FLAGS and definitions
# through MAUL_EXTRA_DEFINITIONS before calling it.
#
# Supported compilers: GCC 14 and Clang 19 or newer, and clang-cl on
# Windows. MSVC's cl.exe does not implement C23 and is refused.

if(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    message(FATAL_ERROR "MSVC's cl.exe cannot compile C23. Use clang-cl: "
                        "the Visual Studio component 'C++ Clang tools for Windows', "
                        "selected with -T ClangCL or CMAKE_C_COMPILER=clang-cl.")
endif()
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_LESS 14)
    message(FATAL_ERROR "GCC 14 or newer is required for C23; found ${CMAKE_C_COMPILER_VERSION}")
endif()
if(CMAKE_C_COMPILER_ID MATCHES "Clang" AND NOT CMAKE_C_COMPILER_ID STREQUAL "AppleClang"
   AND CMAKE_C_COMPILER_VERSION VERSION_LESS 19)
    message(FATAL_ERROR "Clang 19 or newer is required for C23; found ${CMAKE_C_COMPILER_VERSION}")
endif()

function(maul_apply_flags target)
    # Fast math anywhere in the global flags would void the family's
    # floating-point rules, so configuration stops instead.
    foreach(config "" _DEBUG _RELEASE _RELWITHDEBINFO _MINSIZEREL)
        string(REGEX MATCH "fast-math|/fp:fast|-Ofast" fast "${CMAKE_C_FLAGS${config}}")
        if(fast)
            message(FATAL_ERROR "CMAKE_C_FLAGS${config} contains ${fast}, which the family forbids")
        endif()
    endforeach()

    set_target_properties(${target} PROPERTIES
        C_STANDARD 23
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS OFF)

    if(MSVC)
        # clang-cl: MSVC-style driver options, clang options through /clang:.
        # Clang's floating-point model is precise by default; the family
        # turns contraction and errno-setting math off. C23 is asked
        # of clang directly: Visual Studio's ClangCL toolset gave a target
        # made after a fetched project's LanguageStandard_C stdclatest,
        # which it compiled below C23, in place of -clang:-std=c23.
        target_compile_options(${target} PRIVATE /W4 $<$<COMPILE_LANGUAGE:C>:/clang:-std=c23>
                                                 /clang:-ffp-contract=off /clang:-fno-math-errno /clang:-Wshadow
                                                 /clang:-Wmissing-prototypes)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            target_compile_options(${target} PRIVATE /clang:-Wshorten-64-to-32)
        endif()
        if(${MAUL_PREFIX}_WERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        # -fno-math-errno makes sqrt the one rounded instruction on every
        # target, never a C library call (on 32-bit x86, an x87 one). It
        # comes after -fno-fast-math, which in GCC turns errno math back on.
        # Trapping math is left at each compiler's default (on in GCC, off
        # in Clang): GCC's -fno-fast-math turns it back on anyway.
        target_compile_options(${target} PRIVATE
            -ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations -fno-math-errno
            -Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion
            $<$<COMPILE_LANGUAGE:C>:-Wmissing-prototypes>)
        # A 64-bit value cut to 32 bits, where pointers are 64 bits (a
        # 32-bit size_t would flag every size taken from a 64-bit value);
        # GCC has no warning this narrow.
        if(CMAKE_C_COMPILER_ID MATCHES "Clang" AND CMAKE_SIZEOF_VOID_P EQUAL 8)
            target_compile_options(${target} PRIVATE -Wshorten-64-to-32)
        endif()
        # 32-bit x86 computes on the x87 unit by default, with excess
        # precision (FLT_EVAL_METHOD 2), which fpenv.c refuses. SSE math
        # gives every operation its declared type there, as on every other
        # target. An -m32 build on an x86-64 host keeps the host's
        # processor name, so the pointer size tells it apart. Emscripten's
        # toolchain names the processor x86 too, but it targets wasm32.
        if(CMAKE_SIZEOF_VOID_P EQUAL 4 AND NOT EMSCRIPTEN
           AND CMAKE_SYSTEM_PROCESSOR MATCHES "^([iI][3-6]86|x86|X86|x86_64|AMD64|amd64)$")
            target_compile_options(${target} PRIVATE -msse2 -mfpmath=sse)
        endif()
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
