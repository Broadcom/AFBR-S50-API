# Usage:
# Configure:
# `cmake -G "Ninja" . -B build`
# `cmake -G "Unix Makefiles" . -B build`
# Specify Build Type (Debug, Release, RelWithDebInfo, MinSizeRel):
# `cmake -G "Ninja" . -B build -D CMAKE_BUILD_TYPE=Debug`
# `cmake -G "Ninja" . -B build -D CMAKE_BUILD_TYPE=Release`
# `cmake -G "Ninja" . -B build -D CMAKE_BUILD_TYPE=RelWithDebInfo`
# `cmake -G "Ninja" . -B build -D CMAKE_BUILD_TYPE=MinSizeRel`
# Build All:
# `cmake --build build`
# Build Specific Target Only:
# `cmake --build build --target afbr_s50_m0`
# `cmake --build build --target AFBR_S50_ExplorerApp_F401RE`
###############################################################################
cmake_minimum_required(VERSION 3.12...3.27)

# Optional: print out extra messages to see what is going on.
#           Comment it to have less verbose messages
set(CMAKE_VERBOSE_MAKEFILE ON)

# Optional: enable export of compiler commands to compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/Projects/cmake/arm-none-eabi-gcc.cmake")

# ##############################################################################
# Edit your MCU information up to the next border
project(AFBR-S50-API DESCRIPTION "AFBR-S50 API")


# Options:
# Overwrite w/ `-D<option>=<value>`
# Note: list all available options via; `cmake -B build -LH`

set(DEVICE "1" CACHE STRING "Selects the default device address for example projects. This will overwrite the SPI_SLAVE value (default: 1)")

# ##############################################################################

# Put the libaries and binaries that get built into directories at the top of
# the build tree rather than in hard-to-find leaf directories.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Compiler Setup
enable_language(C ASM)
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_STANDARD_REQUIRED ON)
# set(CMAKE_C_EXTENSIONS OFF)

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_C_COMPILER_VERSION: ${CMAKE_C_COMPILER_VERSION}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
# message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
# message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")

set(COMMON_COMPILE_DEFINITIONS
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>:RELEASE NDEBUG>
    $<$<CONFIG:RelWithDebInfo>:RELEASE NDEBUG>
    $<$<CONFIG:MinSizeRel>:RELEASE NDEBUG>
)

set(COMMON_COMPILE_OPTIONS
    # Debug Info
    $<$<CONFIG:Debug>:-g3 -ggdb>
    $<$<CONFIG:Release>: >
    $<$<CONFIG:RelWithDebInfo>:-g3 -ggdb>
    # $<$<CONFIG:MinSizeRel>: >
    # Optimization
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Release>:-Ofast>
    $<$<CONFIG:RelWithDebInfo>:-Og>
    $<$<CONFIG:MinSizeRel>:-Os>
    -c # compile only, not linking
    # -fno-builtin
    # -fno-common
    -mthumb
    # -ffreestanding
    # -nostdlib
    # -fsigned-char
    --specs=nosys.specs

    # ISO C library complience: https://developer.arm.com/documentation/dui0491/i/Compiler-Command-line-Options/--library-interface-lib
    -library_interface=aeabi_clib

    # WARNINGS
    -Wall
    -Wextra
    # -Wpedantic
    # -Wno-unused-parameter
    # -Wunused
    # -Wuninitialized
    # -Wmissing-declarations
    # -Wconversion
    # -Wpointer-arith
    # -Wshadow
    # -Wlogical-op
    # -Waggregate-return
    # -Wfloat-equal

    # Additional Data Output
    -fmessage-length=0
    -fstack-usage
    -fdump-rtl-dfinish
    # -fcyclomatic-complexity
    # remove path from __FILE__ macro
    -fmacro-prefix-map=\"${CMAKE_SOURCE_DIR}/\"=
    )

set(COMMON_LINK_OPTIONS
    # ${CPU_PARAMETERS}
    --specs=nosys.specs
    # -Wl,--verbose
    # -static # no link against shared libs
    # -nostdlib # do not use standard startup code
    # -Wl,--start-group
    # -lc
    # -lm
    # -lstdc++
    # -Wl,--end-group
    # -Wl,--sort-section=alignment
    # -Wl,--cref # create cross reference table
)


# ###############################################################################
# Build the Libraries
add_subdirectory(AFBR-S50)

add_subdirectory(Sources/platform/NXP_MKLxxZ)
add_subdirectory(Sources/platform/Renesas_RA4M2)
add_subdirectory(Sources/platform/STM32F4xx)

# ###############################################################################
# Build the Binaries

add_subdirectory(Projects/STM32CubeIDE/AFBR_S50_ExplorerApp_F401RE)
add_subdirectory(Projects/MCUXpressoIDE/AFBR_S50_ExplorerApp_KL46z)
add_subdirectory(Projects/MCUXpressoIDE/AFBR_S50_ExplorerApp_KL17z)
add_subdirectory(Projects/e2Studio/AFBR_S50_ExplorerApp_RA4M2)

add_subdirectory(Projects/MCUXpressoIDE/AFBR_S50_Example_KL17z)
add_subdirectory(Projects/MCUXpressoIDE/AFBR_S50_Example_KL46z)
add_subdirectory(Projects/STM32CubeIDE/AFBR_S50_Example_F401RE)
add_subdirectory(Projects/e2Studio/AFBR_S50_Example_RA4M2)

# ###############################################################################
