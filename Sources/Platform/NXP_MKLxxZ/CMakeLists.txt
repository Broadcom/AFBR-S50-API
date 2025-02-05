function(build_bsp MCU_TYPE)

    if(NOT MCU_TYPE MATCHES "^(mkl46z|mkl17z)$")
        message(FATAL_ERROR "Invalid MCU_TYPE: ${MCU_TYPE}.")
    endif()

    # message(STATUS "BUILD Platform Layer for ${MCU_TYPE}")

    if (MCU_TYPE STREQUAL "mkl46z")
        set(MCU_FAMILY MKL46Z)
        set(MCU_MODEL MKL46Z256VLL4)
    else()
        set(MCU_FAMILY MKL17Z)
        set(MCU_MODEL MKL17Z256VFM4)
    endif()

    set(LIBRARY ${MCU_TYPE})
    set(DISABLE_WATCHDOG 0)
    set(FP_USE_HW_DIV 0)
    set(FP_USE_64BIT_MUL 0)
    set(EXPLORER_API_PRO 1)
    set(ARGUS_LOG_LEVEL 2)
    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m0
        -mfloat-abi=soft)
    set(LINKER_SCRIPT "${PROJECT_DIR}/script/${MCU_FAMILY}.ld")
    set(API_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/AFBR-S50/Include")
    set(BSP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/platform/NXP_MKLxxZ")
    set(UTIL_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/utility")

    set(COMPILE_DEFINITIONS
        ${COMMON_COMPILE_DEFINITIONS}
        ${MCU_MODEL}
        CPU_${MCU_MODEL}
        DISABLE_WDOG=${DISABLE_WATCHDOG}
        SDK_OS_BAREMETAL
        __USE_CMSIS
        FP_USE_HW_DIV=${FP_USE_HW_DIV}
        FP_USE_64BIT_MUL=${FP_USE_64BIT_MUL}
        EXPLORER_API_PRO=${EXPLORER_API_PRO}
        ARGUS_LOG_LEVEL=${ARGUS_LOG_LEVEL})
    # message(STATUS "COMPILE_DEFINITIONS: ${COMPILE_DEFINITIONS}")

    set(COMPILE_OPTIONS ${COMMON_COMPILE_OPTIONS} ${CPU_COMPILE_OPTIONS})
    # message(STATUS "COMPILE_OPTIONS: ${COMPILE_OPTIONS}")

    set(BSP_INCLUDE_DIRECTORIES_PRIVATE ${API_INCLUDE_DIR} ${UTIL_SOURCE_DIR})
    set(BSP_INCLUDE_DIRECTORIES_PUBLIC ${BSP_SOURCE_DIR})
    file(GLOB BSP_SOURCES FOLLOW_SYMLINKS
        ${BSP_SOURCE_DIR}/board/*.c
        ${BSP_SOURCE_DIR}/board/${MCU_FAMILY}/*.c
        ${BSP_SOURCE_DIR}/driver/*.c
        ${BSP_SOURCE_DIR}/driver/${MCU_FAMILY}/*.c
        ${BSP_SOURCE_DIR}/startup/*.c
        ${BSP_SOURCE_DIR}/startup/${MCU_FAMILY}/*.c)
    # message(STATUS "FILES: ${BSP_SOURCES}")

    if (MCU_TYPE STREQUAL "mkl46z")
        file(GLOB USB_SOURCES FOLLOW_SYMLINKS
            ${BSP_SOURCE_DIR}/usb/*.c
            ${BSP_SOURCE_DIR}/usb/source/*.c)
    endif()

    add_library(${LIBRARY} OBJECT)
    target_sources(${LIBRARY} PRIVATE ${BSP_SOURCES} ${USB_SOURCES})
    target_include_directories(${LIBRARY} PRIVATE ${BSP_INCLUDE_DIRECTORIES_PRIVATE})
    target_include_directories(${LIBRARY} PUBLIC ${BSP_INCLUDE_DIRECTORIES_PUBLIC})
    target_compile_definitions(${LIBRARY} PRIVATE ${COMPILE_DEFINITIONS})
    target_compile_options(${LIBRARY} PRIVATE ${COMPILE_OPTIONS})

endfunction()

build_bsp(mkl17z)
build_bsp(mkl46z)

