project(AFBR-S50-API DESCRIPTION "NXP_MKLxxZ")

function(build_bsp)

    # message(STATUS "BUILD Platform Layer for Renesas RA4M2")

    set(LIBRARY ra4m2)
    set(MCU_FAMILY RA4M2)
    set(CPU_TYPE m33)
    set(FPU_TYPE fpv5-sp-d16)
    set(FP_USE_HW_DIV 1)
    set(FP_USE_64BIT_MUL 1)
    set(EXPLORER_API_PRO 1)
    # set(ARGUS_LOG_LEVEL 2)
    set(PROJECT_DIR "${CMAKE_SOURCE_DIR}/Projects/e2Studio/AFBR_S50_ExplorerApp_RA4M2")
    set(API_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/AFBR-S50/Include")
    set(BSP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/platform/Renesas_${MCU_FAMILY}")
    set(UTIL_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/utility")

    set(COMPILE_DEFINITIONS
        ${COMMON_COMPILE_DEFINITIONS}
        # ${MCU_MODEL}
        # CPU_${MCU_MODEL}
        FP_USE_HW_DIV=${FP_USE_HW_DIV}
        FP_USE_64BIT_MUL=${FP_USE_64BIT_MUL}
        EXPLORER_API_PRO=${EXPLORER_API_PRO}
        # ARGUS_LOG_LEVEL=${ARGUS_LOG_LEVEL})
        _RA_CORE=CM33
        _RENESAS_RA_)
    # message(STATUS "COMPILE_DEFINITIONS: ${COMPILE_DEFINITIONS}")

    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-${CPU_TYPE}
        -mfpu=${FPU_TYPE}
        -mfloat-abi=hard)

    set(COMPILE_OPTIONS ${COMMON_COMPILE_OPTIONS} ${CPU_COMPILE_OPTIONS})
    # message(STATUS "COMPILE_OPTIONS: ${COMPILE_OPTIONS}")

    set(BSP_INCLUDE_DIRECTORIES_PRIVATE
        ${PROJECT_DIR}/ra/fsp/inc
        ${PROJECT_DIR}/ra/fsp/inc/api
        ${PROJECT_DIR}/ra/fsp/inc/instances
        ${PROJECT_DIR}/ra/fsp/src/r_usb_basic/src/driver/inc
        ${PROJECT_DIR}/ra/arm/CMSIS_5/CMSIS/Core/Include
        ${PROJECT_DIR}/src
        ${PROJECT_DIR}/script
        ${PROJECT_DIR}/ra_gen
        ${PROJECT_DIR}/ra_cfg/fsp_cfg/bsp
        ${PROJECT_DIR}/ra_cfg/fsp_cfg
        ${BSP_SOURCE_DIR}
        ${BSP_SOURCE_DIR}/board
        ${BSP_SOURCE_DIR}/driver
        ${BSP_SOURCE_DIR}/usb
        ${APP_SOURCE_DIR}
        ${API_INCLUDE_DIR}
        ${UTIL_SOURCE_DIR}/printf
        ${UTIL_SOURCE_DIR})
    set(BSP_INCLUDE_DIRECTORIES_PUBLIC ${BSP_SOURCE_DIR})

    # Sources
    file(GLOB_RECURSE FSP_SOURCES ${PROJECT_DIR}/*.c)
    # Remove generated main.c to avoid dublicate main()
    list(FILTER FSP_SOURCES EXCLUDE REGEX "main\.c$")
    file(GLOB_RECURSE BSP_SOURCES FOLLOW_SYMLINKS ${BSP_SOURCE_DIR}/*.c)

    add_library(${LIBRARY} OBJECT)
    target_sources(${LIBRARY} PRIVATE ${BSP_SOURCES} ${FSP_SOURCES})
    target_include_directories(${LIBRARY} PRIVATE ${BSP_INCLUDE_DIRECTORIES_PRIVATE})
    target_include_directories(${LIBRARY} PUBLIC ${BSP_INCLUDE_DIRECTORIES_PUBLIC})
    target_compile_definitions(${LIBRARY} PRIVATE ${COMPILE_DEFINITIONS})
    target_compile_options(${LIBRARY} PRIVATE ${COMPILE_OPTIONS})

endfunction()

build_bsp()

