function(build EXAMPLE)

    if(EXAMPLE STREQUAL "1")
        set(PROJECT "AFBR_S50_SimpleExample_KL46z")
    elseif(EXAMPLE STREQUAL "2")
        set(PROJECT "AFBR_S50_AdvancedExample_KL46z")
    elseif(EXAMPLE STREQUAL "3")
        set(PROJECT "AFBR_S50_HighSpeedExample_KL46z")
    elseif(EXAMPLE STREQUAL "4")
        set(PROJECT "AFBR_S50_MultiDeviceExample_KL46z")
    elseif(EXAMPLE STREQUAL "5")
        set(PROJECT "AFBR_S50_SimpleExampleDebug_KL46z")
    else()
        message(FATAL_ERROR "Invalid EXAMPLE: ${EXAMPLE}.")
    endif()

    set(MCU_FAMILY MKL46Z)
    set(MCU_MODEL MKL46Z256VLL4)
    set(CPU_TYPE m0plus)
    # set(FPU_TYPE )
    set(FP_USE_HW_DIV 0)
    set(FP_USE_64BIT_MUL 0)
    set(ARGUS_LOG_LEVEL 2)
    set(DEVICE_COUNT 2)
    set(DISABLE_WATCHDOG 0)
    set(IDE MCUXpressoIDE)
    set(API_LIBRARY afbrs50_m0)
    set(BSP_LIBRARY mkl46z)

    # ##############################################################################
    project(${PROJECT})

    set(PROJECT_DIR "${CMAKE_SOURCE_DIR}/Projects/${IDE}/AFBR_S50_Example_KL46z")
    set(APP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/ExampleApp")
    set(BSP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/platform/NXP_MKLxxZ/")
    set(UTIL_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/utility")
    set(HALTEST_SOURCE_DIR "${CMAKE_SOURCE_DIR}/AFBR-S50/Test")
    set(LINKER_SCRIPT "${PROJECT_DIR}/script/${MCU_FAMILY}.ld")

    set(COMPILE_DEFINITIONS
        ${COMMON_COMPILE_DEFINITIONS}
        ${MCU_MODEL}
        CPU_${MCU_MODEL}
        FP_USE_HW_DIV=${FP_USE_HW_DIV}
        FP_USE_64BIT_MUL=${FP_USE_64BIT_MUL}
        ARGUS_LOG_LEVEL=${ARGUS_LOG_LEVEL}
        API_EXAMPLE=${EXAMPLE}
        SPI_SLAVE=${DEVICE}
        DEVICE_COUNT=${DEVICE_COUNT}
        DISABLE_WDOG=${DISABLE_WATCHDOG}
        SDK_OS_BAREMETAL
        __USE_CMSIS)

    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-${CPU_TYPE}
        # -mfpu=${FPU_TYPE}
        -mfloat-abi=soft)

    set(COMPILE_OPTIONS ${COMMON_COMPILE_OPTIONS} ${CPU_COMPILE_OPTIONS})

    set(LINK_OPTIONS
        ${COMMON_LINK_OPTIONS}
        ${CPU_COMPILE_OPTIONS}
        -L ${PROJECT_DIR}/script
        -T${LINKER_SCRIPT}
        -Wl,-Map=${PROJECT}.map)

    # ##############################################################################

    # Headers
    set(INCLUDE_DIRECTORIES
        ${BSP_SOURCE_DIR}
        ${APP_SOURCE_DIR}
        ${APP_SOURCE_DIR}/examples
        ${HALTEST_SOURCE_DIR}
        ${UTIL_SOURCE_DIR}/printf
        ${UTIL_SOURCE_DIR})

    # Sources
    file(GLOB_RECURSE PROJECT_SOURCES FOLLOW_SYMLINKS
        ${APP_SOURCE_DIR}/*.c
        ${HALTEST_SOURCE_DIR}/*.c
        ${UTIL_SOURCE_DIR}/*.c)

    # ##############################################################################

    # Executable files
    add_executable(${PROJECT} ${PROJECT_SOURCES})
    target_compile_definitions(${PROJECT} PRIVATE ${COMPILE_DEFINITIONS})
    target_include_directories(${PROJECT} PRIVATE ${INCLUDE_DIRECTORIES})

    # Compiler and linker options
    target_compile_options(${PROJECT} PRIVATE ${COMPILE_OPTIONS})
    target_link_options(${PROJECT} PRIVATE ${LINK_OPTIONS})
    target_link_libraries(${PROJECT} PRIVATE ${BSP_LIBRARY} ${API_LIBRARY})

    # Post Build: Print size and convert to *.hex and *.bin
    add_custom_command(TARGET ${PROJECT} POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} $<TARGET_FILE:${PROJECT}>
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT}> $<TARGET_FILE_DIR:${PROJECT}>/${PROJECT}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT}> $<TARGET_FILE_DIR:${PROJECT}>/${PROJECT}.bin
        VERBATIM)

endfunction()

build(1)
build(2)
build(3)
build(4)
build(5)
