function(build LOGGING)

    if(NOT LOGGING MATCHES "^(ON|OFF)$")
        message(FATAL_ERROR "Invalid LOGGING: ${LOGGING}.")
    endif()

    set(PROJECT "AFBR_S50_ExplorerApp_F401RE_UART")

    set(MCU_FAMILY STM32F4xx)
    set(MCU_MODEL STM32F401xE)
    set(CPU_TYPE m4)
    set(FPU_TYPE fpv4-sp-d16)
    set(FP_USE_HW_DIV 1)
    set(FP_USE_64BIT_MUL 1)
    set(EXPLORER_API_PRO 1)
    set(EXPLORER_USE_USB 0)
    set(EXPLORER_DEVICE_COUNT 4)
    set(ARGUS_LOG_LEVEL 2)
    set(IDE STM32CubeIDE)
    set(API_LIBRARY afbrs50_m4_fpu)
    set(BSP_LIBRARY f401re)

    if (NOT LOGGING STREQUAL "OFF")
        set(ARGUS_LOG_LEVEL 0)
        set(PROJECT "${PROJECT}_NoLog")
        set(API_LIBRARY "${API_LIBRARY}_nolog")
    endif()

    # ##############################################################################
    project(${PROJECT})

    set(PROJECT_DIR "${CMAKE_SOURCE_DIR}/Projects/${IDE}/AFBR_S50_ExplorerApp_F401RE")
    set(APP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/ExplorerApp")
    set(BSP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/platform/${MCU_FAMILY}")
    set(UTIL_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Sources/utility")
    set(LINKER_SCRIPT "${PROJECT_DIR}/STM32F401RETX_FLASH.ld")

    set(COMPILE_DEFINITIONS
        ${COMMON_COMPILE_DEFINITIONS}
        ${MCU_MODEL}
        FP_USE_HW_DIV=${FP_USE_HW_DIV}
        FP_USE_64BIT_MUL=${FP_USE_64BIT_MUL}
        ARGUS_LOG_LEVEL=${ARGUS_LOG_LEVEL}
        EXPLORER_API_PRO=${EXPLORER_API_PRO}
        EXPLORER_USE_USB=${EXPLORER_USE_USB}
        EXPLORER_DEVICE_COUNT=${EXPLORER_DEVICE_COUNT}
        USE_HAL_DRIVER)

    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-${CPU_TYPE}
        -mfpu=${FPU_TYPE}
        -mfloat-abi=hard)

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
        ${APP_SOURCE_DIR}
        ${UTIL_SOURCE_DIR}/printf
        ${UTIL_SOURCE_DIR})

    # Sources
    file(GLOB_RECURSE PROJECT_SOURCES FOLLOW_SYMLINKS
        ${APP_SOURCE_DIR}/*.c
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

build(ON)
build(OFF)
