cmake_minimum_required(VERSION 3.15.2)

project(StarTracker LANGUAGES ASM C CXX)

set(APP_VERSION 1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE        OFF)
set(CMAKE_COLOR_MAKEFILE          ON)

set(USE_SOFTDEVICE                ON)

set(SDK_ROOT    "C:/Nordic/nRF5_SDK_17.1.0_ddde560")
set(PROJ_DIR    "${CMAKE_CURRENT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DNRF_LOG_ENABLED=0)
else()
    add_definitions(-DNRF_LOG_ENABLED=1)
    add_definitions(-DNRF_LOG_DEFERRED=0)
#    add_definitions(-DNRFX_WDT_ENABLED=0)
    add_definitions(-DDEBUG -DDEBUG_NRF -DDEBUG_NRF_USER)
endif()

if (USE_SOFTDEVICE)

    add_definitions(-DSOFTDEVICE_PRESENT)
    add_definitions(-DBLE_STACK_SUPPORT_REQD)
#    add_definitions(-DANT_STACK_SUPPORT_REQD)
    add_definitions(-DS140)

else(USE_SOFTDEVICE)

    add_definitions(-DFDS_BACKEND=1)

endif(USE_SOFTDEVICE)

add_definitions(-DBOARD_CUSTOM)
add_definitions(-DPRESS_API_VER=1)

add_definitions(-DTASK_MANAGER_CONFIG_STACK_GUARD=0)
add_definitions(-DTASK_MANAGER_CONFIG_STACK_SIZE=64)
add_definitions(-DPNP_ID_PRODUCT_VERSION=${APP_VERSION})
#add_definitions(-DANT_LICENSE_KEY="3831-521d-7df9-24d8-eff3-467b-225f-a00e")
#add_definitions(-DUSE_SVIEW=1 -DUSE_RTT=0) # we need NRF_LOG_ENABLED=0
add_definitions(-DUSE_SVIEW=0 -DUSE_RTT=1) # we need NRF_LOG_ENABLED=1
#add_definitions(-DUSE_SVIEW=0 -DUSE_JSCOPE -DUSE_RTT=1) # we need NRF_LOG_ENABLED=0
add_definitions(-DUSE_APP_CONFIG)
add_definitions(-DNRF52840_XXAA)
add_definitions(-DBL_SETTINGS_ACCESS_ONLY)
add_definitions(-DNRF_DFU_SVCI_ENABLED)
add_definitions(-DNRF_DFU_TRANSPORT_BLE=1)
add_definitions(-DCONFIG_NFCT_PINS_AS_GPIOS)
add_definitions(-DCONFIG_GPIO_AS_PINRESET)
add_definitions(-D__HEAP_SIZE=2048)
add_definitions(-DNRF_SD_BLE_API_VERSION=7)
add_definitions(-DNRFX_SAADC_API_V2)
add_definitions(-DAPP_TIMER_V2)
add_definitions(-DAPP_TIMER_V2_RTC1_ENABLED)
add_definitions(-DAPP_TIMER_KEEPS_RTC_ACTIVE)
add_definitions(-DSWI_DISABLE0)
add_definitions(-DNRF52_PAN_74)
add_definitions(-DFLOAT_ABI_HARD -fsingle-precision-constant)
add_definitions(-Wdouble-promotion -Wfloat-conversion -Werror=return-type)
add_definitions(-Wno-attributes -Wsign-compare -Wenum-compare)


add_link_options(-L${SDK_ROOT}/modules/nrfx/mdk)
if (USE_SOFTDEVICE)
    add_link_options(-T${CMAKE_CURRENT_SOURCE_DIR}/s140.ld)
else(USE_SOFTDEVICE)
    add_link_options(-T${CMAKE_CURRENT_SOURCE_DIR}/s140.ld)
endif(USE_SOFTDEVICE)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-Wl,--gc-sections)
add_link_options(--specs=nano.specs)

add_compile_options(-DARM_MATH_CM4)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_compile_options(-ffunction-sections -fdata-sections -fno-strict-aliasing)
add_compile_options(-fno-builtin -fshort-enums -nostdlib -fno-exceptions)

set(CMAKE_CXX_FLAGS "-std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti" CACHE STRING "")

# Includes used by the SDK only
include_directories(
        ${PROJ_DIR}/.
        ${PROJ_DIR}/config
        ${PROJ_DIR}/drivers
        ${PROJ_DIR}/helpers
        ${PROJ_DIR}/vault
        ${PROJ_DIR}/sim
        ${PROJ_DIR}/src
        ${PROJ_DIR}/src/parsing
        ${PROJ_DIR}/src/scheduling
        ${PROJ_DIR}/src/sensors
        ${PROJ_DIR}/rf

        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/ant_common # this is where our precious ANT+ key is stored

        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/hardfault
        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/rtt
        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/sysview
        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/task_manager
        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/utils
    )

add_subdirectory(sdk    ./libs/sdk/)

# if a file is added in one of those dirs, the cmake... command must be re-run
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/sim SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/scheduling SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/drivers SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/helpers SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/rf SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/hardfault SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/rtt SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/sysview SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/task_manager SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/utils SRCS)

add_executable(${PROJECT_NAME}.elf ${SRCS} s140.ld)

target_link_libraries(${PROJECT_NAME}.elf nosys)
target_link_libraries(${PROJECT_NAME}.elf -Wl,--whole-archive SDK_LIBS -Wl,--no-whole-archive)

add_custom_target(${PROJECT_NAME}.bin ALL DEPENDS ${PROJECT_NAME}.elf)
add_custom_target(${PROJECT_NAME}.hex ALL DEPENDS ${PROJECT_NAME}.elf)

add_custom_command(TARGET ${PROJECT_NAME}.bin POST_BUILD
        COMMAND ${TOOLCHAIN}-objcopy ARGS -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.hex POST_BUILD
        COMMAND ${TOOLCHAIN}-objcopy ARGS -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND size --format=berkeley "${PROJECT_NAME}.elf"
        COMMENT "Invoking: Print Size")

find_program(NRFUTIL "nrfutil")
find_program(NRFJPROG "nrfjprog")
find_program(MERGEHEX "mergehex" HINTS "c:/Program Files (x86)/Nordic Semiconductor/nrf5x/bin")

if(NOT NRFJPROG)
    message(FATAL_ERROR "nrfjprog not found, not able to merge connectivity firmware with SoftDevice.")
endif()

if(NOT MERGEHEX)
    message(FATAL_ERROR "mergehex not found, not able to merge connectivity firmware with SoftDevice.")
endif()

if(NOT NRFUTIL)
    message(FATAL_ERROR "nrfutil not found, not able to create DFU packages of connectivity firmware.")
endif()

add_custom_target(flash
        COMMAND ${NRFJPROG} --program ${PROJECT_NAME}.hex -f nrf52 --sectorerase
        COMMAND ${NRFJPROG} --reset -f nrf52
        DEPENDS ${PROJECT_NAME}.hex)

add_custom_target(flash_sd
        COMMAND ${NRFJPROG} --program C:/Nordic/nRF5_SDK_17.1.0_ddde560/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex -f nrf52 --sectorerase
        COMMAND ${NRFJPROG} --reset -f nrf52
        )

