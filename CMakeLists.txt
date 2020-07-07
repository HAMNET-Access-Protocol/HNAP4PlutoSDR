cmake_minimum_required(VERSION 3.0.0)
project(transceiver C)

set(CMAKE_C_STANDARD 11)

include_directories(.)
include_directories(src/mac)
include_directories(src/phy)
include_directories(src/platform)
include_directories(src/runtime)
include_directories(src/util)
include_directories(tests/unity/src)

### Create version.h files to include version info in the executables
# get branch
execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
# get latest tag and commit hash
execute_process(
        COMMAND git describe --tags
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Git current branch: ${GIT_BRANCH}")
message(STATUS "Git tag: ${GIT_TAG}")
message(STATUS "Generating version.h")
configure_file(src/runtime/version.h.in version.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

### Group source files to PHY and MAC layer for UE/BS respectively

# PHY layer
set(PHY_OFDM_BS src/phy/phy_ofdm_common.h src/phy/phy_ofdm_bs.h src/phy/phy_ofdm_bs.c src/phy/phy_ofdm_common.c src/phy/phy_config.h src/phy/phy_config.c)
set(PHY_OFDM_UE src/phy/phy_ofdm_common.h src/phy/phy_ofdm_ue.h src/phy/phy_ofdm_ue.c src/phy/phy_ofdm_common.c src/phy/phy_config.h src/phy/phy_config.c)

set(PHY_SC_BS src/phy/phy_sc_common.h src/phy/phy_sc_bs.h src/phy/phy_sc_bs.c src/phy/phy_sc_common.c src/phy/phy_config.h src/phy/phy_config.c)
set(PHY_SC_UE src/phy/phy_sc_common.h src/phy/phy_sc_ue.h src/phy/phy_sc_ue.c src/phy/phy_sc_common.c src/phy/phy_config.h src/phy/phy_config.c)

# MAC layer
set(MAC_COMMON src/mac/mac_config.h src/mac/mac_channels.h src/mac/mac_common.h src/mac/mac_fragmentation.h src/mac/mac_messages.h
        src/mac/mac_channels.c src/mac/mac_messages.c src/mac/mac_common.c src/mac/mac_fragmentation.c src/mac/tap_dev.c)
set(MAC_UE ${MAC_COMMON} src/mac/mac_ue.h src/mac/mac_ue.c)
set(MAC_BS ${MAC_COMMON} src/mac/mac_bs.h src/mac/mac_bs.c)

# Platform
set(PLATFORM_PLUTO src/platform/platform.h src/platform/pluto.h src/platform/pluto.c
                   src/platform/pluto_gpio.c src/platform/pluto_gpio.h)

set(PLATFORM_SIM src/platform/platform.h src/platform/platform_simulation.h src/platform/platform_simulation.c)

# Utility
set(UTIL src/util/log.h src/util/log.c src/util/ringbuf.h src/util/ringbuf.c)

# Unit Tests
set(UNITY tests/unity/src/unity.c)
set(TESTS tests/test_mac_messages.c)

### Add different executables

# Simulation target
add_executable(test_mac src/runtime/test.h src/runtime/test_mac.c ${PLATFORM_SIM}
                        ${PHY_OFDM_BS} ${PHY_OFDM_UE} ${MAC_UE} ${MAC_BS} ${UTIL})
target_link_libraries(test_mac liquid m config)
target_compile_definitions(test_mac PUBLIC USE_SIM SIM_LOG_BER SIM_LOG_DELAY)

# Singlecarrier simulation target
add_executable(test_mac_sc src/runtime/test.h src/runtime/test_mac.c ${PLATFORM_SIM}
                           ${PHY_SC_BS} ${PHY_SC_UE} ${MAC_UE} ${MAC_BS} ${UTIL})
target_link_libraries(test_mac_sc liquid m config)
target_compile_definitions(test_mac_sc PUBLIC USE_SIM SIM_LOG_BER SIM_LOG_DELAY USE_SINGLECARRIER)

# Basestation
add_executable(basestation src/runtime/basestation.c  ${PLATFORM_PLUTO} ${PHY_OFDM_BS} ${MAC_BS} ${UTIL})
target_link_libraries(basestation liquid m iio pthread rt config)
target_compile_definitions(basestation PUBLIC MAC_ENABLE_TAP_DEV)

#Client
add_executable(client src/runtime/client.c ${PLATFORM_PLUTO}
        ${PHY_OFDM_UE} ${MAC_UE} ${UTIL})
target_link_libraries(client liquid m iio pthread rt config)
target_compile_definitions(client PUBLIC MAC_ENABLE_TAP_DEV)

#Client XO calibration tool
add_executable(client-calib src/runtime/client-calib.c ${PLATFORM_PLUTO}
        ${PHY_OFDM_UE} ${MAC_UE} ${UTIL})
target_link_libraries(client-calib liquid m iio pthread rt config)

# CFO estimation accuracy test
add_executable(test_cfo_estimation src/runtime/test_cfo_estimation.c ${PLATFORM_SIM}
        ${PHY_OFDM_BS} ${PHY_OFDM_UE} ${MAC_UE} ${MAC_BS} ${UTIL})
target_link_libraries(test_cfo_estimation liquid m config)
target_compile_definitions(test_cfo_estimation PUBLIC USE_SIM)

# Unit Tests
add_executable(unit_tests ${UNITY} ${TESTS} ${MAC_COMMON} ${UTIL})
target_link_libraries(unit_tests liquid)
