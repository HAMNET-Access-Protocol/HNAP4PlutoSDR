cmake_minimum_required(VERSION 3.0.0)
project(transceiver C)

set(CMAKE_C_STANDARD 11)

include_directories(.)
include_directories(src/mac)
include_directories(src/phy)
include_directories(src/platform)
include_directories(src/runtime)
include_directories(src/util)


### Group source files to PHY and MAC layer for UE/BS respectively

# PHY layer
set(PHY_BS src/phy/phy_common.h src/phy/phy_bs.h src/phy/phy_bs.c src/phy/phy_common.c src/phy/phy_config.h src/phy/phy_config.c)
set(PHY_UE src/phy/phy_common.h src/phy/phy_ue.h src/phy/phy_ue.c src/phy/phy_common.c src/phy/phy_config.h src/phy/phy_config.c)

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


### Add different executables

# Simulation target
add_executable(test_mac src/runtime/test.h src/runtime/test_mac.c ${PLATFORM_SIM}
                        ${PHY_BS} ${PHY_UE} ${MAC_UE} ${MAC_BS} ${UTIL})
target_link_libraries(test_mac liquid m config)
target_compile_definitions(test_mac PUBLIC USE_SIM SIM_LOG_BER SIM_LOG_DELAY)

# Basestation
add_executable(basestation src/runtime/basestation.c  ${PLATFORM_PLUTO} ${PHY_BS} ${MAC_BS} ${UTIL})
target_link_libraries(basestation liquid m iio pthread rt config)
target_compile_definitions(basestation PUBLIC MAC_ENABLE_TAP_DEV)

#Client
add_executable(client src/runtime/client.c ${PLATFORM_PLUTO}
        ${PHY_UE} ${MAC_UE} ${UTIL})
target_link_libraries(client liquid m iio pthread rt config)
target_compile_definitions(client PUBLIC MAC_ENABLE_TAP_DEV)

#Client XO calibration tool
add_executable(client-calib src/runtime/client-calib.c ${PLATFORM_PLUTO}
        ${PHY_UE} ${MAC_UE} ${UTIL})
target_link_libraries(client-calib liquid m iio pthread rt config)

# CFO estimation accuracy test
add_executable(test_cfo_estimation src/runtime/test_cfo_estimation.c ${PLATFORM_SIM}
        ${PHY_BS} ${PHY_UE} ${MAC_UE} ${MAC_BS} ${UTIL})
target_link_libraries(test_cfo_estimation liquid m config)
target_compile_definitions(test_cfo_estimation PUBLIC USE_SIM)
