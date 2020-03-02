cmake_minimum_required(VERSION 3.0.0)
project(transceiver C)

set(CMAKE_C_STANDARD 11)

include_directories(.)
include_directories(mac)
include_directories(phy)
include_directories(platform)
include_directories(runtime)
include_directories(util)


# Group source files to PHY and MAC layer for UE/BS respectively
set(PHY_BS phy/phy_common.h phy/phy_bs.h phy/phy_bs.c phy/phy_common.c)
set(PHY_UE phy/phy_common.h phy/phy_ue.h phy/phy_ue.c phy/phy_common.c)

set(MAC_COMMON mac/mac_config.h mac/mac_channels.h mac/mac_common.h mac/mac_fragmentation.h mac/mac_messages.h
               mac/mac_channels.c mac/mac_messages.c mac/mac_common.c mac/mac_fragmentation.c mac/tap_dev.c)
set(MAC_UE ${MAC_COMMON} mac/mac_ue.h mac/mac_ue.c)
set(MAC_BS ${MAC_COMMON} mac/mac_bs.h mac/mac_bs.c)
set(UTIL util/log.h util/log.c util/ringbuf.h util/ringbuf.c)


# Add different executables

# Simulation target
add_executable(test_mac runtime/test.h runtime/test_mac.c platform/platform.h platform/platform_simulation.h platform/platform_simulation.c
                        ${PHY_BS} ${PHY_UE} ${MAC_COMMON} ${MAC_UE} ${MAC_BS} ${UTIL})
target_link_libraries(test_mac liquid m)
target_compile_definitions(test_mac PUBLIC USE_SIM SIM_LOG_BER SIM_LOG_DELAY)

# Basestation
add_executable(basestation runtime/basestation.c platform/platform.h platform/pluto.h platform/pluto.c
                            ${PHY_BS} ${MAC_COMMON} ${MAC_BS} ${UTIL})
target_link_libraries(basestation liquid m iio pthread)
target_compile_definitions(basestation PUBLIC MAC_ENABLE_TAP_DEV)

#Client
add_executable(client runtime/client.c platform/platform.h platform/pluto.h platform/pluto.c
        ${PHY_UE} ${MAC_COMMON} ${MAC_UE} ${UTIL})
target_link_libraries(client liquid m iio pthread)
target_compile_definitions(client PUBLIC MAC_ENABLE_TAP_DEV)
