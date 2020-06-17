#!/bin/sh

# Configure pluto platform so that the waveform can be started
# Name of the custom FIR filter
FILTER=AD9361_256kSPS.ftr

# Networking: create tap devices
TAP_IP_ADDR=`fw_printenv -n hnap_tap_ip`

tunctl -t tap0
ifconfig tap0 $TAP_IP_ADDR
ifconfig tap0 up

# Install custom FIR filter
cat /root/$FILTER > /sys/bus/iio/devices/iio:device1/filter_fir_config

