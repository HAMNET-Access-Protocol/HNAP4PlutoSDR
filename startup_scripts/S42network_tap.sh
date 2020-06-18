#!/bin/sh

# Networking: create tap devices
# Note: this has to be done before the lldp service is started (S60)
TAP_IP_ADDR=`fw_printenv -n hnap_tap_ip`

tunctl -t tap0
ifconfig tap0 $TAP_IP_ADDR
ifconfig tap0 up