#!/bin/bash

#Script generates the custom root system overlay for the Adalm Pluto linux system firmware

#Path to the plutosdr firmware directory
PLUTOSDR_FW_ROOTOVERLAY=$HOME/plutosdr-fw/rootfs-overlay

#Path to pluto sysroot directory
PLUTO_SYSROOT_DIR=$HOME/pluto-0.31.sysroot

SCRIPT_DIR=`pwd`/`dirname "$0"`
cd $SCRIPT_DIR

## Init rootfs overlay structure
if [[ ! -d $PLUTOSDR_FW_ROOTOVERLAY ]]
then
    mkdir $PLUTOSDR_FW_ROOTOVERLAY
fi
if [[ ! -d $PLUTOSDR_FW_ROOTOVERLAY/usr/ ]]
then
    mkdir $PLUTOSDR_FW_ROOTOVERLAY/usr/
fi
if [[ ! -d $PLUTOSDR_FW_ROOTOVERLAY/usr/lib/ ]]
then
    mkdir $PLUTOSDR_FW_ROOTOVERLAY/usr/lib/
fi
if [[ ! -d $PLUTOSDR_FW_ROOTOVERLAY/root/ ]]
then
    mkdir $PLUTOSDR_FW_ROOTOVERLAY/root/
fi
if [[ ! -d $PLUTOSDR_FW_ROOTOVERLAY/etc/ ]]
then
    mkdir $PLUTOSDR_FW_ROOTOVERLAY/etc/
fi
if [[ ! -d $PLUTOSDR_FW_ROOTOVERLAY/etc/init.d/ ]]
then
    mkdir $PLUTOSDR_FW_ROOTOVERLAY/etc/init.d/
fi

## Build libfec
echo "Building libfec..."
cd libfec
./configure arm --host=arm-linux-gnueabihf --prefix="$PLUTO_SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$PLUTO_SYSROOT_DIR/"
make
make install

cp $PLUTO_SYSROOT_DIR/usr/lib/libfec.so $PLUTOSDR_FW_ROOTOVERLAY/usr/lib/
cd ..

## Build liquidsdr
echo "Building liquidsdr..."
cd liquid-dsp
./configure arm --host=arm-linux-gnueabihf  --prefix="$PLUTO_SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$PLUTO_SYSROOT_DIR"

# Configure somehow does not correctly identify malloc and realloc
# and redefines them in the generated config.h script as rpl_malloc.
# This makes the code unusable, therefore we delete this:
sed -i '/rpl_realloc/d' config.h
sed -i '/rpl_malloc/d' config.h

make
make install
cp $PLUTO_SYSROOT_DIR/usr/lib/libliquid.so $PLUTOSDR_FW_ROOTOVERLAY/usr/lib/
cd ..

## Build Main Apps
echo "Building main application..."
if [[ ! -d cmake-build ]]
then
    mkdir cmake-build
fi
cd cmake-build
cmake -DCMAKE_TOOLCHAIN_FILE=CmakeArmToolchain.cmake ..
make basestation
make client
make client-calib
cp basestation $PLUTOSDR_FW_ROOTOVERLAY/root/
cp client $PLUTOSDR_FW_ROOTOVERLAY/root/
cp client-calib $PLUTOSDR_FW_ROOTOVERLAY/root/
cd ..

## Copy network init script to rootfs
echo "Copy network autoconfig script to rootfs..."
cd startup_scripts
cp S101transceiver.sh $PLUTOSDR_FW_ROOTOVERLAY/etc/init.d/
cp S102iperf.sh $PLUTOSDR_FW_ROOTOVERLAY/etc/init.d/
cp S103basestation.sh $PLUTOSDR_FW_ROOTOVERLAY/etc/init.d/
cd ..

## Copy FIR filter to rootfs
echo "Copy FIR filter coefficients to rootfs..."
cp AD9361_256kSPS.ftr $PLUTOSDR_FW_ROOTOVERLAY/root/

## Copy the default configuration file to rootfs
echo "Copy config.txt to rootfs..."
cp config.txt $PLUTOSDR_FW_ROOTOVERLAY/root/

echo "Done."

