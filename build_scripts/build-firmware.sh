#!/bin/bash

# This script creates our custom firmware image
# based on analog decices plutosdr-fw

# Required plutosdr-fw version
REQUIRED_PLUTOSDR_FW_VERSION="v0.32"


#Check if env variable for Xilinx tools exists
: "${VIVADO_SETTINGS?Need to set VIVADO_SETTINGS (e.g. /opt/Xilinx/Vivado/2019.1/settings64.sh)}"
source $VIVADO_SETTINGS || (echo "Specified wrong Vivado settings path?"; exit 1;)

#Check if env variable for Path to pluto sysroot directory exists
: "${PLUTO_SYSROOT_DIR?Need to set PLUTO_SYSROOT_DIR}"

SCRIPT_DIR=`pwd`/`dirname "$0"`
cd $SCRIPT_DIR || exit 1

# Download the current plutosdr-fw image
if [[ ! -d plutosdr-fw ]]
then
  echo "plutosdr-fw repository not found. Downloading it..."
  git clone --recurse-submodules -j8 https://github.com/analogdevicesinc/plutosdr-fw.git
else
  echo "plutosdr-fw already exists. Skipping git clone."
fi


cd plutosdr-fw ||  (echo "Failed to cd"; exit 1)
if [[ $REQUIRED_PLUTOSDR_FW_VERSION != $(git describe --tags) ]];
then
  echo "Got wrong plutosdr-fw version. Checking out "$REQUIRED_PLUTOSDR_FW_VERSION
  git reset --hard
  git submodule foreach --recursive git reset --hard
  git checkout $PLUTOSDR_FW_VERSION -b master
  git submodule update --recursive

  # Changes to buildroot
  BR_CONFIG=buildroot/configs/zynq_pluto_defconfig
  echo 'BR2_ROOTFS_OVERLAY="'"$PLUTOSDR_FW_ROOTOVERLAY"'"' >> $BR_CONFIG
  echo 'BR2_PACKAGE_LIBCONFIG=y' >> $BR_CONFIG
  echo 'BR2_PACKAGE_FFTW_SINGLE=y' >> $BR_CONFIG
  echo 'BR2_PACKAGE_TUNCTL=y' >> $BR_CONFIG
  echo 'BR2_PACKAGE_IPERF3=y' >> $BR_CONFIG
  echo 'BR2_PACKAGE_LLDPD=y' >> $BR_CONFIG

  # Changes to the linux kernel
  echo "Applying PREEMPT_RT kernel patch..."
  wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/4.19/older/patch-4.19-rt1.patch.gz
  gunzip patch-4.19-rt1.patch.gz
  cd linux || exit 1
  patch -p1 < ../patch-4.19-rt1.patch

  # Configure RT kernel
  KERNEL_CONFIG=arch/arm/configs/zynq_pluto_defconfig
  echo "CONFIG_TUN=y" >> $KERNEL_CONFIG
  echo "CONFIG_PREEMPT_RT_FULL=y" >> $KERNEL_CONFIG
else
  echo "plutosdr-fw already seems to be configured."
fi


## Init rootfs overlay structure
PLUTOSDR_FW_ROOTOVERLAY=$SCRIPT_DIR/plutosdr-fw/rootfs-overlay
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

cd $SCRIPT_DIR || exit 1

## Create sysroot
./create_sysroot.sh || exit 1

# Copy libfec and liquid-dsp to rootfs overlay. These libs are not built with buildroot
# so we have to manually copy them.
cp $PLUTO_SYSROOT_DIR/usr/lib/libliquid.so $PLUTOSDR_FW_ROOTOVERLAY/usr/lib/
cp $PLUTO_SYSROOT_DIR/usr/lib/libfec.so $PLUTOSDR_FW_ROOTOVERLAY/usr/lib

## Build Main Apps
echo "Building main application..."
cd $SCRIPT_DIR/../ || exit 1
if [[ ! -d cmake-build ]]
then
    mkdir cmake-build
fi
cd cmake-build || exit 1
cmake -DCMAKE_TOOLCHAIN_FILE=CmakeArmToolchain.cmake ..
make basestation
make client
make client-calib
cp basestation $PLUTOSDR_FW_ROOTOVERLAY/root/
cp client $PLUTOSDR_FW_ROOTOVERLAY/root/
cp client-calib $PLUTOSDR_FW_ROOTOVERLAY/root/
cd ..

# Create VERSION file
touch $PLUTOSDR_FW_ROOTOVERLAY/root/VERSION
echo "hnap "`git describe --tags` > $PLUTOSDR_FW_ROOTOVERLAY/root/VERSION
echo "plutosdr-fw "$PLUTOSDR_FW_VERSION >> $PLUTOSDR_FW_ROOTOVERLAY/root/VERSION

## Copy network init script to rootfs
echo "Copy network autoconfig script to rootfs..."
cp startup_scripts/* $PLUTOSDR_FW_ROOTOVERLAY/etc/init.d/


## Copy FIR filter to rootfs
echo "Copy FIR filter coefficients to rootfs..."
cp AD9361_256kSPS.ftr $PLUTOSDR_FW_ROOTOVERLAY/root/

## Copy the default configuration file to rootfs
echo "Copy config.txt to rootfs..."
cp config.txt $PLUTOSDR_FW_ROOTOVERLAY/root/

cd $SCRIPT_DIR/plutosdr-fw/ || exit 1
make || exit 1
cp build/pluto.frm ../