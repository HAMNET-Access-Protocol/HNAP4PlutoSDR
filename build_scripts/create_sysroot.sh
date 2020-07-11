#!/bin/bash

# This script customizes the plutosdr-fw sysroot image to our needs.
# NOTE: you only need this script if you want to rebuild the sysroot
#       folder, i.e. after a new release of the plutosdr-fw!
#       The script is automatically invoked by the other build scripts

# Required plutosdr-fw version
PLUTOSDR_FW_TAG="v0.32"


SCRIPT_DIR=`pwd`/`dirname "$0"`
BUILD_DIR=$SCRIPT_DIR/build
PLUTOSDR_FW_DIR=$BUILD_DIR/plutosdr-fw
SYSROOT_DIR=$BUILD_DIR/pluto-custom.sysroot
SRC_DIR=$SCRIPT_DIR/..

mkdir -p $BUILD_DIR
cd $BUILD_DIR || exit 1
mkdir -p .tmp_create_sysroot
cd .tmp_create_sysroot || exit 1

# Read current build status
BUILD_STATUS=$BUILD_DIR/build_status
touch $BUILD_STATUS
source $BUILD_STATUS || exit 1

if [[ $STATUS_DOWNLOAD_SYSROOT != "y" ]]
then
  echo "Could not locate existing sysroot dir. Downloading sources"
  wget https://github.com/analogdevicesinc/plutosdr-fw/releases/download/$PLUTOSDR_FW_TAG/sysroot-$PLUTOSDR_FW_TAG.tar.gz
  tar -xzf sysroot-$PLUTOSDR_FW_TAG.tar.gz || exit 1
  mv staging pluto-custom.sysroot
  mv pluto-custom.sysroot $BUILD_DIR/
  echo "STATUS_DOWNLOAD_SYSROOT=y" >> $BUILD_STATUS
else
  echo "Found existing sysroot folder. Skip download."
fi

echo "Bulding libraries"

if [[ $STATUS_SYSROOT_GOT_FFTW3F != "y" ]]
then
  echo "Downloading fftw3"
  wget http://www.fftw.org/fftw-3.3.8.tar.gz
  tar -xzf fftw-3.3.8.tar.gz

  echo "Installing fftw3"
  cd  fftw-3.3.8 || exit 1
  ./configure arm --enable-float --enable-shared --host=arm-linux-gnueabihf --prefix="$SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$SYSROOT_DIR/"
  make install || exit 1
  echo "STATUS_SYSROOT_GOT_FFTW3F=y" >> $BUILD_STATUS
else
  echo "FFTW3 is already present"
fi

if [[ $STATUS_SYSROOT_GOT_LIBCONFIG != "y" ]]
then
  echo "Downloading libconfig"
  wget https://hyperrealm.github.io/libconfig/dist/libconfig-1.7.2.tar.gz
  tar -xzf libconfig-1.7.2.tar.gz
  echo "Installing libconfig"
  cd libconfig-1.7.2 || exit 1
  ./configure --host=arm-linux-gnueabihf --prefix="$SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$SYSROOT_DIR/"
  make install || exit 1
  echo "STATUS_SYSROOT_GOT_LIBCONFIG=y" >> $BUILD_STATUS
else
  echo "Libconfig is already present"
fi

if [[ $STATUS_SYSROOT_GOT_LIBFEC != "y" ]]
then
  cd $SRC_DIR/libfec || exit 1
  echo "Installing libfec..."
  ./configure arm --host=arm-linux-gnueabihf --prefix="$SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$SYSROOT_DIR/"
  make install
  echo "STATUS_SYSROOT_GOT_LIBFEC=y" >> $BUILD_STATUS
else
  echo "Libfec is already present"
fi

if [[ $STATUS_SYSROOT_GOT_LIBLIQUID != "y" ]]
then
  echo "Installing liquid-dsp..."
  cd $SRC_DIR/liquid-dsp || exit 1
  ./bootstrap.sh
  ./configure arm --host=arm-linux-gnueabihf  --prefix="$SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$SYSROOT_DIR"
  # Configure somehow does not correctly identify malloc and realloc
  # and redefines them in the generated config.h script as rpl_malloc.
  # This makes the code unusable, therefore we delete this:
  sed -i '/rpl_realloc/d' config.h
  sed -i '/rpl_malloc/d' config.h

  make install
  echo "STATUS_SYSROOT_GOT_LIBLIQUID=y" >> $BUILD_STATUS
else
  echo "Liquid-dsp is already present"
fi

echo "Cleaning up..."
cd $BUILD_DIR || exit 1
rm -R .tmp_create_sysroot

SYSROOT_NAME="pluto-custom-"`git describe --tags`".sysroot.tar.gz"
tar -czf $SYSROOT_NAME $SYSROOT_DIR
echo "Created sysroot archive $SYSROOT_NAME in $BUILD_DIR"
