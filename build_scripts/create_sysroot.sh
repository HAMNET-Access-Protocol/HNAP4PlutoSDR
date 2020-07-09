#!/bin/bash

# This script customizes the plutosdr-fw sysroot image to our needs.
# NOTE: you only need this script if you want to rebuild the sysroot
#       folder, i.e. after a new release of the plutosdr-fw!
#       Usually it is enough to download an already customized sysroot
#       from the releases page.


#Check if env variable for Path to pluto sysroot directory exists
: "${PLUTO_SYSROOT_DIR?Need to set PLUTO_SYSROOT_DIR}"

SCRIPT_DIR=`pwd`/`dirname "$0"`
cd $SCRIPT_DIR || exit 1

mkdir .tmp_create_sysroot
cd .tmp_create_sysroot || exit 1

echo "Downloading fftw3"
wget http://www.fftw.org/fftw-3.3.8.tar.gz
tar -xzf fftw-3.3.8.tar.gz

echo "Installing fftw3"
cd  fftw-3.3.8 || exit 1
./configure arm --enable-float --enable-shared --host=arm-linux-gnueabihf --prefix="$PLUTO_SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$PLUTO_SYSROOT_DIR/"
make install

echo "Downloading libconfig"
wget https://hyperrealm.github.io/libconfig/dist/libconfig-1.7.2.tar.gz
tar -xzf libconfig-1.7.2.tar.gz
echo "Installing libconfig"
cd libconfig-1.7.2 || exit 1
./configure --host=arm-linux-gnueabihf --prefix="$PLUTO_SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$PLUTO_SYSROOT_DIR/"
make install

cd $SCRIPT_DIR/../libfec || exit 1
echo "Installing libfec..."
./configure arm --host=arm-linux-gnueabihf --prefix="$PLUTO_SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$PLUTO_SYSROOT_DIR/"
make install

echo "Installing liquid-dsp..."
cd ../liquid-dsp || exit 1
./bootstrap.sh
./configure arm --host=arm-linux-gnueabihf  --prefix="$PLUTO_SYSROOT_DIR/usr/" CC="arm-linux-gnueabihf-gcc --sysroot=$PLUTO_SYSROOT_DIR"
# Configure somehow does not correctly identify malloc and realloc
# and redefines them in the generated config.h script as rpl_malloc.
# This makes the code unusable, therefore we delete this:
sed -i '/rpl_realloc/d' config.h
sed -i '/rpl_malloc/d' config.h

make install

echo "Cleaning up..."
cd $SCRIPT_DIR || exit 1
rm -R .tmp_create_sysroot

SYSROOT_NAME="sysroot-custom-"`git describe --tags`".tar.gz"
tar -czf $SYSROOT_NAME $PLUTO_SYSROOT_DIR
echo "Created sysroot archive "$SYSROOT_NAME
exit 0