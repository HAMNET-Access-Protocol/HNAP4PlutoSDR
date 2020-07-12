#!/bin/bash

# Script builds the standalone applications

SCRIPT_DIR=`pwd`/`dirname "$0"`
BUILD_DIR=$SCRIPT_DIR/build
SYSROOT_DIR=$BUILD_DIR/pluto-custom.sysroot
SRC_DIR=$SCRIPT_DIR/..

# Stop on first error
set -e

echo "Building main application..."
mkdir -p $BUILD_DIR/cmake-build

# force a clean build; force cmake to re-read the toolchain parameters
rm -rf $BUILD_DIR/cmake-build/CMakeFiles $BUILD_DIR/cmake-build/CMakeCache.txt

# Ensure that the toolchain is present
arm-linux-gnueabihf-gcc --version
if [ $? != 0 ]
then
  echo "Could not find the cross-compiler. Make sure that it can be found in the PATH variable"
  exit 1
fi

if [[ ! -d $SYSROOT_DIR ]]
then
  ./create_sysroot.sh
else
  echo "Sysroot is present."
fi

cd $BUILD_DIR/cmake-build || exit 1
export PLUTO_SYSROOT_DIR=$SYSROOT_DIR
cmake -DCMAKE_TOOLCHAIN_FILE=$SRC_DIR/CmakeArmToolchain.cmake $SRC_DIR
make basestation
make client
make client-calib

cp basestation client client-calib $BUILD_DIR/
echo "Standalone applications have been created in $BUILD_DIR"
