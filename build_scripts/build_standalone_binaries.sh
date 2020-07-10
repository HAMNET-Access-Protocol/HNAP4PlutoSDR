#!/bin/bash

# Script builds the standalone applications

SCRIPT_DIR=`pwd`/`dirname "$0"`
BUILD_DIR=$SCRIPT_DIR/build
SYSROOT_DIR=$BUILD_DIR/pluto-custom.sysroot
SRC_DIR=$SCRIPT_DIR/..

echo "Building main application..."
mkdir -p $BUILD_DIR/cmake-build

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
