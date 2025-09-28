#!/bin/sh

mkdir build
cd build

cmake ../ -DCMAKE_INSTALL_PREFIX=$(pwd)/../install \

make -j4 
make install