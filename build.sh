#!/bin/sh

if [ -z "$thirdparty_manage_path" ]; then
    echo "thirdparty_manage_path no set"
    echo "please run command : source \$thirdparty_manage_path/env_thirdparty_manage.sh"
    exit -1
fi


mkdir build
cd build

cmake ../ -DCMAKE_INSTALL_PREFIX=$(pwd)/../install \
            -DCMAKE_PREFIX_PATH=$thirdparty_manage_path 



make -j4 
make install