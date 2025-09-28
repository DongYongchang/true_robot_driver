#!/bin/sh

if [ "${1}" == "clear" ];then
  echo "Clear ...... "
  rm -rf build
  rm -rf install
elif [ "${1}" == "init" ];then
  echo "Init ...... "
  git submodule init
  git submodule update
elif [ "${1}" == "uinit" ];then
  echo "Delete ...... "
  rm -rf build
  rm -rf install
else
  echo "Building ..."
  mkdir build
  cd build

  # exit on failure
  set -e


  cmake ../ \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=$(pwd)/../../../install

  #cmake --build . --config Release --target install

  ##################################################################################
  # Exprot MBuild tool path on windwos platform
  ##################################################################################
  OS_TYPE=$(uname -s)

  case "$OS_TYPE" in
      Linux*)
          echo "Linux Platform"
          make -j4
          # make install

          ;;
      Darwin*)
          echo "macOS Platform"
          # macOS系统上的命令或操作
          ;;
      CYGWIN*|MINGW32*|MSYS*|MINGW*)
          echo "Windows Platform"

          "$thirdparty_manage_mbuild_tool_path" *.sln -maxcpucount:3
          #"$thirdparty_manage_mbuild_tool_path"  INSTALL.vcxproj
          
          ;;
      *)
          echo "Unknow Platform: $OS_TYPE"
          # 其他未知系统的命令或操作
          ;;
  esac

  cd ..
fi


echo "Done"