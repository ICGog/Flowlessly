#!/bin/bash

mkdir -p ext
cd ext
EXT_DIR=${PWD}

################################################################################

function get_dep_wget {
  URL=$2
  NAME=$1
  FILE=$(basename ${URL})
  wget -q -N ${URL}
  if [ ${NAME}-timestamp -ot ${FILE} ]
  then
    touch -r ${FILE} ${NAME}-timestamp
  fi
}

################################################################################

LIB_GTEST=${EXT_DIR}/gtest-1.7.0/lib/.libs/libgtest.a
if [[ ! -f ${LIB_GTEST} ]]; then
    get_dep_wget "gtest" "https://googletest.googlecode.com/files/gtest-1.7.0.zip"
    unzip gtest-1.7.0.zip
    rm gtest-1.7.0.zip
    cd gtest-1.7.0/ ; ./configure ; make
    echo "Please install libgtest using the following command: "
    echo "$ cd ${EXT_DIR}/gtest-1.7.0"
    echo "$ sudo cp -a include/gtest/ /usr/include/ ; sudo cp -a lib/.libs/* /usr/lib/ ; sudo ldconfig"
    echo "Please rerun the fetch-externals script after instalation"
    exit 1
fi
