#!/bin/bash

CURDIR=`dirname $0`;
echo -e "\033[0;32mClean: $(readlink -f $CURDIR) \033[0m";

rm -rf ${CURDIR}/build/ \
       ${CURDIR}/bin/ \
       ${CURDIR}/CMakeCache.txt \
       ${CURDIR}/CMakeFiles/ \
       ${CURDIR}/cmake_install.cmake \
       ${CURDIR}/.project \
       ${CURDIR}/.cproject \
       ${CURDIR}/Makefile \
       ${CURDIR}/test/pc_error 
  
