#!/bin/bash

# Edge Impulse ingestion SDK
# Copyright (c) 2022 EdgeImpulse Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

set -e

PROJECT_FILE=firmware-xg24.slcp

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
OUTPUT_DIR=${SCRIPTPATH}/generated/
PROJECT_NAME=$(grep project_name ${PROJECT_FILE} | cut -d':' -f2 | xargs)
SLC_BIN="slc"

for i in "$@"; do
  case $i in
    -b|--build)
      BUILD=1
      shift # past argument
      ;;
    -c|--clean)
      CLEAN=1
      shift # past argument
      ;;
    -f|--flash)
      FLASH=1
      shift # past argument
      ;;
    -s=*|--slc=*)
      SLC_BIN="${i#*=}"
      shift # past argument=value
      ;;
    *)
      shift # past argument
      ;;
  esac
done

if [ ! -z ${CLEAN} ];
then
    rm -rf ${OUTPUT_DIR}
fi

if [ ! -z ${BUILD} ];
then
    ${SLC_BIN} generate ${PROJECT_FILE} -d ${OUTPUT_DIR} -cp -np --toolchain gcc
    cd ${OUTPUT_DIR}
    make -j -f ${PROJECT_NAME}.Makefile
    cd -
fi

if [ ! -z ${FLASH} ];
then
    commander flash ${OUTPUT_DIR}/build/debug/${PROJECT_NAME}.hex
fi
