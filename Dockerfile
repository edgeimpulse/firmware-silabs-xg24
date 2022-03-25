FROM ubuntu:20.04

WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

# APT packages
RUN apt update && apt install -y wget unzip python3 python3-pip openjdk-16-jre-headless git-lfs

# GCC ARM
RUN cd / && \
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 -q && \
    tar xjf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    echo "PATH=$PATH:/gcc-arm-none-eabi-9-2019-q4-major/bin" >> ~/.bashrc

# Gecko SDK
RUN GECKO_COMMIT=db4e90767174d467158d4c5249ba5be6ab9d9e83 && \
    cd / && \
    wget https://github.com/edgeimpulse/gecko_sdk/archive/${GECKO_COMMIT}.zip -O gecko.zip -q && \
    unzip -q gecko.zip && \
    mv gecko_sdk-${GECKO_COMMIT}/ gecko_sdk/

# SLC-CLI tool
RUN cd / && \
    wget https://cdn.edgeimpulse.com/build-system/slc_cli_linux_xg24.zip -q && \
    unzip -q slc_cli_linux_xg24.zip && \
    cd slc_cli && \
    pip3 install --user -r requirements.txt && \
    chmod +x slc

ENV PATH="/slc_cli:${PATH}"

RUN slc configuration --sdk /gecko_sdk/ && \
    slc signature trust --sdk /gecko_sdk/ && \ 
    slc configuration --gcc-toolchain /gcc-arm-none-eabi-9-2019-q4-major
