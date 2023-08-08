#!/bin/bash
#
# Copyright (c) 2023 - JMPFBMX
#

export LC_ALL=C
export ARCH=arm64

# Get the absolute paths for CLANG_PATH and GCC_PATH
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_OUT="${SCRIPT_DIR}/../KERNEL_OUT"
GCC_PATH="${SCRIPT_DIR}/../prebuilt/gcc/"

# Clone Toolchains if they don't exist
if [ ! -d "$GCC_PATH" ]; then
    git clone -b marshmallow-release --depth=1 https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9 "$GCC_PATH"
fi

# Clean previous build and create output directory
if [ -d "../KERNEL_OUT" ]; then
    rm -rf "$KERNEL_OUT"
    mkdir -p "$KERNEL_OUT"
fi


make -C . O=../KERNEL_OUT ARCH=arm64 CROSS_COMPILE="${GCC_PATH}" nikel_debug_defconfig
make O=../KERNEL_OUT/ -C . ARCH=arm64 CROSS_COMPILE="${GCC_PATH}/bin/aarch64-linux-android-" -j$(nproc --all)

