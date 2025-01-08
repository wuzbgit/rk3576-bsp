#!/bin/sh
make -j2 CROSS_COMPILE=aarch64-none-linux-gnu- ARCH=arm64 tl3576-evm.img
