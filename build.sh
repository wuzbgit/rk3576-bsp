#!/bin/sh
make -j1 CROSS_COMPILE=aarch64-none-linux-gnu- ARCH=arm64 tl3576-evm.img
