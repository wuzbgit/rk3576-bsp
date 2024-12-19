#!/bin/bash -e

TARGET_IMG=boot.img
ITS=boot.its
KERNEL_IMG=arch/arm64/boot/Image
KERNEL_DTB=arch/arm64/boot/dts/rockchip/tl3576-evm.dtb
RESOURCE_IMG=resource.img

if [ ! -f "$TARGET_IMG" ]; then
	echo "$TARGET_IMG not exists!"
	exit 1
fi

if [ ! -f "$ITS" ]; then
	echo "$ITS not exists!"
	exit 1
fi

if [ ! -f "$KERNEL_IMG" ]; then
	echo "$KERNEL_IMG not exists!"
	exit 1
fi

if [ ! -f "$KERNEL_DTB" ]; then
	echo "$KERNEL_DTB not exists!"
	exit 1
fi

if [ ! -f "$RESOURCE_IMG" ]; then
	echo "$RESOURCE_IMG not exists!"
	exit 1
fi

sed -i -e "s~@KERNEL_DTB@~$(realpath -q "$KERNEL_DTB")~" \
	-e "s~@KERNEL_IMG@~$(realpath -q "$KERNEL_IMG")~" \
	-e "s~@RESOURCE_IMG@~$(realpath -q "$RESOURCE_IMG")~" "$ITS"

rkbin/tools/mkimage -f "$ITS"  -E -p 0x800 "$TARGET_IMG"

