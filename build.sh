#!/bin/sh

BUILD_DIR=build-pico
BUILD_DIR2=build-pico2

PICO_SDK_DIR=pico-sdk

main () {
	if [ ! -f "$PICO_SDK_DIR/.git" ]; then
		git submodule sync --recursive
		git submodule update --init --recursive
	fi

	mkdir -p $BUILD_DIR
	cmake -B $BUILD_DIR
	make -C $BUILD_DIR

	mkdir -p $BUILD_DIR2
	cmake -B $BUILD_DIR2 -DPICO_BOARD=pico2
	make -C $BUILD_DIR2

}

main $@
