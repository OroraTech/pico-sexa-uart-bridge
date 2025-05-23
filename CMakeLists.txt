# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(pico_uart_i2c_bridge)

pico_sdk_init()

add_executable(uart_i2c_bridge uart-i2c-bridge.c usb-descriptors.c)

pico_generate_pio_header(uart_i2c_bridge ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)
pico_generate_pio_header(uart_i2c_bridge ${CMAKE_CURRENT_LIST_DIR}/uart_tx.pio)

target_include_directories(uart_i2c_bridge PUBLIC
	./
	../pico-sdk/lib/tinyusb/src)

target_link_libraries(uart_i2c_bridge
	pico_multicore
	pico_stdlib
	hardware_flash
	hardware_i2c
	tinyusb_device
	tinyusb_board
        hardware_pio)

pico_add_extra_outputs(uart_i2c_bridge)
