#!/usr/bin/env bash

arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/catlink-tracker-firmware catlink-tracker-firmware.bin
st-flash --connect-under-reset write catlink-tracker-firmware.bin 0x08000000
