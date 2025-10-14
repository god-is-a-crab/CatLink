#!/usr/bin/env bash

arm-none-eabi-objdump -d -S target/thumbv7em-none-eabihf/release/catlink-tracker-firmware | rustfilt > firmware.asm
arm-none-eabi-size -A target/thumbv7em-none-eabihf/release/catlink-tracker-firmware
