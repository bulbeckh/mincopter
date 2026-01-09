#!/bin/bash

# This script is intended to test a subset of the full list of supported boards, across each of the
# architectures (avr, arm, linux, ...)
#
# We use the following for testing
#
# avr: avr128da64, avr128db64
# arm: stm32f405xx, stm32f407xx
# linux: rpi
#

if [ -d "./build-subset/" ]; then
	echo "build dir exists - removing contents"
	rm -rf ./build-subset/*
else
	echo "build dir does not exist - creating"
	mkdir ./build-subset/
fi

## Copy configuration
cp ./mc-config.cmake ./build-subset

## Run for each target
cd ./build-subset

## ARM Targets
arm_list=("stm32f407xx" "stm32f405xx")

for target in "${arm_list[@]}"; do
	echo ${target}

	mkdir "./build-subset-${target}"
	cp ./mc-config.cmake "./build-subset-${target}"
	cd "./build-subset-${target}"

	cmake ../.. -DTARGET_ARCH="${target}"

	make -j4 2>&1 | tee -a makeout

	arm-none-eabi-size ./bin/*

	cd ..
done

## AVR Targets
avr_list=("avr128db64" "avr128da64")

for avr_target in "${avr_list[@]}"; do
	echo ${avr_target}

	mkdir "./build-subset-${avr_target}"

	cp ./mc-config.cmake "./build-subset-${avr_target}"
	cd "./build-subset-${avr_target}"

	cmake ../.. -DTARGET_ARCH="${avr_target}"
	make -j4 2>&1 | tee -a makeout

	avr-size ./bin/*

	cd ..
done

## Linux Targets
linux_list=("rpi2modelb" "generic")

for linux_target in "${linux_list[@]}"; do
	echo ${linux_target}

	mkdir "./build-subset-${linux_target}"

	cp ./mc-config.cmake "./build-subset-${linux_target}"
	cd "./build-subset-${linux_target}"

	cmake ../.. -DTARGET_ARCH="${linux_target}"
	make -j4 2>&1 | tee -a makeout

	size ./bin/*
	cd ..
done


full_list=("${avr_list[@]}" "${arm_list[@]}" "${linux_list[@]}")
for target in "${full_list[@]}"; do

	cd "./build-subset-${target}"
	echo "${target}:	"

	size ./bin/*

	cd ..

done


