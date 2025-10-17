#!/bin/bash

if [ -d "./build/" ]; then
	echo "build dir exists - removing contents"
	rm -rf ./build/*
else
	echo "build dir does not exist - creating"
	mkdir ./build/
fi

## Copy configuration
cp ./mc-config.cmake ./build

## Run for each target

cd ./build

## ARM Targets
target_list=("stm32f407xx"
	"stm32f405xx"
	"stm32f401xe"
	"stm32f401xc")
for target in "${target_list[@]}"; do
	echo ${target}
	echo "./build-${target}"

	mkdir "./build-${target}"
	cp ./mc-config.cmake "./build-${target}"
	cd "./build-${target}"

	cmake ../.. -DTARGET_ARCH="${target}"

	make -j4

	arm-none-eabi-size ./bin/*

	cd ..
done

## AVR Targets

avr_list=("atmega2560"
	"atmega2561"
	"atmega1280"
	"atmega1281")
for avr_target in "${avr_list[@]}"; do
	echo ${avr_target}
	echo "./build-${avr_target}"

	mkdir "./build-${avr_target}"

	cp ./mc-config.cmake "./build-${avr_target}"
	cd "./build-${avr_target}"

	cmake ../.. -DTARGET_ARCH="${avr_target}"
	make -j4

	avr-size ./bin/*

	cd ..
done



