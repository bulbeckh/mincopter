#!/bin/bash

: '
if [ -d "./build/" ]; then
	echo "build dir exists - removing contents"
	rm -rf ./build/*
else
	echo "build dir does not exist - creating"
	mkdir ./build/
fi

## Copy configuration
cp ./mc-config.cmake ./build

'
## Run for each target

cd ./build

## ARM Targets
target_list=("stm32f407xx"
	"stm32f405xx"
	"stm32f401xe"
	"stm32f401xc")

: '
for target in "${target_list[@]}"; do
	echo ${target}
	echo "./build-${target}"

	mkdir "./build-${target}"
	cp ./mc-config.cmake "./build-${target}"
	cd "./build-${target}"

	cmake ../.. -DTARGET_ARCH="${target}"

	make -j4 2>&1 | tee -a makeout

	arm-none-eabi-size ./bin/*

	cd ..
done
'

## AVR Targets

avr_list=("atmega2560"
	"atmega2561"
	"atmega1280"
	"atmega1281")

: '
for avr_target in "${avr_list[@]}"; do
	echo ${avr_target}
	echo "./build-${avr_target}"

	mkdir "./build-${avr_target}"

	cp ./mc-config.cmake "./build-${avr_target}"
	cd "./build-${avr_target}"

	cmake ../.. -DTARGET_ARCH="${avr_target}"
	make -j4 2>&1 | tee -a makeout

	avr-size ./bin/*

	cd ..
done
'

## Remove root dir README
rm ../README.md
cp ../docs/README.template.md ../README.md

full_list=("${avr_list[@]}" "${target_list[@]}")
for target in "${full_list[@]}"; do
	if [[ -f "./build-${target}/bin/mincopter" ]]; then
		echo "passed ${target}"
		sed -i -e "s|{{${target}_test}}| passed|" ../README.md

		tgt_size=$(size "./build-${target}/bin/mincopter" | tail -n1 | awk '{print $(NF-2)}' | numfmt --to=iec)
		sed -i -e "s|{{${target}_size}} | ${tgt_size} |" ../README.md
	else
		echo "failed ${target}"
		sed -i -e "s|{{${target}_test}}| failed|" ../README.md
		sed -i -e "s|{{${target}_size}} | - |" ../README.md
	fi

	## Get number of binaries built
	tgt_count=$(ls -lF ./build-${target}/bin | grep '\*' | wc -l)
	sed -i -e "s|{{${target}_target_count}}|${tgt_count}|" ../README.md

	## Get number of warnings and error
	warning_count=$(grep -o -w "warning:" "./build-${target}/makeout" | wc -l)
	err_count=$(grep -o -w "error:" "./build-${target}/makeout" | wc -l)
	sed -i -e "s|{{${target}_warnings}}|${warning_count}|" ../README.md
	sed -i -e "s|{{${target}_errors}}|${err_count}|" ../README.md
done


