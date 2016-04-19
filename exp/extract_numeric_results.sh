#!/bin/bash

exp="VA_LP"
benches="n100_soft n200_soft n300_soft"
#exp="clustering"
#benches="ibm01 ibm03 ibm07"
runs=20


for die_count in 2 3 4
do
	dies=$die_count"dies"

	for bench in $benches
	do

		file=$bench"_"$exp"_"$dies".results"

# generate header, reset previous file
		echo "Run Deadspace Outline Power HPWL Power_HPWL TSVs Power_TSVs TSVs/Island Deadspace/TSVs Critical_Delay Temp Runtime" > $file

		for (( run = 1; run <= $runs; run++ ))
		do
# memorize all result parameters in variable, reset variable w/ run
			result="$run"
# Deadspace
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '4 p' -n`
# Outline
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '9 p' -n`
# Power
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '11 p' -n`
# HPWL
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '13 p' -n`
# Power for HPWL
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '15 p' -n`
# TSVs
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '17 p' -n`
# Power for TSVs
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '18 p' -n`
# Avg TSVs per Island
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '21 p' -n`
# Use of Deadspace by TSVs
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '22 p' -n`
# Critical Delay
			result=$result"	"`rev $dies/$exp/$run/$bench'.results' | cut -d ' ' -f 1 | rev | sed '33 p' -n`
# HotSpot Temp
			result=$result"	"`tail -n 1 $dies/$exp/$run/$bench'_HotSpot.txt' | rev | cut -f 1 | rev`
# Runtime
			result=$result"	"`tail -n 1 $dies/$exp/$run/$bench'.results' | rev | cut -d ' ' -f 2 | rev`

# push variable as new line into file, append mode
			echo $result >> $file
		done

	done
done
