#!/bin/bash
root=~/github/Corblivar
base=$root/exp
fitting=$root/thermal_analysis_octave

exp="THERMAL_OPT"
benches="n100 n200 n300"
runs=15

for die_count in 2 3
do
	dies=$die_count"dies"

	# switch to experiments folder; create if required

	cd $base

	if [ ! -d "$dies" ]; then
		mkdir $dies
	fi

	cd $dies

	if [ ! -d "$exp" ]; then
		mkdir $exp
	fi

	cd $exp

	cp $root/Corblivar .

	mkdir exp
	mkdir exp/benches

	for bench in $benches
	do
		cp $base/benches/$bench.* exp/benches/
		cp $base/configs/$dies/$exp/* exp/
		# for this setup, ignore MI
		rm exp/benches/$bench".alr"
	done

	# HotSpot, thermal fitting related
	mkdir octave_scripts
	cp $fitting/*.m octave_scripts/
	cp $base/HotSpot.sh exp/
	cp $base/gp.sh exp/

	cd octave_scripts

	for bench in $benches
	do
		hotspot_result="../exp/"$bench"_TSV_dens_0__thermal_analysis_fitting/"$bench"_HotSpot.steady"

		while [ ! -f "$hotspot_result" ]
		do
			octave optimization.m $bench ../exp/$bench.conf
		done
	done

	cd ..

	# init experiments done
	#

	for (( run = 1; run <= $runs; run++ ))
	do

		if [ ! -d "$run" ]; then
			mkdir $run
		fi

		cd $run

		#
		# we reached the experiments folder
		#

		# copy aux scripts
		cp $base/gp.sh .
		cp $base/clean.sh .
		cp $base/HotSpot.sh .

		# perform experiments; run Corblivar for each benchmark
		for bench in $benches
		do
			dir=`pwd`

			echo "running Corblivar for $bench on $dies; run $run; working dir: $dir"

			$root/Corblivar $bench $base/$dies/$exp/exp/$bench.conf	$base/$dies/$exp/exp/benches/ > $bench.log

			# run individual aux scripts, if required
			#
			./HotSpot.sh $bench $die_count
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
