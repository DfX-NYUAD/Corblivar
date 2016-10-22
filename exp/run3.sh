#!/bin/bash
root=~/code/Corblivar
base=$root/exp
fitting=thermal_analysis_fitting

exp="3D-STAF"
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

	# init experiments, if required
	#

	# HotSpot, thermal fitting related
	cp -r $base/$fitting .
	cp $base/hotspot*.config $fitting/
	cp $base/HotSpot.sh .
	ln -s $base/benches .
	cp $base/gp.sh .

	# HotSpot, thermal fitting related; local copy for thermal fitting
	for bench in $benches
	do
		cp $base/configs/$dies/$exp/$bench.conf .
	done

	# parse 3rd party results
	for bench in $benches
	do
		cp ~/code/3D-STAF-FP/results/$dies/DS_0.1/$bench.log .
		$root/3DSTAF_Parser $bench $bench.conf $base/benches/ > $bench.Corblivar.log

		# retrieve outline from 3DSTAF results; parse into Corblivar.conf
		x=`cat $bench.Corblivar.log | grep 'x =' | awk '{print $4}'`
		y=`cat $bench.Corblivar.log | grep 'y =' | awk '{print $4}'`
		# this sed expr replaces only the first occurance of '10000' each time
		sed -i "0,/10000/{s/10000/$x/}" $bench.conf
		sed -i "0,/10000/{s/10000/$y/}" $bench.conf
	done

	cd $fitting

	for bench in $benches
	do
		octave optimization.m $bench $bench.conf
	done

	cd $base/$dies/$exp

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
		cp $base/hotspot*.config .

		# perform experiments; run Corblivar for each benchmark
		for bench in $benches
		do
			dir=`pwd`

			echo "running Corblivar for $bench on $dies; run $run; working dir: $dir"

			# use local config; configured w/ thermal fitting
			$root/Corblivar $bench $base/$dies/$exp/$bench.conf $base/benches/ > $bench.log

			# run individual aux scripts, if required
			#
			./HotSpot.sh $bench $die_count
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
