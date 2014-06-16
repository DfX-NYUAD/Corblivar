#!/bin/bash
root=~/code/Corblivar
base=$root/exp
fitting=thermal_analysis_fitting

exp="3DFP-Corb"
benches="ami33 xerox"
runs=15

for die_count in 2
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
	cp $base/HotSpot.sh .
	ln -s $base/benches .
	cp $base/gp.sh .

	# HotSpot, thermal fitting related; local copy for thermal fitting
	for bench in $benches
	do
		cp $base/configs/$dies/$exp/$bench.conf .
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
