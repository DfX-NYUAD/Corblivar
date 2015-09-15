#!/bin/bash
root=~/github/Corblivar
base=$root/exp
fitting=thermal_analysis_octave

exp="regular"
benches=$1
#benches=$2
#benches="n100 n200 n300"
#benches="ami33 xerox"
runs=20

for die_count in 2 3
#for die_count in $1
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
	cp -r $root/$fitting .
	cp $base/HotSpot.sh .
	ln -s $base/benches .
	cp $base/gp.sh .

	# local copy of config files
	for bench in $benches
	do
		cp $base/configs/$dies/$exp/*.conf* .
	done

	cd $fitting

	for bench in $benches
	do
		octave optimization.m $bench $base/$dies/$exp/$bench.conf $root
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
