#!/bin/bash

root=~/github/_OLD/Corblivar
base=$root/exp

fitting=thermal_analysis_octave
wait_for_octave=30

exp="regular"

benches=$1
benches="n200_soft"

runs=$2
runs=1

dies_start=$3
dies_start=2
dies_stop=$4
dies_stop=2

only_runs=1

for die_count in $(seq -w $dies_start $dies_stop)
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
	cp -ra $root/$fitting .
	ln -sf $base/hotspot*.config $fitting/
	ln -sf $base/HotSpot.sh .
	ln -sf $base/benches .
	ln -sf $base/gp.sh .
	ln -sf $base/clean.sh .

	# local copy of config files
	for bench in $benches
	do
		cp $base/configs/$dies/$exp/$bench.conf .
		cp $base/configs/$dies/$exp/$bench.tech.conf .
	done

	if [[ $only_runs != 1 ]]; then

		cd $fitting

		for bench in $benches
		do
			# only proceed in case no other octave instance is still running, as the script does not cope well with that
			# note that 1 process will always be there, which is the grep process
			while [[ `ps aux | grep 'octave optimization.m' | wc -l` > 1 ]]
			do
				echo "another instance of 'octave optimization.m' is still running: `ps aux | grep 'octave optimization.m' | head -n 1`"
				echo "retrying in $wait_for_octave ..."
				sleep $wait_for_octave
			done

			# (TODO) provide 0(%) as further, final parameter to ignore all TSVs; provide any other number to override actual TSVs with regular TSV structure of given density,
# ranging from 0 to 100(%)
			octave optimization.m $bench $base/$dies/$exp/$bench.conf $root
		done
	fi

	cd $base/$dies/$exp

	# init experiments done
	#

	for run in $(seq -w 1 $runs)
	do

		if [ ! -d "$run" ]; then
			mkdir $run
		fi

		cd $run

		#
		# we reached the experiments folder
		#

		# link aux scripts
		ln -sf $base/gp.sh .
		ln -sf $base/clean.sh .
		ln -sf $base/HotSpot.sh .
		ln -sf $base/hotspot*.config .

		# perform experiments; run Corblivar for each benchmark
		for bench in $benches
		do
			dir=`pwd`

			echo "running Corblivar for $bench on $dies; run $run; working dir: $dir"

			# use local config; configured w/ thermal fitting
			# also wrap gdb in batch mode, which helps to log any segfault's origin
			gdb -batch -ex=run -ex=backtrace --args $root/Corblivar $bench $base/$dies/$exp/$bench.conf $base/benches/ | tee $bench.log

			# run individual aux scripts, if required
			#
			./HotSpot.sh $bench $die_count
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
