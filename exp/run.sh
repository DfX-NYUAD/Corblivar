#!/bin/bash
root=~/code/Corblivar
base=$root/exp

exp="basic"
benches="ami33 hp xerox n100 n200 n300 ibm01 ibm03 ibm04 ibm07"
runs=5

for dies in 2dies 3dies
do

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

			$root/Corblivar $bench $base/configs/$dies/$exp/$bench.conf $base/benches/ > $bench.log

			# run individual aux scripts
			#
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
