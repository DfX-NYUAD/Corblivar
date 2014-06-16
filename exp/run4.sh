#!/bin/bash
root=~/code/Corblivar
base=$root/exp

exp="packing_2"
benches="n100 n200 n300"
runs=25

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

	for bench in $benches
	do
		cp $base/benches/$bench.* .
		cp $base/configs/$dies/$exp/$bench.* .
	done

	for (( run = 1; run <= $runs; run++ ))
	do

		if [ ! -d "$run" ]; then
			mkdir $run
		fi

		cd $run

		#
		# we reached the experiments folder
		#

		# perform experiments; run Corblivar for each benchmark
		for bench in $benches
		do
			dir=`pwd`

			echo "reparse Corblivar solution for $bench on $dies; working dir: $dir"

			$root/Corblivar $bench $base/$dies/$exp/$bench.conf $base/$dies/$exp/ $bench.solution > $bench.log

			# run individual aux scripts, if required
			#
#			./HotSpot.sh $bench $die_count
		done

		# run experiments-folder aux scripts
#		./gp.sh

		cd $base/$dies/$exp

	done
done
