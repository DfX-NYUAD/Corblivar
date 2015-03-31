#!/bin/bash
root=~/github/Corblivar
base=$root/exp

exp="3DFP"
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

	# prerun experiments; 3DFP
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

		# perform experiments; run 3DFP for each benchmark
		for bench in $benches
		do
			dir=`pwd`

			echo "running 3DFP for $bench on $dies; run $run; working dir: $dir"

			~/code/3dfp/3dfp $base/benches/$bench $die_count 1 > $bench.3DFP.log

			for (( layer = 1; layer <= $die_count; layer++ ))
			do
				cp layer$layer.flp $bench"_layer"$layer".flp"
			done

			echo "parsing 3DFP result for $bench on $dies; run $run; working dir: $dir"

			$root/3DFP_Parser $bench $base/configs/$dies/$exp/$bench.conf $base/benches/ > $bench.parsed.log

			# run individual aux scripts, if required
			#
			./HotSpot.sh $bench $die_count
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
