#!/bin/bash
root=~/github/Corblivar
base=$root/exp

benches=$1
exp=$2
runs=40

for die_count in 2
#for die_count in $2
do
	dies=$die_count"dies"

	cd $base/$dies/$exp

	# init experiments done
	#

	for (( run = 1; run <= $runs; run++ ))
	do
		cd $run

		# perform experiments; run Corblivar for each benchmark
		for bench in $benches
		do
			dir=`pwd`

			echo "running Postprocessing_TSC for $bench on $dies; run $run; working dir: $dir"

			$root/Postprocessing_TSC $bench $base/$dies/$exp/$bench.conf $base/benches/ $bench.solution > $bench.postprocessing.log
		done

		cd $base/$dies/$exp
	done
done
