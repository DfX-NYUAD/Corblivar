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

			echo "running Variation_TSC for $bench on $dies; run $run; working dir: $dir"

			# evaluate leakage via dedicated binary
			$root/Variation_TSC $bench $base/$dies/$exp/$bench.conf $base/benches/ $bench.solution > $bench.variations.log
		done

		cd $base/$dies/$exp
	done
done
