#!/bin/bash
root=~/github/Corblivar
base=$root/exp

exp=$1
#exp="VA_LP"
benches=$2
#benches="n100_soft n200_soft n300_soft"
runs=20

#for die_count in 2 3 4
for die_count in 4
do
	dies=$die_count"dies"

	cd $base/$dies/$exp

	for (( run = 1; run <= $runs; run++ ))
	do
		cd $run

		for bench in $benches
		do
			dir=`pwd`

			echo "re-running Corblivar for $bench on $dies; run $run; working dir: $dir"

			# also wrap gdb in batch mode, which helps to log any segfault's origin
			gdb -batch -ex=run -ex=backtrace --args $root/Corblivar $bench $base/$dies/$exp/$bench.conf $base/benches/ $bench.solution > $bench.log

			# run individual aux scripts, if required
			#
			./HotSpot.sh $bench $die_count

#			# evaluate leakage via dedicated binary
#			$root/Correlation_TSC $bench $base/$dies/$exp/$bench.conf $base/benches/ $bench.solution >> $bench.log
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
