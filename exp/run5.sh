#!/bin/bash
root=~/code/Corblivar
base=$root/exp

exp="packing"
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

	# init experiments, if required
	#
#	cp -r $base/$fitting .
#	cp $base/gp.sh .
#	cp $base/HotSpot.sh .
#	ln -s $base/benches .
#
#	for bench in $benches
#	do
#		cp $base/benches/$bench.* .
#		cp $base/configs/$dies/$exp/$bench.alr .
#	done
#
#	cd $fitting
#
#	for bench in $benches
#	do
#		octave optimization.m $bench $bench.conf
#	done
#
#	cd $base/$dies/$exp

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

			$root/Corblivar $bench $base/configs/$dies/$exp/$bench.conf $base/benches/ > $bench.log

			# run individual aux scripts, if required
			#
#			./HotSpot.sh $bench $die_count
		done

		# run experiments-folder aux scripts
		./gp.sh

		cd $base/$dies/$exp

	done
done
