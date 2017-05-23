#!/bin/bash

#exp="regular"
#exp="VA_HP"
exp="voltage_assignment"
#benches="n100_soft n200_soft n300_soft"
#benches="n300_soft"
#benches="ibm01 ibm03 ibm07"
benches="ibm01"
runs=20
dies_range="2 3 4"
#dies_range="4"

deadspace_threshold_std_dev=$1
power_threshold_std_dev=$2

#exp=$2
#benches=$3
#dies_range=$4

for die_count in $dies_range
do
	dies=$die_count"dies"

	for bench in $benches
	do

		summary_file=$bench"_"$exp"_"$dies".results"

		# drop old files
		rm $summary_file 2> /dev/null

		echo "Current set under consideration: $summary_file"

		# gather data from files
		for (( run = 1; run <= $runs; run++ ))
		do
			file="$dies/$exp/$run/$bench.results"

			file="$dies/$exp/$run/$bench.results"

			# evaluate the file only in case its available
			if [ -f $file ]; then

				# memorize all result parameters in variable, reset variable w/ run
				#
				# the sed 'X p' represents the line with the respective result in the file; be careful that those lines will differ depending on the cost factors!
				#
				# also memorize how many data points / criteria to consider, including run
				criteria=11
				criteria=14

				# Run
				result="$run"

				# Power
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '11 p' -n`
				power_count=2

				# Critical Delay
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '30 p' -n`
				delay_count=3

				# HPWL; keep also track of which criterion count this is, including run
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '13 p' -n`
				HWPL_count=4

				# Routing utilization
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '17 p' -n`

				# Power for HPWL
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '15 p' -n`

				# Deadspace; keep also track of which criterion count this is, including run
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '4 p' -n`
				deadspace_count=7

				# Outline
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '9 p' -n`
				outline_count=8

				# HotSpot Temp; extract from separate file
				result=$result"	"`tail -n 1 $dies/$exp/$run/$bench'_HotSpot.txt' | rev | cut -f 1 | rev`

				# TSVs
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '19 p' -n`

##				# Power for TSVs
##				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '20 p' -n`
##
##				# Avg TSVs per Island
##				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '23 p' -n`
##
##				# Use of Deadspace by TSVs
##				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '24 p' -n`

				# VA: volumes count
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '41 p' -n`
				volumes_count=11

				# VA: avg corners
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '38 p' -n`
				corners_count=12

				# VA: level shifters
				result=$result"	"`rev $file | cut -d ' ' -f 1 | rev | sed '39 p' -n`

				# Runtime, last line
				result=$result"	"`tail -n 1 $file | rev | cut -d ' ' -f 2 | rev`
				runtime_count=11
				runtime_count=14

				# push variable as new line into file, append mode
				echo $result >> $summary_file
			fi
		done

		# drop lines where some data is missing; this indicates a failed Corblivar run; work on file copy, to avoid removing lines while iterating over lines
		count=0
		deleted_lines=0
		summary_file_copy=$summary_file".copy"
		cp $summary_file $summary_file_copy
		for (( run = 1; run <= $runs; run++ ))
		do
			if [ -f $file ]; then
				sed_string="$((run))q;d"
				sed_result=`sed $sed_string $summary_file`
				wc_result=`echo $sed_result | wc -w`
				if [ "$wc_result" -lt $criteria ]; then
					sed -i "$((run - deleted_lines))d" $summary_file_copy
					deleted_lines=$((deleted_lines + 1))
				else
					count=$((count + 1))
				fi
			fi
		done
		mv $summary_file_copy $summary_file

		echo "Valid results: $count"

		# also drop lines where the deadspace is above some threshold; work on file copy, to avoid removing lines while iterating over lines
		if [ "$deadspace_threshold_std_dev" ]; then
			deadspace_avg=`awk "BEGIN {sum=0} {sum+=$\"$deadspace_count\"} END {print sum/NR}" $summary_file`
			deadspace_threshold=`awk "BEGIN {sum_sq=0} {sum_sq+=($\"$deadspace_count\" - \"$deadspace_avg\")**2} END {print \"$deadspace_avg\" - \"$deadspace_threshold_std_dev\"*sqrt(sum_sq/NR)}" $summary_file`

			echo "Average deadspace: $deadspace_avg"
			echo "Deadspace threshold (curr avg - $deadspace_threshold_std_dev x curr std dev): $deadspace_threshold"
			#echo "Dropping the following results, which exceed the threshold:"

			summary_file_copy=$summary_file".copy"
			deleted_lines=0
			count_new=0
			cp $summary_file $summary_file_copy
			for (( run = 1; run <= $count; run++ ))
			do
				if [ -f $file ]; then
					sed_string="$((run))q;d"
					# simply escaping only variable is not sufficient, so just calculate 0 + deadspace_count
					cur_deadspace=`sed $sed_string $summary_file | awk "{print 0 + $\"$deadspace_count\"}"`
					if (( $(echo "$cur_deadspace > $deadspace_threshold" | bc) )); then
						#sed $sed_string $summary_file
						sed -i "$((run - deleted_lines))d" $summary_file_copy
						deleted_lines=$((deleted_lines + 1))
					else
						count_new=$((count_new + 1))
					fi
				fi
			done
			mv $summary_file_copy $summary_file
			count=$count_new

			echo "Count of valid results / data sets after removing those exceeding the deadspace threshold:" `wc -l $summary_file | awk '{print $1}'`
		fi

		# also drop lines where the power is some threshold; work on file copy, to avoid removing lines while iterating over lines
		if [ "$power_threshold_std_dev" ]; then
			power_avg=`awk "BEGIN {sum=0} {sum+=$\"$power_count\"} END {print sum/NR}" $summary_file`
			power_threshold=`awk "BEGIN {sum_sq=0} {sum_sq+=($\"$power_count\" - \"$power_avg\")**2} END {print \"$power_avg\" - \"$power_threshold_std_dev\"*sqrt(sum_sq/NR)}" $summary_file`

			echo "Average power: $power_avg"
			echo "Power threshold (curr avg - $power_threshold_std_dev x curr std dev): $power_threshold"
			#echo "Dropping the following results, which exceed the threshold:"

			summary_file_copy=$summary_file".copy"
			deleted_lines=0
			cp $summary_file $summary_file_copy
			for (( run = 1; run <= $count; run++ ))
			do
				if [ -f $file ]; then
					sed_string="$((run))q;d"
					# simply escaping only variable is not sufficient, so just calculate 0 + power_count
					cur_power=`sed $sed_string $summary_file | awk "{print 0 + $\"$power_count\"}"`
					if (( $(echo "$cur_power > $power_threshold" | bc) )); then
						#sed $sed_string $summary_file
						sed -i "$((run - deleted_lines))d" $summary_file_copy
						deleted_lines=$((deleted_lines + 1))
					fi
				fi
			done
			mv $summary_file_copy $summary_file

			echo "Count of valid results / data sets after removing those exceeding the power threshold:" `wc -l $summary_file | awk '{print $1}'`
		fi

		# calculate averages, for each criteria separately, but excluding the run identifier (1st criterion)
		avg=""
		for (( criterion = 2; criterion <= $criteria; criterion++ ))
		do
			# HPWL criterion, should be put in scientific format
			if [ "$criterion" -eq "$HWPL_count" ]; then
				# http://stackoverflow.com/questions/2451635/howto-pass-a-string-as-parameter-in-awk-within-bash-script
				# https://linuxconfig.org/calculate-column-average-using-bash-shell
				avg=$avg" "`awk "BEGIN {total=0} {total+=$\"$criterion\"} END {printf(\"%.2e\\n\",total/NR)}" $summary_file`
			# outline, scale from um^2 to mm^2
			elif [ "$criterion" -eq "$outline_count" ]; then
				avg=$avg" "`awk "BEGIN {total=0} {total+=$\"$criterion\"} END {printf(\"%.2f\\n\",100*(total/NR))}" $summary_file`
			# deadspace, report average over dies
			elif [ "$criterion" -eq "$deadspace_count" ]; then
				avg=$avg" "`awk "BEGIN {total=0} {total+=$\"$criterion\"} END {printf(\"%.2f\\n\",(total/NR)/\"$die_count\")}" $summary_file`
			# runtime, report no decimal points 
			elif [ "$criterion" -eq "$runtime_count" ]; then
				avg=$avg" "`awk "BEGIN {total=0} {total+=$\"$criterion\"} END {printf(\"%.0f\\n\",total/NR)}" $summary_file`
#			# corners in volumes, calculate average over volumes count
#			elif [ "$criterion" -eq "$corners_count" ]; then
#				avg=$avg" "`awk "BEGIN {total_corners=0;total_volumes=0} {total_corners+=$\"$criterion\";total_volumes+=$\"$volumes_count\"} END {printf(\"%.2f\\n\",((total_corners/total_volumes)/NR))}" $summary_file`
##			# delay criterion, for gathering data for HP/LP VA setups, scaling the delay is required
##			elif [ "$criterion" -eq "$delay_count" ]; then
##				avg=$avg" SCALED_DELAY:"
##				avg=$avg" "`awk "BEGIN {total=0} {total+=$\"$criterion\"} END {printf(\"%.2f\\n\",1.3*(total/NR))}" $summary_file`
			# other criteria shall be reported as regular float
			else
				avg=$avg" "`awk "BEGIN {total=0} {total+=$\"$criterion\"} END {printf(\"%.2f\\n\",total/NR)}" $summary_file`
			fi

		done

#		# recalculate deadspace average, and report average over die count
#		deadspace_avg=`awk "BEGIN {sum=0} {sum+=$\"$deadspace_count\"} END {print sum/NR}" $summary_file`
#		deadspace_avg_per_die=`echo "scale=2; $deadspace_avg / $die_count" | bc`

		# report all the averages
		echo "Average values:"
		echo $avg
#		echo "Average deadspace per die: $deadspace_avg_per_die"
		echo ""

		echo "" >> $summary_file
		echo $avg >> $summary_file
#		echo "Average deadspace per die: $deadspace_avg_per_die" >> $summary_file
		if [ "$deadspace_threshold_std_dev" ]; then
			echo "[Note that other results with deadspace above threshold (initial avg - $deadspace_threshold_std_dev x initial std dev: $deadspace_threshold) have been dropped]" >> $summary_file
		fi
		if [ "$power_threshold_std_dev" ]; then
			echo "[Note that other results with power above threshold (initial avg - $power_threshold_std_dev x initial std dev: $power_threshold) have been dropped]" >> $summary_file
		fi
	done
done
