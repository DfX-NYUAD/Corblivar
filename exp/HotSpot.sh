#!/bin/bash

HS=~/code/HotSpot

if [ "$1" == "" ]; then
	echo "Provide benchmark name as parameter!"
	exit
fi

if [ "$2" == "" ]; then
	echo "Provide layer count!"
	exit
fi

DUMMY_FP=$1_HotSpot_Si_active_1.flp
PTRACE=$1_HotSpot.ptrace
GRID_LCF=$1_HotSpot.lcf
STEADY_OUTPUT=$1_HotSpot.steady
STEADY_GRID_OUTPUT=$1_HotSpot.steady.grid
LOG=$1_HotSpot.txt
DIM=64

# perform HS call
echo "Perform HotSpot run ..."
$HS/hotspot -c $HS/hotspot.config -f $DUMMY_FP -p $PTRACE -grid_steady_file $STEADY_GRID_OUTPUT -steady_file $STEADY_OUTPUT -model_type grid -grid_map_mode max -detailed_3D on -grid_layer_file $GRID_LCF -grid_rows $DIM -grid_cols $DIM > $LOG
STATUS=$?
# log hottest block
tail -n1 $LOG

# render temperature map for active layers; layer IDs must correspond to active Si layers
# defined in  GRID_LCF

if [ "$STATUS" == "0" ]; then

	DIE=1
	for (( layer = 1; layer < ($2 * 4); layer = layer + 4 ))
	do
		echo "Generate temperature map for die $DIE"

		FP=$1_HotSpot_Si_active_$DIE.flp

		# render SVG
		$HS/grid_thermal_map.pl $FP $STEADY_GRID_OUTPUT.layer_$layer $DIM $DIM > $FP.svg
		# generate PDF
		cairosvg-py3 $FP.svg -f pdf -o $FP.pdf
		# drop svg
		rm $FP.svg

		# consider next die
		DIE=$(($DIE + 1))
	done
fi
