#!/bin/bash

HS=~/code/HotSpot_detailed3D

if [ "$1" == "" ]; then
	echo "Provide benchmark name as parameter!"
	exit
fi

BOTTOM_FP=$1_HotSpot_0.flp
PTRACE=$1_HotSpot.ptrace
GRID_LCF=$1_HotSpot.lcf
STEADY_OUTPUT=$1_HotSpot.steady
STEADY_GRID_OUTPUT=$1_HotSpot.steady.grid
DIM=64

# perform HS call
$HS/hotspot -c $HS/hotspot.config -f $BOTTOM_FP -p $PTRACE -grid_steady_file $STEADY_GRID_OUTPUT -steady_file $STEADY_OUTPUT -model_type grid -grid_map_mode max -detailed_3D on -grid_layer_file $GRID_LCF -grid_rows $DIM -grid_cols $DIM
# render SVG of temperature map
$HS/grid_thermal_map.pl $BOTTOM_FP $STEADY_GRID_OUTPUT $DIM $DIM > $BOTTOM_FP.svg
