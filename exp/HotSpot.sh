#!/bin/bash

HS=~/code/HotSpot_detailed3D

if [ "$1" == "" ]; then
	echo "Provide benchmark name as parameter!"
	exit
fi

HS_opts=
BOTTOM_FP=$1_HotSpot_0.flp
PTRACE=$1_HotSpot.ptrace
GRID_LCF=$1_HotSpot.lcf
GRID_OUTPUT=$1_HotSpot.temp_grid

# perform HS call
$HS/hotspot -c $HS/hotspot.config $HS_opts -f $BOTTOM_FP -p $PTRACE -grid_steady_file $GRID_OUTPUT -model_type grid -detailed_3D on -grid_layer_file $GRID_LCF
# render SVG of temperature map
$HS/grid_thermal_map.pl $BOTTOM_FP $GRID_OUTPUT > $GRID_OUTPUT.svg
