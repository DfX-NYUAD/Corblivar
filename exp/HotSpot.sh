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

DUMMY_FP=$1_HotSpot_1.flp
PTRACE=$1_HotSpot.ptrace
GRID_LCF=$1_HotSpot.lcf
STEADY_OUTPUT=$1_HotSpot.steady
STEADY_GRID_OUTPUT=$1_HotSpot.steady.grid
GRID_LCF_TSVS=$1_HotSpot_TSVs.lcf
STEADY_OUTPUT_TSVS=$1_HotSpot_TSVs.steady
STEADY_GRID_OUTPUT_TSVS=$1_HotSpot_TSVs.steady.grid
LOG=$1_HotSpot.log
LOG_TSVS=$1_HotSpot_TSVs.log
DIM=64

# perform HS call
echo "Perform HotSpot run ..."
$HS/hotspot -c $HS/hotspot.config -f $DUMMY_FP -p $PTRACE -grid_steady_file $STEADY_GRID_OUTPUT -steady_file $STEADY_OUTPUT -model_type grid -grid_map_mode max -detailed_3D on -grid_layer_file $GRID_LCF -grid_rows $DIM -grid_cols $DIM > $LOG

if [ "$3" == "1" ]; then
	echo "Perform HotSpot w/ TSVs run ..."
	$HS/hotspot -c $HS/hotspot.config -f $DUMMY_FP -p $PTRACE -grid_steady_file $STEADY_GRID_OUTPUT_TSVS -steady_file $STEADY_OUTPUT_TSVS -model_type grid -grid_map_mode max -detailed_3D on -grid_layer_file $GRID_LCF_TSVS -grid_rows $DIM -grid_cols $DIM > $LOG_TSVS
fi

# render temperature map for active layers; layer IDs must correspond to activ Si layers
# defined in  GRID_LCF
DIE=1
for (( layer = 1; layer < ($2 * 4); layer = layer + 4 ))
do
	echo "Generate temperature map for die $DIE"

	FP=$1_HotSpot_$DIE.flp

	# render SVG
	$HS/grid_thermal_map.pl $FP $STEADY_GRID_OUTPUT.layer_$layer $DIM $DIM > $FP.svg
	# generate PDF
	cairosvg-py3 $FP.svg -f pdf -o $FP.pdf
	# drop svg
	rm $FP.svg

	# consider next die
	DIE=$(($DIE + 1))
done

if [ "$3" == "1" ]; then
	DIE=1
	for (( layer = 1; layer < ($2 * 4); layer = layer + 4 ))
	do
		echo "Generate temperature map w/ TSVs for die $DIE"

		FP=$1_HotSpot_$DIE.flp

		# render SVG
		$HS/grid_thermal_map.pl $FP $STEADY_GRID_OUTPUT_TSVS.layer_$layer $DIM $DIM > $FP"_TSVs.svg"
		# generate PDF
		cairosvg-py3 $FP"_TSVs.svg" -f pdf -o $FP"_TSVs.pdf"
		# drop svg
		rm $FP"_TSVs.svg"

		# consider next die
		DIE=$(($DIE + 1))
	done
fi
