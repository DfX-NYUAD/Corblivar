#!/bin/bash

CUR_DIR=`pwd`;
ROOT_DIR=$CUR_DIR"/..";
BENCH_DIR=$ROOT_DIR"/exp/benches/";
CORBLIVAR_BIN=$ROOT_DIR"/Corblivar";
OCTAVE="thermal_analysis_octave";

if [ "$1" == "" ]; then
	echo "Provide benchmark name as 1st parameter!"
	exit
fi

if [ "$2" == "" ]; then
	echo "Provide config file as 2nd parameter!"
	exit
fi

BENCH=$1
CONFIG_FILE=$2

WORK_DIR_1=$CUR_DIR"/"$BENCH"_TSV_0"
WORK_DIR_2=$CUR_DIR"/"$BENCH"_TSV_100"

SOLUTION_FILE=$BENCH".solution"

echo "initial Corblivar run; try to obtain solution"

while :
do
	$CORBLIVAR_BIN $BENCH $CONFIG_FILE $BENCH_DIR

	file $BENCH".solution" > /dev/null
	
	# only abort (infinite) loop when solution file exists, i.e., initial Corblivar
# run was successful
	if [ "$?" == "0" ]; then
		break;
	fi
done

echo "copy Octave scripts; required for separate working directories"

file $WORK_DIR_1 > /dev/null
if [ "$?" != "0" ]; then
	mkdir $WORK_DIR_1
fi
file $WORK_DIR_2 > /dev/null
if [ "$?" != "0" ]; then
	mkdir $WORK_DIR_2
fi

cd $ROOT_DIR"/"$OCTAVE
cp * $WORK_DIR_1
cp * $WORK_DIR_2

cd $CUR_DIR

echo "copy Corblivar config and solution files; required for separate working directories"

cp -v $CONFIG_FILE $WORK_DIR_1
cp -v $CONFIG_FILE $WORK_DIR_2

cp -v $SOLUTION_FILE $WORK_DIR_1
cp -v $SOLUTION_FILE $WORK_DIR_2

echo "start Octave scripts in parallel, running in separate working directories"

cd $WORK_DIR_1
# the notation ${CONFIG_FILE##*/} delivers the filename w/o full path
echo "working dir 1 (w/o TSVs): `pwd`";
octave optimization.m $BENCH $WORK_DIR_1"/"${CONFIG_FILE##*/} $ROOT_DIR 0 &

PID1=$!

cd $WORK_DIR_2
echo "working dir 2 (w/ TSVs): `pwd`";
octave optimization.m $BENCH $WORK_DIR_2"/"${CONFIG_FILE##*/} $ROOT_DIR 100 &

PID2=$!

while :
do
	sleep 10;
	STATUS1=`ps $PID1 | wc -l`;
	STATUS2=`ps $PID2 | wc -l`;

	if [ "$STATUS1" == "1" ] && [ "$STATUS2" == "1" ]; then
		break;
	else
		echo "Octave scripts still running..."
	fi
done

echo "Octave scripts done; copy resulting config files"

cp -v $WORK_DIR_1"/"${CONFIG_FILE##*/} $CONFIG_FILE
cp -v $WORK_DIR_2"/"${CONFIG_FILE##*/} $CONFIG_FILE"_TSVs"
