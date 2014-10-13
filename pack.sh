#!/bin/bash

rm -rf Corblivar

mkdir Corblivar
cd Corblivar

ln -s ../src
ln -s ../src_aux
ln -s ../thermal_analysis_octave

ln -s ../LICENCE
ln -s ../Makefile
ln -s ../README.md

mkdir exp
cd exp

ln -s ../../exp/benches
ln -s ../../exp/configs
ln -s ../../exp/Corblivar.conf
ln -s ../../exp/Technology.conf

for file in `ls ../../exp/*.sh`
do
	ln -s "$file"
done

cd ../../

zip -r Corblivar.zip Corblivar

rm -rf Corblivar
