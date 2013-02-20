#!/bin/bash
for gp in `ls *.gp`
do
	gnuplot $gp
done
#for eps in `ls *.eps`
#do
#	epstopdf $eps
#done
