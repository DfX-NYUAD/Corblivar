#!/bin/bash
for type in gp eps solution flp ptrace lcf steady grid svg
do
	for file in `ls *.$type`
	do
		rm $file
	done
done
