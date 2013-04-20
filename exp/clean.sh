#!/bin/bash
for type in gp eps solution flp ptrace lcf steady grid grid.layer_* svg data results pdf log
do
	files="*."$type
	rm -f $files
done
