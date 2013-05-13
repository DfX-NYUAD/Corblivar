#!/bin/bash
for type in gp eps solution flp ptrace lcf steady grid grid.layer_* svg data results pdf txt
do
	files="*."$type
	rm -f $files
done
