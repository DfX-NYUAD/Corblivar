#!/bin/bash
for type in gp eps solution flp ptrace lcf steady grid grid.layer_* grid.gp_data.* svg data results pdf txt
do
	files="*."$type
	rm -f $files
done
