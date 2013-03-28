#!/bin/bash
for type in gp eps solution flp ptrace lcf steady grid svg data results pdf
do
	files="*."$type
	rm -f $files
done
