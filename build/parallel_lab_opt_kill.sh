#!/bin/bash

for x in $(seq -f "%02g" 1 25)
do
		ssh lin"$x" "pkill -f parallel_k_opt" # -f is to match full name of process
		sleep .1 	# this sleep is necessary to prevent sshing too quickly and having a conflict in lock file
done

