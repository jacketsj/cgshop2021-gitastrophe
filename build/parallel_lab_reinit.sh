#!/bin/bash

./build.sh
#/bin/ls -A ../output/redo4 > reinit_todo.txt #actually use paul's randomized inputs folder

FILECOUNT=$(< reinit_todo.txt wc -l) # idk what this means, but it counts the lines in the file
PERMACHINE=$(((FILECOUNT+24)/25)) # number we will allocate to each machine
echo "There are $FILECOUNT instances and we will allocate $PERMACHINE instances to each linux machine"

for x in $(seq -f "%02g" 1 24)
do
		ssh lin"$x" "cd socg/cgshop2021/build && cat reinit_todo.txt \
																				| sed -n '$(((x-1) * PERMACHINE + 1)),$((x * PERMACHINE))p' \
																				| ./parallel_fff -f -r 2 &> reinit_log$x.txt" & # add prefix of machine it came from
		sleep .1 	# this sleep is necessary to prevent sshing too quickly and having a conflict in lock file
done
