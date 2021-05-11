#!/bin/bash

./build.sh
#find ../output/redo4/*.out | shuf > todo.txt

FILECOUNT=$(< todo.txt wc -l) # idk what this means, but it counts the lines in the file
PERMACHINE=$(((FILECOUNT+24)/25)) # number we will allocate to each machine
echo "There are $FILECOUNT instances and we will allocate $PERMACHINE instances to each linux machine"

for x in $(seq -f "%02g" 1 25)
do
		ssh lin"$x" "cd socg/cgshop2021/build && cat todo_dist.txt \
																				| sed -n '$(((x-1) * PERMACHINE + 1)),$((x * PERMACHINE))p' \
																				| ./parallel_k_opt -d 4" &> log$x.txt & 
		sleep .1 	# this sleep is necessary to prevent sshing too quickly and having a conflict in lock file
done

