#!/bin/bash

./build.sh
/bin/ls -A ../output/redo4 > init_done.txt
/bin/ls -A ../output/makespan > init_all.txt
grep -F -x -v -f init_done.txt init_all.txt > init_todo.txt

FILECOUNT=$(< init_todo.txt wc -l) # idk what this means, but it counts the lines in the file
PERMACHINE=$(((FILECOUNT+24)/25)) # number we will allocate to each machine
echo "There are $FILECOUNT instances and we will allocate $PERMACHINE instances to each linux machine"

for x in $(seq -f "%02g" 1 25)
do
		ssh lin"$x" "cd socg/cgshop2021/build && cat init_todo.txt \
																				| sed -n '$(((x-1) * PERMACHINE + 1)),$((x * PERMACHINE))p' \
																				| ./parallel_fff 1" \
					&> >(trap "" INT TERM; sed "s/^/lin$x: /") & # add prefix of machine it came from
		sleep .1 	# this sleep is necessary to prevent sshing too quickly and having a conflict in lock file
done

#ssh -t lin01 'cd socg/cgshop2021/build && ls ../output/redo4 | shuf | ./parallel_k_opt -c 4' &

