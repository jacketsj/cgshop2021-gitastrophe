#!/bin/bash

for x in $(seq -f "%02g" 1 25)
do
		ssh lin$x "echo 'Im alive!' &> >(trap '' INT TERM; sed 's/^/lin$x: /')" & # add prefix of machine it came from
		sleep .1
done

