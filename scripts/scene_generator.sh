#!/bin/bash

# Total number of scenes to generate
scenes=$1
# Number of consecutive scenes generated
increment=$2
# Output directory
output=$3

source setup.sh

for (( i=0; i<=scenes; i+=increment))
do
    let "final=($i+1)*$increment" 
    gzserver ./worlds/spawner.world &>/dev/null &
    sleep 2s
    ./build/bin/scene_example -i "$output/images/" -d "$output/annotations/" -s $increment -n $i
    pkill -f gz
done
