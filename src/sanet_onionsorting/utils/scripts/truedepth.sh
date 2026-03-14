#!/bin/bash

hm=./

#cd "$hm"/data

i=1

mkdir depth_images$i
cd depth_images$i
echo "Doing Depth"
python2 ../../grabdepth.py &
pwd
rosbag play ../*.bag
sleep 15
ps aux | grep rosbag | grep -v grep | awk '{print $2}' | xargs kill -SIGKILL > /dev/null 2>&1
ps aux | grep grabdepth | grep -v grep | awk '{print $2}' | xargs kill -SIGKILL > /dev/null 2>&1
cd ..

