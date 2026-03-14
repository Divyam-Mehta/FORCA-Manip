#!/bin/bash

hm=./

#cd "$hm"/data

i=1

mkdir rgb_images$i
cd rgb_images$i
echo "Doing RGB"
python2 ../../grabrgb.py &
pwd
rosbag play ../*.bag
sleep 15
ps aux | grep rosbag | grep -v grep | awk '{print $2}' | xargs kill -SIGKILL > /dev/null 2>&1
ps aux | grep grabrgb | grep -v grep | awk '{print $2}' | xargs kill -SIGKILL > /dev/null 2>&1
cd ..

