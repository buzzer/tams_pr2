#!/bin/bash
#2013-06-28 Sebastian Rockel (rockel at informatik dot uni-hamburg dot de)
#
# Kills all ROS processes

#pids=`ps -fA | grep ros | cut -d' ' -f4`
pids=`ps -fA | grep ros | sed -e 's/^[a-zA-Z0-9]\+\ \+\([0-9]\+\)\ \+.\+$/\1/'`
amount=`echo $pids | wc -w`
if [ $amount -gt 0 ]
then
  echo -n "Killing $amount process(es).. "
  echo $pids
  kill $pids
fi
