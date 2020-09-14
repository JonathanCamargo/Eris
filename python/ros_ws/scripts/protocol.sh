#!/bin/bash

if [[ "$#" -lt 2 ]]; then
  echo "Must provide two arguments: protocol <name> <action>"
  exit
fi

name=$1
action=$2

if [[ $name = "array" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch eris_emg arrayprotocol.launch
  elif [[ $action = "start" ]]; then
    echo "Starting protocol"
    path=`rospack find eris_emg`
    bash "$path"/scripts/start.sh $3
  elif [[ $action = "stop" ]]; then
    echo "Stoping protocol"
    path=`rospack find eris_emg`
    bash "$path"/scripts/stop.sh
  fi
fi
