#!/bin/bash

if [[ "$#" -lt 2 ]]; then
  echo "Must provide two arguments: protocol <name> <action>"
  exit
fi

name=$1
action=$2

#ARRAY PROTOCOL FOR ONLY NEXTFLEX
if [[ $name = "array" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch eris_emg array_protocol.launch
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

#ARRAY PROTOCOL INCLUDING OTHER EMG SOURCE
if [[ $name = "arrayplus" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch eris_emg arrayplus_protocol.launch
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

#ETI PROTOCOL
if [[ $name = "eti" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch eris_emg eti_protocol.launch
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

#ETI PROTOCOL INCLUDING OTHER EMG SOURCE
if [[ $name = "etiplus" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch eris_emg etiplus_protocol.launch
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


#IMU PROTOCOL
if [[ $name = "imu" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch eris_imu imu_protocol.launch
  fi
fi

#IMUML PROTOCOL
if [[ $name = "imuclassifier" ]]; then
  if [[ $action = "launch" ]]; then
    roslaunch ml imuclassifier_protocol.launch
  fi
fi
