#!/bin/bash

if [[ "$#" -eq 1 ]]; then
   rosparam set 'session_duration_seconds' $1
else
   rosparam set 'session_duration_seconds' 30
fi


rostopic pub --once /protocol custom_msgs/String "header:
  seq: 0
  stamp: 'now'
  frame_id: ''
data: 'START'"
