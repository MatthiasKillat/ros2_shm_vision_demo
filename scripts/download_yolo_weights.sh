#!/bin/bash

# if argument is present, use that for filename
if [[ -z "$1" ]]; then
  # default to yolov3.weights
  fname="yolov3.weights"
else
  fname="$1"
fi

# get weights
if [ ! -f "./config/$fname" ]; then
  echo "$fname doesnt exist, downloading now..."
  wget -P config https://pjreddie.com/media/files/$fname
else
  echo "$fname exists in the `config` directory, skipping downloading"
fi
