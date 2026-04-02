#!/bin/bash

#rm /media/chenzhaoqi/data/tmp/output/esos/esos.itb


./build.sh clean
./build.sh config
./build.sh

./build.sh itb
