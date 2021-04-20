#!/bin/bash
rosbag record /waterlinked/position/acoustic/filtered \
              /waterlinked/position/acoustic/raw \
			  -O $1