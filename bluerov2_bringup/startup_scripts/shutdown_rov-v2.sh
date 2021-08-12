#!/usr/bin/env bash
screen -XS rosrecord quit
rosnode kill -a
screen -XS rosrov quit