#!/bin/bash

NMCP=$1
Q=$2
REGEXP=$3
MXTIME=$4
ITERS=$5
DTG=$6
MAP=$7
TARGET=$8
O1=$9
DRAW=${10}
AMAX=10

#make test_krrf

./test_krrf 1 -map $MAP -target $TARGET -problem 0 -draw $DRAW -iters $ITERS -dtg $DTG -mxtime $MXTIME -mcp $NMCP -exprob $Q -regexpl $REGEXP -size 10000 -o $O1 -robot :I10x50 -pt 2 -amax $AMAX
