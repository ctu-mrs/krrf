#!/bin/bash
#./fig2iso.pl input output 1
python fig2poly.py $1
arg1=$1
suffix=".txt";
arg1=${arg1%$suffix}; #Remove suffix
counter=1
echo "exists."
while :
do
    arg2="$arg1$counter.poly"
    arg3="$arg1$counter.1.node"
    arg4="$arg1$counter.1.ele"
    echo "$arg2 exists."
    if test -f "$arg2"; then
        echo "$arg2 exists."
        ./triangle $arg2
        python poly2tri.py $arg3 $arg4
        let "counter += 1"
        #rm $arg3
        #rm $arg4
    else
        break
    fi
    
done
