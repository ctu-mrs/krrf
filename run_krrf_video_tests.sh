#!/bin/bash

# List of parameters (replace with your actual parameters)

N=6

MAPS=("room_of_requirements.tri" "elkoMap.tri" "bugtrap1.tri" "potholes.fig.tri" "map.tri" "potholes.fig.tri" "bugtrap1.tri" "map.tri")
TARGETS=("room_of_requirements.txt" "elkoMap_5.txt" "bugtrap_5.txt" "potholes.fig_5.txt" "map_5.txt" "potholes.fig_10.tri" "bugtrap_10.txt" "map_10.txt")
DTG=(10 20 20 20 20 20 20 20 20 20 20 20 20)
MXTIME=(1.5 1.5 1.5 1.5 1.5 1.5 1.5 1.5 1.5 1.5 1.5 1.5)
OUTPUT=("room_6_10_1.5_07_diff" "elkoMap_5_20_1.5_07_diff" "bugtrap_5_20_1.5_07_diff" "potholes_5_20_1.5_07_diff" "map_5_20_1.5_07_diff" "potholes.fig_10_20_1.5_07_diff" "bugtrap_10_20_1.5_07_diff" "map_10_20_1.5_07_diff")
PROB=(0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7 0.7)
SIZE=(10000 10000 10000 10000 10000 10000 10000 10000 10000 10000 10000 10000 10000)


# SSH command function
run_ssh_command() {
    local index=$1
    echo "Running SSH command for parameters: $index"
    # Replace the following line with your actual SSH command
    
    ./test_krrf 1 -srand 1705622463 -map ./maps/${MAPS[$index]} -target ./targets/${TARGETS[$index]} -problem 0 -draw 1 -iters 1 -dtg ${DTG[$index]} -mxtime ${MXTIME[$index]} -mcp 10 -exprob ${PROB[$index]} -regexpl 100 -size 5000 -o ${OUTPUT[$index]} -robot :I10x50 -pt 2 -genvideo 1

    cd Video
    mkdir ./experiments_for_paper/${OUTPUT[$index]}/
    mv *.pdf ./experiments_for_paper/${OUTPUT[$index]}/
    cd ..
    python3 convert_pdf.py ./Video/experiments_for_paper/${OUTPUT[$index]}/
    cd Video/experiments_for_paper/${OUTPUT[$index]}/pngs/
    mencoder "mf://*.png" -mf fps=100 -o Video_${OUTPUT[$index]}.avi -ovc lavc -lavcopts vcodec=mpeg4:vpass=1:vbitrate=2160000:vqmin=3:threads=2
    cd ../../../../
    # Add more commands if neede
}

# Iterate through the indices of the list
for ((index=0; index<$N; index++)); do
    run_ssh_command $index
done
