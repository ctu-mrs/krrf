#!/bin/bash

MAP=$1
N=$2
O=$3

# rm test_P

make clean
make test_krrf

# > out.txt 2>&1
# gdb -ex=r -ex=quit --args test_krrf 1 -map maps/map.tri -target targets/map.txt -problem 0 -iters $ITERS -dtg 20 -mxtime $MXTIME -mcp $NMCP -exprob $Q -regexpl $REGEXP -size 30000 -o output -robot :I10x50 -pt 1
# gdb -ex=r -ex=quit --args test_krrf 1 -map maps/empty1600.tri -target targets/empty1600_13.txt -problem 0 -iters $ITERS -dtg 20 -mxtime $MXTIME -mcp $NMCP -exprob $Q -regexpl $REGEXP -size 30000 -o output -robot :I10x50 -pt 1

./test_krrf 1 -map $MAP -sgradius 20 -genconfig $N -target _ -problem 0 -iters 1 -dtg 50 -mxtime 1 -mcp 1 -exprob 0 -regexpl 100 -size 5000 -o $3 -robot :I10x50 -pt 2

#gdb -ex=r -ex=quit --args test_krrf 1 -map maps/bugtrap1.tri -target targets/bugtrap.txt -problem 0 -iters $ITERS -dtg 50 -mxtime $MXTIME -mcp $NMCP -exprob $Q -regexpl $REGEXP -size 10000 -o output5 -robot :I10x50 -pt 2


#python3 Statistics_comparison.py output2.stat params.csv KRRFTime SSFTime KRRFDist SSFDist

#gnuplot -e "file1='KRRFTime'" -e "casik=10" -e "file2='SSFTime'" -e "par1='./Comparison_time.eps'" -e "osa='Time [s]'" Distribution_gnuplot_comparison.p
#gnuplot -e "file1='KRRFDist'" -e "casik=10000" -e "file2='SSFDist'" -e "par1='./Comparison_dist.eps'" -e "osa='Distance'" Distribution_gnuplot_comparison.p




#python Statistics_TSP.py "outputp.stat" "TSP_t1" "TSP_t2" "TSP_t3"

#gnuplot -e "file1='TSP_t1'" -e "casik=15" -e "file2='TSP_t2'" -e "file3='TSP_t3'" -e "par1='./Dist_TSP_potholes.eps'" Distribution_gnuplot_TSP.p

#gdb -ex=r -ex=quit --args test_krrf 1 -map maps/empty1600.tri -target targets/empty1600.txt -problem 0 -iters $ITERS -dtg 20 -mxtime $MXTIME -mcp $NMCP -exprob $Q -regexpl $REGEXP -size 20000 -o outpute -robot :I10x50 -pt 1

#python Statistics_TSP.py "outpute.stat" "TSP_t4" "TSP_t5" "TSP_t6"

#gnuplot -e "file1='TSP_t4'" -e "casik=5" -e "file2='TSP_t5'" -e "file3='TSP_t6'" -e "par1='./Dist_TSP_empty.eps'" Distribution_gnuplot_TSP.p

#gdb -ex=r -ex=quit --args test_krrf 1 -map maps/empty1600.tri -target targets/empty1600_10.txt -problem 0 -iters $ITERS -dtg 20 -mxtime $MXTIME -mcp $NMCP -exprob $Q -regexpl $REGEXP -size 20000 -o outpute2 -robot :I10x50 -pt 1

#python Statistics_TSP.py "outpute2.stat" "TSP_t1" "TSP_t2" "TSP_t3"

#gnuplot -e "file1='TSP_t1'" -e "casik=30" -e "file2='TSP_t2'" -e "file3='TSP_t3'" -e "par1='./Dist_TSP_emptybig.eps'" Distribution_gnuplot_TSP.p
