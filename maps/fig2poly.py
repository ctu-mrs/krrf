#!/usr/bin/env python

import sys

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

bgt = open(sys.argv[1], "r")

#obstacles = open("obst.txt", "w")

line_counter = 0
number_of_lines = len(bgt.readlines())
bgt.close()
bgt = open(sys.argv[1], "r")
obst= [[0.0 for x in range(2)] for y in range(number_of_lines)]
verteces = []
vert_dict = {}
vert_indeces = []
segments = []
counter = 0
empty = True
while True:
    line = bgt.readline()
    line_counter+=1
    if not line:
        break    
    if line in ['\n'] and empty == False:
        counter += 1
        argument1 = ""
        argument1 = sys.argv[1].replace(".txt", "") + str(counter) + ".poly";
        print argument1
        bgt_poly = open(argument1, "w")
        empty = True
        bgt_poly.write("{} 2 0 0\n".format(len(verteces)))
        for j in range(len(verteces)):
            bgt_poly.write("{} {} {}\n".format(j+1, str(verteces[j][0]), str(verteces[j][1])))
        #segments
        bgt_poly.write("{}\n".format(len(verteces)))
        for j in range(len(verteces)):
            bgt_poly.write("{} {} {}\n".format(j+1, j+1, (j+1)%len(verteces)+1))
        #holes
        bgt_poly.write("0")
        verteces = []
        bgt_poly.close()
        continue
    elif line in ['\n'] and empty == True:
        continue
    empty = False
    values = map(float, line.split())
    verteces.append((values[0], values[1]))
bgt.close()
