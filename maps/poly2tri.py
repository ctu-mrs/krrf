#!/usr/bin/env python

import sys

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

node = open(sys.argv[1], "r")
ele = open(sys.argv[2], "r")
argument1 = ""
argument1 = sys.argv[1].replace(".1.node", "") + ".tri";
tri = open(argument1, "w")

#obstacles = open("obst.txt", "w")

line_counter = 0
number_of_lines = len(node.readlines()) - 2
node.close()
node = open(sys.argv[1], "r")
verteces = []
segments = []
line = node.readline()
for j in range(number_of_lines):
    line = node.readline()
    values = map(float, line.split())
    verteces.append((100*values[1], 100*values[2]))

number_of_lines = len(ele.readlines()) - 2
ele.close()
ele = open(sys.argv[2], "r")
line = ele.readline()

for j in range(number_of_lines):
    line = ele.readline()
    values = map(int, line.split())
    print values
    tri.write("{} {} {} {} {} {}\n".format(verteces[values[1]-1][0], verteces[values[1]-1][1], verteces[values[2]-1][0], verteces[values[2]-1][1], verteces[values[3]-1][0], verteces[values[3]-1][1]))
tri.close()
ele.close()
node.close()
