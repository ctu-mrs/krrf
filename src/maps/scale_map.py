#!/usr/bin/env python

import sys
import numpy as np

print ('Number of arguments:', len(sys.argv), 'arguments.')
print ('Argument List:' + str(sys.argv))
print ('File for scaling the map')

tri = open(sys.argv[1], "r")
scaled_tri = open(sys.argv[2], "w")
scale = float(sys.argv[3])

line_counter = 0
number_of_lines = len(tri.readlines())
tri.close()
tri = open(sys.argv[1], "r")
for j in range(number_of_lines):
    line = tri.readline()
    values = scale*np.array(list(map(float, line.split())))
    scaled_tri.write("{} {} {} {} {} {}\n".format(values[0], values[1], values[2], values[3], values[4], values[5]))

tri.close()
scaled_tri.close()