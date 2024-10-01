#!/usr/bin/env python

bgt = open("elkoMap.tri", "r")
bgt_poly = open("elkoMap.poly", "w")
bgt_node = open("elkoMap_node.txt", "w")

line_counter = 0
number_of_lines = len(bgt.readlines())
bgt.close()
bgt = open("elkoMap.tri", "r")
print number_of_lines
obst= [[0.0 for x in range(2)] for y in range(number_of_lines)]
print obst
verteces = []
vert_dict = {}
vert_indeces = []
segments = []
counter = 0
while True:
    line = bgt.readline()
    line_counter+=1
    #print line
    if not line:
        break
    values = map(int, line.split())
    #print values
    obst[line_counter-1][0] = float(values[0] + values[2] + values[4])/3
    obst[line_counter-1][1] = float(values[1] + values[3] + values[5])/3  
    for i in range (3):
        if (values[2*i], values[2*i+1]) not in verteces:
            counter +=1
            verteces.append((values[2*i], values[2*i+1]))
            vert_dict[(values[2*i], values[2*i+1])] = counter    
    #print obst
    segments.append((vert_dict[(values[0],values[1])],vert_dict[(values[2],values[3])]))
    segments.append((vert_dict[(values[2],values[3])],vert_dict[(values[4],values[5])]))
    segments.append((vert_dict[(values[4],values[5])],vert_dict[(values[0],values[1])]))
print obst
print verteces
print vert_dict
#verteces
bgt_node.write("{} 2 0 0\n".format(len(verteces)))
bgt_poly.write("{} 2 0 0\n".format(len(verteces)))
for j in range(len(verteces)):
    bgt_node.write("{} {} {}\n".format(j+1, str(verteces[j][0]), str(verteces[j][1])))
    bgt_poly.write("{} {} {}\n".format(j+1, str(verteces[j][0]), str(verteces[j][1])))
#segments
bgt_poly.write("{}\n".format(len(segments)))
for j in range(len(segments)):
    bgt_poly.write("{} {} {}\n".format(j+1, str(segments[j][0]), str(segments[j][1])))
#holes
bgt_poly.write("{}\n".format(len(obst)))
for j in range(len(obst)):
    bgt_poly.write("{} {} {}\n".format(j+1, str(obst[j][0]), str(obst[j][1])))

bgt.close()
