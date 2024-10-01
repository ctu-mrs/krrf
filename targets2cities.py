#!/usr/bin/env python3

import sys
import os
print('Number of arguments:', len(sys.argv), 'arguments.')
print('Argument List:', str(sys.argv))
#target_file = open(sys.argv[1], "r")
#cities_file = open(sys.argv[2], "w")

for file in os.listdir("./targets"):
    cities_file = open("./targets/"+file[:-4] + ".cities", "w")
    target_file = open("./targets/"+file, "r")
    print("./targets/"+file[:-4] + ".cities", "./targets/"+file)
    all_string = ""
    all_string+="["
    while True:    

        line = target_file.readline()
        print("Line", line)
        print(all_string)
        print(len(line))
        if not line or len(line) < 3:
            break
        all_string+="'["
        values = line.split()
        print(values)

        for j in range(3):
            if j == 2:
                all_string+=str(values[j])
            else:
                all_string+=str(values[j])
                all_string+=";"
        all_string+=str("]',")
    all_string = all_string[:-1] + ']'

    cities_file.write(all_string)
