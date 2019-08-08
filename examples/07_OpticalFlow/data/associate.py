#!/usr/bin/python3
import argparse
import sys
import os
import numpy


def read_file_list(filename):
    with open(filename) as file:
        data = file.read()
        lines = data.replace(","," ").replace("\t"," ").split("\n") 
        lst = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
        lst = [(float(l[0]),l[1:]) for l in lst if len(l)>1]
    return dict(lst)

def associate(first_dict, second_dict, offset, max_difference):
    first_keys = first_dict.keys()
    second_keys = second_dict.keys()
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        matches.append((a, b))
    matches.sort()
    return matches

if __name__ == '__main__':  
    parser = argparse.ArgumentParser(description='''
        This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('rgb_file', help='rgb text file (format: timestamp data)')
    parser.add_argument('depth_file', help='depth text file (format: timestamp data)')
    parser.add_argument('--output_file', help='output text filename', default='associate.txt')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    args = parser.parse_args()

    rgb_dict = read_file_list(args.rgb_file)
    depth_dict = read_file_list(args.depth_file)

    matches = associate(rgb_dict, depth_dict,float(args.offset),float(args.max_difference))    

    with open(args.output_file, "w") as file:
        for a,b in matches:
            line = "%f %s %f %s\n"%(a," ".join(rgb_dict[a]),b-float(args.offset)," ".join(depth_dict[b]))
            file.write(line)
