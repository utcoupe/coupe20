#!/usr/bin/env python

import re

__author__ = "Thomas Fuhrmann"
__date__ = 24/10/2017

"""
Converts #define from a .h file to a dictionary :
#define VALUEA
#define VALUEB 'b'
#define VALUEC 42
In :
['VALUEA':"", 'VALUEB':"b", 'VALUEC':"42"]
"""
def protocol_parse(filename):
    parsed_dict = {}
    with open(filename, "r") as f:
        data = f.readlines()
        for line in data:
            if line.find("END_ORDERS") == -1:
                result = re.search('#define[ \t]+(\S+)[ \t]+(\S+)', line)
                if result is not None:
                    parsed_dict[result.group(1)] = result.group(2).replace("'", "")
            else:
                break
    return parsed_dict

