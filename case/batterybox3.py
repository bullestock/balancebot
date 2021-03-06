#! /usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division
import os, sys, re

from solid import *
from solid.utils import *

lidw = 38.5
lidh = 137.5 - 1.5
lidth = 3

holew = 32
holeh = 25
holeoffset = 17

th = 3

e = 0.001
SEGMENTS = 48

def snap():
    w = 10
    th = 2
    h = 6
    return cube([w, th, h]) + translate([0, -1+e, h-1])(cube([w, 1, 1]))

def assembly():
    lid = cube([lidw, lidh, lidth])
    s1 = translate([7, 0, 0])(snap())
    s2 = translate([lidw - 17, 0, 0])(snap())
    h = translate([(lidw - holew)/2, lidh - holeh - holeoffset, -1])(cube([holew, holeh, lidth+2]))
    return lid + s1 + s2 - h

if __name__ == '__main__':
    a = assembly()
    scad_render_to_file( a, file_header='$fn = %s;'%SEGMENTS, include_orig_code=True)
