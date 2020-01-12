#! /usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division
import os, sys, re

from solid import *
from solid.utils import *

pch = 57.5
pcr = 10

boxh = 35
boxw = 45 - 0.5
boxd = 142

plugw = 38
plugh = 30

th = 3

SEGMENTS = 48

def plugcutout():
    p = polygon(points = [ [0, 3.5], [3, 0], [17, 0], [17, 9], [3, 9], [0, 5.5] ])
    return translate([-17/2, 1, boxh/2 - 9/2])(rotate([90, 0, 0])(linear_extrude(height = 3+2)(p)))

def boxend():
    th = 3
    return translate([-boxw/2, -th, 0])(cube([boxw, th, boxh]))

def boxbody():
    return translate([-boxw/2, -boxd, 0])(cube([boxw, boxd, boxh]) - translate([3, 3, 3])(cube([boxw - 2*3, boxd, boxh])))

def ridge():
    return cube([8, 5, boxh-3])

def lidsupport():
    return cube([4, 2, boxh-3])

def handle():
    return rotate_extrude(angle = -180)(translate([13, 0, 0])(circle(r = 4)))

def front_indent():
    l = 15
    d = 2
    h = 5+1
    i = cube([l, 1, 1])
    return translate([-l - d, -boxd + th - 0.9, boxh - h])(i) + translate([ d, -boxd + th - 0.9, boxh - h])(i)
    
def back_indent():
    l = 12
    d = 2
    h = 5+1
    th2 = 5
    i = cube([1, l, 1])
    return translate([-boxw/2 + th - 0.92, - (l + th + th2), boxh - h])(i) + translate([boxw/2 - th - 0.1, - (l + th + th2), boxh - h])(i)
    
def assembly():
    r1 = translate([-boxw/2 + 2.9, -(5+20), 0])(ridge())
    r2 = translate([boxw/2 - 8 - 2.9, -(5+20), 0])(ridge())
    r3 = translate([-boxw/2, -(5+20), 2.9])(cube([boxw, 5, 8]))
    a = boxbody() + (boxend() - plugcutout()) + r1 + r2 + r3 + translate([0, -boxd, boxh/2])(handle())
    for i in range(0, 6):
        a = a + translate([-boxw/2 + 1.9, -3 - (i+1)*22.2, 0])(lidsupport())
        a = a + translate([boxw/2 - 4 - 1.9, -3 - (i+1)*22.2, 0])(lidsupport())
    return a + translate([-2, -boxd+3, 0])(lidsupport()) - front_indent() - back_indent()

if __name__ == '__main__':
    a = rotate([0, 0, 90])(assembly())
    scad_render_to_file( a, file_header='$fn = %s;'%SEGMENTS, include_orig_code=True)
