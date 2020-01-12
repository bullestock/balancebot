#! /usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division
import os, sys, re

from solid import *
from solid.utils import *

plugw = 38
plugh = 27

th = 3

e = 0.001
SEGMENTS = 48

def assembly():
    d = 12
    id = d + 2
    inner = translate([-23/2, -id/2, -e])(cube([23, id, 20]))
    outer = up(plugh/2)(cube([plugw, d, plugh], center = True))
    return outer - inner

if __name__ == '__main__':
    a = assembly()
    scad_render_to_file( a, file_header='$fn = %s;'%SEGMENTS, include_orig_code=True)
