#!/bin/bash

for file in *.in ; do 
    echo $file Triangles that intersect:; ../../build/triangles < $file
done