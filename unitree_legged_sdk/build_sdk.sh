#!/bin/bash
# build SDK
rm -r build  &&  mkdir build && cd build && cmake .. && make