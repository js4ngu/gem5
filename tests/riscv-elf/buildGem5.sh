#!/bin/bash
# Clear the terminal
clear

# Check if a source file is provided as an argument
#if [ -z "$1" ]; then
#  echo "Usage: $0 <source_file>"
#  exit 1
#fi
# Set the file name to compile from the first argument
#SOURCE_FILE="$1"

# Change directory to the specified path
cd /home/gem5 || exit 1

scons build/RISCV/gem5.opt -j 16
