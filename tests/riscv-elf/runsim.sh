#!/bin/bash
# Clear the terminal
clear

# Define the configuration file, debug flags, and output directory
CONFIG_DIR="/home/gem5/configs/learning_gem5/part1/RiscvMinorCPUnew.py"
DEBUG_FLAGS="Exec"  # You can add more debug flags as needed
OUTPUT_DIR="/home/gem5/tests/riscv-elf/out"  # Custom output directory

# Check if the configuration file exists before proceeding
if [[ ! -f "$CONFIG_DIR" ]]; then
  echo "Configuration file not found: $CONFIG_DIR"
  exit 1
fi

# Ensure the output directory exists
mkdir -p "$OUTPUT_DIR"

# Change to the gem5 root directory
cd /home/gem5 || { echo "Failed to change directory to /home/gem5"; exit 1; }

# Run the gem5 simulation with debug flags and explicitly set the output directory
#build/RISCV/gem5.opt --outdir="$OUTPUT_DIR" --debug-flags="$DEBUG_FLAGS" "$CONFIG_DIR" > "$OUTPUT_DIR/gem5_terminal_output.log" 2>&1

# If you want to run without debug flags, you can comment out the above and uncomment below:
build/RISCV/gem5.opt --outdir="$OUTPUT_DIR" "$CONFIG_DIR"
