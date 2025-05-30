#!/bin/bash

# Detect the operating system
OS=$(uname -s)
export ARGUS_ROOT=$(pwd)

# Set the correct mpy-cross executable based on the OS
if [ "$OS" == "Linux" ]; then
    MPY_EXEC="mpy-cross"
elif [ "$OS" == "Darwin" ]; then
    MPY_EXEC="mpy-cross-macos"
elif [[ "$OS" == *_NT* ]]; then
    MPY_EXEC="mpy-cross"
else
    echo "Unsupported OS: $OS"
    exit 1
fi

# Make the correct mpy-cross executable
if [[ ! "$OS" == *_NT* ]]; then 
    chmod +x build_tools/$MPY_EXEC

    echo "$MPY_EXEC is now executable"
fi

# Detect the appropriate Python command
if command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null; then
    PYTHON_CMD="python"
else
    echo "Python is not installed or not found in PATH."
    exit 1
fi

echo "Using Python command: $PYTHON_CMD"

export ARGUS_SIMULATION_FLAG=1
export SIM_REAL_SPEEDUP=275
echo "ARGUS_SIMULATION_FLAG set to 1 for simulation mode."
$PYTHON_CMD build_tools/build-emulator.py
cd build/ && rm -rf sd && $PYTHON_CMD main.py
cd -