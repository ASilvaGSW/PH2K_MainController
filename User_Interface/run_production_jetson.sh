#!/bin/bash
# Launcher for production run GUI on Jetson.
# Increases stack size to mitigate "Stack Smashing Detected" with Tkinter on ARM.
#
# Usage: ./run_production_jetson.sh
#    or: bash run_production_jetson.sh

cd "$(dirname "$0")"

# Increase stack size (default often 8MB; 16MB can help with Tkinter on Jetson)
ulimit -s 16384

# Optional: LD_PRELOAD workaround for some Jetson/Tkinter issues
# export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGLdispatch.so

python3 routes/run.py "$@"
