#!/bin/bash

# Find the most recent .log file in the log directory
recent_log=$(ls -t log/*.log | head -n 1)

# Run the Python command with the most recent log file as a parameter
if [ "$1" == "a" ]; then
    python plot_map.py "$recent_log" "a"
elif [ "$1" == "b" ]; then
    python plot_map.py "$recent_log" "a"
else
    echo "Usage: $0 [a|b]"
    exit 1
fi