#!/bin/bash

# Find the most recent .log file in the log directory
recent_log=$(ls -t log/*.log | head -n 1)

# Run the Python command with the most recent log file as a parameter
python plot.py "$recent_log"