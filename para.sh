#!/bin/bash

# Define variables
REMOTE_HOST="pi@10.1.31.230"
REMOTE_DIR="/home/pi/robotica" 

ssh $REMOTE_HOST "$REMOTE_DIR/para.py"