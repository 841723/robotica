#!/bin/bash

# Define variables
REMOTE_HOST="pi@10.1.31.230"
REMOTE_DIR="/home/pi/robotica"  # Adjust this to your target directory
LOCAL_DIR="."  # Current directory, adjust if needed
FILE="p2_base.py"

# Copy all files from current directory to remote
# Copy all files from lib directory to remote
echo "Copying files to Raspberry Pi..."
scp $LOCAL_DIR/*.py $REMOTE_HOST:$REMOTE_DIR
# Ensure the remote lib directory exists
ssh $REMOTE_HOST "mkdir -p $REMOTE_DIR/lib"

# Copy all files from lib directory to remote
scp $LOCAL_DIR/lib/* $REMOTE_HOST:$REMOTE_DIR/lib

# If copy was successful, execute the Python script
if [ $? -eq 0 ]; then
    echo "Files copied successfully. Running $FILE..."
    ssh $REMOTE_HOST "cd $REMOTE_DIR && python3 $FILE"
else
    echo "Error copying files to Raspberry Pi"
    exit 1
fi