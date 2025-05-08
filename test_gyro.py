#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
from Robot import Robot
from lib.utils import rad_to_deg

import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from lib.MapLib import Map2D

def main(args):
    
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot(log_filename=args.log, verbose=args.verbose)

        # 1. launch updateOdometry thread()
        robot.startOdometry()
        while True:
            # 2. get the current position of the robot
            x, y, th = robot.readOdometry()
            print("Current angle: ", th, "radians, ", rad_to_deg(th), "degrees")
            time.sleep(0.1)
        
    
    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
    #    robot.stopOdometry()
        print("Keyboard interrupt")
        robot.setSpeed(0, 0)
        robot.stopOdometry()
        print("STOPPED BY USER")
        
if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--log", help="Log file",
                type=str, default="default.log") 
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                    action="store_true", default=True)
    args = parser.parse_args()
    main(args)
        