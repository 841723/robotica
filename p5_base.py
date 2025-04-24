#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
from Robot import Robot


import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from lib.MapLib import Map2D


def main(args):
    try:
        robot = Robot(log_filename=args.log, verbose=args.verbose)
        
        robot.definePositionValues(0.6,3,-np.pi/2)
        time.sleep(2)

        robot.startOdometry()
        currentMap = 'A'
        # currentMap = 'B'

        # do S 
        robot.doS(currentMap)
    
        
        
        # wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.setSpeed(0, 0)
        time.sleep(2) 
        robot.stopOdometry()
        

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
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="./mapas/mapa0.txt")
    parser.add_argument("-l", "--log", help="Log file",
                type=str, default="default.log") 
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                    action="store_true", default=True)
    args = parser.parse_args()
    main(args)
