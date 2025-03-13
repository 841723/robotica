#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time

import cv2
import numpy as np
from Robot import Robot


def main(args):
    catch = True
    celebrate = False
    colorMasks = {
        'red': [
            (np.array([0, 70, 50]),np.array([10, 255, 255])),
            (np.array([170, 70, 50]),np.array([180, 255, 255])),
        ],
        'blue': [
            (np.array([100, 70, 50]),np.array([130, 255, 255])),
            (np.array([90, 70, 50]),np.array([110, 255, 255])),
        ]
    }
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot(log_filename=args.log, verbose=args.verbose)

        # 1. launch updateOdometry thread()
        robot.startOdometry()

        # 2. track object
        robot.trackObject(v_base=0.4, w_base=np.pi/2, catch=True, targetX=320/2-10, minObjectiveTargetSize=4500, maxObjectiveTargetSize=8500, detection_tolerance=30, maxYValue=32, colorMasks=colorMasks[args.mask])
        if args.celebrate:
            robot.setSpeed(0, 2*np.pi)
            time.sleep(2)
            robot.release()
            time.sleep(0.1)
            robot.catch()
            time.sleep(0.1)

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.release()
        robot.setSpeed(0, 0)
        time.sleep(2) 
        robot.stopOdometry()


    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
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
                    action="store_true", default=False)
    parser.add_argument("-c", "--celebrate", help="celebrate when ball is catched",
                    action="store_true", default=False)
    parser.add_argument("-m", "--mask", help="color of the ball to track",
                    type=str, default="red")
    args = parser.parse_args()

    main(args)


