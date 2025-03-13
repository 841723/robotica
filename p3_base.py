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
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot(log_filename=args.log, verbose=True)

        # 1. launch updateOdometry thread()
        robot.startOdometry()

        # 2. Loop running the tracking until ??, then catch the ball
        robot.trackObject(catch=catch)
        if celebrate:
            robot.setSpeed(0, 2*np.pi)
            time.sleep(2)
            for _ in range(3):
                robot.release()
                time.sleep(0.4)
                robot.catch()
                time.sleep(0.4)

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
    # parser.add_argument("-c", "--color", help="color of the ball to track",
    #                     type=float, default='r')
    parser.add_argument("-l", "--log", help="Log file",
                    type=str, default="default.log") 
    args = parser.parse_args()

    main(args)


