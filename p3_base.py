#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import time

import cv2
import numpy as np
from Robot import Robot


def main(args):
    catch = True
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot(log_filename=args.log)

        # 1. launch updateOdometry thread()
        robot.startOdometry()

        # 2. Loop running the tracking until ??, then catch the ball
        # TODO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
        
        res = robot.trackObject(catch=catch)
        # robot.catch()
        # robot.release()
        robot.setSpeed(0, 0)
        robot.setSpeed(0, np.pi)

        # if args.color == 'r':
        #     print("Tracking red ball ...")
        #     colorRangeMin = redMin
        #     colorRangeMax = redMax
            
        # else:
        #     print("Tracking blue ball ...")
        #     colorRangeMin = blueMin
        #     colorRangeMax = blueMax

        # res = robot.trackObject(colorRangeMin, colorRangeMax, target, targetSize, catch)
        # robot.setSpeed(0, 0)

        # if res:
        #   robot.catch()


        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors, 
        # and restore the LED to the control of the BrickPi3 firmware.
        time.sleep(10)
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


