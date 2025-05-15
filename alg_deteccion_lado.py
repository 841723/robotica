#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
import math
import sys
import os
import cv2
from Robot import Robot
import lib.sample_matching as sample_matching




def giro30gradosDerecha(robot, w_base, lado = "r"):
    # 1. Giro 30 grados
    if lado == "r":
        robot.setSpeed(0, w_base)
        robot.waitAngle(np.pi/8 + np.pi/2)
    else:
        robot.setSpeed(0, -w_base)
        robot.waitAngle(np.pi/2 - np.pi/8)
    robot.setSpeed(0, 0)
    print("1. Giro 30 grados")
    
def DecideSide(robot, bb8, r2d2, robot_deciding):
    # 2. Toma de foto
    found_bb8 = False
    found_r2d2 = False
    while not found_bb8 and not found_r2d2:
        img = robot.takePhoto()
        img = cv2.rotate(img, cv2.ROTATE_180)
        # 3. Match images
        bb8 = cv2.imread(bb8, cv2.IMREAD_COLOR)
        r2d2 = cv2.imread(r2d2, cv2.IMREAD_COLOR)
        
        bb8_center, found_bb8 = sample_matching.match_images(bb8, img)
        r2d2_center, found_r2d2 = sample_matching.match_images(r2d2, img)
        # 4. Decide side
        if found_bb8 and found_r2d2:
            if bb8_center[0] < r2d2_center[0]:
                if robot_deciding == "BB8":
                    return "l"
                else:
                    return "r"
            else:
                if robot_deciding == "BB8":
                    return "r"
                else:
                    return "l"
                
        _,_,th = robot.readOdometry()
        
        if th > np.pi/2:
            robot.setSpeed(0, -0.2)
            robot.waitAngle(np.pi/2)
        else:
            robot.setSpeed(0, 0.2)
            robot.waitAngle(np.pi/2)
        
        robot.setSpeed(0.2, 0)
        time.sleep(robot.P)
        robot.setSpeed(0, 0)
        if robot.get_obstacle_distance() < 0.15:
            robot.setSpeed(0, 0)
            print("Obstacle detected, stopping")
            break
        
    
    # if found_bb8 and bb8_center[0] < img.shape[1] / 2:
    #     print("BB8 found on the left")
    #     if robot_deciding == "BB8":
    #         return "l"
    #     else:
    #         return "r"
    # elif found_r2d2 and r2d2_center[0] < img.shape[1] / 2:
    #     print("R2D2 found on the left")
    #     if robot_deciding == "BB8":
    #         return "r"
    #     else:
    #         return "l"
        
    return None
        
    
            
            
    
    
    


def main(args):
    try:
        assert args.w_base > 0, "Base speed must be positive"
        assert args.bb8 != args.r2d2, "BB8 and R2D2 images must be different"
        assert args.bb8 is not None, "BB8 image path must be provided"
        assert args.r2d2 is not None, "R2D2 image path must be provided"
        assert os.path.isfile(args.bb8), "BB8 image file does not exist"
        assert os.path.isfile(args.r2d2), "R2D2 image file does not exist"


        robot = Robot(log_filename=args.log, verbose=args.verbose)

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # testcase = 0 # ochos
        # testcase = 1 # 2 esferas
        # testcase = 2 # otro
        # 1. launch updateOdometry Process()
        robot.definePositionValues(0, 0, np.pi/2)
        robot.startOdometry()
        robot.init_camera()
        side = DecideSide(robot, args.bb8, args.r2d2, args.robot)
        print("Decide side: %s" % side)
        # giro30gradosDerecha(robot, args.w_base, lado="l")
        
        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f, %.2f" % (robot.x.value, robot.y.value, robot.th.value, np.degrees(robot.th.value)))
        robot.lock_odometry.release()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        time.sleep(3)
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
    parser.add_argument("-v", "--verbose", help="Verbose mode",
                        action='store_true', default=False)
    parser.add_argument("-w", "--w_base", help="Base speed",
                        type=float, default=np.pi/4)
    parser.add_argument("-b", "--bb8", help="BB8 path image",
                        type=str, default="./fotos/BB8_s.png")
    parser.add_argument("-r", "--r2d2", help="R2D2 path image",
                        type=str, default="./fotos/R2-D2_s.png")
    parser.add_argument("-l", "--log", help="Log file",
                        type=str, default=None) 
    parser.add_argument("-s", "--robot", help="name of the robot that decides the side",
                        type=str, default="BB8")
    args = parser.parse_args()

    main(args)



