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

# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change these method signatures if you need, depending how you have implemented things


def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile
        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        myMap.verbose = True

        x_ini, y_ini, th_ini = 0,0, 0
        x_end, y_end = 2,0
        
        myMap.planPath(x_ini, y_ini, x_end, y_end)
        print(myMap.currentPath)
        # myMap.drawMap(saveSnapshot=False)
        
        # Initialize Odometry. 
        robot = Robot(log_filename=args.log, verbose=args.verbose)
        x_ini_meter = x_ini * myMap.sizeCell/1000 + myMap.sizeCell/2000 # El número de baldosas y se añade la media baldosa inicial
        y_ini_meter = y_ini * myMap.sizeCell/1000 + myMap.sizeCell/2000
        robot.definePositionValues(x_ini_meter, y_ini_meter, th_ini)
        robot.startOdometry()
            
        # 3. perform trajectory
        robotLocations = [ [x_ini_meter*1000, y_ini_meter*1000, 0] ] # The locations for the map have to be in mm


        i = 0
        while i < len(myMap.currentPath):
            print("Going to ", myMap.currentPath[i])
            x, y = myMap.currentPath[i]

            x *= myMap.sizeCell/1000
            y *= myMap.sizeCell/1000
            x += myMap.sizeCell/2000
            y += myMap.sizeCell/2000
            
            x_act, y_act, th_act, direction = robot.go_to(x, y)
            robotLocations.append([x_act*1000, y_act*1000, th_act])

            if direction is not None:
                # Obstacle detected, replan the path
                print("Obstacle detected at ", x_act, y_act, ", replanning path")
                myMap.replan_path(x_act, y_act, direction, x_end, y_end)
                i = 0  # Restart from the beginning of the new path
                continue
            i += 1

        # 4. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.setSpeed(0, 0)
        time.sleep(2) 
        robot.stopOdometry()
        
        myMap.drawMapWithRobotLocations(robotLocations, saveSnapshot=False)


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
                        default="./mapas/mapa1.txt")
    parser.add_argument("-l", "--log", help="Log file",
                type=str, default="default.log") 
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                    action="store_true", default=False)
    args = parser.parse_args()
    main(args)
