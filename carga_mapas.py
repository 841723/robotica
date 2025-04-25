#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
import math
import cv2
from Robot import Robot

from lib.MapLib import Map2D

# Calibration using ultrasonic sensors
def calibrate_before_maze(robot: Robot, side_to_calibrate, w_base=0.5):
    if side_to_calibrate == "left":
        sign = 1
        angle_to_reach = 0
    elif side_to_calibrate == "right":
        sign = -1
        angle_to_reach = -np.pi
    robot.setSpeed(0, sign*w_base)
    # robot.waitAngle(sign*np.pi/2)
    robot.waitAngle(angle_to_reach)
    robot.calibrateOdometry(number_of_cells=1)
    robot.setSpeed(0, -sign*w_base)
    robot.waitAngle(-np.pi/2)
    robot.calibrateOdometry(number_of_cells=2)
    
# def calibrate_using_image(robot: Robot): # TODO 


def solve_maze(robot: Robot, map_file, maze_ini_x, maze_ini_y, 
               maze_ini_th, maze_end_x, maze_end_y):
    # 1. load map and compute costs and path
    myMap = Map2D(map_file)
    myMap.verbose = True
    
    myMap.planPath(maze_ini_x, maze_ini_y, maze_end_x, maze_end_y)
    print(myMap.currentPath)
    # myMap.drawMap(saveSnapshot=False)
    
    # Initialize Odometry. 
    x_ini_meter = maze_ini_x * myMap.sizeCell/1000 + myMap.sizeCell/2000 
    y_ini_meter = maze_ini_y * myMap.sizeCell/1000 + myMap.sizeCell/2000
    robot.definePositionValues(x_ini_meter, y_ini_meter, maze_ini_th)
    robot.startOdometry()
        
    # 3. perform trajectory in mm
    robotLocations = [ [x_ini_meter*1000, y_ini_meter*1000, 0] ]


    i = 0
    path_found = True
    while i < len(myMap.currentPath):
        print("-------------------------------------------------------------------------")
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
            if args.verbose:
                print("Obstacle detected at ", x_act, y_act, ", replanning path")
                print("Direction of the obstacle: ", direction)
            
            x_prev, y_prev = myMap.currentPath[i-1]
            if args.verbose:
                print("Current cell: ", x_prev, y_prev)
            try:
                path_found = myMap.replan_path(x_prev, y_prev, direction, maze_end_x, maze_end_y)
            except Exception as e:
                print("Error during path replanning: ", e)
            if path_found:
                i = 0  # Restart from the beginning of the new path
                print("Path found")
                continue
            else:
                print("No path found, rebooting map")
                myMap = Map2D(map_file)
                myMap.replan_path(x_prev, y_prev, direction, maze_end_x, maze_end_y)
                i=0
                continue
                
            
        i += 1
        print("Current location: ", robot.readOdometry())
        print("----------------------------------------")

    # 4. wrap up and close stuff ...
    # This currently unconfigure the sensors, disable the motors,
    # and restore the LED to the control of the BrickPi3 firmware.
    robot.setSpeed(0, 0)


def main(args):
    robot = Robot(log_filename=args.log, verbose=args.verbose)
    
    # Detect side
    if robot.is_starting_position_A():
        print("Starting position A")
        starting_position = "A"
        map_file = args.mapfileA
        image_path = args.r2d2
        maze_ini_x = 1
        maze_ini_y = 2
        maze_ini_th = -np.pi/2
        maze_end_x = 3
        maze_end_y = 3
        side_to_calibrate = "right"
    else:
        print("Starting position B")
        starting_position = "B"
        map_file = args.mapfileB
        image_path = args.bb8
        
        maze_ini_x = 5
        maze_ini_y = 2 
        maze_ini_th = -np.pi/2 # TODO: este ángulo?
        maze_end_x = 3
        maze_end_y = 3
        side_to_calibrate = "left"
        
    robot.disable_light_sensor()
    
    try:
        if not os.path.isfile(map_file):
            print('Map file %s does not exist' % map_file)
            exit(1)
        elif not os.path.isfile(image_path):
            print("Image file %s does not exist" % image_path)
            exit(1)
           
        robot.definePositionValues(0,0, maze_ini_th)
        robot.startOdometry()
        # calibrate_before_maze(robot, side_to_calibrate=side_to_calibrate)
        solve_maze(robot, map_file, maze_ini_x, maze_ini_y,
                   maze_ini_th, maze_end_x, maze_end_y)

        

        time.sleep(2) 
        robot.stopOdometry()
        
        # myMap.drawMapWithRobotLocations(robotLocations, saveSnapshot=False)


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
    parser.add_argument("-A", "--mapfileA", help="Path to the map file for the starting position A",
                        type=str, default="mapas/mapaA_CARRERA.txt")
    parser.add_argument("-B", "--mapfileB", help="Path to the map file for the starting position B",
                        type=str, default="mapas/mapaB_CARRERA.txt")
    parser.add_argument("-bb8", help="Path to BB8 image",
                        type=str, default="fotos/BB8_s.png")
    parser.add_argument("-r2d2", help="Path to R2D2 image",
                        type=str, default="fotos/R2-D2_s.png")
    parser.add_argument("-l", "--log", help="Log file",
                        type=str, default="robot.log")
    parser.add_argument("-v", "--verbose", help="Increase output verbosity",
                        type=bool, default=True)
    args = parser.parse_args()
    main(args)