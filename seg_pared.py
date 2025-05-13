#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import numpy as np
import time
from Robot import Robot
from alg_deteccion_lado import DecideSide
import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot
from lib.MapLib import Map2D
from enum import Enum

def pruebaBaldosasOdometria(robot, v_base, num_baldosas):
    TAMANO_BALDOSA = 0.4
    robot.setSpeed(v_base, 0)
    robot.waitPosition(robot.x.value + TAMANO_BALDOSA * num_baldosas,
                           robot.y.value)

# Calibration using ultrasonic sensors
def calibrate_before_maze(robot: Robot, side_to_calibrate, w_base=0.5):
    if side_to_calibrate == "left":
        sign = 1
        angle_to_reach = 0
    elif side_to_calibrate == "right":
        sign = -1
        angle_to_reach = -np.pi
    # robot.setSpeed(0, sign*w_base)
    # # robot.waitAngle(sign*np.pi/2)
    # robot.waitAngle(angle_to_reach)
    robot.calibrateOdometry(number_of_cells=1)
    robot.setSpeed(0, -sign*w_base)
    robot.waitAngle(-np.pi/2)
    print(robot.readOdometry())
    robot.setSpeed(0, 0)
    print(robot.readOdometry())
    robot.calibrateOdometry(number_of_cells=2)
    robot.setSpeed(0, 0)
    
    
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
    print("initial theta: ", maze_ini_th)
        
    print("Initial position: ", robot.readOdometry())
    # 3. perform trajectory in mm
    # robotLocations = [ [x_ini_meter*1000, y_ini_meter*1000, 0] ]

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
        
        x_act, y_act, th_act, direction = robot.go_to_cell(x, y)
        # robotLocations.append([x_act*1000, y_act*1000, th_act])
        print("Current cell: ", x_act, y_act)
        print("Current angle: ", th_act)
        print("Current position with ReadOdometry: ", robot.readOdometry())
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
    
    # myMap.drawMapWithRobotLocations(robotLocations, saveSnapshot=False)


def calibrate_before_recognition(robot, w_base=np.pi/6):
    _, _, th = robot.readOdometry()
    # Si está cerca de 0 giramos hacia 180 (para evitar que la cesta se choque con la pared)
    if th < np.pi/2 or th > -np.pi/2:
        robot.setSpeed(0, w_base)
        robot.waitAngle(np.pi)
    else: 
        robot.setSpeed(0, -w_base)
        robot.waitAngle(0)
        
    robot.setSpeed(0, 0)
    robot.calibrateOdometry(distObj=75)
    
    if th < np.pi/2 or th > -np.pi/2:
        robot.setSpeed(0, -w_base)
    else:
        robot.setSpeed(0, w_base)
    
    robot.waitAngle(np.pi/2)
    robot.setSpeed(0, 0)
    robot.calibrateOdometry(distObj=75)
    return


def main(args):
    _map = {
        'A' :{
            'map_file': args.mapfileA,
            'image_path': args.r2d2,

            'maze_ini_x': 1,
            'maze_ini_y': 2,
            'maze_ini_th': -np.pi/2,
            'maze_end_x': 3,
            'maze_end_y': 3,

            'initial_position': (0.6,3),
            'initial_angle': -np.pi/2,

            'side_to_calibrate': "right"
        },
        'B' :{
            'map_file': args.mapfileB,
            'image_path': args.bb8,
            'maze_ini_x': 5,
            'maze_ini_y': 2,
            'maze_ini_th': -np.pi/2,
            'maze_end_x': 3,
            'maze_end_y': 3,

            'initial_position': (2.2,2.6),
            'initial_angle': -np.pi/2,

            'side_to_calibrate': "left"
        }
    }
    try:
        robot = Robot(log_filename=args.log, verbose=args.verbose)
        time.sleep(2)
        robot.startOdometry()
        robot.definePositionValues(0,0,np.pi/2)
        
        robot.waitPositionWithWallCorrection(
            0,
            1.5,
            v_base=0.1,
        )
        
        
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
    parser.add_argument("-A", "--mapfileA", help="Path to the map file for the starting position A",
                        type=str, default="mapas/mapaA_CARRERA.txt")
    parser.add_argument("-B", "--mapfileB", help="Path to the map file for the starting position B",
                        type=str, default="mapas/mapaB_CARRERA.txt")
    parser.add_argument("-s", "--mapSide", help="Map side to use (\"A\" or \"B\")",
                        type=str, default="B")
    
    parser.add_argument("-w", "--waitKey", help="Wait for key press before starting",
                        type=bool, default=True)
    
    parser.add_argument("-bb8", help="Path to BB8 image",
                        type=str, default="fotos/BB8_s.png")
    parser.add_argument("-r2d2", help="Path to R2D2 image",
                        type=str, default="fotos/R2-D2_s.png")
    
    parser.add_argument("-m", "--mask", help="color of the ball to track",
                        type=str, default="red")
                        
    parser.add_argument("-l", "--log", help="Log file",
                type=str, default="default.log") 
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                    action="store_true", default=True)
    args = parser.parse_args()
    main(args)
