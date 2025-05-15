#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import os
import time

import matplotlib
import numpy as np
from alg_deteccion_lado import DecideSide
from Robot import Robot

matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

from enum import Enum

# from Robot import Robot
from lib.MapLib import Map2D


# Calibration using ultrasonic sensors
def calibrate_before_maze(robot: Robot, side_to_calibrate, maze_ini_position, w_base=0.5):
    if side_to_calibrate == "left":
        sign = 1
        angle_to_reach = 0
    elif side_to_calibrate == "right":
        sign = -1
        angle_to_reach = -np.pi
    # robot.setSpeed(0, sign*w_base)
    # # robot.waitAngle(sign*np.pi/2)
    # robot.waitAngle(angle_to_reach)
    robot.calibrateOdometry(distObj=55)
    robot.setSpeed(0, -sign*w_base)
    robot.waitAngle(-np.pi/2, initial_w=-sign*w_base)
    print(robot.readOdometry())
    robot.setSpeed(0, 0)
    print(robot.readOdometry())
    robot.calibrateOdometry(distObj=95, 
                            expected_x = maze_ini_position[0], 
                            expected_y = maze_ini_position[1])
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
        # print("Current cell: ", x_act, y_act)
        # print("Current angle: ", th_act)
        # print("Current position with ReadOdometry: ", robot.readOdometry())
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


def calibrate_before_recognition(robot, center_calibrate_position, w_base=np.pi/6):
    _, _, th = robot.readOdometry()
    # Si está cerca de 0 giramos hacia 180 (para evitar que la cesta se choque con la pared)
    if th < np.pi/2 or th > -np.pi/2:
        robot.setSpeed(0, w_base)
        robot.waitAngle(np.pi, initial_w=w_base)
    else: 
        robot.setSpeed(0, -w_base)
        robot.waitAngle(0, initial_w=-w_base)
        
    robot.setSpeed(0, 0)
    robot.calibrateOdometry(distObj=75)
    
    if th < np.pi/2 or th > -np.pi/2:
        robot.setSpeed(0, -w_base)
        robot.waitAngle(np.pi/2, initial_w=-w_base)
    else:
        robot.setSpeed(0, w_base)
        robot.waitAngle(np.pi/2, initial_w=w_base)
    
    
    robot.setSpeed(0, 0)
    robot.calibrateOdometry(distObj=75, 
                            expected_x = center_calibrate_position[0], 
                            expected_y = center_calibrate_position[1]) 
    return


def main(args):
    _map = {
        'A' :{
            'map_file': args.mapfileA,
            'image_path': args.r2d2,

            'maze_ini_x': 1,
            'maze_ini_y': 2,
            'maze_ini_th': -np.pi/2,
            'maze_ini_position': (1*.4+.2, 2*.4+.2),
            'maze_end_x': 3,
            'maze_end_y': 3,
            'sign': -1,

            'initial_position': (0.6,2.6),

            'side_to_calibrate': "right",
            
            'center_calibrate_position': (5*.4, 5*.4),
            'l_position_x': 3*.4+.2,
            'r_position_x': 6*.4+.2,
            'robot' : 'R2D2'
        },
        'B' :{
            'map_file': args.mapfileB,
            'image_path': args.bb8,
            'maze_ini_x': 5,
            'maze_ini_y': 2,
            'maze_ini_th': -np.pi/2,
            'maze_ini_position': (5*.4+.2, 2*.4+.2),
            'maze_end_x': 3,
            'maze_end_y': 3,
            'sign': 1,

            'initial_position': (2.2,2.6),

            'side_to_calibrate': "left",
            
            'center_calibrate_position': (2*.4, 5*.4),
            'l_position_x': 0+.2,
            'r_position_x': 3*.4+.2,
            'robot' : 'BB8'
        }
    }
    try:
        robot = Robot(log_filename=args.log, verbose=args.verbose)
        

        # Detect side
        if args.mapSide is None:
            starting_position = 'A' if robot.is_starting_position_A() else 'B'
        else:
            starting_position = args.mapSide.upper()
        
        map_config = _map[starting_position]
        map_config.update({
            's_w_base': np.pi/4,
            's_radioD': 0.4,
            
            'initial_angle': -np.pi/2,
            'final_position_y': 7*.4 + .2,
            'final_position_th': np.pi/2,

            'ball_v_base': 0.2,
            'ball_w_base': np.pi/2,
            'ball_catch': True,
            'ball_targetX': 320/2-10,
            'ball_minObjectiveTargetSize': 4500,
            'ball_maxObjectiveTargetSize': 8500,
            'ball_detection_tolerance': 100,
            'ball_maxYValue': 32,
            'ball_colorMasks': {
                'red': [
                    (np.array([0, 70, 50]),np.array([5, 255, 255])),
                    (np.array([170, 70, 50]),np.array([180, 255, 255])),
                ],
                'blue': [
                    (np.array([100, 70, 50]),np.array([130, 255, 255])),
                    (np.array([90, 70, 50]),np.array([110, 255, 255])),
                ]
            }
        })
        
        map_images = {
            'A': './fotos/BB8_s.png',
            'B': './fotos/R2-D2_s.png',
            'robot': map_config['robot']
        }
        

        print("Running on map ", starting_position)
                    
        robot.disable_light_sensor()

        if not os.path.isfile(map_config['map_file']):
            print('Map file %s does not exist' % map_config['map_file'])
            exit(1)
        elif not os.path.isfile(map_config['image_path']):
            print("Image file %s does not exist" % map_config['image_path'])
            exit(1)
           
        robot.definePositionValues(
            map_config['initial_position'][0], 
            map_config['initial_position'][1], 
            map_config['initial_angle']
        ) 

        robot.startOdometry()

        # wait until 'enter' is pressed
        if args.waitKey:
            input("Press Enter to continue...")


        # do S 
        robot.doS(
            starting_position,
            w_base=map_config['s_w_base'], 
            radioD=map_config['s_radioD']
        )

        # calibrate robot position
        calibrate_before_maze(
            robot,
            side_to_calibrate=map_config['side_to_calibrate'],
            maze_ini_position=map_config['maze_ini_position'],
            w_base=map_config['s_w_base']
        )
        
        # do maze
        solve_maze(
            robot, 
            map_config['map_file'], 
            map_config['maze_ini_x'], 
            map_config['maze_ini_y'],
            map_config['maze_ini_th'], 
            map_config['maze_end_x'], 
            map_config['maze_end_y']
        )

        # go for ball
        robot.trackObject(
            v_base=map_config['ball_v_base'], 
            w_base=map_config['ball_w_base'], 
            catch=map_config['ball_catch'],
            targetX=map_config['ball_targetX'],
            minObjectiveTargetSize=map_config['ball_minObjectiveTargetSize'],
            maxObjectiveTargetSize=map_config['ball_maxObjectiveTargetSize'],
            detection_tolerance=map_config['ball_detection_tolerance'],
            maxYValue=map_config['ball_maxYValue'],
            colorMasks=map_config['ball_colorMasks'][args.mask],
            starting_w=map_config['side_to_calibrate']
        )

       
        
        calibrate_before_recognition(robot, map_config['center_calibrate_position'], w_base=map_config['s_w_base'])
        
        side = DecideSide(robot, map_images['A'], map_images['B'], map_images['robot'])
        
        print("Decide side: %s" % side)
        # robot.setSpeed(0, 0)
        if side == "l":
            # robot.setSpeed(0, map_config['s_w_base'])
            # robot.waitAngle(3*np.pi/4)
            # robot.setSpeed(0, 0)
            print("Going left x:", map_config['l_position_x'])
            robot.setSpeed(0, map_config['s_w_base'])
            robot.waitAngle(3*np.pi/4, initial_w=-map_config['s_w_base'])
            robot.go_to_free(
                x_obj=map_config['l_position_x'],
                y_obj=map_config['final_position_y'],
                th_obj=map_config['final_position_th']
            )
        else:
            # robot.setSpeed(0, -map_config['s_w_base'])
            # robot.waitAngle(np.pi/4)
            # robot.setSpeed(0, 0)   
            print("Going right x:", map_config['r_position_x'])
            actual_x, actual_y, actual_th = robot.readOdometry()
            print("Actual position: ", actual_x, actual_y, actual_th)
            if actual_th < np.pi/2:
                robot.setSpeed(0, 0.5)
                robot.waitAngle(np.pi/2, initial_w=-map_config['s_w_base']) 
            else:
                robot.setSpeed(0, -0.5)
                robot.waitAngle(np.pi/2, initial_w=map_config['s_w_base'])
            
            # robot.calibrateOdometry(distObj=125, expected_y=actual_y-0.4)
            robot.setSpeed(0, -map_config['s_w_base'])
            robot.waitAngle(0, initial_w=-map_config['s_w_base'])
            robot.setSpeed(0, 0)
            robot.calibrateOdometry(distObj=30, expected_x=map_config['r_position_x']-0.2)
            robot.setSpeed(0, map_config['s_w_base'])
            robot.waitAngle(np.pi/2, initial_w=map_config['s_w_base'])
            robot.setSpeed(0, 0)
            v_base = 0.2
            robot.setSpeed(v_base, 0)
            robot.waitPositionWithWallCorrection(
                x_deseado=map_config['r_position_x'],
                y_deseado=map_config['final_position_y'],
                v_base=v_base,
                tolerancia=0.02
            )
            robot.setSpeed(0, 0)
            # robot.go_to_free(
            #     x_obj=map_config['r_position_x'],
            #     y_obj=map_config['final_position_y'],
            #     th_obj=map_config['final_position_th']
            # )
        
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
                        type=str, default=None)
    
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
