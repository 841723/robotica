#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot


def trayectoria1(robot, radioD, w_base):
    # 1. Giro 90 grados
    robot.setSpeed(0, -w_base)
    time.sleep(np.pi/(2*w_base))
    
    # 2. Semicírculo radio d izquierda
    robot.setSpeed(w_base * radioD, w_base)
    time.sleep(np.pi*radioD/w_base)
    
    # 3. Círculo radio d derecha
    robot.setSpeed(-w_base * radioD, -w_base)
    time.sleep(2*np.pi*radioD/w_base)
    
    # 4. Semicírculo radio d izquierda
    robot.setSpeed(w_base * radioD, w_base)
    time.sleep(np.pi*radioD/w_base)


def trayectoria2(robot, radioD, w_base, alfa, R, v_base):
    # 1. Giro 90 grados
    robot.setSpeed(0, w_base)
    time.sleep(np.pi/(2*w_base))
    
    # 2. Cuarto de círculo radio alfa (derecha)
    v_alfa = -w_base * alfa
    robot.setSpeed(v_alfa, -w_base)
    time.sleep(np.pi*v_alfa/(4*w_base))
    
    # 3. Línea recta longitud R
    robot.setSpeed(v_base, 0)
    time.sleep(R/v_base)
    
    # 4. Semicírculo radio d (derecha)
    v_d = -w_base * radioD
    robot.setSpeed(v_d, -w_base)
    time.sleep(np.pi*v_d/(2*w_base))
    
    # 5. Línea recta longitud R
    robot.setSpeed(v_base, 0)
    time.sleep(R/v_base)
    
    # 6. Cuarto de círculo radio alfa (derecha)
    robot.setSpeed(v_alfa, -w_base)
    time.sleep(np.pi*v_alfa/(4*w_base))

def trayectoria3(robot, v, tiempo):
    for _ in range(4):
        robot.setSpeed(v, 0)
        time.sleep(tiempo)

        # girar 90 grados
        robot.setSpeed(0, np.pi/4)
        time.sleep(np.pi/4)

    robot.setSpeed(0, 0)



def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory
        # trayectoria3(robot, 0.5, 1)
        robot.setSpeed(0.4,0)
        time.sleep(1)

        robot.setSpeed(0, np.pi/2)
        time.sleep(np.pi/2)

        robot.setSpeed(0,0)
    

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



