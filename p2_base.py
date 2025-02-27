#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
import math
import sys
from Robot import Robot

# Avanza num_baldosas a velocidad v_base
# Cada baldosa tiene un tamaño de 0.4 m
def pruebaBaldosas(robot, v_base, num_baldosas):
    TAMANO_BALDOSA = 0.4
    tiempo = (TAMANO_BALDOSA * num_baldosas) / v_base
    robot.setSpeed(v_base, 0)
    time.sleep(tiempo)
    
# Avanza num_baldosas a velocidad v_base
# Cada baldosa tiene un tamaño de 0.4 m
def pruebaBaldosasOdometria(robot, v_base, num_baldosas):
    TAMANO_BALDOSA = 0.4
    robot.setSpeed(v_base, 0)
    robot.waitPosition(robot.x.value + TAMANO_BALDOSA * num_baldosas,
                           robot.y.value)
    
# Gira num_giros a velocidad w_base
# Cada giro es de 180 grados
def pruebaGiros180(robot, w_base, num_giros):
    tiempo = (np.pi * num_giros) / w_base
    robot.setSpeed(0, w_base)
    if(tiempo <0):
        tiempo = -tiempo
    time.sleep(tiempo)

# def ocho(robot, radioD, w_base):

#     # 1. Giro 90 grados 
#     # 0 -> -pi/2
#     robot.setSpeed(0, w_base)
#     th = None
#     while th == None or th > -np.pi/2:
#         time.sleep(0.01)
#         _,_,th = robot.readOdometry()
#         # print("th = %.2f" % (th))

#     print("1. Giro 90 grados ")
#     print("th/pi = %.2f" % (th/np.pi))
    
#     # 2. Semicírculo radio d izquierda
#     # -pi/2 -> pi/2
#     robot.setSpeed(w_base * radioD, -w_base)
#     th = None
#     while th == None or th < np.pi/2:
#         time.sleep(0.01)
#         _,_,th = robot.readOdometry()
#         # print("th = %.2f" % (th))

#     print("2. Semicírculo radio d izquierda")
#     print("th = %.2f" % (th))
#     print("th/pi = %.2f" % (th/np.pi))

#     # 3.1. Círculo radio d derecha
#     # pi/2 -> -pi/2
#     robot.setSpeed(w_base * radioD, w_base)
#     th = None
#     while th == None or th > -np.pi/2:
#         time.sleep(0.01)
#         _,_,th = robot.readOdometry()
#         # print("th = %.2f" % (th))

#     print("3.1. Círculo radio d derecha")
#     print("th = %.2f" % (th))
#     print("th/pi = %.2f" % (th/np.pi))

#     # 3.2. Círculo radio d derecha
#     # -pi/2 -> pi/2
#     robot.setSpeed(w_base * radioD, w_base)
#     th = None
#     while th == None or th < np.pi/2:
#         time.sleep(0.01)
#         _,_,th = robot.readOdometry()
#         # print("th = %.2f" % (th))
        
#     print("3.2. Círculo radio d derecha")
#     print("th = %.2f" % (th))
#     print("th/pi = %.2f" % (th/np.pi))
    
#     # 4. Semicírculo radio d izquierda
#     # pi/2 -> -pi/2
#     robot.setSpeed(w_base * radioD, -w_base)
#     th = None
#     while th == None or th > -np.pi/2:
#         time.sleep(0.01)
#         _,_,th = robot.readOdometry()
#         # print("th = %.2f" % (th))

#     print("4. Semicírculo radio d izquierda")
#     print("th = %.2f" % (th))
#     print("th/pi = %.2f" % (th/np.pi))

#     robot.setSpeed(0, 0)
    

def trayectoriaOcho(robot, radioD, w_base):
    # 1. Giro 90 grados 
    robot.setSpeed(0, -w_base)
    robot.waitAngle(-np.pi/2)
    print("1. Giro 90 grados ")

    # 2. Semicírculo radio d izquierda
    robot.setSpeed(w_base * radioD, w_base)
    robot.waitAngle(np.pi/2)
    print("2. Semicírculo radio d izquierda")

    # 3.1. Círculo radio d derecha
    robot.setSpeed(w_base * radioD, -w_base)
    robot.waitAngle(-np.pi/2)
    print("3.1. Círculo radio d derecha")

    # 3.2. Círculo radio d derecha
    robot.setSpeed(w_base * radioD, -w_base)
    robot.waitAngle(np.pi/2)
    print("3.2. Círculo radio d derecha")

    # 4. Semicírculo radio d izquierda
    robot.setSpeed(w_base * radioD, w_base)
    robot.waitAngle(-np.pi/2)
    print("4. Semicírculo radio d izquierda")

    robot.setSpeed(0, 0)


def trayectoria2Esperas(robot, radioD, w_base, alfa, R, v_base):
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


def trayectoriaTang(robot, radioD, radioAlfa, R, w_base, v_base):
    
    distancia_centros = math.sqrt(R**2 - (radioD-radioAlfa)**2)
    angulo_tangencia = math.pi/2 + math.asin((radioD-radioAlfa)/distancia_centros)
    print("angulo_tangencia = %.2f" % (angulo_tangencia))
    print("distancia_centros = %.2f" % (distancia_centros))
    
    # 1. Giro 90 grados
    print("1. Giro 90 grados")
    robot.setSpeed(0, w_base)
    robot.waitAngle(np.pi/2)
    

    # 2. Cuarto de círculo radio radioAlfa (izquierda)
    print("2. Cuarto de círculo radio radioAlfa (izquierda)")
    v_alfa = w_base * radioAlfa
    robot.setSpeed(v_alfa, -w_base)
    robot.waitAngle(angulo_tangencia - np.pi/2)
    robot.setSpeed(0, 0)

    # 3. Línea recta longitud R
    print("3. Línea recta longitud R")
    x, y, th = robot.readOdometry()
    x_obj = x + R * math.cos(th)
    y_obj = y + R * math.sin(th)
    robot.setSpeed(v_base, 0)
    robot.waitPosition(x_obj, y_obj)
    robot.setSpeed(0, 0)

    # 4. Semicírculo radio d (derecha)
    print("4. Semicírculo radio d (derecha)")
    v_d = w_base * radioD
    robot.setSpeed(v_d, -w_base)
    robot.waitAngle(-np.pi/2)
    robot.waitAngle(-np.pi/2-angulo_tangencia)
    robot.setSpeed(0, 0)

    # 5. Línea recta longitud R
    print("5. Línea recta longitud R")
    x, y, th = robot.readOdometry()
    robot.setSpeed(v_base, 0)
    x_obj = x + R * math.cos(th)
    y_obj = y + R * math.sin(th)
    robot.waitPosition(x_obj, y_obj)
    robot.setSpeed(0, 0)

    # 6. Cuarto de círculo radio radioAlfa (derecha)
    print("6. Cuarto de círculo radio radioAlfa (derecha)")
    robot.setSpeed(v_alfa, -w_base)
    robot.waitAngle(np.pi/2)

    robot.setSpeed(0, 0)


def trayectorioGiros(robot, w_base):
    # 1. Giro 90 grados a la izquierda
    robot.setSpeed(0, w_base)
    robot.waitAngle(np.pi/2) 
    

def main(args):
    try:
        if args.radioD < 0 or args.radioAlfa < 0:
            print('radioD y radioAlfa tienen que ser positivos')
            exit(1)
            
        robot = Robot(log_filename=args.log)

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # testcase = 0 # ochos
        # testcase = 1 # 2 esferas
        # testcase = 2 # otro
        testcase = args.testcase
        # 1. launch updateOdometry Process()
        robot.startOdometry()

        if testcase == 0:
            # print(f"trayectoriaOcho(robot, {args.radioD}, 0.4)")
            trayectoriaOcho(robot, args.radioD, 0.4)
        elif testcase == 1:
            # print(f"trayectoriaTang(robot, {args.radioD}, {args.radioAlfa}, 0.6, 0.1, 0.1)")
            trayectoriaTang(robot, args.radioD, args.radioAlfa, args.R, 0.4, 0.2)   
        else:       
            # print("pruebaBaldosasOdometria(robot, 0.2, 4)")
            pruebaBaldosasOdometria(robot, 0.2, 4)

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
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
    parser.add_argument("-d", "--radioD", help="Radio para el 8 (m)",
                        type=float, default=0.4)
    parser.add_argument("-a", "--radioAlfa", help="Radio para alfa de la trayectoria 2 (m)",
                        type=float, default=0.4)
    parser.add_argument("-R", "--R", help="Longitud de la recta de la trayectoria 2 (m)",
                        type=float, default=0.8)
    parser.add_argument("-l", "--log", help="Log file",
                        type=str, default=None) 
    parser.add_argument("-t", "--testcase", help="Testcase",
                        type=int, default=2)
    # TODO: velocidad base /angular??
    args = parser.parse_args()

    main(args)



