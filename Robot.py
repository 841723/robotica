#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import \
    print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division

import math
import sys
import time  # import the time library for the sleep function
from multiprocessing import Lock, Process, Value

import brickpi3  # import the BrickPi3 drivers
import cv2
import numpy as np
import picamera
from lib.simubot import hom, loc
from lib.utils import (c2m, deg_to_rad, distancia_angular, norm_pi, polares,
                       rad_to_deg)
from lib.utilsrobot import calcAngleSpeed, calcSearchSpeed, calcTrackSpeed
from numpy.linalg import inv
from picamera.array import PiRGBArray


class Robot:
    def __init__(self, log_filename=None, verbose=True):
        """
        Initialize basic robot params. 

        Initialize Motors and Sensors according to the set up in your robot
        """

        # Robot wheel radius 
        self.R = 28/1000

        # Distance between the wheels
        self.L = 114/1000

        # Robot Basket Positions (in degrees)
        self.PINZA_ARRIBA = 0
        self.PINZA_ABAJO = -98

        self.SIDE_CAMERA_OFF = 0
        self.SIDE_CAMERA_LEFT = 1
        self.SIDE_CAMERA_RIGHT = 2

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()
        self.PORT_MOTOR_PINZA = self.BP.PORT_A
        self.PORT_MOTOR_DERECHA = self.BP.PORT_B
        self.PORT_MOTOR_IZQUIERDA = self.BP.PORT_C

        # set up gyro
        self.PORT_GYRO = self.BP.PORT_3
        self.BP.set_sensor_type(self.PORT_GYRO, self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)


        # Configure sensors
        self.motor_dps_limit = 150
        self.motor_wait_time = self.motor_dps_limit / -self.PINZA_ABAJO

        # move motor A to the starting position
        self.BP.set_motor_limits(self.PORT_MOTOR_PINZA, dps=self.motor_dps_limit)
        self.BP.set_motor_position(self.PORT_MOTOR_PINZA, self.PINZA_ARRIBA)


        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.PORT_MOTOR_DERECHA,
           self.BP.get_motor_encoder(self.PORT_MOTOR_DERECHA))
        self.BP.offset_motor_encoder(self.PORT_MOTOR_IZQUIERDA,
           self.BP.get_motor_encoder(self.PORT_MOTOR_IZQUIERDA))
        
        # Ultrasonic sensor P4
        self.ultrasonic_sensor_port = self.BP.PORT_1
        self.BP.set_sensor_type(self.ultrasonic_sensor_port, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM) 
        self.SENSOR_LADO_DERECHO = self.BP.PORT_4
        self.BP.set_sensor_type(self.SENSOR_LADO_DERECHO, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        
        # Light sensor 
        self.light_sensor_port = self.BP.PORT_2
        self.BP.set_sensor_type(self.light_sensor_port, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

        # configure camera
        self.camera = None
        
        ## Verbose mode
        self.verbose = verbose

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.prev_th = Value('d',0.0)
        self.v = Value('d',0.0)
        self.w = Value('d',0.0)
        self.anguloDerecha = Value('d',0.0)
        self.anguloIzquierda = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        self.rawCapture = None

        self.ignore_small_w = Value('b', False)
        self.th_ini = Value('d',0.0)

        # If we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        
        # Odometry update period
        self.P = 0.05
        
        # Tracking update period
        self.tracking_period = 0.01


        # Minimum target size to consider the object as the target
        self.minTargetSize = 5

        # Camera resolution
        self.width = 320
        self.height = 240

        # Save into log file
        self.log_filename = None
        if log_filename is not None:
            self.log_filename = log_filename
        else:
            self.log_filename = "odometry_" + time.strftime("%H_%M_%S_%d_%m_%Y") + ".log"
        self.log_filename = "log/" + self.log_filename
        
        self.log_file = open(self.log_filename, "w")
        self.log_file.close()
        with open(self.log_filename, "a") as f:
            f.write("#%s\n" % time.strftime("%H:%M:%S %d/%m/%Y"))
        
        time.sleep(1)
        
    def setSpeed(self, v,w):
        """ Establece la velocidad de los motores según la velocidad lineal y angular dadas 
        :param v: velocidad lineal
        :param w: velocidad angular
        """
        if v == 0 and w == 0:
            # print("Parando motores")
            self.BP.set_motor_dps(self.PORT_MOTOR_IZQUIERDA, 0)
            self.BP.set_motor_dps(self.PORT_MOTOR_DERECHA, 0)
            # print("Current Odometry SetSpeed: X --> %.2f, Y --> %.2f, th --> %.2f" % (self.x.value, self.y.value, self.th.value))
            
        
        ## Calculo de las velocidades angulares de cada rueda
        w_d = v/self.R + w*self.L/(2*self.R)
        w_i = v/self.R - w*self.L/(2*self.R)
        
        # if self.verbose:
        #     print("v --> %.2f; w --> %.2f; w_d --> %.2f; w_i--> %.2f" % (v, w, w_d, w_i))

        ## Añadir las velocidades a los motores
        self.BP.set_motor_dps(self.PORT_MOTOR_IZQUIERDA, rad_to_deg(w_i))
        self.BP.set_motor_dps(self.PORT_MOTOR_DERECHA, rad_to_deg(w_d))
        
        ## Actualizar los valores
        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.lock_odometry.release()


    def readSpeed(self):
        """Obtención de la velocidad lineal y angular 
        
        :return: v, w
        """
        self.lock_odometry.acquire()
        v = self.v.value
        w = self.w.value
        self.lock_odometry.release()
        return v, w

    def readOdometry(self):
        """Devuelve la posición y orientación del robot en un plano 2D 

        :return: x, y, th
        """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = norm_pi(self.th.value)
        self.lock_odometry.release()
        return x, y, th

    def initializeSensors(self, timeout=10):
        """Inicializa los sensores del robot
        """
        sensor_error = True
        start_time = time.time()
        while sensor_error:
            try:
                self.BP.get_sensor(self.PORT_GYRO)
                self.BP.get_sensor(self.ultrasonic_sensor_port)
                self.BP.get_sensor(self.SENSOR_LADO_DERECHO)
                sensor_error = False
                print("Sensores inicializados correctamente")
            except brickpi3.SensorError as error:
                if self.verbose:
                    print("Error initializing sensors: ", error)
                if time.time() - start_time > timeout:
                    raise RuntimeError("Sensor initialization timed out")
                time.sleep(0.2)

    def startOdometry(self):
        """This starts a new process/thread that will be updating the odometry periodically 
        """
        self.finished.value = False
        self.initializeSensors()
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)
        time.sleep(2)      
        
    def gyro_sensor(self):
        """Get the gyro sensor value
        """
        try:
            gyro = self.BP.get_sensor(self.PORT_GYRO)
            return gyro[0]
        except brickpi3.SensorError as error:
            if self.verbose:
                print("Error reading gyro sensor: ", error)
            return None

    def set_ignore_small_w(self, ignore): 
        self.lock_odometry.acquire() 
        self.ignore_small_w.value = ignore
        self.lock_odometry.release()

    def updateOdometry(self): 
        """This function will be run periodically to update the odometry values
         """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()
            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                [encoderIzquierda, encoderDerecha] = [self.BP.get_motor_encoder(self.PORT_MOTOR_IZQUIERDA),
                   self.BP.get_motor_encoder(self.PORT_MOTOR_DERECHA)]
                
                
                resta1 = encoderIzquierda - self.anguloIzquierda.value
                resta2 = encoderDerecha - self.anguloDerecha.value
                wIzquierda = float(deg_to_rad(resta1) / self.P)
                wDerecha = float(deg_to_rad(resta2) / self.P)

                self.anguloDerecha.value = encoderDerecha
                self.anguloIzquierda.value = encoderIzquierda
                
                v = self.R/2 * (wDerecha + wIzquierda)
                # Calcular la velocidad angular usando la odometría
                w = self.R/self.L * (wDerecha - wIzquierda)
                # Calcular la velocidad angular usando el giroscopio
                
                
                self.lock_odometry.acquire() 
                
                if self.ignore_small_w.value:
                        if w < 0.3 and w > -0.3:
                            w = 0
                self.lock_odometry.release()

                x_read, y_read, th_read = self.readOdometry()
                dx, dy, new_th = 0, 0, 0
                try:
                    new_th = self.BP.get_sensor(self.PORT_GYRO)
                    new_th = norm_pi(deg_to_rad(-new_th[0])+self.th_ini.value)
                    
                    # prev_th sólo usada por updateOdometry
                    # w = self.angular_velocity(new_th, self.prev_th.value, self.P)
                    
                except brickpi3.SensorError as error:
                    if self.verbose:
                        print("Error reading gyro sensor: ", error)
                    # Tomamos el ángulo de la odometría	
                    new_th = th_read + w*self.P
                
                
                if w == 0:
                    dx = self.P*v*np.cos(th_read)
                    dy = self.P*v*np.sin(th_read)
                else:
                    dx = v/w*(np.sin(new_th) - np.sin(th_read))
                    dy = -v/w*(np.cos(new_th) - np.cos(th_read))

                self.lock_odometry.acquire()
                self.x.value += dx
                self.y.value += dy
                # self.th.value = th_read + w*self.P # Using odometry
                # if w != 0:
                self.prev_th.value =th_read
                self.th.value = new_th
                self.v.value = v
                self.w.value = w
                self.lock_odometry.release()
                

                # save LOG
                with open(self.log_filename, "a") as f:
                    f.write("%.2f,%.2f,%.2f,%.2f,%.2f\n" % (self.x.value, self.y.value, self.th.value, self.v.value, self.w.value))
                    
                # print ("X --> %.2f, Y --> %.2f, th --> %.2f, v --> %.2f, w --> %.2f" % (self.x.value, self.y.value, self.th.value, self.v.value, self.w.value))

                
            except IOError as error:
                sys.stdout.write(error)

            tEnd = time.clock()
            """ para que se ejecute cada P segundos, teniendo en cuenta el tiempo que ha tardado en ejecutarse """
            if tEnd - tIni < self.P:
                time.sleep(self.P - (tEnd-tIni))

        sys.stdout.write("Stopping odometry ... X -->  %.2f, \
                Y -->  %.2f, th -->  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    def stopOdometry(self):
        """Stop the odometry thread
        """
        self.finished.value = True
        self.BP.reset_all()
        
    def angleDistance(self, th2, th1):
        """Calculates the distance between two angles in radians
        :param th2: angle 2
        :param th1: angle 1
        :return: distance between the angles
        """
        return min(((th2 - th1 ) % (2*np.pi)), ((th1 - th2 ) % (2*np.pi)))

    
    def waitAngle(self, ang_final, tolerancia=0.05, initial_w=None, initial_R=0):
        """Waits until the robot reaches a specific angle with a given tolerance
        :param ang_final: desired angle in radians
        :param tolerancia: tolerance in radians 
        """
        error_count = 0
        if self.verbose:
            print("Esperando hasta alcanzar el ángulo: ", ang_final)
        t_siguiente = time.time()
        [_, _, ang_actual] = self.readOdometry()
        distancia_a_final = self.angleDistance(ang_final, ang_actual)

        while distancia_a_final > tolerancia: 
            ultima_distancia = distancia_a_final
            t_siguiente += 0.01
            t_actual = time.time()
            if t_siguiente > t_actual:
                time.sleep(t_siguiente - time.time())
                
            [_, _, ang_actual] = self.readOdometry()
            distancia_a_final = self.angleDistance(ang_final, ang_actual)

            # if self.verbose:
            #     print("Angulo destino -->", ang_final , "; Angulo actual --> ", ang_actual, "; Error --> ", distancia_a_final)
            
            if distancia_a_final > ultima_distancia: 
                error_count += 1
                if error_count > 3:
                    # If we are moving away from the desired angle, we stop
                    if self.verbose:
                        print("Error aumentando", distancia_a_final, "; es mayor que", ultima_distancia)
                    break
            elif distancia_a_final == ultima_distancia:
                continue
            else:
                error_count = 0
            
            if initial_w is not None:
                adjusted_v, adjusted_w = calcAngleSpeed(initial_w, initial_R, distancia_a_final)
                self.setSpeed(adjusted_v, adjusted_w)

        self.setSpeed(0, 0)
        [_, _, ang_actual] = self.readOdometry()
        if self.verbose:
            print("Angulo deseado: ", ang_final, "Angulo alcanzado: ", ang_actual)
            print("Error: ", distancia_a_final)

        return
            
        
    def waitPosition(self, x_deseado, y_deseado, tolerancia=0.03, eje=None):
        """Waits until the robot reaches a specific position with a given tolerance
        :param x_deseado: desired x position
        :param y_deseado: desired y position
        :param tolerancia: tolerance in meters
        """

        # time.sleep(self.P)
        
        t_siguiente = time.time()
        [x_actual, y_actual, th_actual] = self.readOdometry()
        
        if eje is not None:
            diferencia_posicion = abs(x_deseado - x_actual) if eje == "x" else abs(y_deseado - y_actual)
        else:    
            diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)

        if self.verbose:
            print("Posición inicial:", x_actual, y_actual)
            print("Esperando hasta alcanzar:", x_deseado, y_deseado)
            print("Error:", diferencia_posicion)
            print("Angulo actual:", th_actual)
        error_count = 0

        while diferencia_posicion > tolerancia:
            diferencia_posicion_anterior = diferencia_posicion
            t_siguiente += 0.01
            t_actual = time.time()
            if t_siguiente > t_actual:
                time.sleep(t_siguiente - time.time())
            
            [x_actual, y_actual, th_actual] = self.readOdometry()
            diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)
            
            if self.verbose:
                print("Posición actual:", x_actual, y_actual, th_actual, "Error:", diferencia_posicion, "velocidad:", self.v.value, "angular:", self.w.value)
            
            if diferencia_posicion > diferencia_posicion_anterior:
                error_count += 1
                if error_count > 2:
                    # If we are moving away from the desired position, we stop
                    print("Error aumentando", diferencia_posicion, "; es mayor que", diferencia_posicion_anterior)
                    break
            elif diferencia_posicion == diferencia_posicion_anterior:
                continue
            else:
                error_count = 0
                
        if self.verbose:
            print("Posición deseada:", x_deseado, y_deseado, "Posición alcanzada:", x_actual, y_actual)
            print("Error:", diferencia_posicion)
        
        return

    def waitPositionWithWallCorrection(self, x_deseado, y_deseado, v_base=0.1, tolerancia=0.02, marcha_atras=False):
        """Waits until the robot reaches a specific position with a given tolerance and corrects the position
        :param x_deseado: desired x position
        :param y_deseado: desired y position
        :param tolerancia: tolerance in meters
        """
        d_min = 0.20
        
        time.sleep(self.P)
        
        t_siguiente = time.time()
        x_actual, y_actual, _ = self.readOdometry()
        w_max = np.pi
        dD = 0.0
        d = self.get_side_obstacle_distance()
        ## pass from cm to m
        d = d/100.0
        k1=0.5
        k2=-30.0
        
        
        # diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)
        diferencia_posicion = y_deseado - y_actual
        if self.verbose:
            print("Posición inicial:", x_actual, y_actual)
            print("Esperando hasta alcanzar:", x_deseado, y_deseado)
            print("Error:", diferencia_posicion)
        error_count = 0

        while diferencia_posicion > tolerancia:
            diferencia_posicion_anterior = diferencia_posicion
            t_siguiente += self.P
            t_actual = time.time()
            if t_siguiente > t_actual:
                time.sleep(t_siguiente - time.time())
            
            x_actual, y_actual, th_actual = self.readOdometry()
            # diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)
            diferencia_posicion = y_deseado - y_actual
            
            if self.verbose:
                print("Posición actual:", x_actual, y_actual, th_actual, "Error:", diferencia_posicion)
            
            if diferencia_posicion > diferencia_posicion_anterior:
                error_count += 1
                if error_count > 2:
                    # If we are moving away from the desired position, we stop
                    print("Error aumentando", diferencia_posicion, "; es mayor que", diferencia_posicion_anterior)
                    break
            else:
                error_count = 0
                
            w = k1 * (d_min - d) + k2* dD
            if w > 0:
                w_c = min(w, w_max)
            else:
                w_c = max(w, -w_max)
            if self.verbose:
                print("Setting speed: v --> %.2f; w --> %.2f" % (v_base, w_c))
            
            d1 = self.get_side_obstacle_distance()
            if d1 > 80.0:
                break
            d1 = d1/100.0
            dD = d1 - d
            d = d1
            if marcha_atras:
                # If we are moving backwards, we need to change the sign of the angular speed
                w_c = -w_c
            self.setSpeed(v_base, w_c)
            
                
        if self.verbose:
            print("Posición deseada:", x_deseado, y_deseado, "Posición alcanzada:", x_actual, y_actual)
            print("Error:", diferencia_posicion)
        
        return
    
    def catch(self):
        """Lowers the basket to catch the ball
        """
        self.BP.set_motor_position(self.PORT_MOTOR_PINZA, self.PINZA_ABAJO)
        time.sleep(self.motor_wait_time)
        if self.verbose:
            print("La cesta ha bajado")
        return True
    
    def checkBallCaught(self, minObjectiveSize, maxObjectiveSize, maxYValue, colorMasks=None):
        """Checks if the ball has been caught moving backwards 
        :param minObjectiveSize: minimum area of the ball to consider it as caught
        :param maxObjectiveSize: maximum area of the ball to consider it as caught
        :param maxYValue: maximum y value of the ball to consider it as caught
        """
        # Set speed to move backwards     
        self.setSpeed(-0.15, 0)
        bola_cogida = False
        timeout = time.time() + 2
        # loop until the ball is caught or the timeout is reached
        while time.time() < timeout:
            _, y, area = self.detectBall(colorMasks)
            # if the ball is in the objective area and the y position is valid we consider the ball caught
            if minObjectiveSize < area < maxObjectiveSize and y <= maxYValue:
                bola_cogida = True
                break        
        # Stop the robot
        self.setSpeed(0, 0)
        return bola_cogida


    def release(self):
        """Lifts the basket to release the ball
        """
        self.BP.set_motor_position(self.PORT_MOTOR_PINZA, self.PINZA_ARRIBA)
        time.sleep(self.motor_wait_time)
        if self.verbose:
            print("La cesta ha subido de nuevo")

    def init_camera(self): 
        """Initializes the camera
        """
        self.camera = picamera.PiCamera()

        #set camera resolution and framerate
        self.camera.resolution = (self.width, self.height)
        #self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(self.width, self.height))
        
        # Set camera parameters to optimize capture's time
        self.camera.exposure_mode = 'sports'
        self.camera.awb_mode = 'auto'  

        # allow the camera to warmup
        time.sleep(0.1)
            
    def takePhoto(self):
        """Returns the current image from the camera
        """
        self.rawCapture.truncate(0)
        self.rawCapture.seek(0)        
        self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)
        img = self.rawCapture.array
        return img
    
    
    def detectBall(self, colorMasks=None):
        """Takes a photo, tries to detect the ball in it and returns the position and area of the ball (if detected)
        """        
        img_BGR = self.takePhoto()

        # Convert BGR to HSV
        img_HSV = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)

        totalMask = np.zeros((self.height, self.width), np.uint8)
        for colorMask in colorMasks:
            # print("Color mask: ", colorMask)
            mask = cv2.inRange(img_HSV, colorMask[0], colorMask[1])
            totalMask = cv2.bitwise_or(totalMask, mask)


        # Find contours in the mask
        _, contours, _ = cv2.findContours(totalMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx, cy, area = 0, 0, 0 

        # Get the x, y and area of the ball
        if contours:
            # Tomar el contorno más grande (suponiendo que es la pelota)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Calcular el centroide (posición de la pelota)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0  # Evitar división por cero

        if self.verbose:
            print("Objeto detectado en posición: ({}, {}) con área: {}".format(cx, cy, area))

        return cx, cy, area 
    
    
    def angular_velocity(self, theta_new, theta_old, dt):
        delta_theta = math.atan2(math.sin(theta_new - theta_old), math.cos(theta_new - theta_old))
        return delta_theta / dt


    def trackObject(self, v_base=0.4, w_base=np.pi/2, catch=True, targetX=160, minObjectiveTargetSize=4500, maxObjectiveTargetSize=8500, detection_tolerance=30, maxYValue=32, colorMasks=None, starting_w=None):
        """
        Tracks the ball and tries to catch it. The robot will move towards the ball until it is in the objective area
        and then it will try to catch it. If the catch is successful, the robot will stop. If the catch is not
        successful the robot will start looking for the ball again.
        :param v_base: base linear speed
        :param w_base: base angular speed
        :param catch: boolean to indicate if the robot should catch the ball
        :param targetX: x position of the target
        :param minObjectiveTargetSize: minimum area of the ball to consider it as the target
        :param maxObjectiveTargetSize: maximum area of the ball to consider it as the target
        :param detection_tolerance: tolerance in the x position of the ball
        :param maxYValue: maximum y position of the ball
        :param colorMasks: list of color masks to detect the ball
        :param starting_w: initial sign of the angular speed
        """
        self.init_camera()
        finished = False
        ball_caught = False
        ### Tracking parameters
        # positive or negative to know the direction of the error
        last_error = -1 if starting_w is "right" else 1

        self.release() # Check if the basket is up

        while not finished:
            targetPositionReached = False
            # 1. search the most promising blob
            x, y, area = self.detectBall(colorMasks)

            # Rotate until the object is found
            while area < self.minTargetSize and abs(x - targetX) >  detection_tolerance:
                search_v, search_w = calcSearchSpeed(np.sign(last_error) * w_base)
                self.setSpeed(search_v, search_w)
                time.sleep(self.tracking_period)
                x, y, area = self.detectBall(colorMasks)

            # Object found
            if self.verbose:
                print("Objeto encontrado")
            self.setSpeed(0, 0)

            while not targetPositionReached and not ball_caught: 
                # 2. v, w calculation to reach the target ( ball )
                x, y, area = self.detectBall(colorMasks)

                if area < self.minTargetSize:
                    if self.verbose:
                        print("Objeto perdido o muy pequeño")
                    break
                else:
                    last_error = x-targetX

                v, w = calcTrackSpeed(x,area, targetX, minObjectiveTargetSize, maxObjectiveTargetSize, v=v_base, w=w_base)
                self.setSpeed(v, w)

                # 3. check if the ball is in the target position
                if (abs(x - targetX) <  detection_tolerance and (minObjectiveTargetSize < area < maxObjectiveTargetSize) and y <= maxYValue):
                    if self.verbose:
                        print("Posición válida para coger la bola")
                    self.setSpeed(0, 0)

                    if catch:     
                        self.catch()
                        # Check if the ball has been caught
                        if not self.checkBallCaught(minObjectiveTargetSize, maxObjectiveTargetSize, maxYValue, colorMasks):
                            self.release()
                            if self.verbose:
                                print("Bola no cogida")
                        else:
                            ball_caught = True
                            if self.verbose:
                                print("Bola cogida")

                    else: # If we don't want to catch the ball we just stop
                        targetPositionReached = True

            # 4. Check if the ball has been caught or the target position has been reached to finish the tracking
            if targetPositionReached or ball_caught:
                finished = True
                self.setSpeed(0, 0)

        return
    
       
    def definePositionValues(self, x=None, y=None, th=None):
        """ Define la posición del robot """
        self.lock_odometry.acquire()
        if x is not None:
            self.x.value = x
        if y is not None:
            self.y.value = y
        if th is not None: 
            self.th.value = th
            self.th_ini.value = th
        self.lock_odometry.release()


    def get_neighbor_from_angle(self, th):
        """
        Determines which neighbor the robot is looking at based on its angle.
        :param th: Angle in radians (normalized to [-π, π])
        :return: Neighbor number (0, 2, 4, or 6)
        """
        # Normalize the angle to [-π, π]
        th = norm_pi(th)

        # Define angle thresholds for each neighbor
        if np.pi/4 <= th < 3*np.pi/4:  # Close to π/2 radians
            return 0  # Up
        elif -3*np.pi/4 <= th < -np.pi/4:  # Close to -π/2 radians
            return 4  # Down
        elif th < -3*np.pi/4 or th >= 3*np.pi/4:  # Close to π radians
            return 6  # Left
        elif -np.pi/4 <= th < np.pi/4:  # Close to 0 radians 
            return 2  # Right


    def go_to_cell(self, x_obj, y_obj, v_base=0.2, w_base=np.pi/2):
        """ Mueve la entidad al objetivo. Comprueba si hay obstáculo y si lo hay, calibra la odometría y replanifica la ruta
            x_obj, y_obj: coordenadas del objetivo en m
        """        
        x, y, th = self.readOdometry()
        # if th % np.pi > 0.5 or th % np.pi < -0.5:
        #     robot.setSpeed(0, w_base)
        #     robot.waitAngle(
        odo_angle = math.atan2(y_obj - y, x_obj - x)
        # Round the angle to the nearest multiple of π/2
        angle = round(odo_angle / (np.pi / 2)) * (np.pi / 2)
        # angle = (odo_angle + angle) /2

        # Calcular la diferencia angular
        angle_diff = norm_pi(angle - th)
        
        if abs(angle_diff) > 0.2:
            if self.verbose:
                print("Girando a ", math.degrees(angle), "grados")
            # Determinar la dirección del giro
            self.setSpeed(0, w_base if angle_diff > 0 else -w_base)

            self.waitAngle(angle, initial_w=w_base if angle_diff > 0 else -w_base, tolerancia=0.05)
            self.setSpeed(0,0)
        elif abs(angle_diff) > 0.01:
            if self.verbose:
                print("Recalibrando a ", math.degrees(angle), "grados")
            # Determinar la dirección del giro
            self.setSpeed(0, w_base if angle_diff > 0 else -w_base)

            self.waitAngle(angle, initial_w=None, tolerancia=0.04)
            self.setSpeed(0,0)

        self.set_ignore_small_w(True)
        
        x, y, th = self.readOdometry()
        if self.verbose:
            print("GO_T0 ODOMETRY: X --> {:.5f} y --> {:.5f} Ángulo --> {:.5f} rad --> {:.5f}".format(x, y, math.degrees(th), th))
            print("Moviéndose a la posición ", x_obj, ", ", y_obj)
        # return x, y, th, None
            
        
        if(self.detect_obstacle()):
            if self.verbose:
                print("Obstáculo detectado, calibramos odometría y replanificamos ruta...")
            self.calibrateOdometry()
            obstacle_direction = self.get_neighbor_from_angle(th)
            self.set_ignore_small_w(False)
            return x, y, th, obstacle_direction
        
        
        self.setSpeed(v_base, 0)
        # self.waitPositionWithCorrection(x_obj, y_obj, v_base, w_base) #### NO FUNCIONA BIEN, PERO SE PUEDE PROBAR
        # d = self.get_side_obstacle_distance()
        # print("Distancia al obstáculo de lado: ", d)
        # if d < 30:
        #     print("HAY PARED, SEGUIMIENTO PARED")
        #     self.waitPositionWithWallCorrection(x_obj, y_obj, v_base=v_base)
        # else:
        #     self.waitPosition(x_obj, y_obj)
        self.waitPosition(x_obj, y_obj, eje='x' if angle == 0 or angle == np.pi else 'y', tolerancia=0.02)
        self.setSpeed(0,0)
        
        x, y, th = self.readOdometry()

        # Para que calibre después de cada celda en caso de que encuentre un obstáculo
        if(self.detect_obstacle()):
            print("Obstáculo detectado, calibramos odometría")
            if abs(th) > np.pi/4 and abs(th) < 3*np.pi/4:
                # Si el robot está mirando hacia arriba o hacia abajo
                if self.verbose:
                    print("Obstáculo detectado en eje y, calibramos odometría de y hasta ", y_obj)
                self.calibrateOdometry(expected_y=y_obj)
            else: 
                if self.verbose:
                    print("Obstáculo detectado en eje x, calibramos odometría de x hasta ", x_obj)
                self.calibrateOdometry(expected_x=x_obj)
            
        self.set_ignore_small_w(False)

        return x, y, th, None


    def get_light_sensor_value(self):
        """
        Devuelve el valor del sensor de luz. Como referencia,
        cuando el robot está sobre el suelo blanco, el valor debería estar alrededor de 1500
        y para el suelo negro, el valor debería estar alrededor de 2300.
        """
        sensor_error = True
        while sensor_error:
            try:
                value = self.BP.get_sensor(self.light_sensor_port)
                sensor_error = False
            except brickpi3.SensorError as error:
                print("Error getting light_sensor value", error)
                time.sleep(0.2)
        return value
    
    def disable_light_sensor(self):
        self.BP.set_sensor_type(self.light_sensor_port, self.BP.SENSOR_TYPE.NONE)
    
    def is_starting_position_A(self):
        light_sensor_value = self.get_light_sensor_value()
        if self.verbose:
            print("Light sensor value: ", light_sensor_value)
        return light_sensor_value < 1900
    

    def get_obstacle_distance(self):
        """ Devuelve la distancia al obstáculo en cm """
        try:
            distance = self.BP.get_sensor(self.ultrasonic_sensor_port)
        except brickpi3.SensorError as error:
            print(error)
            distance = 1000
        return distance


    def get_side_obstacle_distance(self):
        """ Devuelve la distancia al obstáculo en cm """
        try:
            distance = self.BP.get_sensor(self.SENSOR_LADO_DERECHO)
        except brickpi3.SensorError as error:
            print(error)
            distance = 1000
        return distance

    def detect_obstacle(self):
        """ Simulación de detección de obstáculos con ultrasonido """
        obstacle_threshold = 30  # Umbral de distancia para considerar un obstáculo (en cm)
        # Es un poco mayor que lo que debe llegar porque se ejecuta cuando acaba de rotar antes de partir a una nueva posición
        max_checks = 3
        # Obtener lectura del sensor de ultrasonido
        obstacle_distance = self.get_obstacle_distance()
        if self.verbose:
            print("Distancia al obstáculo: ", obstacle_distance)
        # Verificar si la distancia al obstáculo está por debajo del umbral
        while obstacle_distance < obstacle_threshold and max_checks > 0:
            max_checks -= 1
            time.sleep(self.P)
            obstacle_distance = self.get_obstacle_distance()

        if max_checks == 0:  # if the obstacle was detected many times
            print("Obstáculo detectado a una distancia de", obstacle_distance, "cm")
            return True
        
    def detect_side_obstacle(self):
        """ Simulación de detección de obstáculos con ultrasonido """
        obstacle_threshold = 10
        max_checks = 2
        # Obtener lectura del sensor de ultrasonido
        obstacle_distance = self.get_side_obstacle_distance()
        if self.verbose:
            print("Distancia al obstáculo derecho: ", obstacle_distance)
        while obstacle_distance < obstacle_threshold and max_checks > 0:
            max_checks -= 1
            time.sleep(self.P)
            obstacle_distance = self.get_side_obstacle_distance()
        
        if max_checks == 0:
            print("Obstáculo detectado a una distancia de", obstacle_distance, "cm")
            return True 
        
        
    def go_to_free(self, x_obj, y_obj, th_obj, v_base=0.1, w_base=np.pi/4):
        """ Mueve la entidad al objetivo. No comprueba si hay obstáculos
            :param x_obj: coordenadas del objetivo en m
            :param y_obj: coordenadas del objetivo en m
            :param th_obj: ángulo del objetivo en rad
            :param v_base: velocidad lineal base
            :param w_base: velocidad angular base
        """
        print("GO_TO_FREE")
        w_max = 1.5
        v_max = 0.4
        
        rho_max = 8.0
        alpha_max = np.pi
        beta_max = np.pi
        
        kp, ka, kb = v_max/rho_max, w_max/alpha_max, w_max/beta_max
        kp = 0.25
        ka = 1.5
        kb = .8
        K = [[kp,0,0],[0,ka,kb]]

        wXr = self.readOdometry()
        
        while True:
            wXg = [x_obj, y_obj, th_obj]
            gXr = loc(np.dot(inv(hom(wXg)),hom(wXr)))

            gPr = polares(gXr[0], gXr[1], gXr[2])

            [v,w]= np.dot(K, gPr)

            print("v: ", v, "w: ", w)
            self.setSpeed(v,w)
            time.sleep(0.05)

            if gPr[0] < 0.1 and np.abs(gPr[2]) < np.deg2rad(30):
                return True

            wXr = self.readOdometry()

    def doS(self, currentMap, w_base=-np.pi/4, radioD=0.2):
        assert currentMap == 'A' or currentMap == 'B', "Mapa no válido"

        radioD = 0.4
        angleTolerance = 0.09

        if currentMap == 'A':        # 1. Giro 90 grados 
            self.setSpeed(0, -w_base)
            self.waitAngle(-np.pi, tolerancia=angleTolerance, initial_w=-w_base)            
            self.setSpeed(0, 0)

            # 2. Semicírculo radio d izquierda
            self.setSpeed(w_base * radioD, w_base)
            self.waitAngle(0, tolerancia=angleTolerance, initial_w=w_base, initial_R=radioD)
            self.setSpeed(0, 0)
            
            
            # 3.1. Círculo radio d derecha
            self.setSpeed(w_base * radioD, -w_base)
            self.waitAngle(-np.pi, tolerancia=angleTolerance, initial_w=-w_base, initial_R=radioD)
            self.setSpeed(0, 0)

        elif currentMap == 'B':
            self.setSpeed(0, w_base)
            self.waitAngle(0, tolerancia=angleTolerance, initial_w=w_base)            
            self.setSpeed(0, 0)

            # 2. Semicírculo radio d derecha
            self.setSpeed(w_base * radioD, -w_base)
            self.waitAngle(-np.pi, tolerancia=angleTolerance, initial_w=-w_base, initial_R=radioD)
            self.setSpeed(0, 0)

            # 3.1. Círculo radio d izquierda
            self.setSpeed(w_base * radioD, w_base)
            self.waitAngle(0, tolerancia=angleTolerance, initial_w=w_base, initial_R=radioD)
            self.setSpeed(0, 0)
    

    def calibrateOdometry(self, number_of_cells=0, cell_size=40, expected_x=None, expected_y=None, distObj=None):
        """ Calibra la odometría del robot """
        # Comprueba con la distancia al obstáculo (estando en la misma celda)
        # De momento sólo se llama cuando detecta un obstáculo aparte 
        if distObj is None: 
            distObj = 13 + (number_of_cells * cell_size)
            
        if(self.verbose):
            print("Calibrando odometría")
            print("Distancia al objeto deseada: ", distObj)
            
        distanceError = 1
        distance = self.get_obstacle_distance()
        
        while (abs(distObj - distance)) > distanceError:
            if self.verbose:
                print("Distancia al objeto: ", distance, "cm", 
                      " Diferencia de ", distObj - distance, "cm")

            if (distObj - distance) > 0:
                self.setSpeed(-0.1,0)
            else:
                self.setSpeed(0.1,0)
            distance =self.get_obstacle_distance()
            
        if self.verbose:
            print("Distancia final al objeto: ", distance)
            print("Nueva posición: ", expected_x, expected_y)
        
        self.setSpeed(0,0)  
        
        self.definePositionValues(expected_x, expected_y) 
        

    def __del__(self):
        # self.finished.value = True
        # if self.verbose:
        #     print("Robot object deleted")
        self.setSpeed(0,0)
        self.BP.reset_all()

