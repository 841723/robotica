#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import \
    print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division

import io  # ''
import sys
import time  # import the time library for the sleep function
from multiprocessing import Lock, Process, Value

import brickpi3  # import the BrickPi3 drivers
import cv2
import numpy as np
import picamera
from get_color_blobs_diego import detect_ball
from lib.utils import deg_to_rad, norm_pi, rad_to_deg
from lib.utilsrobot import calcSearchSpeed, calcTrackSpeed
from picamera.array import PiRGBArray


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0], log_filename=None, verbose=False):
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

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()
        self.PORT_MOTOR_PINZA = self.BP.PORT_A
        self.PORT_MOTOR_DERECHA = self.BP.PORT_B
        self.PORT_MOTOR_IZQUIERDA = self.BP.PORT_C
        self.PORT_GYRO = self.BP.PORT_3

        # set up gyro
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
        

        # configure camera
        self.camera = None
        
        ## Verbose mode
        self.verbose = verbose

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.v = Value('d',0.0)
        self.w = Value('d',0.0)
        self.anguloDerecha = Value('d',0.0)
        self.anguloIzquierda = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished
        self.rawCapture = None

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
        
        ## Calculo de las velocidades angulares de cada rueda
        w_d = v/self.R + w*self.L/(2*self.R)
        w_i = v/self.R - w*self.L/(2*self.R)
        
        if self.verbose:
            print("v --> %.2f; w --> %.2f; w_d --> %.2f; w_i--> %.2f" % (v, w, w_d, w_i))

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

    def startOdometry(self):
        """This starts a new process/thread that will be updating the odometry periodically 
        """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)
        time.sleep(2)   

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
                
                wIzquierda = float(deg_to_rad(encoderIzquierda - self.anguloIzquierda.value) / self.P)
                wDerecha = float(deg_to_rad(encoderDerecha - self.anguloDerecha.value) / self.P)

                self.anguloDerecha.value = encoderDerecha
                self.anguloIzquierda.value = encoderIzquierda

                v = self.R/2 * (wDerecha + wIzquierda)
                w = self.R/self.L * (wDerecha - wIzquierda)

                x_read, y_read, th_read = self.readOdometry()
                dx, dy, new_th = 0, 0, 0
                try:
                    new_th = self.BP.get_sensor(self.PORT_GYRO)
                    # print("Gyro: ", new_th)
                    new_th = norm_pi(deg_to_rad(-new_th[0]))
                    print("Gyro rads: ", new_th)
                except brickpi3.SensorError as error:
                    if self.verbose:
                        print("Error reading gyro sensor: ", error)
                    new_th = th_read + w*self.P
                

                if w == 0:
                    dx = self.P*v*np.cos(th_read)
                    dy = self.P*v*np.sin(th_read)
                else:
                    dx = v/w*(np.sin(th_read + w*self.P) - np.sin(th_read))
                    dy = -v/w*(np.cos(th_read + w*self.P) - np.cos(th_read))
                
                self.lock_odometry.acquire()
                self.x.value = x_read + dx
                self.y.value = y_read + dy
                self.th.value = th_read + w*self.P
                self.v.value = v
                self.w.value = w
                self.lock_odometry.release()

                # save LOG
                with open(self.log_filename, "a") as f:
                    f.write("%.2f,%.2f,%.2f,%.2f,%.2f\n" % (self.x.value, self.y.value, self.th.value, self.v.value, self.w.value))

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

    
    def waitAngle(self, ang_final, tolerancia=0.02):
        """Waits until the robot reaches a specific angle with a given tolerance
        :param ang_final: desired angle in radians
        :param tolerancia: tolerance in radians 
        """
        error_count = 0
        time.sleep(self.P)
        if self.verbose:
            print("Esperando hasta alcanzar el ángulo: ", ang_final)
        t_siguiente = time.time()
        [_, _, ang_actual] = self.readOdometry()
        distancia_a_final = self.angleDistance(ang_final, ang_actual)

        while distancia_a_final > tolerancia: 
            ultima_distancia = distancia_a_final
            t_siguiente += self.P
            t_actual = time.time()
            if t_siguiente > t_actual:
                time.sleep(t_siguiente - time.time())
                
            [_, _, ang_actual] = self.readOdometry()
            distancia_a_final = self.angleDistance(ang_final, ang_actual)

            if self.verbose:
                print("Angulo destino -->", ang_final , "; Angulo actual --> ", ang_actual, "; Error --> ", distancia_a_final)
            
            if distancia_a_final > ultima_distancia :
                error_count += 1
                if error_count > 3:
                    # If we are moving away from the desired angle, we stop
                    if self.verbose:
                        print("Error aumentando", distancia_a_final, "; es mayor que", ultima_distancia)
                    break
            else:
                error_count = 0
        if self.verbose:
            print("Angulo deseado: ", ang_final, "Angulo alcanzado: ", ang_actual)
            print("Error: ", distancia_a_final)
            
        
    def waitPosition(self, x_deseado, y_deseado, tolerancia=0.03):
        """Waits until the robot reaches a specific position with a given tolerance
        :param x_deseado: desired x position
        :param y_deseado: desired y position
        :param tolerancia: tolerance in meters
        """

        time.sleep(self.P)
        
        t_siguiente = time.time()
        [x_actual, y_actual, _] = self.readOdometry()
        
        diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)
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
            
            [x_actual, y_actual, _] = self.readOdometry()
            diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)
            
            if self.verbose:
                print("Posición actual:", x_actual, y_actual, "Error:", diferencia_posicion)
            
            if diferencia_posicion > diferencia_posicion_anterior:
                error_count += 1
                if error_count > 3:
                    # If we are moving away from the desired position, we stop
                    print("Error aumentando", diferencia_posicion, "; es mayor que", diferencia_posicion_anterior)
                    break
                
        if self.verbose:
            print("Posición deseada:", x_deseado, y_deseado, "Posición alcanzada:", x_actual, y_actual)
            print("Error:", diferencia_posicion)

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


    def trackObject(self, v_base=0.4, w_base=np.pi/2, catch=True, targetX=160, minObjectiveTargetSize=4500, maxObjectiveTargetSize=8500, detection_tolerance=30, maxYValue=32, colorMasks=None):
        """
        Tracks the ball and tries to catch it. The robot will move towards the ball until it is in the objective area
        and then it will try to catch it. If the catch is successful, the robot will stop. If the catch is not
        successful the robot will start looking for the ball again.
        :param v_base: base linear speed
        :param w_base: base angular speed
        :param catch: boolean to indicate if the robot should catch the ball
        """
        self.init_camera()
        finished = False
        ball_caught = False
        ### Tracking parameters
        last_error = 1 # positive or negative to know the direction of the error


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

    def __del__(self):
        # self.finished.value = True
        if self.verbose:
            print("Robot object deleted")
        self.setSpeed(0,0)
        self.BP.reset_all()

