#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys

from lib.utils import rad_to_deg, deg_to_rad
from lib.simubot import simubot

import numpy as np

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.R = 24/1000
        self.L = 115/1000
        #self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()
        self.PORT_MOTOR_PINZA = self.BP.PORT_A
        self.PORT_MOTOR_DERECHA = self.BP.PORT_B
        self.PORT_MOTOR_IZQUIERDA = self.BP.PORT_C

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        # self.BP.offset_motor_encoder(self.PORT_MOTOR_PINZA,
        #    self.BP.get_motor_encoder(self.PORT_MOTOR_PINZA))
        self.BP.offset_motor_encoder(self.PORT_MOTOR_DERECHA,
           self.BP.get_motor_encoder(self.PORT_MOTOR_DERECHA))
        self.BP.offset_motor_encoder(self.PORT_MOTOR_IZQUIERDA,
           self.BP.get_motor_encoder(self.PORT_MOTOR_IZQUIERDA))


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

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period --> UPDATE value!
        self.P = 0.1



    def setSpeed(self, v,w):
        """ To be filled - These is all dummy sample code """
        """ PAGINA 8 de transparencias """
        print("setting speed to %.2f %.2f" % (v, w))

        w_d = v/self.R + w*self.L/(2*self.R)
        w_i = v/self.R - w*self.L/(2*self.R)
        
        speedDPS_left = rad_to_deg(w_i)
        speedDPS_right = rad_to_deg(w_d)

        print("grados setting speed to %.2f %.2f" % (speedDPS_left, speedDPS_right))
        print("rad setting speed to %.2f %.2f" % (w_i, w_d))

        # compute the speed that should be set in each motor ...

        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        # speedDPS_left = 180
        # speedDPS_right = 180
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)


    def readSpeed(self):
        """ To be filled"""
        """ GUARDAR EL VALOR DE v Y w de la misma forma que se hace con la odometria(x,y,theta) """
        """ devolver esos valores de v y w """

        self.lock_odometry.acquire()
        v = self.v.value
        w = self.w.value
        self.lock_odometry.release()
        return v, w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            sys.stdout.write("Dummy update of odometry ...., X=  %d, \
                Y=  %d, th=  %d \n" %(self.x.value, self.y.value, self.th.value) )
            #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                sys.stdout.write("Reading encoder values .... \n")
                [encoderIzquierda, encoderDerecha] = [self.BP.get_motor_encoder(self.PORT_MOTOR_IZQUIERDA),
                   self.BP.get_motor_encoder(self.PORT_MOTOR_DERECHA)]
                
                self.anguloDerecha.value = encoderDerecha
                self.anguloIzquierda.value = encoderIzquierda
                
                wIzquierda = deg_to_rad(encoderIzquierda - self.anguloIzquierda.value) / self.P
                wDerecha = deg_to_rad(encoderDerecha - self.anguloDerecha.value) / self.P

                v = self.R/2 * (wDerecha + wIzquierda)
                w = self.R/self.L * (wDerecha - wIzquierda)

                x_read, y_read, th_read = self.readOdometry()
                dx, dy = 0, 0

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
                


            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            """ para que se ejecute cada P segundos, teniendo en cuenta el tiempo que ha tardado en ejecutarse """
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()

