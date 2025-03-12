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
    def __init__(self, init_position=[0.0, 0.0, 0.0], log_filename=None):
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

        # Configure sensors

        # move motor A to the starting position
        self.BP.set_motor_limits(self.PORT_MOTOR_PINZA, dps=100)
        self.BP.set_motor_position(self.PORT_MOTOR_PINZA, self.PINZA_ARRIBA)
        self.motor_wait_time = 2 


        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.PORT_MOTOR_DERECHA,
           self.BP.get_motor_encoder(self.PORT_MOTOR_DERECHA))
        self.BP.offset_motor_encoder(self.PORT_MOTOR_IZQUIERDA,
           self.BP.get_motor_encoder(self.PORT_MOTOR_IZQUIERDA))
        # self.BP.offset_motor_encoder(self.PORT_MOTOR_PINZA,
        #       self.BP.get_motor_encoder(self.PORT_MOTOR_PINZA))
        

        # configure camera
        self.camera = None

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

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        
        # odometry update period
        self.P = 0.05
        
        # Tracking update period
        self.tracking_period = 0.01
        
        self.stream = None
        
        ##################################################
        #### VARIABLES PARA EL SEGUIMIENTO DE OBJETOS ####
        ##################################################
        
        self.minTargetSize = 30
        self.width = 320
        self.height = 240
        # self

        # save into log file
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
        
        
    def setSpeed(self, v,w):
        """ Establece la velocidad de los motores según la velocidad lineal y angular dadas """
        
        ## Calculo de las velocidades angulares de cada rueda
        w_d = v/self.R + w*self.L/(2*self.R)
        w_i = v/self.R - w*self.L/(2*self.R)

        ## Añadir las velocidades a los motores
        self.BP.set_motor_dps(self.PORT_MOTOR_IZQUIERDA, rad_to_deg(w_i))
        self.BP.set_motor_dps(self.PORT_MOTOR_DERECHA, rad_to_deg(w_d))
        
        ## Actualizar los valores
        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.lock_odometry.release()


    def readSpeed(self):
        """ Obtención de la velocidad lineal y angular """
        self.lock_odometry.acquire()
        v = self.v.value
        w = self.w.value
        self.lock_odometry.release()
        return v, w

    def readOdometry(self):
        """ Devuelve la posición y orientación del robot en un plano 2D """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = norm_pi(self.th.value)        
        self.lock_odometry.release()
        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): 
        """ This function will be run periodically to update the odometry values """

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

                # save LOG
                with open(self.log_filename, "a") as f:
                    f.write("%.2f,%.2f,%.2f,%.2f,%.2f\n" % (self.x.value, self.y.value, self.th.value, self.v.value, self.w.value))

            except IOError as error:
                sys.stdout.write(error)

            tEnd = time.clock()
            """ para que se ejecute cada P segundos, teniendo en cuenta el tiempo que ha tardado en ejecutarse """
            if tEnd - tIni < self.P:
                time.sleep(self.P - (tEnd-tIni))

        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.BP.reset_all()
        
    def angleDistance(self, th2, th1):
        """
        Devuelve la distancia entre dos ángulos en radianes. Los parametros son los ángulos en radianes de los que se quiere calcular la distancia.
        """
        return min(((th2 - th1 ) % (2*np.pi)), ((th1 - th2 ) % (2*np.pi)))

    
    def waitAngle(self, ang_final, tolerancia=0.000005):
        """
        Espera hasta que el robot alcance un ángulo específico.
        Se recibe un ángulo en radianes y se espera hasta que el robot alcance ese ángulo.
        Estimación según la tolerancia al fallo entre el angulo objetivo y el calculado.
        """
        error_count = 0
        time.sleep(self.P)
        print(ang_final)
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

            print("Ang_actual: ", ang_actual,
                  "Ang_final: ", ang_final,
                  "Error: ", distancia_a_final)
            
            if distancia_a_final > ultima_distancia :
                error_count += 1
                if error_count > 3:
                    # si nos alejamos del ángulo deseado, paramos
                    print("Error aumentando", distancia_a_final, "; es mayor que", ultima_distancia)
                    break
            else:
                error_count = 0
                
        print("Angulo deseado: ", ang_final, "Angulo alcanzado: ", ang_actual)
        print("Error: ", distancia_a_final)
            
        
    def waitPosition(self, x_deseado, y_deseado, tolerancia=0.03):
        """
        Espera hasta que el robot alcance una posición específica.
        :param x_deseado: Posición x deseada
        :param y_deseado: Posición y deseada
        :param tolerancia: Tolerancia de posición
        """
        time.sleep(self.P)
        
        t_siguiente = time.time()
        [x_actual, y_actual, _] = self.readOdometry()
        
        diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)

        print("Posición inicial:", x_actual, y_actual)
        print("Esperando hasta alcanzar:", x_deseado, y_deseado)
        error_count = 0

        while diferencia_posicion > tolerancia:
            diferencia_posicion_anterior = diferencia_posicion
            t_siguiente += self.P
            t_actual = time.time()
            if t_siguiente > t_actual:
                time.sleep(t_siguiente - time.time())
            
            [x_actual, y_actual, _] = self.readOdometry()
            diferencia_posicion = np.sqrt((x_deseado - x_actual)**2 + (y_deseado - y_actual)**2)
            
            
            print("Posición actual:", x_actual, y_actual, "Error:", diferencia_posicion)
            
            if diferencia_posicion > diferencia_posicion_anterior:
                error_count += 1
                if error_count > 3:
                    # si nos alejamos de la posición deseada, paramos
                    print("Error aumentando", diferencia_posicion, "; es mayor que", diferencia_posicion_anterior)
                    break
        print("Posición deseada:", x_deseado, y_deseado, "Posición alcanzada:", x_actual, y_actual)
        print("Error:", diferencia_posicion)

    # Baja la cesta
    def catch(self):
        self.BP.set_motor_position(self.PORT_MOTOR_PINZA, self.PINZA_ABAJO)
        time.sleep(self.motor_wait_time)
        return True
    
    # Va hacia atrás y comprueba si hay algo del color de la bola en el rango
    # TODO: PROBLEMA SI CHOCARA HACIA ATRÁS, se puede plantear sensor de distancia hacia atrás
    def checkBallCaught(self, minObjectiveSize, maxObjectiveSize):        # en funcion a posicion de camara
        # TODO: comprobar que no hay nada detras
        self.setSpeed(-0.1, 0)
        bola_cogida = False
        timeout = time.time() + 4
        while time.time() < timeout:
            _, y, area = self.detectBall()
            if minObjectiveSize < area < maxObjectiveSize and y <= self.height/4:
                bola_cogida = True
                print("Bola cogida correctamente")
                break
        
        self.setSpeed(0, 0)

        return bola_cogida

    
    # Sube la cesta
    def release(self):
        self.BP.set_motor_position(self.PORT_MOTOR_PINZA, self.PINZA_ARRIBA)
        time.sleep(self.motor_wait_time)
        return True

    def init_camera(self): # TODO: check
        self.camera = picamera.PiCamera()

        self.camera.resolution = (self.width, self.height)
        #self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(self.width, self.height))
        
        self.camera.exposure_mode = 'sports'
        self.camera.awb_mode = 'auto'  

        # self.stream = io.BytesIO()
        # allow the camera to warmup
        time.sleep(0.1)
            
    # Devuelve la imagen capturada por la cámara
    def takePhoto(self):
        self.rawCapture.truncate(0)
        self.rawCapture.seek(0)        
        self.camera.capture(self.rawCapture, format="bgr", use_video_port=True)
        img = self.rawCapture.array
        return img
    
        # with picamera.PiCamera() as camera:
        # stream = io.BytesIO()
        # for foo in camera.capture_continuous(stream, format='jpeg'):
        #     # Truncate the stream to the current position (in case
        #     # prior iterations output a longer image)
        #     stream.truncate()
        #     stream.seek(0)
        
        
        # stream = io.BytesIO()
        # self.camera.capture(stream, format='jpeg')
        # self.camera.stop_preview()
        # self.camera.close()
        # stream.seek(0)
        # image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        # img_BGR = cv2.imdecode(image, cv2.IMREAD_COLOR)
        # return img_BGR
    
    # Busca la bola en la imagen y devuelve su posición y área
    def detectBall(self):
        
        # mask=cv2.inRange(img, colorRangeMin, colorRangeMax)
        # # apply the mask
        # img = cv2.bitwise_and(img, img, mask = mask) # TODO: añadir erosión o difuminado? 
        # keypoints = detector.detect(255-mask)
        
        # # Take biggest blob
        # x, y, area = 0, 0, 0
        # for kp in keypoints:
        #     if kp.size > area:
        #         x = kp.x
        #         y = kp.y
        #         area = kp.size

        # # posicion a la esquina superior izquierda

        # detection_tolerance = 0.1
        # targetX = 0.5
        # objectiveTargetSize = 0.1

        img = self.takePhoto()
        x, y, area = detect_ball(img) # TODO: poner en la misma función
        print(x, y, area)
        return x, y, area 


    def trackObject(self, v_base=0.2, w_base=np.pi/3, catch=True):
        self.init_camera()

        finished = False
        ball_caught = False
        targetX = self.width/2-40
        # targetY = 240/4
        minObjectiveTargetSize = 6600
        maxObjectiveTargetSize = 7000
        detection_tolerance = 7
        last_error = 1 # positivo o negativo para determinar la dirección de giro

        self.release() # Comprobamos que la cesta está arriba

        # clear log file
        self.log_file = open(self.log_filename, "w")
        self.log_file.close() 

        while not finished:
            
            targetPositionReached = False
            # 1. search the most promising blob
            x, y, area = self.detectBall()         
            print(x, y, area)
            
            
            # Dar vueltas hasta encontrar un objeto
            while area < self.minTargetSize and abs(x - targetX) >  detection_tolerance: 
                # print("Objeto no encontrado")
                search_v, search_w = calcSearchSpeed(np.sign(last_error) * w_base)             
                self.setSpeed(search_v, search_w)
                time.sleep(self.tracking_period)
                x, y, area = self.detectBall()


            # Objeto encontrado, paramos y buscamos
            print("Objeto encontrado")
            self.setSpeed(0, 0)
            
            while not targetPositionReached and not ball_caught: 
                # 2. v y w para acercarse a la bola
                x, y, area = self.detectBall()  
                print("x: ", x, "y: ", y, "area: ", area)
            
                if area < self.minTargetSize or area > self.maxTargetSize:
                    print("Objeto perdido")
                    break
                else:
                    last_error = x-targetX
                
                v, w = calcTrackSpeed(x,area, targetX, minObjectiveTargetSize, maxObjectiveTargetSize, v=v_base, w=w_base)
                self.setSpeed(v, w)
                
                # 3. check la posición válida para coger la bola
                if (abs(x - targetX) <  detection_tolerance and (minObjectiveTargetSize < area < maxObjectiveTargetSize) and y <= self.height/3):
                    # and abs(y - targetY) < detection_tolerance
                    
                    print("Posición válida para coger la bola")
                    self.setSpeed(0, 0)
                    
                    if catch:     
                        self.catch()
                        
                        if not self.checkBallCaught(minObjectiveTargetSize, maxObjectiveTargetSize):
                            print("Fallo al coger la bola")
                            self.release()
                        else:
                            print("Bola cogida")
                            ball_caught = True
                            
                    else: # Si no hace falta coger la bola
                        targetPositionReached = True
                        
                # Si no estamos en el target o se ha escapado la bola continuamos           
                # time.sleep(self.tracking_period)

            if targetPositionReached or ball_caught:
                finished = True
                self.setSpeed(0, 0)
                    
        return finished
    

    def __del__(self):
        # self.finished.value = True
        self.setSpeed(0,0)
        self.BP.reset_all()

