def setSpeed(self, v, w):
    w_d = v/self.r + w*self.L/(2*self.r)
    w_i = v/self.r - w*self.L/(2*self.r)
    
    speedDPS_left = degrees(w_i)
    speedDPS_right = degrees(w_d)
    wr = [speedDPS_right, speedDPS_left]
    return wr

def updateOdometry():
    vc = readSpeed(self)
    
    # leftEncoder = self.BP.get_motor_encoder
    # rightEncoder = self.BP.get_motor_encoder
    def updateOdometry(self):
        prev_encoder_left = self.getLeftEncoder()
        prev_encoder_right = self.getRightEncoder()
    
        while not fin:
            t_start = time.time()
            
            curr_encoder_left = self.getLeftEncoder()
            curr_encoder_right = self.getRightEncoder()
            
            speed_left = np.radians((curr_encoder_left - prev_encoder_left) / self.P)
            speed_right = np.radians((curr_encoder_right - prev_encoder_right) / self.P)
            
            # Actualización de valores previos
            prev_encoder_left = curr_encoder_left
            prev_encoder_right = curr_encoder_right
            
            # Matriz de transformación para obtener v y w
            wheel_speeds = np.array([speed_right, speed_left])
            transform_matrix = self.wheel_radius * np.array([
                [0.5, 0.5],
                [1/self.inter_wheel_distance, -1/self.inter_wheel_distance]
            ])
            
            # Cálculo de velocidades del robot
            robot_speeds = np.dot(transform_matrix, wheel_speeds)
            v = robot_speeds[0]
            w = robot_speeds[1]
            
            
            # Actualización de la posición
            with self.lock_odometry:
                self.v.value = v
                self.w.value = w
                if w == 0:  # Movimiento recto
                    self.x.value += v * self.P * np.cos(self.th.value)
                    self.y.value += v * self.P * np.sin(self.th.value)
                else:  # Movimiento curvo
                    self.x.value += (v/w) * (np.sin(self.th.value + w*self.P) - 
                                           np.sin(self.th.value))
                    self.y.value += (v/w) * (-np.cos(self.th.value + w*self.P) + 
                                           np.cos(self.th.value))
                
                # Actualización del ángulo
                if self.use_mean_odometry.value:
                    # Promedio entre odometría y giroscopio
                    odo_th = self.th.value + self.w_correction_factor.value * w * self.P
                    gyro_th = self.getGyroSensorValue()
                    
                    # Promedio vectorial
                    x_mean = (np.cos(odo_th) + np.cos(gyro_th)) / 2
                    y_mean = (np.sin(odo_th) + np.sin(gyro_th)) / 2
                    self.th.value = np.arctan2(y_mean, x_mean)
                else:
                    # Actualización directa
                    self.th.value += self.w_correction_factor.value * w * self.P
            
            # Control de tiempo del ciclo
            t_end = time.time()
            sleep_time = self.P - (t_end - t_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    
    
    
    
"""    
Descripción de trayectoria 1                       v                    w                    Tiempo de ejecución
1. Giro 90 grados                            0                       -w_base            pi/(2*w_base)
2. Semicírculo radio d izquierda             v = w * radioD          w_base             pi*radioD*v/w_base 
3. Círculo radio d derecha                   v = -w * radioD         -w_base            2*pi*radioD*v/w_base
4. Semicírculo radio d izquierda             v = w * radioD          w_base             pi*radioD*v/w_base

Descripción de trayectoria 2                       v                    w                   Tiempo de ejecución
1. Giro 90 grados                            0                       w_base             pi/(2*w_base)            
2. Cuarto de círculo radio alfa (derecha)    v_alfa = -w * alfa      -w_base            pi*v_alfa/(4*w_base)            
3. Línea recta longitud R                    v_base                  0                  R/v_base  
4. Semicírculo radio d (derecha)             v_d  = -w * d           -w_base            pi*v_d/(2*w_base)           
5. Línea recta longitud R                    v_base                  0                  R/v_base       
6. Cuarto de círculo radio alfa (derecha)    v_alfa = -w * alfa      -w_base            pi*v_alfa/(4*w_base)            
"""