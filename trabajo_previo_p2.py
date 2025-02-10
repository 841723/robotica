def setSpeed(self, v, w):
    w_d = v/self.r + w*self.L/(2*self.r)
    w_i = v/self.r - w*self.L/(2*self.r)
    
    speedDPS_left = degrees(wr[1])
    speedDPS_right = degrees(wr[0])
    wr = [speedDPS_right, speedDPS_left]
    return wr

def updateOdometry():
    vc = readSpeed(self)
    
    # leftEncoder = self.BP.get_motor_encoder
    # rightEncoder = self.BP.get_motor_encoder
    
    
    
Descripción de trayectoria 1                 v                  w                    Tiempo de ejecución
1. Giro 90 grados                   0                       -w_base                    pi/(2*w_base)
2. Semicírculo radio d izquierda    v = w * radioD          w_base                     pi*radioD*v/w_base 
3. Círculo radio d derecha          v = -w * radioD         -w_base                 2*pi*radioD*v/w_base
4. Semicírculo radio d izquierda    v = w * radioD          w_base                 pi*radioD*v/w_base

Descripción de trayectoria 2                      v                  w                          Tiempo de ejecución
1. Giro 90 grados                                   0                w_base                     pi/(2*w_base)            
2. Cuarto de círculo radio alfa (derecha)   v_alfa = -w * alfa      -w_base                     pi*v_alfa/(4*w_base)            
3. Línea recta longitud R                      v_base                0                           R/v_base  
4. Semicírculo radio d (derecha)            v_d  = -w * d         -w_base                       pi*v_d/(2*w_base)           
5. Línea recta longitud R                       v_base                    0                      R/v_base       
6. Cuarto de círculo radio alfa (derecha)  v_alfa = -w * alfa          -w_base                    pi*v_alfa/(4*w_base)            