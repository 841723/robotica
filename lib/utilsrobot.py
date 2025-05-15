import cv2
import numpy as np


def calcTrackSpeed(x, area, targetX, minObjectiveSize, maxObjectiveSize, v, w):
    """ Speed calculation for tracking a target.
    :param x: x coordinate of the target
    :param area: area of the target
    :param targetX: x coordinate of the target
    :param minObjectiveSize: minimum size of the target
    :param maxObjectiveSize: maximum size of the target
    :param v: linear speed
    :param w: angular speed
    :return: linear and angular speed
    """
    # Calcular error de posición
    error_x = targetX - x
    error_size = 0
    # Calcular error de tamaño
    if area < minObjectiveSize:
        error_size = np.sqrt(minObjectiveSize) - np.sqrt(area)
    elif area > maxObjectiveSize:
        error_size = np.sqrt(maxObjectiveSize) - np.sqrt(area)

    
    # Constantes para ajustar sensibilidad (cómo de lineal es la sigmoidal)
    kp_angle = 0.05   # Factor proporcional para posición
    kp_size = 0.1      # Factor proporcional para tamaño
    
    # Función sigmoid para suavizar la respuesta 
    # k regula cómo de lineal es la sigmoid
    def sigmoid(x, k=1):
        return 1 / (1 + np.exp(-k * x))
    
    # Ajuste de velocidad lineal (v) basado en error de tamaño
    # Si la bola es más pequeña que el objetivo -> avanzar
    # Si la bola es más grande que el objetivo -> retroceder
    v_adjusted =  (v * sigmoid(error_size, kp_size))
    
    # Ajuste de velocidad angular (w) basado en error de posición
    # Si la bola está a la derecha del objetivo -> girar izquierda
    # Si la bola está a la izquierda del objetivo -> girar derecha
    w_adjusted = - (w * sigmoid(error_x, kp_angle) - (w/2))
    
    if (abs(w_adjusted) < 0.33):
        w_adjusted = 0
    
    # print("sigmoid", sigmoid(error_size, kp_size))
    # print("error_x: ", error_x)
    # print("v_adjusted: ", v_adjusted)
    # print("w_adjusted: ", w_adjusted)
    
    return v_adjusted, w_adjusted
    
def calcAngleSpeed(w, r, angle_distance, kp_angle=5):
    
    # Constantes para ajustar sensibilidad (cómo de lineal es la sigmoidal)

    # Función sigmoid para suavizar la respuesta 
    # k regula cómo de lineal es la sigmoid
    def sigmoid(x, k=1):
        return 1 / (1 + np.exp(-k * x))
    

    # Ajuste de velocidad angular (w) basado en error de ángulo
    w_adjusted = (2* w * sigmoid(angle_distance, kp_angle) - (w))
    
    
    v_adjusted =  abs(r * w_adjusted)
    
    
    # if (abs(w_adjusted) < 0.33):
    #     w_adjusted = 0
    
    # print("w_initial ", w, "w_adjusted: ", w_adjusted, "angle_distance: ", 
        #   angle_distance, "sigmoid", sigmoid(angle_distance, kp_angle))
    
    return v_adjusted, w_adjusted


# Devuelve la velocidad (v,w) necesaria para dar vueltas sobre sí mismo 
# hasta que encuentra una bola 
def calcSearchSpeed(w):
    """ Speed calculation for searching a target.
    :param w: angular speed
    :return: linear and angular speed
    """
    # hace elipses por si la bola está lejos
    v = 0
    return v, w
