# autor: Diego Roldan Urueña - 841723
# fecha: 09/02/2025

import math
import numpy as np 

def rad_to_deg(rads):
    return rads * 180 / math.pi

def deg_to_rad(deg):
    return deg * math.pi / 180

def loc_to_deg(loc):
    return [loc[0], loc[1], rad_to_deg(loc[2])]

  
def hom(x):
    """
    Crea una matriz de transformación homogénea 3x3 a partir de un punto 2D (x, y)
    y un ángulo de rotación.
    
    Argumentos:
        x (numpy.ndarray): Un vector de 3 elementos que contiene:
            - x[0]: coordenada x
            - x[1]: coordenada y
            - x[2]: ángulo de rotación en grados
    
    Retorna:
        numpy.ndarray: Una matriz de transformación homogénea 3x3
        
    Ejemplo:
        >>> x = np.array([1, 2, 30])  # x=1, y=2, ángulo=30°
        >>> T = hom(x)
    """
    angulo_rad = np.radians(x[2])
    cos_theta = np.cos(angulo_rad)
    sin_theta = np.sin(angulo_rad)
    
    T = np.array([
        [cos_theta, -sin_theta, x[0]],
        [sin_theta,  cos_theta, x[1]],
        [       0,          0,    1]
    ])
    return T

def loc(T):
    """
    Extrae la ubicación (x, y) y el ángulo de rotación desde una 
    matriz de transformación homogénea 3x3.
    
    Argumentos:
        T (numpy.ndarray): Una matriz de transformación homogénea 3x3
    
    Retorna:
        numpy.ndarray: Un vector de 3 elementos que contiene:
            - [0]: coordenada x
            - [1]: coordenada y
            - [2]: ángulo de rotación en grados
            
    Ejemplo:
        >>> T = np.array([[0.866, -0.5, 1],
        ...               [0.5, 0.866, 2],
        ...               [0, 0, 1]])
        >>> x = loc(T)  # Retorna: array([1, 2, 30])
    """
    # Extrae el ángulo usando arctan2 de los componentes seno y coseno
    angulo = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    
    # Extrae la posición de la última columna
    x = np.array([T[0, 2], T[1, 2], angulo])
    return x

def norm_pi(theta):
    """
    Normaliza un ángulo al rango [-π, π].
    
    Argumentos:
        theta (float): Ángulo a normalizar.
        
    Retorna:
        float: Ángulo normalizado en el rango [-π, π].
    
    Ejemplo:
        >>> norm_pi(3.5 * np.pi)
        -0.5 * np.pi
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi
