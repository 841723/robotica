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

def c2m(cells):
    return cells * 0.4

def m2c(meters):
    return meters / 0.4

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


def polares(dx, dy, theta):
    """
    Convierte un vector de desplazamiento (dx, dy) y un ángulo theta
    en coordenadas polares (P, A, B).
    
    Argumentos:
        dx (float): Desplazamiento en x.
        dy (float): Desplazamiento en y.
        theta (float): Ángulo de rotación en radianes.
        
    Retorna:
        numpy.ndarray: Un vector de 3 elementos que contiene:
            - [0]: Magnitud de la distancia
            - [1]: Ángulo A
            - [2]: Ángulo B
    
    Ejemplo:
        >>> polares(3, 4, np.pi/2)
        array([5., 0., 1.57079633])
    """
    P = np.sqrt(dx**2 + dy**2)
    B = norm_pi(np.arctan2(dy, dx) + np.pi)
    A = B - theta
    return np.array([P, A, B])

def distancia_angular(th1, th2):
    return norm_pi(min(((th2 - th1 ) % (2*np.pi)), ((th1 - th2 ) % (2*np.pi))))


def distancia(x1,x2):
  return np.sqrt((x1[0]-x2[0])**2+(x1[1]-x2[1])**2)