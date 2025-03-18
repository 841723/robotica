# autor: Diego Roldan Urueña - 841723
# fecha: 09/02/2025

import numpy as np

# Simula movimiento del robot con vc=[v,w] en T seg. desde xWR
def simubot(vc,xWR,T):
  if vc[1]==0:   # w=0
      xRk=np.array([vc[0]*T, 0, 0])
  else:
      R=vc[0]/vc[1]
      dtitak=vc[1]*T
      titak=norm_pi(dtitak);
      xRk=np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])  

  xWRp=loc(np.dot(hom(xWR),hom(xRk)))
  return xWRp

def norm_pi(ang):
  while ang>np.pi:
    ang=ang-2*np.pi
  while ang<-np.pi:
    ang=ang+2*np.pi
  return ang

def hom(x):
  """
  Crea una matriz de transformación homogénea 3x3 a partir de un punto 2D (x, y)
  y un ángulo de rotación.
  
  Argumentos:
      x (numpy.ndarray): Un vector de 3 elementos que contiene:
          - x[0]: coordenada x
          - x[1]: coordenada y
          - x[2]: ángulo de rotación 
  
  Retorna:
      numpy.ndarray: Una matriz de transformación homogénea 3x3
      
  Ejemplo:
      >>> x = np.array([1, 2, 30])  # x=1, y=2, ángulo=30°
      >>> T = hom(x)
  """
  return np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]],
                   [np.sin(x[2]), np.cos(x[2]), x[1]],
                   [0, 0, 1]])

def loc(x):
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
  return np.array([x[0,2], x[1,2], norm_pi(np.arctan2(x[1,0],x[0,0]))])



