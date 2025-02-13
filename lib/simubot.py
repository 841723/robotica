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
  return np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]],
                   [np.sin(x[2]), np.cos(x[2]), x[1]],
                   [0, 0, 1]])

def loc(x):
  return np.array([x[0,2], x[1,2], norm_pi(np.arctan2(x[1,0],x[0,0]))])

