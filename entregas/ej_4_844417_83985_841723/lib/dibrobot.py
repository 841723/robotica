# autores: Diego Roldan Urueña - 841723, Andrei Dumbrava Luca - 844417
# fecha: 09/02/2025

import numpy as np
import matplotlib.pyplot as plt
from lib.simubot import simubot

# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c='red',tamano='p', ax=None):
  if tamano=='p':
    largo=0.1
    corto=0.05
    descentre=0.01
  else:
    largo=0.5
    corto=0.25
    descentre=0.05

  trasera_dcha=np.array([-largo,-corto,1])
  trasera_izda=np.array([-largo,corto,1])
  delantera_dcha=np.array([largo,-corto,1])
  delantera_izda=np.array([largo,corto,1])
  frontal_robot=np.array([largo,0,1])
  tita=loc_eje[2]
  Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
  Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
  extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
  robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
  if ax is None:
    plt.plot(robot[0,:], robot[1,:], c)
  else:
    ax.plot(robot[0,:], robot[1,:], c)
  
  
  
def dibrecorrido(vc, wXr, time, steps = 10, color='red', tamano='g'):
  wXr_p = wXr
  if color != None:
    dibuja = True

  if dibuja:
    dibrobot(wXr_p, color, tamano)
  for i in range(0, steps):
    wXr_p = simubot(vc, wXr_p, time/steps)
    if dibuja:
      dibrobot(wXr_p, color, tamano)

  return wXr_p


