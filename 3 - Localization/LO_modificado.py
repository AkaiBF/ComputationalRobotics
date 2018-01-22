#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Computational Robotics - School year 2017/2018
# Degree in Computer Engineering (Fourth year)
# Assignment:
#      Simulation of holonomic and non-holonomic mobile robots.

import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
# ******************************************************************************
# Functions declaration.

def distancia(a,b):
  # Distance between two points (it admits poses). 
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Angular difference between a pose and a' p' target point.
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def mostrar(objetivos,ideal,trayectoria):
  # Show targets and trajectory:
  plt.ion() # Interactive mode.
  # Setting the graphic borders.
  objT   = np.array(objetivos).T.tolist()
  trayT  = np.array(trayectoria).T.tolist()
  ideT   = np.array(ideal).T.tolist()
  bordes = [min(trayT[0]+objT[0]+ideT[0]),max(trayT[0]+objT[0]+ideT[0]),
            min(trayT[1]+objT[1]+ideT[1]),max(trayT[1]+objT[1]+ideT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])*.75
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representing objectives and trajectory.
  idealT = np.array(ideal).T.tolist()
  plt.plot(idealT[0],idealT[1],'-g')
  plt.plot(trayectoria[0][0],trayectoria[0][1],'or')
  r = radio * .1
  for p in trayectoria:
    plt.plot([p[0],p[0]+r*cos(p[2])],[p[1],p[1]+r*sin(p[2])],'-r')
    #plt.plot(p[0],p[1],'or')
  objT   = np.array(objetivos).T.tolist()
  plt.plot(objT[0],objT[1],'-.o')
  plt.show()
  raw_input()
  plt.clf()

def localizacion(balizas, real, ideal, centro, radio, mostrar=0):
  #  Search for the robot's most likely location, from its sensory system,
   # within a given region.
  mejor_pose = []
  mejor_peso = -1
  medidas = real.sense(balizas)

  PRECISION = 0.05
  r = int(radio/PRECISION)
  imagen = [[float('nan') for i in range(2*r)] for j in range(2*r)]
  for i in range(2*r):
    for j in range(2*r):
      x = centro[0]+(j-r)*PRECISION
      y = centro[1]+(i-r)*PRECISION
      ideal.set(x,y,ideal.orientation)
      peso = ideal.measurement_prob(medidas,balizas);
      if peso > mejor_peso:
        mejor_peso = peso
        mejor_pose = ideal.pose()
      imagen[i][j] = peso
  ideal.set(*mejor_pose)

  if mostrar:
    plt.ion() # Interactive mode.
    plt.xlim(centro[0]-radio,centro[0]+radio)
    plt.ylim(centro[1]-radio,centro[1]+radio)
    imagen.reverse()
    plt.imshow(imagen,extent=[centro[0]-radio,centro[0]+radio,\
                              centro[1]-radio,centro[1]+radio])
    balT = np.array(balizas).T.tolist();
    plt.plot(balT[0],balT[1],'or',ms=10)
    plt.plot(ideal.x,ideal.y,'D',c='#ff00ff',ms=10,mew=2)
    plt.plot(real.x, real.y, 'D',c='#00ff00',ms=10,mew=2)
    plt.show()
    raw_input()
    plt.clf()

# ******************************************************************************

# Robot definition:
P_INICIAL = [0.,4.,0.] #  Initial pose (position and orientation)
V_LINEAL  = .7         # Linear velocity   (m/s)
V_ANGULAR = 140.       # Angular velocity   (º/s)
FPS       = 10.        # Time resolution (fps)

HOLONOMICO = 1
GIROPARADO = 0
LONGITUD   = .2

# Definition of paths:
trayectorias = [
    [[1,3]],
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)]
    ]

# Definition of the target points:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <índice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definition of constants:
EPSILON = .1                # Distance threshold
V = V_LINEAL/FPS            # Meters per frame
W = V_ANGULAR*pi/(180*FPS)  # Radians per frame

ideal = robot()
ideal.set_noise(0,0,.1)   # Linear / radial / sensing noise
ideal.set(*P_INICIAL)     # 'splat' operator

real = robot()
real.set_noise(.01,.01,.1)  # Linear / radial / sensing noise
real.set(*P_INICIAL)

random.seed(0)
localizacion(objetivos,real,ideal,[2,2],3,mostrar=1)
tray_ideal = [ideal.pose()]  # Perceived trajectory
tray_real = [real.pose()]     # Trajectory followed

tiempo  = 0.
espacio = 0.
#random.seed(0)
random.seed(datetime.now())
for punto in objetivos:
  while distancia(tray_ideal[-1],punto) > EPSILON and len(tray_ideal) <= 1000:
    pose = ideal.pose()

    w = angulo_rel(pose,punto)
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto)
    if (v > V): v = V
    if (v < 0): v = 0

    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:
        v = 0
      ideal.move(w,v)
      real.move(w,v)
      medidas2 = real.sense(objetivos)
      prob2 = ideal.measurement_prob(medidas2,objetivos);
      if(prob2 < 0.7):
        localizacion(objetivos,real,ideal,ideal.pose(),real.sense_noise,mostrar=0)
    else:
      ideal.move_triciclo(w,v,LONGITUD)
      real.move_triciclo(w,v,LONGITUD)
    tray_ideal.append(ideal.pose())
    tray_real.append(real.pose())
    
    espacio += v
    tiempo  += 1

if len(tray_ideal) > 1000:
  print "<!> Trayectoria muy larga - puede que no se haya alcanzado la posición final."
print "Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s"
print "Distancia real al objetivo: "+\
    str(round(distancia(tray_real[-1],objetivos[-1]),3))+"m"
mostrar(objetivos,tray_ideal,tray_real)  # Representación gráfica

