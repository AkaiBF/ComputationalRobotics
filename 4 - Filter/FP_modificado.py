#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Computational Robotics 2017
# Computer Engineering
# Particle filter

from math import *
from robot import *
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import select
from datetime import datetime
# ******************************************************************************
# Functions declarations

def distancia(a,b):
  # Distance between two points (poses allowed)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Angular diference between a pose and a target point 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def pinta(secuencia,args):
  # Draws a points sequence
  t = np.array(secuencia).T.tolist()
  plt.plot(t[0],t[1],args)

def mostrar(objetivos,trayectoria,trayectreal,filtro):
  # Shows the map and the path
  plt.ion() # Interactive mode
  plt.clf()
  plt.axis('equal')
  # Set the graphics border
  objT   = np.array(objetivos).T.tolist()
  bordes = [min(objT[0]),max(objT[0]),min(objT[1]),max(objT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Map representation
  for p in filtro:
    dx = cos(p.orientation)*.05
    dy = sin(p.orientation)*.05
    plt.arrow(p.x,p.y,dx,dy,head_width=.05,head_length=.05,color='k')
  pinta(trayectoria,'--g')
  pinta(trayectreal,'-r')
  pinta(objetivos,'-.ob')
  p = hipotesis(filtro)
  dx = cos(p[2])*.05
  dy = sin(p[2])*.05
  plt.arrow(p[0],p[1],dx,dy,head_width=.075,head_length=.075,color='m')
  # Showing and checking of the keyboard pressing
  plt.draw()
#  if sys.stdin in select.select([sys.stdin],[],[],.01)[0]:
#    line = sys.stdin.readline()
  raw_input()

def genera_filtro(num_particulas, balizas, real, centro=[2,2], radio=3):
  # Initialization of a filter of size equal to 'num_particulas' which particles
  # imitate the given sample and distribute themselves randomly above a given area
  filtro = []
  medidas = real.sense(balizas)
  for i in range(num_particulas):
    filtro.append(real.copy())
    filtro[-1].set(centro[0]+random.uniform(-radio,radio),\
                   centro[1]+random.uniform(-radio,radio),\
                   random.uniform(-pi,pi))
    filtro[-1].measurement_prob(medidas,balizas)
  return filtro

def dispersion(filtro):
  # Spatial dispersion of the particle filter
  x = [p.x for p in filtro]
  y = [p.y for p in filtro]
  return distancia([min(x),min(y)],[max(x),max(y)])

def peso_medio(filtro):
  # Average normalized weight of the particle filter
  mayor = max([p.weight for p in filtro])
  return sum([p.weight/mayor for p in filtro])/len(filtro)

# ******************************************************************************

random.seed(0)

# Robot definition
P_INICIAL = [0.,4.,0.] # Initial pose (Position and orientation)
V_LINEAL  = .7         # Lineal speed    (m/s)
V_ANGULAR = 140.       # Angular speed   (�/s)
FPS       = 10.        # Time resolution (fps)
HOLONOMICO = 0         # Holonomic robot
GIROPARADO = 0         # Lineal speed = 0 required to rotate
LONGITUD   = .1        # Robot length

N_PARTIC  = 50         # Particle filter size
N_INICIAL = 2000       # Particle filter initial size

# Path definition
trayectorias = [
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.4*pi*i),2+2*cos(.4*pi*i)] for i in range(5)],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)],
    [[2+2*sin(1.2*pi*i),2+2*cos(1.2*pi*i)] for i in range(5)],
    [[2*(i+1),4*(1+cos(pi*i))] for i in range(6)],
    [[2+.2*(22-i)*sin(.1*pi*i),2+.2*(22-i)*cos(.1*pi*i)] for i in range(20)],
    [[2+(22-i)/5*sin(.1*pi*i),2+(22-i)/5*cos(.1*pi*i)] for i in range(20)]
    ]

# Target points definition
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <Índice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Constants definition
EPSILON = .1                # Distance threshold
V = V_LINEAL/FPS            # Meters per frame
W = V_ANGULAR*pi/(180*FPS)  # Radians per frame

real = robot()
real.set_noise(.01,.01,.01) # Lineal / radial / sense noise
real.set(*P_INICIAL)

#Particle filter and trajectory initialization
pf = genera_filtro(N_INICIAL,objetivos,real)
trayectoria = [hipotesis(pf)]
trayectreal = [real.pose()]
#
# Initial state display
mostrar(objetivos,trayectoria,trayectreal,pf)
pf = resample(pf,N_PARTIC)
#
tiempo  = 0.
espacio = 0.
for punto in objetivos:
  while distancia(trayectoria[-1],punto) > EPSILON and len(trayectoria) <= 1000:

    # Pose selection based on hypothesis
    pose = hipotesis(pf)
    #
    w = angulo_rel(pose,punto)
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto)
    if (v > V): v = V
    if (v < 0): v = 0
    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:v = 0
      real.move(w,v)
      # Particle Movement
      for i in range(N_PARTIC):
        pf[i].move(w,v)
      #
    else:
      real.move_triciclo(w,v,LONGITUD)
      # Particle Movement
      for i in range(N_PARTIC):
        pf[i].move_triciclo(w,v,LONGITUD)
      #
    # Localization hypothesis selection and actualization of the path
    for i in pf:
      i.measurement_prob(real.sense(objetivos),objetivos)

    # Hypothesis update
    trayectoria.append(hipotesis(pf))
    #
    trayectreal.append(real.pose())
    mostrar(objetivos,trayectoria,trayectreal,pf)

    # Particles resample
    if dispersion(pf) > 0.8:
      pf = resample(pf,N_PARTIC)
    #
    
    espacio += v
    tiempo  += 1

if len(trayectoria) > 1000:
  print "<< ! >> Puede que no se haya alcanzado la posición final."
print "Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s"
print "Error medio de la trayectoria: "+str(round(sum(\
    [distancia(trayectoria[i],trayectreal[i])\
    for i in range(len(trayectoria))])/tiempo,3))+"m"
raw_input()