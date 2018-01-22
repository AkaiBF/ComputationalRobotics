#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Computational Robotics 2017
# Computer Engineering
# Resolution of a Inverse Kinematic problem with the CCD algorithm
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Functions declaration

def muestra_origenes(O,final=0):
  # Shows the coordinate origin for each articulation
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Shows the robot in a graphic display
  plt.figure(1)
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
  raw_input()
  plt.clf()

def matriz_T(d,th,a,al):
  # Calculates T matrix with degree input angles
  
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  # Let 'th' be theta's vector
  # Let 'a' be length's vector
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

# ******************************************************************************
# Iterative Inverse Kinematic calculation with CCD method

# Random articular values for the initialization of the Forward Kinematic
th=[pi/2,-pi/2.,0.,0.]
a =[5.,2.,3.,4.]

# Limits
ang_inf=[0,-pi/2,-pi/2,-pi/2]
ang_sup=[0, pi/2, pi/2, pi/2]
lon_inf=[0,    0,    0,    0]
lon_sup=[9,    0,    0,    0]
#

# Joint type: 0 for angular and 1 for extendible
tipo=[1, 0, 0, 0]
#

L = sum(a) # Graphic representation variable
EPSILON = .01

plt.ion() # Interactive mode

# Introduction of the point for the Inverse Kinematic
if len(sys.argv) != 3:
  sys.exit("python " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]

O=range(len(th)+1) # The structure is stored in memory
O[0]=cin_dir(th,a) # Calculates the initial position
print "- Posicion inicial:"
muestra_origenes(O[0])

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  # For each articular combination:
  for i in range(len(tipo)):
    # Inverse kinematic calculations
    actual = len(tipo)-i-1
    # if added to diferentiate between angular and extensible operations
    if (tipo[actual] == 0):
        v1=np.subtract(objetivo,O[i][actual])
        v2=np.subtract(O[i][-1],O[i][actual])
        
        c1=atan2(v1[1],v1[0])
        c2=atan2(v2[1],v2[0])
        
        th[actual] += c1 - c2
        while th[actual] >  pi: th[actual] -= 2*pi
        while th[actual] < -pi: th[actual] += 2*pi
        O[i+1] = cin_dir(th,a)
    # Extensible joint control
    else:
        sumatorio = np.sum(th[:actual+1])
        oR = np.subtract(objetivo,O[i][-1])
        distancia = cos(sumatorio) * oR[0] + sin(sumatorio) * oR[1]
        a[actual]+=distancia
        if a[actual] > lon_sup[actual]: a[actual] = lon_sup[actual]
        if a[actual] < lon_inf[actual]: a[actual] = lon_inf[actual]
        O[i+1] = cin_dir(th,a)
    #
  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print "\n- Iteracion " + str(iteracion) + ':'
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print "Distancia al objetivo = " + str(round(dist,5))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print "\n" + str(iteracion) + " iteraciones para converger."
else:
  print "\nNo hay convergencia tras " + str(iteracion) + " iteraciones."
print "- Umbral de convergencia epsilon: " + str(EPSILON)
print "- Distancia al objetivo:          " + str(round(dist,5))
print "- Valores finales de las articulaciones:"
for i in range(len(th)):
  print "  theta" + str(i+1) + " = " + str(round(th[i],3))
for i in range(len(th)):
  print "  L" + str(i+1) + "     = " + str(round(a[i],3))
