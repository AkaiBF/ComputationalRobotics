#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Computational Robotics 2017
# Computer Engineering
# Robot class

from math import *
import random
import numpy as np
import copy

class robot:
  def __init__(self):
    # Noise parameters and pose initialization
    self.x             = 0.
    self.y             = 0.
    self.orientation   = 0.
    self.forward_noise = 0.
    self.turn_noise    = 0.
    self.sense_noise   = 0.
    self.weight        = 1.
    self.old_weight    = 1.
    self.size          = 1.

  def copy(self):
    # Copy constructor
    return copy.deepcopy(self)

  def set(self, new_x, new_y, new_orientation):
    # Pose modification
    self.x = float(new_x)
    self.y = float(new_y)
    self.orientation = float(new_orientation)
    while self.orientation >  pi: self.orientation -= 2*pi
    while self.orientation < -pi: self.orientation += 2*pi

  def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
    # Noise parameters modifications
    self.forward_noise = float(new_f_noise);
    self.turn_noise    = float(new_t_noise);
    self.sense_noise   = float(new_s_noise);

  def pose(self):
    # Getter of the real position
    return [self.x, self.y, self.orientation]

  def sense1(self,landmark,noise):
    # Calculates the beacon distances
    return np.linalg.norm(np.subtract([self.x,self.y],landmark)) \
                                        + random.gauss(0.,noise)

  def sense(self, landmarks):
    # Calculates the distance for each beacons
    d = [self.sense1(l,self.sense_noise) for l in landmarks]
    d.append(self.orientation + random.gauss(0.,self.sense_noise))
    return d

  def move(self, turn, forward):
    # Modifies the robot pose (holonomic)
    self.orientation += float(turn) + random.gauss(0., self.turn_noise)
    while self.orientation >  pi: self.orientation -= 2*pi
    while self.orientation < -pi: self.orientation += 2*pi
    dist = float(forward) + random.gauss(0., self.forward_noise)
    self.x += cos(self.orientation) * dist
    self.y += sin(self.orientation) * dist

  def move_triciclo(self, turn, forward, largo):
    # Modifies the robot pose (Ackermann)
    dist = float(forward) + random.gauss(0., self.forward_noise)
    self.orientation += dist * tan(float(turn)) / largo\
              + random.gauss(0.0, self.turn_noise)
    while self.orientation >  pi: self.orientation -= 2*pi
    while self.orientation < -pi: self.orientation += 2*pi
    self.x += cos(self.orientation) * dist
    self.y += sin(self.orientation) * dist

  def Gaussian(self, mu, sigma, x):
    # Calculates the 'x' probability for a normal distribution
    # with average 'mu' and typical deviation 'sigma'
    if sigma:
      return exp(-(((mu-x)/sigma)**2)/2)/(sigma*sqrt(2*pi))
    else:
      return 0

  def measurement_prob(self, measurements, landmarks):
    # Calculates an average probability
    self.weight = 1.
    for i in range(len(measurements)-1):
      self.weight *= self.Gaussian(self.sense1(landmarks[i],0),\
                            self.sense_noise, measurements[i])
    diff = self.orientation - measurements[-1]
    while diff >  pi: diff -= 2*pi
    while diff < -pi: diff += 2*pi
    self.weight *= self.Gaussian(0, self.sense_noise, diff)
    return self.weight

  def __repr__(self):
    # Robot class representation
    return '[x=%.6s y=%.6s orient=%.6s]' % \
            (str(self.x), str(self.y), str(self.orientation))

def genera_filtro(num_particulas, balizas, real, centro=[2,2], radio=3):
  # Initialization of a filter of size equal to 'num_particulas', which particles
  # imitate the given sample and distribute themselves randomly over a given area
  filtro = []
  medidas = real.sense(balizas)
  for i in range(num_particulas):
    filtro.append(real.copy())
    filtro[-1].set(centro[0]+random.uniform(-radio,radio),\
                   centro[1]+random.uniform(-radio,radio),\
                   random.uniform(-pi,pi))
    filtro[-1].measurement_prob(medidas,balizas)
  return filtro

def hipotesis(pf):
  # Heavier particle's pose of the particle filter
  return max(pf,key=lambda r:r.weight).pose()

def resample(pf_in, particulas):
  # Resampling
  histograma_acumulativo = [0]
  for robot in pf_in:
    histograma_acumulativo.append(histograma_acumulativo[-1]+robot.weight)
  maximo = histograma_acumulativo[-1]
  if not maximo:
    return pf_in[:particulas]
  histograma_acumulativo = [h/maximo for h in histograma_acumulativo[1:]]
  pf_out = []
  for i in range(particulas):
    r = random.random()
    j = 0
    while r > histograma_acumulativo[j]: j += 1
    pf_out.append(pf_in[j].copy())
    pf_out[-1].old_weight = pf_out[-1].weight
    pf_out[-1].weight = 1.
  return pf_out

