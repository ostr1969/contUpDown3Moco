#!/bin/python
#export PYTHONPATH=/home/barako/MOCOBIN/opensim-core/sdk/Python:./
import os
import opensim as osim
from math import pi
from opensim import Vec3
model = osim.Model('src/base3springs.osim')
model.buildSystem()
ground=model.updGround()
floor=osim.ContactHalfSpace(Vec3(0),Vec3(0,0,-1),ground,'floor')
model.addComponent(floor)
stiffness = 1.0e7; dissipation = 0.1; friction = 0.2; viscosity = 0.01
elastic=osim.ElasticFoundationForce()
elastic.setStiffness(stiffness)
elastic.setDissipation(dissipation)
elastic.setStaticFriction(friction)
elastic.setDynamicFriction(friction)
elastic.setViscousFriction(viscosity)
elastic.addGeometry('floor')
elastic.setName('contactfloor')
model.addComponent(elastic)
tip=osim.ContactSphere(.01,Vec3(0),ground,'tipcontact')
