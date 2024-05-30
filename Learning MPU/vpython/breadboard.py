from posixpath import split
from vpython import *
import numpy as np
import time 
import serial 
sleep(1)

ad = serial.Serial('com3', 115200)

scene.range = 5
scene.forward = vector(-1,-1,-1)
scene.width = 600
scene.height =600 
toDeg = 180./np.pi
toRad = 1/toDeg

bboard = box(length =8,height = .2, width = 2,  color = color.white, opacity =.4)
arduino = box(length =1.75,height= .05, width= .75,  color= color.blue, pos = vector(-3, .125, 0))
mpu = box(length =1,height= .05, width= .75,  color= color.green, pos = vector(-1.2, .125, 0))


myObj = compound([bboard, arduino, mpu ])

Xarrow = arrow(axis=vector(1,0,0),lenght=3,shaftwidth=.1,color=color.red)
Yarrow = arrow(axis=vector(0,1,0),lenght=3,shaftwidth=.1,color=color.blue)
Zarrow = arrow(axis=vector(0,0,1),lenght=3,shaftwidth=.1,color=color.green)

forwordArrow = arrow(axis=vector(1,0,0),lenght=4,shaftwidth=.1,color=color.purple)
upwardArrow = arrow(axis=vector(0,1,0),lenght=4,shaftwidth=.1,color=color.magenta)
sidewordArrow = arrow(axis=vector(0,0,1),lenght=4,shaftwidth=.1,color=color.orange)
while (True):

    while(ad.inWaiting() == 0):
        pass
    dataPacket= ad.readline()
    dataPacket= str(dataPacket, "utf_8")
    splitData = dataPacket.split(" ")
    
    pitch = float(splitData[0])*toRad
    roll = float(splitData[1])*toRad
    yaw = float(splitData[2])*toRad
    print("yaw=", yaw*toDeg, " pitch=", pitch*toDeg, " roll=", roll*toDeg)

    rate(50)
    k = vector(cos(yaw)*cos(pitch),sin(pitch),sin(yaw)*sin(pitch))
    y = vector(0,1,0)
    s = cross(k, y)
    v = cross(s, k)
    Vrot = v*cos(roll)+ cross(k, v)*sin(roll)


    forwordArrow.axis = k
    upwardArrow.axis = Vrot
    sidewordArrow.axis = cross(k, Vrot)
    myObj.axis = k
    myObj.up = Vrot
    forwordArrow.length = 4
    upwardArrow.length = 4
    sidewordArrow.length = 4





