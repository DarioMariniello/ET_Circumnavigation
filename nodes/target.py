#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import threading as thd
import numpy as np
import math

#Lock
LOCK=thd.Lock()

#Target initial position
target_position = np.array(rp.get_param('target_initial_position'))

rp.init_node('target')

#Frequency
FREQUENCY = 15e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY

#Publisher
target_pub = rp.Publisher('target_position', gms.Point, queue_size=10)

start = False

while not rp.is_shutdown():
    LOCK.acquire()
    #Target motion
    t=rp.get_time()
    d_target_position=np.array([0.05*math.sin(0.05*t),0.05*math.cos(0.05*t)])
    #Integration
    target_position = target_position+d_target_position*TIME_STEP
    LOCK.release()
    #Position publishing
    target_pub.publish(gms.Point(x=target_position[0], y=target_position[1]))
    RATE.sleep()


