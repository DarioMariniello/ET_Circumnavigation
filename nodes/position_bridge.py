#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import geomtwo.msg as gms
import math as mt
import time
import threading as thd
import et_circumnavigation.srv as dns
import numpy as np


started=False
t=0.0
start_time=0.0
position=None
frequency=100

LOCK=thd.Lock()


node_name = rospy.get_param('node_name')


rospy.init_node('bridge')

RATE = rospy.Rate(frequency)
TIME_STEP = 1.0/frequency




start = False
worldFrame='/world'




#Publisher
pub_position = rospy.Publisher('position', gms.Point, queue_size=1)

listener = tf.TransformListener()


while not rospy.is_shutdown() :
    LOCK.acquire()
    listener.waitForTransform(worldFrame, "/"+node_name, rospy.Time(), rospy.Duration(2.0))
    (trans,rot) = listener.lookupTransform(worldFrame,"/"+node_name, rospy.Time(0), )
    LOCK.release()
    pub_position.publish(gms.Point(x=trans[0], y=trans[1]))
    RATE.sleep()