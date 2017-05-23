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

circum=False
started=False
t=0.0
start_time=0.0
position=None

LOCK=thd.Lock()


initial_goal = np.array(rospy.get_param('initial_goal'))



rospy.init_node('bridge')

RATE = rospy.Rate(10)
TIME_STEP = 1.0/10

x = initial_goal[0]
y = initial_goal[1]
z = initial_goal[2]

velocity = None
goal=None
start = False
worldFrame='/world'


def circum_handler(req):
    LOCK.acquire()
    global circum
    circum=True
    LOCK.release()
    return dns.CircumResponse()   

rospy.Service('Circum',dns.Circum,circum_handler)



#Publisher
pub_goal = rospy.Publisher('goal', PoseStamped, queue_size=1)
pub_position = rospy.Publisher('position', gms.Point, queue_size=10)

listener = tf.TransformListener()

#Subscriber
def cmdvel_callback(msg):
    global velocity
    LOCK.acquire()
    velocity = np.array([msg.x, msg.y])
    LOCK.release()
rospy.Subscriber(
    name='cmdvel',
    data_class=gms.Vector,
    callback=cmdvel_callback,
    queue_size=10)

#listener.waitForTransform(worldFrame, "/Crazyflie1", rospy.Time(), rospy.Duration(10.0))

while not rospy.is_shutdown() and not circum:
    LOCK.acquire()
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    listener.waitForTransform(worldFrame, "/Crazyflie1", rospy.Time(), rospy.Duration(2.0))
    (trans,rot) = listener.lookupTransform(worldFrame,'/Crazyflie1', rospy.Time(0), )
    LOCK.release()
    #hover goal publishing
    msg.header.seq += 1
#    rospy.logwarn(circum)
#    rospy.logwarn('nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn')
    pub_goal.publish(msg)
    pub_position.publish(gms.Point(x=trans[0], y=trans[1]))
    RATE.sleep()


while not rospy.is_shutdown() and circum and not start:
    LOCK.acquire()
    if not velocity is None:
        start = True
    else:
        rospy.logwarn('waiting for cmdvel')

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    listener.waitForTransform(worldFrame, "/Crazyflie1", rospy.Time(), rospy.Duration(0.5))
    (trans,rot) = listener.lookupTransform('/world','/Crazyflie1', rospy.Time(0))
    LOCK.release()
    #hover goal publishing
    msg.header.seq += 1
#    rospy.logwarn(circum)
#    rospy.logwarn('nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn')
    pub_goal.publish(msg)
    pub_position.publish(gms.Point(x=trans[0], y=trans[1]))
    RATE.sleep()

while not rospy.is_shutdown() and circum:
    LOCK.acquire()
    listener.waitForTransform(worldFrame, "/Crazyflie1", rospy.Time(), rospy.Duration(0.5))
    (trans,rot) = listener.lookupTransform('/world','/Crazyflie1', rospy.Time(0))
    position=np.array([trans[0],trans[1]])
    #Integration
    goal= position+velocity*TIME_STEP
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = goal[0]
    msg.pose.position.y = goal[1]
    msg.pose.position.z = z ####trans[2] doesnt hold
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    LOCK.release()
    #Position publishing
    pub_goal.publish(msg)
    pub_position.publish(gms.Point(x=trans[0], y=trans[1]))
    RATE.sleep()




