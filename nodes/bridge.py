#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import geomtwo.msg as gms
import geomtwo.impl as gmi
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
frequency=5

LOCK=thd.Lock()


initial_goal = np.array(rospy.get_param('initial_goal'))

node_name = rospy.get_param('node_name')


rospy.init_node('bridge')

RATE = rospy.Rate(frequency)
TIME_STEP = 1.0/frequency

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
    queue_size=1)

def position_callback(msg):
    global position
    LOCK.acquire()
    position = np.array([msg.x, msg.y])
    LOCK.release()
rospy.Subscriber(
    name='position',
    data_class=gms.Point,
    callback=position_callback,
    queue_size=1)

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
    goal=initial_goal
    LOCK.release()
    #hover goal publishing
    msg.header.seq += 1
#    rospy.logwarn(circum)
#    rospy.logwarn('nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn')
    pub_goal.publish(msg)
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
    LOCK.release()
    #hover goal publishing
    msg.header.seq += 1
    pub_goal.publish(msg)
    RATE.sleep()

while not rospy.is_shutdown() and circum:
    LOCK.acquire()
    #Integration
    goal2=np.array([goal[0], goal[1]])
    rospy.logwarn(np.linalg.norm(position-goal2))
    if np.linalg.norm(position-goal2)<0.07:
        goal= position+velocity*TIME_STEP
        rospy.logwarn("goal updated!!!!!!!!!!!!!!!!")
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
    rospy.logwarn(goal)
    LOCK.release()
    #Position publishing
    pub_goal.publish(msg)
    RATE.sleep()
