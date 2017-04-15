#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import geometry_msgs.msg as gm
import ET_circumnavigation.srv as dns

#Parameters
k_d= rp.get_param('k_d')
estimate_gain= rp.get_param('estimate_gain')
desired_distance= rp.get_param('desired_distance')

delay=rp.get_param('delay')
rp.sleep(delay)
rp.init_node('sensor_simulator')

position = None
estimate=None
est_distance=None

TARGET_POSITION=np.array(rp.get_param("target_position"))
stop_publish=False
stop=False

LOCK = thd.Lock()

# Handler for the service "RemoveSensor"
def remove_sensor_handler(req):   
    global stop_publish
    LOCK.acquire()
    stop_publish=True
    LOCK.release()
    return dns.RemoveAgentResponse()

rp.Service('RemoveSensor', dns.RemoveAgent, remove_sensor_handler)



##Subscriber
#Position
def position_callback(msg):
    global position
    LOCK.acquire()
    position = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='position',
    data_class=gms.Point,
    callback=position_callback,
    queue_size=10)
#Estimate 
def estimate_callback(msg):
    global estimate
    LOCK.acquire()
    estimate = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='estimate',
    data_class=gms.Point,
    callback=estimate_callback,
    queue_size=10)
#Estimated distance
def est_distance_callback(msg):
    global est_distance
    LOCK.acquire()
    est_distance = msg.point.x
    LOCK.release()
rp.Subscriber(
    name='est_distance',
    data_class=gm.PointStamped,
    callback=est_distance_callback,
    queue_size=10)


RATE = rp.Rate(150.0)
start = False
#Publishers
bearing_pub = rp.Publisher(
    name='bearing_measurement',
    data_class=gms.Vector,
    queue_size=10)
distance_pub = rp.Publisher(
    name='distance',
    data_class=gm.PointStamped,
    queue_size=10)
truemeasure_pub= rp.Publisher(
    name='truemeasure',
    data_class=sms.Bool,
    queue_size=10)


while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [ position,estimate,est_distance ]]):
           start = True
    #else:
        #rp.logwarn('waiting for measurements')
    LOCK.release()
    RATE.sleep()

while not rp.is_shutdown():
    LOCK.acquire()
    if stop_publish:
    	rp.signal_shutdown("agent sensor removed")
    

    #EVENT TRIGGERING MECHANISM

#   rp.logwarn(bearing)
    distance=np.linalg.norm(TARGET_POSITION-position)
#    rp.logwarn(est_distance)
#    rp.logwarn(distance)
    if est_distance > (desired_distance/5) and distance<est_distance :
	bearing = (estimate-position)/est_distance
        truemeasure=False
    else:
 	bearing = (TARGET_POSITION-position)/np.linalg.norm(TARGET_POSITION-position)
        truemeasure=True

       
    

    #Distance publishing
    distance_msg=gm.PointStamped()
    distance_msg.header.seq=0
    distance_msg.header.stamp = rp.Time.now()
    distance_msg.point.x=distance
    distance_msg.point.y=0
    distance_msg.point.z=0
    distance_pub.publish(distance_msg)
    #Bearing vector publishing
    bearing_msg = gms.Vector(*bearing)
    bearing_pub.publish(bearing_msg)
    #Truemeasure publishing
    truemeasure_msg=sms.Bool(truemeasure)
    truemeasure_pub.publish(truemeasure_msg)
    LOCK.release()
    RATE.sleep()

