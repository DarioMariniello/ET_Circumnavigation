#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import math as mt
import numpy as np
import geometry_msgs.msg as gm
import et_circumnavigation.msg as bms
import et_circumnavigation.srv as dns

#Parameters
k_d= rp.get_param('k_d')
k_fi= rp.get_param('k_fi')
estimate_gain= rp.get_param('estimate_gain')
desired_distance= rp.get_param('desired_distance')
alpha=rp.get_param('alpha')
TARGET_POSITION=np.array(rp.get_param("target_position"))
delay=rp.get_param('delay')

#Variables
position = None
estimate=None
est_distance=None
curr_bearing=[(1,1)]

global true_request
true_request=False

#stop_publish=False
#stop=False

LOCK = thd.Lock()

rp.sleep(delay)
rp.init_node('sensor_simulator')

# Handler for the service "TrueRequest"
# def true_request_handler(req):   
#     global true_request
#     LOCK.acquire()
#     true_request=True
#     LOCK.release()
#     return dns.TopologyResponse()

# rp.Service('TrueRequest', dns.Topology, true_request_handler)



#Subscribers
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


#rate=(2*alpha*k_fi)/mt.pi (offline calculation if Beta=0)
RATE = rp.Rate(150.0)



#Publishers

#bearing
bearing_pub = rp.Publisher(
    name='bearing_measurement',
    data_class=bms.Bearing,
    queue_size=10)

#distance
distance_pub = rp.Publisher(
    name='distance',
    data_class=gm.PointStamped,
    queue_size=10)

rp.wait_for_service('LastBeta')
last_beta_proxy=rp.ServiceProxy('LastBeta', dns.LastBeta)


start = False

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [ position]]):
        start = True
    LOCK.release()
    RATE.sleep()

start=False

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    bearing = (TARGET_POSITION-position)/np.linalg.norm(TARGET_POSITION-position)
    curr_bearing=bearing
    truemeasure=True
    last_beta=-100
    distance=np.linalg.norm(TARGET_POSITION-position)
    #Distance publishing
    distance_msg=gm.PointStamped()
    distance_msg.header.seq=0
    distance_msg.header.stamp = rp.Time.now()
    distance_msg.point.x=distance
    distance_msg.point.y=0
    distance_msg.point.z=0
    distance_pub.publish(distance_msg)
    #Bearing vector publishing
    bearing_msg= bms.Bearing()
    bearing_msg.bearing_measurement = gms.Vector(*bearing)
    bearing_msg.truemeasure = truemeasure
    bearing_pub.publish(bearing_msg)
    if all([not data is None for data in [ estimate,est_distance ]]):
        start = True
    LOCK.release()
    RATE.sleep()

while not rp.is_shutdown():
    LOCK.acquire()
#    if stop_publish:
#    	rp.signal_shutdown("agent sensor removed")

    #SELF-TRIGGERING MECHANISM
    distance=np.linalg.norm(TARGET_POSITION-position)
    est_bearing= (estimate-position)/np.linalg.norm(estimate-position)
    if np.dot(curr_bearing,est_bearing)<0.05 or true_request:
        bearing = (TARGET_POSITION-position)/np.linalg.norm(TARGET_POSITION-position)
        truemeasure=True
        last_beta=last_beta_proxy.call().last_beta
        curr_bearing=bearing
    else:
        bearing=(estimate-position)/np.linalg.norm(estimate-position)
        truemeasure=False

    #Distance publishing
    distance_msg=gm.PointStamped()
    distance_msg.header.seq=0
    distance_msg.header.stamp = rp.Time.now()
    distance_msg.point.x=distance
    distance_msg.point.y=0
    distance_msg.point.z=0
    distance_pub.publish(distance_msg)
    #Bearing vector publishing
    bearing_msg= bms.Bearing()
    bearing_msg.bearing_measurement = gms.Vector(*bearing)
    bearing_msg.truemeasure = truemeasure
    bearing_msg.last_beta = last_beta
    bearing_pub.publish(bearing_msg)
    LOCK.release()
    RATE.sleep()