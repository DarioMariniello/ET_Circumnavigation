#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import geometry_msgs.msg as gm
import circumnavigation_moving_target.srv as dns


delay=rp.get_param('delay')
rp.sleep(delay)
rp.init_node('sensor_simulator')

position = None
target_position=None
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



#Subscriber
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

def target_position_callback(msg):
    global target_position
    LOCK.acquire()
    target_position = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='target_position',
    data_class=gms.Point,
    callback=target_position_callback,
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

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [ position , target_position ]]):
           start = True
    #else:
        #rp.logwarn('waiting for measurements')
    LOCK.release()
    RATE.sleep()

while not rp.is_shutdown():
    LOCK.acquire()
    if stop_publish:
    	rp.signal_shutdown("agent sensor removed")
    #Bearing vector (phi)
    bearing = (target_position-position)/np.linalg.norm(target_position-position)
    #rp.logwarn(bearing)
    if not target_position is None:
        distance=np.linalg.norm(target_position-position)
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
    LOCK.release()
    RATE.sleep()

