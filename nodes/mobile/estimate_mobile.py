#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import geometry_msgs.msg as gm
import et_circumnavigation.srv as dns


#Paramaters
estimate_gain=rp.get_param('estimate_gain') #from the .yaml file
#Initial estimate
estimate=np.array(rp.get_param('initial_estimate')) #from the .launch file
position=None
bearing_measurement=None
target_position=None
#Lock
LOCK=thd.Lock();

stop=False
stop_publish=False

rp.init_node('estimate')

FREQUENCY = 15e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY #Integration step




# Handler for the service "RemoveSensor"
def remove_estimate_handler(req):
    global stop_publish   
    LOCK.acquire()
    stop_publish=True
    LOCK.release()
    return dns.RemoveAgentResponse()

rp.Service('RemoveEstimate', dns.RemoveAgent, remove_estimate_handler)



#This node publishes the estimate of the target position and subscribes to the agent position topic and to the bearing measurements vector(phi)
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
    
def bearing_measurement_callback(msg):
    global bearing_measurement
    LOCK.acquire()
    bearing_measurement = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='bearing_measurement',
    data_class=gms.Vector,
    callback=bearing_measurement_callback,
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


#Publisher
estimate_pub = rp.Publisher(
    name='estimate',
    data_class=gms.Point,
    queue_size=10)
#Error norm publisher
error_pub = rp.Publisher(
    name='error',
    data_class=gm.PointStamped,
    queue_size=10)

start = False
while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [position, bearing_measurement]]):
           start = True
    #else:
        #rp.logwarn('waiting for position and measurements')
    LOCK.release()
    #Initial estimate publishing
    estimate_pub.publish(gms.Point(x=estimate[0], y=estimate[1]))
    RATE.sleep()
while not rp.is_shutdown() and not stop:
    LOCK.acquire()
    if stop_publish:
    #    rp.signal_shutdown("agent estimate removed")
         stop=stop_publish
    #Estimate algorithm
    d_estimate=-estimate_gain*(np.eye(2)-np.outer(bearing_measurement,bearing_measurement)).dot(estimate-position)
    #Integration
    estimate= estimate+d_estimate*TIME_STEP
    # Error norm 
    if not target_position is None:
        error=np.linalg.norm(target_position-estimate)
        #Error norm publishing
        error_msg=gm.PointStamped()
        error_msg.header.seq=0
        error_msg.header.stamp = rp.Time.now()
        error_msg.point.x=error
        error_msg.point.y=0
        error_msg.point.z=0
        error_pub.publish(error_msg)
    LOCK.release()    
    #Estimate publishing
    estimate_pub.publish(gms.Point(x=estimate[0], y=estimate[1]))
    RATE.sleep()
