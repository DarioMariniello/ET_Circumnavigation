#! /usr/bin/python

import rospy as rp
import geometry_msgs.msg as gm

import threading as thd

import numpy as np

import et_circumnavigation.srv as dns


LOCK = thd.Lock()
#Initial position
position = np.array(rp.get_param('initial_position'))
#Velocity
velocity = None

stop=False
stop_publish=False

delay=rp.get_param('delay')
rp.sleep(delay)
rp.init_node('integrator')

FREQUENCY = 15e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY


# Handler for the service "RemoveSensor"
def remove_vehicle_handler(req):   
    global stop_publish
    LOCK.acquire()
    stop_publish=True
    LOCK.release()
    return dns.RemoveAgentResponse()

rp.Service('RemoveVehicle', dns.RemoveAgent, remove_vehicle_handler)


#Publisher
pub = rp.Publisher('position', gm.Point, queue_size=10)


#Subscriber
def cmdvel_callback(msg):
    global velocity
    LOCK.acquire()
    velocity = np.array([msg.x, msg.y, msg.z])
    LOCK.release()
rp.Subscriber(
    name='cmdvel',
    data_class=gm.Vector3,
    callback=cmdvel_callback,
    queue_size=10)


start = False
while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if not velocity is None:
        start = True
    #else:
        #rp.logwarn('waiting for cmdvel')
    LOCK.release()
    #Initial position publishing
    pub.publish(gm.Point(x=position[0], y=position[1], z=position[2]))
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    if stop_publish:
        rp.signal_shutdown("agent vehicle removed")
    #Integration
    position = position+velocity*TIME_STEP
    #rp.logwarn(position)
    LOCK.release()
    #Position publishing
    pub.publish(gm.Point(x=position[0], y=position[1], z=position[2]))
    RATE.sleep()
