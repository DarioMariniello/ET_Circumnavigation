#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import geomtwo.impl as gmi

import threading as thd

import numpy as np

import et_circumnavigation.srv as dns


LOCK = thd.Lock()
#Initial position
position = gmi.Vector(*rp.get_param('initial_position'))
velocity = None

#stop=False
stop_publish=False

delay=rp.get_param('delay')
rp.sleep(delay)
rp.init_node('integrator')

FREQUENCY = 10.0
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY



def remove_vehicle_handler(req):
    global stop_publish
    LOCK.acquire()
    stop_publish=True
    LOCK.release()
    return dns.RemoveAgentResponse()
rp.Service('RemoveVehicle', dns.RemoveAgent, remove_vehicle_handler)


#Publisher
pub = rp.Publisher('position', gms.Point, queue_size=5)


#Subscriber
def cmdvel_callback(msg):
    global velocity
    LOCK.acquire()
    velocity = gmi.Vector(msg)
    LOCK.release()
rp.Subscriber(
    name='cmdvel',
    data_class=gms.Vector,
    callback=cmdvel_callback,
    queue_size=10)


start = False
while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if not velocity is None:
        start = True
    LOCK.release()
    pub.publish(position.serialize())
    RATE.sleep()

while not rp.is_shutdown():
    LOCK.acquire()
    if not velocity is None:
        # if stop_publish:
        #     rp.signal_shutdown("agent vehicle removed")
        #Integration
        position = position+velocity*TIME_STEP
        velocity = None
        #rp.logwarn(position)
    LOCK.release()
    pub.publish(position.serialize())
    #Position publishing
    RATE.sleep()
