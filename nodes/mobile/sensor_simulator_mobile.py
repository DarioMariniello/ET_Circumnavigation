#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import geomtwo.impl as gmi
import threading as thd
import et_circumnavigation.srv as dns
import numpy as np

#Parameters
target_position = None
target_position_buff = None
DELAY = rp.get_param('delay')
LOCK = thd.Lock()

#Variables
position = None


rp.init_node('sensor_simulator')
rp.sleep(DELAY)


#Subscribers
def position_callback(msg):
    global position
    LOCK.acquire()
    position = gmi.Point(msg)
    LOCK.release()
rp.Subscriber(
    name='position',
    data_class=gms.Point,
    callback=position_callback,
    queue_size=10)

def target_position_callback(msg):
    global target_position
    LOCK.acquire()
    target_position=gmi.Point(msg)
    LOCK.release()
rp.Subscriber(
    name='target_position',
    data_class=gms.Point,
    callback=target_position_callback,
    queue_size=10)

rp.wait_for_message('position', gms.Point)

def bearing_measurement_handler(req):
    LOCK.acquire()
    versor = gmi.Versor(target_position-position).serialize()
    LOCK.release()
    return dns.BearingMeasurementResponse(versor)
rp.Service(
    'bearing_measurement',
    dns.BearingMeasurement,
    bearing_measurement_handler)



rp.spin()