#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import geomtwo.impl as gmi
import threading as thd
import et_circumnavigation.srv as dns
import numpy as np

#Parameters
TARGET_POSITION = gmi.Point(*rp.get_param("target_position"))
DELAY = rp.get_param('delay')
LOCK = thd.Lock()

#Variables
position = None
stop_publish=False

rp.init_node('sensor_simulator')
rp.sleep(DELAY)

def remove_sensor_handler(req):   
    global stop_publish
    LOCK.acquire()
    stop_publish=True
    LOCK.release()
    return dns.RemoveAgentResponse()
rp.Service('RemoveSensor', dns.RemoveAgent, remove_sensor_handler)

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
    queue_size=1)


rp.wait_for_message('position', gms.Point)

def bearing_measurement_handler(req):
    LOCK.acquire()
    versor = gmi.Versor(TARGET_POSITION-position).serialize()
    LOCK.release()
    return dns.BearingMeasurementResponse(versor)
rp.Service(
    'bearing_measurement',
    dns.BearingMeasurement,
    bearing_measurement_handler)



rp.spin()
