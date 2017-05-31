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


rp.wait_for_message('position', gms.Point)

def bearing_measurement_handler(req):
    versor = gmi.Versor(TARGET_POSITION-position).serialize()
    return dns.BearingMeasurementResponse(versor)
rp.Service(
    'bearing_measurement',
    dns.BearingMeasurement,
    bearing_measurement_handler)



rp.spin()
