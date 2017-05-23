#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import geometry_msgs.msg as gm
import et_circumnavigation.srv as dns


#Paramaters
estimate_gain=rp.get_param('estimate_gain')
desired_dist=rp.get_param('desired_distance')
TARGET_POSITION=np.array(rp.get_param('target_position'))

#Variables
estimate=None
position=None
d_estimate=None
bearing_measurement=None
initial_est_dist=None
truemeasure=None
est_distance=None

#Lock
LOCK=thd.Lock();

#stop=False
#stop_publish=False

#delay=rp.get_param('delay')
#rp.sleep(delay)
rp.init_node('estimate')

FREQUENCY = 15e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY #Integration step




# Handler for the service "RemoveEstimate"
# def remove_estimate_handler(req):
#     global stop_publish   
#     LOCK.acquire()
#     stop_publish=True
#     LOCK.release()
#     return dns.RemoveAgentResponse()

# rp.Service('RemoveEstimate', dns.RemoveAgent, remove_estimate_handler)


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

def truemeasure_callback(msg):
    global truemeasure
    LOCK.acquire()
    truemeasure=msg.data 
    LOCK.release()
rp.Subscriber(
    name='truemeasure',
    data_class=sms.Bool,
    callback=truemeasure_callback,
    queue_size=10)



#Publishers

#Estimate
estimate_pub = rp.Publisher(
    name='estimate',
    data_class=gms.Point,
    queue_size=10)

#Estimated distance
est_distance_pub = rp.Publisher(
    name='est_distance',
    data_class=gm.PointStamped,
    queue_size=10)

#Error norm publisher
error_pub = rp.Publisher(
    name='error',
    data_class=gm.PointStamped,
    queue_size=10)

start = False

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [position, bearing_measurement,truemeasure]]):
           start = True
    LOCK.release()
    RATE.sleep()


while not rp.is_shutdown():
    LOCK.acquire()
#    if stop_publish:
#    #    rp.signal_shutdown("agent estimate removed")
#         stop=stop_publish
    if truemeasure:
        estimate = desired_dist*bearing_measurement+position
        truemeasure=False

    # Error norm 
    error=np.linalg.norm(TARGET_POSITION-estimate)

    # Estimate distance
    est_dist= np.linalg.norm(estimate-position)

    #Error norm publishing
    error_msg=gm.PointStamped()
    error_msg.header.seq=0
    error_msg.header.stamp = rp.Time.now()
    error_msg.point.x=error
    error_msg.point.y=0
    error_msg.point.z=0
    error_pub.publish(error_msg)

    #Estimated Distance publishing
    est_distance_msg=gm.PointStamped()
    est_distance_msg.header.seq=0
    est_distance_msg.header.stamp = rp.Time.now()
    est_distance_msg.point.x=est_dist
    est_distance_msg.point.y=0
    est_distance_msg.point.z=0

    #Estimated distance publishing
    est_distance_pub.publish(est_distance_msg)

    #Estimate publishing
    estimate_pub.publish(gms.Point(x=estimate[0], y=estimate[1]))

    LOCK.release()
    RATE.sleep()