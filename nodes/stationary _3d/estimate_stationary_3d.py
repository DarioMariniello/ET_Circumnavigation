#! /usr/bin/python

import rospy as rp

import std_msgs.msg as sms
import threading as thd
import numpy as np
import geometry_msgs.msg as gm
import ET_circumnavigation.srv as dns


#Paramaters
estimate_gain=rp.get_param('estimate_gain') #from the .yaml file
#Initial estimate
estimate=np.array(rp.get_param('initial_estimate')) #from the .launch file
position=np.array(rp.get_param('initial_position'))
desired_dist=rp.get_param('desired_distance')
bearing_measurement=None
initial_est_dist=None
initiale_xy_est_dist=None
truemeasure=None
est_distance=None
xy_est_distance=None

TARGET_POSITION=np.array(rp.get_param('target_position'))
#Lock
LOCK=thd.Lock();

stop=False
stop_publish=False

delay=rp.get_param('delay')
rp.sleep(delay)
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
    position = np.array([msg.x, msg.y, msg.z])
    LOCK.release()
rp.Subscriber(
    name='position',
    data_class=gm.Point,
    callback=position_callback,
    queue_size=10)
    
def bearing_measurement_callback(msg):
    global bearing_measurement
    LOCK.acquire()
    bearing_measurement = np.array([msg.x, msg.y, msg.z])
    LOCK.release()
rp.Subscriber(
    name='bearing_measurement',
    data_class=gm.Vector3,
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



##########Publisher

#Estimate
estimate_pub = rp.Publisher(
    name='estimate',
    data_class=gm.Point,
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
    #else:
        #rp.logwarn('waiting for position and measurements')
    LOCK.release()
    #Initial estimate and est_distance publishing
    estimate_pub.publish(gm.Vector3(x=estimate[0], y=estimate[1], z=estimate[2]))
    initial_est_dist=np.linalg.norm(estimate-position)
    initiale_xy_est_dist=np.linalg.norm(np.array([estimate[0],estimate[1], 0.0])-np.array([position[0], position[1], 0.0]))
    est_distance_msg=gm.PointStamped()
    est_distance_msg.header.seq=0
    est_distance_msg.header.stamp = rp.Time.now()
    est_distance_msg.point.x=initial_est_dist
    est_distance_msg.point.y=initiale_xy_est_dist
    est_distance_msg.point.z=0
    est_distance_pub.publish(est_distance_msg)
    RATE.sleep()
while not rp.is_shutdown() and not stop:
    LOCK.acquire()
    if stop_publish:
    #    rp.signal_shutdown("agent estimate removed")
         stop=stop_publish

    #Estimate algorithm


#    rp.logwarn(truemeasure)
#    truemeasure=False
    est_distance= np.linalg.norm(estimate-position)
    xy_est_distance=np.linalg.norm(np.array([estimate[0],estimate[1], 0.0])-np.array([position[0], position[1], 0.0]))
    if truemeasure:
    	#estimate = (np.outer(bearing_measurement,bearing_measurement).dot(estimate-position))+position
        estimate = desired_dist*bearing_measurement+position
#        rp.logwarn("ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt")
#        rp.logwarn(bearing_measurement)
        truemeasure=False

    # Error norm 
    error=np.linalg.norm(TARGET_POSITION-estimate)
    # Estimate distance
    est_dist= np.linalg.norm(estimate-position)
    xy_est_distance=np.linalg.norm(np.array([estimate[0],estimate[1], 0.0])-np.array([position[0], position[1], 0.0]))
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
    est_distance_msg.point.y=xy_est_distance
    est_distance_msg.point.z=0
    LOCK.release()  
    #Estimated distance publishing
    est_distance_pub.publish(est_distance_msg)
    #Estimate publishing
    estimate_pub.publish(gm.Point(x=estimate[0], y=estimate[1], z=estimate[2]))
    RATE.sleep()
