#! /usr/bin/python

import rospy as rp

import std_msgs.msg as sms
import threading as thd
import numpy as np
import math
import copy as cp
import et_circumnavigation.srv as dns
import geometry_msgs.msg as gm


# Variables
position = None
estimate = None
est_distance=0
xy_est_distance=0
# Vector fi of the agent
bearing_measurement = None
# Parameters
DESIRED_DISTANCE = rp.get_param('desired_distance')  # from the .yaml file
alpha= rp.get_param('alpha') # from the .yaml file
k_fi= rp.get_param('k_fi')
k_d= rp.get_param('k_d')
estimate_gain= rp.get_param('estimate_gain')


node_name=rp.get_param('node_name')
delay=rp.get_param('delay')

z_axis=np.array([0.0,0.0,1.0])

# Lock
LOCK = thd.Lock()

stop_publish=False


rp.sleep(delay)
rp.init_node('planner') 

# Counterclockwise_angle function
# This function returns the counterclockwise angle between two vectors
def Angle(bearing_measurement,neighbor_bearing_measurement):    
    phi_i=np.array([bearing_measurement[0],bearing_measurement[1],0.0]) 
    phi_j=np.array([neighbor_bearing_measurement[0],neighbor_bearing_measurement[1],0.0])
    n_i=np.linalg.norm(phi_i)
    n_j=np.linalg.norm(phi_j)
    sp=np.inner(phi_i,phi_j)
    vp=np.cross(phi_i,phi_j)
    cos_beta=sp/(n_i*n_j)
    sin_beta=vp[2]/(n_i*n_j)
    beta=math.atan2(sin_beta,cos_beta)
    if beta<0:
        beta=beta+2*math.pi
    return beta 


# Subscribers to the bearing vectors of the other agents
agent_names=[]
agent_bearing_measurement = {}
def agent_callback(msg, name):
    global agent_bearing_measurement
    LOCK.acquire()
    agent_bearing_measurement[name] = [msg.x, msg.y ,msg.z]
    LOCK.release()



bearing_measurement_subscribers={}
# Handler for the service "AddAgent": the new agent call "AddAgent" and the others add its name in agent_name and subscribe to its fi 
def add_agent_handler(req):
    global agent_names
    global agent_bearing_measurement
    global bearing_measurement_subscribers
    bearing_measurement_subscribers[req.name]=rp.Subscriber(
        name='/'+req.name+'/bearing_measurement',
        data_class=gm.Vector3,
        callback=agent_callback,
        callback_args=req.name,
        queue_size=1)   
    LOCK.acquire()
    agent_names.append(req.name)
    agent_bearing_measurement[req.name]=None
    LOCK.release()
    return dns.AddAgentResponse()

rp.Service('AddAgent', dns.AddAgent, add_agent_handler)

# Handler for the service "RemoveAgent"
def remove_agent_handler(req):   
    global bearing_measurement_subscribers
    global agent_names
    global agent_bearing_measurement
    LOCK.acquire()
    bearing_measurement_subscribers[req.name].unregister()
    agent_names.remove(req.name)
    del agent_bearing_measurement[req.name]
    LOCK.release()
    return dns.RemoveAgentResponse()

rp.Service('RemoveAgent', dns.RemoveAgent, remove_agent_handler)

# Call to the service "AddMe": the agent requires to the cloud to add his name
rp.wait_for_service('/AddMe')
add_me_proxy=rp.ServiceProxy('/AddMe', dns.AddAgent)
add_me_proxy.call(node_name)


# Handler for the service "RemovePlanner"
def remove_planner_handler(req):
    global stop_publish   
    LOCK.acquire()
    stop_publish=True
    LOCK.release()
    return dns.RemoveAgentResponse()

rp.Service('RemovePlanner', dns.RemoveAgent, remove_planner_handler)

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

def estimate_callback(msg):
    global estimate
    LOCK.acquire()
    estimate = np.array([msg.x, msg.y, msg.z])
    LOCK.release()
rp.Subscriber(
    name='estimate',
    data_class=gm.Point,
    callback=estimate_callback,
    queue_size=10)

def est_distance_callback(msg):
    global est_distance
    LOCK.acquire()
    est_distance = msg.point.x
    xy_est_distance=msg.point.y
    LOCK.release()
rp.Subscriber(
    name='est_distance',
    data_class=gm.PointStamped,
    callback=est_distance_callback,
    queue_size=10)


RATE = rp.Rate(150.0)
start = False

#Publishers
cmdvel_pub = rp.Publisher(
    name='cmdvel',
    data_class=gm.Vector3,
    queue_size=10)

beta_pub = rp.Publisher(
    name='beta',
    data_class=gm.PointStamped,
    queue_size=10)


while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [position, estimate, bearing_measurement,est_distance]]):
           start = True
           rp.logwarn(start)
    else:
        rp.logwarn('waiting for measurements')
    LOCK.release()
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    if stop_publish:
    	rp.signal_shutdown("agent planner removed")
    #Bearing vector in the clockwise direction
    bearing_measurement=(estimate-position)/np.linalg.norm(estimate-position)
    phi_bar=np.array([bearing_measurement[1],-bearing_measurement[0],0.0])/np.linalg.norm(np.array([bearing_measurement[1],-bearing_measurement[0],0.0]))
    #Number of other agents in the network
    agent_beta=[]
    #print(agent_names)
    #Counterclockwise angle
    for name in agent_names:
        #if  name!=node_name and not agent_bearing_measurement[name] is None:
        if  not agent_bearing_measurement[name] is None:
            agent_beta.append(Angle(bearing_measurement,agent_bearing_measurement[name]))
    # If there is only one agent beta=0
    beta=0
    if len(agent_beta)>0:
        beta=min(agent_beta)

    xy_bearing=np.array([bearing_measurement[0],bearing_measurement[1],0.0])/np.linalg.norm(np.array([bearing_measurement[0],bearing_measurement[1],0.0]))
    vel = k_d*xy_bearing*(xy_est_distance-DESIRED_DISTANCE)+k_fi*phi_bar*(alpha)+k_d*np.dot([np.cross(phi_bar,bearing_measurement)],xy_bearing)*z_axis
    #Velocity message                                                                                            2*k_fi*est_distance*phi_bar*(alpha+beta)

    rp.logwarn(vel)
    cmdvel_msg = gm.Vector3(x=vel[0], y=vel[1], z=vel[2])
    #Beta message
    beta_msg=gm.PointStamped()
    beta_msg.header.seq=0
    beta_msg.header.stamp = rp.Time.now()
    beta_msg.point.x=beta
    beta_msg.point.y=0
    beta_msg.point.z=0
    LOCK.release()
    # Publishing
    cmdvel_pub.publish(cmdvel_msg)
    beta_pub.publish(beta_msg)
    RATE.sleep()
