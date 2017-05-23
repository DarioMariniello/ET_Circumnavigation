#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import math
import copy as cp
import et_circumnavigation.srv as dns
import geometry_msgs.msg as gm

# Parameters
DESIRED_DISTANCE = rp.get_param('desired_distance')  # from the .yaml file
alpha= rp.get_param('alpha') # from the .yaml file
k_fi= rp.get_param('k_fi')
k_d= rp.get_param('k_d')
estimate_gain= rp.get_param('estimate_gain')
node_name=rp.get_param('node_name')
delay=rp.get_param('delay')

# Variables
est_distance=0.0
bearing_measurement = None


agents_number=0

neighbor=None

agent_beta=0.0


LOCK = thd.Lock()

#stop_publish=False


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
agent_truemeasure = {}

def agent_callback_b(msg, name):
    global agent_bearing_measurement
    LOCK.acquire()
    agent_bearing_measurement[name] = [msg.x, msg.y]
    LOCK.release()

def agent_callback_t(msg, name):
    global agent_turemeasure
    LOCK.acquire()
    agent_truemeasure[name] = msg.data
    LOCK.release()



bearing_measurement_subscribers={}
truemeasure_subscribers={}
# Handler for the service "AddAgent": the new agent call "AddAgent" and the others add its name in agent_name and subscribe to its fi 
def add_agent_handler(req):
    global agent_names
    global agent_bearing_measurement
    global agent_truemeasure
    global bearing_measurement_subscribers
    global truemeasure_subscribers
    bearing_measurement_subscribers[req.name]=rp.Subscriber(
        name='/'+req.name+'/bearing_measurement',
        data_class=gms.Vector,
        callback=agent_callback_b,
        callback_args=req.name,
        queue_size=1)   

    truemeasure_subscribers[req.name]=rp.Subscriber(
        name='/'+req.name+'/truemeasure',
        data_class=sms.Bool,
        callback=agent_callback_t,
        callback_args=req.name,
        queue_size=1)

    LOCK.acquire()
    agent_names.append(req.name)
    agent_bearing_measurement[req.name]=None
    agent_truemeasure[req.name]=None
    LOCK.release()
    return dns.AddAgentResponse()

rp.Service('AddAgent', dns.AddAgent, add_agent_handler)


# Handler for the service "RemoveAgent"
# def remove_agent_handler(req):   
#     global bearing_measurement_subscribers
#     global agent_names
#     global agent_bearing_measurement
#     LOCK.acquire()
#     bearing_measurement_subscribers[req.name].unregister()
#     agent_names.remove(req.name)
#     del agent_bearing_measurement[req.name]
#     LOCK.release()
#     return dns.RemoveAgentResponse()

#rp.Service('RemoveAgent', dns.RemoveAgent, remove_agent_handler)

# Call to the service "AddMe": the agent requires to the cloud to add his name
rp.wait_for_service('/AddMe')
add_me_proxy=rp.ServiceProxy('/AddMe', dns.AddAgent)
add_me_proxy.call(node_name)

# rp.wait_for_service('/Topology')
# topology_proxy=rp.ServiceProxy('/Topology', dns.Topology)
# topology_proxy.call(node_name)


# Handler for the service "RemovePlanner"
# def remove_planner_handler(req):
#     global stop_publish   
#     LOCK.acquire()
#     stop_publish=True
#     LOCK.release()
#     return dns.RemoveAgentResponse()

# rp.Service('RemovePlanner', dns.RemoveAgent, remove_planner_handler)

#Subscribers
# def position_callback(msg):
#     global position
#     LOCK.acquire()
#     position = np.array([msg.x, msg.y])
#     LOCK.release()
# rp.Subscriber(
#     name='position',
#     data_class=gms.Point,
#     callback=position_callback,
#     queue_size=10)

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

# def estimate_callback(msg):
#     global estimate
#     LOCK.acquire()
#     estimate = np.array([msg.x, msg.y])
#     LOCK.release()
# rp.Subscriber(
#     name='estimate',
#     data_class=gms.Point,
#     callback=estimate_callback,
#     queue_size=10)

def est_distance_callback(msg):
    global est_distance
    LOCK.acquire()
    est_distance = msg.point.x
    LOCK.release()
rp.Subscriber(
    name='est_distance',
    data_class=gm.PointStamped,
    callback=est_distance_callback,
    queue_size=10)




#Publishers

#velocity command
cmdvel_pub = rp.Publisher(
    name='cmdvel',
    data_class=gms.Vector,
    queue_size=10)

#bearing angle
beta_pub = rp.Publisher(
    name='beta',
    data_class=gm.PointStamped,
    queue_size=10)

start = False
RATE = rp.Rate(150.0)
STEP=1.0/150.0
#global count
#count=0
go=False


while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [bearing_measurement,est_distance]]) :
        print(node_name+" sees ",agent_names," at the BEGINNNNNNNNNNNNNNNNG")
        start = True
    LOCK.release()
    RATE.sleep()

while not rp.is_shutdown() :
    LOCK.acquire()


    phi_bar=np.array([bearing_measurement[1],-bearing_measurement[0]])
    


    if neighbor is None and not len(agent_names)==0:
        buff_beta=0.0
        agent_beta=1000
        rp.logwarn("????????????????????????????????????????????????????????? per %s",node_name)
        for name in agent_names:
#            rp.logwarn("%s controlla %s",node_name,name)
            while not go:
#                rp.logwarn(agent_bearing_measurement[name])
                if not agent_bearing_measurement[name] is None :
                    rp.wait_for_service('/Topology')
                    topology_proxy=rp.ServiceProxy('/Topology', dns.Topology)
                    topology_proxy.call(node_name)
                    buff_beta=Angle(bearing_measurement,agent_bearing_measurement[name])
#                   rp.logwarn("%s vede con %s questo: %s",node_name,name,buff_beta)
                    if buff_beta<agent_beta:
                        agent_beta=buff_beta
                        neighbor=name
#                    rp.logwarn(agent_bearing_measurement[name])
#                    rp.logwarn(agent_truemeasure[name])
                    go=True
                else:
                    go=False
#                    rp.logwarn("%s got stuck!!!!!!",node_name)
            go=False
        agents_number=len(agent_names)
        rp.logwarn("11111111111111111111111111111111111111111111111111111111111 per %s",node_name)
#        rp.logwarn("Il neighbor di %s e' %s",node_name,neighbor)

    elif not neighbor is None and not agents_number==len(agent_names):
        neighbor=None
        agents_number=len(agent_names)
        rp.logwarn("222222222222222222222222222222222222222222222222222222222222 per %s",node_name)

    elif not neighbor is None and agents_number==len(agent_names):
#        rp.logwarn("333333333333333333333333333333333333333333333333333333333333333333 per %s",node_name)
        if agent_truemeasure[neighbor]:
            agent_beta=Angle(bearing_measurement,agent_bearing_measurement[neighbor])
        else :
            agent_beta=agent_beta+STEP*k_fi*(alpha+2*math.pi/(agents_number+1))


#    rp.logwarn(agent_beta)
#    rp.logwarn("Il neighbor di %s e' %s",node_name,neighbor)
#    rp.logwarn(len(agent_names))
    vel = k_d*bearing_measurement*(est_distance-DESIRED_DISTANCE)+k_fi*est_distance*phi_bar*(alpha+agent_beta)
    #Velocity message:  approach therm could be removed but helps arc trajectories following

#    if count<5:
#        vel=(0.0,0.0)
#        count=count+1

    cmdvel_msg = gms.Vector(x=vel[0], y=vel[1])
    #Beta message
    beta_msg=gm.PointStamped()
    beta_msg.header.seq=0
    beta_msg.header.stamp = rp.Time.now()
    beta_msg.point.x=agent_beta
    beta_msg.point.y=0
    beta_msg.point.z=0
    LOCK.release()
    # Publishing
    cmdvel_pub.publish(cmdvel_msg)
    beta_pub.publish(beta_msg)
    RATE.sleep()
