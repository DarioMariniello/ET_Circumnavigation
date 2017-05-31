#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import math as mt
import copy as cp
import et_circumnavigation.srv as dns
import geometry_msgs.msg as gm

# Parameters
RADIUS = rp.get_param('communication_radius')
LOCK = thd.Lock()


rp.init_node('neighbor_designer')



# Subscribers to the bearing vectors of the other agents
agent_names=[]
agent_bearing_measurement = {}
agent_truemeasure = {}
agent_last_beta = {}
agent_position = {}
def agent_bearing_callback(msg, name):
    global agent_bearing_measurement
    global agent_truemeasure
    global agent_last_beta
    LOCK.acquire()
    agent_bearing_measurement[name] = np.array([msg.bearing_measurement.x, msg.bearing_measurement.y])
    agent_truemeasure[name]=msg.truemeasure
    agent_last_beta[name]=msg.last_beta
    LOCK.release()

def agent_position_callback(msg, name):
    global agent_position
    LOCK.acquire()
    agent_position[name] = np.array([msg.x, msg.y])
    LOCK.release()


bearing_measurement_subscribers={}
position_subscribers={}
# Handler for the service "AddAgent": the new agent call "AddAgent" and the others add its name in agent_name and subscribe to its fi
def add_agent_handler(req):
    global agent_names
    global agent_bearing_measurement
    global bearing_measurement_subscribers
    bearing_measurement_subscribers[req.name]=rp.Subscriber(
        name='/'+req.name+'/bearing_measurement',
        data_class=bms.Bearing,
        callback=agent_bearing_callback,
        callback_args=req.name)
    position_subscribers[req.name]=rp.Subscriber(
        name='/'+req.name+'/position',
        data_class=gms.Point,
        callback=agent_position_callback,
        callback_args=req.name)
    LOCK.acquire()
    agent_names.append(req.name)
    agent_bearing_measurement[req.name]=None
    agent_position[req.name]=None
    LOCK.release()
    return dns.AddAgentResponse()

rp.Service('AddAgent', dns.AddAgent, add_agent_handler)


# Call to the service "AddMe": the agent requires to the cloud to add his name
rp.wait_for_service('/AddMe')
add_me_proxy=rp.ServiceProxy('/AddMe', dns.AddAgent)
add_me_proxy.call(node_name)



rp.wait_for_service('Neighbor')
neighbor_proxy=rp.ServiceProxy('Neighbor', dns.Neighbor)


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
    global truemeasure
    LOCK.acquire()
    bearing_measurement = np.array([msg.bearing_measurement.x, msg.bearing_measurement.y])
    truemeasure=msg.truemeasure
    LOCK.release()
rp.Subscriber(
    name='bearing_measurement',
    data_class=bms.Bearing,
    callback=bearing_measurement_callback,
    queue_size=10)




start = False
RATE = rp.Rate(150.0)
STEP=1.0/150.0

local_agent_names=[]
local_agent_bearing={}
theta={}
current_x={}
current_y={}
local_agent_last_beta={}

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [ bearing_measurement]]):
        start = True
    LOCK.release()
    RATE.sleep()

while not rp.is_shutdown() :
    LOCK.acquire()

    for name in agent_names:
        if not agent_position[name] is None and not name in local_agent_names:
            if np.linalg.norm(position-agent_position[name])<c_radius:
                local_agent_names.append(name)
                current_x[name]=0.0
                current_y[name]=0.0
                local_agent_bearing[name]=None
                local_agent_last_beta[name]=None
            elif name in local_agent_names:
                current_x.remove(name)
                current_y.remove(name)
                local_agent_bearing.remove(name)
                local_agent_last_beta.remove(name)
                local_agent_names.remove(name)

#    if node_name=="agent2":
#        rp.logwarn(local_agent_names)



    if not len(local_agent_names)==0:
        buff_beta=0.0
        agent_beta=1000
        agents_number=len(local_agent_names)

        for name in local_agent_names:
            if all([not data is None for data in [agent_bearing_measurement[name], agent_truemeasure[name],agent_last_beta[name]]]):
                if agent_truemeasure[name]:
                    theta[name]=0.0
                    local_agent_bearing[name]=agent_bearing_measurement[name]
                    local_agent_last_beta[name]=agent_last_beta[name]
                    current_x[name]=local_agent_bearing[name][0]
                    current_y[name]=local_agent_bearing[name][1]
                    buff_beta=Angle(bearing_measurement,local_agent_bearing[name])

                else :
                    if current_y[name]!=0.0 and current_x[name]!=0.0 and local_agent_last_beta[name]>0:
                        theta[name]=theta[name]+STEP*k_fi*DESIRED_DISTANCE*(alpha+local_agent_last_beta[name])#2*mt.pi/(agents_number+1)
                        local_agent_bearing[name][0]=current_x[name]*mt.cos(theta[name])-current_y[name]*mt.sin(theta[name])
                        local_agent_bearing[name][1]=current_y[name]*mt.cos(theta[name])+current_x[name]*mt.sin(theta[name])
                        buff_beta=Angle(bearing_measurement,local_agent_bearing[name])

#                if node_name=="agent2":
#                    rp.logwarn(buff_beta)

                if buff_beta<agent_beta and buff_beta!=0.0 :
                    local_neighbor=name
                    agent_beta=buff_beta

        if neighbor!=local_neighbor:
            neighbor=local_neighbor
            neighbor_proxy.call(neighbor,agents_number)

#                    rp.logwarn(agent_bearing_measurement[name])
#                    rp.logwarn(agent_truemeasure[name])
#                    rp.logwarn("%s got stuck!!!!!!",node_name)
#        rp.logwarn("11111111111111111111111111111111111111111111111111111111111 per %s",node_name)
 #       rp.logwarn("Il neighbor di %s e' %s",node_name,neighbor)

    LOCK.release()
    RATE.sleep()
