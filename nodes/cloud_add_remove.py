#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as smsp
import threading as thd
import numpy as np
import math
import copy as cp
import ET_circumnavigation.srv as dns

agent_names=[]
add_agent_proxies={}
remove_agent_proxies={}
remove_sensor_proxies={}
remove_estimate_proxies={}
remove_vehicle_proxies={}
remove_planner_proxies={}

LOCK=thd.Lock()

rp.wait_for_service('AddAgentArtist')
plotter_proxy=rp.ServiceProxy('AddAgentArtist',dns.AddAgent)
#rp.wait_for_service('RemoveAgentArtist')
#plotter_proxy_remove=rp.ServiceProxy('RemoveAgentArtist',dns.RemoveAgent)
# Handle for the service "AddMe": when an agent calls this service, the Cloud add a proxy associated to its name to the dictionary 
# add_agent_proxies, calls the service "AddAgent" for all the agents in the list agent_name, to notify them of the enter ot the new one,
# and calls the service "AddAgent" to the new one, to notify it of the existence of the others in the network.
# Furthermore, the Cloud notifies the plotter for the enter of the new agent.
def add_me_handler(req):
    LOCK.acquire()
    add_agent_proxies[req.name]=rp.ServiceProxy(req.name+'/AddAgent',dns.AddAgent)     
    # remove_agent_proxies[req.name]=rp.ServiceProxy(req.name+'/RemoveAgent',dns.RemoveAgent)
    # remove_sensor_proxies[req.name]=rp.ServiceProxy(req.name+'/RemoveSensor',dns.RemoveAgent)
    # remove_estimate_proxies[req.name]=rp.ServiceProxy(req.name+'/RemoveEstimate',dns.RemoveAgent)
    # remove_vehicle_proxies[req.name]=rp.ServiceProxy(req.name+'/RemoveVehicle',dns.RemoveAgent)
    # remove_planner_proxies[req.name]=rp.ServiceProxy(req.name+'/RemovePlanner',dns.RemoveAgent)
    for name in agent_names:
        add_agent_proxies[name].call(req.name)
        add_agent_proxies[req.name].call(name)
    agent_names.append(req.name)
    LOCK.release()
    plotter_proxy.call(req.name)
    return dns.AddAgentResponse()   

rp.Service('AddMe',dns.AddAgent,add_me_handler)

# Handle for the service "Remove"
# def remove_handler(req):
#     LOCK.acquire()
#     agent_names.remove(req.name)
#     del add_agent_proxies[req.name]
#     for name in agent_names:
#         remove_agent_proxies[name].call(req.name)
#     plotter_proxy_remove.call(req.name)
#     del remove_agent_proxies[req.name]
#     remove_planner_proxies[req.name].call(req.name)
#     remove_sensor_proxies[req.name].call(req.name)
#     remove_estimate_proxies[req.name].call(req.name)
#     remove_vehicle_proxies[req.name].call(req.name)
#     del remove_planner_proxies[req.name]
#     del remove_sensor_proxies[req.name]
#     del remove_vehicle_proxies[req.name]
#     del remove_estimate_proxies[req.name]
#     LOCK.release()
#     return dns.RemoveAgentResponse()   

# rp.Service('Remove',dns.RemoveAgent,remove_handler)

rp.init_node('cloud_add_remove')
rp.spin()
