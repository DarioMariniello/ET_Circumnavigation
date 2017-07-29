#! /usr/bin/python

import rospy as rp
import numpy as np
import geomtwo.msg as gms
import geomtwo.impl as gmi
import geometry_msgs.msg as gm
import threading as thd
import et_circumnavigation.srv as dns
import math as m
import std_msgs.msg

agent_names = list()
neighbor_bearing_proxies = dict()
#neighbor_beta_proxies = dict()
position_subscribers = dict()
estimate_subscribers = dict()
repulsion_publishers = dict()
beta_error_publishers = dict()
beta_tilde_publishers = dict()
agents_proxies = dict()

bearings = dict()
beta_errors=dict()
beta_tilde= dict()
total_error=0.0
neigh=None
betas = dict()
topology = dict()
positions = dict()
estimates = dict()
repulsions= dict()
ag_dist= dict()
remove_sensor_proxies={}
remove_vehicle_proxies={}
remove_planner_proxies={}

rp.init_node("cloud")

LOCK = thd.Lock()
FREQUENCY = 70.0
RATE = rp.Rate(FREQUENCY)
COMM_RADIUS = rp.get_param("communication_radius")
COLL_RADIUS = rp.get_param("collision_radius")
ALPHA = rp.get_param("alpha")
K_PHI = rp.get_param("k_fi")
K_OBST = rp.get_param("k_obst")
STEP = 1.0/FREQUENCY


rp.wait_for_service('AddAgentArtist')
plotter_proxy=rp.ServiceProxy('AddAgentArtist',dns.AddAgent)

rp.wait_for_service('RemoveAgentArtist')
plotter_proxy_remove=rp.ServiceProxy('RemoveAgentArtist',dns.RemoveAgent)

def position_callback(req, name):
    global positions
    LOCK.acquire()
    positions[name] = gmi.Point(req)
    LOCK.release()

def estimate_callback(req, name):
    global estimates
    LOCK.acquire()
    estimates[name] = gmi.Point(req)
    LOCK.release()

def add_me_handler(req):
    LOCK.acquire()
    agent_names.append(req.name)
    neighbor_bearing_proxies[req.name] = rp.ServiceProxy(req.name+"/neighbor_bearing", dns.NeighborBearing)
    remove_sensor_proxies[req.name]=rp.ServiceProxy(req.name+'/RemoveSensor',dns.RemoveAgent)
    remove_vehicle_proxies[req.name]=rp.ServiceProxy(req.name+'/RemoveVehicle',dns.RemoveAgent)
    remove_planner_proxies[req.name]=rp.ServiceProxy(req.name+'/RemovePlanner',dns.RemoveAgent)

    position_subscribers[req.name] = rp.Subscriber(req.name+"/position", gms.Point, position_callback, callback_args=req.name)
    estimate_subscribers[req.name] = rp.Subscriber(req.name+"/estimate", gms.Point, estimate_callback, callback_args=req.name)
    repulsion_publishers[req.name] = rp.Publisher(
    name=req.name+"/repulsion",
    data_class=gms.Vector,
    queue_size=10)
    beta_error_publishers[req.name] = rp.Publisher(
    name=req.name+"/beta_error",
    data_class=gm.PointStamped,
    queue_size=10)
    beta_tilde_publishers[req.name] = rp.Publisher(
    name=req.name+"/beta_tilde",
    data_class=gm.PointStamped,
    queue_size=10)
    LOCK.release()
    plotter_proxy.call(req.name)
    return dns.AddAgentResponse()
rp.Service('add_me', dns.AddAgent, add_me_handler)



def remove_handler(req):
    LOCK.acquire()
    agent_names.remove(req.name)
    plotter_proxy_remove.call(req.name)
    

    remove_planner_proxies[req.name].call(req.name)
    del remove_planner_proxies[req.name]
    del neighbor_bearing_proxies[req.name]
    position_subscribers[req.name].unregister()
    del position_subscribers[req.name]
    estimate_subscribers[req.name].unregister()
    del estimate_subscribers[req.name]
    repulsion_publishers[req.name].unregister()
    del repulsion_publishers[req.name]
    beta_error_publishers[req.name].unregister()
    del beta_error_publishers[req.name]
    beta_tilde_publishers[req.name].unregister()
    del beta_tilde_publishers[req.name]

    for key, value in topology.items():
        if key==req.name or value==req.name:#if the key is name or the value is name , delete key:value
            del topology[key]



    remove_sensor_proxies[req.name].call(req.name)
    del remove_sensor_proxies[req.name]
    remove_vehicle_proxies[req.name].call(req.name)
    del remove_vehicle_proxies[req.name]
    rp.logwarn("unregistered")
    LOCK.release()
    return dns.RemoveAgentResponse() 
rp.Service('Remove',dns.RemoveAgent,remove_handler)



def control_handler(req):
    LOCK.acquire()
    for name in agent_names:
            agents_proxies[name] = rp.ServiceProxy(name+"/Circum", dns.Circum)
            try: agents_proxies[name].call()
            except: rp.logwarn("Error in service call Circum")
    LOCK.release()
    return dns.ControlResponse()
rp.Service('Control', dns.Control, control_handler)




def share_bearing_handler(req):
    LOCK.acquire()
    bearings[req.name] = gmi.Versor(req.bearing) #NO MORE NEEDED because i'm building the bearings locally as bearings[name] = gmi.Versor(estimates[name]-positions[name])
#    rp.logwarn(bearings[req.name])
    # if req.name=="Crazyflie1":
    #     rp.logwarn("cf1 is doing new measurement!")
    #     rp.logwarn(bearings[req.name])
    # if req.name=="agent1":
    #     rp.logwarn("cf1 is doing new measurement!")
    #     rp.logwarn(bearings[req.name])
    for name in topology:
        if topology[name] == req.name:
            try: neighbor_bearing_proxies[name].call(req.bearing)
            except: rp.logwarn("Error in service call")
    LOCK.release()
    return dns.ShareBearingResponse()
rp.Service("share_bearing", dns.ShareBearing, share_bearing_handler)

def share_beta_handler(req):
    LOCK.acquire()
    betas[req.name] = req.beta
#    rp.logwarn(req.beta)
#    for name in topology:
#        if topology[name] == req.name:
#            try: change_topology_proxy.call(name, req.name)
#            except: rp.logwarn("Something wrong with change topology")
#            try: neighbor_beta_proxies[name].call(req.beta)
#            except: rp.logwarn("Error in service call")
    LOCK.release()
    return dns.ShareBetaResponse()
rp.Service("share_beta", dns.ShareBeta, share_beta_handler)


err = rp.Publisher('total_error', std_msgs.msg.Float32, queue_size=10)

rp.wait_for_service("change_topology")
change_topology_proxy = rp.ServiceProxy("change_topology", dns.ChangeTopology)



while not rp.is_shutdown():
    LOCK.acquire()
    total_error=0.0
    for name in agent_names:
        if name in positions and name in bearings:
            neighbor = None
            pos = positions[name]
            brg = bearings[name]
            smallest_angle = 2*m.pi
            for other in agent_names:
                if other != name and other in positions and other in bearings and (pos-positions[other]).norm < COMM_RADIUS:
                    angle = brg.angle_to(bearings[other], force_positive=True)
                    if angle < smallest_angle:
                        smallest_angle = angle
                        neighbor = other
            if neighbor != topology.get(name, None):
                topology[name] = neighbor
                try: change_topology_proxy.call(name, neighbor)
                except: rp.logwarn("something went wrong with ChangeTopology")
            if  name in estimates:
                bearings[name] = gmi.Versor(estimates[name]-positions[name])



#############error 
        if name in bearings:
            if name in betas:
                neigh=topology[name]    
                beta_errors[name]=gm.PointStamped()
                beta_errors[name].header.seq=0
                beta_errors[name].header.stamp = rp.Time.now()
                beta_errors[name].point.x=(betas[name]-(bearings[name].angle_to(bearings[neigh], force_positive=False)))*180.0/m.pi
                beta_errors[name].point.y=0
                beta_errors[name].point.z=0

                beta_tilde[name]=gm.PointStamped()
                beta_tilde[name].header.seq=0
                beta_tilde[name].header.stamp = rp.Time.now()
                beta_tilde[name].point.x=(bearings[name].angle_to(bearings[neigh], force_positive=True))-2*m.pi/len(agent_names)*180.0/m.pi
                beta_tilde[name].point.y=0
                beta_tilde[name].point.z=0
#                for error in beta_errors:
#                     total_error+=beta_errors[error]
#                     total_error= ( total_error + 180.0) % (2 * 180.0 ) - 180.0
            else:  
                beta_errors[name]=gm.PointStamped()
                beta_errors[name].header.seq=0
                beta_errors[name].header.stamp = rp.Time.now()
                beta_errors[name].point.x=0
                beta_errors[name].point.y=0
                beta_errors[name].point.z=0

                beta_tilde[name]=gm.PointStamped()
                beta_tilde[name].header.seq=0
                beta_tilde[name].header.stamp = rp.Time.now()
                beta_tilde[name].point.x=0
                beta_tilde[name].point.y=0
                beta_tilde[name].point.z=0
#           beta_errors[name]=0.0
        else: 
            beta_errors[name]=gm.PointStamped()
            beta_errors[name].header.seq=0
            beta_errors[name].header.stamp = rp.Time.now()
            beta_errors[name].point.x=0
            beta_errors[name].point.y=0
            beta_errors[name].point.z=0
            
            beta_tilde[name]=gm.PointStamped()
            beta_tilde[name].header.seq=0
            beta_tilde[name].header.stamp = rp.Time.now()
            beta_tilde[name].point.x=0
            beta_tilde[name].point.y=0
            beta_tilde[name].point.z=0
#########



#################collision avoidance check only communication circle
        if name in positions and name in repulsions :
            pos = positions[name]
            repulsions[name]=gmi.Vector(0.0,0.0)
            for other in agent_names:
                if other != name and other in positions :
                    if (pos-positions[other]).norm<= (COLL_RADIUS) :
                        ag_dist[other]=pos-positions[other]
                        repulsions[name]+=K_OBST*(1/((ag_dist[other].norm)**3))*(ag_dist[other])
        else :
            repulsions[name]=gmi.Vector(0.0,0.0)
#############################################

#    rp.logwarn(topology)
    LOCK.release()
    for name in repulsion_publishers.keys():
        repulsion_publishers[name].publish(repulsions[name].serialize())
    for name in beta_error_publishers.keys():
        beta_error_publishers[name].publish(beta_errors[name])
    for name in beta_tilde_publishers.keys():
        beta_tilde_publishers[name].publish(beta_tilde[name])
    err.publish(total_error)
    RATE.sleep()
