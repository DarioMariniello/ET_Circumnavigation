#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import geomtwo.impl as gmi
import threading as thd
import et_circumnavigation.srv as dns
import math as m

agent_names = list()
neighbor_bearing_proxies = dict()
neighbor_beta_proxies = dict()
position_subscribers = dict()

bearings = dict()
betas = dict()
topology = dict()
positions = dict()
# remove_agent_proxies={}
# remove_sensor_proxies={}
# remove_estimate_proxies={}
# remove_vehicle_proxies={}
# remove_planner_proxies={}

rp.init_node("cloud")

LOCK = thd.Lock()
FREQUENCY = 60.0
RATE = rp.Rate(FREQUENCY)
RADIUS = 1.0
ALPHA = rp.get_param("alpha")
K_PHI = rp.get_param("k_fi")
STEP = 1.0/FREQUENCY


rp.wait_for_service('AddAgentArtist')
plotter_proxy=rp.ServiceProxy('AddAgentArtist',dns.AddAgent)

def position_callback(req, name):
    global positions
    LOCK.acquire()
    positions[name] = gmi.Point(req)
    LOCK.release()

def add_me_handler(req):
    LOCK.acquire()
    agent_names.append(req.name)
    neighbor_bearing_proxies[req.name] = rp.ServiceProxy(req.name+"/neighbor_bearing", dns.NeighborBearing)
    neighbor_beta_proxies[req.name] = rp.ServiceProxy(req.name+"/neighbor_beta", dns.NeighborBeta)
    position_subscribers[req.name] = rp.Subscriber(req.name+"/position", gms.Point, position_callback, callback_args=req.name)
    LOCK.release()
    plotter_proxy.call(req.name)
    return dns.AddAgentResponse()
rp.Service('add_me', dns.AddAgent, add_me_handler)





def share_bearing_handler(req):
    LOCK.acquire()
    bearings[req.name] = gmi.Versor(req.bearing)
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
    for name in topology:
        if topology[name] == req.name:
            try: change_topology_proxy.call(name, req.name)
            except: rp.logwarn("Something wrong with change topology")
            try: neighbor_beta_proxies[name].call(req.beta)
            except: rp.logwarn("Error in service call")
    LOCK.release()
    return dns.ShareBetaResponse()
rp.Service("share_beta", dns.ShareBeta, share_beta_handler)



rp.wait_for_service("change_topology")
change_topology_proxy = rp.ServiceProxy("change_topology", dns.ChangeTopology)




while not rp.is_shutdown():
    LOCK.acquire()
    for name in agent_names:
        if name in positions and name in bearings:
            neighbor = None
            pos = positions[name]
            brg = bearings[name]
            smallest_angle = 2*m.pi
            for other in agent_names:
                if other != name and other in positions and other in bearings and (pos-positions[other]).norm < RADIUS:
                    angle = brg.angle_to(bearings[other], force_positive=True)
                    if angle < smallest_angle:
                        smallest_angle = angle
                        neighbor = other
            if neighbor != topology.get(name, None):
                topology[name] = neighbor
                #try: change_topology_proxy.call(name, neighbor)
                #except: rp.logwarn("something went wrong with ChangeTopology")
        if name in bearings:
            bearings[name] = bearings[name].rotate(STEP*K_PHI*ALPHA)
            if name in betas:
                bearings[name] = bearings[name].rotate(STEP*K_PHI*betas[name])
    #rp.logwarn(topology)
    LOCK.release()
    RATE.sleep()





rp.init_node('cloud_add_remove')
rp.spin()
