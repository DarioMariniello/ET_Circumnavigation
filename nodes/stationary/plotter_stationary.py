#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import geomtwo.impl as gmi

import threading as thd
import copy as cp

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import mpl_toolkits.axisartist.floating_axes as floating_axes
import et_circumnavigation.srv as dns
rp.init_node('plotter')



AGENT_COLOR = rp.get_param('agent_color','blue')
ESTIMATE_COLOR = rp.get_param('estimate_color','red')
TARGET_COLOR = rp.get_param('target_color','black')

#AGENT_NAMES = rp.get_param('agent_names').split()

TARGET_POSITION = rp.get_param('target_position')
DESIRED_DISTANCE = rp.get_param('desired_distance')

RATE = rp.Rate(4e1)




LOCK = thd.Lock()
XMIN=rp.get_param('xmin')
XMAX=rp.get_param('xmax')
YMIN=rp.get_param('ymin')
YMAX=rp.get_param('ymax')
plt.ion()
SCALE = 1.32#1.035
fig=plt.figure(figsize=(SCALE*6.4, SCALE*4.8))

plot_extents = XMIN, XMAX, YMIN, YMAX
transform = Affine2D().rotate_deg(90)
helper = floating_axes.GridHelperCurveLinear(transform, plot_extents)
ax = floating_axes.FloatingSubplot(fig, 111, grid_helper=helper)
fig.add_subplot(ax)
plt.scatter(*TARGET_POSITION, color=TARGET_COLOR)
circle=plt.Circle(TARGET_POSITION, DESIRED_DISTANCE+0.02, color='r', fill=False, alpha=1)
ax.add_artist(circle)
plt.axis('equal')
plt.xlim((XMIN,XMAX))
plt.ylim((YMIN,YMAX))
plt.grid(True)
plt.draw()


agent_names=[]

agent_positions={}
agent_artists={}

agent_estimates={}
estimate_artists={}

subscribers_position={}
subscribers_estimate={}

topology = {}
topology_artists = {}

# Service Handlers
def add_agent_artist_handler(req):
    global agent_names
    global agent_positions
    global agent_estimates
    global agent_artists
    global estimate_artists
    global subscribers_position
    global subscribers_estimate
    subscribers_position[req.name]=rp.Subscriber(
        name='/'+req.name+'/position',
        data_class=gms.Point,
        callback=agent_callback,
        callback_args=req.name,
        queue_size=1)
    subscribers_estimate[req.name]=rp.Subscriber(
        name='/'+req.name+'/estimate',
        data_class=gms.Point,
        callback=estimate_callback,
        callback_args=req.name,
        queue_size=1)
    LOCK.acquire()
    agent_names.append(req.name)
    agent_positions[req.name]=None
    agent_estimates[req.name]=None
    agent_artists[req.name]=None
    estimate_artists[req.name]=None
    LOCK.release()
    return dns.AddAgentResponse()
rp.Service('AddAgentArtist', dns.AddAgent, add_agent_artist_handler)

def remove_agent_artist_handler(req):
    global agent_names
    global agent_positions
    global ag_pos
    global agent_estimates
    global est
    global agent_artists
    global estimate_artists
    global topology_artists
    global topology
    global subscribers_position
    global subscribers_estimate
    LOCK.acquire()
    if not agent_artists[req.name] is None:
        for artist in agent_artists[req.name]:
            artist.remove()
        del agent_artists[req.name]
    if not estimate_artists[req.name] is None:
        for artist in estimate_artists[req.name]:
            artist.remove()
        del estimate_artists[req.name]
    if not topology_artists.get(req.name, None) is None: #this will give you none both if there is no name in topology and if his value is none
        for artist in topology_artists[req.name]:
            artist.remove()
        del topology_artists[req.name]
    for key, value in topology.items():
        if key==req.name or value==req.name:#if the key is name or the value is name , delete key:value
            del topology[key]
    agent_names.remove(req.name)
    del agent_positions[req.name]
    del ag_pos[req.name]
    del agent_estimates[req.name]
    del est[req.name]
    subscribers_position[req.name].unregister()
    del subscribers_position[req.name]
    subscribers_estimate[req.name].unregister()
    del subscribers_estimate[req.name]
    LOCK.release()
    return dns.RemoveAgentResponse()
rp.Service('RemoveAgentArtist', dns.RemoveAgent, remove_agent_artist_handler)

def change_topology_handler(req):
    global topology
    LOCK.acquire()
    if not req.neighbor=="":
        topology[req.agent] = req.neighbor
    elif req.neighbor=="" :
        del topology[req.agent]
#    rp.logwarn(req.neighbor)
    LOCK.release()
    return dns.ChangeTopologyResponse()
rp.Service(name="change_topology", service_class=dns.ChangeTopology, handler=change_topology_handler)


# Subscribers
def agent_callback(msg, name):
    global agent_positions
    LOCK.acquire()
    agent_positions[name] = gmi.Point(msg)
    LOCK.release()

def estimate_callback(msg, name):
    global agent_estimates
    LOCK.acquire()
    agent_estimates[name] = gmi.Point(msg)
    LOCK.release()


# Main
while not rp.is_shutdown():
    ag_pos = {}
    est = {}
    LOCK.acquire()
    
    for name in agent_names:
#        rp.logwarn(name)
        if not agent_positions[name] is None:
            ag_pos[name] = cp.copy(agent_positions[name])
#            agent_positions[name] = None
        if not agent_estimates[name] is None:
            est[name] = cp.copy(agent_estimates[name])
            agent_estimates[name] = None
    top = cp.copy(topology)
    LOCK.release()
    for name,pos in ag_pos.items():
        if not pos is None:
            # if not agent_artists[name] is None:
            #     for artist in agent_artists[name]:
            #         artist.remove()
            agent_artists[name] = ag_pos[name].draw( color=AGENT_COLOR,alpha=0.4)
    for name,estim in est.items():
        if not estim is None:
            if not estimate_artists[name] is None:
                for artist in estimate_artists[name]:
                    artist.remove()
            estimate_artists[name] = est[name].draw( color=ESTIMATE_COLOR)
    for ag, nbr in top.items():
        if not topology_artists.get(ag, None) is None:
            for artist in topology_artists[ag] :
                artist.remove()
    #dnbr can be empty because when an agents gets a neighbor and at a certain point he doesn't have any, cloud will call change_topology with None as neighbor. So, the service stores empty in 
    #the name [neighbor]. Actually, the check on nbr is just a workaround: indeed the artist stays fixed there(the green arrow remains sill "floating") ; the proper solution should be remove 
    #the item from the topology dictionary, but then one could have different topology in the cloud node...but i don't know
        if not nbr is None:
            topology_artists[ag] = (ag_pos[nbr]-ag_pos[ag]).saturate(0.15).draw(x0=ag_pos[ag].x, y0=ag_pos[ag].y, color="green", alpha=1)
    plt.draw()
    RATE.sleep()
