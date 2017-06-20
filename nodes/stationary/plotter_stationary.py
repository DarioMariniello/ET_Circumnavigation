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

RATE = rp.Rate(3e1)




LOCK = thd.Lock()
XMIN=rp.get_param('xmin')
XMAX=rp.get_param('xmax')
YMIN=rp.get_param('ymin')
YMAX=rp.get_param('ymax')
plt.ion()
#SCALE = 1.035
fig=plt.figure()#figsize=(SCALE*6.4, SCALE*4.8)

plot_extents = XMIN, XMAX, YMIN, YMAX
transform = Affine2D().rotate_deg(90)
helper = floating_axes.GridHelperCurveLinear(transform, plot_extents)
ax = floating_axes.FloatingSubplot(fig, 111, grid_helper=helper)
fig.add_subplot(ax)
plt.scatter(*TARGET_POSITION, color=TARGET_COLOR)
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



def change_topology_handler(req):
    global topology
    LOCK.acquire()
    topology[req.agent] = req.neighbor
    LOCK.release()
    return dns.ChangeTopologyResponse()
rp.Service(name="change_topology", service_class=dns.ChangeTopology, handler=change_topology_handler)


rp.Service('AddAgentArtist', dns.AddAgent, add_agent_artist_handler)


# def remove_agent_artist_handler(req):
#     global agent_names
#     global agent_positions
#     global agent_estimates
#     global agent_artists
#     global estimate_artists
#     global subscribers_position
#     global subscribers_estimate
#     LOCK.acquire()
#     agent_artists[req.name].set_visible(False)
#     estimate_artists[req.name].set_visible(False)
#     agent_names.remove(req.name)
#     del agent_positions[req.name]
#     del agent_estimates[req.name]
#     del agent_artists[req.name]
#     del estimate_artists[req.name]
#     subscribers_position[req.name].unregister()
#     subscribers_estimate[req.name].unregister()
#     LOCK.release()
#     return dns.RemoveAgentResponse()


# rp.Service('RemoveAgentArtist', dns.RemoveAgent, remove_agent_artist_handler)



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
            agent_positions[name] = None
        if not agent_estimates[name] is None:
            est[name] = cp.copy(agent_estimates[name])
            agent_estimates[name] = None
    top = cp.copy(topology)
#    rp.logwarn(topology)
    LOCK.release()
    for name,pos in ag_pos.items():
        if not pos is None:
            if not agent_artists[name] is None:
                for artist in agent_artists[name]:
                    artist.remove()
            agent_artists[name] = ag_pos[name].draw( color=AGENT_COLOR)
    for name,estim in est.items():
        if not estim is None:
            if not estimate_artists[name] is None:
                for artist in estimate_artists[name]:
                    artist.remove()
            estimate_artists[name] = est[name].draw( color=ESTIMATE_COLOR)
    for ag, nbr in top.items():
        if not topology_artists.get(ag, None) is None:
            for artist in topology_artists[ag]:
                artist.remove()
        topology_artists[ag] = (ag_pos[nbr]-ag_pos[ag]).saturate(0.15).draw(x0=ag_pos[ag].x, y0=ag_pos[ag].y, color="green", alpha=0.3)
    plt.draw()
    RATE.sleep()
