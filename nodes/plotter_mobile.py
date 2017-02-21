#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms

import threading as thd
import copy as cp

import numpy as np
import matplotlib.pyplot as plt

import circumnavigation_moving_target.srv as dns
rp.init_node('plotter')



AGENT_COLOR = rp.get_param('agent_color','blue')
ESTIMATE_COLOR = rp.get_param('estimate_color','red')
TARGET_COLOR = rp.get_param('target_color','black')

#AGENT_NAMES = rp.get_param('agent_names').split()
agent_names=[]
target_position = rp.get_param('target_initial_position')

RATE = rp.Rate(8e1)

LOCK = thd.Lock()
XMIN=rp.get_param('xmin')
XMAX=rp.get_param('xmax')
YMIN=rp.get_param('ymin')
YMAX=rp.get_param('ymax')
plt.ion()
plt.figure()
#plt.scatter(*TARGET_POSITION, color=TARGET_COLOR)
plt.axis('equal')
plt.xlim((XMIN,XMAX))
plt.ylim((YMIN,YMAX))
plt.grid(True)
plt.draw()

agent_positions={}
agent_artists={}

agent_estimates={}
estimate_artists={}

subscribers_position={}
subscribers_estimate={}

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
    global agent_estimates
    global agent_artists
    global estimate_artists
    global subscribers_position
    global subscribers_estimate
    LOCK.acquire()
    agent_artists[req.name].set_visible(False)
    estimate_artists[req.name].set_visible(False)
    agent_names.remove(req.name)
    del agent_positions[req.name]
    del agent_estimates[req.name]
    del agent_artists[req.name]
    del estimate_artists[req.name]
    subscribers_position[req.name].unregister()
    subscribers_estimate[req.name].unregister()
    LOCK.release()
    return dns.RemoveAgentResponse()   


rp.Service('RemoveAgentArtist', dns.RemoveAgent, remove_agent_artist_handler)



# Subscribers
def agent_callback(msg, name):
    global agent_positions
    LOCK.acquire()
    agent_positions[name] = [msg.x, msg.y]
    LOCK.release()

def estimate_callback(msg, name):
    global agent_estimates
    LOCK.acquire()
    agent_estimates[name] = [msg.x, msg.y]
    LOCK.release()

def target_position_callback(msg):
    global target_position
    LOCK.acquire()
    target_position=np.array([msg.x,msg.y])
    LOCK.release()
rp.Subscriber(
    name='target_position',
    data_class=gms.Point,
    callback=target_position_callback,
    queue_size=10)


# Main
while not rp.is_shutdown():
    ag_pos = {}
    est = {}
    LOCK.acquire()
    for name in agent_names:
        if not agent_positions[name] is None:
            ag_pos[name] = cp.copy(agent_positions[name])
            agent_positions[name] = None
        if not agent_estimates[name] is None:
            est[name] = cp.copy(agent_estimates[name])
            agent_estimates[name] = None
    LOCK.release()
    for name,pos in ag_pos.items():
        if not pos is None:
            if not agent_artists[name] is None:
                agent_artists[name].remove()
            agent_artists[name] = plt.scatter(*ag_pos[name], color=AGENT_COLOR)
    for name,estim in est.items():
        if not estim is None:
            if not estimate_artists[name] is None:
                estimate_artists[name].remove()
            estimate_artists[name] = plt.scatter(*est[name], color=ESTIMATE_COLOR)
    LOCK.acquire()
    plt.scatter(*target_position, s=10, color=TARGET_COLOR)
    LOCK.release()
    plt.draw()
    RATE.sleep()
