#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import et_circumnavigation.msg as bms
import threading as thd
import numpy as np
import math as mt
import copy as cp
import et_circumnavigation.srv as dns
import geometry_msgs.msg as gm

# Parameters
DESIRED_DISTANCE = rp.get_param('desired_distance')
alpha= rp.get_param('alpha')
k_fi= rp.get_param('k_fi')
k_d= rp.get_param('k_d')

node_name=rp.get_param('node_name')
delay=rp.get_param('delay')

# Variables
est_distance=0.0
bearing_measurement = None




agents_number=0

neighbor=""
neighbor_bearing_measurement=None
neighbor_truemeasure=None
neighbor_last_beta=None
local_neighbor_bearing=np.array([0.0,0.0])

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
    beta=mt.atan2(sin_beta,cos_beta)
    if beta<0:
        beta=beta+2*mt.pi
    return beta 





# Subscribers to the bearing vectors of the other agents
agent_names=[]
agent_bearing_measurement = {}
agent_truemeasure = {}
agent_last_beta = {}

def neighbor_callback(msg):
    global neighbor_bearing_measurement
    global neighbor_truemeasure
    global neighbor_last_beta
    LOCK.acquire()
    neighbor_bearing_measurement = np.array([msg.bearing_measurement.x, msg.bearing_measurement.y])
    neighbor_truemeasure=msg.truemeasure
    neighbor_last_beta=msg.last_beta
    LOCK.release()
neighbor_bearing_measurement_subscriber=rp.Subscriber(
    name='/'+neighbor+'/bearing_measurement',
    data_class=bms.Bearing,
    callback=neighbor_callback,
    queue_size=1)

def neighbor_handler(req):
    global neighbor
    global neighbor_bearing_measurement_subscriber
    global agents_number
    LOCK.acquire()    
    neighbor_bearing_measurement_subscriber.unregister()
    neighbor=req.name
    agents_number=req.number_of_agents
    theta=0.0
    LOCK.release()    
    neighbor_bearing_measurement_subscriber=rp.Subscriber(
        name='/'+neighbor+'/bearing_measurement',
        data_class=bms.Bearing,
        callback=neighbor_callback,
        queue_size=1)
    # LOCK.acquire()
    # neighbor_bearing_measurement[req.name]=None
    # LOCK.release()
    return dns.NeighborResponse()

rp.Service('Neighbor', dns.Neighbor, neighbor_handler)



def last_beta_handler(req):
    global agent_beta
    return dns.LastBetaResponse(agent_beta)

rp.Service('LastBeta', dns.LastBeta, last_beta_handler)



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


start = False
RATE = rp.Rate(150.0)
STEP=1.0/150.0
theta=0.0
current_x=0.0
current_y=0.0



while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [est_distance, bearing_measurement]]):
        start = True
    LOCK.release()
    RATE.sleep()

while not rp.is_shutdown() :
    LOCK.acquire()



    phi_bar=np.array([bearing_measurement[1],-bearing_measurement[0]])
    

    
    if not neighbor=="":
#        rp.logwarn("%s planner vede %s",node_name,neighbor)
        if all([not data is None for data in [neighbor_bearing_measurement, neighbor_truemeasure, neighbor_last_beta]]):
            if neighbor_truemeasure:
                theta=0.0
                local_neighbor_bearing=neighbor_bearing_measurement
                current_x=local_neighbor_bearing[0]
                current_y=local_neighbor_bearing[1]
                agent_beta=Angle(bearing_measurement,local_neighbor_bearing)
#                if node_name=="agent2":
#                    rp.logwarn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  x=%s   y=%s",current_x,current_y)
#                    rp.logwarn(mt.cos(theta))
#                    rp.logwarn(mt.sin(theta))
            else :
                if current_y!=0.0 and current_x!=0.0 and neighbor_last_beta>0:
#                    if node_name=="agent2":
#                        rp.logwarn("????????????????????????????????????????????????????????  x=%s   y=%s",current_x,current_y)
                    theta=theta+STEP*k_fi*DESIRED_DISTANCE*(alpha+neighbor_last_beta) ########2*mt.pi/(agents_number+1)
                    local_neighbor_bearing[0]=current_x*mt.cos(theta)-current_y*mt.sin(theta)
                    local_neighbor_bearing[1]=current_y*mt.cos(theta)+current_x*mt.sin(theta)
                    agent_beta=Angle(bearing_measurement,local_neighbor_bearing)
#                else:
#                    if node_name=="agent2":
#                        rp.logwarn("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        else:
            agent_beta=0.0

#    if node_name=="agent2":
#        rp.logwarn(agent_beta)
#        rp.logwarn(agents_number)
#        rp.logwarn("%s neighbor is %s",node_name,neighbor)


    vel = k_d*bearing_measurement*(est_distance-DESIRED_DISTANCE)+k_fi*est_distance*phi_bar*(alpha+agent_beta)


    cmdvel_msg = gms.Vector(x=vel[0], y=vel[1])

    # #Beta message
    # beta_msg=gm.PointStamped()
    # beta_msg.header.seq=0
    # beta_msg.header.stamp = rp.Time.now()
    # beta_msg.point.x=agent_beta
    # beta_msg.point.y=0
    # beta_msg.point.z=0
    LOCK.release()
    # Publishing
    cmdvel_pub.publish(cmdvel_msg)
    RATE.sleep()
