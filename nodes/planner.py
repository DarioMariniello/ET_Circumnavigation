#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import geomtwo.impl as gmi
import threading as thd
import numpy as np
import et_circumnavigation.srv as dns


# Parameters
DESIRED_DISTANCE = rp.get_param('desired_distance')
BEARING_THRESHOLD = 0.0
ALPHA = rp.get_param('alpha')
K_PHI = rp.get_param('k_fi')
K_D = rp.get_param('k_d')

NODE_NAME = rp.get_param('node_name')
DELAY = rp.get_param('delay')

# Variables
estimated_distance = None
bearing_measurement = None
estimated_bearing = None
estimated_target = None
position = None




neighbor_bearing_measurement = None
neighbor_last_beta = None
neighbor_estimated_bearing = None
agent_beta = None
follower = None


LOCK = thd.Lock()
NBR_LOCK = thd.Lock()

rp.init_node('planner')


def neighbor_bearing_handler(req):
    global neighbor_bearing_measurement, neighbor_estimated_bearing
    NBR_LOCK.acquire()
    neighbor_bearing_measurement = gmi.Versor(req.bearing)
#    rp.logwarn("received nbr bearing")
    NBR_LOCK.release()
    return dns.NeighborBearingResponse()
rp.Service('neighbor_bearing', dns.NeighborBearing, neighbor_bearing_handler)


def neighbor_beta_handler(req):
    global neighbor_last_beta
    NBR_LOCK.acquire()
    neighbor_last_beta = req.beta
#    rp.logwarn("received nbr beta")
    NBR_LOCK.release()
    return dns.NeighborBetaResponse()
rp.Service('neighbor_beta', dns.NeighborBeta, neighbor_beta_handler)





def position_callback(msg):
    global position
    LOCK.acquire()
    position = gmi.Point(msg)
    LOCK.release()
rp.Subscriber("position", gms.Point, position_callback)


cmdvel_pub = rp.Publisher(
    name='cmdvel',
    data_class=gms.Vector,
    queue_size=10)

est_pub = rp.Publisher(
    name='estimate',
    data_class=gms.Point,
    queue_size=10)


start = False
FREQUENCY = 60.0
RATE = rp.Rate(FREQUENCY)
STEP=1.0/FREQUENCY

rp.sleep(DELAY)

rp.wait_for_service("/add_me")
add_me_proxy = rp.ServiceProxy("/add_me", dns.AddAgent)
add_me_proxy.call(NODE_NAME)

rp.wait_for_service("/share_bearing")
share_bearing_proxy = rp.ServiceProxy('/share_bearing', dns.ShareBearing)
rp.wait_for_service("/share_beta")
share_beta_proxy = rp.ServiceProxy('/share_beta', dns.ShareBeta)

rp.wait_for_service("bearing_measurement")
bearing_measurement_proxy = rp.ServiceProxy(name="bearing_measurement", service_class=dns.BearingMeasurement)
bearing_measurement = gmi.Versor(bearing_measurement_proxy.call().bearing)
estimated_bearing = gmi.Versor(bearing_measurement)

start = False
while not rp.is_shutdown() and not start:
    LOCK.acquire()
    start = all([not data is None for data in [position, bearing_measurement]])
    LOCK.release()
    RATE.sleep()

estimate = position + bearing_measurement.vector*DESIRED_DISTANCE

while not rp.is_shutdown():

    LOCK.acquire()
    pos = gmi.Point(position)
    LOCK.release()

    NBR_LOCK.acquire()
    if not neighbor_last_beta is None:
        nbr_lb = float(neighbor_last_beta)
        neighbor_last_beta = None
    if not neighbor_bearing_measurement is None:
        nbr_lbm = gmi.Versor(neighbor_bearing_measurement)
        neighbor_estimated_bearing = gmi.Versor(nbr_lbm)
        neighbor_bearing_measurement = None
    NBR_LOCK.release()

    estimated_distance = (pos-estimate).norm
    phi_bar = estimated_bearing.rotate(-np.pi/2)
    if not neighbor_estimated_bearing is None:
        agent_beta = estimated_bearing.angle_to(neighbor_estimated_bearing, force_positive=True)
    #if not neighbor_last_beta is None:
        #neighbor_estimated_bearing = neighbor_estimated_bearing.rotate(
            #STEP*K_PHI*(ALPHA+neighbor_last_beta))
        neighbor_estimated_bearing = neighbor_estimated_bearing.rotate(
            STEP*K_PHI*(ALPHA+agent_beta))
    vel = K_D*estimated_bearing.vector*(estimated_distance-DESIRED_DISTANCE)
    if not agent_beta is None:
        vel += K_PHI*estimated_distance*phi_bar.vector*(ALPHA+agent_beta)
        estimated_bearing = estimated_bearing.rotate(STEP*K_PHI*(ALPHA+agent_beta))
    else:
        vel += K_PHI*estimated_distance*phi_bar.vector*ALPHA
        estimated_bearing = estimated_bearing.rotate(STEP*K_PHI*ALPHA)
    #rp.logwarn(estimated_bearing.vector*bearing_measurement.vector)
    if estimated_bearing.vector*bearing_measurement.vector < BEARING_THRESHOLD:
        bearing_measurement = gmi.Versor(bearing_measurement_proxy.call().bearing)
        estimate = position + bearing_measurement.vector*DESIRED_DISTANCE
        estimated_bearing = gmi.Versor(bearing_measurement.vector)
        if not neighbor_estimated_bearing is None:
            agent_beta = estimated_bearing.angle_to(neighbor_estimated_bearing, force_positive=True)
        try: share_bearing_proxy.call(NODE_NAME, bearing_measurement)
        except: rp.logwarn("Error in service call")
        if not agent_beta is None:
            try: share_beta_proxy.call(NODE_NAME, agent_beta)
            except: rp.logwarn("Error in service call")
    cmdvel_pub.publish(vel.serialize())
    est_pub.publish(estimate.serialize())
    RATE.sleep()
