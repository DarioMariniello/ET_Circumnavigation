#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import threading as thd
import numpy as np
import circumnavigation_moving_target.srv as dns


delay=rp.get_param('delay')
agent_to_remove=rp.get_param('agent_to_remove')
rp.sleep(delay)
# Call to the service "Remove": the agent requires to the cloud to add his name
rp.wait_for_service('Remove')
remove_proxy=rp.ServiceProxy('Remove', dns.RemoveAgent)
remove_proxy.call(agent_to_remove)


rp.init_node('remove_agent') 
rp.spin()