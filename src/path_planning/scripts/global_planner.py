#!/usr/bin/env python3
# -*- coding: utf -8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json

from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point32, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path

current_path=os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)


"""
    summary:
    1. Read the Mgeo Data
    2. Define Start node and final node
    3. Calculate Weight value
    4. Dijkstra Path initialization
    5. Dijkstra Class
    6. Node path
    7. Link Path
    8. result
    9 Point Path
    10. Define: Dijkstra data to Ros path message
    11. Publish
"""
class dijkstra_path_pub:
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)
        self.global_path_pub=rospy.Publisher('/global_path', Path, queue_size=1)
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)
        
        #TODO: (1) Read Mgeo DATA and Confirm the data
        load_path=os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/SeongNamecity'))