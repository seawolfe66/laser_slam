#!/usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_from_euler as qfe
from actionlib import SimpleActionClient

import numpy as np
from math import radians


from geometry_msgs.msg import PolygonStamped, Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

import shapely.geometry as geo


class PolygonGatherNode(object):
    """
    This is a ROS node that is responsible for gathering points 
    and transform those points to a polygon published as a PolygonStamped message meanwhile
    publishing the planning path.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('gatherpolygon')

        # ROS params
        self.gap_points = rospy.get_param("~gap_points",0.2)

        # Setup publishers and subscribers
        rospy.Subscriber('/odom', PolygonStamped, self.odom_callback)
        self.polygon_pub = rospy.Publisher('polygon_gather',
                                               PolygonStamped,
                                               queue_size=10)
        self.path_pub = rospy.Publisher('coverage_path',Path,queue_size=10)

        # Setup initial variables
        self.robot_pose = None
        self.path = None
        self.polygon_area = None
        self.origin = None
        self.start_gathering = False
        # Spin until shutdown or we are ready for polygon gathering
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.start_gathering:
                # Run until stopped
                heading = 0
                # Setup path following
                self.setup_path_following(heading)    
                # Iterate on path following
                while not rospy.is_shutdown():
                    if not self.step_path_following():
                        break
                self.start_gathering = False

    def odom_callback(self,msg):
        """
        Watches for the robot's Odometry data, which is used in the point array.
        """
        self.robot_pose = msg 
    
    def prepare_polygon_gather(self):
        """
        Set the originial position , Wait untile the robot_pose is not none
        """
        # Wait for the robot position
        while self.robot_pose == None:
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Print message about the waiting
            msg = "Qualification: waiting on initial robot pose."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        # Now we should start to gather using the robot's initial pose
        self.origin = (self.robot_pose.pose.pose.position.x,
                  self.robot_pose.pose.pose.position.y)
    
    def run_polygon_gather(self):
        """
        Gathering points with current position , checking if the distance between last two points
        is bigger than the gap_points param value,
        """
        