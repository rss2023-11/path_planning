#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

import rrt

def pose_to_xytheta(pose):
    """
    Converts a rospy Pose object to (x, y, theta) format.
    
    Args:
        pose (geometry_msgs.msg.Pose): A ROS Pose message.
    
    Returns:
        tuple: A tuple (x, y, theta) representing the pose.
    (Thanks ChatGPT)
    """
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    # Calculate theta (yaw) from quaternion
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    return (x, y, yaw)

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialize_current_location, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size = 10) #pose subscriber--tells you where the robot is


        self.map = None
        self.map_resolution = None
        self.map_origin = None
        self.current_location = None
        self.goal_location = None #(x,y)

    def initialize_current_location(self, msg):
        """
        given: PoseStamped message of initial position (x,y)
        function: update the current location for initial position
        """
        self.current_location = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo("INITIALIZED")
        rospy.loginfo(self.current_location)

    def map_cb(self, msg):
        width, height = msg.info.width, msg.info.height
        map = msg.data
        self.map = [map[s:s + width] for s in range(0, len(map), width)] # Convert to 2D
        self.map_resolution = msg.info.resolution
        self.map_origin = pose_to_xytheta(msg.info.origin)
        rospy.loginfo("MAP INITIALIZED")

    def odom_cb(self, msg):
        """
        given: odometry message Odometry
        function: update current location (don't need to update path, will just update path periodically)
        """
        self.current_location = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_cb(self, msg):
        """
        given: PoseStamped message
        function: update new trajectory with path 
        """
        #extract x,y position coordinates from message PoseStamped
        self.goal_location = (msg.pose.position.x, msg.pose.position.y)
        #make path with new goal location
        rospy.loginfo(self.goal_location)
        self.plan_path()

    def plan_path(self):
        """
        start_point: The start point of the path, in world coordinates
        end_point: The end point of the path, in world coordinates
        map: The occupancy grid of the map, in map coordinates
        
        """
        # Step 1: Transform the start point and end point to map coordinates
        def translate(point, dx, dy):
            return [point[0] + dx, point[1] + dy]

        def rotate(point, angle):
            return np.dot([[math.cos(angle), -math.sin(angle)],
                           [math.sin(angle), math.cos(angle)]], point)
        
        def dilate(point, scale):
            return [point[0] * scale, point[1] * scale]
        
        def transform_to_map_coords(point):
            translated = translate(point, -self.map_origin[0], -self.map_origin[1])
            rotated = rotate(translated, -self.map_origin[2])
            scaled = dilate(rotated, 1 / self.map_resolution)
            return scaled
        
        def transform_from_map_coords(point):
            scaled = dilate(point, self.map_resolution)
            rotated = rotate(scaled, self.map_origin[2])
            translated = translate(rotated, self.map_origin[0], self.map_origin[1])
            return translated

        start = transform_to_map_coords(self.current_location)
        goal = transform_to_map_coords(self.goal_location)
        path_planning = rrt.RRT_Connect(start, goal, self.map)
        path = path_planning.get_path(max_iter=5000, delta=1)
        new_traj = LineTrajectory(viz_namespace="debug_traj")
        for point in path:
            new_traj.addPoint(*transform_from_map_coords(point))
        self.trajectory = new_traj # Delay actually setting till the new trajectory is complete for threading reasons

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
