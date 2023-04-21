#!/usr/bin/env python
# from scipy import ndimage
from scipy import ndimage
from scipy.misc import imsave

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
import matplotlib.pyplot as plt


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

        self.map_pub = rospy.Publisher('/dilated_map', OccupancyGrid, queue_size=10)


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
        # rospy.loginfo("INITIALIZED")
        # rospy.loginfo(self.current_location)

    def map_cb(self, msg):
        width, height = msg.info.width, msg.info.height
        map = msg.data
        map2d = [map[s:s + width] for s in range(0, len(map), width)] # Convert to 2D

        # Define the structuring element for erosion and dilation
        erode_radius = 6
        dilate_radius = 12
        se = ndimage.generate_binary_structure(2, 2)
        se_erode = ndimage.iterate_structure(se, 2*erode_radius+1)
        se_dilate = ndimage.iterate_structure(se, 2*dilate_radius+1)
        # Define the structuring element for the dilation, which determines the shape and size of the dilation
        # structuring_element = np.array([[0, 1, 0],
        #                         [1, 1, 1],
        #                         [0, 1, 0]], dtype=np.uint8)
        # structuring_element = np.array([[0, 0, 1, 0, 0, 0],
        #                         [0, 0, 1, 0, 0, 0],
        #                         [0, 0, 1, 0, 0, 0],
        #                         [1, 1, 1, 1, 1, 1],
        #                         [0, 0, 1, 0, 0, 0],
        #                         [0, 0, 1, 0, 0, 0]], dtype=np.uint8)

        # Erode + Dilate the obstacles in the map
        eroded_map = ndimage.binary_erosion(map2d, se_erode)
        dilated_map = ndimage.binary_dilation(eroded_map, se_dilate)
        # Invert the binary values in the dilated map to match the ROS occupancy grid convention
        # eroded_map = np.invert(eroded_map)
        # dilated_map = np.invert(dilated_map)
        # rospy.loginfo(dilated_map)

        # Create the OccupancyGrid message and fill in its fields
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = 1.0
        map_msg.info.width = dilated_map.shape[1]
        map_msg.info.height = dilated_map.shape[0]
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = list(dilated_map.ravel())

        self.map_pub.publish(map_msg)

        # Save the dilated map as a PNG file
        imsave('dilated_map_official.png', dilated_map)

        self.map = dilated_map
        self.map_resolution = msg.info.resolution
        self.map_origin = pose_to_xytheta(msg.info.origin)

        rospy.loginfo("MAP INITIALIZED")

        
        # Visualize the original map and the dilated map side by side
        fig, axs = plt.subplots(1, 2)
        axs[0].imshow(map2d, cmap='gray')
        axs[0].set_title('Original Map')
        axs[1].imshow(dilated_map, cmap='gray')
        axs[1].set_title('Eroded+Dilated Map')
        plt.show()


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
        # rospy.loginfo(self.goal_location)
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
        path = path_planning.get_path(max_iter=3500, delta=8)
        new_traj = LineTrajectory(viz_namespace="debug_traj")
        if path == None:
            rospy.loginfo("No Path Planned")
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
