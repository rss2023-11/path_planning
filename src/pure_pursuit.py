#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Point, Pose
from ackermann_msgs.msg import AckermannDriveStamped
import math
import time
import utils
import tf
from numpy.linalg import norm
from sklearn.linear_model import LinearRegression

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry

class PurePursuit:
    def __init__(self):

        # Define subscriber to get current pose
        # rospy.Subscriber('/pose', PoseStamped, self.pose_callback)

        # Define publisher to send drive commands
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        # Define control parameters
        self.odom_topic = rospy.get_param("~odom_topic")
        self.lookahead = 0.05 #This is a guess  
        self.linear_speed = 1  
        self.max_angular_speed = 0.34  # Maximum angular speed (rad/s)
        self.wheelbase_length = 0.35  # Distance between front and rear axles of car
        self.trajectory  = utils.LineTrajectory("/followed_trajectory").toPoseArray().poses
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        
        # self.lookahead_pub = rospy.Publisher("/lookahead", Pose, queue_size=1)
        
        self.pose_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.define_robot_pose_callback, queue_size = 1) #pose subscriber--tells you where the robot is

        # Define variables to store current position and orientation
        self.current_position = None
        self.current_orientation = None

        # Define variable to store the target point
        self.target_position = None

    def define_robot_pose_callback(self, msg):

        self.current_position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation #this is given in quaternions
            
        self.find_target_point(self.trajectory)

        rospy.logwarn('current: {x},  {y}'.format(x=repr(self.current_position.x), y=repr(self.current_position.y)))

        # self.lookahead_pub.publish(self.target_point)

        
        if self.target_position:
            rospy.logwarn("target: {x} , {y}".format(x=repr(self.target_position.x), y=repr(self.target_position.y)))
            distance = math.sqrt((self.current_position.x - self.target_position.x)**2 + (self.current_position.y - self.target_position.y)**2)

            # If the distance is less than the target distance, find a new target point
            if distance < self.lookahead:
                self.find_target_point(self.trajectory)


            # Calculate the steering angle to the target point
            steering_angle = self.calculate_steering_angle()
            # steering_angle = 0

            # Calculate the linear and angular velocities based on the steering angle and maximum speeds
            linear_velocity = self.linear_speed
            angular_velocity = steering_angle * self.max_angular_speed

            # Create an AckermannDriveStamped message with the linear and angular velocities
            ack_stamped = AckermannDriveStamped()
            ack_stamped.drive.speed = linear_velocity
            # ack_stamped.drive.speed = 0
            ack_stamped.drive.steering_angle = steering_angle
            # ack_stamped.drive.steering_angle = 0

            # Publish the AckermannDriveStamped message
            rospy.logwarn(steering_angle)
            self.drive_pub.publish(ack_stamped)

    def find_target_point(self, trajectory):
        # Find the nearest waypoint to the current position as the target point
        # This will be modified with the circle formula, but for now this is fine I think
        # waypoint type: pose
        
        nearest_distance = float('inf')
        nearest_point = None
        
        # waypoint is a POSE
        for waypoint in trajectory:
            distance = math.sqrt((waypoint.position.x - self.current_position.x)**2 + (waypoint.position.y - self.current_position.y)**2)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point = waypoint
        
        self.target_position = nearest_point.position

        if not nearest_point:
            rospy.logwarn("no target point found")


    def calculate_steering_angle(self):
        # Calculate the steering angle to the target point using Ackermann steering geometry
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        alpha = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)

        steering_angle = math.atan2(2.0 * self.wheelbase_length * math.sin(alpha), distance)

        return steering_angle

    # def trajectory_callback(self, msg):
    def trajectory_callback(self, msg): #Fxn will be changed to take in a path message once we have that        
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        # print("Receiving new trajectory:", len(msg.poses), "points")
        rospy.logwarn("trajectory_callback is running")
        self.trajectory = msg.poses


if __name__=="__main__":
    rospy.init_node("pure_pursuit")    
    pf = PurePursuit()
    # pf.trajectory_callback()
    rospy.spin()
