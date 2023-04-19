#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Point
from ackermann_msgs.msg import AckermannDriveStamped
import math
import time
import utils
import tf
from numpy.linalg import norm
from sklearn.linear_model import LinearRegression

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Point
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
        self.lookahead = 0.5 #This is a guess  
        self.linear_speed = 1  
        self.max_angular_speed = 0.34  # Maximum angular speed (rad/s)
        self.wheelbase_length = 0.35  # Distance between front and rear axles of car
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        # self.trajectory = traj_points #remove this line when listening for a trajectory
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        
        self.pose_sub = rospy.Subscriber("/pf/pose/odom", Odometry, queue_size = 1)

        # Define variables to store current position and orientation
        self.current_position = None
        self.current_orientation = None

        # Define variable to store the target point
        self.target_point = None

    def find_target_point(self, trajectory):
        # Find the nearest waypoint to the current position as the target point
        # This will be modified with the circle formula, but for now this is fine
        nearest_distance = float('inf')
        nearest_point = None

        for waypoint in trajectory:
            distance = math.sqrt((waypoint.x - self.current_position.x)**2 + (waypoint.y - self.current_position.y)**2)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point = waypoint

        return nearest_point

    def calculate_steering_angle(self, target_point):
        # Calculate the steering angle to the target point using Ackermann steering geometry
        dx = target_point.x - self.current_position.x
        dy = target_point.y - self.current_position.y
        alpha = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)

        steering_angle = math.atan2(2.0 * self.wheelbase_length * math.sin(alpha), distance)

        return steering_angle

    # def trajectory_callback(self, msg):
    def trajectory_callback(self, msg): #Fxn will be changed to take in a path message once we have that
        # Define control rate. This line will get deleted when listening to a subscriber
        rate = rospy.Rate(50)
        rospy.loginfo("trajectory_callback is runnig")
        
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        # print "Receiving new trajectory:", len(msg.poses), "points"
        # self.trajectory.clear()
        # self.trajectory.fromPoseArray(msg)

        while not rospy.is_shutdown():
            # Calculate the distance to the target point
            self.current_position = msg.pose.position
            self.current_orientation = msg.pose.orientation
            
            rospy.loginfo("calculating distance to target. current pos then target pos") #print statement
            rospy.loginfo(self.current_position) #print statement
            rospy.loginfo(self.target_point) #print statement
            distance = math.sqrt((self.current_position.x - self.target_point.x)**2 + (self.current_position.y - self.target_point.y)**2)

            # If the distance is less than the target distance, find a new target point
            if distance < self.lookahead:
                self.target_point = self.find_target_point(waypoints)

            # Calculate the steering angle to the target point
            steering_angle = self.calculate_steering_angle(self.target_point)

            # Calculate the linear and angular velocities based on the steering angle and maximum speeds
            linear_velocity = self.linear_speed
            angular_velocity = steering_angle * self.max_angular_speed

            # Create an AckermannDriveStamped message with the linear and angular velocities
            ack_stamped = AckermannDriveStamped()
            ack_stamped.drive.speed = linear_velocity
            ack_stamped.drive.steering_angle = steering_angle

            # Publish the AckermannDriveStamped message
            rospy.loginfo("publishing to driver") #print statement
            self.drive_pub.publish(ack_stamped)

            # Sleep for the remaining time to maintain the control loop rate
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    
    traj_points = [
    Point(1.0, 0.0, 0.0),
    Point(2.0, 0.0, 0.0),
    Point(3.0, 1.0, 0.0),
    Point(3.0, 2.0, 0.0),
    Point(2.0, 3.0, 0.0),
    Point(1.0, 3.0, 0.0),
    Point(0.0, 2.0, 0.0),
    Point(0.0, 1.0, 0.0),
    Point(1.0, 0.0, 0.0)
    ]
    
    message=PoseArray
    message
    
    pf = PurePursuit()
    # pf.trajectory_callback()
    rospy.spin()