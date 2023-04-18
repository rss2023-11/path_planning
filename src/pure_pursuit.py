#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf
from numpy.linalg import norm

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 1 #should be modified, this is a guess
        self.speed            = 1 #to be modified probably
        self.wheelbase_length = 0.1 # i have no clue what this is--is it the width of our robot?
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        
        self.pose_sub = rospy.Subscriber("/pf/pose/odom", Odometry, queue_size = 1)
        
        self.pose_x = self.pose_sub.pose.position.x
        self.pose_y = self.pose_sub.pose.position.y
        self.orientation = self.pose_sub.pose.orientation #this is given in quaternians
        
        #should get rid of this when doing pure pursuit?
        self.K_P = 0.25
        self.K_I = 0
        self.K_D = 0.5
        self.error_I = 0
        self.prev_error = 0

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
    
    def driving_decision(self):
        ack_stamped = AckermannDriveStamped()
        ack_stamped.header = Header(stamp=rospy.Time.now())
        ack_stamped.drive.speed=self.speed
        
        error=self.calc_perp_dist(self.trajectory.points[0][0], self.trajectory.points[0][1], self.trajectory.points[-1][0], self.trajectory.points[-1][1])
        
        #will be replaced by pure pursuit later
        error_P = self.K_P * error
        self.error_I += self.K_I * error
        error_D = self.K_D * slope
        ack_stamped.drive.steering_angle = error_P + self.error_I + error_D

        # store current error for next D term calculation. we never use this?
        self.prev_error = error
        
        return ack_stamped
    
        self.drive_pub.publish(ack_stamped)
        
    def circle_path_intersect(self):
        Q = np.array([self.pose_x, self.pose_y]) # Centre of circle
        r = self.lookahead # Radius of circle
        P1 = np.array([self.trajectory.points[0][0], self.trajectory.points[0][1]]) # Start of line segment
        V = np.array([self.trajectory.points[-1][0], self.trajectory.points[-1][1] - P1]) # Vector along line segment
        #t = 0 #t is a parameter in finding the point P1 + t * V, our desired point
        
        a = V.dot(V)
        b = 2 * V.dot(P1 - Q)
        c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2
        
        disc = b**2 - 4 * a * c
        if disc < 0:
            #steer towards the start point (or the end point if it's closer?)
            pose_start = calc_distance(self.trajectory.points[0][0], self.trajectory.points[0][1], self.pose_x, self.pose_y)
            pose_end = calc_distance(self.trajectory.points[-1][0], self.trajectory.points[-1][1], self.pose_x, self.pose_y)
            start_end = calc_distance(self.trajectory.points[0][0], self.trajectory.points[0][1], self.trajectory.points[-1][0], self.trajectory.points[-1][1])
            
            if pose_end>start_end and pose_start<pose_end:
                #return start
                return np.array([self.trajectory.points[0][0], self.trajectory.points[0][1]])
            else:
                #return end
                return self.trajectory.points[-1][0], self.trajectory.points[-1][1]
                        
        elif disc==0: #one root=one intersection
            t = (-b + sqrt_disc) / (2 * a)
            if not (0 <= t <= 1):
                if t<0:
                    #return start
                    return np.array([self.trajectory.points[0][0], self.trajectory.points[0][1]])
                elif t<1:
                    #return end
                    return self.trajectory.points[-1][0], self.trajectory.points[-1][1]
            
        else: #2 intersections
            t1 = (-b + sqrt_disc) / (2 * a)
            t2 = (-b - sqrt_disc) / (2 * a)

            if (0 <= t1 <= 1 and 0 <= t2 <= 1): #both on line segment
                t1_end = calc_distance(t1[0], t1[1], self.trajectory.points[-1][0], self.trajectory.points[-1][1])
                t2_end = calc_distance(t2[0], t2[1], self.trajectory.points[-1][0], self.trajectory.points[-1][1])
                
                if t2_end>=t1_end:
                    t=t2
                else:
                    t=t1
            
            if not (0 <= t1 <= 1 or 0 <= t2 <= 1): #neither on line segment--segment is inside the circle
                #return end
                return self.trajectory.points[-1][0], self.trajectory.points[-1][1]
                
            elif (0 <= t2 <= 1): #t1 only on line segment
                t=t2
                
            elif (0 <= t2 <= 1) #t2 only on line segment
                t=t1
                
            return P1 + t * V
        
    def calc_perp_dist(self, x1, y1, x2, y2):
        
        pose=np.array([self.pose_x, self.pose_y])
        line_start=np.array([x1, y1])
        line_end=np.array([x2, y2])
        
        return np.cross(line_end-line_start,pose-line_start)/norm(line_end-line_start)

    def calc_distance(x1, y1, x2, y2)
    #calculates distance between (x1, y1) and (x2, y2)
        p1=np.array([x1, y1])
        p2=np.array([x2, y2])
        
        return norm(p1-p2)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
