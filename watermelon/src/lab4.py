#!/usr/bin/env python

""""
* Filename: lab4.py
* Student: Arthi Haripriyan, Pratyusha Ghosh, Alex Chow, Saikiran Komatieni
* Lab 4: Multimodal Control
*
* Description: This is the main control file to control the robot to
* detect goals and avoid obstacles at the same time. The main class'
* init method defines all variables that subscribe to the blobs and 
* point cloud topic. The callback method for blob detection implements
* the goal detection logic and the controller() method implements the 
* logic to perform the appropriate action given the current robot state.
*
* How to use:
* Usage:
* roscore
* roslaunch turtlebot_bringup minimal.launch
* roslaunch astra_launch astra_pro.launch
*
* <close it when done>
* roslaunch cmvision cmvisionlaunch image:=/camera/rgb/
* image_raw
* <Ctrl-C>
* rosparam set /cmvision/color_file ~/turtlebot_ws/src/
* cmvision/colors.txt
*
* rosrun cmvision cmvision image:=/camera/rgb/image_raw
*
*
* rosrun dance_bot dance_bot
"""

import rospy
import time
import numpy as np

import sensor_msgs.point_cloud2 as pc2
from cmvision.msg import Blob, Blobs
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

CENTER_X = 300                                              # pixel index indicating center of camera's view region
BUFFER_ROTATE = 60                                          # width of acceptable goal region
LEFT_MARGIN = CENTER_X - BUFFER_ROTATE                      # pixel index to mark left margin of goal region
RIGHT_MARGIN = CENTER_X + BUFFER_ROTATE                     # pixel index to mark right margin of goal region
MIN_BLOB_AREA = 20                                          # minimum area for a blob to be considered as a goal
BLOB_NEAR_GOAL_AREA_THRESHOLD = 8900                        # if the blob area is greater than this value, goal is reached
COLOR = 'Pink'                                              # color of goal

# Mapping robot to states to integer values
IDLE = 0
SEARCH = 1
MOVE = 2
WAIT = 3
END = 4
DETOUR = 5

BACKUP_LIMIT = 20                                           # number of iterations up to which the robot should backup when obstacle is detcted
DETOUR_TURN_THRESH = 0.5                                    # Limit for the robot's angular movement
NOT_DETECT_COUNT_THRESH = 5                                 # Threshold for the number of iterations which can pass with the goal not being detected

class MultiModalControl:
    goal_x, goal_y = None, None                             # x, y positions of the detected goal will be stored here
    goal_area = 0                                           # area of the detected goal 
    goal_detected = False                                   # bool indicating whether the goal has been detected
    closeToTarget = False                                   # bool indicating whether the robot is close to the target
    goal_reached = False                                    # bool indicating whether the robot has reached the goal                         
    thresh = 8                                              # threshold for stopping at the goal, for denoising purposes
    thresh_reached = 0                                      # number of times the robot detects how close it is to the goal, for denoising purposes
    backup_iter = 0                                         # keeps track of the number of iterations for which the robot is backing up
    backup_z = 0                                            # the centroid_z value once the robot stops backing up
    centroid_z = 0                                          # the average z value for all points in the PCL
    prev_detect = False                                     # whether it had detected the goal in the previous cycle, for denoising purposes
    not_detect_count = 0                                    # the number of consecutive times the robot has not detected the goal, for denoising purposes

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lab4_example')

        # Subscribe to the /blobs topic
        self.blobs_sub = rospy.Subscriber('/blobs', Blobs, self.blobs_cb)

        # Subscribe to the /camera/depth/points topic
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', 
                PointCloud2, self.pointcloud_cb)

        # Publish commands to the robot
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                Twist, queue_size=1)

        # Current FSM state
        self.state = IDLE

    """"
    Name: blobs_cb(self, blobsIn)
    Purpose: The callback function for the /blobs topic.
            This is called whenever we receive a message from /blobs.
            It finds the center of each found COLOR blob
    @input blobsIn : data structure containing blobs information
    """ 
    def blobs_cb(self, blobsIn):
        self.prev_detect = self.goal_detected               # store before update

        blob_list_color = [item for item in blobsIn.blobs if item.name == COLOR]
        blob_list = sorted(blob_list_color, key=lambda b: b.area)           # sorts blobs by area
        if len(blob_list) > 0 and blob_list[-1].area > MIN_BLOB_AREA:
            self.goal_x = blob_list[-1].x
            self.goal_area = blob_list[-1].area
            self.goal_detected = True
        else:
            # No meaningful blobs found
            self.goal_detected = False

    """"
    Name: pointcloud_cb(self, cloud):
    Purpose: The callback function for the /camera/depth/points topic.
        This is called whenever we receive a message from this topic.
        Prints a message when there's an object below the threshold.
    @input cloud : data structure that stores point cloud information
    """
    def pointcloud_cb(self, cloud):
        # Note that point[0] is x, point[1] is y, point[2] is z
        points = list(pc2.read_points(cloud, skip_nans=True))
        sum = 0

        for point in points:
            sum += point[2]
        self.centroid_z = sum/len(points)
        
        if self.centroid_z < 3 and self.thresh_reached < self.thresh and self.state != DETOUR:
            # accumulate number of times centroid_z indicates being close to the goal, and only
            # flip closeToTarget state to true once it has accumulates some number of times (denoising)
            self.thresh_reached += 1
            if self.thresh_reached >= self.thresh:
                self.closeToTarget = True
    
    """"
    Name: controller(self):
    Purpose: The main conntrol logic for the robot which takes actions according to 
            the current state of the robot. Move handles movement of the robot 
            and Detour handles obstacle avoidance.
    """
    def controller(self):
        t = Twist()
        t.linear.x = 0                          # x is moving back and forth
        t.angular.z = 0                         # z is turning left and right

        # Multiple if else statements to handle various state cases
        if self.state == IDLE:
            # if goal_x is not defined
            if self.goal_x is None:
                pass
            else:
                self.state = SEARCH

        # Robot rotates in place in SEARCH state
        elif self.state == SEARCH:
            if self.goal_detected == False:
                t.angular.z = -0.5
            else:
                self.state = MOVE
        
        # Robot moves towards goal and avoids obstacles
        elif self.state == MOVE:
            if not self.goal_detected:
                if self.prev_detect == False:
                    # goal not previous detected
                    self.not_detect_count += 1
                else:
                    self.not_detect_count = 0
                
                if self.not_detect_count > NOT_DETECT_COUNT_THRESH: 
                    self.state = DETOUR

            if not (self.goal_x < RIGHT_MARGIN and self.goal_x > LEFT_MARGIN):
                # if goal is not within region --> try to center goal
                if self.goal_x > RIGHT_MARGIN:
                    t.angular.z = -0.3
                else:
                    t.angular.z = 0.3
            else:
                if self.closeToTarget:
                    # ending
                    self.state = END
                else:
                    # move forward
                    t.angular.z = 0
                    t.linear.x = 0.1
        
        # Robot avoids obstacle
        elif self.state == DETOUR:
            # back up sequence
            if self.backup_iter < BACKUP_LIMIT:
                t.linear.x = -0.1
                self.backup_z = self.centroid_z
                self.backup_iter += 1
            elif self.centroid_z < self.backup_z + DETOUR_TURN_THRESH: # check for centroid z
                # turn left
                t.angular.z = 0.2
            else:
                # go forward clockwise until goal found again
                self.backup_z = np.NINF             # to avoid the above elif condition from being satisfied again
                t.linear.x = 0.1
                t.angular.z = -0.11
                if self.goal_detected:
                    self.backup_iter = 0            # reset variables right before switching state
                    self.state = MOVE

        # Robot has reached the goal
        elif self.state == END:
            self.goal_reached = True
            
        self.velocity_pub.publish(t)

    """
    Name: run(self):
    Purpose: The main run method for the entire program. The robot keeps running till the goal is not reached
    """
    def run(self):
        r_time_f=rospy.Rate(20)
        while not rospy.is_shutdown():
            self.controller()
            r_time_f.sleep()
            if self.goal_reached:
                break

if __name__ == '__main__':
    mmc = MultiModalControl()
    mmc.run()
