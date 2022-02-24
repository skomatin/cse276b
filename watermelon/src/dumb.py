#!/usr/bin/env python

import rospy
import time

import sensor_msgs.point_cloud2 as pc2

from cmvision.msg import Blob, Blobs
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist


def run_part1():
    t = Twist()
    t.linear.x = 0 # x is moving back and forth
    t.angular.z = 0 # z is turning left and right

    time_start = time.time()
    while time.time() - time_start < 3:
        t.angular.z = -0.3
        self.velocity_pub.publish(t)

    time_start = time.time()
    while time.time() - time_start < 3:
        t.angular.z = 0.0
        t.angular.x = 0.1
        self.velocity_pub.publish(t)





if __name__ == '__main__':
    run_part1()





















# CENTER_X = 300                                              #pixel index indicating center of camera's view region
# BUFFER_ROTATE = 3                                          #width of acceptable goal region
# LEFT_MARGIN = CENTER_X - BUFFER_ROTATE                      #pixel index to mark left margin of goal region
# RIGHT_MARGIN = CENTER_X + BUFFER_ROTATE                     #pixel index to mark right margin of goal region
# MIN_BLOB_AREA = 20                                           #minimum area for a blob to be considered as a goal
# BLOB_NEAR_GOAL_AREA_THRESHOLD = 8900                         #if the blob area is greater than this value, goal is reached
# COLOR = 'Blue'
# IDLE = 0
# SEARCH = 1
# MOVE = 2
# WAIT = 3
# END = 4
# PRINT_FREQUENCY = 3
# # TIME_SICNE_LAST_PRINT = 0

# class Lab4Example:
#     goal_x, goal_y = None, None
#     goal_area = None
#     goal_detected = False
#     closeToTarget = False
#     goal_reached = False
#     time_start = 0
#     goal_area = 0
#     count = 0

#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('lab4_example')

#         # Subscribe to the /blobs topic
#         # print('init')
#         self.blobs_sub = rospy.Subscriber('/blobs', Blobs, self.blobs_cb)

#         # Subscribe to the /camera/depth/points topic
#         self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', 
#                 PointCloud2, self.pointcloud_cb)

#         # Publish commands to the robot
#         self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
#                 Twist, queue_size=1)

#         # Current FSM state
#         self.state = SEARCH

#         # Whether we're close to target
#         # self.closeToTarget = False

#     """ The callback function for the /blobs topic.
#         This is called whenever we receive a message from /blobs.
#         It finds the center of each found COLOR blob """
#     def blobs_cb(self, blobsIn):
#         # maxBlob = None
#         # maxBlobArea = MAXBLOBAREA

#         blob_list_color = [item for item in blobsIn.blobs if item.name == COLOR]
#         blob_list = sorted(blob_list_color, key=lambda b: b.area)
#         if len(blob_list) > 0 and blob_list[-1].area > MIN_BLOB_AREA:
#             self.goal_x = blob_list[-1].x
#             self.goal_area = blob_list[-1].area
#             # if blob_list[-1].area > BLOB_NEAR_GOAL_AREA_THRESHOLD:
#             #     self.closeToTarget = True
#             self.goal_detected = True
#         else:
#             # No meaningful blobs found
#             self.goal_detected = False
        
#         # if (int(time.time() - self.time_start) % PRINT_FREQUENCY) == 0:
#         if (self.count % PRINT_FREQUENCY) == 0:
#             if len(blob_list)> 0:
#                 print(len(blobsIn.blobs), " total blobs detected   ", len(blob_list_color), " ", COLOR, " blobs detected", "   Goal Detected: ", self.goal_detected, " Near Target: ", self.goal_reached, " area: ", blob_list[0].area)
#                 # if self.goal_reached:
#                 #     print("Goal area: ", blob_list[-1].area)
#                 print("---------------------------------------------------------------------------------------------")

#         self.count += 1

#     def findCentroidZ(self, cloud):
#         sum = 0
#         points = list(pc2.read_points(cloud, skip_nans=True))
#         for point in points:
#             sum += point[2]
#         return sum / len(points)


#     """ The callback function for the /camera/depth/points topic.
#         This is called whenever we receive a message from this topic.
#         Prints a message when there's an object below the threshold. """
#     def pointcloud_cb(self, cloud):
#         # Note that point[0] is x, point[1] is y, point[2] is z
#         # print('cloud', len(cloud))
#         for point in pc2.read_points(cloud, skip_nans=True):
#             if point[0] > 0 and point[1] > 0 and point[2] < 0.5:
#                 print('aiya')
#                 self.closeToTarget = True
#                 # print(round(point[0]), round(point[1]), round(point[2]))
#             # print(self.count)
#             # if self.count % 10000 == 0:
#             #     print('\t ', round(point[2], 2))
#         # pass
#         # centroid = self.findCentroidZ(cloud)
#         # print('centroid', centroid)
#         # if centroid < 0.5: # 150 is chosen arbitrarily here
#         #     print("We are close to something!")
#         #     self.closeToTarget = True 
#         # else:
#         #     self.closeToTarget = False

#             # self.count += 1

#     def controller(self):
#         # print("Controller, state = ", self.state)
#         t = Twist()
#         t.linear.x = 0 # x is moving back and forth
#         t.angular.z = 0 # z is turning left and right

#         # if self.state == IDLE:
#         #     if self.goal_x is None or self.goal_y is None:
#         #         # print("Goal Not Defined")
#         #         pass
#         #     else:
#         #         self.state = SEARCH

#         if self.state == SEARCH:
#             if self.goal_detected == False:
#                 t.angular.z = -0.5
#             else:
#                 self.state = MOVE
#         elif self.state == MOVE:
#             action = "Searching"
#             if RIGHT_MARGIN < self.goal_x < LEFT_MARGIN:
#                 #if goal is not within region --> search
#                 t.angular.z = -0.2
#             else:
#                 if self.closeToTarget:
#                     action = "Close To Target"
#                     self.state = END
#                 else:
#                     action = "Moving Forward"
#                     t.angular.z = 0
#                     t.linear.x = 0.1    
#             if (int(time.time() - self.time_start) % PRINT_FREQUENCY) == 0:
#                 pass
#                 # print(action)
#                 # print("---------------------------------------------------------------------------------------------")
#         elif self.state == END:
#             self.goal_reached = True
#             print("GOAL REACHED, goal area: ", self.goal_area)
            
#         self.velocity_pub.publish(t)

#     def run(self):
#         self.time_start = time.time()
#         r_time_f=rospy.Rate(20)
#         while not rospy.is_shutdown():
#             self.controller()
#             r_time_f.sleep()
#             if self.goal_reached:
#                 break

# if __name__ == '__main__':
#     l4e = Lab4Example()
#     l4e.run()
