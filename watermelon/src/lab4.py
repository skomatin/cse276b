#!/usr/bin/env python

import rospy
import time

import sensor_msgs.point_cloud2 as pc2

from cmvision.msg import Blob, Blobs
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

BUFFER_ROTATE = 100
IDLE = 0
SEARCH = 1
MOVE = 2
WAIT = 3
END = 4

class Lab4Example:
    goal_x, goal_y = None, None
    goal_detected = False

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lab4_example')

        # Subscribe to the /blobs topic
        # print('init')
        self.blobs_sub = rospy.Subscriber('/blobs', Blobs, self.blobs_cb)

        # Subscribe to the /camera/depth/points topic
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', 
                PointCloud2, self.pointcloud_cb)

        # Publish commands to the robot
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                Twist, queue_size=1)

        # Current FSM state
        self.state = SEARCH

    """ The callback function for the /blobs topic.
        This is called whenever we receive a message from /blobs.
        It finds the center of each found RED blob """
    def blobs_cb(self, blobsIn):

        # goal_x, goal_y = 0, 0

        print(len(blobsIn.blobs), " blobs detected")
        maxBlob = None
        maxBlobArea = 500

        for blob in blobsIn.blobs:
            # print("Color: ", blob.name)
            # print("Blob\n", blob)
            # Note that we define 'RED' to be (255, 0, 0) in the 
            # colors.txt calibration file
            # print(blob.name, blob.area)
            if blob.name == 'Pink' and blob.area >= maxBlobArea:
                maxBlobArea = blob.area
                maxBlob = blob

        if maxBlob:
            # blob.x and blob.y are the center of the blob
            self.goal_detected = True
            self.goal_x = maxBlob.x
            self.goal_y = maxBlob.y
            time.sleep(0.5)
            print(maxBlob.name, maxBlob.area)
        else:
            self.goal_detected = False
            print('no pink blob detected')


    """ The callback function for the /camera/depth/points topic.
        This is called whenever we receive a message from this topic.
        Prints a message when there's an object below the threshold. """
    def pointcloud_cb(self, cloud):
        # Note that point[0] is x, point[1] is y, point[2] is z
        for point in pc2.read_points(cloud, skip_nans=True):
            if point[2] < 150: # 150 is chosen arbitrarily here
                # print("We are close to something!")
                pass

    def controller(self):
        # print("Controller")
        t = Twist()
        t.linear.x = 0 # x is moving back and forth
        t.angular.z = 0 # z is turning left and right

        # if self.state == IDLE:
        #     if self.goal_x is None or self.goal_y is None:
        #         # print("Goal Not Defined")
        #         pass
        #     else:
        #         self.state = SEARCH

        if self.state == SEARCH:
            print('searching, goal detected?:', self.goal_detected)
            if not self.goal_detected:
                print('goal not detected')
                t.angular.z = -0.5
            else:
                self.state = MOVE
        # elif self.state == MOVE:
        #     print(self.goal_x, BUFFER_ROTATE)
        #     if self.goal_x < BUFFER_ROTATE:
        #         # turn left
        #         t.angular.z = -0.3
        #         print('rotating left')
        #     else:
        #         print('moving forward')
        #         # move forward
        #         t.angular.z = 0
        #         t.linear.x = 0.1
        #     print(self.goal_x)
        #     # pass
        self.velocity_pub.publish(t)

    def run(self):
        # print("Run..")
        # self.controller()
        # rospy.spin()
        r_time_f=rospy.Rate(10)
        while not rospy.is_shutdown():
            self.controller()
            r_time_f.sleep()

if __name__ == '__main__':
    l4e = Lab4Example()
    l4e.run()
