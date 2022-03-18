#!/usr/bin/env python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from tokenize import Double
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np

def goForward():
    pose_msg.x = 0.1
    pose_msg.y = 0
    pose_msg.theta = 0  
    #return pose_msg


def turnLeft():
    pose_msg.x = 0.1
    pose_msg.y = 0
    pose_msg.theta = np.pi/16
    #return pose_msg


def turnRight():
    pose_msg.x = 0.1
    pose_msg.y = 0
    pose_msg.theta = -np.pi/16
    #return pose_msg


def rangeRegionCollector(lidar_data): 
    #print("Minimum angle is: ", lidar_data.angle_min)
    #print("Maximum angle is: ", lidar_data.angle_max)
    #print("Angle increment is: ", lidar_data.angle_increment)
    #print("In range collector")
    num_beams = 360
    # Front region is defined as the beams between 7*num_beams/8 and num_beams/8

    front_beams = np.zeros(0,float)
    right_beams = np.zeros(0,float)
    # Determine front region minimum range
   # print("Num beams: ", len(lidar_data.ranges))

    #front_beams = lidar_data.ranges(num_beams/16):[]
    for i in range(num_beams):
       # print("Range at ", (lidar_data.angle_min + (i*lidar_data.angle_increment)), " is: ", lidar_data.ranges[i])
       
       if (i <= (num_beams/16)) or (i >= (15*num_beams/16)): # Front region
           #print("Appending ",lidar_data.ranges[i]," to front beams")
           #np.append(front_beams, [lidar_data.ranges[i]], axis=0)
           front_beams = np.append(front_beams, lidar_data.ranges[i])
           #print("Adding to front because i = ", i)

       elif (i >= (11 * num_beams / 16)) and (i < (13 * num_beams / 16)) : # Right region
           #print("Appending ",lidar_data.ranges[i]," to right beams")
           right_beams = np.append(right_beams, lidar_data.ranges[i])
           #print("Adding to right because i = ", i)

    
    front_ofa = np.amin(front_beams)
    right_ofa = np.amin(right_beams)
    print("Min front is: ", np.round(front_ofa,2), ", Min right is: ", np.round(right_ofa, 2))
    
    
    # range_regions = 0
    #print("Length of front beams array: ", front_beams.shape)
    #print("Length of right beams array: ", right_beams.shape)

    stateEstimator(front_ofa, right_ofa)
   
def stateEstimator(front_ofa, right_ofa):
   
    # Define states
    # Define front region threshold
    front_threshold = 0.9
    # Define right goldilocks threshld start and end
    right_goldilocks_begin = 0.72
    right_goldilocks_end = 0.78
    right_horizon = 1.5
    #print("in state estimator")
    # Set state
    if (front_ofa < front_threshold): # State 21
        # Turn left
        #print("Too close in front")
        print("State 21- Turning left")
        turnLeft()
    else: # State 22
        if right_ofa < right_goldilocks_begin: # State 11, 22
            # Turn left
           # print("Too close on the right")
            print("State 11, 22 - Turning left")
            turnLeft()
        elif right_ofa > right_goldilocks_end and right_ofa < right_horizon: # State 13, 22
            # Turn right
            print("State 12, 22 - Turning right ")
            turnRight()
        elif right_ofa > right_horizon:
            #print("Can't see shit dude - going straight")
            goForward()
        else: # State 12, 22
            # Go forward
            print("State 12, 22 - Go forward")
            goForward()
    #print("Vel cmd set")
    #publisher()
    
   


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
    #rate = rospy.Rate(30) # 10hz
    #print("About to call callback")
    rospy.Subscriber('/scan', LaserScan, rangeRegionCollector)
    #rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    


def publisher():

        #rospy.loginfo(pose_msg)
        #print("About to publish: (",pose_msg.x, ", ", pose_msg.theta,")")
        
    pub.publish(pose_msg)
   # print("And published")
    rospy.spin()
        #rate.sleep()
        #listener()
        #print("Finished sleeping")
    
    #rospy.spin()

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    #q_table = 
    pose_msg = Pose2D()
    pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
    rospy.init_node('wall_following_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('/scan', LaserScan, rangeRegionCollector)

    try:
        while not rospy.is_shutdown():
           # print("Start of iteration")
            pub.publish(pose_msg)
            #print("After listener")
            #rospy.spin()
            rate.sleep()
            #print("End iteration")
            
    except rospy.ROSInterruptException:
        pose_msg.x = 0
        pose_msg.y = 0
        pose_msg.theta =0
        pub.publish(pose_msg)
