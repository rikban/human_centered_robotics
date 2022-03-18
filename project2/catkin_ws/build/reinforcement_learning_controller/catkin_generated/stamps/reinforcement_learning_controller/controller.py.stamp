#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np

def goForward():
    pose_msg.x = 0.1
    pose_msg.y = 0
    pose_msg.theta = 0  


def turnLeft():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = np.pi/10


def turnRight():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = -np.pi/10


def rangeRegionCollector(lidar_data): 
    num_beams = 360

    # Front region is defined as the beams between num_beams/16 and 15*num_beams/16
    front_beams = np.zeros(0,float)
    # Right region is defined as the beams  between 11*num_beams/16 and 13*num_beams/16
    right_beams = np.zeros(0,float)

    # Collect the front and right beams
    for i in range(num_beams):      
       if (i <= (num_beams/16)) or (i >= (15*num_beams/16)): # Front region
           front_beams = np.append(front_beams, lidar_data.ranges[i])

       elif (i >= (11 * num_beams / 16)) and (i < (13 * num_beams / 16)) : # Right region
           right_beams = np.append(right_beams, lidar_data.ranges[i])

    # Determine the shortest distance within the fov          
    front_ofa = np.amin(front_beams)
    right_ofa = np.amin(right_beams)
    # Output position to terminal
    print("Min front is: ", np.round(front_ofa,2), ", Min right is: ", np.round(right_ofa, 2))
    
    stateEstimator(front_ofa, right_ofa)
   
def stateEstimator(front_ofa, right_ofa):
   
    # Define states
    # Define front region threshold
    front_threshold = 0.9
    # Define right goldilocks threshld start and end, as well as a horizon threshold
    right_goldilocks_begin = 0.7
    right_goldilocks_end = 0.8
    right_horizon = 1.5

    # Set state
    if (front_ofa < front_threshold): # State 21
        # Turn left
        print("State 21- Turning left")
        turnLeft()
    else: # State 22
        if right_ofa < right_goldilocks_begin: # State 11, 22
            # Turn left
            print("State 11, 22 - Turning left")
            turnLeft()
        elif right_ofa > right_goldilocks_end and right_ofa < right_horizon: # State 13, 22
            # Turn right
            print("State 12, 22 - Turning right ")
            turnRight()
        elif right_ofa > right_horizon:
            goForward()
        else: # State 12, 22
            # Go forward
            print("State 12, 22 - Go forward")
            goForward()

if __name__ == '__main__':

    pose_msg = Pose2D()
    pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=40)
    rospy.init_node('wall_following_controller', anonymous=True)
    rate = rospy.Rate(40) # 40hz
    rospy.Subscriber('/scan', LaserScan, rangeRegionCollector)

    try:
        while not rospy.is_shutdown():

            pub.publish(pose_msg)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
