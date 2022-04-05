#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
import numpy as np
import pprint
import pickle
np.set_printoptions(precision=3)

def loadQTableFromFile():
    pass

def writeQTableToFile():
    pass


def goForward():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = 0  


def turnLeft():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = np.pi/5


def turnRight():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = -np.pi/5


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
    global front_ofa
    global right_ofa
    global personal_space_radius
    front_ofa = np.amin(front_beams)
    right_ofa = np.amin(right_beams)
    personal_space_radius = np.amin(lidar_data.ranges)
    # Output position to terminal
    # print("Min front is: ", np.round(front_ofa, 2), ", Min right is: ", np.round(right_ofa, 2))

    
    

def actionSelectionUsingQTable(state):
    action_row = Q_table[state]
    action = action_row.index(max(action_row))
    if action == 0:
        print("Go forward")
        goForward()
    elif action == 1:
        print("Turning left")
        turnLeft()
    elif action == 2:
        print("Turning right")
        turnRight()
    return action

def selectRandomAction():
    # Generate random number
    r = np.random.uniform(low=0.0,high=1.0)
    action = 0

    if r <= (1.0/3.0):
        print("Go forward")
        goForward()
        action = 0

    elif r > 1.0/3.0 and r <= 2.0/3.0:
        print("Turning left")
        turnLeft()
        action = 1

    else:
        print("Turning right")
        turnRight()
        action = 2

    return action


def selectActionUsingEpsGreedyPolicy(state, episode_number):



    # Define parameters
    epsilon_0 = 0.9
    d = 0.985

    # Generate random number
    r = np.random.uniform(low=0.0,high=1.0)
    epsilon = epsilon_0 * pow(d,episode_number)
    print("Epsilon: ", epsilon)

    if r > epsilon :
       print("Selecting Q Table action")
       action = actionSelectionUsingQTable(state)
    else:
       print("Selecting random action")
       action = selectRandomAction()

    return action
    


   # Used for Deliverable #1
def stateEstimator(front_ofa, right_ofa, personal_space_radius):
   
    # Define states
    # Define front region threshold
    front_threshold = 0.9
    front_horizon = 1.8
    # Define right goldilocks threshld start and end, as well as a horizon threshold
    right_goldilocks_begin = 0.7
    right_goldilocks_end = 0.8
    right_horizon = 1.0
    print("Personal space radius: ", np.round(personal_space_radius, 2))
    print("Min front is: ", np.round(front_ofa, 2), ", Min right is: ", np.round(right_ofa, 2))

    # Set state
    if personal_space_radius < 0.15 or math.isinf(abs(front_ofa)) or math.isinf(abs(right_ofa)):
        state = 'm' # Abort state
        
    else: # If not crashed
        if (front_ofa < front_threshold): # State 21
            if (right_ofa < right_goldilocks_begin): # State 11
                state = 'a'
                #print("Updating to state A")
            elif (right_ofa > right_goldilocks_begin) and (right_ofa < right_goldilocks_end): # State 12
                state = 'd'
            elif (right_ofa > right_goldilocks_end) and (right_ofa < right_horizon): # State 13
                state = 'g'
            elif (right_ofa > right_horizon): # State 14
                state = 'j'
        elif (front_ofa > front_threshold) and (front_ofa < front_horizon): # State 22
            if (right_ofa < right_goldilocks_begin): # State 11
                state = 'b'
            elif (right_ofa > right_goldilocks_begin) and (right_ofa < right_goldilocks_end): # State 12
                state = 'e'
            elif (right_ofa > right_goldilocks_end) and (right_ofa < right_horizon): # State 13
                state = 'h'
            elif (right_ofa > right_horizon): # State 14
                state = 'k'
        elif (front_ofa > front_horizon): # State 23
            if (right_ofa < right_goldilocks_begin): # State 11
                state = 'c'
            elif (right_ofa > right_goldilocks_begin) and (right_ofa < right_goldilocks_end): # State 12
                state = 'f'
            elif (right_ofa > right_goldilocks_end) and (right_ofa < right_horizon): # State 13
                state = 'i'
            elif (right_ofa > right_horizon): # State 14
                state = 'l'
    
    return state


def calculateReward(state, action):
    if state == 'e' or state == 'f': # Goldilocks right/ Front far and medium
        reward = 25

    elif state == 'd': # Goldilocks right/ Front too close
        reward = 5
    
    elif state == 'b' or state == 'c': # Too close right / Front far and medium
        reward = 5

    elif state == 'a': # Too close right / Too close front
        reward = -3
    
    elif state == 'h' or state == 'i': # Too far right / Front far and medium
        reward = 5
    
    elif state == 'g': # Too far right / Too close front
        reward = -1

    elif state == 'j' or state == 'k': # Too far right / Front close and medium
        reward = 1

    elif state == 'l': # In the middle of nowhere
        reward = -1

    elif state == 'm':
        reward = -5

    else:
        reward = -1
        
    return reward


def updateQTableUsingQLearning(Q_table, state, action, previous_state):
    # Define parameters
    alpha = 0.2
    gamma = 0.8
    reward = calculateReward(state, action)
    # print("state: ", state)
    # print("action: ", action)
    # print('F,L,R: ', type(Q_table[state][action]))
    # print("Reward: ", reward)
    previous_q_table_entry = Q_table[previous_state][action]
    Q_table[previous_state][action] = Q_table[previous_state][action] + alpha*(reward + (gamma*max(Q_table[state])) - Q_table[previous_state][action])
    print("Training Residual: ", (Q_table[previous_state][action] - previous_q_table_entry))




if __name__ == '__main__':

    pose_msg = Pose2D()
    # Start from scratch
#     Q_table = {
#     'a' : [0,0,0],
#     'b' : [0,0,0],
#     'c' : [0,0,0],
#     'd' : [0,0,0],
#     'e' : [0,0,0],
#     'f' : [0,0,0],
#     'g' : [0,0,0],
#     'h' : [0,0,0],
#     'i' : [0,0,0],
#     'j' : [0,0,0],
#     'k' : [0,0,0],
#     'l' : [0,0,0],
#     'm' : [0,0,0]
# }
    # Re-initialize Q-Table from saved state
     
    with open('Q_learning_saved_state.pickle', 'rb') as handle:
        Q_table = pickle.load(handle)
    # Global state variable
    previous_state = 'm'
    state = 'm' # Initialized to abort state

    # Episode number
    #episode_number = 0 # From scratch

    with open('episode_number_saved_state.pickle', 'rb') as handle:
        episode_number = pickle.load(handle)

    front_ofa = 0.0
    right_ofa = 0.0
    personal_space_radius = 0.0

    # Instantiate the publisher and initialize the node
    pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
    reset_triton = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    
   

    rospy.init_node('wall_following_controller', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    rospy.Subscriber('/scan', LaserScan, rangeRegionCollector)
    # Using the Q-Learning reinforcement learning algorithm
    # Initialize the Q-Table
    pp = pprint.PrettyPrinter(indent=4)
    
    try:
        while not rospy.is_shutdown():
            # Reinforcement Learning algorithm
            """ if episode_number == 0 or state == 'm':
            # Reset robot to random pose
                random_pose.pose.orientation.z = np.random.uniform(0,2*np.pi)
                reset_triton.publish(random_pose)
            else: """
            previous_state = stateEstimator(front_ofa, right_ofa, personal_space_radius)
            print("State returned is: ", previous_state)
                # Reset robot if crashed
            random_pose = ModelState()
            random_pose.model_name = 'triton_lidar'
            random_pose.pose.position.x = np.random.uniform(-3.4, 3.4)
            random_pose.pose.position.y = np.random.uniform(-3.4, 3.4)
            random_pose.pose.orientation.z = np.random.uniform(0,2*np.pi)

            if previous_state == 'm':
                rospy.wait_for_service('/gazebo/set_model_state')
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(random_pose)
                episode_number = episode_number + 1
                print("State M - Resetting")

            action = selectActionUsingEpsGreedyPolicy(previous_state, episode_number)
            print("Episode number: ", episode_number)

            pub.publish(pose_msg)
            rate.sleep()

            # Observe new state, calculate reward, and update Q table
            state = stateEstimator(front_ofa, right_ofa, personal_space_radius)
            updateQTableUsingQLearning(Q_table, state=state, action=action, previous_state=previous_state)
            # updateQTableUsingSARSA(Q_table)
            pp.pprint(Q_table)

             # Store the Q Table
            if episode_number % 30 == 0:
                with open('Q_learning_saved_state.pickle', 'wb') as handle:
                    pickle.dump(Q_table, handle, protocol=pickle.HIGHEST_PROTOCOL)
                with open('episode_number_saved_state.pickle', 'wb') as handle:
                    pickle.dump(episode_number, handle, protocol=pickle.HIGHEST_PROTOCOL)
                    print("Pickling the Q Table")

    except rospy.ROSInterruptException:
        pass
