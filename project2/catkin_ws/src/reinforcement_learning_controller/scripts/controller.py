#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
import numpy as np
import collections
import pprint
import pickle
np.set_printoptions(precision=3)

def goForward():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = 0  


def turnSharpLeft():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = np.pi/3

def turnSharpRight():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = -np.pi/3

def turnGentleLeft():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = np.pi/9

def turnGentleRight():
    pose_msg.x = 0.15
    pose_msg.y = 0
    pose_msg.theta = -np.pi/9



def rangeRegionCollector(lidar_data): 
    num_beams = 360

    # Front region is defined as the beams between num_beams/16 and 15*num_beams/16
    front_beams = np.zeros(0,float)
    # Right region is defined as the beams  between 11*num_beams/16 and 13*num_beams/16
    right_beams = np.zeros(0,float)

    corner_beams = np.zeros(0,float)

    left_beams = np.zeros(0,float)

    # Collect the front, right, right corner, and front beams
    for i in range(num_beams):      
       if (i <= (num_beams/16)) or (i >= (15*num_beams/16)): # Front region
           front_beams = np.append(front_beams, lidar_data.ranges[i])

       elif (i >= (2 * num_beams / 16)) and (i < (7 * num_beams / 16)) : # Left region
           left_beams = np.append(left_beams, lidar_data.ranges[i])

       elif (i >= (10 * num_beams / 16)) and (i < (14 * num_beams / 16)) : # Right region
           right_beams = np.append(right_beams, lidar_data.ranges[i])

           if (i >= (12 * num_beams / 16)) and (i < (14 * num_beams / 16)) : # Corner region
                corner_beams = np.append(corner_beams, lidar_data.ranges[i])
        
       

    # Determine the shortest distance within the fov          
    global front_ofa
    global right_ofa
    global corner_ofa
    global left_ofa
    global personal_space_radius
    front_ofa = np.amin(front_beams)
    right_ofa = np.amin(right_beams)
    left_ofa = np.amin(left_beams)
    corner_ofa = np.amin(corner_beams)
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
        print("Hardly turning left")
        turnGentleLeft()
    elif action == 2:
        print("Hardly turning right")
        turnGentleRight()
    elif action == 3:
        print("Turning left hardly")
        turnSharpLeft()
    elif action == 4:
        print("Turning right hardly")
        turnSharpRight()
    return action

def selectRandomAction():
    # Generate random number
    r = np.random.uniform(low=0.0,high=1.0)
    action = 0

    if r <= (1.0/5.0):
        print("Go forward")
        goForward()
        action = 0

    elif r > 1.0/5.0 and r <= 2.0/5.0:
        print("Hardly turning left")
        turnGentleLeft()
        action = 1

    elif r > 2.0/5.0 and r <= 3.0/5.0:
        print("Hardly turning right")
        turnGentleRight()
        action = 2

    elif r > 3.0/5.0 and r <= 4.0/5.0:
        print("Turning left hardly")
        turnSharpLeft()
        action = 3

    elif r > 4.0/5.0 and r <= 5.0/5.0:
        print("Turning right hardly")
        turnSharpRight()
        action = 4

    return action


def selectActionUsingEpsGreedyPolicy(state, episode_number):

    # Define parameters
    epsilon_0 = 0.9
    d = 0.985

    if state not in Q_table:
        print("New state observed, appending ", state, " to Q table")
        Q_table[state] = [0.0, 0.0, 0.0, 0.0, 0.0]  # Forward, Gentle left, Gentle right, Sharp left, Sharp right

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
def stateEstimator(front_ofa, right_ofa, left_ofa, corner_ofa, personal_space_radius):
   
    # Define states
    # Define front region threshold
    front_too_close = 0.3
    front_close = 0.6
    front_medium = 1.0
    

    # Define right goldilocks threshld start and end, as well as a horizon threshold
    right_too_close = 0.3
    right_close = 0.6
    right_medium = 0.7
    right_far = 0.8


    # Define corner threshold
    corner_threshold = 1.5
    left_threshold = 0.3

    state = 'abort'
    #print("Personal space radius: ", np.round(personal_space_radius, 2))
    print("R, C, F, L: ", np.round(right_ofa, 2), np.round(corner_ofa, 2), np.round(front_ofa, 2), np.round(left_ofa, 2))
    
    # Set state
    if personal_space_radius < 0.15 or math.isinf(abs(front_ofa)) or math.isinf(abs(right_ofa)):
        state = 'abort' # Abort state
    else:
        if right_ofa <= right_too_close:  # State 1000 series
            right_state = 1
        elif (right_ofa > right_too_close) and (right_ofa <= right_close): # State 2000
            right_state = 2
        elif (right_ofa > right_close) and (right_ofa <= right_medium): #State 3000 
            right_state = 3
        elif (right_ofa > right_medium) and right_ofa <= right_far: # State 4000 and Reward state!
            right_state = 4
        elif (right_ofa > right_far): # State 5000
            right_state = 5
     

        if corner_ofa < corner_threshold: # State 100
            corner_state = 1
        elif corner_ofa >= corner_threshold: ## State 200
            corner_state = 2 

        if front_ofa <= front_too_close: # State 10
            front_state = 1
        elif front_ofa > front_too_close and front_ofa <= front_close: # State 20
            front_state = 2
        elif front_ofa > front_close and front_ofa <= front_medium: # State 30
            front_state = 3
        elif front_ofa > front_medium: # State 40
            front_state = 4
        
        if left_ofa < left_threshold:
            left_state = 1
        else:
            left_state = 2

        # Combine states
        state = str(1000*right_state + 100*corner_state + 10*front_state + 1*left_state)

    return state


def calculateReward(state, action):
    
    reward = -1

    if state[0] == '4' and state[2] != '1':
        reward = 0

    elif state == 'abort':
        reward = -2
    
        
    print("Getting reward of ", reward)
    return reward


def updateQTableUsingQLearning(Q_table, state, action, previous_state):
    # Define parameters
    gamma = 0.8
    reward = calculateReward(state, action)
    global episode_number
    global accumulated_rewards
    alpha = 0.2

    previous_q_table_entry = Q_table[previous_state][action]
    Q_table[previous_state][action] = Q_table[previous_state][action] + alpha*(reward + (gamma*max(Q_table[state])) - Q_table[previous_state][action])
    print("Training Residual: ", (Q_table[previous_state][action] - previous_q_table_entry))
    accumulated_rewards = accumulated_rewards + reward


def updateQTableUsingSARSA(Q_table, state, action, previous_state, previous_action):
    # Define parameters
    gamma = 0.8
    reward = calculateReward(state, action)
    global episode_number
    alpha = 0.3

    previous_q_table_entry = Q_table[previous_state][previous_action]
    Q_table[previous_state][previous_action] = Q_table[state][action] + alpha*(reward + (gamma*(Q_table[state][action])) - Q_table[previous_state][previous_action])
    print("Training Residual: ", (Q_table[previous_state][previous_action] - previous_q_table_entry))


if __name__ == '__main__':

    pose_msg = Pose2D()
    # Global state variable
    previous_state = 'abort'
    state = 'abort' # Initialized to abort state
    front_ofa = 0.0
    right_ofa = 0.0
    left_ofa = 0.0
    corner_ofa = 0.0
    personal_space_radius = 0.0

    # Start from scratch
    #Q_table = {}

#     # Episode number
    #episode_number = 0 # From scratch

    # Re-initialize Q-Table from saved state
     
    with open('Q_learning_saved_state_highLR.pickle', 'rb') as handle:
        Q_table = pickle.load(handle)
    with open('episode_number_saved_state_highLR.pickle', 'rb') as handle:
        episode_number = pickle.load(handle)


    # Instantiate the publisher and initialize the node
    pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
    reset_triton = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    
   

    rospy.init_node('wall_following_controller', anonymous=True)
    rate = rospy.Rate(3) # 5hz
    rospy.Subscriber('/scan', LaserScan, rangeRegionCollector)
    # Using the Q-Learning reinforcement learning algorithm
    # Initialize the Q-Table
    pp = pprint.PrettyPrinter(indent=4)
    iteration = 0
    iteration_deque = collections.deque()
    accumulated_rewards_deque = collections.deque()
    accumulated_rewards = 0.0
    try:
        while not rospy.is_shutdown():
            # Reinforcement Learning algorithm

            previous_state = stateEstimator(front_ofa, right_ofa, left_ofa, corner_ofa, personal_space_radius)
            print("State returned is: ", previous_state)
                # Reset robot if crashed
            random_pose = ModelState()
            random_pose.model_name = 'triton_lidar'
            random_pose.pose.position.x = np.random.uniform(-3.4, 3.4)
            random_pose.pose.position.y = np.random.uniform(-3.4, 3.4)
            random_pose.pose.orientation.z = np.random.uniform(0,2*np.pi)
            #print("Random orientation: ", random_pose.pose.orientation.z)
            


            if (previous_state == 'abort' and iteration != 1) or iteration == 2000:
                rospy.wait_for_service('/gazebo/set_model_state')
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(random_pose)
                episode_number = episode_number + 1
                iteration_deque.appendleft(iteration)
                iteration_deque.appendleft(iteration)
                accumulated_rewards_deque.appendleft(float(accumulated_rewards) / float(iteration+1))
                with open('QLearning_training_metric1.pickle', 'wb') as handle:
                    pickle.dump(accumulated_rewards_deque, handle, protocol=pickle.HIGHEST_PROTOCOL)
                if len(iteration_deque) > 10:
                    iteration_deque.pop()

                accumulated_rewards = 0
                iteration = 0
                print("State Abort - Resetting")
            

            print("Iteration: ", iteration)
            print("The last 10 sims have run for ", iteration_deque)
            action = selectActionUsingEpsGreedyPolicy(previous_state, episode_number)
            print("Episode number: ", episode_number)

            pub.publish(pose_msg)
            iteration = iteration + 1
            print("Accumulated rewards: ", float(accumulated_rewards) / float(iteration))
            rate.sleep()

            # Observe new state, calculate reward, and update Q table
            state = stateEstimator(front_ofa, right_ofa, left_ofa, corner_ofa, personal_space_radius)
            updateQTableUsingQLearning(Q_table, state=state, action=action, previous_state=previous_state)
            # updateQTableUsingSARSA(Q_table)
            #pp.pprint(Q_table)

             # Store the Q Table
            if episode_number % 10 == 0 or iteration == 600:
                with open('Q_learning_saved_state_highLR.pickle', 'wb') as handle:
                    pickle.dump(Q_table, handle, protocol=pickle.HIGHEST_PROTOCOL)
                with open('episode_number_saved_state_highLR.pickle', 'wb') as handle:
                    pickle.dump(episode_number, handle, protocol=pickle.HIGHEST_PROTOCOL)
                    print("Pickling the Q Table")

    except rospy.ROSInterruptException:
        pass
