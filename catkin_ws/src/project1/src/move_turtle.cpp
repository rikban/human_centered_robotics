#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <math.h>

// Topic messages callback
/* void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    ROS_INFO("x: %.2f, y: %.2f, rot: %.2f", msg->x, msg->y, msg->);
} */

struct pose
{
    float x;
    float y;
    float orientation;
};
/* class Turtle
{
    public:
        Turtle(){}
       void updateCurrentTurtlePose(const turtlesim::PoseConstPtr& current_pose)
        {
            turtle_pose.x = current_pose->x;
            turtle_pose.y = current_pose->y;
            turtle_pose.orientation = current_pose->theta;
        }

        pose turtle_pose;

}; */
static pose current_turtle_pose = {0,0,0};
//pose current_turtle_pose = {0,0,0};
void poseCallback(const turtlesim::PoseConstPtr& current_pose)
{
   // ROS_INFO("I heard: [%s]", current_pose->data.c_str());
   /*  ROS_INFO("x: %.2f, y: %.2f", current_pose->x, current_pose->y); */

    current_turtle_pose.x = current_pose->x;
    current_turtle_pose.y = current_pose->y;
    current_turtle_pose.orientation = current_pose->theta;
    
}
float errorBetweenCurrentAndGoalPose(pose current_pose, pose goal_pose)
{
    float pose_norm = 0;
    pose_norm = sqrt(pow((goal_pose.x - current_pose.x), 2) + pow((goal_pose.y - current_pose.y), 2));
    std::cout<< "Norm of the pose error: "<<pose_norm<<std::endl;
    return pose_norm;
}
float calculateLinearVelocity(float pose_error)
{
    float proportional_linear_gain = 0.5;
    return proportional_linear_gain*pose_error;
}

float calculateAngularVelocity(float orientation_error)
{
    float proportional_angular_gain = 0.5;
    return proportional_angular_gain*orientation_error;
}
int main(int argc, char **argv)
{
   // const double FORWARD_SPEED_MPS = 0.5;
    // Define trajectory points
    const int num_trajectory_points = 5;
    int goal_point = 0;
/*     std::array<pose, num_trajectory_points>goal = {{1,1, M_PI/2},
                                                    {1,5, 0},
                                                    {5,5, -M_PI/2},
                                                    {5,1, -M_PI},
                                                    {1,1, M_PI/2}}; */
    pose goal[5] = {{1,1, M_PI/2},
                    {1,5, 0},
                    {5,5, -M_PI/2},
                    {5,1, -M_PI},
                    {1,1, M_PI/2}};

   // Turtle fred;
    // Initialize the node
    ros::init(argc, argv, "move_turtle");
    ros::NodeHandle node;
    // A publisher for the movement data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    // A listener for pose
    

    // Drive forward at a given speed. The robot points up the x-axis.
    // The default constructor will set all commands to 0
    geometry_msgs::Twist velocity_command;
    //velocity_command.linear.x = 5;
   // pub.publish(velocity_command);
    // Loop at 10Hz, publishing movement commands until we shut down
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");
    ros::Subscriber sub = node.subscribe("/turtle1/pose", 100, poseCallback);

    uint8_t counter = 0;
    while(ros::ok() && (goal_point < num_trajectory_points))
    {
        // RMS error between goal position and current position
        float pose_error = 15.0;
        // Difference between current orientation and desired heading
        float heading_error = M_PI/2;
        float orientation_error = M_PI/2;
        std::cout<< "Goal point: (" << goal[goal_point].x <<", "<< goal[goal_point].y<<")"<<std::endl;

        while(pose_error > 0.01)
        {
            float desired_heading = atan2(( goal[goal_point].y - current_turtle_pose.y),
                                         ( goal[goal_point].x - current_turtle_pose.x));
           
            ROS_INFO("Desired Heading: %.2f", desired_heading);
            //(M_PI + atan2(goal[goal_point].y, goal[goal_point].x));
          
            while(abs(orientation_error) > 0.01 )
            {

                float desired_heading = atan2(-(current_turtle_pose.y - goal[goal_point].y),
                                         -(current_turtle_pose.x - goal[goal_point].x));
           
                ROS_INFO("Desired Heading: %.2f", desired_heading);
                orientation_error = desired_heading - current_turtle_pose.orientation;

                std::cout<<"orientation error: "<< orientation_error << std::endl;
                std::cout<< "Fred thinks he's at: "<< current_turtle_pose.x<< ", "
                     <<current_turtle_pose.y<<", "<< current_turtle_pose.orientation<<std::endl;
                velocity_command.linear.x = 0;
                velocity_command.angular.x = 0.0;
                velocity_command.angular.y = 0.0;
                velocity_command.angular.z = calculateAngularVelocity(orientation_error);
                pub.publish(velocity_command);
                ros::spinOnce();
                rate.sleep();
            }
            //std::cout<< "I'm oriented and headed to the next trajectory point!!"<< std::endl;
            pose_error = errorBetweenCurrentAndGoalPose(current_turtle_pose, goal[goal_point]);
            
            //ROS_INFO("Pose error: %.2f", pose_error);
           //  std::cout<< "Fred thinks he's at: "<< current_turtle_pose.x<< ", "
           //          <<current_turtle_pose.y<<", "<< current_turtle_pose.orientation<<std::endl;

            //velocity_command.linear.x = calculateLinearVelocity(pose_error);
            velocity_command.linear.x = calculateLinearVelocity(pose_error);
            velocity_command.linear.y = 0.0;
            velocity_command.linear.z = 0.0;
            velocity_command.angular.x = 0.0;
            velocity_command.angular.y = 0.0;
            velocity_command.angular.z = calculateAngularVelocity(orientation_error);


            pub.publish(velocity_command);
           // std::cout<< "Requested: "<<velocity_command.linear.x << " forward and " 
            //                         << velocity_command.angular.z << " about z"<<std::endl;
            std::cout<< "Fred thinks he's at: "<< current_turtle_pose.x<< ", "
                     <<current_turtle_pose.y<<", "<< current_turtle_pose.orientation<<std::endl;
            //++counter;
            ros::spinOnce();
            rate.sleep();
        }
        

       // pose_error = errorBetweenCurrentAndGoalPose(current_turtle_pose, goal[goal_point]);
        velocity_command.linear.x = 0.0;
        pub.publish(velocity_command);
        ros::spinOnce();
        rate.sleep();
        if (goal_point == 0)
        {

        }
        ++goal_point;
        pose_error = errorBetweenCurrentAndGoalPose(current_turtle_pose, goal[goal_point]);
        ROS_INFO("Heading to the next trajectory point!");

    }
     // Allow processing of incoming messages
    

}