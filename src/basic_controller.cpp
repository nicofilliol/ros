#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"


class TurtleController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::ServiceServer pause_server;
    ros::Subscriber pose_sub;

    turtlesim::Pose latest_pose;

    float linear_vel;
    float angular_vel;

    bool is_moving;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        if (this->is_moving) { 
            // Update the control values
            this->commandUpdate();
            
            msg.linear.x = linear_vel; // move forward (m/s -> unit of measure convention)
            msg.angular.z = angular_vel; // turn counterclockwise (rad/s -> unit of measure convention)
        }
        
        return msg;
    }

    bool flip(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep) {
        this->is_moving = !(this->is_moving);

        return true;
    }

    void poseUpdate(const turtlesim::Pose::ConstPtr &msg) {
        this->latest_pose.x = msg->x;
        this->latest_pose.y = msg->y;
        this->latest_pose.theta = msg->theta;
        this->latest_pose.linear_velocity = msg->theta;
        this->latest_pose.angular_velocity = msg->theta;
    }

    void commandUpdate() {
        this->linear_vel = this->latest_pose.x;
        //this->angular_vel = 0;
    }

public:
    TurtleController(){
        this->is_moving = true;
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5); // size of queue

        // Create a service server
        this->pause_server = this->n.advertiseService("pause_service", &TurtleController::flip, this);


        // Read parameters from server
        auto priv_nh = ros::NodeHandle("~"); // node handle declared in private space
        priv_nh.getParam("linear_vel", this->linear_vel);
        priv_nh.getParam("angular_vel", this->angular_vel);

        // Get position of turtle
        this->pose_sub = this->n.subscribe("pose", 10, &TurtleController::poseUpdate, this);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            // Allows other threads to be launched and processes callbacks in queue
            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Create our controller object and run it
    auto controller = TurtleController();
    controller.run();

    // And make good on our promise
    return 0;
}