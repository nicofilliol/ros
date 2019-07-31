#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


class TurtleController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;

    float linear_vel;
    float angular_vel;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        msg.linear.x = linear_vel; // move forward (m/s -> unit of measure convention)
        msg.angular.z = angular_vel; // turn counterclockwise (rad/s -> unit of measure convention)

        return msg;
    }

public:
    TurtleController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5); // size of queue

        // Read parameters from server
        auto priv_nh = ros::NodeHandle("~"); // node handle declared in private space
        priv_nh.getParam("linear_vel", this->linear_vel);
        priv_nh.getParam("angular_vel", this->angular_vel);
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