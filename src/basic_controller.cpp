#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


class TurtleController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        msg.linear.x = 2.0; // move forward (m/s -> unit of measure convention)
        msg.angular.z = 1.0; // turn counterclockwise (rad/s -> unit of measure convetion)

        return msg;
    }

public:
    TurtleController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
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