#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

/*

write a node called odom_to_tf which subscribe to:
    - odometry message:
        - type: nav_msgs/Odometry
        - topic name: input_odom (this is a generic name, you are
          supposed to use remapping to subscribe to the correct topic)

    - publish with a tf broadcaster an odometry between two values set
      as node parameters:
        - the two node parameters are called "root_frame" and "child_frame"
        - the node is supposed to be started from launch file with topic
          remapping for input and parameter for the tf_broardcaster

*/

class odom_to_tf
{

private:
    ros::NodeHandle n;
    ros::NodeHandle private_n; // local node handler
    ros::Subscriber sub;
    tf::TransformBroadcaster odom_broadcaster;

    std::string input_odom;  // generic string for input topic, to set using launch.launch (local)
    std::string root_frame;  // to set using launch.launch, must be equal to "world" (global)
    std::string child_frame; // to set using launch.launch: "wheel_odom" or "gps_odom" (local)

    ros::Time current_time;

    void initialize_parameters() {
        if (!private_n.getParam("input_odom", input_odom)) {
            ROS_ERROR("Failed to get parameter input_odom");
        }
        if (!private_n.getParam("child_frame", child_frame)) {
            ROS_ERROR("Failed to get parameter child_frame");
        }
        if (!n.getParam("root_frame", root_frame)) {
            ROS_ERROR("Failed to get parameter root_frame");
        }
    }

public:
    odom_to_tf() : private_n("~")
    {
        initialize_parameters();

        sub = n.subscribe<nav_msgs::Odometry>(input_odom, 1, &odom_to_tf::callback, this );

    
        ros::Rate loop_rate(5);

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void callback(nav_msgs::Odometry msg)
    {

        tf::Transform transform; // tf message

        current_time = ros::Time::now();

        // Extracting position and orientation from odometry message
        tf::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);

        tf::Vector3 origin(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            0.0);

        transform.setOrigin(origin);
        transform.setRotation(q);

        odom_broadcaster.sendTransform(tf::StampedTransform(transform, current_time, root_frame, child_frame));
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_tf");
    odom_to_tf odom_to_tf;
    ros::spin();
    return 0;
}