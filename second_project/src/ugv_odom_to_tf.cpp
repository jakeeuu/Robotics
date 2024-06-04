#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    static ros::Time last_stamp;

    // Only broadcast if the timestamp is new
    if (msg->header.stamp != last_stamp)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));

        last_stamp = msg->header.stamp;  // Update last_stamp
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ugv_odom_to_tf");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/ugv/odom", 10, odomCallback);

    ros::spin();

    return 0;
}
