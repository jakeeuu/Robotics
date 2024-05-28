#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

class PointCloudToLaserScan
{
public:
    PointCloudToLaserScan()
    {
        // Initialize ROS
        ros::NodeHandle nh;

        // Subscribe to the point cloud topic
        pointcloud_sub_ = nh.subscribe("/ugv/rslidar_points", 1, &PointCloudToLaserScan::pointCloudCallback, this);

        // Publisher for the laser scan topic
        laserscan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);

        // Frame parameters
        nh.param<std::string>("base_frame", base_frame_, "base_link");
        nh.param<std::string>("laser_frame", laser_frame_, "laser_frame");

        // Static transform between base and laser frames
        static_broadcaster_ = new tf::TransformBroadcaster();
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.1)); // Set the translation: x, y, z
        tf::Quaternion q;
        q.setRPY(0, 0, 0); // Set the rotation: roll, pitch, yaw
        transform.setRotation(q);
        static_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_, laser_frame_));
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        // Create a LaserScan message
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header = cloud_msg->header;
        scan_msg.header.frame_id = laser_frame_;
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0; // 1 degree
        scan_msg.range_min = 0.0;
        scan_msg.range_max = 100.0;

        uint32_t num_readings = std::ceil((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
        scan_msg.ranges.assign(num_readings, scan_msg.range_max + 1.0);

        for (const auto& point : cloud.points)
        {
            double angle = std::atan2(point.y, point.x);
            if (angle < scan_msg.angle_min || angle > scan_msg.angle_max)
                continue;

            int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;
            double range = std::sqrt(point.x * point.x + point.y * point.y);

            if (range < scan_msg.ranges[index])
            {
                scan_msg.ranges[index] = range;
            }
        }

        // Publish the LaserScan message
        laserscan_pub_.publish(scan_msg);
    }

private:
    ros::Subscriber pointcloud_sub_;
    ros::Publisher laserscan_pub_;
    tf::TransformBroadcaster* static_broadcaster_;
    std::string base_frame_;
    std::string laser_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_laserscan");

    PointCloudToLaserScan converter;

    ros::spin();

    return 0;
}