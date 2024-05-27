#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
        laserscan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/base_scan", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        // Create a LaserScan message
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header = cloud_msg->header;
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_laserscan");

    PointCloudToLaserScan converter;

    ros::spin();

    return 0;
}