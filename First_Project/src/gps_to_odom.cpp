#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class gps_to_odom
{

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    bool flag;

    // reference system data in radiants
    double ref_latitude;
    double ref_longitude;
    double ref_altitude;

    std::vector<double> ref_ecef{3, 0.0}; // ecef format of reference system
    std::vector<double>ref_enu{3, 0.0}; // enu format of reference system
    std::vector<std::vector<double>> ref_matrix{3, std::vector<double>(3)}; // Matrix to obtain ENU

    std::vector<double> prec_pose{3, 0}; //ENU format, precedent position of the robot, necessary to calculate orientation
    int odom_seq_id;

public:
    gps_to_odom()
    {
        flag = false;
        sub = n.subscribe("/fix", 1, &gps_to_odom::callback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        ros::Rate loop_rate(5);

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        nav_msgs::Odometry odom;

        double latitude = msg->latitude*3.14159/180;
        double longitude = msg->longitude*3.14159/180;
        double altitude = msg->altitude*3.14159/180;


        std::vector<double> ecef;
        std::vector<double> enu; //(x, y, z, orientation)
        tf2::Quaternion quat_orientation;

        if (!flag)
        {
            this->set_reference_system(latitude, longitude, altitude);
            ecef = this->from_gps_to_ECEF(latitude, longitude, altitude);
            prec_pose = this->from_ECEF_to_ENU(ecef);
            flag = true;
            odom_seq_id = 0;
            ROS_INFO("----------reference system\n lat=%f , long=%f, height=%f\n enu: x=%f , y=%f , z=%f\n", latitude, longitude, altitude, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);

        }

        ecef = this->from_gps_to_ECEF(latitude, longitude, altitude);
        enu = this->from_ECEF_to_ENU(ecef);

        // creating the odometry
        // header
        odom.header.seq = odom_seq_id;
        odom.header.stamp = ros::Time ::now();
        odom.header.frame_id = "gps_odom";
        odom.child_frame_id = "gps_odom";
        // position
        odom.pose.pose.position.x = enu[0];
        odom.pose.pose.position.y = enu[1];
        odom.pose.pose.position.z = 0.0; // enu[2];
        // orientation
        odom.pose.pose.orientation = get_orientation(enu, prec_pose);

        pub.publish(odom);

        ROS_INFO("-----------------------%d\n   input gps: lat=%f , long=%f, height=%f \n   enu: x=%f , y=%f , z=%f \n ",odom_seq_id, latitude, longitude, altitude, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);

        odom_seq_id += 1;
        prec_pose=enu;
    }

    std::vector<double> from_gps_to_ECEF(double latitude, double longitude, double altitude)
    {
        std::vector<double> ecef;

        int a = 6378137;
        int b = 6356752;

        double e_square = 1 - ((b * b) / (a * a));

        double n = this->n_calculation(latitude, e_square);

        ecef.push_back((n + altitude) * cos(latitude) * cos(longitude)); // x
        ecef.push_back((n + altitude) * cos(latitude) * sin(longitude)); // y
        ecef.push_back((n * (1 - e_square) + altitude) * sin(latitude)); // z

        return ecef;
    }

    std::vector<double> from_ECEF_to_ENU(std::vector<double> &ecef)
    {
        std::vector<double> enu;
        std::vector<double> pose_diff;

        pose_diff.push_back(ecef[0] - ref_latitude);
        pose_diff.push_back(ecef[1] - ref_longitude);
        pose_diff.push_back(ecef[2] - ref_altitude);

        enu = this->vectorial_mult(this->ref_matrix, pose_diff);

        return enu;
    }

    void set_reference_system(double latitude, double longitude, double altitude)
    {
        ref_latitude = latitude;   // theta in radiants
        ref_longitude = longitude; // lambda in radiants
        ref_altitude = altitude;   // h in meters

        ref_ecef = this->from_gps_to_ECEF(ref_latitude, ref_longitude, ref_altitude);
        ref_enu = this->from_ECEF_to_ENU(ref_ecef);

        // da ricontrollare assolutamenteeee!!!!!!!!!!!!!!!!!!!! gradi o radianti ??
        ref_matrix[0][0] = -sin(ref_longitude);
        ref_matrix[0][1] = cos(ref_longitude);
        ref_matrix[0][2] = 0;

        ref_matrix[1][0] = -sin(ref_latitude) * cos(ref_longitude);
        ref_matrix[1][1] = -sin(ref_latitude) * cos(ref_longitude);
        ref_matrix[1][2] = cos(ref_latitude);

        ref_matrix[2][0] = cos(ref_latitude) * cos(ref_longitude);
        ref_matrix[2][1] = cos(ref_latitude) * sin(ref_longitude);
        ref_matrix[2][2] = sin(ref_latitude);
    }

    double n_calculation(double phi, double e_square)
    {
        int a = 6378137;
        double n = a / sqrt(1 - e_square * sin(phi) * sin(phi));

        return n;
    }

    std::vector<double> vectorial_mult(std::vector<std::vector<double>> &matrix, std::vector<double> &vect)
    {
        std::vector<double> res(matrix.size(), 0);

        for (int i = 0; i < matrix.size(); i++)
        {
            for (int j = 0; j < matrix.size(); j++)
            {
                res[i] += matrix[i][j] * vect[j];
            }
        }
        return res;
    }

    geometry_msgs::Quaternion get_orientation(std::vector<double> curr_pose_enu, std::vector<double> prec_pose_enu)
    {
        double dx = curr_pose_enu[0] - prec_pose_enu[0];
        double dy = curr_pose_enu[1] - prec_pose_enu[1];
        double dz = curr_pose_enu[2] - prec_pose_enu[2];

        // Calculate yaw (heading) angle
        double yaw = atan2(dy, dx);

        // Calculate pitch angle
        double pitch = atan2(-dz, sqrt(dx * dx + dy * dy));
        
        // Convert Euler angles to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        // Convert tf2 quaternion to geometry_msgs quaternion
        geometry_msgs::Quaternion orientation;
        tf2::convert(quat, orientation);

        return orientation;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_to_odom");
    gps_to_odom gps_to_odom_node;
    ros::spin();
    return 0;
}