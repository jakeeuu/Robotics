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

/*

publish:
- odometry message:
    - type: nav_msgs/Odometry
    - topic name: gps_odom

- to convert gps data to odometry:
    - convert (latitude-longitude-altitude) to Cartesian ECEF
    - convert Cartesian ECEF to ENU
    - ENU is a relative position, so you need to specify the reference point

- gps_to_odom have three parameters :
    - lat_r
    - lon_r
    - alt_r
- These parameters are set in the launch file

*/

std::vector<double> vectorial_mult(const std::vector<std::vector<double>> &matrix, const std::vector<double> &vect)
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

class gps_coord
{

public:

    double latitude;  // in radiants
    double longitude; // in radiants
    double altitude;  // in radiants

    gps_coord() {}
    gps_coord(double lat, double lng, double alt) // input coords must be in degrees
    {
        this->latitude = deg2rad(lat);
        this->longitude = deg2rad(lng);
        this->altitude = alt;
    }

private:
    double deg2rad(double grad)
    {
        double rad = grad * 3.14159 / 180;
        return rad;
    }
};

class ECEF_coord
{

public:
    double x;
    double y;
    double z;

    ECEF_coord() {}
    ECEF_coord(const gps_coord &gps)
    {
        init_const(gps.latitude);

        x = (N + gps.altitude) * cos(gps.latitude) * cos(gps.longitude);
        y = (N + gps.altitude) * cos(gps.latitude) * sin(gps.longitude);
        z = (N * (1 - e2) + gps.altitude) * sin(gps.latitude);
    }
    ECEF_coord(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

private:
    double a = 6378137;
    double b = 6356752;
    double N;
    double e2;

    void init_const(double latitude_in_rad)
    { // initialize constants
        e2 = get_e2();
        N = get_N(latitude_in_rad);
    }

    double get_N(double latitude)
    {

        double N;
        N = a / sqrt(1 - e2 * sin(latitude) * sin(latitude));

        return N;
    }

    double get_e2()
    {
        double e2;
        e2 = 1 - (b * b) / (a * a);
        return e2;
    }
};

class Ref_syst
{
public:
    ECEF_coord ecef;
    gps_coord gps;
    std::vector<std::vector<double>> matrix;
    Ref_syst() {}
    Ref_syst(const gps_coord &gps_ref_syst, const ECEF_coord &ecef_ref_syst) : gps(gps_ref_syst), ecef(ecef_ref_syst), matrix(get_matrix(gps_ref_syst)) {}

private:
    std::vector<std::vector<double>> get_matrix(const gps_coord &gps)
    {
        std::vector<std::vector<double>> matrix{3, std::vector<double>(3)};

        double lambda;
        double theta;

        lambda = gps.longitude;
        theta = gps.latitude;

        matrix[0][0] = -sin(lambda);
        matrix[0][1] = cos(lambda);
        matrix[0][2] = 0;

        matrix[1][0] = -sin(theta) * cos(lambda);
        matrix[1][1] = -sin(theta) * sin(lambda);
        matrix[1][2] = cos(theta);

        matrix[2][0] = cos(theta) * cos(lambda);
        matrix[2][1] = cos(theta) * sin(lambda);
        matrix[2][2] = sin(theta);

        return matrix;
    }
};

class ENU_coord
{
private:
    double rotation_angle = 128; // angle in degrees
    std::vector<double> rotate(double x, double y)
    {
        double angle = rotation_angle * 3.14159 / 180;
        std::vector<double> res;
        res.push_back(x * cos(angle) - y * sin(angle));
        res.push_back(x * sin(angle) + y * cos(angle));

        return res;
    }

public:
    double x;
    double y;
    double z;

    ENU_coord() {}
    ENU_coord(const ECEF_coord &ecef, const Ref_syst &ref_syst)
    { // input coords must be in degrees

        std::vector<double> diff(3);
        std::vector<double> res(3);
        diff[0] = ecef.x - ref_syst.ecef.x;
        diff[1] = ecef.y - ref_syst.ecef.y;
        diff[2] = ecef.z - ref_syst.ecef.z;

        res = vectorial_mult(ref_syst.matrix, diff);

        // result without rotation adjustments
        x = res[0];
        y = res[1];
        z = res[2];

        // now apply the rotation
        res=rotate(x, y);
        x = res[0];
        y = res[1];

    }
    ENU_coord(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

class gps_to_odom // class of the node
{

private:
    geometry_msgs::Quaternion get_orientation(ENU_coord &curr_pose, ENU_coord &prec_pose)
    {
        double dx = curr_pose.x - prec_pose.x;
        double dy = curr_pose.y - prec_pose.y;
        double dz = curr_pose.z - prec_pose.z;

        // Calculate yaw (heading) angle
        double yaw = atan2(dy, dx);

        // Calculate pitch angle, the pitch is not necessary
        double pitch = atan2(-dz, sqrt(dx * dx + dy * dy));

        // Convert Euler angles to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        // Convert tf2 quaternion to geometry_msgs quaternion
        geometry_msgs::Quaternion orientation;
        tf2::convert(quat, orientation);

        return orientation;
    }

public:
    ros::NodeHandle n;
    ros::NodeHandle private_n;
    ros::Subscriber sub;
    ros::Publisher pub;

    int odom_seq_id;

    Ref_syst ref;
    ENU_coord prec_pose;

    gps_to_odom() : private_n("~")
    {

        sub = n.subscribe("/fix", 1, &gps_to_odom::callback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        set_ref();

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
        tf2::Quaternion quat_orientation;

        gps_coord gps(msg->latitude, msg->longitude, msg->altitude); // gps format of msgs
        ECEF_coord ecef(gps);                                        // ECEF format of msg
        ENU_coord enu(ecef, ref);                                    // ENU format of msg
                                  // it is calculated here because to compute the ENU is necessary the ref system

        // creating the odometry
        // header
        odom.header.seq = odom_seq_id;
        odom.header.stamp = ros::Time ::now();
        odom.header.frame_id = "gps_odom";
        odom.child_frame_id = "scout_link";
        // position
        odom.pose.pose.position.x = enu.x;
        odom.pose.pose.position.y = enu.y;
        odom.pose.pose.position.z = 0.0; // enu[2];
        // orientation
        odom.pose.pose.orientation = get_orientation(enu, prec_pose);

        pub.publish(odom);

        odom_seq_id += 1;
        prec_pose = enu;
        
    }

    void set_ref()
    {
        double lat_r;
        double lon_r;
        double alt_r;

        // retrive reference system coordinates
        if (!private_n.getParam("lat_r", lat_r))
        {
            ROS_ERROR("Failed to get parameter lat_r");
        }
        if (!private_n.getParam("lon_r", lon_r))
        {
            ROS_ERROR("Failed to get parameter lon_r");
        }
        if (!private_n.getParam("alt_r", alt_r))
        {
            ROS_ERROR("Failed to get parameter alt_r");
        }

        gps_coord gps_r(lat_r, lon_r, alt_r);
        ECEF_coord ecef_r(gps_r);
        ref = Ref_syst(gps_r, ecef_r);
        prec_pose = ENU_coord(ecef_r, ref);

        ROS_INFO("----------reference system\n lat=%f , long=%f, height=%f \n ecef: x=%f , y=%f , z=%f\n enu: x=%f , y=%f , z=%f\n", lat_r, lon_r, alt_r, ecef_r.x, ecef_r.y, ecef_r.z, prec_pose.x, prec_pose.y, prec_pose.z);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_to_odom");
    gps_to_odom gps_to_odom_node;
    ros::spin();
    return 0;
}