#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/dyn_conf_third_nodeConfig.h> //file generated from the python script

/*

Final node for lidar data visualization (called lidar_remap)

- The node subscribe to the /os_cloud_node/points topic and change the frame set in the header
- The value set in the header is regulated by a dynamic reconfigure callback which allow to dynamically change it to be set to the wheel_odom or gps_odom frame
- The final node allows the user to select from rqt_reconfigure to which tf the lidar is connected
- The node publish on the topic /pointcloud_remapped

*/




class lidar_remap
{
private:
    ros::NodeHandle n;
    ros::Subscriber lidar_sub;
    ros::Publisher lidar_pub;

    std::string dyn_frame="gps_odom";

    dynamic_reconfigure::Server<first_project::dyn_conf_third_nodeConfig> server;
    dynamic_reconfigure::Server<first_project::dyn_conf_third_nodeConfig>::CallbackType f;

public:
    lidar_remap()
    {
        lidar_sub = n.subscribe("/os_cloud_node/points", 1, &lidar_remap::lidar_callback, this);
        lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

        dyn_frame="gps_odom";

        f = boost::bind(&lidar_remap::dyn_frame_callback, this, _1, _2);
        server.setCallback(f);

        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void lidar_callback(const sensor_msgs::PointCloud2 &msg)
    {

        sensor_msgs::PointCloud2 modified_msg = msg;
        modified_msg.header.frame_id = dyn_frame;
        modified_msg.header.stamp = ros::Time ::now();

        lidar_pub.publish(modified_msg);
    }

    void dyn_frame_callback(const first_project::dyn_conf_third_nodeConfig &config, uint32_t level){

        int int_dyn_frame=config.frame_set;

        switch(int_dyn_frame){
            case 0:
                dyn_frame="wheel_odom";
                break;
            case 1:
                dyn_frame="gps_odom";
                break;
            default:
                dyn_frame="wheel_odom";
                break;
        }
        //ros::ROS_INFO(" dyn_frame reconfigured to : %s", dyn_frame);

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_remap");
    lidar_remap lidar_remap;

    ros::spin();
    return 0;
}