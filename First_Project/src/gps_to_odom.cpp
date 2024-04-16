#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFi.h"  // in caso :: al posto di /


class gps_to_odom{
    

    private:
    ros::NodeHandle n; 
    ros::Subscriber sub;

    public:
    gps_to_odom(){
		sub = n.subscribe("/fix", 1, &gps_to_odom::callback, this);

        ros::Rate loop_rate(5);
        while (ros::ok()){
                    
                ros::spinOnce();
                    
                loop_rate.sleep();

        }
    }


    void callback(const sensor_msgs::NavSatFi::::ConstPtr& msg){
        
    }
}






int main(int argc, char **argv){
    ros::init(argc, argv, "gps_to_odom");
 	gps_to_odom gps_to_odom_node;
 	ros::spin();
 	return 0;

}